/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2026  PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with PCSX2.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

/* The network adapter's Ethernet cable.
 *
 * The cable is the frontend's link bus (RETRO_ENVIRONMENT_GET_LINK_INTERFACE),
 * protocol "ethernet-1", on the IOP's 36.864 MHz, and the room it runs in is a
 * hub: every frame a console puts on the cable reaches every other console on
 * it, and each keeps what is addressed to it, to a group or to everybody. Each
 * frame is stamped on the cable's clock and handed to the SMAP when this
 * console reaches that tick, so what a game hears depends on emulated time and
 * not on how the host's threads happened to run.
 *
 * The pacing is the i.LINK cable's (FW.cpp), for the same reasons: a console
 * may not run further ahead of the others than they have promised not to
 * originate anything before, and the frontend's advance() that enforces it
 * blocks, so it is called from a helper thread while the EE watches for a
 * pause. The promise is a millisecond rather than half of one -- an Ethernet
 * frame carries its own timing and no driver waits on a split transaction --
 * and frames line up the same way, so a console's burst of work after vsync
 * lands while the others are doing theirs. */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <vector>

#include <libretro.h>

#include "common/Pcsx2Defs.h"
#include "IopHw.h"
#include "R3000A.h"
#include "R5900.h"
#include "VMManager.h"

#include <rthreads/rthreads.h>

#include "DEV9.h"
#include "link_adapter.h"

using namespace PacketReader;
using namespace PacketReader::IP;

/* ---- timing, in IOP cycles (36.864 MHz) ---- */

/* The cable's clock is the IOP's nominal one, stated once at attach. */
#define ETH_IOP_HZ        36864000u
/* How far ahead of itself each console promises not to originate a frame,
 * which is the longest a frame takes to cross: a millisecond, a long way
 * inside anything a LAN game waits on and short against a frame. */
#define ETH_AHEAD_LINKED  (ETH_IOP_HZ / 1000)
/* How often the cable meets the other consoles: half the promise, and the
 * other half is slack, so consoles standing a little apart all run together
 * instead of taking turns (FW.cpp, FW_GRAIN_LINKED, has the measurements). */
#define ETH_GRAIN_LINKED  (ETH_AHEAD_LINKED / 2)
/* With nobody on the cable the only thing to learn is that somebody arrived. */
#define ETH_GRAIN_ALONE   (ETH_IOP_HZ / 60)
/* A step of the IOP's cycle counter longer than this between two looks at it
 * is not time passing -- the cable is looked at far more often -- but a state
 * load or a reset moving the counter, and is not counted. Counting it would
 * put this console that far ahead on the cable and park it there until every
 * other console caught up. */
#define ETH_CLOCK_GAP     (ETH_IOP_HZ / 4)

static u64 eth_ahead_linked = ETH_AHEAD_LINKED;
/* LRPS2_ETH_DIAG=1: where the cable is, every 300 frames and whenever a
 * rendezvous has waited longer than a second. Wall clock is read for the log
 * and nothing else. */
static bool eth_diag = false;
static u64 eth_pause_exits = 0;
static u64 eth_grain_linked = ETH_GRAIN_LINKED;
static bool eth_align = true;

/* ---- frames ---- */

/* The longest frame the SMAP sends, and a tagged one besides; anything longer
 * on the cable is not Ethernet and is dropped. The shortest is what the wire
 * pads every frame to. */
#define ETH_FRAME_MAX     1518
#define ETH_FRAME_MIN     60
/* Frames received and not yet due. Past this the rest wait on the bus, which
 * keeps them in order: back-pressure, not loss. */
#define ETH_RX_QUEUE      512

struct eth_frame
{
	u64 tick;
	std::vector<u8> data;
};

/* ---- state ---- */

/* The cable outlives a VM: attach and detach follow the content, the adapter
 * follows the machine. */
static struct
{
	const struct retro_link_interface* link;
	retro_link_port_t* handle;
	bool attached;

	int self;          /* this console's index on the bus, -1 if none   */
	unsigned peers;    /* consoles on the bus, this one included         */

	u64 safe;          /* the commit horizon last published             */
	u64 next_rv;       /* when the cable next meets the other consoles  */
	u64 last_stamp;

	/* A monotonic clock accumulated from psxRegs.cycle, which is 32 bits and
	 * wraps; only forward steps are taken (ETH_CLOCK_GAP). */
	u32 last_raw;
	bool have_raw;
	u64 now;

	/* Lining this console's frames up with the others' (ETHvsync), as the
	 * i.LINK cable does. */
	bool anchored;
	bool align;
	u64 anchor;
	u64 skip;
	u64 last_vsync;
	u64 period;

	/* This console's address on the cable: the adapter's MAC, and the IP its
	 * DHCP is answered with. */
	u8 mac[6];

	std::deque<eth_frame> rx;

	u64 sent, received, filtered;
} eth;

/* LRPS2_ETH_PCAP=<path>: every frame this console puts on or takes off the
 * cable, in a capture file a packet analyser opens -- <path>.<MAC tail>.pcap,
 * one per console. Stamped with the cable's clock, not the host's. */
static FILE* eth_pcap;

static void eth_pcap_open(void)
{
	const char* base = getenv("LRPS2_ETH_PCAP");
	char path[1024];
	if (!base || eth_pcap)
		return;
	snprintf(path, sizeof(path), "%s.%02x%02x%02x.pcap", base, eth.mac[3], eth.mac[4], eth.mac[5]);
	eth_pcap = fopen(path, "wb");
	if (eth_pcap)
	{
		/* Classic pcap, microseconds, Ethernet. */
		const u32 hdr[6] = {0xa1b2c3d4u, 0x00040002u, 0, 0, 65535, 1};
		fwrite(hdr, sizeof(hdr), 1, eth_pcap);
	}
}

static void eth_pcap_frame(const u8* f, size_t len, u64 tick)
{
	u64 us;
	u32 rec[4];
	if (!eth_pcap)
		return;
	us = tick / (ETH_IOP_HZ / 1000000u);
	rec[0] = (u32)(us / 1000000u);
	rec[1] = (u32)(us % 1000000u);
	rec[2] = rec[3] = (u32)len;
	fwrite(rec, sizeof(rec), 1, eth_pcap);
	fwrite(f, len, 1, eth_pcap);
	fflush(eth_pcap);
}

static void eth_log_frame(const char* what, const u8* f, size_t len, u64 tick)
{
	static int trace = -1;
	eth_pcap_frame(f, len, tick);
	if (trace < 0)
		trace = getenv("LRPS2_ETH_LOG") != NULL;
	if (!trace || !log_cb || len < 14)
		return;
	log_cb(RETRO_LOG_WARN, "Ethernet: %s @%llu %02x:%02x:%02x:%02x:%02x:%02x <- %02x:%02x:%02x:%02x:%02x:%02x type %02x%02x, %u bytes\n",
		what, (unsigned long long)tick, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11],
		f[12], f[13], (unsigned)len);
}

/* ---- clock ---- */

static u64 eth_clock(void)
{
	u32 raw = psxRegs.cycle;
	if (!eth.have_raw)
	{
		eth.have_raw = true;
		eth.last_raw = raw;
		return eth.now;
	}
	s32 delta = (s32)(raw - eth.last_raw);
	eth.last_raw = raw;
	if (delta > 0 && (u32)delta <= ETH_CLOCK_GAP)
		eth.now += (u32)delta;
	return eth.now;
}

/* ---- the cable ---- */

static void eth_refresh_peers(void)
{
	unsigned count = 0;
	unsigned was = eth.peers;
	int self;

	if (!eth.attached)
		return;
	self = eth.link->peers(eth.handle, &count);
	if (self < 0 || count < 2)
	{
		eth.self = -1;
		eth.peers = 0;
	}
	else
	{
		eth.self = self;
		eth.peers = count;
	}
	if (eth.peers != was && log_cb)
		log_cb(RETRO_LOG_WARN, "Ethernet: the cable now joins %u console(s)%s\n", eth.peers ? eth.peers : 1,
			eth.peers >= 2 ? "" : " -- nothing on the other end");
}

/* Frames from the cable, kept in the order they are due. They are taken as
 * soon as they are on the bus and handed over at their tick (LinkAdapter::
 * recvDue); what a sender stamps is never before its promise, and this console
 * is never granted past that, so none is due before it arrives. */
static void eth_drain(void)
{
	u8 buf[ETH_FRAME_MAX];
	u64 tick;
	unsigned from;
	size_t len;

	for (;;)
	{
		if (eth.rx.size() >= ETH_RX_QUEUE)
			break;
		len = sizeof(buf);
		if (!eth.link->recv(eth.handle, &tick, &from, buf, &len))
			break;
		if (len < 14)
			continue;
		/* A hub repeats every frame to every port; the adapter keeps a
		 * frame for its own address, and any for a group, broadcast among
		 * them. */
		if (!(buf[0] & 1) && memcmp(buf, eth.mac, 6) != 0)
		{
			eth.filtered++;
			continue;
		}
		eth_frame f;
		f.tick = tick;
		f.data.assign(buf, buf + len);
		auto at = std::upper_bound(eth.rx.begin(), eth.rx.end(), tick,
			[](u64 t, const eth_frame& e) { return t < e.tick; });
		eth.rx.insert(at, std::move(f));
	}
}

static void eth_send(const u8* frame, size_t len)
{
	u8 buf[ETH_FRAME_MAX];
	u64 tick;

	/* A frame onto a cable with nobody at the far end goes nowhere, as it
	 * would on a hub with nothing else plugged in. */
	if (!eth.attached || eth.peers < 2 || len < 14 || len > ETH_FRAME_MAX)
		return;
	/* Stamped no earlier than the promise last published, which is what lets
	 * the other consoles run up to it without asking. */
	tick = eth_clock();
	if (tick < eth.safe)
		tick = eth.safe;
	if (tick < eth.last_stamp)
		tick = eth.last_stamp;
	eth.last_stamp = tick;

	memcpy(buf, frame, len);
	if (len < ETH_FRAME_MIN)
	{
		memset(buf + len, 0, ETH_FRAME_MIN - len);
		len = ETH_FRAME_MIN;
	}
	eth_log_frame("out", buf, len, tick);
	if (eth.link->send(eth.handle, tick, RETRO_LINK_BROADCAST, buf, len))
		eth.sent++;
}

/* ---- the rendezvous ----
 *
 * As the i.LINK cable's, for the same reason: advance() blocks until the
 * other consoles have published far enough, with no timeout, so a console
 * waiting on one the frontend has stopped would hold up a pause asked of it
 * -- and every save and load starts with one. The EE never calls advance()
 * itself; a helper does, and the EE waits for its answer while watching for a
 * pause, stopping where it stands if one comes. */
static struct
{
	sthread_t* thread;
	slock_t* lock;
	scond_t* cond;
	bool quit;
	bool posted;
	bool done;
	bool pending;
	u64 now, safe, request, grain, skip;
	uint64_t grant;
	uint32_t wake;
} rv;

static void rv_thread(void*)
{
	slock_lock(rv.lock);
	for (;;)
	{
		u64 now, safe, request;
		uint32_t wake = RETRO_LINK_WAKE_NONE;
		uint64_t grant;

		while (!rv.posted && !rv.quit)
			scond_wait(rv.cond, rv.lock);
		if (rv.quit)
			break;
		rv.posted = false;
		now = rv.now;
		safe = rv.safe;
		request = rv.request;
		slock_unlock(rv.lock);

		grant = eth.link->advance(eth.handle, now, safe, request, &wake);

		slock_lock(rv.lock);
		rv.grant = grant;
		rv.wake = wake;
		rv.done = true;
		scond_broadcast(rv.cond);
	}
	slock_unlock(rv.lock);
}

/* The cable's clock jumps `d` on. Nothing this console keeps for itself is
 * timed on it -- frames from the cable stay where the cable put them -- so
 * all that moves is where its frames fall on the cable's clock. */
static void eth_skip(u64 d)
{
	eth_clock();
	eth.now += d;
	if (eth.last_vsync)
		eth.last_vsync += d;
}

/* True once the cable has granted this console its next stretch; false if a
 * pause arrived first, in which case nothing here has changed. */
static bool eth_rendezvous(void)
{
	if (!rv.pending)
	{
		u64 now = eth_clock();
		u64 grain = eth.peers >= 2 ? eth_grain_linked : ETH_GRAIN_ALONE;
		u64 ahead = eth.peers >= 2 ? eth_ahead_linked : ETH_GRAIN_ALONE;
		u64 skip = eth.peers >= 2 ? eth.skip : 0;

		if (now + skip + ahead > eth.safe)
			eth.safe = now + skip + ahead;
		slock_lock(rv.lock);
		rv.now = now;
		rv.safe = eth.safe;
		rv.request = now + skip + grain;
		rv.grain = grain;
		rv.skip = skip;
		rv.done = false;
		rv.posted = true;
		scond_broadcast(rv.cond);
		slock_unlock(rv.lock);
		rv.pending = true;
	}

	slock_lock(rv.lock);
	{
		const auto waited_from = std::chrono::steady_clock::now();
		long long logged_s = 0;
		while (!rv.done)
		{
			if (VMManager::Internal::IsExecutionInterrupted())
			{
				slock_unlock(rv.lock);
				eth_pause_exits++;
				return false;
			}
			scond_wait_timeout(rv.cond, rv.lock, 1000);
			if (eth_diag && log_cb)
			{
				const long long s_waited = std::chrono::duration_cast<std::chrono::seconds>(
					std::chrono::steady_clock::now() - waited_from).count();
				if (s_waited > logged_s)
				{
					logged_s = s_waited;
					log_cb(RETRO_LOG_WARN, "Ethernet[diag]: waited %llds for the cable: now=%llu safe=%llu request=%llu peers=%u\n",
						s_waited, (unsigned long long)rv.now, (unsigned long long)rv.safe,
						(unsigned long long)rv.request, eth.peers);
				}
			}
		}
	}
	rv.done = false;
	slock_unlock(rv.lock);
	rv.pending = false;
	if (rv.skip)
	{
		eth_skip(rv.skip);
		eth.skip = 0;
	}

	{
		/* A changed count or a TOPOLOGY wake means the bus re-anchored this
		 * console at this call: the frames line up against it from here. */
		const unsigned before = eth.peers;
		eth_refresh_peers();
		if (eth.peers < 2)
			eth.anchored = false;
		else if (!eth.anchored || eth.peers != before || (rv.wake & RETRO_LINK_WAKE_TOPOLOGY))
		{
			eth.anchored = true;
			eth.anchor = rv.now;
			eth.align = true;
			eth.skip = 0;
		}
	}
	eth_drain();
	eth.next_rv = rv.now + rv.skip + rv.grain;
	return true;
}

static void eth_schedule(void)
{
	u64 now;
	s64 delta;

	if (!eth.attached)
		return;
	now = eth_clock();
	delta = eth.next_rv > now ? (s64)(eth.next_rv - now) : 0;
	if (delta < 16)
		delta = 16;
	if (delta > 0x10000000)
		delta = 0x10000000;
	if ((psxRegs.interrupt & (1 << IopEvt_ETH)) &&
	    (s64)(psxRegs.sCycle[IopEvt_ETH] + psxRegs.eCycle[IopEvt_ETH] - psxRegs.cycle) <= delta)
		return;
	PSX_INT(IopEvt_ETH, (s32)delta);
}

void ethInterrupt(void)
{
	if (eth.attached && eth_clock() >= eth.next_rv && !eth_rendezvous())
	{
		/* A pause is waiting and the cable has not answered: stop here, and
		 * collect the answer when the machine runs again (fwInterrupt). */
		psxRegs.iopBreak += psxRegs.iopCycleEE;
		psxRegs.iopCycleEE = 0;
		Cpu->ExitExecution();
		PSX_INT(IopEvt_ETH, 1);
		return;
	}
	eth_schedule();
}

/* The boot's psxReset wipes every pending IOP event, the cable's with it, and
 * the other consoles are held at the moment this one joined until it first
 * says where it stands. */
void ETHiopReset(void)
{
	if (eth.attached)
		eth_schedule();
}

/* Cabled consoles start their frames at the same moment of the cable's clock;
 * see FWvsync, whose measurements this rests on. */
void ETHvsync(void)
{
	u64 now, phase;
	if (!eth.attached)
		return;
	now = eth_clock();
	if (eth_diag && log_cb)
	{
		static u64 frames;
		if (++frames % 300 == 0)
			log_cb(RETRO_LOG_WARN, "Ethernet[diag]: frame %llu now=%llu next_rv=%llu safe=%llu pending=%d peers=%u rx=%u sent=%llu recv=%llu pause_exits=%llu\n",
				(unsigned long long)frames, (unsigned long long)now, (unsigned long long)eth.next_rv,
				(unsigned long long)eth.safe, rv.pending ? 1 : 0, eth.peers, (unsigned)eth.rx.size(),
				(unsigned long long)eth.sent, (unsigned long long)eth.received, (unsigned long long)eth_pause_exits);
	}
	if (eth.last_vsync && now > eth.last_vsync)
		eth.period = now - eth.last_vsync;
	eth.last_vsync = now;
	if (!eth.align || !eth.period || eth.peers < 2 || now < eth.anchor)
		return;
	eth.align = false;
	phase = (now - eth.anchor) % eth.period;
	eth.skip = eth_align && phase ? eth.period - phase : 0;
	if (log_cb)
		log_cb(RETRO_LOG_INFO, "Ethernet: frames start %llu ticks into the cable's %llu; %s\n",
			(unsigned long long)phase, (unsigned long long)eth.period,
			!phase ? "already lined up" : eth_align ? "skipping the rest" : "left there (LRPS2_ETH_ALIGN=0)");
}

/* ---- attach and detach ---- */

void ETHlinkAttach(const struct retro_link_interface* link, unsigned port)
{
	if (eth.attached || !link)
		return;
	{
		const char* us = getenv("LRPS2_ETH_AHEAD_US");
		const long v = us ? strtol(us, NULL, 10) : 0;
		eth_ahead_linked = v > 0 ? (u64)ETH_IOP_HZ * (u64)v / 1000000u : ETH_AHEAD_LINKED;
		eth_grain_linked = eth_ahead_linked / 2;
		if (v > 0 && log_cb)
			log_cb(RETRO_LOG_WARN, "Ethernet: promise %ld us, stretch %ld us (LRPS2_ETH_AHEAD_US)\n", v, v / 2);
		us = getenv("LRPS2_ETH_ALIGN");
		eth_align = !(us && us[0] == '0');
		eth_diag = getenv("LRPS2_ETH_DIAG") != NULL;
	}
	eth.handle = link->attach(port, ETH_LINK_PROTOCOL, ETH_IOP_HZ);
	if (!eth.handle)
	{
		if (log_cb)
			log_cb(RETRO_LOG_WARN, "Ethernet: the frontend refused a link on port %u\n", port);
		return;
	}
	eth.link = link;
	if (!rv.lock)
	{
		rv.lock = slock_new();
		rv.cond = scond_new();
	}
	rv.quit = rv.posted = rv.done = rv.pending = false;
	rv.thread = sthread_create(rv_thread, NULL);
	eth.attached = true;
	eth.self = -1;
	eth.peers = 0;
	eth.safe = 0;
	eth.next_rv = 0;
	eth.last_stamp = 0;
	eth.anchored = eth.align = false;
	eth.anchor = eth.skip = eth.last_vsync = eth.period = 0;
	eth.rx.clear();
	eth.sent = eth.received = eth.filtered = 0;

	/* A Sony OUI, as the adapter's own default has, and the rest from the
	 * handle the frontend gave this console: distinct for every console it
	 * runs, where every console otherwise reads the same EEPROM default and
	 * two of them on one hub would share an address. */
	{
		uintptr_t h = (uintptr_t)eth.handle;
		u32 s = (u32)(h ^ (h >> 32)) * 2654435761u;
		eth.mac[0] = 0x00;
		eth.mac[1] = 0x04;
		eth.mac[2] = 0x1f;
		/* Never 0 or 255: the byte is also the second of this console's IP. */
		eth.mac[3] = (u8)(1 + (s >> 24) % 254);
		eth.mac[4] = (u8)(s >> 16);
		eth.mac[5] = (u8)(s >> 8);
	}
	if (log_cb)
		log_cb(RETRO_LOG_WARN, "Ethernet: cable attached to port %u as %02x:%02x:%02x:%02x:%02x:%02x\n", port,
			eth.mac[0], eth.mac[1], eth.mac[2], eth.mac[3], eth.mac[4], eth.mac[5]);
	eth_pcap_open();
}

/* Called from the frontend's thread, possibly while the EE is inside a bus
 * call: that call returns once the port is gone. */
void ETHlinkDetach(void)
{
	if (!eth.attached)
		return;
	eth.attached = false;
	eth.link->detach(eth.handle);
	if (rv.thread)
	{
		slock_lock(rv.lock);
		rv.quit = true;
		scond_broadcast(rv.cond);
		slock_unlock(rv.lock);
		sthread_join(rv.thread);
		rv.thread = NULL;
	}
	rv.pending = false;
	if (eth_pcap)
	{
		fclose(eth_pcap);
		eth_pcap = NULL;
	}
	if (log_cb)
		log_cb(RETRO_LOG_WARN, "Ethernet: cable detached (%llu frames sent, %llu received, %llu for other consoles)\n",
			(unsigned long long)eth.sent, (unsigned long long)eth.received, (unsigned long long)eth.filtered);
}

/* ---- the adapter ---- */

LinkAdapter::LinkAdapter()
{
	InitAddresses();
}

void LinkAdapter::InitAddresses()
{
	/* Without a cable the adapter keeps its default MAC; with one, the
	 * console's own. */
	if (eth.attached)
	{
		MAC_Address mac;
		memcpy(mac.bytes, eth.mac, 6);
		SetMACAddress(&mac);
	}

	/* DHCP answered here, inside the console, so no two consoles ever answer
	 * each other and a game set to configure itself gets an address at once.
	 * 10.x.y.z/8, where x.y.z is the tail of the MAC: as distinct as the MACs
	 * are, and all of them one subnet. Nothing at the gateway answers -- there
	 * is no router on a hub -- but a lease with none is one some stacks
	 * refuse. */
	{
		IP_Address ip{{{10, ps2MAC.bytes[3], ps2MAC.bytes[4], ps2MAC.bytes[5]}}};
		IP_Address mask{{{255, 0, 0, 0}}};
		IP_Address gateway{{{10, 0, 0, 1}}};
		InitInternalServer(nullptr, true, ip, mask, gateway);
		if (log_cb)
			log_cb(RETRO_LOG_WARN, "Ethernet: DHCP offers %u.%u.%u.%u/8\n", ip.bytes[0], ip.bytes[1], ip.bytes[2], ip.bytes[3]);
	}
}

void LinkAdapter::reloadSettings()
{
	InitAddresses();
}

bool LinkAdapter::send(NetPacket* pkt)
{
	/* DHCP and DNS for this console are answered inside it, never on the
	 * cable. */
	if (NetAdapter::send(pkt))
		return true;
	if (pkt->size > 0)
		eth_send((const u8*)pkt->buffer, (size_t)pkt->size);
	return true;
}

bool LinkAdapter::recvDue(NetPacket* pkt)
{
	if (eth.rx.empty())
		return false;
	if (eth.rx.front().tick > eth_clock())
		return false;
	const eth_frame& f = eth.rx.front();
	const size_t len = std::min(f.data.size(), sizeof(pkt->buffer));
	memcpy(pkt->buffer, f.data.data(), len);
	pkt->size = (int)len;
	eth_log_frame("in", f.data.data(), f.data.size(), f.tick);
	eth.rx.pop_front();
	eth.received++;
	return true;
}
