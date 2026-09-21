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

/* The i.LINK (IEEE 1394) controller at 0x1F808400, and the cable.
 *
 * The console's link layer is an LSI Logic node controller core behind an
 * IEEE 1394a PHY. Nothing public documents it; what is modelled here is what
 * the drivers that ship on game discs actually use -- Sony's ILINK.IRX (the
 * sce1394 stack, which Time Crisis II loads at boot) and the ps2sdk's
 * iLinkman -- worked out from their code:
 *
 *   - PHY registers through PHYAccess (0x14): a request is reg<<24 | data<<16
 *     with bit 31 (read) or bit 30 (write); a read completes with the
 *     register in bits 11:8 and the data in 7:0, and raises PhyRRx. The PHY
 *     also reports register 0 (physical ID, root) on its own after every bus
 *     reset, which is where Sony's driver takes its node ID from.
 *   - A bus reset raises PhyRst; the self-ID phase then lands in DBUF0 as
 *     0x000000E1, one quadlet per node, and a closing 0x00000001, with DRFR
 *     raised and NodeID bit 0 set once the node has an ID again.
 *   - Asynchronous packets go out through the UBUF: every quadlet but the last
 *     is written to 0x40 and the last to 0x44, which sends it. The header is
 *     the host form -- destination in quadlet 1 -- and arrives at the far end
 *     in wire form, destination in quadlet 0 and source in quadlet 1, in the
 *     receiver's UBUF with a trailer carrying the speed in bits 18:16. URx
 *     announces it; UTD (intr1) and AckRcvd/AckMiss close the transmit, with
 *     the ack code in ack_status bits 31:28.
 *
 * The cable is the frontend's link bus (RETRO_ENVIRONMENT_GET_LINK_INTERFACE).
 * Each console is a node on it, the bus index is the physical ID, and the
 * highest ID is root, which is what a chain of consoles resolves to. What
 * crosses is bus resets, link-layer state and packets, stamped on the IOP's
 * clock, so both ends see the same bus at the same emulated moment.
 *
 * The ack a receiver would return is decided at the sender: every node on
 * this bus is this same controller configured the same way, and the rule the
 * receiving driver itself assumes -- a request taken into the UBUF was
 * answered ack_pending, a response ack_complete -- is known at both ends. A
 * packet to an ID nobody holds is AckMiss. Nothing has to make the round trip
 * to learn it, so a transmit completes in the time the wire takes and the
 * outcome depends on emulated time alone. */

#include <stdlib.h>
#include <string.h> /* memset */
#include <stdio.h>
#include <stdint.h>

#include <libretro.h>

#include "IopDma.h"
#include "IopHw.h"
#include "IopMem.h"
#include "R3000A.h"
#include "R5900.h"
#include "SaveState.h"
#include "VMManager.h"

#include <rthreads/rthreads.h>

#include "FW.h"

extern retro_log_printf_t log_cb;

static int fw_trace = -1;
#define FWTRACE(...) do { \
	if (fw_trace < 0) fw_trace = getenv("LRPS2_FW_LOG") != NULL; \
	if (fw_trace && log_cb) log_cb(RETRO_LOG_INFO, __VA_ARGS__); \
} while (0)

/* LRPS2_ILINK_LOG=1: every packet, bus reset and self-ID, without the
 * register-level noise of LRPS2_FW_LOG. */
#define FW_MAX_QUADS_LOG 12
static int il_trace = -1;
static bool il_log(void)
{
	if (il_trace < 0)
		il_trace = getenv("LRPS2_ILINK_LOG") != NULL || getenv("LRPS2_FW_LOG") != NULL;
	return il_trace && log_cb;
}

static void il_dump(const char* what, const u32* q, unsigned n, u64 tick)
{
	char line[64 + FW_MAX_QUADS_LOG * 9];
	unsigned i, o;
	if (!il_log())
		return;
	o = (unsigned)snprintf(line, sizeof(line), "i.LINK: %s @%llu:", what, (unsigned long long)tick);
	for (i = 0; i < n && i < FW_MAX_QUADS_LOG; i++)
		o += (unsigned)snprintf(line + o, sizeof(line) - o, " %08x", q[i]);
	log_cb(RETRO_LOG_INFO, "%s%s\n", line, n > FW_MAX_QUADS_LOG ? " ..." : "");
}

/* ---- registers ---- */

#define FW_BASE        0x1f808400u
#define FW_REG(off)    fw.regs[((off) & 0x1ff) >> 2]

#define R_NODEID       0x00
#define R_CYCLETIME    0x04
#define R_CTRL0        0x08
#define R_CTRL1        0x0c
#define R_CTRL2        0x10
#define R_PHYACC       0x14
#define R_INTR0        0x20
#define R_INTR0MASK    0x24
#define R_INTR1        0x28
#define R_INTR1MASK    0x2c
#define R_INTR2        0x30
#define R_INTR2MASK    0x34
#define R_DMAR         0x38
#define R_ACKSTATUS    0x3c
#define R_UBUF_TXNEXT  0x40
#define R_UBUF_TXLAST  0x44
#define R_UBUF_TXCLEAR 0x48
#define R_UBUF_RXCLEAR 0x4c
#define R_UBUF_RX      0x50
#define R_UBUF_RXLEVEL 0x54
#define R_POWER        0x7c
#define R_PHT_CTRL0    0x80
#define R_DBUF_LVL0    0xc0
#define R_DBUF_TX0     0xc4
#define R_PHT_SPLITTO0 0x84
#define R_PHT_HDR0_0   0x88
#define R_PHT_HDR1_0   0x8c
#define R_PHT_HDR2_0   0x90
#define R_DTRANS0      0xa4
#define R_DBUF_RX0     0xc8
#define R_PHT_CTRL1    0x100
#define R_DBUF_LVL1    0x140
#define R_DBUF_TX1     0x144
#define R_DBUF_RX1     0x148

#define INTR0_DRFR     0x00000001u
#define INTR0_PBCNTR   0x00000200u
#define INTR0_STO      0x00000400u
#define INTR0_RETEX    0x00000800u
#define INTR0_ACKMISS  0x00002000u
#define INTR0_ACKRCVD  0x00004000u
#define INTR0_CYCSEC   0x00100000u
#define INTR0_URX      0x00400000u
#define INTR0_PHYRST   0x20000000u
#define INTR0_PHYRRX   0x40000000u
#define INTR1_UTD      0x00000002u

#define CTRL0_ROOT     0x00080000u
#define CTRL0_CYCTMREN 0x00200000u
#define CTRL0_BUSIDRST 0x00800000u
#define CTRL0_RXRST    0x01000000u
#define CTRL0_TXRST    0x02000000u
#define CTRL0_RXEN     0x04000000u

#define CTRL2_LPSEN    0x00000002u
#define CTRL2_SOK      0x00000008u

#define PHT_RST        0x00200000u
#define PHT_PRBR       0x00000200u /* stopped by a bus reset          */
#define PHT_PSTK       0x00000400u /* stopped: see the ack/rcode      */
#define PHT_EBCNT      0x00008000u
#define PHT_EWREQ      0x00010000u
#define PHT_ERREQ      0x00020000u
#define PHT_STATUS     0x000000ffu /* ack in 7:4, rcode in 3:0        */

#define PHY_RDREQ      0x80000000u
#define PHY_WRREQ      0x40000000u

#define PHY1_IBR       0x40
#define PHY4_LCTRL     0x80
#define PHY5_ISBR      0x40

#define ACK_COMPLETE   1
#define ACK_PENDING    2

/* ---- timing, in IOP cycles (36.864 MHz) ---- */

/* The cable's clock is the IOP's nominal one. Fixed rather than PSXCLK, which
 * can move at run time: a rate is stated once, at attach. */
#define FW_IOP_HZ         36864000u

/* How long a packet occupies the wire, and the gap from a bus reset to the
 * end of its self-ID phase. Both are far below anything a driver times out
 * on and far above a single instruction, which is all they have to be. */
#define FW_TX_CYCLES      1024
#define FW_SELFID_CYCLES  2048
/* How often the cable meets the other consoles, and how far ahead of itself
 * each console promises not to originate anything. Half a millisecond keeps a
 * request and its response well inside the default 100 ms split timeout. */
#define FW_GRAIN_LINKED   (FW_IOP_HZ / 2000)
/* With nobody on the bus the only thing to learn is that somebody arrived. */
#define FW_GRAIN_ALONE    (FW_IOP_HZ / 60)

/* The 1394 cycle timer runs at 24.576 MHz, two thirds of the IOP clock. */
#define CYC_OFFSETS       3072u
#define CYC_PER_SECOND    8000u

/* ---- FIFOs ---- */

#define FIFO_QUADS 1024

struct fw_fifo
{
	u32 q[FIFO_QUADS];
	u32 head, count;
};

static void fifo_clear(fw_fifo* f) { f->head = f->count = 0; }

static void fifo_push(fw_fifo* f, u32 v)
{
	if (f->count >= FIFO_QUADS)
		return;
	f->q[(f->head + f->count) % FIFO_QUADS] = v;
	f->count++;
}

static u32 fifo_pop(fw_fifo* f)
{
	u32 v;
	if (!f->count)
		return 0;
	v = f->q[f->head];
	f->head = (f->head + 1) % FIFO_QUADS;
	f->count--;
	return v;
}

/* ---- timed events ---- */

enum
{
	EV_BUS_RESET = 1, /* the bus goes into reset                       */
	EV_SELF_ID,       /* the self-ID phase has ended                   */
	EV_TX_DONE,       /* this node's packet has left the wire          */
	EV_RX_PACKET,     /* a packet from the cable is due                */
	EV_PEER_STATE,    /* a peer's link-layer state is due              */
	EV_PHT_TIMEOUT    /* a PHT write waited out its split timeout       */
};

#define FW_MAX_QUADS 160
#define FW_EVENTS    128

struct fw_event
{
	u64 tick;
	u64 seq;
	int type;
	u32 arg;
	u32 nquads;
	u32 quads[FW_MAX_QUADS];
};

/* ---- the bus message ---- */

#define FW_PROTOCOL "ps2-ilink-1394"

/* Packed by hand, as little-endian words, so a different build (or a
 * different core) could speak it without sharing a struct layout. */
enum
{
	MSG_STATE = 1,  /* word1: link-layer flags; word2: "I have not heard you" */
	MSG_RESET,      /* a node is initiating a bus reset                        */
	MSG_PACKET      /* word1: speed; word2: quadlet count; then the quadlets   */
};
#define STATE_LINK_ON 1

/* ---- state ---- */

static struct
{
	u32 regs[0x180 / 4];
	u8 phy[16];
	u8 phy_page1[8];

	fw_fifo ubuf_rx;
	fw_fifo ubuf_tx;
	fw_fifo dbuf_rx[2];
	fw_fifo dbuf_tx0;

	/* PHT0, the block-transfer engine: a write the driver describes once --
	 * destination, offset, packet size, total -- and feeds through the DBUF,
	 * which goes out as block write requests, each closed by the response
	 * that comes back with the PHT's own transaction label. */
	struct
	{
		bool active;
		bool waiting;      /* a request is out, its response is not back */
		u16 dest;
		u16 off_hi;
		u32 off_lo;
		u32 tl;
		u32 speed;
		u32 max_bytes;
		u32 cur_bytes;
		u32 remaining;
		u64 generation;    /* which waiting request a timeout belongs to */
	} pht;

	/* The bus as this node last worked it out. */
	unsigned nodes;
	unsigned phy_id;
	bool root;
	bool id_valid;
	bool tx_busy;

	/* A monotonic clock accumulated from psxRegs.cycle, which is 32 bits,
	 * wraps and jumps at a state load; only forward steps are taken. */
	u32 last_raw;
	bool have_raw;
	u64 now;

	/* The cycle timer is (clock * 2/3 + base), so a write sets it. */
	s64 cyc_base;
	u64 next_second;

	fw_event ev[FW_EVENTS];
	unsigned nev;
	u64 next_seq;
	bool scheduled;
} fw;

/* The cable. Held apart from `fw` because it outlives a VM: attach and detach
 * follow the content, FWopen and FWclose follow the machine. */
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

	/* What the other consoles have said about their link layers. Indexed
	 * by bus index; only a two-console cable is expected, but a chain costs
	 * nothing more. */
	u8 peer_flags[64];
	bool peer_heard[64];
	bool announced;

	u64 sent, received;
} cable;

/* ---- clock ---- */

static u64 fw_clock(void)
{
	u32 raw = psxRegs.cycle;
	if (!fw.have_raw)
	{
		fw.have_raw = true;
		fw.last_raw = raw;
		return fw.now;
	}
	s32 delta = (s32)(raw - fw.last_raw);
	fw.last_raw = raw;
	if (delta > 0)
		fw.now += (u32)delta;
	return fw.now;
}

static u64 fw_cycle_ticks(void)
{
	return (u64)((s64)(fw_clock() * 2 / 3) + fw.cyc_base);
}

static u32 fw_cycle_time(void)
{
	u64 t = fw_cycle_ticks();
	u32 offset = (u32)(t % CYC_OFFSETS);
	u64 cycles = t / CYC_OFFSETS;
	u32 count = (u32)(cycles % CYC_PER_SECOND);
	u32 secs = (u32)((cycles / CYC_PER_SECOND) & 0x7f);
	return secs << 25 | count << 12 | offset;
}

static void fw_cycle_write(u32 v)
{
	u64 want = (u64)(v >> 25) * CYC_PER_SECOND * CYC_OFFSETS +
	           (u64)((v >> 12) & 0x1fff) * CYC_OFFSETS + (v & 0xfff);
	fw.cyc_base = (s64)want - (s64)(fw_clock() * 2 / 3);
}

/* The IOP cycle at which the cycle timer next rolls into a new second. */
static u64 fw_next_second(void)
{
	const u64 per_sec = (u64)CYC_PER_SECOND * CYC_OFFSETS;
	u64 t = fw_cycle_ticks();
	u64 next = (t / per_sec + 1) * per_sec;
	return (u64)(((s64)next - fw.cyc_base) * 3 / 2) + 1;
}

/* ---- interrupts ---- */

static void fw_irq_update(u32 raised0, u32 raised1)
{
	if ((raised0 & FW_REG(R_INTR0MASK)) || (raised1 & FW_REG(R_INTR1MASK)))
		fwIrq();
}

static void fw_raise(u32 bits0, u32 bits1)
{
	u32 new0 = bits0 & ~FW_REG(R_INTR0);
	u32 new1 = bits1 & ~FW_REG(R_INTR1);
	FW_REG(R_INTR0) |= bits0;
	FW_REG(R_INTR1) |= bits1;
	fw_irq_update(new0 | (bits0 & INTR0_URX) | (bits0 & INTR0_DRFR), new1 | (bits1 & INTR1_UTD));
}

/* ---- events ---- */

static void fw_schedule(void);
static void fw_release(void);

static fw_event* fw_queue(u64 tick, int type)
{
	fw_event* e;
	if (fw.nev >= FW_EVENTS)
	{
		if (log_cb)
			log_cb(RETRO_LOG_WARN, "i.LINK: event queue full, dropping\n");
		return NULL;
	}
	e = &fw.ev[fw.nev++];
	e->tick = tick;
	e->seq = fw.next_seq++;
	e->type = type;
	e->arg = 0;
	e->nquads = 0;
	return e;
}

/* ---- the bus as this node sees it ---- */

static bool fw_link_on(void) { return (fw.phy[4] & PHY4_LCTRL) != 0; }

static unsigned fw_bus_nodes(void)
{
	if (cable.attached && cable.self >= 0 && cable.peers >= 2)
		return cable.peers;
	return 1;
}

static unsigned fw_bus_self(void)
{
	return fw_bus_nodes() > 1 ? (unsigned)cable.self : 0;
}

/* Self-ID packet zero for node `id` of `n`, a chain with the root at the top.
 * Port codes: 3 = child, 2 = parent, 1 = not connected. */
static u32 fw_self_id(unsigned id, unsigned n, bool link_on)
{
	unsigned p0, p1;
	if (n <= 1)
		p0 = 1, p1 = 1;
	else if (id == n - 1)
		p0 = 3, p1 = 1;
	else if (id == 0)
		p0 = 2, p1 = 1;
	else
		p0 = 2, p1 = 3;
	return 0x80000000u | (id & 0x3f) << 24 | (link_on ? 1u : 0u) << 22 | 0x3fu << 16 |
	       2u << 14 /* S400 */ | 1u << 11 /* contender */ | p0 << 6 | p1 << 4;
}

static void pht_stop(u32 status, u32 intr0);

static void fw_bus_reset_start(void)
{
	if (fw.pht.active)
		pht_stop(PHT_PRBR, INTR0_RETEX);
	fw.id_valid = false;
	FW_REG(R_NODEID) &= ~1u;
	fw_raise(INTR0_PHYRST, 0);
	fw_queue(fw_clock() + FW_SELFID_CYCLES, EV_SELF_ID);
	if (il_log())
		log_cb(RETRO_LOG_INFO, "i.LINK: bus reset @%llu\n", (unsigned long long)fw_clock());
}

static void fw_self_id_complete(void)
{
	unsigned n = fw_bus_nodes();
	unsigned self = fw_bus_self();
	unsigned i;

	fw.nodes = n;
	fw.phy_id = self;
	fw.root = self == n - 1;
	fw.id_valid = true;

	FW_REG(R_NODEID) = (FW_REG(R_NODEID) & 0xffc00000u) | (self & 0x3f) << 16 | 1u;

	fw.phy[0] = (u8)((self & 0x3f) << 2 | (fw.root ? 2 : 0) | 1);

	fifo_push(&fw.dbuf_rx[0], 0x000000e1);
	for (i = 0; i < n; i++)
	{
		bool on = i == self ? fw_link_on() : (cable.peer_heard[i] ? (cable.peer_flags[i] & STATE_LINK_ON) != 0 : true);
		fifo_push(&fw.dbuf_rx[0], fw_self_id(i, n, on));
	}
	fifo_push(&fw.dbuf_rx[0], 0x00000001);

	/* The PHY's own register-0 report, which is how the driver learns its ID. */
	FW_REG(R_PHYACC) = (FW_REG(R_PHYACC) & ~0xfffu) | (0u << 8) | fw.phy[0];
	fw_raise(INTR0_DRFR | INTR0_PHYRRX, 0);

	if (log_cb)
		log_cb(RETRO_LOG_INFO, "i.LINK: self-ID complete, node %u of %u%s\n", self, n, fw.root ? " (root)" : "");
}

/* ---- the cable ---- */

static void cable_send(u64 tick, const u32* words, unsigned nwords)
{
	u8 buf[4 * (FW_MAX_QUADS + 4)];
	unsigned i;

	if (!cable.attached || cable.peers < 2)
		return;
	if (tick < cable.safe)
		tick = cable.safe;
	if (tick < cable.last_stamp)
		tick = cable.last_stamp;
	cable.last_stamp = tick;

	for (i = 0; i < nwords; i++)
	{
		buf[i * 4 + 0] = (u8)(words[i]);
		buf[i * 4 + 1] = (u8)(words[i] >> 8);
		buf[i * 4 + 2] = (u8)(words[i] >> 16);
		buf[i * 4 + 3] = (u8)(words[i] >> 24);
	}
	cable.link->send(cable.handle, tick, RETRO_LINK_BROADCAST, buf, nwords * 4);
	cable.sent++;
}

static void cable_announce(void)
{
	u32 w[3];
	bool deaf = false;
	unsigned i;
	for (i = 0; i < cable.peers && i < 64; i++)
		if ((int)i != cable.self && !cable.peer_heard[i])
			deaf = true;
	w[0] = MSG_STATE;
	w[1] = fw_link_on() ? STATE_LINK_ON : 0;
	w[2] = deaf ? 1 : 0;
	cable_send(fw_clock(), w, 3);
	cable.announced = true;
}

/* A bus reset every node takes at the same emulated tick: stamped for this
 * console's commit horizon, which is the earliest any peer can still be
 * told about, and taken here at that same tick. */
static void fw_initiate_bus_reset(void)
{
	u64 at = fw_clock();
	if (cable.attached && cable.peers >= 2)
	{
		u32 w[1] = {MSG_RESET};
		at = cable.safe > at ? cable.safe : at;
		if (at < cable.last_stamp)
			at = cable.last_stamp;
		cable_send(at, w, 1);
	}
	fw_queue(at, EV_BUS_RESET);
	fw_schedule();
}

static void cable_refresh_peers(void)
{
	unsigned count = 0;
	unsigned was = cable.peers;
	int self;

	if (!cable.attached)
		return;
	self = cable.link->peers(cable.handle, &count);
	if (self < 0 || count < 2)
	{
		cable.self = -1;
		cable.peers = 0;
	}
	else
	{
		cable.self = self;
		cable.peers = count > 64 ? 64 : count;
	}
	if (cable.peers == was)
		return;

	memset(cable.peer_heard, 0, sizeof(cable.peer_heard));
	memset(cable.peer_flags, 0, sizeof(cable.peer_flags));
	cable.announced = false;
	if (log_cb)
		log_cb(RETRO_LOG_INFO, "i.LINK: cable now joins %u console(s)%s\n", cable.peers ? cable.peers : 1,
			cable.peers >= 2 ? "" : " -- nothing on the other end");

	/* Plugging a cable in or pulling it out resets the bus, at both ends. */
	fw_queue(cable.safe > fw_clock() ? cable.safe : fw_clock(), EV_BUS_RESET);
}

static void cable_drain(void)
{
	u8 buf[4 * (FW_MAX_QUADS + 4)];
	u32 w[FW_MAX_QUADS + 4];
	u64 tick;
	unsigned from;
	size_t len;

	for (;;)
	{
		unsigned n, i;
		if (fw.nev >= FW_EVENTS - 4)
			break; /* leave it on the bus: back-pressure, not loss */
		len = sizeof(buf);
		if (!cable.link->recv(cable.handle, &tick, &from, buf, &len))
			break;
		n = (unsigned)(len / 4);
		for (i = 0; i < n; i++)
			w[i] = (u32)buf[i * 4] | (u32)buf[i * 4 + 1] << 8 | (u32)buf[i * 4 + 2] << 16 | (u32)buf[i * 4 + 3] << 24;
		if (!n)
			continue;
		cable.received++;
		switch (w[0])
		{
			case MSG_STATE:
			{
				fw_event* e;
				if (n < 3 || from >= 64)
					break;
				e = fw_queue(tick, EV_PEER_STATE);
				if (e)
				{
					e->arg = from;
					e->nquads = 2;
					e->quads[0] = w[1];
					e->quads[1] = w[2];
				}
				break;
			}
			case MSG_RESET:
				fw_queue(tick, EV_BUS_RESET);
				break;
			case MSG_PACKET:
			{
				fw_event* e;
				unsigned nq;
				if (n < 3)
					break;
				nq = w[2];
				if (nq > FW_MAX_QUADS || nq + 3 > n)
					break;
				e = fw_queue(tick, EV_RX_PACKET);
				if (e)
				{
					e->arg = w[1];
					e->nquads = nq;
					memcpy(e->quads, &w[3], nq * 4);
				}
				break;
			}
		}
	}
}

/* ---- transmit and receive ---- */

static bool tcode_is_response(unsigned tcode)
{
	return tcode == 2 || tcode == 6 || tcode == 7 || tcode == 0xb;
}

static bool tcode_has_dest(unsigned tcode)
{
	return tcode <= 2 || (tcode >= 4 && tcode <= 7) || tcode == 9 || tcode == 0xb;
}

#define ACK_MISSING    0x100

/* Put a packet, already in wire form, on the cable, and say what ack it
 * earned. A node does not hear its own packets, so one addressed to this
 * node goes unacknowledged, exactly like one to an ID nobody holds. */
static u32 fw_send_wire(const u32* q, unsigned n, u32 speed, u64 now, const char* what)
{
	unsigned tcode = (q[0] >> 4) & 0xf;
	unsigned dest_phy = (q[0] >> 16) & 0x3f;
	bool broadcast = dest_phy == 63;
	bool reachable = broadcast || (dest_phy < fw_bus_nodes() && dest_phy != fw_bus_self());
	u32 ack;

	if (broadcast)
		ack = 0;
	else if (!reachable)
		ack = ACK_MISSING;
	else
		ack = tcode_is_response(tcode) ? ACK_COMPLETE : ACK_PENDING;

	if (reachable)
	{
		u32 msg[FW_MAX_QUADS + 3];
		msg[0] = MSG_PACKET;
		msg[1] = speed;
		msg[2] = n;
		memcpy(&msg[3], q, n * 4);
		cable_send(now + FW_TX_CYCLES, msg, n + 3);
	}
	il_dump(broadcast ? "tx broadcast" : reachable ? what : "tx unanswered", q, n, now);
	return ack;
}

static void fw_transmit(void)
{
	u32 q[FW_MAX_QUADS];
	unsigned n = 0, tcode;
	u64 now = fw_clock();
	fw_event* done;

	while (fw.ubuf_tx.count && n < FW_MAX_QUADS)
		q[n++] = fifo_pop(&fw.ubuf_tx);
	fifo_clear(&fw.ubuf_tx);
	if (!n)
		return;

	tcode = (q[0] >> 4) & 0xf;
	done = fw_queue(now + FW_TX_CYCLES, EV_TX_DONE);
	fw.tx_busy = true;

	if (tcode_has_dest(tcode) && n >= 2)
	{
		u16 dest = (u16)(q[1] >> 16);
		u16 src = (u16)(FW_REG(R_NODEID) >> 16);
		u32 ack;

		/* Host form to wire form: the destination moves up into quadlet
		 * zero and this node's ID takes its place in quadlet one. */
		q[0] = (u32)dest << 16 | (q[0] & 0xffff);
		q[1] = (u32)src << 16 | (q[1] & 0xffff);
		ack = fw_send_wire(q, n, (q[0] >> 16) & 7, now, "tx");
		if (done)
			done->arg = ack;
	}
	else if (done)
	{
		/* Streams and PHY packets are not acknowledged. Nothing on this bus
		 * listens to a stream yet; a PHY packet is taken as sent. */
		done->arg = 0;
		il_dump("tx unacked", q, n, now);
	}
	fw_schedule();
}

/* ---- PHT0, the block-transfer engine ---- */

/* The split timeout is in 1394 cycles of 125 us: 4608 IOP cycles each. */
#define PHT_SPLIT_CYCLES(v) ((u64)((v) ? (v) : 0x320) * 4608u)

static void pht_stop(u32 status, u32 intr0)
{
	fw.pht.active = false;
	fw.pht.waiting = false;
	FW_REG(R_PHT_CTRL0) = (FW_REG(R_PHT_CTRL0) & ~PHT_STATUS) | PHT_PSTK | status;
	fw_raise(intr0, 0);
}

/* Send whatever the DBUF now holds enough of. One request is out at a time:
 * the next goes when the response to this one is back. */
static void pht_step(void)
{
	while (fw.pht.active && !fw.pht.waiting && fw.pht.remaining)
	{
		u32 q[FW_MAX_QUADS];
		u32 size = fw.pht.max_bytes && fw.pht.max_bytes < fw.pht.remaining ? fw.pht.max_bytes : fw.pht.remaining;
		unsigned quads = (size + 3) / 4, i;
		u64 now = fw_clock();
		u32 ack;

		if (quads + 4 > FW_MAX_QUADS)
		{
			if (log_cb)
				log_cb(RETRO_LOG_WARN, "i.LINK: PHT packet of %u bytes is larger than this model carries\n", size);
			pht_stop(0, INTR0_RETEX);
			return;
		}
		if (fw.dbuf_tx0.count < quads)
			return;

		q[0] = (u32)fw.pht.dest << 16 | (fw.pht.tl & 0x3f) << 10 | 1u << 8 | 1u << 4;
		q[1] = (FW_REG(R_NODEID) & 0xffff0000u) | fw.pht.off_hi;
		q[2] = fw.pht.off_lo;
		q[3] = size << 16;
		for (i = 0; i < quads; i++)
			q[4 + i] = fifo_pop(&fw.dbuf_tx0);
		fw.pht.cur_bytes = size;

		ack = fw_send_wire(q, 4 + quads, fw.pht.speed, now, "pht tx");
		if (ack != ACK_PENDING)
		{
			/* Nobody acknowledged it: the PHT stops, and the driver reads the
			 * missing ack out of the status byte. */
			pht_stop(0, INTR0_RETEX);
			return;
		}
		fw.pht.waiting = true;
		fw.pht.generation++;
		{
			fw_event* e = fw_queue(now + PHT_SPLIT_CYCLES(FW_REG(R_PHT_SPLITTO0) & 0xffff), EV_PHT_TIMEOUT);
			if (e)
				e->arg = (u32)fw.pht.generation;
		}
		fw_schedule();
	}
}

/* A write response carrying the PHT's own label closes its request. */
static bool pht_response(const u32* q, unsigned n)
{
	u32 rcode, done;
	if (!fw.pht.active || !fw.pht.waiting || n < 2)
		return false;
	if (((q[0] >> 4) & 0xf) != 2 || ((q[0] >> 10) & 0x3f) != (fw.pht.tl & 0x3f))
		return false;
	if ((u16)(q[1] >> 16) != fw.pht.dest)
		return false;

	rcode = (q[1] >> 12) & 0xf;
	done = fw.pht.cur_bytes < fw.pht.remaining ? fw.pht.cur_bytes : fw.pht.remaining;
	fw.pht.waiting = false;
	fw.pht.remaining -= done;
	if (fw.pht.off_lo + done < fw.pht.off_lo)
		fw.pht.off_hi++;
	fw.pht.off_lo += done;
	FW_REG(R_DTRANS0) = (FW_REG(R_DTRANS0) & ~0xffffu) | fw.pht.remaining;
	FW_REG(R_PHT_CTRL0) = (FW_REG(R_PHT_CTRL0) & ~PHT_STATUS) | ACK_PENDING << 4 | rcode;
	il_dump("pht response", q, n, fw_clock());

	if (rcode)
		pht_stop(ACK_PENDING << 4 | rcode, INTR0_RETEX);
	else if (!fw.pht.remaining)
	{
		fw.pht.active = false;
		fw_raise(INTR0_PBCNTR, 0);
	}
	else
		pht_step();
	return true;
}

static void pht_start(u32 ctrl)
{
	fw.pht.dest = (u16)(FW_REG(R_PHT_HDR0_0) >> 16);
	fw.pht.off_hi = (u16)FW_REG(R_PHT_HDR0_0);
	fw.pht.off_lo = FW_REG(R_PHT_HDR1_0);
	fw.pht.tl = (FW_REG(R_PHT_HDR2_0) >> 19) & 0x3f;
	fw.pht.speed = (FW_REG(R_PHT_HDR2_0) >> 16) & 3;
	fw.pht.max_bytes = FW_REG(R_PHT_HDR2_0) & 0xffff;
	fw.pht.remaining = FW_REG(R_DTRANS0) & 0xffff;
	fw.pht.waiting = false;
	fw.pht.active = true;
	if (ctrl & PHT_ERREQ)
	{
		static bool said;
		if (!said && log_cb)
			log_cb(RETRO_LOG_WARN, "i.LINK: PHT block read requested; only writes are modelled\n");
		said = true;
		pht_stop(0, INTR0_RETEX);
		return;
	}
	pht_step();
}

static void fw_tx_done(u32 ack)
{
	fw.tx_busy = false;
	if (ack == 0x100)
		fw_raise(INTR0_ACKMISS, INTR1_UTD);
	else if (ack)
	{
		FW_REG(R_ACKSTATUS) = ack << 28;
		fw_raise(INTR0_ACKRCVD, INTR1_UTD);
	}
	else
		fw_raise(0, INTR1_UTD);
}

static void fw_receive(const u32* q, unsigned n, u32 speed)
{
	unsigned dest_phy, i;
	if (n < 1)
		return;
	dest_phy = (q[0] >> 16) & 0x3f;
	if (dest_phy != 63 && (!fw.id_valid || dest_phy != fw.phy_id))
		return;
	if (!(FW_REG(R_CTRL0) & CTRL0_RXEN))
		return;
	if (pht_response(q, n))
		return;
	if (fw.ubuf_rx.count + n + 1 > FIFO_QUADS)
	{
		if (log_cb)
			log_cb(RETRO_LOG_WARN, "i.LINK: receive FIFO full, packet dropped\n");
		return;
	}
	for (i = 0; i < n; i++)
		fifo_push(&fw.ubuf_rx, q[i]);
	fifo_push(&fw.ubuf_rx, (speed & 7) << 16);
	fw_raise(INTR0_URX, 0);
	il_dump("rx", q, n, fw_clock());
}

/* ---- the event loop ---- */

static void fw_apply(const fw_event* e)
{
	switch (e->type)
	{
		case EV_BUS_RESET:
			fw_bus_reset_start();
			break;
		case EV_SELF_ID:
			fw_self_id_complete();
			break;
		case EV_TX_DONE:
			fw_tx_done(e->arg);
			break;
		case EV_PHT_TIMEOUT:
			/* Status 0x2f: pending, and no response -- the driver's timeout. */
			if (fw.pht.active && fw.pht.waiting && (u32)fw.pht.generation == e->arg)
				pht_stop(ACK_PENDING << 4 | 0xf, INTR0_STO);
			break;
		case EV_RX_PACKET:
			fw_receive(e->quads, e->nquads, e->arg);
			break;
		case EV_PEER_STATE:
		{
			unsigned from = e->arg;
			bool first = !cable.peer_heard[from];
			bool changed = first || cable.peer_flags[from] != (u8)e->quads[0];
			cable.peer_heard[from] = true;
			cable.peer_flags[from] = (u8)e->quads[0];
			/* Answer a console that has not heard this one, and the first
			 * time this one hears it: the pair settles after one exchange. */
			if (first || e->quads[1])
				cable_announce();
			/* A peer whose link comes up resets the bus itself and says so
			 * with MSG_RESET, so a change here needs nothing more. */
			(void)changed;
			break;
		}
	}
}

/* Everything whose moment has come, oldest first, ties by arrival. */
static void fw_release(void)
{
	u64 now = fw_clock();
	for (;;)
	{
		unsigned i, best = FW_EVENTS;
		for (i = 0; i < fw.nev; i++)
		{
			if (fw.ev[i].tick > now)
				continue;
			if (best == FW_EVENTS || fw.ev[i].tick < fw.ev[best].tick ||
			    (fw.ev[i].tick == fw.ev[best].tick && fw.ev[i].seq < fw.ev[best].seq))
				best = i;
		}
		if (best == FW_EVENTS)
			break;
		{
			fw_event e = fw.ev[best];
			fw.ev[best] = fw.ev[--fw.nev];
			fw_apply(&e);
		}
	}

	if ((FW_REG(R_CTRL0) & CTRL0_CYCTMREN) && now >= fw.next_second)
	{
		fw.next_second = fw_next_second();
		fw_raise(INTR0_CYCSEC, 0);
	}
	fw_schedule();
}

/* ---- the rendezvous ----
 *
 * advance() blocks until the other consoles have published far enough, and
 * nothing can wake it early but a message or a change of cable: there is no
 * timeout, by design. A console waiting on a peer the frontend has stopped
 * running would therefore sit in it for as long as the peer stays stopped,
 * and a pause asked of this console meanwhile -- which every save and load
 * starts with -- would wait on it in turn.
 *
 * So the EE never calls advance() itself. A helper thread does, and the EE
 * waits for the helper's answer while watching for a pause. If one comes,
 * the IOP gives up the rest of its slice and the EE leaves its loop at the
 * next block boundary: the machine stops where it stands, at a point a save
 * state can be taken from, and emulated time has not moved past what the
 * cable granted. The request stays with the helper, and the EE picks the
 * answer up when it runs again. */
static struct
{
	sthread_t* thread;
	slock_t* lock;
	scond_t* cond;
	bool quit;
	bool posted;       /* a request is waiting for the helper        */
	bool done;         /* the helper's answer is waiting for the EE  */
	bool pending;      /* a request is out and not yet collected     */
	u64 now, safe, request, grain;
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

		grant = cable.link->advance(cable.handle, now, safe, request, &wake);

		slock_lock(rv.lock);
		rv.grant = grant;
		rv.wake = wake;
		rv.done = true;
		scond_broadcast(rv.cond);
	}
	slock_unlock(rv.lock);
}

/* True once the cable has granted this console its next stretch; false if a
 * pause arrived first, in which case nothing here has changed. */
static bool cable_rendezvous(void)
{
	if (!rv.pending)
	{
		u64 now = fw_clock();
		u64 grain = cable.peers >= 2 ? FW_GRAIN_LINKED : FW_GRAIN_ALONE;

		/* Published before anything is read: a peer parked on this
		 * console's horizon cannot move until it has been told the horizon
		 * moved. */
		if (now + grain > cable.safe)
			cable.safe = now + grain;
		slock_lock(rv.lock);
		rv.now = now;
		rv.safe = cable.safe;
		rv.request = now + grain;
		rv.grain = grain;
		rv.done = false;
		rv.posted = true;
		scond_broadcast(rv.cond);
		slock_unlock(rv.lock);
		rv.pending = true;
	}

	slock_lock(rv.lock);
	while (!rv.done)
	{
		if (VMManager::Internal::IsExecutionInterrupted())
		{
			slock_unlock(rv.lock);
			return false;
		}
		/* The wait is a wall-clock one only in how often it looks for a
		 * pause; what the machine does depends on the grant alone. */
		scond_wait_timeout(rv.cond, rv.lock, 1000);
	}
	rv.done = false;
	slock_unlock(rv.lock);
	rv.pending = false;

	cable_refresh_peers();
	if (cable.peers >= 2)
	{
		/* Keep saying where this link layer stands until every other console
		 * has answered: the bus drops a message for a peer that has not yet
		 * published its own position, and both re-anchor when a cable goes in. */
		bool deaf = !cable.announced;
		unsigned i;
		for (i = 0; i < cable.peers; i++)
			if ((int)i != cable.self && !cable.peer_heard[i])
				deaf = true;
		if (deaf)
			cable_announce();
	}
	cable_drain();
	cable.next_rv = rv.now + rv.grain;
	return true;
}

static void fw_schedule(void)
{
	u64 now = fw_clock();
	u64 next = (u64)-1;
	unsigned i;
	s64 delta;

	for (i = 0; i < fw.nev; i++)
		if (fw.ev[i].tick < next)
			next = fw.ev[i].tick;
	if (cable.attached && cable.next_rv < next)
		next = cable.next_rv;
	if ((FW_REG(R_CTRL0) & CTRL0_CYCTMREN) && fw.next_second < next)
		next = fw.next_second;
	if (next == (u64)-1)
		return;

	delta = next > now ? (s64)(next - now) : 0;
	if (delta < 16)
		delta = 16;
	if (delta > 0x10000000)
		delta = 0x10000000;
	/* Re-arm only when this is sooner than what is pending already. */
	if ((psxRegs.interrupt & (1 << IopEvt_FW)) &&
	    (s64)(psxRegs.sCycle[IopEvt_FW] + psxRegs.eCycle[IopEvt_FW] - psxRegs.cycle) <= delta)
		return;
	PSX_INT(IopEvt_FW, (s32)delta);
}

void fwInterrupt(void)
{
	if (cable.attached && fw_clock() >= cable.next_rv && !cable_rendezvous())
	{
		/* A pause is waiting and the cable has not answered. Stop here: the
		 * IOP hands the rest of its slice back (the cycles are accounted for
		 * when it next runs) and the EE leaves its loop once this event test
		 * is over. This event comes straight back when the machine runs
		 * again, and collects the answer then. */
		psxRegs.iopBreak += psxRegs.iopCycleEE;
		psxRegs.iopCycleEE = 0;
		Cpu->ExitExecution();
		PSX_INT(IopEvt_FW, 1);
		return;
	}
	fw_release();
}

/* ---- PHY ---- */

static u8 fw_phy_read(unsigned reg)
{
	if (reg < 8)
		return fw.phy[reg];
	/* Registers 8-15 are paged by register 7 (page in bits 7:5). */
	if (((fw.phy[7] >> 5) & 7) == 1)
		return fw.phy_page1[reg - 8];
	if (reg == 8)
	{
		/* Port status for port 0, the socket on the back of the console:
		 * connected (bit 2) when there is somebody at the other end. */
		return fw_bus_nodes() > 1 ? 0xf4 : 0x00;
	}
	return 0;
}

static void fw_phy_write(unsigned reg, u8 data)
{
	switch (reg)
	{
		case 1:
			fw.phy[1] = (u8)((fw.phy[1] & 0x80) | (data & 0x3f));
			if (data & PHY1_IBR)
				fw_initiate_bus_reset();
			break;
		case 4:
		{
			bool was = fw_link_on();
			fw.phy[4] = data;
			if (!was && fw_link_on())
			{
				/* The link has come up. The PHY has been running since power
				 * on and the link missed that reset; it learns the bus the next
				 * time one happens, which here is now. */
				if (cable.attached && cable.peers >= 2)
					cable_announce();
				fw_initiate_bus_reset();
			}
			else if (was != fw_link_on() && cable.attached && cable.peers >= 2)
				cable_announce();
			break;
		}
		case 5:
			fw.phy[5] = (u8)(data & ~PHY5_ISBR);
			if (data & PHY5_ISBR)
				fw_initiate_bus_reset();
			break;
		case 7:
			fw.phy[7] = data;
			break;
		default:
			if (reg < 8)
				fw.phy[reg] = data;
			break;
	}
}

static void fw_phy_access(u32 value)
{
	unsigned reg = (value >> 24) & 0xf;
	if (value & PHY_WRREQ)
	{
		fw_phy_write(reg, (u8)(value >> 16));
		FW_REG(R_PHYACC) = value & ~(PHY_WRREQ | PHY_RDREQ);
	}
	else if (value & PHY_RDREQ)
	{
		FW_REG(R_PHYACC) = (value & ~(PHY_RDREQ | 0xfffu)) | reg << 8 | fw_phy_read(reg);
		/* The status bit latches whatever the mask says; the mask only
		 * decides whether the interrupt line follows. */
		fw_raise(INTR0_PHYRRX, 0);
	}
	else
		FW_REG(R_PHYACC) = value;
}

/* ---- lifecycle ---- */

static void fw_reset_state(void)
{
	/* Everything but the clock: the cable's timeline only goes forward, and a
	 * reset (or a state from before the controller was saved) is not time
	 * going back. It counts on from wherever psxRegs.cycle now stands. */
	const u64 now = fw.now;
	memset(&fw, 0, sizeof(fw));
	fw.now = now;
	/* The PHY ran its own bus reset at power on, so the node already has an
	 * ID -- the only one on a bus of one -- and says so in bit 0. Sony's
	 * driver waits for that bit before it goes any further. */
	FW_REG(R_NODEID) = 0xffc00001u;
	fw.id_valid = true;
	fw.root = true;
	FW_REG(R_POWER) = 0x10000001u;

	fw.phy[0] = 0x01;
	fw.phy[1] = 0x3f;          /* gap count */
	fw.phy[2] = 0xe2;          /* extended registers, two ports */
	fw.phy[3] = 0x40;          /* S400 */
	fw.phy[4] = 0x00;
	fw.phy[5] = 0x00;
	fw.phy_page1[0] = 0x01;    /* 1394a compliant */
	fw.phy_page1[2] = 0x08;    /* vendor 0x080046 (Sony) */
	fw.phy_page1[3] = 0x00;
	fw.phy_page1[4] = 0x46;
	fw.nodes = 1;
	fw.next_second = (u64)-1;
}

void FWopen(void)
{
	fw_reset_state();
	cable.next_rv = 0;
	cable.safe = 0;
	cable.last_stamp = 0;
	if (cable.attached)
		fw_schedule();
}

void FWclose(void)
{
	/* Debug: LRPS2_IOP_DUMP=<path> writes IOP RAM out as the machine stops,
	 * with every module the game loaded still in place. */
	const char* dump = getenv("LRPS2_IOP_DUMP");
	if (dump && iopMem)
	{
		char path[1024];
		FILE* f;
		/* One file per console on a cable: <path>.<bus index>. */
		if (cable.self >= 0)
			snprintf(path, sizeof(path), "%s.%d", dump, cable.self);
		else
			snprintf(path, sizeof(path), "%s", dump);
		f = fopen(path, "wb");
		if (f)
		{
			fwrite(iopMem->Main, 1, sizeof(iopMem->Main), f);
			fclose(f);
		}
	}
	fw_reset_state();
}

/* ---- registers ---- */

static u32 FWread32_(u32 addr)
{
	u32 off = addr - FW_BASE;
	if (off >= 0x180)
		return 0;

	switch (off)
	{
		case R_CYCLETIME:
			return fw_cycle_time();
		case R_CTRL0:
			return (FW_REG(R_CTRL0) & ~CTRL0_ROOT) | (fw.id_valid && fw.root ? CTRL0_ROOT : 0);
		case R_CTRL2:
			return (FW_REG(R_CTRL2) & ~CTRL2_SOK) | (FW_REG(R_CTRL2) & CTRL2_LPSEN ? CTRL2_SOK : 0) | CTRL2_SOK;
		case R_UBUF_RX:
		{
			/* A driver polling the FIFO sees a packet the cycle it lands. */
			fw_release();
			return fifo_pop(&fw.ubuf_rx);
		}
		case R_UBUF_RXLEVEL:
			fw_release();
			return fw.ubuf_rx.count;
		case R_DBUF_LVL0:
		case R_DBUF_LVL1:
		{
			/* Receive bytes in 28:16, transmit bytes in 12:0. */
			const fw_fifo* f = &fw.dbuf_rx[off == R_DBUF_LVL1];
			u32 tx = off == R_DBUF_LVL0 ? fw.dbuf_tx0.count * 4 : 0;
			return ((f->count * 4) & 0x1fff) << 16 | (tx & 0x1fff);
		}
		case R_DBUF_RX0:
			return fifo_pop(&fw.dbuf_rx[0]);
		case R_DBUF_RX1:
			return fifo_pop(&fw.dbuf_rx[1]);
		case R_INTR0:
		case R_INTR1:
		case R_INTR2:
			fw_release();
			return FW_REG(off);
		default:
			return FW_REG(off);
	}
}

/* Debug: LRPS2_IOP_POKE=addr=value[,addr=value...] writes IOP words on every
 * controller access -- how a driver's own trace switches get turned on. */
static void fw_debug_pokes(void)
{
	static int state = -1;
	static u32 addr[8], val[8];
	static int n;
	int i;
	if (state < 0)
	{
		const char* s = getenv("LRPS2_IOP_POKE");
		state = 0;
		n = 0;
		while (s && *s && n < 8)
		{
			char* end;
			addr[n] = (u32)strtoul(s, &end, 16);
			if (*end != '=')
				break;
			val[n] = (u32)strtoul(end + 1, &end, 16);
			n++;
			s = *end == ',' ? end + 1 : end;
		}
		state = n > 0;
	}
	for (i = 0; state && i < n; i++)
		*(u32*)&iopMem->Main[addr[i] & 0x1ffffc] = val[i];
}

u32 FWread32(u32 addr)
{
	u32 v;
	fw_debug_pokes();
	v = FWread32_(addr);
	FWTRACE("FW r32 %08x -> %08x pc=%08x\n", addr, v, psxRegs.pc);
	return v;
}

void FWwrite32(u32 addr, u32 value)
{
	u32 off = addr - FW_BASE;
	FWTRACE("FW w32 %08x <- %08x pc=%08x\n", addr, value, psxRegs.pc);
	if (off >= 0x180)
		return;

	switch (off)
	{
		case R_NODEID:
			/* The bus ID (31:22) is the driver's to set. The physical ID
			 * (21:16) is the PHY's, from the last self-ID phase, and bit 0 is
			 * the hardware saying it holds one: Sony's driver writes the bus
			 * ID with the physical-ID field zeroed, and every packet this
			 * node sent afterwards would otherwise claim to come from node 0. */
			FW_REG(R_NODEID) = (value & 0xffc00000u) | (FW_REG(R_NODEID) & 0x003f0001u);
			break;
		case R_CYCLETIME:
			fw_cycle_write(value);
			fw.next_second = fw_next_second();
			break;
		case R_CTRL0:
		{
			u32 was = FW_REG(R_CTRL0);
			if (value & CTRL0_TXRST)
				fifo_clear(&fw.ubuf_tx);
			if (value & CTRL0_RXRST)
				fifo_clear(&fw.ubuf_rx);
			FW_REG(R_CTRL0) = value & ~(CTRL0_TXRST | CTRL0_RXRST | CTRL0_BUSIDRST | CTRL0_ROOT);
			if (!(was & CTRL0_CYCTMREN) && (value & CTRL0_CYCTMREN))
			{
				fw.next_second = fw_next_second();
				fw_schedule();
			}
			break;
		}
		case R_CTRL2:
			FW_REG(R_CTRL2) = value & 0x7;
			break;
		case R_PHYACC:
			fw_phy_access(value);
			break;
		case R_INTR0:
		case R_INTR1:
		case R_INTR2:
			FW_REG(off) &= ~value;
			break;
		case R_INTR0MASK:
		case R_INTR1MASK:
		case R_INTR2MASK:
		{
			u32 was = FW_REG(off);
			FW_REG(off) = value;
			if ((value & ~was) & FW_REG(off - 4))
				fwIrq();
			break;
		}
		case R_UBUF_TXNEXT:
			fifo_push(&fw.ubuf_tx, value);
			break;
		case R_UBUF_TXLAST:
			fifo_push(&fw.ubuf_tx, value);
			fw_transmit();
			break;
		case R_UBUF_TXCLEAR:
			fifo_clear(&fw.ubuf_tx);
			break;
		case R_UBUF_RXCLEAR:
			fifo_clear(&fw.ubuf_rx);
			break;
		case R_PHT_CTRL0:
			if (value & PHT_RST)
			{
				fw.pht.active = false;
				fw.pht.waiting = false;
			}
			FW_REG(off) = value & ~(PHT_RST | PHT_PSTK | PHT_PRBR | PHT_STATUS);
			if (!(value & PHT_RST) && (value & (PHT_EWREQ | PHT_ERREQ)))
				pht_start(value);
			break;
		case R_PHT_CTRL1:
			FW_REG(off) = value & ~PHT_RST;
			break;
		case R_DBUF_TX0:
			fifo_push(&fw.dbuf_tx0, value);
			pht_step();
			break;
		case R_DBUF_LVL0:
		case R_DBUF_LVL1:
			if (value & 0x80000000u)
				fifo_clear(&fw.dbuf_rx[off == R_DBUF_LVL1]);
			if ((value & 0x8000u) && off == R_DBUF_LVL0)
				fifo_clear(&fw.dbuf_tx0);
			break;
		case R_POWER:
			break;
		default:
			FW_REG(off) = value;
			break;
	}
}

u16 FWread16(u32 addr)
{
	u16 v = (u16)(FWread32_(addr & ~3u) >> ((addr & 2) * 8));
	FWTRACE("FW r16 %08x -> %04x pc=%08x\n", addr, v, psxRegs.pc);
	return v;
}

u8 FWread8(u32 addr)
{
	u8 v = (u8)(FWread32_(addr & ~3u) >> ((addr & 3) * 8));
	FWTRACE("FW r8  %08x -> %02x pc=%08x\n", addr, v, psxRegs.pc);
	return v;
}

void FWwrite16(u32 addr, u16 value)
{
	u32 off = (addr & ~3u) - FW_BASE;
	FWTRACE("FW w16 %08x <- %04x pc=%08x\n", addr, value, psxRegs.pc);
	if (off < 0x180)
	{
		u32 shift = (addr & 2) * 8;
		FWwrite32(addr & ~3u, (FW_REG(off) & ~(0xffffu << shift)) | (u32)value << shift);
	}
}

void FWwrite8(u32 addr, u8 value)
{
	u32 off = (addr & ~3u) - FW_BASE;
	FWTRACE("FW w8  %08x <- %02x pc=%08x\n", addr, value, psxRegs.pc);
	if (off < 0x180)
	{
		u32 shift = (addr & 3) * 8;
		FWwrite32(addr & ~3u, (FW_REG(off) & ~(0xffu << shift)) | (u32)value << shift);
	}
}

/* ---- attach and detach ---- */

void FWlinkAttach(const struct retro_link_interface* link, unsigned port)
{
	if (cable.attached || !link)
		return;
	cable.handle = link->attach(port, FW_PROTOCOL, FW_IOP_HZ);
	if (!cable.handle)
	{
		if (log_cb)
			log_cb(RETRO_LOG_WARN, "i.LINK: the frontend refused a link on port %u\n", port);
		return;
	}
	cable.link = link;
	if (!rv.lock)
	{
		rv.lock = slock_new();
		rv.cond = scond_new();
	}
	rv.quit = rv.posted = rv.done = rv.pending = false;
	rv.thread = sthread_create(rv_thread, NULL);
	cable.attached = true;
	cable.self = -1;
	cable.peers = 0;
	cable.safe = 0;
	cable.next_rv = 0;
	cable.last_stamp = 0;
	cable.announced = false;
	memset(cable.peer_heard, 0, sizeof(cable.peer_heard));
	if (log_cb)
		log_cb(RETRO_LOG_INFO, "i.LINK: cable attached to port %u\n", port);
}

/* Called from the frontend's thread, possibly while the EE is inside a bus
 * call: that call returns once the port is gone, and the handle and the
 * interface are left as they are, since a frontend never reuses a handle and
 * its interface outlives the core. */
void FWlinkDetach(void)
{
	if (!cable.attached)
		return;
	cable.attached = false;
	/* Leaving the bus is also what returns a helper parked in advance(). */
	cable.link->detach(cable.handle);
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
	if (log_cb)
		log_cb(RETRO_LOG_INFO, "i.LINK: cable detached (%llu sent, %llu received)\n",
			(unsigned long long)cable.sent, (unsigned long long)cable.received);
}

/* Two consoles in one frontend usually read the same NVRAM, and so the same
 * i.LINK ID -- which is the EUI-64 a socket addresses its peer by. A console
 * on a cable salts it with the handle the frontend gave it, which is distinct
 * for every console on the bus. Zero when there is no cable. */
u32 FWlinkIdSalt(void)
{
	uintptr_t h = (uintptr_t)cable.handle;
	u32 s;
	if (!cable.attached)
		return 0;
	s = (u32)(h ^ (h >> 32));
	s = s * 2654435761u;
	return s ? s : 1;
}

/* ---- save states ----
 *
 * The controller is saved with the machine, so that a state puts the
 * controller back as the driver in the restored RAM left it. The cable is
 * not: it is the present, shared with other consoles, and its clock only ever
 * goes forward -- the bus may never see a console step back. So what is kept
 * of time is relative: pending events are saved as distances from now and
 * come back as the same distances from the moment of the load, and the cycle
 * timer as the value it showed. */

#define FW_STATE_VERSION 1

static void fw_freeze_fifo(SaveStateBase& st, fw_fifo& f)
{
	u32 n = f.count, i;
	st.Freeze(n);
	if (n > FIFO_QUADS)
		n = FIFO_QUADS;
	if (st.IsSaving())
	{
		for (i = 0; i < n; i++)
		{
			u32 v = f.q[(f.head + i) % FIFO_QUADS];
			st.Freeze(v);
		}
	}
	else
	{
		fifo_clear(&f);
		for (i = 0; i < n; i++)
		{
			u32 v = 0;
			st.Freeze(v);
			fifo_push(&f, v);
		}
	}
}

bool SaveStateBase::fwFreeze()
{
	u32 version = FW_STATE_VERSION;
	u64 now;
	u64 cycle_ticks;
	u32 nev = fw.nev, i;
	u8 flags;

	/* The restored psxRegs.cycle is a different counter from the one the
	 * clock last read: count on from it rather than take the jump between
	 * them as time passing. */
	if (IsLoading())
		fw.have_raw = false;
	now = fw_clock();
	cycle_ticks = fw_cycle_ticks();

	if (!FreezeTag("iLink"))
		return false;
	Freeze(version);
	if (IsLoading() && version != FW_STATE_VERSION)
	{
		if (log_cb)
			log_cb(RETRO_LOG_WARN, "i.LINK: state has controller version %u, not %u; starting it fresh\n",
				version, FW_STATE_VERSION);
		m_error = true;
		return false;
	}

	Freeze(fw.regs);
	Freeze(fw.phy);
	Freeze(fw.phy_page1);
	fw_freeze_fifo(*this, fw.ubuf_rx);
	fw_freeze_fifo(*this, fw.ubuf_tx);
	fw_freeze_fifo(*this, fw.dbuf_rx[0]);
	fw_freeze_fifo(*this, fw.dbuf_rx[1]);
	fw_freeze_fifo(*this, fw.dbuf_tx0);
	Freeze(fw.nodes);
	Freeze(fw.phy_id);
	flags = (u8)((fw.root ? 1 : 0) | (fw.id_valid ? 2 : 0) | (fw.tx_busy ? 4 : 0));
	Freeze(flags);
	Freeze(fw.pht);
	Freeze(cycle_ticks);

	Freeze(nev);
	if (nev > FW_EVENTS)
	{
		m_error = true;
		return false;
	}
	for (i = 0; i < nev; i++)
	{
		fw_event& e = fw.ev[i];
		s64 rel = IsSaving() ? (s64)e.tick - (s64)now : 0;
		Freeze(rel);
		Freeze(e.type);
		Freeze(e.arg);
		Freeze(e.nquads);
		if (e.nquads > FW_MAX_QUADS)
		{
			m_error = true;
			return false;
		}
		FreezeMem(e.quads, (int)(e.nquads * 4));
		if (IsLoading())
		{
			e.tick = rel > 0 ? now + (u64)rel : now;
			e.seq = fw.next_seq++;
		}
	}

	if (IsLoading())
	{
		fw.nev = nev;
		fw.root = (flags & 1) != 0;
		fw.id_valid = (flags & 2) != 0;
		fw.tx_busy = (flags & 4) != 0;
		fw.cyc_base = (s64)cycle_ticks - (s64)(now * 2 / 3);
		fw.next_second = (FW_REG(R_CTRL0) & CTRL0_CYCTMREN) ? fw_next_second() : (u64)-1;
		fw_schedule();
	}
	return IsOkay();
}
