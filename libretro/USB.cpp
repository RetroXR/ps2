/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2021  PCSX2 Dev Team
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

#include <cstdlib>

#include <retro_atomic.h>

#include "../pcsx2/USB/USB.h"
#include "../pcsx2/USB/libretro-usb/USBinternal.h"
#include "../pcsx2/USB/libretro-usb/usb-guncon2.h"
#include "../pcsx2/USB/libretro-usb/usb-hid.h"
#include "../pcsx2/SaveState.h"

#include "libretro.h"

/* IOP clock, 36.864 MHz. */
#define PSXCLK 36864000

/* OHCI host-controller clock-timing globals. Computed inside ohci_create()
 * from usb_get_ticks_per_second(); defined here so the controller core has
 * storage to link against. */
s64 g_usb_frame_time = 0;
s64 g_usb_bit_time   = 0;
s64 g_usb_last_cycle = 0;

static OHCIState* s_qemu_ohci   = nullptr;
static s64        s_usb_clocks  = 0;
static s64        s_usb_remaining = 0;

/* Desired devices are written by retro_set_controller_port_device() on the
 * libretro thread and consumed by USBasync() on the CPU/IOP thread. Device and
 * input port share one atomic word so the consumer cannot observe a mixed pair. */
static constexpr int PackPortSelection(int device, unsigned input_port)
{
	return (device & 0xffff) | (static_cast<int>(input_port & 0xffff) << 16);
}

static void UnpackPortSelection(int selection, int* device, unsigned* input_port)
{
	*device = selection & 0xffff;
	*input_port = static_cast<unsigned>(selection >> 16) & 0xffff;
}

static retro_atomic_int_t s_port_selection[USB::NUM_PORTS] = {
	RETRO_ATOMIC_INT_INITIALIZER(PackPortSelection(USB_DEV_NONE, 0)),
	RETRO_ATOMIC_INT_INITIALIZER(PackPortSelection(USB_DEV_NONE, 1))};
static retro_atomic_int_t s_devices_dirty = RETRO_ATOMIC_INT_INITIALIZER(0);

/* Attached-device metadata is owned exclusively by the CPU/IOP thread. */
static USBDevice* s_usb_device[USB::NUM_PORTS] = { nullptr, nullptr };
static int        s_usb_device_type[USB::NUM_PORTS] = { USB_DEV_NONE, USB_DEV_NONE };
static unsigned   s_usb_device_input_port[USB::NUM_PORTS] = { 0, 1 };

/* From PAD.cpp - the libretro input_state callback the frontend installed. */
extern retro_input_state_t PADGetInputStateCallback(void);

/* Set the device type for a USB port and which frontend controller port it
 * reads input from. Takes effect on the next USBopen()/USBreset(). For most
 * cases input_port == port; the combined keyboard+mouse selection routes both
 * USB ports to the same frontend port. */
void USBSetPortDevice(unsigned port, int device, unsigned input_port)
{
	if (port < USB::NUM_PORTS)
	{
		retro_atomic_store_release_int(&s_port_selection[port], PackPortSelection(device, input_port));
		retro_atomic_store_release_int(&s_devices_dirty, 1);
	}
}

static OHCIPort& GetOHCIPort(u32 port)
{
	const u32 rhport = (port < s_qemu_ohci->num_ports) ? port : 0;
	return s_qemu_ohci->rhport[rhport];
}

static void USBCreateDevice(u32 port)
{
	int device;
	unsigned input_port;
	UnpackPortSelection(retro_atomic_load_acquire_int(&s_port_selection[port]), &device, &input_port);
	USBDevice* dev = nullptr;
	switch (device)
	{
		case USB_DEV_KEYBOARD:
			dev = usb_hid::usb_hid_create_kbd(port);
			break;
		case USB_DEV_MOUSE:
			dev = usb_hid::usb_hid_create_mouse(port);
			break;
		case USB_DEV_GUNCON2:
			dev = usb_guncon2::CreateDevice(port, input_port);
			break;
		default:
			return;
	}
	if (!dev)
		return;

	GetOHCIPort(port).port.dev = dev;
	dev->attached = true;
	usb_attach(&GetOHCIPort(port).port);
	s_usb_device[port] = dev;
	s_usb_device_type[port] = device;
	s_usb_device_input_port[port] = input_port;
}

static void USBDestroyDevice(u32 port)
{
	USBDevice* dev = s_usb_device[port];
	if (dev)
	{
		USBPort& ohci_port = GetOHCIPort(port).port;
		if (dev->state != USB_STATE_NOTATTACHED)
			usb_detach(&ohci_port);
		if (dev->klass.unrealize)
			dev->klass.unrealize(dev);
		ohci_port.dev = nullptr;
	}
	s_usb_device[port] = nullptr;
	s_usb_device_type[port] = USB_DEV_NONE;
	s_usb_device_input_port[port] = port;
}

static void USBReconcileDevices()
{
	for (u32 port = 0; port < USB::NUM_PORTS; port++)
	{
		int desired_device;
		unsigned desired_input_port;
		UnpackPortSelection(retro_atomic_load_acquire_int(&s_port_selection[port]),
			&desired_device, &desired_input_port);
		if (s_usb_device_type[port] == desired_device &&
			(s_usb_device_type[port] == USB_DEV_NONE || s_usb_device_input_port[port] == desired_input_port))
		{
			continue;
		}

		USBDestroyDevice(port);
		USBCreateDevice(port);
	}
}

int usb_get_ticks_per_second(void)
{
	return PSXCLK;
}

s64 usb_get_clock(void)
{
	return s_usb_clocks;
}

void USB::CheckForConfigChanges(const Pcsx2Config& old_config) { }

void USBconfigure(void) {}

void USBinit(void) {}

void USBshutdown(void) {}

bool USBopen(void)
{
	s_qemu_ohci = ohci_create(0x1f801600, 2);
	if (!s_qemu_ohci)
		return false;

	s_usb_clocks     = 0;
	s_usb_remaining  = 0;
	g_usb_last_cycle = 0;

	for (u32 port = 0; port < USB::NUM_PORTS; port++)
		USBCreateDevice(port);
	/* Do not clear s_devices_dirty here: a frontend selection may race this
	 * open, and USBasync() must still reconcile the latest packed selection. */
	return true;
}

void USBclose(void)
{
	for (u32 port = 0; port < USB::NUM_PORTS; port++)
		USBDestroyDevice(port);
	if (s_qemu_ohci)
	{
		free(s_qemu_ohci);
		s_qemu_ohci = nullptr;
	}
}

void USBreset(void)
{
	u32 port;

	s_usb_clocks     = 0;
	s_usb_remaining  = 0;
	g_usb_last_cycle = 0;
	if (!s_qemu_ohci)
		return;

	/* Reconcile attached devices with the current per-port selection, so a
	 * device-type change applied while running takes effect on reset. */
	for (port = 0; port < USB::NUM_PORTS; port++)
		USBDestroyDevice(port);

	ohci_hard_reset(s_qemu_ohci);

	for (port = 0; port < USB::NUM_PORTS; port++)
		USBCreateDevice(port);
	/* As in USBopen(), preserve a concurrently published dirty notification. */
}

void USBasync(u32 cycles)
{
	if (!s_qemu_ohci)
		return;

	/* RetroArch commonly selects controller devices after retro_load_game(),
	 * when USBopen() has already attached the initial set. Reconcile here so
	 * creation/destruction remains on the CPU/IOP thread and takes effect
	 * before the guest's next OHCI transaction. */
	if (retro_atomic_exchange_int(&s_devices_dirty, 0))
		USBReconcileDevices();

	/* Pump one frame of frontend keyboard/mouse input into attached HID
	 * devices before advancing the controller. GunCon input is published by
	 * Input::Update on the libretro thread and must never pass through this
	 * HID-only cast. Dispatch on the attached type rather than the pending
	 * selection, since selections take effect at USBopen()/USBreset(). */
	{
		retro_input_state_t input_cb = PADGetInputStateCallback();
		u32 port;
		for (port = 0; port < USB::NUM_PORTS; port++)
		{
			if (s_usb_device[port] &&
				(s_usb_device_type[port] == USB_DEV_KEYBOARD || s_usb_device_type[port] == USB_DEV_MOUSE))
			{
				usb_hid::usb_hid_update(s_usb_device[port], input_cb, s_usb_device_input_port[port]);
			}
		}
	}

	s_usb_remaining += cycles;
	s_usb_clocks    += s_usb_remaining;
	if (s_qemu_ohci->eof_timer > 0)
	{
		while ((u64)s_usb_remaining >= s_qemu_ohci->eof_timer)
		{
			s_usb_remaining -= s_qemu_ohci->eof_timer;
			s_qemu_ohci->eof_timer = 0;
			ohci_frame_boundary(s_qemu_ohci);

			/* Break out of the loop if the bus was stopped. */
			if (!s_qemu_ohci->eof_timer)
				break;
		}
		if ((s_usb_remaining > 0) && (s_qemu_ohci->eof_timer > 0))
		{
			s64 m = s_qemu_ohci->eof_timer;
			if (s_usb_remaining < m)
				m = s_usb_remaining;
			s_qemu_ohci->eof_timer -= m;
			s_usb_remaining        -= m;
		}
	}
}

s32 USBfreeze(FreezeAction mode, freezeData* data) { return 0; }

u8 USBread8(u32 addr) { return 0; }
u16 USBread16(u32 addr) { return 0; }

u32 USBread32(u32 addr)
{
	if (!s_qemu_ohci)
		return 0;
	return ohci_mem_read(s_qemu_ohci, addr);
}

void USBwrite8(u32 addr, u8 value) {}
void USBwrite16(u32 addr, u16 value) {}

void USBwrite32(u32 addr, u32 value)
{
	if (s_qemu_ohci)
		ohci_mem_write(s_qemu_ohci, addr, value);
}
