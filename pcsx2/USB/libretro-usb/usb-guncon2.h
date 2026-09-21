/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2026 PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 */

#pragma once

#include "USB/libretro-usb/qusb.h"
#include "USB/libretro-usb/usb-guncon2-protocol.h"

#include "libretro.h"

namespace usb_guncon2
{
/* Called on the libretro thread after the frontend has been polled. */
void UpdateInput(unsigned input_port, retro_input_state_t input_cb);
/* Called by the USB/IOP thread. Returns false instead of spinning if a
 * libretro-thread publication is in progress. */
bool ReadInput(unsigned input_port, InputSnapshot* result);
void ResetInput(unsigned input_port);
void ResetAllInputs();

USBDevice* CreateDevice(u32 physical_port, unsigned input_port);
} // namespace usb_guncon2
