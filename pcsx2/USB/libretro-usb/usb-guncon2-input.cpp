/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2026 PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 */

#include "USB/libretro-usb/usb-guncon2.h"

#include <retro_atomic.h>

namespace usb_guncon2
{
struct AtomicInputSnapshot
{
	retro_atomic_int_t sequence;
	retro_atomic_int_t screen_x;
	retro_atomic_int_t screen_y;
	retro_atomic_int_t buttons;
	retro_atomic_int_t flags;
};

enum : int
{
	INPUT_FLAG_OFFSCREEN = 1 << 0,
	INPUT_FLAG_RELOAD = 1 << 1,
	INPUT_FLAG_RECALIBRATE = 1 << 2,
};

static AtomicInputSnapshot s_inputs[2] = {
	{RETRO_ATOMIC_INT_INITIALIZER(0), RETRO_ATOMIC_INT_INITIALIZER(0), RETRO_ATOMIC_INT_INITIALIZER(0), RETRO_ATOMIC_INT_INITIALIZER(0), RETRO_ATOMIC_INT_INITIALIZER(INPUT_FLAG_OFFSCREEN)},
	{RETRO_ATOMIC_INT_INITIALIZER(0), RETRO_ATOMIC_INT_INITIALIZER(0), RETRO_ATOMIC_INT_INITIALIZER(0), RETRO_ATOMIC_INT_INITIALIZER(0), RETRO_ATOMIC_INT_INITIALIZER(INPUT_FLAG_OFFSCREEN)},
};

static void PublishInput(unsigned input_port, const InputSnapshot& input)
{
	if (input_port >= 2)
		return;

	AtomicInputSnapshot& dst = s_inputs[input_port];
	u32 sequence = static_cast<u32>(retro_atomic_load_acquire_int(&dst.sequence));
	if (sequence & 1)
		sequence++;

	/* The odd sequence prevents the USB thread from accepting a partially
	 * published frame. The payload fields are atomic as well, so retries do
	 * not race with the libretro-thread writer. */
	retro_atomic_exchange_int(&dst.sequence, static_cast<int>(sequence + 1u));
	retro_atomic_store_release_int(&dst.screen_x, input.screen_x);
	retro_atomic_store_release_int(&dst.screen_y, input.screen_y);
	retro_atomic_store_release_int(&dst.buttons, input.buttons);
	retro_atomic_store_release_int(&dst.flags,
		(input.offscreen ? INPUT_FLAG_OFFSCREEN : 0) |
			(input.reload ? INPUT_FLAG_RELOAD : 0) |
			(input.recalibrate ? INPUT_FLAG_RECALIBRATE : 0));
	retro_atomic_store_release_int(&dst.sequence, static_cast<int>(sequence + 2u));
}

bool ReadInput(unsigned input_port, InputSnapshot* result)
{
	if (input_port >= 2 || !result)
		return false;

	AtomicInputSnapshot& src = s_inputs[input_port];
	for (unsigned attempt = 0; attempt < 4; attempt++)
	{
		const int before = retro_atomic_load_acquire_int(&src.sequence);
		if (before & 1)
			continue;

		InputSnapshot candidate;
		candidate.screen_x = static_cast<s16>(retro_atomic_load_acquire_int(&src.screen_x));
		candidate.screen_y = static_cast<s16>(retro_atomic_load_acquire_int(&src.screen_y));
		candidate.buttons = static_cast<u16>(retro_atomic_load_acquire_int(&src.buttons));
		const int flags = retro_atomic_load_acquire_int(&src.flags);
		candidate.offscreen = (flags & INPUT_FLAG_OFFSCREEN) != 0;
		candidate.reload = (flags & INPUT_FLAG_RELOAD) != 0;
		candidate.recalibrate = (flags & INPUT_FLAG_RECALIBRATE) != 0;

		const int after = retro_atomic_load_acquire_int(&src.sequence);
		if (before == after && !(after & 1))
		{
			*result = candidate;
			return true;
		}
	}

	return false;
}

void UpdateInput(unsigned input_port, retro_input_state_t input_cb)
{
	if (input_port >= 2 || !input_cb)
		return;

	InputSnapshot input;
	input.screen_x = input_cb(input_port, RETRO_DEVICE_LIGHTGUN, 0, RETRO_DEVICE_ID_LIGHTGUN_SCREEN_X);
	input.screen_y = input_cb(input_port, RETRO_DEVICE_LIGHTGUN, 0, RETRO_DEVICE_ID_LIGHTGUN_SCREEN_Y);
	input.offscreen = input_cb(input_port, RETRO_DEVICE_LIGHTGUN, 0, RETRO_DEVICE_ID_LIGHTGUN_IS_OFFSCREEN) != 0;
	input.reload = input_cb(input_port, RETRO_DEVICE_LIGHTGUN, 0, RETRO_DEVICE_ID_LIGHTGUN_RELOAD) != 0;

	const struct
	{
		unsigned id;
		u16 bit;
	} button_map[] = {
		{RETRO_DEVICE_ID_LIGHTGUN_AUX_C, BUTTON_C},
		{RETRO_DEVICE_ID_LIGHTGUN_AUX_B, BUTTON_B},
		{RETRO_DEVICE_ID_LIGHTGUN_AUX_A, BUTTON_A},
		{RETRO_DEVICE_ID_LIGHTGUN_DPAD_UP, BUTTON_DPAD_UP},
		{RETRO_DEVICE_ID_LIGHTGUN_DPAD_RIGHT, BUTTON_DPAD_RIGHT},
		{RETRO_DEVICE_ID_LIGHTGUN_DPAD_DOWN, BUTTON_DPAD_DOWN},
		{RETRO_DEVICE_ID_LIGHTGUN_DPAD_LEFT, BUTTON_DPAD_LEFT},
		{RETRO_DEVICE_ID_LIGHTGUN_TRIGGER, BUTTON_TRIGGER},
		{RETRO_DEVICE_ID_LIGHTGUN_SELECT, BUTTON_SELECT},
		{RETRO_DEVICE_ID_LIGHTGUN_START, BUTTON_START},
	};

	for (const auto& binding : button_map)
	{
		if (input_cb(input_port, RETRO_DEVICE_LIGHTGUN, 0, binding.id))
			input.buttons |= binding.bit;
	}

	/* The calibration shot is Start + Select held together. Current
	 * frontends read the deprecated LIGHTGUN_PAUSE ID through the Start
	 * bind, so it cannot be told apart from Start and would fire a
	 * recalibration on every Start press. The chord is withheld from the
	 * game so it does not also see Start and Select. */
	const u16 chord = BUTTON_START | BUTTON_SELECT;
	if ((input.buttons & chord) == chord)
	{
		input.recalibrate = true;
		input.buttons &= static_cast<u16>(~chord);
	}

	PublishInput(input_port, input);
}

void ResetInput(unsigned input_port)
{
	PublishInput(input_port, InputSnapshot{});
}

void ResetAllInputs()
{
	for (unsigned port = 0; port < 2; port++)
		ResetInput(port);
}
} // namespace usb_guncon2
