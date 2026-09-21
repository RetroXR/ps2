/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2026 PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 */

#include "USB/libretro-usb/usb-guncon2-protocol.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>

namespace usb_guncon2::protocol
{
struct GameConfig
{
	const char* serial;
	float scale_x;
	float scale_y;
	u32 center_x;
	u32 center_y;
	u32 screen_width;
	u32 screen_height;
};

/* GunCon 2 titles use game-specific CRT calibration. These values are the
 * historical PCSX2/nuvee table and deliberately remain local to the device. */
static constexpr GameConfig s_game_config[] = {
	{"SLES-50930", 90.25f, 94.5f, 390, 132, 640, 256}, // Dino Stalker (E, English)
	{"SLES-51095", 90.25f, 94.5f, 390, 132, 640, 256}, // Dino Stalker (E, French)
	{"SLES-51096", 90.25f, 94.5f, 390, 132, 640, 256}, // Dino Stalker (E, German)
	{"SLUS-20485", 90.25f, 92.5f, 390, 132, 640, 240}, // Dino Stalker (U)
	{"SLUS-20389", 89.25f, 93.5f, 422, 141, 640, 240}, // Endgame (U)
	{"SLES-50936", 112.0f, 100.0f, 320, 120, 512, 256}, // Endgame (E, GunCon 2 on USB port 2)
	{"SLPM-65139", 90.0f, 91.5f, 320, 120, 640, 240}, // Gun Survivor 3: Dino Crisis (J)
	{"SLES-52620", 89.5f, 112.3f, 390, 147, 640, 256}, // Guncom 2 (E)
	{"SLES-51289", 84.5f, 89.0f, 456, 164, 640, 256}, // Gunfighter 2 - Jesse James (E)
	{"SLPS-25165", 90.25f, 98.0f, 390, 138, 640, 240}, // Gunvari Collection (J, 480i)
	{"SCES-50889", 90.25f, 94.5f, 390, 169, 640, 256}, // Ninja Assault (E)
	{"SLPS-20218", 90.0f, 92.0f, 320, 134, 640, 240}, // Ninja Assault (J)
	{"SLUS-20492", 90.25f, 92.5f, 390, 132, 640, 240}, // Ninja Assault (U)
	{"SLES-50650", 84.75f, 96.0f, 454, 164, 640, 240}, // Resident Evil Survivor 2 (E)
	{"SLES-51448", 90.25f, 95.0f, 420, 132, 640, 240}, // Resident Evil - Dead Aim (E)
	{"SLUS-20669", 90.25f, 93.5f, 420, 132, 640, 240}, // Resident Evil - Dead Aim (U)
	{"SLUS-20619", 90.25f, 91.75f, 453, 154, 640, 256}, // Starsky & Hutch (U)
	{"SCES-50300", 90.25f, 102.75f, 390, 138, 640, 256}, // Time Crisis II (E)
	{"SLUS-20219", 90.25f, 97.5f, 390, 154, 640, 240}, // Time Crisis 2 (U)
	{"SCES-51844", 90.25f, 102.75f, 390, 138, 640, 256}, // Time Crisis 3 (E)
	{"SLUS-20645", 90.25f, 97.5f, 390, 154, 640, 240}, // Time Crisis 3 (U)
	{"SCES-52530", 90.25f, 99.0f, 390, 153, 640, 256}, // Crisis Zone (E)
	{"SLUS-20927", 90.25f, 99.0f, 390, 153, 640, 240}, // Time Crisis - Crisis Zone (U, 480i)
	{"SCES-50411", 89.8f, 99.9f, 421, 138, 640, 256}, // Vampire Night (E)
	{"SLPS-25077", 90.0f, 97.5f, 422, 118, 640, 240}, // Vampire Night (J)
	{"SLUS-20221", 89.8f, 102.5f, 422, 124, 640, 228}, // Vampire Night (U)
	{"SLES-51229", 110.15f, 100.0f, 433, 159, 512, 256}, // Virtua Cop - Elite Edition (E,J, 480i)
};

Position CalculatePosition(const InputSnapshot& input, const Calibration& calibration, const Parameters& parameters)
{
	if (input.offscreen || input.reload)
		return {0, 0};

	const float pointer_x = (static_cast<float>(input.screen_x) + 32768.0f) / 65535.0f;
	const float pointer_y = (static_cast<float>(input.screen_y) + 32768.0f) / 65535.0f;

	float fx = (pointer_x * static_cast<float>(calibration.screen_width)) - static_cast<float>(calibration.screen_width / 2u);
	float fy = (pointer_y * static_cast<float>(calibration.screen_height)) - static_cast<float>(calibration.screen_height / 2u);
	fx *= calibration.scale_x;
	fy *= calibration.scale_y;

	s32 x = static_cast<s32>(std::round(fx + calibration.center_x));
	s32 y = static_cast<s32>(std::round(fy + calibration.center_y));
	if (parameters.mode & PROGRESSIVE_FLAG)
	{
		x -= parameters.x / 2;
		y -= parameters.y / 2;
	}
	else
	{
		x -= parameters.x;
		y -= parameters.y;
	}

	x = std::clamp<s32>(x, 1, INT16_MAX);
	y = std::clamp<s32>(y, 1, INT16_MAX);
	return {static_cast<s16>(x), static_cast<s16>(y)};
}

static std::array<u8, 6> EncodeReport(u16 pressed_buttons, Position position, const Parameters& parameters)
{
	const u16 buttons = static_cast<u16>(~pressed_buttons) | (parameters.mode & PROGRESSIVE_FLAG);
	const u16 x = static_cast<u16>(position.x);
	const u16 y = static_cast<u16>(position.y);
	return {
		static_cast<u8>(buttons),
		static_cast<u8>(buttons >> 8),
		static_cast<u8>(x),
		static_cast<u8>(x >> 8),
		static_cast<u8>(y),
		static_cast<u8>(y >> 8),
	};
}

std::array<u8, 6> BuildReport(const InputSnapshot& input, const Calibration& calibration, const Parameters& parameters)
{
	u16 pressed_buttons = input.buttons;
	if (input.reload)
		pressed_buttons |= BUTTON_TRIGGER;
	return EncodeReport(pressed_buttons, CalculatePosition(input, calibration, parameters), parameters);
}

std::array<u8, 6> BuildReportWithCalibration(const InputSnapshot& input, const Calibration& calibration,
	const Parameters& parameters, RecalibrationState* state)
{
	if (!state)
		return BuildReport(input, calibration, parameters);

	if (input.recalibrate && state->reports_remaining == 0)
	{
		InputSnapshot aim_input = input;
		aim_input.reload = false;
		state->reports_remaining = RECALIBRATION_REPORTS;
		state->latched_position = CalculatePosition(aim_input, calibration, parameters);
	}

	if (state->reports_remaining > 0)
	{
		Position position = state->latched_position;
		state->reports_remaining--;
		/* Match PCSX2's GunCon 2 timing exactly: games wait through the
		 * latched reports, then detect calibration from five (0, 0) polls. */
		if (state->reports_remaining < RECALIBRATION_ZERO_REPORTS)
			position = {0, 0};
		return EncodeReport(input.buttons | BUTTON_TRIGGER, position, parameters);
	}

	return BuildReport(input, calibration, parameters);
}

bool DecodeParameters(const u8* data, size_t size, Parameters* parameters)
{
	if (!data || size < 6 || !parameters)
		return false;

	const auto decode_s16 = [](u8 lo, u8 hi) {
		const u16 value = static_cast<u16>(lo) | (static_cast<u16>(hi) << 8);
		return static_cast<s16>((value <= INT16_MAX) ? static_cast<s32>(value) : static_cast<s32>(value) - 0x10000);
	};

	parameters->x = decode_s16(data[0], data[1]);
	parameters->y = decode_s16(data[2], data[3]);
	parameters->mode = static_cast<u16>(data[4]) | (static_cast<u16>(data[5]) << 8);
	return true;
}

bool LookupCalibration(const char* serial, Calibration* calibration)
{
	if (!serial || !serial[0] || !calibration)
		return false;

	for (const GameConfig& config : s_game_config)
	{
		if (std::strcmp(serial, config.serial) != 0)
			continue;

		calibration->scale_x = config.scale_x / 100.0f;
		calibration->scale_y = config.scale_y / 100.0f;
		calibration->center_x = static_cast<float>(config.center_x);
		calibration->center_y = static_cast<float>(config.center_y);
		calibration->screen_width = config.screen_width;
		calibration->screen_height = config.screen_height;
		return true;
	}

	return false;
}
} // namespace usb_guncon2::protocol
