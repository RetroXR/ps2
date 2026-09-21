/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2026 PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 */

#pragma once

#include "common/Pcsx2Types.h"

#include <array>
#include <cstddef>

namespace usb_guncon2
{
enum : u16
{
	BUTTON_C = 1u << 1,
	BUTTON_B = 1u << 2,
	BUTTON_A = 1u << 3,
	BUTTON_DPAD_UP = 1u << 4,
	BUTTON_DPAD_RIGHT = 1u << 5,
	BUTTON_DPAD_DOWN = 1u << 6,
	BUTTON_DPAD_LEFT = 1u << 7,
	BUTTON_TRIGGER = 1u << 13,
	BUTTON_SELECT = 1u << 14,
	BUTTON_START = 1u << 15,
};

struct InputSnapshot
{
	s16 screen_x = 0;
	s16 screen_y = 0;
	u16 buttons = 0;
	bool offscreen = true;
	bool reload = false;
	bool recalibrate = false;
};

namespace protocol
{
constexpr u16 PROGRESSIVE_FLAG = 0x0100;
constexpr u16 RECALIBRATION_REPORTS = 12;
constexpr u16 RECALIBRATION_ZERO_REPORTS = 5;

inline constexpr std::array<u8, 18> DEVICE_DESCRIPTOR = {
	0x12, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x08,
	0x9a, 0x0b, 0x6a, 0x01, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x01,
};

inline constexpr std::array<u8, 25> CONFIG_DESCRIPTOR = {
	0x09, 0x02, 0x19, 0x00, 0x01, 0x01, 0x00, 0x80,
	0x19, 0x09, 0x04, 0x00, 0x00, 0x01, 0xff, 0x6a,
	0x00, 0x00, 0x07, 0x05, 0x81, 0x03, 0x08, 0x00,
	0x08,
};

struct Calibration
{
	u32 screen_width = 640;
	u32 screen_height = 240;
	float center_x = 320.0f;
	float center_y = 120.0f;
	float scale_x = 1.0f;
	float scale_y = 1.0f;
};

struct Parameters
{
	s16 x = 0;
	s16 y = 0;
	u16 mode = 0;
};

struct Position
{
	s16 x;
	s16 y;
};

struct RecalibrationState
{
	u16 reports_remaining = 0;
	Position latched_position{};
};

Position CalculatePosition(const InputSnapshot& input, const Calibration& calibration, const Parameters& parameters);
std::array<u8, 6> BuildReport(const InputSnapshot& input, const Calibration& calibration, const Parameters& parameters);
std::array<u8, 6> BuildReportWithCalibration(const InputSnapshot& input, const Calibration& calibration,
	const Parameters& parameters, RecalibrationState* state);
bool DecodeParameters(const u8* data, size_t size, Parameters* parameters);
bool LookupCalibration(const char* serial, Calibration* calibration);
} // namespace protocol
} // namespace usb_guncon2
