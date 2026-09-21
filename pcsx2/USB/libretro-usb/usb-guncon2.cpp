/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2026 PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 */

#include "USB/libretro-usb/usb-guncon2.h"

#include "USB/libretro-usb/USBinternal.h"
#include "USB/libretro-usb/desc.h"

#include "VMManager.h"
#include "common/Pcsx2Defs.h"

#include <algorithm>

namespace usb_guncon2
{
static const USBDescStrings s_desc_strings = {
	"Namco GunCon2",
};

struct GunCon2State
{
	GunCon2State(u32 physical_port_, unsigned input_port_)
		: physical_port(physical_port_)
		, input_port(input_port_)
	{
	}

	USBDevice dev{};
	USBDesc desc{};
	USBDescDevice desc_dev{};
	u32 physical_port;
	unsigned input_port;
	protocol::Calibration calibration;
	protocol::Parameters parameters;
	protocol::RecalibrationState recalibration;
	InputSnapshot last_input;
	bool auto_config_done = false;

	void AutoConfigure()
	{
		const std::string current_serial = VMManager::GetDiscSerialCopy();
		if (current_serial.empty())
			return;

		if (protocol::LookupCalibration(current_serial.c_str(), &calibration))
		{
			auto_config_done = true;
			log_cb(RETRO_LOG_INFO, "(GunCon2) USB port %u using automatic config for '%s'\n", physical_port + 1, current_serial.c_str());
			return;
		}

		auto_config_done = true;
		log_cb(RETRO_LOG_WARN, "(GunCon2) No automatic config found for '%s'; using defaults\n", current_serial.c_str());
	}
};

static void HandleControl(USBDevice* dev, USBPacket* packet, int request, int value, int index, int length, u8* data)
{
	GunCon2State* const state = USB_CONTAINER_OF(dev, GunCon2State, dev);
	if (!state->auto_config_done)
		state->AutoConfigure();

	if (usb_desc_handle_control(dev, packet, request, value, index, length, data) >= 0)
		return;

	if (request == (ClassInterfaceOutRequest | 0x09) &&
		protocol::DecodeParameters(data, (length > 0) ? static_cast<size_t>(length) : 0, &state->parameters))
	{
		return;
	}

	packet->status = USB_RET_STALL;
}

static void HandleData(USBDevice* dev, USBPacket* packet)
{
	GunCon2State* const state = USB_CONTAINER_OF(dev, GunCon2State, dev);
	if (!state->auto_config_done)
		state->AutoConfigure();

	if (packet->pid != USB_TOKEN_IN || !packet->ep || packet->ep->nr != 1)
	{
		packet->status = USB_RET_STALL;
		return;
	}

	InputSnapshot input;
	if (ReadInput(state->input_port, &input))
		state->last_input = input;

	std::array<u8, 6> report = protocol::BuildReportWithCalibration(
		state->last_input, state->calibration, state->parameters, &state->recalibration);
	const size_t length = std::min<size_t>(report.size(), packet->buffer_size);
	usb_packet_copy(packet, report.data(), length);
}

static void Unrealize(USBDevice* dev)
{
	GunCon2State* const state = USB_CONTAINER_OF(dev, GunCon2State, dev);
	delete state;
}

USBDevice* CreateDevice(u32 physical_port, unsigned input_port)
{
	GunCon2State* const state = new GunCon2State(physical_port, input_port);
	state->desc.full = &state->desc_dev;
	state->desc.str = s_desc_strings;

	if (usb_desc_parse_dev(protocol::DEVICE_DESCRIPTOR.data(), protocol::DEVICE_DESCRIPTOR.size(), state->desc, state->desc_dev) < 0 ||
		usb_desc_parse_config(protocol::CONFIG_DESCRIPTOR.data(), protocol::CONFIG_DESCRIPTOR.size(), state->desc_dev) < 0)
	{
		delete state;
		return nullptr;
	}

	state->dev.speed = USB_SPEED_FULL;
	state->dev.klass.handle_attach = usb_desc_attach;
	state->dev.klass.handle_control = HandleControl;
	state->dev.klass.handle_data = HandleData;
	state->dev.klass.unrealize = Unrealize;
	state->dev.klass.usb_desc = &state->desc;
	state->dev.klass.product_desc = "Namco GunCon2";

	usb_desc_init(&state->dev);
	usb_ep_init(&state->dev);
	log_cb(RETRO_LOG_INFO, "(GunCon2) Attached to USB port %u from frontend input port %u\n", physical_port + 1, input_port + 1);
	return &state->dev;
}
} // namespace usb_guncon2
