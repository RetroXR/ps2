/* GunCon 2 protocol tests: host coordinates/buttons to the exact six-byte
 * device report consumed by PS2 games. */

#include "USB/libretro-usb/usb-guncon2.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

using namespace usb_guncon2;

static int failures;

#define CHECK(condition) \
	do \
	{ \
		if (!(condition)) \
		{ \
			std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
			failures++; \
		} \
	} while (0)

static u16 ReportButtons(const std::array<u8, 6>& report)
{
	return static_cast<u16>(report[0]) | (static_cast<u16>(report[1]) << 8);
}

static s16 ReportCoordinate(const std::array<u8, 6>& report, size_t offset)
{
	const u16 value = static_cast<u16>(report[offset]) | (static_cast<u16>(report[offset + 1]) << 8);
	return static_cast<s16>((value <= 0x7fff) ? static_cast<s32>(value) : static_cast<s32>(value) - 0x10000);
}

static bool Near(float a, float b)
{
	return std::fabs(a - b) < 0.0001f;
}

static void TestDescriptors()
{
	const auto& device = protocol::DEVICE_DESCRIPTOR;
	CHECK(device.size() == 18);
	CHECK(device[0] == 0x12 && device[1] == 0x01);
	CHECK(device[2] == 0x00 && device[3] == 0x01);
	CHECK(device[7] == 0x08);
	CHECK(device[8] == 0x9a && device[9] == 0x0b);
	CHECK(device[10] == 0x6a && device[11] == 0x01);
	CHECK(device[17] == 0x01);

	const auto& config = protocol::CONFIG_DESCRIPTOR;
	CHECK(config.size() == 25);
	CHECK(config[0] == 0x09 && config[1] == 0x02);
	CHECK(config[2] == config.size() && config[3] == 0x00);
	CHECK(config[14] == 0xff && config[15] == 0x6a);
	CHECK(config[20] == 0x81);
	CHECK(config[21] == 0x03);
	CHECK(config[22] == 0x08 && config[23] == 0x00);
	CHECK(config[24] == 0x08);
}

static void TestCoordinates()
{
	protocol::Calibration calibration;
	protocol::Parameters parameters;
	InputSnapshot input;
	input.offscreen = false;

	protocol::Position position = protocol::CalculatePosition(input, calibration, parameters);
	CHECK(position.x == 320);
	CHECK(position.y == 120);

	input.screen_x = -32768;
	input.screen_y = -32768;
	position = protocol::CalculatePosition(input, calibration, parameters);
	CHECK(position.x == 1);
	CHECK(position.y == 1);

	input.screen_x = 32767;
	input.screen_y = 32767;
	position = protocol::CalculatePosition(input, calibration, parameters);
	CHECK(position.x == 640);
	CHECK(position.y == 240);

	input.screen_x = 0;
	input.screen_y = 0;
	parameters.x = 10;
	parameters.y = -10;
	position = protocol::CalculatePosition(input, calibration, parameters);
	CHECK(position.x == 310);
	CHECK(position.y == 130);

	parameters.mode = protocol::PROGRESSIVE_FLAG;
	position = protocol::CalculatePosition(input, calibration, parameters);
	CHECK(position.x == 315);
	CHECK(position.y == 125);

	input.offscreen = true;
	position = protocol::CalculatePosition(input, calibration, parameters);
	CHECK(position.x == 0 && position.y == 0);

	input.offscreen = false;
	input.reload = true;
	position = protocol::CalculatePosition(input, calibration, parameters);
	CHECK(position.x == 0 && position.y == 0);
}

static void TestReports()
{
	protocol::Calibration calibration;
	protocol::Parameters parameters;
	InputSnapshot input;
	input.offscreen = false;

	auto report = protocol::BuildReport(input, calibration, parameters);
	CHECK(ReportButtons(report) == 0xffff);
	CHECK(ReportCoordinate(report, 2) == 320);
	CHECK(ReportCoordinate(report, 4) == 120);

	const u16 button_bits[] = {
		BUTTON_C, BUTTON_B, BUTTON_A,
		BUTTON_DPAD_UP, BUTTON_DPAD_RIGHT, BUTTON_DPAD_DOWN, BUTTON_DPAD_LEFT,
		BUTTON_TRIGGER, BUTTON_SELECT, BUTTON_START,
	};
	for (const u16 bit : button_bits)
	{
		input.buttons = bit;
		report = protocol::BuildReport(input, calibration, parameters);
		CHECK(ReportButtons(report) == static_cast<u16>(~bit));
	}

	input.buttons = BUTTON_A | BUTTON_TRIGGER | BUTTON_START;
	report = protocol::BuildReport(input, calibration, parameters);
	CHECK(ReportButtons(report) == static_cast<u16>(~input.buttons));

	input.buttons = 0;
	input.reload = true;
	report = protocol::BuildReport(input, calibration, parameters);
	CHECK((ReportButtons(report) & BUTTON_TRIGGER) == 0);
	CHECK(ReportCoordinate(report, 2) == 0);
	CHECK(ReportCoordinate(report, 4) == 0);

	input.reload = false;
	input.offscreen = true;
	report = protocol::BuildReport(input, calibration, parameters);
	CHECK(ReportButtons(report) == 0xffff);
	CHECK(ReportCoordinate(report, 2) == 0);
	CHECK(ReportCoordinate(report, 4) == 0);

	input.offscreen = false;
	input.buttons = protocol::PROGRESSIVE_FLAG;
	parameters.mode = 0;
	report = protocol::BuildReport(input, calibration, parameters);
	CHECK((ReportButtons(report) & protocol::PROGRESSIVE_FLAG) == 0);
	parameters.mode = protocol::PROGRESSIVE_FLAG;
	report = protocol::BuildReport(input, calibration, parameters);
	CHECK((ReportButtons(report) & protocol::PROGRESSIVE_FLAG) != 0);
}

static void TestRecalibrationSequence()
{
	protocol::Calibration calibration;
	protocol::Parameters parameters;
	protocol::RecalibrationState state;
	InputSnapshot input;
	input.offscreen = false;
	input.recalibrate = true;

	for (u16 report_index = 0; report_index < protocol::RECALIBRATION_REPORTS; report_index++)
	{
		const auto report = protocol::BuildReportWithCalibration(input, calibration, parameters, &state);
		CHECK((ReportButtons(report) & BUTTON_TRIGGER) == 0);
		const u16 reports_remaining = protocol::RECALIBRATION_REPORTS - report_index - 1;
		CHECK(state.reports_remaining == reports_remaining);
		if (reports_remaining < protocol::RECALIBRATION_ZERO_REPORTS)
		{
			CHECK(ReportCoordinate(report, 2) == 0);
			CHECK(ReportCoordinate(report, 4) == 0);
		}
		else
		{
			CHECK(ReportCoordinate(report, 2) == 320);
			CHECK(ReportCoordinate(report, 4) == 120);
		}
	}

	/* Holding the binding repeats the upstream twelve-report sequence. */
	auto report = protocol::BuildReportWithCalibration(input, calibration, parameters, &state);
	CHECK(state.reports_remaining == protocol::RECALIBRATION_REPORTS - 1);
	CHECK(ReportCoordinate(report, 2) == 320);
	CHECK(ReportCoordinate(report, 4) == 120);

	/* Reload must not replace the position latched by a calibration shot. */
	state = {};
	input.reload = true;
	report = protocol::BuildReportWithCalibration(input, calibration, parameters, &state);
	CHECK(ReportCoordinate(report, 2) == 320);
	CHECK(ReportCoordinate(report, 4) == 120);

	state = {};
	input.reload = false;
	report = protocol::BuildReportWithCalibration(input, calibration, parameters, nullptr);
	CHECK(ReportCoordinate(report, 2) == 320);
	CHECK(ReportCoordinate(report, 4) == 120);
}

static void TestParameterDecoding()
{
	const u8 encoded[] = {0x34, 0x12, 0x00, 0x80, 0x00, 0x01};
	protocol::Parameters parameters;
	CHECK(protocol::DecodeParameters(encoded, sizeof(encoded), &parameters));
	CHECK(parameters.x == 0x1234);
	CHECK(parameters.y == -32768);
	CHECK(parameters.mode == protocol::PROGRESSIVE_FLAG);

	const protocol::Parameters original = parameters;
	CHECK(!protocol::DecodeParameters(encoded, 5, &parameters));
	CHECK(parameters.x == original.x && parameters.y == original.y && parameters.mode == original.mode);
	CHECK(!protocol::DecodeParameters(nullptr, sizeof(encoded), &parameters));
	CHECK(!protocol::DecodeParameters(encoded, sizeof(encoded), nullptr));
}

static s16 s_frontend_input[2][RETRO_DEVICE_ID_LIGHTGUN_RELOAD + 1];

static s16 RETRO_CALLCONV FakeInput(unsigned port, unsigned device, unsigned index, unsigned id)
{
	if (port >= 2 || device != RETRO_DEVICE_LIGHTGUN || index != 0 || id > RETRO_DEVICE_ID_LIGHTGUN_RELOAD)
		return 0;
	return s_frontend_input[port][id];
}

static void TestInputBridge()
{
	std::memset(s_frontend_input, 0, sizeof(s_frontend_input));
	ResetAllInputs();

	s_frontend_input[0][RETRO_DEVICE_ID_LIGHTGUN_SCREEN_X] = -1234;
	s_frontend_input[0][RETRO_DEVICE_ID_LIGHTGUN_SCREEN_Y] = 5678;
	s_frontend_input[0][RETRO_DEVICE_ID_LIGHTGUN_TRIGGER] = 1;
	s_frontend_input[0][RETRO_DEVICE_ID_LIGHTGUN_AUX_A] = 1;
	s_frontend_input[0][RETRO_DEVICE_ID_LIGHTGUN_AUX_C] = 1;
	s_frontend_input[0][RETRO_DEVICE_ID_LIGHTGUN_DPAD_LEFT] = 1;
	s_frontend_input[0][RETRO_DEVICE_ID_LIGHTGUN_START] = 1;
	/* Current frontends answer the deprecated PAUSE ID from the Start bind;
	 * Start alone must not be read as the calibration shot. */
	s_frontend_input[0][RETRO_DEVICE_ID_LIGHTGUN_PAUSE] = 1;

	s_frontend_input[1][RETRO_DEVICE_ID_LIGHTGUN_SCREEN_X] = 2345;
	s_frontend_input[1][RETRO_DEVICE_ID_LIGHTGUN_SCREEN_Y] = -6789;
	s_frontend_input[1][RETRO_DEVICE_ID_LIGHTGUN_IS_OFFSCREEN] = 1;
	s_frontend_input[1][RETRO_DEVICE_ID_LIGHTGUN_RELOAD] = 1;
	s_frontend_input[1][RETRO_DEVICE_ID_LIGHTGUN_AUX_B] = 1;
	s_frontend_input[1][RETRO_DEVICE_ID_LIGHTGUN_SELECT] = 1;
	s_frontend_input[1][RETRO_DEVICE_ID_LIGHTGUN_DPAD_RIGHT] = 1;

	UpdateInput(0, FakeInput);
	UpdateInput(1, FakeInput);

	InputSnapshot first;
	InputSnapshot second;
	CHECK(ReadInput(0, &first));
	CHECK(ReadInput(1, &second));
	CHECK(first.screen_x == -1234 && first.screen_y == 5678);
	CHECK(!first.offscreen && !first.reload && !first.recalibrate);
	CHECK(first.buttons == (BUTTON_TRIGGER | BUTTON_A | BUTTON_C | BUTTON_DPAD_LEFT | BUTTON_START));
	CHECK(second.screen_x == 2345 && second.screen_y == -6789);
	CHECK(second.offscreen && second.reload && !second.recalibrate);
	CHECK(second.buttons == (BUTTON_B | BUTTON_DPAD_RIGHT | BUTTON_SELECT));

	/* Start + Select is the calibration shot, and the game sees neither. */
	s_frontend_input[0][RETRO_DEVICE_ID_LIGHTGUN_SELECT] = 1;
	UpdateInput(0, FakeInput);
	CHECK(ReadInput(0, &first));
	CHECK(first.recalibrate);
	CHECK(first.buttons == (BUTTON_TRIGGER | BUTTON_A | BUTTON_C | BUTTON_DPAD_LEFT));

	ResetInput(0);
	CHECK(ReadInput(0, &first));
	CHECK(first.screen_x == 0 && first.screen_y == 0 && first.buttons == 0);
	CHECK(first.offscreen && !first.reload && !first.recalibrate);
	CHECK(ReadInput(1, &second));
	CHECK(second.screen_x == 2345 && second.screen_y == -6789);

	CHECK(!ReadInput(2, &first));
	CHECK(!ReadInput(0, nullptr));
	UpdateInput(2, FakeInput);
	UpdateInput(0, nullptr);
}

static thread_local s16 s_stress_value;

static s16 RETRO_CALLCONV StressInput(unsigned port, unsigned device, unsigned index, unsigned id)
{
	if (port != 0 || device != RETRO_DEVICE_LIGHTGUN || index != 0)
		return 0;

	switch (id)
	{
		case RETRO_DEVICE_ID_LIGHTGUN_SCREEN_X:
			return s_stress_value;
		case RETRO_DEVICE_ID_LIGHTGUN_SCREEN_Y:
			return static_cast<s16>(-s_stress_value);
		case RETRO_DEVICE_ID_LIGHTGUN_TRIGGER:
			return s_stress_value & 1;
		case RETRO_DEVICE_ID_LIGHTGUN_AUX_A:
			return (s_stress_value & 2) != 0;
		default:
			return 0;
	}
}

static void TestInputBridgeConcurrency()
{
	ResetInput(0);
	std::atomic<bool> done{false};
	std::atomic<int> errors{0};

	std::thread producer([&done]() {
		for (int iteration = 1; iteration <= 200000; iteration++)
		{
			s_stress_value = static_cast<s16>((iteration % 30000) + 1);
			UpdateInput(0, StressInput);
		}
		done.store(true, std::memory_order_release);
	});

	std::thread consumer([&done, &errors]() {
		do
		{
			InputSnapshot input;
			if (!ReadInput(0, &input))
				continue;

			if (input.screen_x == 0 && input.screen_y == 0 && input.buttons == 0 && input.offscreen)
				continue;

			const u16 expected_buttons =
				((input.screen_x & 1) ? BUTTON_TRIGGER : 0) |
				((input.screen_x & 2) ? BUTTON_A : 0);
			if (input.screen_y != -input.screen_x || input.buttons != expected_buttons || input.offscreen || input.reload || input.recalibrate)
				errors.fetch_add(1, std::memory_order_relaxed);
		} while (!done.load(std::memory_order_acquire));
	});

	producer.join();
	consumer.join();
	CHECK(errors.load(std::memory_order_relaxed) == 0);
}

static void TestCalibrationTable()
{
	protocol::Calibration calibration;
	CHECK(protocol::LookupCalibration("SLUS-20219", &calibration));
	CHECK(calibration.screen_width == 640);
	CHECK(calibration.screen_height == 240);
	CHECK(Near(calibration.center_x, 390.0f));
	CHECK(Near(calibration.center_y, 154.0f));
	CHECK(Near(calibration.scale_x, 0.9025f));
	CHECK(Near(calibration.scale_y, 0.975f));

	CHECK(protocol::LookupCalibration("SLES-50936", &calibration));
	CHECK(calibration.screen_width == 512);
	CHECK(calibration.screen_height == 256);
	CHECK(Near(calibration.scale_x, 1.12f));
	CHECK(Near(calibration.scale_y, 1.0f));

	calibration.center_x = 123.0f;
	CHECK(!protocol::LookupCalibration("UNKNOWN", &calibration));
	CHECK(Near(calibration.center_x, 123.0f));
	CHECK(!protocol::LookupCalibration("", &calibration));
	CHECK(!protocol::LookupCalibration(nullptr, &calibration));
	CHECK(!protocol::LookupCalibration("SLUS-20219", nullptr));
}

int main()
{
	TestDescriptors();
	TestCoordinates();
	TestReports();
	TestRecalibrationSequence();
	TestParameterDecoding();
	TestInputBridge();
	TestInputBridgeConcurrency();
	TestCalibrationTable();

	if (failures)
	{
		std::fprintf(stderr, "GunCon2 protocol: %d failure(s)\n", failures);
		return EXIT_FAILURE;
	}

	std::puts("GunCon2 protocol: all tests passed");
	return EXIT_SUCCESS;
}
