#include "test_framework.h"
#include "eye_tracking.h"
#include "app_state.h"
#include <chrono>
#include <cmath>
#include <thread>

extern int test_pvr_tracking_mode;
extern int test_pvr_last_set_mode;
extern bool test_pvr_set_mode_rc;
extern int test_pvr_set_mode_calls;

struct test_eye_data
{
	int ls, rs, cs;
	float cv[3];
	float lo, ro;
};
extern test_eye_data test_pvr_eye;
extern bool test_pvr_eye_ok;

static const int MODE_POSITION = 0x2, MODE_EYE = 0x4;

static bool wait_mode(int want, int ms = 3000)
{
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
	while (std::chrono::steady_clock::now() < deadline)
	{
		if (test_pvr_last_set_mode == want)
			return true;
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}
	return false;
}

TEST(eye_tracking, init_detects_eye_support_and_starts_position_only)
{
	test_pvr_tracking_mode = MODE_POSITION | MODE_EYE;
	initEyeTrackingMode();
	CHECK(gEyeSupported.load());
	CHECK_EQ(test_pvr_last_set_mode, MODE_POSITION);
	CHECK(!gEyeOnline.load());
	CHECK(!gGazeValid.load());
}

TEST(eye_tracking, init_without_eye_bit)
{
	test_pvr_tracking_mode = MODE_POSITION;
	initEyeTrackingMode();
	CHECK(!gEyeSupported.load());
	CHECK_EQ(test_pvr_last_set_mode, MODE_POSITION);
	test_pvr_tracking_mode = MODE_POSITION | MODE_EYE;
	initEyeTrackingMode();
}

TEST(eye_tracking, streaming_request_enables_ir)
{
	gEyeDebugOn.store(false);
	test_pvr_last_set_mode = -1;
	applyServerEyeTracking(true);
	CHECK(wait_mode(MODE_POSITION | MODE_EYE));
	CHECK(gServerEyeEnabled);
}

TEST(eye_tracking, stop_streaming_disables_ir_and_gaze)
{
	gEyeDebugOn.store(false);
	test_pvr_last_set_mode = -1;
	applyServerEyeTracking(false);
	CHECK(wait_mode(MODE_POSITION));
	CHECK(!gServerEyeEnabled);
	CHECK(!gEyeOnline.load());
	CHECK(!gGazeValid.load());
}

TEST(eye_tracking, unsupported_device_keeps_ir_off)
{
	test_pvr_tracking_mode = MODE_POSITION;
	initEyeTrackingMode();
	CHECK(!gEyeSupported.load());
	test_pvr_last_set_mode = -1;
	applyServerEyeTracking(true);
	CHECK(wait_mode(MODE_POSITION));
	CHECK(!gServerEyeEnabled);
	test_pvr_tracking_mode = MODE_POSITION | MODE_EYE;
	initEyeTrackingMode();
	CHECK(gEyeSupported.load());
}

TEST(eye_tracking, debug_toggle_lights_ir_without_stream)
{
	gEyeDebugOn.store(true);
	test_pvr_last_set_mode = -1;
	applyServerEyeTracking(false);
	CHECK(wait_mode(MODE_POSITION | MODE_EYE));
	gEyeDebugOn.store(false);
	test_pvr_last_set_mode = -1;
	applyServerEyeTracking(false);
	CHECK(wait_mode(MODE_POSITION));
}

TEST(eye_tracking, read_gazes_ir_off_early_return)
{
	test_pvr_last_set_mode = -1;
	applyServerEyeTracking(false);
	CHECK(wait_mode(MODE_POSITION));
	XrPosef out[2];
	bool vL = true, vR = true;
	CHECK(!readEyeGazes(out, &vL, &vR, 0, {0, 0, 0, 1}));
	CHECK(!vL);
	CHECK(!vR);
}

TEST(eye_tracking, read_gazes_sdk_failure)
{
	test_pvr_last_set_mode = -1;
	applyServerEyeTracking(true);
	CHECK(wait_mode(MODE_POSITION | MODE_EYE));
	test_pvr_eye_ok = false;
	XrPosef out[2];
	bool vL, vR;
	CHECK(!readEyeGazes(out, &vL, &vR, 1, {0, 0, 0, 1}));
}

TEST(eye_tracking, read_gazes_invalid_combined_vector_still_captures_openness)
{
	test_pvr_eye_ok = true;
	test_pvr_eye.ls = 1;
	test_pvr_eye.rs = 1;
	test_pvr_eye.cs = 1;
	test_pvr_eye.cv[0] = 0;
	test_pvr_eye.cv[1] = 0;
	test_pvr_eye.cv[2] = 0;   // zero magnitude: gaze invalid, openness still real
	test_pvr_eye.lo = 0.25f;
	test_pvr_eye.ro = 0.75f;
	gEyeHaveOpen = false;
	XrPosef out[2];
	bool vL, vR;
	CHECK(!readEyeGazes(out, &vL, &vR, 1, {0, 0, 0, 1}));
	CHECK(!gGazeValid.load());
	CHECK(gEyeHaveOpen);
	CHECK(std::abs(gEyeOpen[0] - 0.25f) < 1e-6f);
	CHECK(std::abs(gEyeOpen[1] - 0.75f) < 1e-6f);
}

TEST(eye_tracking, read_gazes_straight_ahead)
{
	test_pvr_eye_ok = true;
	test_pvr_eye.ls = 1;
	test_pvr_eye.rs = 1;
	test_pvr_eye.cs = 1;
	test_pvr_eye.cv[0] = 0;
	test_pvr_eye.cv[1] = 0;
	test_pvr_eye.cv[2] = -1;
	gEyeOnline.store(false);
	XrPosef out[2];
	bool vL, vR;
	CHECK(readEyeGazes(out, &vL, &vR, 0, {0, 0, 0, 1}));
	CHECK(vL);
	CHECK(vR);
	CHECK(gEyeOnline.load());
	CHECK(gGazeValid.load());
	// Straight down -Z gives the identity gaze orientation.
	CHECK(std::abs(out[0].orientation.x) < 1e-5f);
	CHECK(std::abs(out[0].orientation.y) < 1e-5f);
	CHECK(std::abs(out[0].orientation.z) < 1e-5f);
	CHECK(std::abs(out[0].orientation.w - 1.0f) < 1e-5f);
	CHECK(std::abs(out[1].orientation.w - 1.0f) < 1e-5f);
	CHECK(std::abs(gGazeLocal[2].load() + 1.0f) < 1e-6f);
}

TEST(eye_tracking, read_gazes_pitch_and_yaw)
{
	test_pvr_eye_ok = true;
	test_pvr_eye.cs = 1;
	test_pvr_eye.cv[0] = 0.0f;
	test_pvr_eye.cv[1] = 0.5f;
	test_pvr_eye.cv[2] = -std::sqrt(0.75f);
	XrPosef out[2];
	bool vL, vR;
	// pitch = atan2(0.5, sqrt(.75)) = 30deg -> quat x = sin(15deg)
	CHECK(readEyeGazes(out, &vL, &vR, 1, {0, 0, 0, 1}));
	CHECK(std::abs(out[0].orientation.x - std::sin(M_PI / 12)) < 1e-4f);
	CHECK(std::abs(out[0].orientation.y) < 1e-5f);
	// Compose with a 90deg yaw head: gaze should rotate with the head.
	Quat headQ = {0.0f, std::sin(M_PI / 4), 0.0f, std::cos(M_PI / 4)};
	CHECK(readEyeGazes(out, &vL, &vR, 1, headQ));
	CHECK(std::abs(out[0].orientation.y) > 0.3f);
}

TEST(eye_tracking, read_gazes_unnormalized_vector_gets_normalized)
{
	test_pvr_eye_ok = true;
	test_pvr_eye.cs = 1;
	test_pvr_eye.cv[0] = 0;
	test_pvr_eye.cv[1] = 0;
	test_pvr_eye.cv[2] = -1.9f;   // magnitude < 2.0 passes the validity gate
	XrPosef out[2];
	bool vL, vR;
	CHECK(readEyeGazes(out, &vL, &vR, 1, {0, 0, 0, 1}));
	float n = std::sqrt(out[0].orientation.x * out[0].orientation.x +
	                    out[0].orientation.y * out[0].orientation.y +
	                    out[0].orientation.z * out[0].orientation.z +
	                    out[0].orientation.w * out[0].orientation.w);
	CHECK(std::abs(n - 1.0f) < 1e-5f);
}

TEST(eye_tracking, read_gazes_oversized_vector_rejected)
{
	test_pvr_eye_ok = true;
	test_pvr_eye.cs = 1;
	test_pvr_eye.cv[0] = 0;
	test_pvr_eye.cv[1] = 0;
	test_pvr_eye.cv[2] = -2.5f;   // magnitude > 2.0 fails the gate
	XrPosef out[2];
	bool vL, vR;
	CHECK(!readEyeGazes(out, &vL, &vR, 1, {0, 0, 0, 1}));
}

TEST(eye_tracking, read_gazes_no_status_rejected)
{
	test_pvr_eye_ok = true;
	test_pvr_eye.ls = 0;
	test_pvr_eye.rs = 0;
	test_pvr_eye.cs = 0;
	test_pvr_eye.cv[2] = -1.0f;
	gEyeHaveOpen = false;
	XrPosef out[2];
	bool vL, vR;
	CHECK(!readEyeGazes(out, &vL, &vR, 1, {0, 0, 0, 1}));
	CHECK(!gEyeHaveOpen);
}

TEST(eye_tracking, poll_gaze_offline_clears_validity)
{
	gEyeOnline.store(false);
	gGazeValid.store(true);
	gEyeOpennessValid.store(true);
	pollEyeGaze();
	CHECK(!gGazeValid.load());
	CHECK(!gEyeOpennessValid.load());
}

TEST(eye_tracking, poll_gaze_online_mirrors_state)
{
	gEyeOnline.store(true);
	gGazeLocal[0].store(0.0f);
	gGazeLocal[1].store(0.0f);
	gGazeLocal[2].store(-1.0f);
	gEyeHaveOpen = true;
	gEyeOpen[0] = 0.4f;
	gEyeOpen[1] = 0.6f;
	pollEyeGaze();
	CHECK(gGazeValid.load());
	CHECK(std::abs(gGazePitch.load()) < 1e-6f);
	CHECK(std::abs(gGazeYaw.load()) < 1e-6f);
	CHECK(std::abs(gGazeQuat[3].load() - 1.0f) < 1e-6f);
	CHECK(gEyeOpennessValid.load());
	CHECK(std::abs(gEyeOpenness[0].load() - 0.4f) < 1e-6f);
	CHECK(std::abs(gEyeOpenness[1].load() - 0.6f) < 1e-6f);
}

TEST(eye_tracking, poll_gaze_up_vector)
{
	gEyeOnline.store(true);
	gGazeLocal[0].store(0.0f);
	gGazeLocal[1].store(1.0f);
	gGazeLocal[2].store(0.0f);
	gEyeHaveOpen = false;
	gEyeOpennessValid.store(false);
	pollEyeGaze();
	CHECK(std::abs(gGazePitch.load() - M_PI_2) < 1e-5f);
	CHECK(!gEyeOpennessValid.load());
}
