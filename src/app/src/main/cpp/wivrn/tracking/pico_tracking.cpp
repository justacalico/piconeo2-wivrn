#include "pico_tracking.h"
#include "wivrn_client_pico.h"
#include "pico_sdk.h"
#include "eye_tracking.h"
#include "latency_tracker.h"
#include "input.h"
#include "app_state.h"          // gSoftIpdMm / softIpdM (unified IPD source)

#include <spdlog/spdlog.h>
#include <android/log.h>
#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO, "wivrn-native", __VA_ARGS__)

#include <cmath>
#include <cstring>

using namespace wivrn;

namespace
{

constexpr long k_period_ns = 1000000000L / 300;
constexpr int k_uplink_div = 1;

constexpr float k_yaw_offset_deg = 35.0f;
constexpr float k_rot_offset_x_deg = 10.0f;
constexpr float k_rot_offset_y_deg = -34.0f;
constexpr float k_grip_side_mm = 0.0f;
constexpr float k_grip_up_mm = 12.5f;
constexpr float k_grip_back_mm = 40.0f;
constexpr float k_rot_swing = 1.0f;

constexpr float k_predict = 0.8f;

// Head has no hardware velocity on this SDK; it's differentiated position.
// Linear and angular reach must match: sending full-strength rotation
// extrapolation with dialed-back position extrapolation (the old 0.4/1.0
// split) desyncs the eyes' predicted swing around the neck from the
// predicted orientation, so nearby geometry swims during head turns even
// though the frame rate itself is smooth. That swim is a much stronger
// sickness trigger than the reach itself, so both axes use the same factor.
// Now that the velocity filter (step_head_filter) tracks real motion within
// ~80ms instead of ~660ms, this can run closer to full strength without the
// old overshoot-after-stop behaviour.
constexpr float k_head_predict = 0.7f;

neo2::quat apply_controller_orientation(const float raw_orient[4], int hand)
{
	neo2::quat oq = {-raw_orient[0], -raw_orient[1], raw_orient[2], raw_orient[3]};

	float yaw_deg = (hand == 0 ? +k_yaw_offset_deg : -k_yaw_offset_deg);
	float yaw_rad = yaw_deg * 0.01745329f;
	neo2::quat q_yaw = {0.0f, std::sin(yaw_rad * 0.5f), 0.0f, std::cos(yaw_rad * 0.5f)};
	oq = neo2::normalize_quat(neo2::multiply_quat(oq, q_yaw));

	float rx = k_rot_offset_x_deg * 0.01745329f;
	float ry = (hand == 0 ? k_rot_offset_y_deg : -k_rot_offset_y_deg) * 0.01745329f;
	float rz = 0.0f;

	neo2::quat qrx = {std::sin(rx * 0.5f), 0.0f, 0.0f, std::cos(rx * 0.5f)};
	neo2::quat qry = {0.0f, std::sin(ry * 0.5f), 0.0f, std::cos(ry * 0.5f)};
	neo2::quat qrz = {0.0f, 0.0f, std::sin(rz * 0.5f), std::cos(rz * 0.5f)};
	neo2::quat q_off = neo2::multiply_quat(neo2::multiply_quat(qrx, qry), qrz);
	oq = neo2::normalize_quat(neo2::multiply_quat(oq, q_off));

	return oq;
}

void compute_grip_offset(const neo2::quat & orient, int hand, float out_world[3])
{
	float side = k_grip_side_mm * 0.001f;
	float up = k_grip_up_mm * 0.001f;
	float back = k_grip_back_mm * 0.001f;
	float side_sign = (hand == 0) ? +1.0f : -1.0f;

	float local[3] = {side * side_sign, up, back};

	float swing = k_rot_swing;
	if (swing < 0.0f) swing = 0.0f; else if (swing > 1.0f) swing = 1.0f;

	float yn = std::sqrt(orient.y * orient.y + orient.w * orient.w);
	neo2::quat orient_yaw = (yn > 1e-6f)
		? neo2::quat{0.0f, orient.y / yn, 0.0f, orient.w / yn}
		: neo2::quat{0.0f, 0.0f, 0.0f, 1.0f};

	float orbit[3] = {local[0] * swing, local[1] * swing, local[2] * swing};
	float level[3] = {local[0] * (1.0f - swing), local[1] * (1.0f - swing), local[2] * (1.0f - swing)};

	float w_orbit[3], w_level[3];
	neo2::rotate_vector(orient, orbit, w_orbit);
	neo2::rotate_vector(orient_yaw, level, w_level);

	out_world[0] = w_orbit[0] + w_level[0];
	out_world[1] = w_orbit[1] + w_level[1];
	out_world[2] = w_orbit[2] + w_level[2];
}

} // namespace

void pico_native_tracker::start()
{
	if (running.load())
		return;
	running = true;
	// Preserve height calibration across reconnects; only explicit recenter
	// should recalibrate.
	thread = std::thread([this] { run(); });
	spdlog::info("Native tracker started (height_calibrated={})", height_calibrated);
}

void pico_native_tracker::stop()
{
	running = false;
	if (thread.joinable())
		thread.join();
	spdlog::info("Native tracker stopped");
}

void pico_native_tracker::set_head_pose(const float orient[4], const float pos[3],
                                       const float * lin_vel, const float * ang_vel)
{
	uint64_t ts = (uint64_t)now_ns();
	neo2::quat hq = neo2::normalize_quat({orient[0], orient[1], orient[2], orient[3]});

	std::lock_guard lock(state_mutex);
	std::memcpy(head_orient, orient, sizeof(head_orient));
	std::memcpy(head_pos, pos, sizeof(head_pos));
	head_valid = true;
	head_pose_seq++;

	if (!height_calibrated && !floor_relative.load())
	{
		constexpr float k_standing_eye_height = 1.5f;
		float offset = k_standing_eye_height - pos[1];
		height_offset.store(offset);
		height_calibrated = true;
		spdlog::info("Height calibrated: initial_Y={:.3f} offset={:.3f}", pos[1], offset);
	}

	if (lin_vel && ang_vel)
	{
		head_lin_vel[0] = lin_vel[0];
		head_lin_vel[1] = lin_vel[1];
		head_lin_vel[2] = lin_vel[2];
		head_ang_vel[0] = ang_vel[0];
		head_ang_vel[1] = ang_vel[1];
		head_ang_vel[2] = ang_vel[2];
		hw_velocity_valid = true;
		head_prev_pos[0] = pos[0];
		head_prev_pos[1] = pos[1];
		head_prev_pos[2] = pos[2];
		head_prev_orient = hq;
		head_prev_ts = ts;
		head_filter_init = true;
		// Reset One-Euro state so it re-initializes cleanly if we later
		// fall back to finite-differenced velocity.
		for (int i = 0; i < 3; i++)
		{
			head_lin_filter[i].reset();
			head_ang_filter[i].reset();
		}
		// Deadband hw velocity too: even IMU-fused velocity has small noise
		// at rest that the server will extrapolate into jitter.
		constexpr float lin_db = 0.01f;
		constexpr float ang_db = 0.01f;
		if (std::abs(head_lin_vel[0]) < lin_db) head_lin_vel[0] = 0;
		if (std::abs(head_lin_vel[1]) < lin_db) head_lin_vel[1] = 0;
		if (std::abs(head_lin_vel[2]) < lin_db) head_lin_vel[2] = 0;
		if (std::abs(head_ang_vel[0]) < ang_db) head_ang_vel[0] = 0;
		if (std::abs(head_ang_vel[1]) < ang_db) head_ang_vel[1] = 0;
		if (std::abs(head_ang_vel[2]) < ang_db) head_ang_vel[2] = 0;
	}
	else
	{
		hw_velocity_valid = false;
		step_head_filter(pos, hq, ts);
	}
}

void pico_native_tracker::recenter_height()
{
	std::lock_guard lock(state_mutex);
	float actual_height = head_pos[1] + height_offset.load();
	height_offset.store(actual_height);
	height_calibrated = true;
	spdlog::info("Recenter height: actual_height={:.3f} new_offset={:.3f}", actual_height, actual_height);
}

void pico_native_tracker::set_prediction_ns(int64_t ns)
{
	base_prediction_ns.store(ns);
	prediction_ns.store(ns + latency_correction_ns.load());
}

void pico_native_tracker::set_latency_correction_ns(int64_t ns)
{
	latency_correction_ns.store(ns);
	prediction_ns.store(base_prediction_ns.load() + ns);
}

void pico_native_tracker::get_head_pose(float out_orient[4], float out_pos[3])
{
	std::lock_guard lock(state_mutex);
	std::memcpy(out_orient, head_orient, sizeof(head_orient));
	std::memcpy(out_pos, head_pos, sizeof(head_pos));
}

void pico_native_tracker::get_controllers(controller_sample out[2])
{
	std::lock_guard lock(state_mutex);
	out[0] = controllers[0];
	out[1] = controllers[1];
}

void pico_native_tracker::update_controller(int hand, const float orient[4], const float pos[3],
                                            int trigger, const int touch[2], int battery,
                                            bool a, bool b, bool grip, bool click, bool menu,
                                            const float * ang_vel)
{
	if (hand < 0 || hand > 1)
		return;
	std::lock_guard lock(state_mutex);
	auto & c = controllers[hand];
	std::memcpy(c.orientation, orient, sizeof(c.orientation));
	std::memcpy(c.position, pos, sizeof(c.position));
	c.trigger = trigger;
	c.battery = battery;
	c.button_a = a;
	c.button_b = b;
	c.grip = grip;
	c.thumbstick_click = click;
	c.menu = menu;
	c.connected = true;
	if (touch)
	{
		c.touch[0] = touch[0];
		c.touch[1] = touch[1];
	}
	if (ang_vel)
	{
		c.angular_velocity[0] = ang_vel[0];
		c.angular_velocity[1] = ang_vel[1];
		c.angular_velocity[2] = ang_vel[2];
		c.has_angular_velocity = true;
	}
	else
	{
		c.has_angular_velocity = false;
	}
}

void pico_native_tracker::clear_controller(int hand)
{
	if (hand < 0 || hand > 1)
		return;
	std::lock_guard lock(state_mutex);
	controllers[hand].connected = false;
}

void pico_native_tracker::update_controller_from_jni(int hand, int conn, const float * sensor,
                                                     const float * ang_vel, const int * keys)
{
	if (hand < 0 || hand > 1)
		return;

	std::lock_guard lock(state_mutex);
	auto & c = controllers[hand];

	if (conn != 1)
	{
		c.connected = false;
		return;
	}

	c.connected = true;

	if (sensor)
	{
		c.orientation[0] = sensor[0];
		c.orientation[1] = sensor[1];
		c.orientation[2] = sensor[2];
		c.orientation[3] = sensor[3];
		c.position[0] = sensor[4];
		c.position[1] = sensor[5];
		c.position[2] = sensor[6];
	}

	if (ang_vel)
	{
		c.angular_velocity[0] = ang_vel[0];
		c.angular_velocity[1] = ang_vel[1];
		c.angular_velocity[2] = ang_vel[2];
		c.has_angular_velocity = true;
	}
	else
	{
		c.has_angular_velocity = false;
	}

	if (keys)
	{
		int analog_trig = keys[8];
		int digital_trig = keys[2] ? 255 : 0;
		c.trigger = std::max(analog_trig, digital_trig);
		c.touch[0] = keys[0];
		c.touch[1] = keys[1];
		c.battery = keys[10];
		c.button_a = keys[6] != 0;
		c.button_b = keys[7] != 0;
		c.grip = keys[3] != 0;
		c.thumbstick_click = keys[4] != 0;
		c.menu = keys[5] != 0;
		c.home = keys[11] != 0;
	}
}

void pico_native_tracker::step_head_filter(const float pos[3], const neo2::quat & orient, uint64_t ts)
{
	// Two 300Hz loops (this one and the render thread's) beat against each
	// other, so we frequently see the same sample twice. Differencing a
	// duplicate yields spurious zero velocity; skip it so dt accumulates
	// across duplicates and only genuinely new samples feed the filter.
	if (head_filter_init && pos[0] == head_prev_pos[0] && pos[1] == head_prev_pos[1] &&
	    pos[2] == head_prev_pos[2] && orient.x == head_prev_orient.x &&
	    orient.y == head_prev_orient.y && orient.z == head_prev_orient.z &&
	    orient.w == head_prev_orient.w)
		return;

	if (head_filter_init)
	{
		double dt = (double)(ts - head_prev_ts) * 1e-9;
		if (dt > 0.002 && dt < 0.05)
		{
			float lvx = (float)((pos[0] - head_prev_pos[0]) / dt);
			float lvy = (float)((pos[1] - head_prev_pos[1]) / dt);
			float lvz = (float)((pos[2] - head_prev_pos[2]) / dt);

			neo2::quat delta = neo2::normalize_quat(
				neo2::multiply_quat(orient, neo2::conjugate_quat(head_prev_orient)));
			if (delta.w < 0)
			{
				delta.x = -delta.x; delta.y = -delta.y;
				delta.z = -delta.z; delta.w = -delta.w;
			}
			float w = delta.w > 1.0f ? 1.0f : delta.w;
			float angle = 2.0f * std::acos(w);
			float s = std::sqrt(1.0f - w * w);
			float avx = 0, avy = 0, avz = 0;
			if (s > 1e-6f)
			{
				float k = (angle / s) / (float)dt;
				avx = delta.x * k; avy = delta.y * k; avz = delta.z * k;
			}

			// One-Euro adaptive filter replaces the old fixed-tau EMA. The EMA
			// with tau=0.08s was a compromise: smooth enough at rest to hide
			// some jitter, but it lagged real motion by ~80ms and still let
			// per-sample noise through during slow drift. One-Euro scales its
			// cutoff with speed, so it's far smoother when the head is still
			// (kills the "shaky when not moving" complaint) and equally
			// responsive when turning.
			float fdt = (float)dt;
			head_lin_vel[0] = head_lin_filter[0].filter(lvx, fdt);
			head_lin_vel[1] = head_lin_filter[1].filter(lvy, fdt);
			head_lin_vel[2] = head_lin_filter[2].filter(lvz, fdt);
			head_ang_vel[0] = head_ang_filter[0].filter(avx, fdt);
			head_ang_vel[1] = head_ang_filter[1].filter(avy, fdt);
			head_ang_vel[2] = head_ang_filter[2].filter(avz, fdt);

			// Velocity deadband: when the filtered velocity is small enough that
			// extrapolating it over the prediction horizon (typically 20-40ms)
			// would move the pose by less than ~0.2mm / 0.05deg, zero it out.
			// This is the single most effective fix for "shaky when completely
			// still": without it, residual noise of e.g. 0.005 m/s extrapolated
			// 30ms forward produces 0.15mm of wander every frame.
			constexpr float lin_deadband = 0.01f;  // m/s -> 0.3mm at 30ms
			constexpr float ang_deadband = 0.01f;  // rad/s -> 0.03deg at 30ms
			if (std::abs(head_lin_vel[0]) < lin_deadband) head_lin_vel[0] = 0;
			if (std::abs(head_lin_vel[1]) < lin_deadband) head_lin_vel[1] = 0;
			if (std::abs(head_lin_vel[2]) < lin_deadband) head_lin_vel[2] = 0;
			if (std::abs(head_ang_vel[0]) < ang_deadband) head_ang_vel[0] = 0;
			if (std::abs(head_ang_vel[1]) < ang_deadband) head_ang_vel[1] = 0;
			if (std::abs(head_ang_vel[2]) < ang_deadband) head_ang_vel[2] = 0;
		}
	}
	head_prev_pos[0] = pos[0];
	head_prev_pos[1] = pos[1];
	head_prev_pos[2] = pos[2];
	head_prev_orient = orient;
	head_prev_ts = ts;
	head_filter_init = true;
}

void pico_native_tracker::step_ctrl_filter(int hand, const float pos_m[3], const neo2::quat & orient, uint64_t ts, const float * hw_ang_vel)
{
	if (ctrl_filter_init[hand])
	{
		double dt = (double)(ts - ctrl_prev_ts[hand]) * 1e-9;
		if (dt > 0.002 && dt < 0.05)
		{
			float lvx = (float)((pos_m[0] - ctrl_prev_pos[hand][0]) / dt);
			float lvy = (float)((pos_m[1] - ctrl_prev_pos[hand][1]) / dt);
			float lvz = (float)((pos_m[2] - ctrl_prev_pos[hand][2]) / dt);

			float avx, avy, avz;
			if (hw_ang_vel)
			{
				avx = -hw_ang_vel[0];
				avy = -hw_ang_vel[1];
				avz = hw_ang_vel[2];
			}
			else
			{
				neo2::quat delta = neo2::normalize_quat(
					neo2::multiply_quat(orient, neo2::conjugate_quat(ctrl_prev_orient[hand])));
				if (delta.w < 0)
				{
					delta.x = -delta.x; delta.y = -delta.y;
					delta.z = -delta.z; delta.w = -delta.w;
				}
				float w = delta.w > 1.0f ? 1.0f : delta.w;
				float angle = 2.0f * std::acos(w);
				float s = std::sqrt(1.0f - w * w);
				if (s > 1e-6f)
				{
					float k = (angle / s) / (float)dt;
					avx = delta.x * k; avy = delta.y * k; avz = delta.z * k;
				}
				else
				{
					avx = avy = avz = 0;
				}
			}

			// One-Euro adaptive filter, same rationale as step_head_filter.
			// Controllers have more positional noise than the HMD (6DOF optical
			// tracking vs SLAM-fused IMU), so min_cutoff is lower (1.0 Hz vs
			// 1.2 Hz) for stronger rest smoothing.
			float fdt = (float)dt;
			ctrl_lin_vel[hand][0] = ctrl_lin_filter[hand][0].filter(lvx, fdt);
			ctrl_lin_vel[hand][1] = ctrl_lin_filter[hand][1].filter(lvy, fdt);
			ctrl_lin_vel[hand][2] = ctrl_lin_filter[hand][2].filter(lvz, fdt);
			ctrl_ang_vel[hand][0] = ctrl_ang_filter[hand][0].filter(avx, fdt);
			ctrl_ang_vel[hand][1] = ctrl_ang_filter[hand][1].filter(avy, fdt);
			ctrl_ang_vel[hand][2] = ctrl_ang_filter[hand][2].filter(avz, fdt);

			// Velocity deadband. Controllers are the worse case for "shaky when
			// still" because optical 6DOF tracking has more low-frequency drift
			// than IMU-fused head tracking. The deadband zeroes residual noise
			// so the server's prediction holds the pose rock-steady at rest.
			constexpr float lin_deadband = 0.008f;  // m/s -> 0.24mm at 30ms
			constexpr float ang_deadband = 0.015f;  // rad/s -> 0.045deg at 30ms
			if (std::abs(ctrl_lin_vel[hand][0]) < lin_deadband) ctrl_lin_vel[hand][0] = 0;
			if (std::abs(ctrl_lin_vel[hand][1]) < lin_deadband) ctrl_lin_vel[hand][1] = 0;
			if (std::abs(ctrl_lin_vel[hand][2]) < lin_deadband) ctrl_lin_vel[hand][2] = 0;
			if (std::abs(ctrl_ang_vel[hand][0]) < ang_deadband) ctrl_ang_vel[hand][0] = 0;
			if (std::abs(ctrl_ang_vel[hand][1]) < ang_deadband) ctrl_ang_vel[hand][1] = 0;
			if (std::abs(ctrl_ang_vel[hand][2]) < ang_deadband) ctrl_ang_vel[hand][2] = 0;
		}
	}
	ctrl_prev_pos[hand][0] = pos_m[0];
	ctrl_prev_pos[hand][1] = pos_m[1];
	ctrl_prev_pos[hand][2] = pos_m[2];
	ctrl_prev_orient[hand] = orient;
	ctrl_prev_ts[hand] = ts;
	ctrl_filter_init[hand] = true;
}

void pico_native_tracker::run()
{
	ALOGI("pico_native_tracker::run() entered, session=%p", (void*)session);
	int frame_counter = 0;
	uint32_t last_tx_seq = 0;

	auto ts_add_ns = [](struct timespec & t, long ns) {
		t.tv_nsec += ns;
		while (t.tv_nsec >= 1000000000L)
		{
			t.tv_nsec -= 1000000000L;
			t.tv_sec += 1;
		}
	};

	struct timespec next_tick;
	clock_gettime(CLOCK_MONOTONIC, &next_tick);

	while (running.load())
	{
		uint64_t ts = (uint64_t)now_ns();

		float h_orient[4], h_pos[3];
		bool h_valid;
		uint32_t cur_seq;

		{
			std::lock_guard lock(state_mutex);
			h_valid = head_valid;
			cur_seq = head_pose_seq;
			if (h_valid)
			{
				std::memcpy(h_orient, head_orient, sizeof(h_orient));
				std::memcpy(h_pos, head_pos, sizeof(h_pos));
			}
		}

		// Read head pose from gHeadData (published by the render thread's
		// tracking loop, which initializes the PVR SDK).
		{
			float qx, qy, qz, qw, px, py, pz;
			{
				std::lock_guard<std::mutex> lk(gHeadMutex);
				qx = gHeadData[0]; qy = gHeadData[1]; qz = gHeadData[2]; qw = gHeadData[3];
				px = gHeadData[4]; py = gHeadData[5]; pz = gHeadData[6];
			}
			{
				std::lock_guard lock(state_mutex);
				head_orient[0] = qx; head_orient[1] = qy; head_orient[2] = qz; head_orient[3] = qw;
				head_pos[0] = px; head_pos[1] = py; head_pos[2] = pz;
				head_valid = true;
				head_pose_seq++;
				cur_seq = head_pose_seq;
				h_valid = true;
				std::memcpy(h_orient, head_orient, sizeof(h_orient));
				std::memcpy(h_pos, head_pos, sizeof(h_pos));

				hw_velocity_valid = false;
				neo2::quat hq = neo2::normalize_quat({qx, qy, qz, qw});
				step_head_filter(h_pos, hq, ts);
			}
		}

		static int sensor_log_count = 0;
		if (sensor_log_count < 3 || (sensor_log_count % 3000) == 0) {
			ALOGI("pico_tracker[%d]: valid=%d seq=%u q=(%.3f,%.3f,%.3f,%.3f) p=(%.3f,%.3f,%.3f)",
				sensor_log_count, h_valid, cur_seq, h_orient[0], h_orient[1], h_orient[2], h_orient[3], h_pos[0], h_pos[1], h_pos[2]);
		}
		sensor_log_count++;

		if (!h_valid)
		{
			frame_counter++;
			ts_add_ns(next_tick, k_period_ns);
			struct timespec now_t;
			clock_gettime(CLOCK_MONOTONIC, &now_t);
			if (now_t.tv_sec > next_tick.tv_sec ||
			    (now_t.tv_sec == next_tick.tv_sec && now_t.tv_nsec > next_tick.tv_nsec))
			{
				next_tick = now_t;
			}
			else
			{
				clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, nullptr);
			}
			continue;
		}

		controller_sample cs[2];
		{
			std::lock_guard lock(state_mutex);
			cs[0] = controllers[0];
			cs[1] = controllers[1];
		}

		static bool prev_home[2] = {false, false};
		static uint64_t home_press_ts[2] = {0, 0};
		for (int h = 0; h < 2; h++)
		{
			bool home_now = cs[h].connected && cs[h].home;
			if (home_now && !prev_home[h])
			{
				home_press_ts[h] = ts;
			}
			else if (!home_now && prev_home[h])
			{
				if (home_press_ts[h] > 0 && ts - home_press_ts[h] > 800000000ULL)
				{
					spdlog::info("recenter triggered by controller {} home button long press", h);
					// Capture current standing height before sensor reset.
					recenter_height();
					Pvr_ResetSensorAll();
					svrRecenterOrientation();
					recenterHeadTrackerAW();
					recenter_requested.store(true);
					lobby_recenter_requested.store(true);
				}
				home_press_ts[h] = 0;
			}
			prev_home[h] = home_now;
		}

		for (int h = 0; h < 2; h++)
		{
			if (!cs[h].connected)
				continue;

			neo2::quat cq = apply_controller_orientation(cs[h].orientation, h);
			float grip_world[3];
			compute_grip_offset(cq, h, grip_world);

			float pos_m[3] = {
				cs[h].position[0] * 0.001f + grip_world[0],
				cs[h].position[1] * 0.001f + grip_world[1],
				cs[h].position[2] * 0.001f + grip_world[2],
			};
			const float * hw_ang = cs[h].has_angular_velocity ? cs[h].angular_velocity : nullptr;
			step_ctrl_filter(h, pos_m, cq, ts, hw_ang);
		}

		bool do_uplink = (frame_counter % k_uplink_div) == 0;
		bool new_pose = (cur_seq != last_tx_seq);
		if (do_uplink && new_pose && session)
		{
			last_tx_seq = cur_seq;
			if ((frame_counter % 30) == 0)
			{
				int64_t measured = g_latency.get_avg_total_latency_ns();
				if (measured > 0)
				{
					int64_t base = base_prediction_ns.load();
					constexpr int64_t photon_safety_ns = 3'000000LL;
					int64_t target = measured + photon_safety_ns;
					int64_t correction = target - base;
					if (correction < 0) correction = 0;
					if (correction > 50000000LL) correction = 50000000LL;
					int64_t prev = latency_correction_ns.load();
					int64_t smoothed = (prev + correction) / 2;
					latency_correction_ns.store(smoothed);
					prediction_ns.store(base + smoothed);
					spdlog::info("Prediction adjusted: base={}ms measured={}ms target={}ms correction={}ms total={}ms",
						base / 1000000, measured / 1000000,
						target / 1000000, smoothed / 1000000,
						(base + smoothed) / 1000000);
				}
			}

			transmit_tracking((int64_t)ts);
			transmit_inputs((int64_t)ts);
		}

		frame_counter++;

		ts_add_ns(next_tick, k_period_ns);
		struct timespec now_t;
		clock_gettime(CLOCK_MONOTONIC, &now_t);
		if (now_t.tv_sec > next_tick.tv_sec ||
		    (now_t.tv_sec == next_tick.tv_sec && now_t.tv_nsec > next_tick.tv_nsec))
		{
			next_tick = now_t;
		}
		else
		{
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, nullptr);
		}
	}
}

void pico_native_tracker::transmit_tracking(int64_t headset_ns)
{
	if (!session)
		return;

	float h_orient[4], h_pos[3];
	bool h_valid;
	controller_sample cs[2];
	{
		std::lock_guard lock(state_mutex);
		h_valid = head_valid;
		if (!h_valid)
			return;
		std::memcpy(h_orient, head_orient, sizeof(h_orient));
		std::memcpy(h_pos, head_pos, sizeof(h_pos));
		cs[0] = controllers[0];
		cs[1] = controllers[1];
	}

	from_headset::tracking pkt{};

	pkt.production_timestamp = headset_ns;
	// Send the pose at production_timestamp. The official WiVRn client adds
	// prediction_ns because OpenXR already extrapolates; we don't have that,
	// so we must NOT add it (the server would skip extrapolation).
	pkt.timestamp = to_xr_time(headset_ns);

	pkt.view_flags = XR_VIEW_STATE_ORIENTATION_VALID_BIT |
	                 XR_VIEW_STATE_POSITION_VALID_BIT |
	                 XR_VIEW_STATE_ORIENTATION_TRACKED_BIT |
	                 XR_VIEW_STATE_POSITION_TRACKED_BIT;
	pkt.state_flags = 0;
	if (recenter_requested.exchange(false))
		pkt.state_flags = from_headset::tracking::recentered;

	pkt.interaction_profiles[0] = interaction_profile::bytedance_pico_neo3_controller;
	pkt.interaction_profiles[1] = interaction_profile::bytedance_pico_neo3_controller;

	neo2::quat hq = neo2::normalize_quat({h_orient[0], h_orient[1], h_orient[2], h_orient[3]});

	// Head position is sent raw. See note in pico_tracking.h: any position
	// filter either causes eye swim during turns or drift after stopping.
	// Jitter is handled by the velocity One-Euro + deadband and the server's
	// polynomial extrapolator.
	XrPosef head_pose;
	head_pose.orientation = neo2::to_xr_quat(hq);
	float h_offset = floor_relative.load() ? 0.0f : height_offset.load();
	head_pose.position = {
		h_pos[0],
		h_pos[1] + h_offset,
		h_pos[2]};

	// Pico Neo 2 has a square ~101 degree per-eye FOV. The OpenXR runtime
	// may report wider values; use the known-correct value from the Pico SDK.
	constexpr float k_fov_half = 101.0f * 0.5f * 0.01745329252f;
	float ipd = softIpdM();
	for (int eye = 0; eye < 2; eye++)
	{
		float eye_offset = (eye == 0 ? -ipd * 0.5f : ipd * 0.5f);
		pkt.views[eye].pose.orientation = {0, 0, 0, 1};
		pkt.views[eye].pose.position = {eye_offset, 0, 0};
		pkt.views[eye].fov = {-k_fov_half, k_fov_half, k_fov_half, -k_fov_half};
	}

	from_headset::tracking::pose head_tp{};
	head_tp.pose = head_pose;
	head_tp.device = device_id::HEAD;
	head_tp.flags = from_headset::pose_flags::orientation_valid |
	                from_headset::pose_flags::position_valid |
	                from_headset::pose_flags::orientation_tracked |
	                from_headset::pose_flags::position_tracked |
	                from_headset::pose_flags::linear_velocity_valid |
	                from_headset::pose_flags::angular_velocity_valid;
	head_tp.linear_velocity = {
		head_lin_vel[0] * k_head_predict,
		head_lin_vel[1] * k_head_predict,
		head_lin_vel[2] * k_head_predict};
	head_tp.angular_velocity = {
		head_ang_vel[0] * k_head_predict,
		head_ang_vel[1] * k_head_predict,
		head_ang_vel[2] * k_head_predict};
	pkt.device_poses.push_back(head_tp);

	for (int h = 0; h < 2; h++)
	{
		if (!cs[h].connected)
		{
			ctrl_smooth_init[h] = false;
			continue;
		}

		neo2::quat cq = apply_controller_orientation(cs[h].orientation, h);

		float grip_world[3];
		compute_grip_offset(cq, h, grip_world);

		float pos_m[3] = {
			cs[h].position[0] * 0.001f + grip_world[0],
			cs[h].position[1] * 0.001f + grip_world[1] + h_offset,
			cs[h].position[2] * 0.001f + grip_world[2],
		};

		// One-Euro on position and slerp on orientation. The old fixed-tau EMA
		// (tau=0.015s) was too light to kill rest jitter. One-Euro smooths hard
		// at rest (low min_cutoff=2Hz) and opens up the cutoff when the
		// controller is moving, so there's no perceptible lag during swings.
		if (!ctrl_smooth_init[h])
		{
			ctrl_smooth_pos[h][0] = pos_m[0];
			ctrl_smooth_pos[h][1] = pos_m[1];
			ctrl_smooth_pos[h][2] = pos_m[2];
			ctrl_smooth_orient[h] = cq;
			ctrl_pos_filter[h][0].reset();
			ctrl_pos_filter[h][1].reset();
			ctrl_pos_filter[h][2].reset();
			ctrl_smooth_init[h] = true;
		}
		else
		{
			float dt_smooth = 1.0f / 150.0f;
			ctrl_smooth_pos[h][0] = ctrl_pos_filter[h][0].filter(pos_m[0], dt_smooth);
			ctrl_smooth_pos[h][1] = ctrl_pos_filter[h][1].filter(pos_m[1], dt_smooth);
			ctrl_smooth_pos[h][2] = ctrl_pos_filter[h][2].filter(pos_m[2], dt_smooth);

			constexpr float orient_tau = 0.02f;
			float oa = 1.0f - expf(-dt_smooth / orient_tau);
			ctrl_smooth_orient[h] = neo2::slerp_quat(ctrl_smooth_orient[h], cq, oa);
		}

		neo2::quat smooth_cq = neo2::normalize_quat(ctrl_smooth_orient[h]);

		XrPosef ctrl_pose;
		ctrl_pose.orientation = neo2::to_xr_quat(smooth_cq);
		ctrl_pose.position = {ctrl_smooth_pos[h][0], ctrl_smooth_pos[h][1], ctrl_smooth_pos[h][2]};

		bool is_left = (h == 0);

		uint8_t pflags = from_headset::pose_flags::orientation_valid |
		                 from_headset::pose_flags::position_valid |
		                 from_headset::pose_flags::orientation_tracked |
		                 from_headset::pose_flags::position_tracked |
		                 from_headset::pose_flags::linear_velocity_valid |
		                 from_headset::pose_flags::angular_velocity_valid;

		from_headset::tracking::pose grip_p{};
		grip_p.pose = ctrl_pose;
		grip_p.device = is_left ? device_id::LEFT_GRIP : device_id::RIGHT_GRIP;
		grip_p.flags = pflags;
		grip_p.linear_velocity = {
			ctrl_lin_vel[h][0] * k_predict,
			ctrl_lin_vel[h][1] * k_predict,
			ctrl_lin_vel[h][2] * k_predict};
		grip_p.angular_velocity = {
			ctrl_ang_vel[h][0] * k_predict,
			ctrl_ang_vel[h][1] * k_predict,
			ctrl_ang_vel[h][2] * k_predict};

		float lever_vel[3] = {0, 0, 0};
		{
			float av[3] = {
				ctrl_ang_vel[h][0] * k_predict,
				ctrl_ang_vel[h][1] * k_predict,
				ctrl_ang_vel[h][2] * k_predict};
			float lever[3] = {grip_world[0], grip_world[1], grip_world[2]};
			lever_vel[0] = av[1] * lever[2] - av[2] * lever[1];
			lever_vel[1] = av[2] * lever[0] - av[0] * lever[2];
			lever_vel[2] = av[0] * lever[1] - av[1] * lever[0];
		}
		grip_p.linear_velocity.x += lever_vel[0];
		grip_p.linear_velocity.y += lever_vel[1];
		grip_p.linear_velocity.z += lever_vel[2];

		pkt.device_poses.push_back(grip_p);

		from_headset::tracking::pose aim_p{};
		aim_p.pose.orientation = ctrl_pose.orientation;
		aim_p.pose.position = {
			ctrl_smooth_pos[h][0] - grip_world[0],
			ctrl_smooth_pos[h][1] - grip_world[1],
			ctrl_smooth_pos[h][2] - grip_world[2],
		};
		aim_p.device = is_left ? device_id::LEFT_AIM : device_id::RIGHT_AIM;
		aim_p.flags = pflags;
		aim_p.linear_velocity = grip_p.linear_velocity;
		aim_p.angular_velocity = grip_p.angular_velocity;
		pkt.device_poses.push_back(aim_p);
	}

	static int track_count = 0;
	if (++track_count % 120 == 1)
	{
		spdlog::warn("TRACKING: HMD q=({:.3f},{:.3f},{:.3f},{:.3f}) p=({:.3f},{:.3f},{:.3f}) "
		             "lv=({:.2f},{:.2f},{:.2f}) av=({:.2f},{:.2f},{:.2f})",
			head_pose.orientation.x, head_pose.orientation.y,
			head_pose.orientation.z, head_pose.orientation.w,
			head_pose.position.x, head_pose.position.y, head_pose.position.z,
			head_tp.linear_velocity.x, head_tp.linear_velocity.y, head_tp.linear_velocity.z,
			head_tp.angular_velocity.x, head_tp.angular_velocity.y, head_tp.angular_velocity.z);
		for (int h = 0; h < 2; h++)
		{
			if (!cs[h].connected)
			{
				spdlog::warn("TRACKING {} controller: DISCONNECTED", h == 0 ? "LEFT" : "RIGHT");
				continue;
			}

			neo2::quat cq = apply_controller_orientation(cs[h].orientation, h);
			float grip_world[3];
			compute_grip_offset(cq, h, grip_world);

			spdlog::warn("TRACKING {} controller:\n"
			             "  raw:    q=({:.4f},{:.4f},{:.4f},{:.4f}) pos_mm=({:.1f},{:.1f},{:.1f})\n"
			             "  orient: q=({:.4f},{:.4f},{:.4f},{:.4f})\n"
			             "  grip_off: ({:.4f},{:.4f},{:.4f})\n"
			             "  final:  q=({:.4f},{:.4f},{:.4f},{:.4f}) pos_m=({:.4f},{:.4f},{:.4f})",
				h == 0 ? "LEFT" : "RIGHT",
				cs[h].orientation[0], cs[h].orientation[1], cs[h].orientation[2], cs[h].orientation[3],
				cs[h].position[0], cs[h].position[1], cs[h].position[2],
				cq.x, cq.y, cq.z, cq.w,
				grip_world[0], grip_world[1], grip_world[2],
				cq.x, cq.y, cq.z, cq.w,
				cs[h].position[0] * 0.001f + grip_world[0],
				cs[h].position[1] * 0.001f + grip_world[1] + h_offset,
				cs[h].position[2] * 0.001f + grip_world[2]);
		}
	}

	// Eye gaze (Neo 2 EYE): head-local gaze orientation, zero position.
	pollEyeGaze();
	if (gEyeOnline.load() && gGazeValid.load())
	{
		from_headset::tracking::pose gaze_p{};
		gaze_p.pose.orientation = {
			gGazeQuat[0].load(),
			gGazeQuat[1].load(),
			gGazeQuat[2].load(),
			gGazeQuat[3].load()};
		gaze_p.pose.position = {0, 0, 0};
		gaze_p.device = device_id::EYE_GAZE;
		gaze_p.flags = from_headset::pose_flags::orientation_valid |
		               from_headset::pose_flags::orientation_tracked;
		pkt.device_poses.push_back(gaze_p);
	}

	// Face tracking (fb_face2): eye blink (closedness) + eye look directions.
	// Pupil dilation is encoded in two unused blendshape slots (see below).
	if (gEyeOnline.load() && gEyeOpennessValid.load())
	{
		auto & face = pkt.face.emplace<from_headset::tracking::fb_face2>();
		face.time = pkt.timestamp;
		face.is_valid = true;
		face.is_eye_following_blendshapes_valid = gGazeValid.load();
		face.weights.fill(0.0f);
		face.confidences.fill(1.0f);

		// Eye closedness = 1 - openness (0=open, 1=closed)
		// Index 12 = EYES_CLOSED_L, Index 13 = EYES_CLOSED_R
		float openL = gEyeOpenness[0].load();
		float openR = gEyeOpenness[1].load();
		face.weights[12] = 1.0f - openL;
		face.weights[13] = 1.0f - openR;

		// Eye look directions from gaze pitch/yaw (indices 14-21).
		// Max look angle ~30 degrees; scale to 0..1 blendshape weight.
		if (gGazeValid.load())
		{
			float pitch = gGazePitch.load();
			float yaw   = gGazeYaw.load();
			constexpr float max_angle = 0.5236f; // ~30 deg
			float pitch_n = std::clamp(pitch / max_angle, -1.0f, 1.0f);
			float yaw_n   = std::clamp(yaw / max_angle, -1.0f, 1.0f);
			// pitch > 0 = looking up, pitch < 0 = looking down
			if (pitch_n > 0) {
				face.weights[20] = pitch_n; // EYES_LOOK_UP_L
				face.weights[21] = pitch_n; // EYES_LOOK_UP_R
			} else {
				face.weights[14] = -pitch_n; // EYES_LOOK_DOWN_L
				face.weights[15] = -pitch_n; // EYES_LOOK_DOWN_R
			}
			// yaw > 0 = looking left, yaw < 0 = looking right
			if (yaw_n > 0) {
				face.weights[16] = yaw_n; // EYES_LOOK_LEFT_L
				face.weights[17] = yaw_n; // EYES_LOOK_LEFT_R
			} else {
				face.weights[18] = -yaw_n; // EYES_LOOK_RIGHT_L
				face.weights[19] = -yaw_n; // EYES_LOOK_RIGHT_R
			}
		}
	}

	try {
		static int log_counter = 0;
		if (log_counter < 3 || (log_counter % 3000) == 0) {
			ALOGI("transmit_tracking[%d]: %zu device_poses, prod_ts=%lld, ts=%lld, view_flags=0x%x",
				log_counter, pkt.device_poses.size(),
				(long long)pkt.production_timestamp,
				(long long)pkt.timestamp,
				(unsigned int)pkt.view_flags);
			for (const auto & dp : pkt.device_poses)
				ALOGI("  pose: device=%d flags=0x%x pos=(%.3f,%.3f,%.3f)",
					(int)dp.device, dp.flags,
					dp.pose.position.x, dp.pose.position.y, dp.pose.position.z);
		}
		log_counter++;
		session->send_stream(pkt);
	} catch (std::exception & e) {
		spdlog::warn("transmit_tracking: send failed: {}", e.what());
	}
}

void pico_native_tracker::transmit_inputs(int64_t headset_ns)
{
	if (!session)
		return;

	controller_sample cs[2];
	{
		std::lock_guard lock(state_mutex);
		cs[0] = controllers[0];
		cs[1] = controllers[1];
	}

	from_headset::inputs pkt{};
	XrTime now = to_xr_time(headset_ns);

	// Index into prev_inputs[c]: 0=trigger_value, 1=trigger_click, 2=stick_x, 3=stick_y,
	// 4=stick_click, 5=button_a/x, 6=button_b/y, 7=menu/system, 8=squeeze_value, 9=squeeze_click
	auto add = [&](int c, int idx, device_id id, float value) {
		auto & prev = prev_inputs[c][idx];
		bool changed = (prev.last_change_time == 0 || prev.value != value);
		if (changed)
			prev.last_change_time = now;
		prev.value = value;
		pkt.values.push_back({.id = id, .value = value, .last_change_time = prev.last_change_time});
		if (changed && idx >= 5 && idx <= 7)
			spdlog::warn("INPUT_CHG c={} idx={} id={} val={:.1f} t={}", c, idx, (int)id, value, prev.last_change_time);
	};

	for (int c = 0; c < 2; c++)
	{
		if (!cs[c].connected)
			continue;

		bool is_left = (c == 0);

		float trigger_val = cs[c].trigger / 255.0f;
		float touch_x = (cs[c].touch[0] - 128) / 128.0f;
		float touch_y = (cs[c].touch[1] - 128) / 128.0f;
		float trigger_click = trigger_val > 0.5f ? 1.0f : 0.0f;
		float squeeze = cs[c].grip ? 1.0f : 0.0f;

		if (is_left)
		{
			add(c, 0, device_id::LEFT_TRIGGER_VALUE, trigger_val);
			add(c, 1, device_id::LEFT_TRIGGER_CLICK, trigger_click);
			add(c, 2, device_id::LEFT_THUMBSTICK_X, touch_x);
			add(c, 3, device_id::LEFT_THUMBSTICK_Y, touch_y);
			add(c, 4, device_id::LEFT_THUMBSTICK_CLICK, cs[c].thumbstick_click ? 1.0f : 0.0f);
			add(c, 5, device_id::X_CLICK, cs[c].button_a ? 1.0f : 0.0f);
			add(c, 6, device_id::Y_CLICK, cs[c].button_b ? 1.0f : 0.0f);
			add(c, 7, device_id::MENU_CLICK, cs[c].menu ? 1.0f : 0.0f);
			add(c, 8, device_id::LEFT_SQUEEZE_VALUE, squeeze);
			add(c, 9, device_id::LEFT_SQUEEZE_CLICK, squeeze);
		}
		else
		{
			add(c, 0, device_id::RIGHT_TRIGGER_VALUE, trigger_val);
			add(c, 1, device_id::RIGHT_TRIGGER_CLICK, trigger_click);
			add(c, 2, device_id::RIGHT_THUMBSTICK_X, touch_x);
			add(c, 3, device_id::RIGHT_THUMBSTICK_Y, touch_y);
			add(c, 4, device_id::RIGHT_THUMBSTICK_CLICK, cs[c].thumbstick_click ? 1.0f : 0.0f);
			add(c, 5, device_id::A_CLICK, cs[c].button_a ? 1.0f : 0.0f);
			add(c, 6, device_id::B_CLICK, cs[c].button_b ? 1.0f : 0.0f);
			add(c, 7, device_id::SYSTEM_CLICK, cs[c].menu ? 1.0f : 0.0f);
			add(c, 8, device_id::RIGHT_SQUEEZE_VALUE, squeeze);
			add(c, 9, device_id::RIGHT_SQUEEZE_CLICK, squeeze);
		}
	}

	if (!pkt.values.empty()) {
		try {
			session->send_stream(pkt);
		} catch (std::exception & e) {
			spdlog::warn("transmit_inputs: send failed: {}", e.what());
		}
	}
}
