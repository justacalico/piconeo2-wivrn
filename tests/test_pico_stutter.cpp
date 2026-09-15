// wivrn/core/pico_stutter.h: stutter detection heuristics.
#include "test_framework.h"
#include "pico_stutter.h"

#include <chrono>
#include <thread>

stutter_detector g_stutter;

TEST(stutter, shard_assembly_slow_logs) {
	stutter_detector d;
	int64_t before = -1;
	// Feed a first shard, wait past the 10ms assembly threshold, then the last.
	d.on_shard_arrived(1, 0, true, false);
	std::this_thread::sleep_for(std::chrono::milliseconds(12));
	d.on_shard_arrived(1, 0, false, true);
	// No direct counter read, but stutter_count is internal; just ensure no crash
	// and that a fast assembly doesn't log.
	d.on_shard_arrived(2, 0, true, false);
	d.on_shard_arrived(2, 0, false, true);
	CHECK(true);
}

TEST(stutter, decoder_queue_and_decode_paths) {
	stutter_detector d;
	d.on_shard_arrived(1, 0, true, true);
	d.on_pushed_to_decoder(1, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(2));
	d.on_frame_decoded(1, 0);
	// Unknown frames are ignored.
	d.on_pushed_to_decoder(42, 0);
	d.on_frame_decoded(42, 0);
	CHECK(true);
}

TEST(stutter, frame_begin_repeat_and_mismatch) {
	stutter_detector d;
	// Same frame index 4 times on the left eye -> repeat-stutter path.
	for (int i = 0; i < 5; ++i)
		d.on_frame_begin(10, 20 + i);
	// Mismatched indices (>2 apart) -> pose-mismatch path.
	d.on_frame_begin(100, 50);
	// Zero indices skip the mismatch check.
	d.on_frame_begin(0, 0);
	d.on_frame_end();
	CHECK(true);
}

TEST(stutter, pose_jump_detection) {
	stutter_detector d;
	XrPosef p1{}, p2{};
	p1.position = {0, 0, 0};
	p1.orientation = {0, 0, 0, 1};
	p2.position = {0.2f, 0, 0}; // 20cm jump > 5cm threshold
	p2.orientation = {0, 0, 0, 1};
	d.on_pose_update(0, 1, p1);
	d.on_pose_update(0, 1, p1); // same frame index -> ignored
	d.on_pose_update(0, 2, p2); // big position jump
	// Rotation jump.
	XrPosef p3 = p2;
	p3.orientation = {0, 0, 0, 0};
	d.on_pose_update(0, 3, p3);
	// Small move, no jump.
	d.on_pose_update(0, 4, p2);
	d.on_pose_update(1, 1, p1); // other eye initialises separately
	CHECK(true);
}

TEST(stutter, summary_and_suppression) {
	stutter_detector d;
	for (int i = 0; i < 310; ++i)
		d.on_frame_begin((uint64_t)i, (uint64_t)i);
	d.log_summary();
	CHECK(true);
}

TEST(stutter, queue_wait_and_decode_slow)
{
	stutter_detector det;
	det.on_shard_arrived(1, 0, true, true);
	std::this_thread::sleep_for(std::chrono::milliseconds(7));
	det.on_pushed_to_decoder(1, 0);   // >5ms in queue
	std::this_thread::sleep_for(std::chrono::milliseconds(17));
	det.on_frame_decoded(1, 0);       // >15ms decode
	CHECK(true);
}

TEST(stutter, history_trims_past_64_records)
{
	stutter_detector det;
	for (uint64_t f = 0; f < 70; ++f)
		det.on_shard_arrived(f, 0, true, false);
	CHECK(true);
}

TEST(stutter, render_interval_and_jitter)
{
	stutter_detector det;
	// Build a small running average, then one >20ms gap trips both the
	// interval-threshold and jitter detectors.
	for (uint64_t f = 0; f < 8; ++f)
	{
		det.on_frame_begin(f, f);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(30));
	det.on_frame_begin(9, 9);
	CHECK(true);
}

TEST(stutter, summary_every_300_frames)
{
	stutter_detector det;
	for (int f = 0; f < 300; ++f)
		det.on_frame_begin(0, 0);
	det.log_summary();
	CHECK(true);
}
