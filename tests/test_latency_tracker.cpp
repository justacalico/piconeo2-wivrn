// wivrn/core/latency_tracker.h: per-frame pipeline timing aggregation.
#include "test_framework.h"
#include "latency_tracker.h"

#include <chrono>
#include <thread>

// Defined in streaming_client.cpp on device.
latency_tracker g_latency;

namespace
{
// Drive one frame through the full pipeline with a known server-side window.
void push_frame(latency_tracker & t, uint64_t idx, int64_t enc_begin)
{
	t.on_server_timing(idx, 0, enc_begin, enc_begin + 2'000'000,
	                   enc_begin + 3'000'000, enc_begin + 4'000'000);
	t.on_shard_received(idx, 0, true, false);
	t.on_shard_received(idx, 0, false, true);
	t.on_pushed_to_decoder(idx, 0);
	t.on_frame_decoded(idx, 0);
	t.on_frame_rendered(idx, 0);
	t.on_frame_submitted(idx, 0, enc_begin + 30'000'000);
}
} // namespace

TEST(latency, empty_tracker_reports_zero) {
	latency_tracker t;
	CHECK_EQ(t.get_avg_total_latency_ns(), 0);
	auto bd = t.get_avg_breakdown_ms();
	for (float v : bd)
		CHECK(v == 0.0f);
}

TEST(latency, complete_frame_averaged) {
	latency_tracker t;
	push_frame(t, 1, 1'000'000'000);
	push_frame(t, 2, 2'000'000'000);

	int64_t avg = t.get_avg_total_latency_ns();
	// submitted - encode_begin = 30ms - 0 = ~29ms each.
	CHECK(avg > 25'000'000);
	CHECK(avg < 35'000'000);

	auto bd = t.get_avg_breakdown_ms();
	CHECK(bd[0] == 2.0f);  // encode = enc_end - enc_begin
	CHECK(bd[1] == 1.0f);  // send = send_end - send_begin
}

TEST(latency, incomplete_frames_skipped) {
	latency_tracker t;
	push_frame(t, 1, 1'000'000'000);
	// Frame 2 never completes: no decoded/rendered/submitted.
	t.on_server_timing(2, 0, 0, 1, 2, 3);
	t.on_shard_received(2, 0, true, true);

	int64_t only = t.get_avg_total_latency_ns();
	CHECK(only > 0);
}

TEST(latency, missing_records_are_noops) {
	latency_tracker t;
	// decoded/rendered/submitted for a frame that was never registered.
	t.on_frame_decoded(99, 0);
	t.on_frame_rendered(99, 0);
	t.on_frame_submitted(99, 0, 123);
	CHECK_EQ(t.get_avg_total_latency_ns(), 0);
}

TEST(latency, per_stream_separation) {
	latency_tracker t;
	push_frame(t, 7, 1'000'000'000);        // stream 0
	t.on_server_timing(7, 1, 5, 6, 7, 8);   // same index, other stream
	t.on_shard_received(7, 1, true, true);
	t.on_pushed_to_decoder(7, 1);
	t.on_frame_decoded(7, 1);
	t.on_frame_rendered(7, 1);
	t.on_frame_submitted(7, 1, 5 + 30'000'000);
	CHECK(t.get_avg_total_latency_ns() > 0);
}

TEST(latency, reset_clears_everything) {
	latency_tracker t;
	push_frame(t, 1, 1'000'000'000);
	t.reset();
	CHECK_EQ(t.get_avg_total_latency_ns(), 0);
}

TEST(latency, history_trims_at_cap) {
	latency_tracker t;
	// Push more than HISTORY_SIZE frames; oldest get evicted.
	for (uint64_t i = 0; i < latency_tracker::HISTORY_SIZE + 20; ++i)
		t.on_server_timing(i, 0, 0, 1, 2, 3);
	// No crash and still consistent state after eviction.
	CHECK(true);
}

TEST(latency, logging_paths_run) {
	latency_tracker t;
	// First 10 completes log the frame line; 90th logs the summary too.
	for (uint64_t i = 0; i < latency_tracker::SUMMARY_INTERVAL; ++i)
		push_frame(t, i + 1, (int64_t)i * 50'000'000 + 1'000'000'000);
	// An incomplete frame hitting log_frame's early branch.
	latency_tracker t2;
	t2.on_frame_submitted(5, 0, 1); // no record -> no-op
	t2.on_server_timing(6, 0, 0, 0, 0, 0);
	t2.on_frame_submitted(6, 0, 1); // incomplete -> INCOMPLETE log path
	CHECK(true);
}

TEST(latency, records_created_by_shard_and_decoder_events)
{
	latency_tracker tr;
	// No server timing registered: the event itself creates the record.
	tr.on_shard_received(5, 0, true, false);
	tr.on_pushed_to_decoder(5, 0);
	tr.on_frame_submitted(5, 0, 123456789LL);
	// Frame is complete but lacks server timing and most fields: logged as
	// INCOMPLETE and skipped by every aggregate.
	CHECK_EQ(tr.get_avg_total_latency_ns(), 0);
	auto bd = tr.get_avg_breakdown_ms();
	for (float v: bd)
		CHECK_EQ(v, 0.0f);
}

TEST(latency, breakdown_empty_tracker)
{
	latency_tracker tr;
	CHECK_EQ(tr.get_avg_total_latency_ns(), 0);
	auto bd = tr.get_avg_breakdown_ms();
	for (float v: bd)
		CHECK_EQ(v, 0.0f);
}

TEST(latency, summary_with_no_complete_frames)
{
	latency_tracker tr;
	// 90 completed-but-empty frames: logs INCOMPLETE for the first 10 and a
	// "no complete frames" summary at the 90th.
	for (uint64_t f = 1; f <= 90; ++f)
	{
		tr.on_shard_received(f, 0, true, true);
		tr.on_frame_submitted(f, 0, 123456789LL);
	}
	CHECK_EQ(tr.get_avg_total_latency_ns(), 0);
}

TEST(latency, decoder_push_creates_record) {
	latency_tracker t;
	// Decoder push is the first event seen for this frame index.
	t.on_pushed_to_decoder(9001, 0);
	CHECK_EQ(t.get_avg_total_latency_ns(), 0);
}

TEST(latency, complete_but_missing_fields_skip_summaries) {
	latency_tracker t;
	// 90 submitted frames that never saw a decode event: complete records that
	// fail the field check in every aggregation loop. The 90th submit also runs
	// the periodic summary, which finds zero qualifying frames and warns.
	for (uint64_t i = 1; i <= 90; ++i) {
		t.on_server_timing(i, 0, 100, 200, 300, 400);
		t.on_shard_received(i, 0, true, true);
		t.on_pushed_to_decoder(i, 0);
		t.on_frame_rendered(i, 0);
		t.on_frame_submitted(i, 0, 500);
	}
	CHECK_EQ(t.get_avg_total_latency_ns(), 0);
	auto bd = t.get_avg_breakdown_ms();
	for (float v : bd)
		CHECK(v == 0.0f);
}

TEST(latency, summary_with_complete_frames) {
	latency_tracker t;
	// Fully-instrumented frames: every field set, so the periodic summary at
	// frame 90 aggregates real stage numbers instead of warning about emptiness.
	for (uint64_t i = 1; i <= 90; ++i) {
		t.on_server_timing(i, 0, 100, 200, 300, 400);
		t.on_shard_received(i, 0, true, true);
		t.on_pushed_to_decoder(i, 0);
		t.on_frame_decoded(i, 0);
		t.on_frame_rendered(i, 0);
		t.on_frame_submitted(i, 0, 500);
	}
	// submitted(500) - encode_begin(100) = 400ns per frame.
	CHECK_EQ(t.get_avg_total_latency_ns(), 400);
	auto bd = t.get_avg_breakdown_ms();
	CHECK(bd[0] > 0.0f);   // encode
	CHECK(bd[1] > 0.0f);   // send
	CHECK(bd[2] > 0.0f);   // network (real now_ns > 400ns)
	CHECK(bd[5] != 0.0f);  // blit (negative: fake submit < real render time)
}
