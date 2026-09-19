// input/input.cpp: queueHaptic validation, clamping and coalescing.
#include "test_framework.h"
#include "input.h"

namespace
{
void reset_haptics()
{
	std::lock_guard<std::mutex> lk(gHapticMutex);
	gHaptic[0] = PendingHaptic{};
	gHaptic[1] = PendingHaptic{};
}

PendingHaptic snapshot(int hand)
{
	std::lock_guard<std::mutex> lk(gHapticMutex);
	return gHaptic[hand];
}
} // namespace

TEST(input, rejects_invalid_hand) {
	reset_haptics();
	queueHaptic(-1, 1.0f, 100000000);
	queueHaptic(2, 1.0f, 100000000);
	CHECK(!snapshot(0).pending);
	CHECK(!snapshot(1).pending);
}

TEST(input, rejects_nonpositive_amplitude_and_duration) {
	reset_haptics();
	queueHaptic(0, 0.0f, 100000000);
	queueHaptic(0, -0.5f, 100000000);
	queueHaptic(1, 0.5f, 0);
	queueHaptic(1, 0.5f, -1000000);
	CHECK(!snapshot(0).pending);
	CHECK(!snapshot(1).pending);
}

TEST(input, clamps_amplitude_to_one) {
	reset_haptics();
	queueHaptic(0, 5.0f, 100000000);
	PendingHaptic p = snapshot(0);
	CHECK(p.pending);
	CHECK(p.amplitude == 1.0f);
}

TEST(input, duration_floor_and_cap) {
	reset_haptics();
	// 1ms rounds up to the 12ms floor.
	queueHaptic(0, 0.5f, 1000000);
	CHECK_EQ(snapshot(0).durationMs, 12);

	// 5s clamps down to the 1000ms cap.
	reset_haptics();
	queueHaptic(0, 0.5f, 5000000000LL);
	CHECK_EQ(snapshot(0).durationMs, 1000);
}

TEST(input, duration_truncates_to_ms) {
	reset_haptics();
	queueHaptic(0, 0.5f, 100999999); // 100.999ms -> 100
	CHECK_EQ(snapshot(0).durationMs, 100);
	queueHaptic(1, 0.5f, 101000000); // 101ms -> 101
	CHECK_EQ(snapshot(1).durationMs, 101);
}

TEST(input, coalesce_keeps_strongest_and_longest) {
	reset_haptics();
	queueHaptic(0, 0.4f, 50000000);
	queueHaptic(0, 0.9f, 20000000); // stronger, shorter
	PendingHaptic p = snapshot(0);
	CHECK(p.amplitude == 0.9f);
	CHECK_EQ(p.durationMs, 50);

	queueHaptic(0, 0.1f, 80000000); // weaker, longer
	p = snapshot(0);
	CHECK(p.amplitude == 0.9f);
	CHECK_EQ(p.durationMs, 80);
}

TEST(input, coalesce_after_drain) {
	reset_haptics();
	queueHaptic(0, 0.4f, 50000000);
	{
		std::lock_guard<std::mutex> lk(gHapticMutex);
		gHaptic[0].pending = false; // simulate the poller draining it
	}
	// A weaker pulse after drain still wins because pending was cleared.
	queueHaptic(0, 0.2f, 30000000);
	PendingHaptic p = snapshot(0);
	CHECK(p.pending);
	CHECK(p.amplitude == 0.2f);
	CHECK_EQ(p.durationMs, 30);
}

TEST(input, hands_are_independent) {
	reset_haptics();
	queueHaptic(0, 0.3f, 50000000);
	queueHaptic(1, 0.7f, 20000000);
	CHECK(snapshot(0).amplitude == 0.3f);
	CHECK(snapshot(1).amplitude == 0.7f);
}
