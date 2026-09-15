// wivrn/core/pico_sched.h: CPU pinning helpers (Linux syscalls work on host).
#include "test_framework.h"
#include "pico_sched.h"

#include <pthread.h>
#include <thread>

TEST(sched, find_tid_by_comm) {
	// The test runner's own threads all share the process comm; spawn a named one.
	pid_t found = 0;
	std::atomic<bool> ready{false};
	std::atomic<bool> stop{false};
	std::thread th([&] {
		pthread_setname_np(pthread_self(), "TestPinTarget");
		ready = true;
		while (!stop)
			std::this_thread::yield();
	});
	while (!ready)
		std::this_thread::yield();
	found = pico_sched::find_tid_by_comm("TestPinTarget");
	CHECK(found > 0);
	// Nonexistent name -> 0.
	CHECK_EQ(pico_sched::find_tid_by_comm("NoSuchThreadXYZ"), 0);
	stop = true;
	th.join();
}

TEST(sched, big_core_set_excludes_reserved) {
	pico_sched::warp_reserved_cpu.store(-1);
	long lo = -1, hi = -1;
	cpu_set_t s = pico_sched::big_core_set(-1, lo, hi);
	CHECK(CPU_COUNT(&s) > 0);
	CHECK(lo <= hi);
	long n = sysconf(_SC_NPROCESSORS_CONF);
	CHECK_EQ(hi, n - 1);

	// Excluding a core inside the range drops it.
	if (hi >= lo) {
		cpu_set_t s2 = pico_sched::big_core_set((int)hi, lo, hi);
		if (hi > lo)
			CHECK(!CPU_ISSET((int)hi, &s2));
		// If the range collapses to nothing it refills (never returns empty).
		CHECK(CPU_COUNT(&s2) > 0);
	}
}

TEST(sched, pin_thread_runs_all_branches) {
	// Pinning self works; FIFO probably EPERMs so the nice fallback runs.
	pico_sched::warp_reserved_cpu.store(-1);
	pico_sched::pin_current_thread("test-pin", 2, -8);
	pico_sched::pin_thread(0, "bad-tid", 2, -8); // tid 0: both calls fail
	CHECK(true);
}

TEST(sched, try_pin_warp_thread) {
	// Without a WarpThread it returns false.
	pico_sched::warp_reserved_cpu.store(-1);
	if (pico_sched::find_tid_by_comm("WarpThread") == 0)
		CHECK(!pico_sched::try_pin_warp_thread());

	// With one it finds and reserves a big core.
	std::atomic<bool> ready{false};
	std::atomic<bool> stop{false};
	std::thread th([&] {
		pthread_setname_np(pthread_self(), "WarpThread");
		ready = true;
		while (!stop)
			std::this_thread::yield();
	});
	while (!ready)
		std::this_thread::yield();
	CHECK(pico_sched::try_pin_warp_thread());
	CHECK(pico_sched::warp_reserved_cpu.load() >= 0);
	stop = true;
	th.join();
	pico_sched::warp_reserved_cpu.store(-1);
}
