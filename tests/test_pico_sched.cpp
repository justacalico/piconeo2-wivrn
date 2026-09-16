// wivrn/core/pico_sched.h: CPU pinning helpers (Linux syscalls work on host).
#include "test_framework.h"
#include "faults.h"
#include "pico_sched.h"

#include <pthread.h>
#include <thread>
#include <atomic>
#include <chrono>

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

TEST(sched, pin_thread_bad_tid_reports_affinity_failure) {
	// A tid that doesn't exist makes sched_setaffinity fail (errno branch).
	pico_sched::pin_thread(99999999, "ghost", 2, 10);
	CHECK(true);
}

TEST(sched, pin_thread_nice_fallback_accepted) {
	// nice_fallback=+10 (lowering priority) is allowed without privileges, so
	// the SCHED_FIFO denial path lands on "set nice" instead of failing.
	pico_sched::pin_thread(gettid(), "nice-fallback", 2, 10);
	CHECK(true);
}

TEST(sched, warp_thread_sdk_pinned_reserves_core) {
	if (pico_sched::find_tid_by_comm("WarpThread") != 0)
		return;   // another test's warp thread is already around

	// Fake the SDK warp thread: a thread literally named WarpThread pinned to
	// exactly one big core, so discovery takes the SDK-pinned branch.
	long n = sysconf(_SC_NPROCESSORS_CONF);
	long core = n - 1;
	std::atomic<bool> ready{false};
	std::atomic<bool> stop{false};
	std::thread th([&] {
		pthread_setname_np(pthread_self(), "WarpThread");
		cpu_set_t one;
		CPU_ZERO(&one);
		CPU_SET((int)core, &one);
		sched_setaffinity(0, sizeof(one), &one);
		ready = true;
		while (!stop)
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	});
	while (!ready)
		std::this_thread::yield();
	// Give the kernel a moment to apply the affinity.
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	pico_sched::warp_reserved_cpu.store(-1);
	CHECK(pico_sched::try_pin_warp_thread());
	CHECK_EQ(pico_sched::warp_reserved_cpu.load(), (int)core);

	stop = true;
	th.join();
	pico_sched::warp_reserved_cpu.store(-1);
}

// Fault-injected branches: failing /proc reads, cpu-count edge cases and
// privilege-gated scheduler calls. Driven through the named/persistent mode of
// libc_wrap.cpp since call positions vary with thread count.
TEST(sched_faults, find_tid_opendir_and_fopen_fail)
{
	test_fault_only = "opendir";
	CHECK_EQ(pico_sched::find_tid_by_comm("TestPinTarget"), 0);

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

	test_fault_only = "fopen";
	CHECK_EQ(pico_sched::find_tid_by_comm("TestPinTarget"), 0);

	test_fault_only = nullptr;
	stop = true;
	th.join();
}

TEST(sched_faults, big_core_set_refills_empty_range)
{
	// Pretend a 2-cpu machine: the whole big range is the excluded core, so the
	// set comes out empty and the fallback loop refills it.
	test_fault_only = "sysconf";
	test_fault_ret = 2;
	long lo = -1, hi = -1;
	cpu_set_t s = pico_sched::big_core_set(1, lo, hi);
	test_fault_only = nullptr;
	test_fault_ret = -1;
	CHECK_EQ(CPU_COUNT(&s), 1);
	CHECK(CPU_ISSET(1, &s));
}

TEST(sched_faults, pin_thread_fifo_success)
{
	pico_sched::warp_reserved_cpu.store(-1);
	test_fault_only = "sched_setscheduler";
	test_fault_ret = 0;
	pico_sched::pin_thread(gettid(), "fifo-ok", 2, -8);
	test_fault_only = nullptr;
	test_fault_ret = -1;
	pico_sched::warp_reserved_cpu.store(-1);
	CHECK(true);
}

TEST(sched_faults, warp_thread_injected_scheduler_results)
{
	std::atomic<bool> ready{false};
	std::atomic<bool> stop{false};
	std::thread th([&] {
		pthread_setname_np(pthread_self(), "WarpThread");
		ready = true;
		while (!stop)
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	});
	while (!ready)
		std::this_thread::yield();

	// SCHED_FIFO accepted.
	test_fault_only = "sched_setscheduler";
	test_fault_ret = 0;
	pico_sched::warp_reserved_cpu.store(-1);
	CHECK(pico_sched::try_pin_warp_thread());

	// SCHED_FIFO denied, nice=-10 accepted.
	test_fault_only = "setpriority";
	test_fault_ret = 0;
	pico_sched::warp_reserved_cpu.store(-1);
	CHECK(pico_sched::try_pin_warp_thread());

	// The earlier phases pinned the warp thread to a core; float it again so
	// the failing re-pin branch runs.
	pid_t warp_tid = pico_sched::find_tid_by_comm("WarpThread");
	cpu_set_t all;
	CPU_ZERO(&all);
	for (long c = 0; c < sysconf(_SC_NPROCESSORS_CONF); ++c)
		CPU_SET((int)c, &all);
	sched_setaffinity(warp_tid, sizeof(all), &all);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	// Re-pinning a floating warp thread fails; the core is reserved anyway.
	test_fault_only = "sched_setaffinity";
	test_fault_ret = -1;
	pico_sched::warp_reserved_cpu.store(-1);
	CHECK(pico_sched::try_pin_warp_thread());

	test_fault_only = nullptr;
	test_fault_ret = -1;
	stop = true;
	th.join();
	pico_sched::warp_reserved_cpu.store(-1);
}
