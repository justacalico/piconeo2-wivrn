#pragma once

#include <spdlog/spdlog.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>
#include <atomic>
#include <cstdio>
#include <cstring>

// CPU pinning + priority for the latency-critical threads on the Neo 2:
// SD845 has cpu0-3 little / cpu4-7 big. Background tasks
// parking our decode/render threads on little cores stretch decode->present
// latency and add frame-arrival jitter. SCHED_FIFO usually EPERMs for a normal
// app so we fall back to a favourable nice value. The top big core is reserved
// for the SDK's per-vsync WarpThread.

namespace pico_sched
{

inline std::atomic<int> warp_reserved_cpu{-1};

inline pid_t find_tid_by_comm(const char * name)
{
	DIR * d = opendir("/proc/self/task");
	if (!d)
		return 0;
	pid_t tid = 0;
	struct dirent * e;
	while ((e = readdir(d)) != nullptr)
	{
		if (e->d_name[0] < '0' || e->d_name[0] > '9')
			continue;
		char path[80];
		snprintf(path, sizeof(path), "/proc/self/task/%s/comm", e->d_name);
		FILE * f = fopen(path, "r");
		if (!f)
			continue;
		char comm[64] = {0};
		if (fgets(comm, sizeof(comm), f))
		{
			char * nl = strchr(comm, '\n');
			if (nl)
				*nl = 0;
			if (strcmp(comm, name) == 0)
				tid = (pid_t)atoi(e->d_name);
		}
		fclose(f);
		if (tid)
			break;
	}
	closedir(d);
	return tid;
}

// Big-core half minus the warp's reserved core.
inline cpu_set_t big_core_set(int except_cpu, long & lo_out, long & hi_out)
{
	long n = sysconf(_SC_NPROCESSORS_CONF);
	long lo = (n >= 2) ? n / 2 : 0, hi = n - 1;
	cpu_set_t set;
	CPU_ZERO(&set);
	for (long c = lo; c <= hi; c++)
		if ((int)c != except_cpu)
			CPU_SET((int)c, &set);
	if (CPU_COUNT(&set) == 0)
		for (long c = lo; c <= hi; c++)
			CPU_SET((int)c, &set);
	lo_out = lo;
	hi_out = hi;
	return set;
}

inline void pin_thread(pid_t tid, const char * label, int rt_prio, int nice_fallback)
{
	int reserved = warp_reserved_cpu.load();
	long lo = 0, hi = 0;
	cpu_set_t set = big_core_set(reserved, lo, hi);
	if (sched_setaffinity(tid, sizeof(set), &set) == 0)
		spdlog::info("{} tid={} pinned to big cores [{}..{}] minus warp core {}", label, (int)tid, lo, hi, reserved);
	else
		spdlog::info("{} affinity failed (errno={})", label, errno);

	struct sched_param sp{};
	sp.sched_priority = rt_prio;
	if (sched_setscheduler(tid, SCHED_FIFO, &sp) == 0)
		spdlog::info("{} SCHED_FIFO prio={}", label, rt_prio);
	else if (setpriority(PRIO_PROCESS, tid, nice_fallback) == 0)
		spdlog::info("{} SCHED_FIFO denied; set nice={}", label, nice_fallback);
	else
		spdlog::info("{} prio elevation denied (errno={})", label, errno);
}

inline void pin_current_thread(const char * label, int rt_prio = 2, int nice_fallback = -8)
{
	pin_thread(gettid(), label, rt_prio, nice_fallback);
}

// Raise the SDK's DIATW warp thread priority and discover which big core it runs
// on so everything else stays off it. The SDK pins its own warp thread after any
// affinity we set, so read where it landed instead of fighting it. Returns true
// once found + handled.
inline bool try_pin_warp_thread()
{
	pid_t warp_tid = find_tid_by_comm("WarpThread");
	if (!warp_tid)
		return false;

	struct sched_param sp{};
	sp.sched_priority = 3;
	if (sched_setscheduler(warp_tid, SCHED_FIFO, &sp) == 0)
		spdlog::info("WarpThread tid={} SCHED_FIFO prio=3", (int)warp_tid);
	else if (setpriority(PRIO_PROCESS, warp_tid, -10) == 0)
		spdlog::info("WarpThread SCHED_FIFO denied; set nice=-10");
	else
		spdlog::info("WarpThread prio elevation denied (errno={})", errno);

	long n = sysconf(_SC_NPROCESSORS_CONF);
	long big_lo = (n >= 2) ? n / 2 : 0, big_hi = n - 1;
	int warp_core = (int)big_hi;
	cpu_set_t cur;
	CPU_ZERO(&cur);
	if (sched_getaffinity(warp_tid, sizeof(cur), &cur) == 0)
	{
		int big_set = 0, high_big = -1;
		for (long c = big_lo; c <= big_hi; c++)
			if (CPU_ISSET((int)c, &cur))
			{
				big_set++;
				high_big = (int)c;
			}
		if (big_set == 1)
		{
			warp_core = high_big;
			spdlog::info("WarpThread SDK-pinned to big core {} -> reserving it", warp_core);
		}
		else
		{
			cpu_set_t one;
			CPU_ZERO(&one);
			CPU_SET(warp_core, &one);
			if (sched_setaffinity(warp_tid, sizeof(one), &one) == 0)
				spdlog::info("WarpThread was floating ({} big cores) -> pinned to core {}", big_set, warp_core);
			else
				spdlog::info("WarpThread float-pin to cpu{} failed (errno={}); reserving anyway", warp_core, errno);
		}
	}
	warp_reserved_cpu.store(warp_core);
	return true;
}

} // namespace pico_sched
