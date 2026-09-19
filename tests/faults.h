// Shared fault-injection knobs for the --wrap harnesses in openssl_wrap.cpp
// and libc_wrap.cpp. test_fault_in is a countdown over wrapped calls: when it
// reaches 0 the current call fails, so sweeping N exercises every error branch
// of an operation. -1 disables injection entirely. test_fault_ret chooses the
// return value injected into integer-returning libc calls (default -1).
#pragma once

extern "C" int test_fault_in;
extern "C" long test_fault_ret;
// Persistent mode: the named libc function fails on every call while set.
extern "C" const char * test_fault_only;
// Synthetic poll revents mask; see libc_wrap.cpp.
extern "C" int test_poll_revents;

// Run op() with the Nth wrapped call failing for N=1,2,... until it completes
// without throwing: that is when every injectable site in the call chain has
// been exercised. Ops must be idempotent and free of side effects. Returns the
// number of injected failures that made op() throw.
inline int exercise_faults(auto op)
{
	int threw = 0;
	for (int n = 1; n <= 600; ++n)
	{
		test_fault_in = n;
		try
		{
			op();
			test_fault_in = -1;
			return threw;
		}
		catch (...)
		{
			++threw;
		}
	}
	test_fault_in = -1;
	return threw;
}
