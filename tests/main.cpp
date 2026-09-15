#include "test_framework.h"
#include <cstdio>
#include <unistd.h>

extern "C" void __gcov_dump();

int main() {
	int rc = testfw::run_all();
	// The eye-mode worker is a detached thread parked on a condvar; letting
	// exit() run static destructors while it lives deadlocks the process.
	__gcov_dump();
	fflush(nullptr);
	_exit(rc);
}
