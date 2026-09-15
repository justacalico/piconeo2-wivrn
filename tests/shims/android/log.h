#pragma once
// Host shim for <android/log.h>: routes log lines to stderr so tests can see
// (and a debugger can catch) what the code emits.
#include <cstdarg>
#include <cstdio>

enum {
	ANDROID_LOG_UNKNOWN = 0,
	ANDROID_LOG_DEFAULT,
	ANDROID_LOG_VERBOSE,
	ANDROID_LOG_DEBUG,
	ANDROID_LOG_INFO,
	ANDROID_LOG_WARN,
	ANDROID_LOG_ERROR,
	ANDROID_LOG_FATAL,
	ANDROID_LOG_SILENT,
};

static inline int __android_log_print(int prio, const char *tag, const char *fmt, ...) {
	(void)prio;
	std::fprintf(stderr, "[%s] ", tag);
	va_list ap;
	va_start(ap, fmt);
	int n = std::vfprintf(stderr, fmt, ap);
	va_end(ap);
	std::fputc('\n', stderr);
	return n;
}
