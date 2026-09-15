#pragma once
// Tiny test framework: TEST(suite, name) registers a case, main() runs them
// all and reports. CHECK logs a failure but keeps going so one bad assertion
// doesn't hide the rest.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace testfw {

struct Case {
	const char *suite;
	const char *name;
	std::function<void()> fn;
};

inline std::vector<Case> &registry() {
	static std::vector<Case> r;
	return r;
}

struct Registrar {
	Registrar(const char *suite, const char *name, std::function<void()> fn) {
		registry().push_back({suite, name, std::move(fn)});
	}
};

inline int &checks() { static int n = 0; return n; }
inline int &failures() { static int n = 0; return n; }

inline void fail_at(const char *file, int line, const char *expr) {
	++failures();
	std::fprintf(stderr, "  FAIL %s:%d  %s\n", file, line, expr);
}

inline int run_all() {
	int passed = 0, failed = 0;
	for (const auto &c : registry()) {
		int before = failures();
		std::fprintf(stderr, "[RUN ] %s.%s\n", c.suite, c.name);
		try {
			c.fn();
		} catch (const std::exception &e) {
			fail_at(__FILE__, __LINE__, "uncaught exception");
			std::fprintf(stderr, "       threw: %s\n", e.what());
		} catch (...) {
			fail_at(__FILE__, __LINE__, "uncaught non-std exception");
		}
		if (failures() == before) {
			++passed;
			std::fprintf(stderr, "[ OK ] %s.%s\n", c.suite, c.name);
		} else {
			++failed;
		}
	}
	std::fprintf(stderr, "\n%d/%zu cases passed, %d checks, %d failures\n",
	             passed, registry().size(), checks(), failures());
	return failed;
}

} // namespace testfw

#define TEST(suite, name)                                                              \
	static void suite##_##name##_body();                                           \
	static testfw::Registrar suite##_##name##_reg(#suite, #name,                   \
	                                            suite##_##name##_body);            \
	static void suite##_##name##_body()

#define CHECK(cond)                                                                    \
	do {                                                                           \
		++testfw::checks();                                                    \
		if (!(cond))                                                           \
			testfw::fail_at(__FILE__, __LINE__, "CHECK(" #cond ")");       \
	} while (0)

#define CHECK_EQ(a, b)                                                                 \
	do {                                                                           \
		++testfw::checks();                                                    \
		auto _a = (a);                                                         \
		auto _b = (b);                                                         \
		if (!(_a == _b)) {                                                     \
			testfw::fail_at(__FILE__, __LINE__,                            \
			                "CHECK_EQ(" #a ", " #b ")");                   \
		}                                                                      \
	} while (0)

#define CHECK_NEAR(a, b, tol)                                                          \
	do {                                                                           \
		++testfw::checks();                                                    \
		double _a = (a), _b = (b);                                             \
		if (std::fabs(_a - _b) > (tol)) {                                      \
			char _buf[256];                                                \
			std::snprintf(_buf, sizeof(_buf),                              \
			              "CHECK_NEAR(%s=%g, %s=%g, tol=%g)", #a, _a, #b,  \
			              _b, (double)(tol));                              \
			testfw::fail_at(__FILE__, __LINE__, _buf);                     \
		}                                                                      \
	} while (0)

#define CHECK_THROWS(expr)                                                             \
	do {                                                                           \
		++testfw::checks();                                                    \
		bool _threw = false;                                                   \
		try {                                                                  \
			(void)(expr);                                                  \
		} catch (...) {                                                        \
			_threw = true;                                                 \
		}                                                                      \
		if (!_threw)                                                           \
			testfw::fail_at(__FILE__, __LINE__,                            \
			                "CHECK_THROWS(" #expr ")");                    \
	} while (0)

#define CHECK_THROWS_AS(expr, exc_type)                                                \
	do {                                                                           \
		++testfw::checks();                                                    \
		bool _right = false;                                                   \
		try {                                                                  \
			(void)(expr);                                                  \
		} catch (const exc_type &) {                                           \
			_right = true;                                                 \
		} catch (...) {                                                        \
		}                                                                      \
		if (!_right)                                                           \
			testfw::fail_at(__FILE__, __LINE__,                            \
			                "CHECK_THROWS_AS(" #expr ", " #exc_type ")");  \
	} while (0)
