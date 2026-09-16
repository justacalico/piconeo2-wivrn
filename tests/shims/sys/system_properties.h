#pragma once
// Host shim for <sys/system_properties.h>. Properties are held in a small
// test-controlled table; set them with test_set_property().
#include <cstddef>

#define PROP_VALUE_MAX 92

#ifdef __cplusplus
extern "C" {
#endif

// Test hook: set a property value (nullptr value clears it). Returns 0 on
// success. Implemented in tests/stubs.cpp.
int test_set_property(const char *name, const char *value);

int __system_property_get(const char *name, char *value);

#ifdef __cplusplus
}
#endif
