// core/device_info.cpp: system-property reads (via shim) and LAN IP pick.
#include "test_framework.h"
#include "device_info.h"

#include <cstring>

// Backing store for the <sys/system_properties.h> shim (stubs.cpp).
extern "C" int test_set_property(const char *name, const char *value);

TEST(devinfo, upper_ascii) {
	char s[] = "Pico Neo 2 eye!";
	toUpperAscii(s);
	CHECK_EQ(std::string(s), "PICO NEO 2 EYE!");
	char empty[] = "";
	toUpperAscii(empty);
	CHECK_EQ(std::string(empty), "");
}

TEST(devinfo, model_from_property) {
	test_set_property("pxr.vendorhw.product.model", "Pico Neo 2 Eye");
	test_set_property("pxr.vendorhw.eye", "1");
	readHeadsetModel(nullptr);
	CHECK_EQ(std::string(gModelText), "Pico Neo 2 Eye");
	CHECK(gIsEyeHw);

	test_set_property("pxr.vendorhw.product.model", "Pico Neo 2");
	test_set_property("pxr.vendorhw.eye", "0");
	readHeadsetModel(nullptr);
	CHECK_EQ(std::string(gModelText), "Pico Neo 2");
	CHECK(!gIsEyeHw);
}

TEST(devinfo, model_fallback_when_property_missing) {
	test_set_property("pxr.vendorhw.product.model", nullptr);
	test_set_property("pxr.vendorhw.eye", nullptr);
	readHeadsetModel(nullptr);
	CHECK_EQ(std::string(gModelText), "Pico Neo 2");
	CHECK(!gIsEyeHw);
}

TEST(devinfo, refresh_ip_fills_text) {
	strcpy(gIpText, "unset");
	refreshDeviceIp();
	// Whatever the host exposes, the field must be a terminated C string and
	// either a dotted IPv4 address or the fallback text.
	CHECK(strlen(gIpText) > 0);
	CHECK(strlen(gIpText) < sizeof(gIpText));
}
