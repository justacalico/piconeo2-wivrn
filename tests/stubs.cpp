// Host-side stubs for the Android/Pico-side symbols the portable code calls.
// Property table backs the <sys/system_properties.h> shim; the androidUi*
// functions are the JNI pushers the panels call (they record the last call so
// tests can observe them); alvr_* are the client-core C API entry points.
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <sys/system_properties.h>
#include "server_list.h"

// ---- sys/system_properties.h backing store --------------------------------
static std::vector<std::pair<std::string, std::string>> g_props;
static std::mutex g_props_mutex;

extern "C" int test_set_property(const char *name, const char *value) {
	std::lock_guard<std::mutex> lk(g_props_mutex);
	auto it = std::find_if(g_props.begin(), g_props.end(),
	                       [&](const auto &p) { return p.first == name; });
	if (value == nullptr) {
		if (it != g_props.end())
			g_props.erase(it);
		return 0;
	}
	if (it != g_props.end())
		it->second = value;
	else
		g_props.emplace_back(name, value);
	return 0;
}

extern "C" int __system_property_get(const char *name, char *value) {
	std::lock_guard<std::mutex> lk(g_props_mutex);
	for (const auto &p : g_props) {
		if (p.first == name) {
			std::strncpy(value, p.second.c_str(), PROP_VALUE_MAX - 1);
			value[PROP_VALUE_MAX - 1] = 0;
			return (int)p.second.size();
		}
	}
	return 0;
}

// ---- android_ui pushers (defined in jni.cpp on device) ----------------------
std::vector<ServerInfo> gLastPushedServers;
int gPushServersCalls = 0;
std::string gLastConnError;
int gPushConnErrorCalls = 0;
int gLastConnecting = -1;
int gPushConnectingCalls = 0;
int gLastStreaming = -1;
int gPushSettingsCalls = 0;
float gLastTouch[4] = {0, 0, 0, 0};
int gLastBattery[4] = {0, 0, 0, 0};

void androidUiPushTouch(float x, float y, bool pressed, bool clickEdge, float stickY) {
	gLastTouch[0] = x;
	gLastTouch[1] = y;
	gLastTouch[2] = pressed ? 1.0f : 0.0f;
	gLastTouch[3] = clickEdge ? 1.0f : 0.0f;
	(void)stickY;
}

void androidUiPushServers(const std::vector<ServerInfo> &servers) {
	gLastPushedServers = servers;
	++gPushServersCalls;
}

void androidUiPushConnecting(bool connecting) {
	gLastConnecting = connecting ? 1 : 0;
	++gPushConnectingCalls;
}

void androidUiPushConnError(const std::string &err) {
	gLastConnError = err;
	++gPushConnErrorCalls;
}

void androidUiPushStreaming(bool streaming) { gLastStreaming = streaming ? 1 : 0; }
void androidUiPushBattery(int hmdBatt, int leftBatt, bool leftConn, int rightBatt, bool rightConn) {
	gLastBattery[0] = hmdBatt;
	gLastBattery[1] = leftBatt;
	gLastBattery[2] = leftConn ? 1 : 0;
	gLastBattery[3] = rightBatt;
	(void)rightConn;
}
void androidUiPushSettings() { ++gPushSettingsCalls; }
void androidUiPushDiag(int, const float *, const float *) {}
void androidUiPushDiagOverlayOnly(bool) {}
void androidUiPushStats(int, float, float, float, float, float, float, float,
                        float, float, float, float, int, int, int, bool) {}
void androidUiPushRunningApps(const std::vector<std::string> &, const std::vector<int> &,
                              const std::vector<bool> &) {}
void androidUiPushAvailableApps(const std::vector<std::string> &, const std::vector<std::string> &) {}

// ---- ALVR client-core C API -------------------------------------------------
// foveation.cpp reads the settings JSON through this; tests inject JSON via
// test_set_settings_json().
static std::string g_settings_json;

void test_set_settings_json(const std::string &json) { g_settings_json = json; }

extern "C" uint64_t alvr_get_settings_json_bounded(char *out_buffer, uint64_t cap) {
	uint64_t full = g_settings_json.size();
	uint64_t n = std::min<uint64_t>(full, cap - 1);
	if (cap > 0) {
		std::memcpy(out_buffer, g_settings_json.data(), n);
		out_buffer[n] = 0;
	}
	return full;
}

extern "C" uint64_t alvr_hud_message_bounded(char *out_buffer, uint64_t cap) {
	if (cap > 0)
		out_buffer[0] = 0;
	return 0;
}

extern "C" uint64_t alvr_get_decoder_config_bounded(char *out_buffer, uint64_t cap) {
	if (cap > 0)
		out_buffer[0] = 0;
	return 0;
}

extern "C" void alvr_send_eye_openness(float, float) {}
extern "C" void alvr_set_eq_gains(const float *, int) {}
extern "C" void alvr_set_eq_enabled(bool) {}
extern "C" bool alvr_get_client_stats(float *) { return false; }
extern "C" bool alvr_get_frame_timeout(uint64_t *, void **, uint64_t) { return false; }
extern "C" void alvr_set_decoder_paused(bool) {}
