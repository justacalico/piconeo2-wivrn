// core/app_state.cpp: PIN flow, unified config.txt persistence, legacy
// migration and value clamping on load.
#include "test_framework.h"
#include "app_state.h"
#include "eq_panel.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

namespace
{
// Redirects $HOME to a fresh temp dir for the duration of a test.
struct HomeFix {
	char old[512];
	bool had = false;
	char dir[128];
	HomeFix()
	{
		const char *h = getenv("HOME");
		if (h) {
			had = true;
			strncpy(old, h, sizeof(old) - 1);
			old[sizeof(old) - 1] = 0;
		}
		strcpy(dir, "/tmp/p2wcfg_XXXXXX");
		mkdtemp(dir);
		setenv("HOME", dir, 1);
	}
	~HomeFix()
	{
		if (had)
			setenv("HOME", old, 1);
		else
			unsetenv("HOME");
	}
	std::string path(const char *name) { return std::string(dir) + "/" + name; }
	void write(const char *name, const std::string &contents)
	{
		FILE *f = fopen(path(name).c_str(), "w");
		fwrite(contents.data(), 1, contents.size(), f);
		fclose(f);
	}
	bool exists(const char *name) { return access(path(name).c_str(), F_OK) == 0; }
	std::string slurp(const char *name)
	{
		FILE *f = fopen(path(name).c_str(), "r");
		if (!f)
			return {};
		std::string s;
		char buf[1024];
		size_t n;
		while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
			s.append(buf, n);
		fclose(f);
		return s;
	}
};
} // namespace

TEST(appstate, pin_flow) {
	gOnPinSubmit = nullptr;
	gPinEntryRequested.store(false);
	requestPinEntryUI();
	CHECK(gPinEntryRequested.load());

	std::string got;
	gOnPinSubmit = [&](const std::string &p) { got = p; };
	submitPin("4321");
	CHECK(!gPinEntryRequested.load());
	CHECK_EQ(got, "4321");

	// No callback registered: still clears the flag.
	gOnPinSubmit = nullptr;
	gPinEntryRequested.store(true);
	submitPin("0000");
	CHECK(!gPinEntryRequested.load());
}

TEST(appstate, soft_ipd_helper) {
	gSoftIpdMm.store(65.0f);
	CHECK(std::abs(softIpdM() - 0.065f) < 1e-6);
}

TEST(appstate, save_writes_key_value_file) {
	HomeFix h;
	gSoftIpdMm.store(66.5f);
	gEyeDebugOn.store(true);
	gDiagHudMode.store(2);
	gBrightnessFrac.store(0.5f);
	gBrightnessSaved.store(true);
	gEqPresetIdx = 1;
	for (int i = 0; i < kEqBands; i++) {
		gEqCustoms[0][i] = (float)i;
		gEqCustoms[1][i] = (float)-i;
	}
	gStreamFovDeg.store(90.0f);
	gWivrnTcpOnly.store(true);
	gWivrnResolutionScale.store(1.5f);
	gWivrnBitrateMbps.store(120.0f);
	gWivrnMicrophone.store(true);
	gWivrnCtrlVibration.store(0.5f);
	gWivrnEyeTracking.store(false);
	gWivrnPassthrough.store(true);
	gWivrnEyeFoveation.store(false);
	gFoveationEnabled.store(false);

	saveAllConfig();
	std::string cfg = h.slurp("config.txt");
	CHECK(cfg.find("version=2\n") == 0);
	CHECK(cfg.find("softIpd=66.50") != std::string::npos);
	CHECK(cfg.find("eyeDebug=1") != std::string::npos);
	CHECK(cfg.find("wivrnTcpOnly=1") != std::string::npos);
}

TEST(appstate, save_then_load_roundtrip) {
	HomeFix h;
	gSoftIpdMm.store(61.0f);
	gEyeDebugOn.store(true);
	gDiagHudMode.store(1);
	gBrightnessFrac.store(0.25f);
	gBrightnessSaved.store(true);
	gEqPresetIdx = 1;
	for (int i = 0; i < kEqBands; i++)
		gEqCustoms[1][i] = 3.0f;
	gStreamFovDeg.store(80.0f);
	gWivrnTcpOnly.store(true);
	gWivrnResolutionScale.store(1.25f);
	gWivrnBitrateMbps.store(77.0f);
	gWivrnMicrophone.store(true);
	gWivrnCtrlVibration.store(0.3f);
	gWivrnEyeTracking.store(false);
	gWivrnPassthrough.store(true);
	gWivrnEyeFoveation.store(false);
	gFoveationEnabled.store(false);
	saveAllConfig();

	// Scramble everything, then reload.
	gSoftIpdMm.store(65.0f);
	gEyeDebugOn.store(false);
	gDiagHudMode.store(0);
	gBrightnessFrac.store(1.0f);
	gBrightnessSaved.store(false);
	gEqPresetIdx = 0;
	memset(gEqCustoms, 0, sizeof(gEqCustoms));
	gStreamFovDeg.store(101.0f);
	gWivrnTcpOnly.store(false);
	gWivrnResolutionScale.store(1.0f);
	gWivrnBitrateMbps.store(50.0f);
	gWivrnMicrophone.store(false);
	gWivrnCtrlVibration.store(1.0f);
	gWivrnEyeTracking.store(true);
	gWivrnPassthrough.store(false);
	gWivrnEyeFoveation.store(true);
	gFoveationEnabled.store(true);
	memset(gEqGains, 0, sizeof(gEqGains));

	loadAllConfig();
	CHECK(std::abs(gSoftIpdMm.load() - 61.0f) < 0.01f);
	CHECK(gEyeDebugOn.load());
	CHECK_EQ(gDiagHudMode.load(), 1);
	CHECK(std::abs(gBrightnessFrac.load() - 0.25f) < 0.001f);
	CHECK(gBrightnessSaved.load());
	CHECK_EQ(gEqPresetIdx, 1);
	CHECK(std::abs(gEqCustoms[1][0] - 3.0f) < 0.01f);
	// Live gains mirror the active slot.
	CHECK(std::abs(gEqGains[0] - 3.0f) < 0.01f);
	CHECK(std::abs(gStreamFovDeg.load() - 80.0f) < 0.01f);
	CHECK(gWivrnTcpOnly.load());
	CHECK(std::abs(gWivrnResolutionScale.load() - 1.25f) < 0.01f);
	CHECK(std::abs(gWivrnBitrateMbps.load() - 77.0f) < 0.5f);
	CHECK(gWivrnMicrophone.load());
	CHECK(std::abs(gWivrnCtrlVibration.load() - 0.3f) < 0.01f);
	CHECK(!gWivrnEyeTracking.load());
	CHECK(gWivrnPassthrough.load());
	CHECK(!gWivrnEyeFoveation.load());
	CHECK(!gFoveationEnabled.load());
}

TEST(appstate, load_clamps_out_of_range) {
	HomeFix h;
	h.write("config.txt",
	        "version=2\n"
	        "softIpd=999.0\n"          // clamps to 72
	        "diagHud=9\n"             // clamps to 2
	        "streamFov=200.0\n"       // clamps to 101
	        "wivrnBitrateMbps=99999\n"// clamps to 200
	        "wivrnResolutionScale=9\n"// clamps to 2.0
	        "wivrnCtrlVibration=7\n"  // clamps to 1.0
	        "eqPreset=99\n"           // clamps to 0
	        "eqCustom1=99 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n"
	        "eqCustom2=\n"            // empty list: leaves defaults
	        "unknownKey=hello\n"      // skipped
	        "garbage line no equals\n");
	loadAllConfig();
	CHECK(std::abs(gSoftIpdMm.load() - kIpdMax) < 0.01f);
	CHECK_EQ(gDiagHudMode.load(), 2);
	CHECK(std::abs(gStreamFovDeg.load() - kFovMax) < 0.01f);
	CHECK(std::abs(gWivrnBitrateMbps.load() - 200.0f) < 0.01f);
	CHECK(std::abs(gWivrnResolutionScale.load() - 2.0f) < 0.01f);
	CHECK(std::abs(gWivrnCtrlVibration.load() - 1.0f) < 0.01f);
	CHECK_EQ(gEqPresetIdx, 0);
	CHECK(std::abs(gEqCustoms[0][0] - kEqGainMax) < 0.01f); // 99 -> clamped to +12
}

TEST(appstate, load_negative_brightness_ignored) {
	HomeFix h;
	gBrightnessSaved.store(false);
	h.write("config.txt",
	        "version=2\n"
	        "brightness=-1.0\n"); // sentinel for "never saved"
	loadAllConfig();
	CHECK(!gBrightnessSaved.load());
}

TEST(appstate, missing_config_uses_defaults) {
	HomeFix h;
	gSoftIpdMm.store(60.0f); // non-default; load should leave it alone
	loadAllConfig();
	CHECK(std::abs(gSoftIpdMm.load() - 60.0f) < 0.01f);
	CHECK(!h.exists("config.txt")); // nothing migrated, nothing written
}

TEST(appstate, legacy_positional_migration) {
	HomeFix h;
	std::string legacy =
	        "63.5\n"   // softIpd
	        "1\n"      // eyeDebug
	        "2\n"      // diagHud
	        "0.5\n"    // brightness
	        "1\n"      // eqPreset
	        "0.5 1 1.5 2 2.5 3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8\n" // custom1
	        "-8 -7.5 -7 -6.5 -6 -5.5 -5 -4.5 -4 -3.5 -3 -2.5 -2 -1.5 -1 -0.5\n" // custom2
	        "85.0\n"   // streamFov
	        "1\n"      // tcpOnly
	        "1.5\n"    // resScale
	        "100\n"    // bitrate
	        "1\n"      // mic
	        "0.5\n"    // vibration
	        "0\n"      // eyeTracking
	        "1\n"      // passthrough
	        "0\n"      // eyeFoveation
	        "0\n";     // foveationEnabled
	h.write("config.txt", legacy);
	loadAllConfig();
	CHECK(std::abs(gSoftIpdMm.load() - 63.5f) < 0.01f);
	CHECK(gEyeDebugOn.load());
	CHECK_EQ(gDiagHudMode.load(), 2);
	CHECK(std::abs(gBrightnessFrac.load() - 0.5f) < 0.01f);
	CHECK(gBrightnessSaved.load());
	CHECK_EQ(gEqPresetIdx, 1);
	CHECK(std::abs(gEqCustoms[1][15] + 0.5f) < 0.01f);
	CHECK(std::abs(gStreamFovDeg.load() - 85.0f) < 0.01f);
	CHECK(gWivrnTcpOnly.load());
	CHECK(std::abs(gWivrnBitrateMbps.load() - 100.0f) < 0.01f);
	CHECK(!gFoveationEnabled.load());
	// The file was rewritten in key=value format.
	CHECK(h.slurp("config.txt").find("version=2") == 0);
}

TEST(appstate, legacy_per_file_migration) {
	HomeFix h;
	h.write("software_ipd.txt", "70.0");
	h.write("eye_debug.txt", "1");
	h.write("diag_hud.txt", "1");
	h.write("brightness.txt", "0.8");
	h.write("eq_profile.txt",
	        "1\n"
	        "1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n"
	        "2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2\n");
	h.write("theme.txt", "dead"); // leftover from a removed feature
	loadAllConfig();
	CHECK(std::abs(gSoftIpdMm.load() - 70.0f) < 0.01f);
	CHECK(gEyeDebugOn.load());
	CHECK_EQ(gDiagHudMode.load(), 1);
	CHECK(std::abs(gBrightnessFrac.load() - 0.8f) < 0.01f);
	CHECK(gBrightnessSaved.load());
	CHECK_EQ(gEqPresetIdx, 1);
	CHECK(std::abs(gEqCustoms[1][3] - 2.0f) < 0.01f);
	// All legacy files swept, unified file written.
	CHECK(!h.exists("software_ipd.txt"));
	CHECK(!h.exists("theme.txt"));
	CHECK(h.slurp("config.txt").find("version=2") == 0);
}

TEST(appstate, empty_config_uses_defaults) {
	HomeFix h;
	h.write("config.txt", "");
	loadAllConfig(); // must not crash
	CHECK(true);
}

TEST(appstate, save_wrappers_all_write) {
	HomeFix h;
	gBrightnessFrac.store(0.6f);
	saveBrightness();
	CHECK(gBrightnessSaved.load());
	CHECK(h.exists("config.txt"));
	remove(h.path("config.txt").c_str());
	saveSoftIpd();
	CHECK(h.exists("config.txt"));
	remove(h.path("config.txt").c_str());
	saveEyeDebug();
	CHECK(h.exists("config.txt"));
	remove(h.path("config.txt").c_str());
	saveDiagHud();
	CHECK(h.exists("config.txt"));
	remove(h.path("config.txt").c_str());
	saveStreamFov();
	CHECK(h.exists("config.txt"));
}

TEST(appstate, no_home_is_safe) {
	HomeFix h;
	unsetenv("HOME");
	saveAllConfig(); // must not crash or write anywhere
	loadAllConfig();
	CHECK(true);
}
