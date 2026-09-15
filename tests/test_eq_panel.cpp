// ui/eq_panel.cpp: coordinate helpers, preset switching and geometry build.
#include "test_framework.h"
#include "eq_panel.h"
#include "app_state.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

namespace
{
// applyEqPreset() persists via saveAllConfig(); give it a scratch HOME.
struct HomeFix {
	char old[512];
	bool had = false;
	HomeFix()
	{
		const char *h = getenv("HOME");
		if (h) {
			had = true;
			strncpy(old, h, sizeof(old) - 1);
			old[sizeof(old) - 1] = 0;
		}
		char dir[] = "/tmp/p2weq_XXXXXX";
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
};
} // namespace

TEST(eq, col_centers_spread_across_panel) {
	float first = eqColCenterX(0);
	float last = eqColCenterX(kEqBands - 1);
	CHECK(first > kEqX0);
	CHECK(last < kEqX1);
	CHECK(std::abs((first + last) * 0.5f - (kEqX0 + kEqX1) * 0.5f) < 1e-6);
	// Even spacing.
	float step = (kEqX1 - kEqX0) / kEqBands;
	CHECK(std::abs(eqColCenterX(3) - eqColCenterX(2) - step) < 1e-6);
}

TEST(eq, y_to_gain_mapping_and_clamp) {
	CHECK(std::abs(eqYToGain(kEqYTrackBot) + kEqGainMax) < 1e-5);
	CHECK(std::abs(eqYToGain(kEqYTrackTop) - kEqGainMax) < 1e-5);
	CHECK(std::abs(eqYToGain((kEqYTrackTop + kEqYTrackBot) * 0.5f)) < 1e-5);
	// Outside the track clamps.
	CHECK(std::abs(eqYToGain(kEqYTrackTop + 1.0f) - kEqGainMax) < 1e-5);
	CHECK(std::abs(eqYToGain(kEqYTrackBot - 1.0f) + kEqGainMax) < 1e-5);
}

TEST(eq, gain_to_y_roundtrip) {
	for (float g = -12.0f; g <= 12.0f; g += 1.5f)
		CHECK(std::abs(eqYToGain(eqGainToY(g)) - g) < 1e-4);
	// Out-of-range gains clamp back into the track.
	CHECK(std::abs(eqGainToY(99.0f) - kEqYTrackTop) < 1e-6);
	CHECK(std::abs(eqGainToY(-99.0f) - kEqYTrackBot) < 1e-6);
}

TEST(eq, apply_preset_loads_curve) {
	HomeFix h;
	for (int i = 0; i < kEqBands; i++) {
		gEqCustoms[0][i] = 1.0f;
		gEqCustoms[1][i] = -2.0f;
		gEqGains[i] = 0.0f;
	}
	applyEqPreset(1);
	CHECK_EQ(gEqPresetIdx, 1);
	for (int i = 0; i < kEqBands; i++)
		CHECK(std::abs(gEqGains[i] + 2.0f) < 1e-6);

	applyEqPreset(0);
	CHECK_EQ(gEqPresetIdx, 0);
	CHECK(std::abs(gEqGains[5] - 1.0f) < 1e-6);

	// Out-of-range index is a no-op.
	applyEqPreset(-1);
	applyEqPreset(kEqNumPresets);
	CHECK_EQ(gEqPresetIdx, 0);
}

TEST(eq, build_verts_emits_geometry) {
	HomeFix h;
	for (int i = 0; i < kEqBands; i++)
		gEqGains[i] = (i % 2) ? 3.0f : -3.0f; // mix boost/cut colours
	gEqActiveBand.store(3);

	std::vector<float> v;
	buildEqVerts(v, 3, true, true, -1);
	CHECK(!v.empty());
	CHECK_EQ(v.size() % 6, 0u); // vertex records are 6 floats

	// Closed vs open dropdown: open emits the item rows.
	size_t closed = v.size();
	gEqPresetOpen = true;
	v.clear();
	buildEqVerts(v, -1, false, false, 1);
	CHECK(v.size() > closed);
	gEqPresetOpen = false;

	// No readout when nothing hovered/active.
	gEqActiveBand.store(-1);
	v.clear();
	buildEqVerts(v, -1, false, false, -1);
	CHECK(!v.empty());
}
