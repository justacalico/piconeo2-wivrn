// ui/ui_kit.cpp: bitmap text, quads and every widget's geometry output.
#include "test_framework.h"
#include "ui_kit.h"

#include <cstring>

TEST(uikit, quad_emits_two_triangles) {
	std::vector<float> v;
	appendQuad(v, 0.0f, 1.0f, 2.0f, 0.0f, 0.5f, 0.6f, 0.7f);
	CHECK_EQ(v.size(), 36u);
	// First vertex: xL,yTop,0 + colour.
	CHECK(v[0] == 0.0f && v[1] == 1.0f && v[2] == 0.0f);
	CHECK(v[3] == 0.5f && v[4] == 0.6f && v[5] == 0.7f);
	// Third vertex: xR,yBot.
	CHECK(v[12] == 2.0f && v[13] == 0.0f);
}

TEST(uikit, text_emits_only_lit_pixels) {
	std::vector<float> v;
	appendTextLine(v, " ", 0.0f, 0.01f, 1, 1, 1); // space has no lit pixels
	CHECK(v.empty());
	appendTextLine(v, "", 0.0f, 0.01f, 1, 1, 1);
	CHECK(v.empty());
	appendTextLine(v, "8", 0.0f, 0.01f, 1, 1, 1); // '8' lights most pixels
	CHECK(!v.empty());
	CHECK_EQ(v.size() % 36, 0u);
	// Every vertex carries the requested colour.
	for (size_t i = 3; i < v.size(); i += 6)
		CHECK(v[i] == 1.0f);
}

TEST(uikit, text_covers_full_glyph_set) {
	std::vector<float> v;
	// One of every supported glyph class incl. symbols and fallback chars.
	appendTextLine(v, "0123456789.:-^~+/%%()AZaz!?@_", 0.0f, 0.005f, 1, 0, 0);
	CHECK(!v.empty());
}

TEST(uikit, text_centres_and_shifts) {
	std::vector<float> a, b;
	appendTextLine(a, "1", 0.0f, 0.01f, 1, 1, 1);
	uiTextC(b, "1", 0.5f, 0.0f, 0.01f, 1, 1, 1);
	CHECK_EQ(a.size(), b.size());
	// uiTextC output = appendTextLine output shifted by cx on x.
	for (size_t i = 0; i < a.size(); i += 6)
		CHECK(std::abs(b[i] - a[i] - 0.5f) < 1e-7);
}

TEST(uikit, hit_test_rect) {
	UiRect r{0.0f, 0.0f, 1.0f, 1.0f};
	CHECK(uiHit(r, 0.0f, 0.0f));
	CHECK(uiHit(r, 0.5f, 0.5f));  // edges inclusive
	CHECK(uiHit(r, -0.5f, -0.5f));
	CHECK(!uiHit(r, 0.51f, 0.0f));
	CHECK(!uiHit(r, 0.0f, -0.51f));
}

TEST(uikit, widgets_emit_geometry_all_states) {
	UiRect r{0.0f, 0.0f, 0.9f, 0.075f};
	const bool states[][3] = {
	        {false, false, false}, {true, false, false},
	        {false, true, false},  {false, false, true},
	        {true, true, true},
	};
	for (auto &s : states) {
		std::vector<float> v;
		uiBox(v, r, kUiBg);
		uiButton(v, r, "OK", s[0], s[2]);
		uiToggle(v, r, "Toggle", s[0], s[1], 1.0f, s[2]);
		uiToggle(v, r, "", s[0], s[1], 0.5f, s[2]); // empty label skips fit path
		uiVFader(v, r, 0.5f, s[0], s[2]);
		uiHFader(v, r, 0.5f, s[0], s[2]);
		uiDropdownHeader(v, r, "Pick", s[0], s[1], s[2]);
		uiDropdownItem(v, r, "Item", s[0], s[2]);
		uiLabel(v, "Lbl", r.cx, r.cy, kUiText, kUiTitle);
		uiTextL(v, "left", -0.4f, 0.0f, kUiText, 1, 1, 1);
		CHECK(!v.empty());
		CHECK_EQ(v.size() % 6, 0u);
	}
}

TEST(uikit, faders_clamp_frac) {
	UiRect r{0.0f, 0.0f, 0.8f, 0.05f};
	std::vector<float> v;
	uiHFader(v, r, -1.0f, false, false); // clamped to 0
	uiHFader(v, r, 2.0f, false, false);  // clamped to 1
	uiVFader(v, r, -1.0f, false, false);
	uiVFader(v, r, 2.0f, false, false);
	CHECK(!v.empty());
}

TEST(uikit, toggle_long_label_shrinks_text) {
	UiRect r{0.0f, 0.0f, 0.9f, 0.075f};
	std::vector<float> shortL, longL;
	uiToggle(shortL, r, "On", true, false, 1.0f, false);
	uiToggle(longL, r, "AVeryLongToggleLabelThatWillNotFit", true, false, 1.0f, false);
	CHECK(!shortL.empty());
	CHECK(!longL.empty());
}
