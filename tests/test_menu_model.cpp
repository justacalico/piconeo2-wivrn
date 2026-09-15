// render/menu_model.cpp: row metrics, hit testing, input application.
#include "test_framework.h"
#include "menu_model.h"

#include <cmath>
#include <thread>
#include <chrono>

namespace
{
MenuItem toggle_item(float & v)
{
	MenuItem it;
	it.kind = MK_TOGGLE;
	it.label = "tog";
	it.get = [&] { return v; };
	it.set = [&](float x) { v = x; };
	return it;
}
} // namespace

TEST(menu, row_height_per_kind) {
	MenuItem it;
	it.kind = MK_TOGGLE;
	CHECK(std::abs(menuRowHeight(it) - 0.130f) < 1e-6);
	it.kind = MK_BUTTON;
	CHECK(std::abs(menuRowHeight(it) - 0.130f) < 1e-6);
	it.kind = MK_FADER;
	CHECK(std::abs(menuRowHeight(it) - 0.160f) < 1e-6);
	it.kind = MK_STEPPER;
	CHECK(std::abs(menuRowHeight(it) - 0.210f) < 1e-6);
	it.kind = MK_DROPDOWN;
	it.options = {"a", "b"};
	it.dropOpen = false;
	float closed = menuRowHeight(it);
	it.dropOpen = true;
	float open = menuRowHeight(it);
	CHECK(open > closed);
	CHECK(std::abs((open - closed) - 2 * 0.060f) < 1e-6);
	it.kind = MK_CUSTOM;
	it.customH = 0.5f;
	CHECK(std::abs(menuRowHeight(it) - 0.52f) < 1e-6);
}

TEST(menu, row_top_stacks_downward) {
	MenuCategory c;
	float v = 0;
	c.items.push_back(toggle_item(v));
	c.items.push_back(toggle_item(v));
	CHECK(std::abs(menuRowTop(c, 0) - kMenuTopY) < 1e-6);
	CHECK(menuRowTop(c, 1) < menuRowTop(c, 0));
	// Out-of-range index just sums all preceding heights.
	CHECK(menuRowTop(c, 5) <= menuRowTop(c, 1));
}

TEST(menu, content_height_sums_rows) {
	MenuCategory c;
	float v = 0;
	c.items.push_back(toggle_item(v));
	c.items.push_back(toggle_item(v));
	CHECK(std::abs(menuContentH(c) - 2 * menuRowHeight(c.items[0])) < 1e-6);
}

TEST(menu, hit_toggle_band) {
	MenuCategory c;
	float v = 0;
	c.items.push_back(toggle_item(v));
	// Row 0 toggle rect: centre (0, 0.30 - 0.0375).
	MenuHover h = menuHit(c, 0.0f, 0.2625f);
	CHECK_EQ(h.item, 0);
	CHECK(h.grab);
	// Outside the band.
	h = menuHit(c, 0.0f, 0.0f);
	CHECK_EQ(h.item, -1);
	CHECK(!h.grab);
}

TEST(menu, hit_skips_disabled) {
	MenuCategory c;
	float v = 0;
	c.items.push_back(toggle_item(v));
	c.items[0].disabled = true;
	MenuHover h = menuHit(c, 0.0f, 0.2625f);
	CHECK_EQ(h.item, -1);
}

TEST(menu, hit_button_and_fader) {
	MenuCategory c;
	float v = 0;
	MenuItem b;
	b.kind = MK_BUTTON;
	c.items.push_back(b);
	MenuItem f;
	f.kind = MK_FADER;
	f.get = [&] { return v; };
	f.set = [&](float x) { v = x; };
	c.items.push_back(f);

	float yTop0 = menuRowTop(c, 0);
	// Button rect centre: (0, yTop - 0.035).
	MenuHover h = menuHit(c, 0.0f, yTop0 - 0.035f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 0);
	CHECK(h.grab);

	float yTop1 = menuRowTop(c, 1);
	// Fader rect centre: (0, yTop - 0.105).
	h = menuHit(c, 0.0f, yTop1 - 0.105f);
	CHECK_EQ(h.item, 1);
	CHECK(h.grab);
}

TEST(menu, custom_item_inside_regular_category) {
	// A MK_CUSTOM row in a non-custom category delegates hit/act per-item.
	MenuCategory c;
	float v = 0;
	c.items.push_back(toggle_item(v));
	MenuItem cu;
	cu.kind = MK_CUSTOM;
	cu.customH = 0.2f;
	int hits = 0, acts = 0, builds = 0;
	cu.cHit = [&](float, float, MenuHover & h) { ++hits; h.item = 1; h.grab = true; };
	cu.cAct = [&](const MenuHover &, bool, bool, float, float) { ++acts; };
	cu.cBuild = [&](std::vector<float> &, const MenuHover &) { ++builds; };
	c.items.push_back(cu);

	// Hit inside the custom row's band (below the toggle).
	float yTop1 = menuRowTop(c, 1);
	MenuHover h = menuHit(c, 0.0f, yTop1 - 0.05f);
	CHECK_EQ(hits, 1);
	CHECK_EQ(h.item, 1);
	// cAct only runs for whole-custom categories; here apply is a no-op.
	menuApply(9, c, h, true, true, 0, 0);
	CHECK_EQ(acts, 0);

	std::vector<float> buf;
	menuBuild(buf, c, MenuHover{});
	CHECK_EQ(builds, 1);
}

TEST(menu, hit_stepper_parts) {
	MenuCategory c;
	float v = 5;
	MenuItem it;
	it.kind = MK_STEPPER;
	it.get = [&] { return v; };
	it.set = [&](float x) { v = x; };
	c.items.push_back(it);
	float yTop = menuRowTop(c, 0);
	// minus button centre (-0.20, yTop - 0.150)
	MenuHover h = menuHit(c, -0.20f, yTop - 0.150f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 0);
	// plus button centre (+0.20, yTop - 0.150)
	h = menuHit(c, 0.20f, yTop - 0.150f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 1);
}

TEST(menu, apply_toggle_flips_and_notifies) {
	MenuCategory c;
	float v = 0;
	int changes = 0;
	MenuItem it = toggle_item(v);
	it.onChange = [&] { ++changes; };
	c.items.push_back(it);

	MenuHover h{.item = 0, .part = 0, .grab = true};
	menuApply(0, c, h, true, true, 0, 0);
	CHECK(v == 1.0f);
	CHECK_EQ(changes, 1);
	menuApply(0, c, h, true, true, 0, 0);
	CHECK(v == 0.0f);
	CHECK_EQ(changes, 2);
	// No click edge: nothing happens.
	menuApply(0, c, h, false, true, 0, 0);
	CHECK(v == 0.0f);
	CHECK_EQ(changes, 2);
}

TEST(menu, apply_button_click) {
	MenuCategory c;
	int clicks = 0;
	MenuItem it;
	it.kind = MK_BUTTON;
	it.onClick = [&] { ++clicks; };
	c.items.push_back(it);
	MenuHover h{.item = 0, .part = 0, .grab = true};
	menuApply(0, c, h, true, true, 0, 0);
	CHECK_EQ(clicks, 1);
	menuApply(0, c, h, false, true, 0, 0);
	CHECK_EQ(clicks, 1);
}

TEST(menu, apply_stepper_steps_and_clamps) {
	MenuCategory c;
	float v = 5;
	int changes = 0;
	MenuItem it;
	it.kind = MK_STEPPER;
	it.vmin = 0;
	it.vmax = 10;
	it.vstep = 2;
	it.get = [&] { return v; };
	it.set = [&](float x) { v = x; };
	it.onChange = [&] { ++changes; };
	c.items.push_back(it);

	menuApply(1, c, MenuHover{0, 1, true}, true, true, 0, 0); // plus click
	CHECK(v == 7.0f);
	CHECK_EQ(changes, 1);
	menuApply(1, c, MenuHover{0, 0, true}, true, true, 0, 0); // minus click
	CHECK(v == 5.0f);

	// Clamp at vmax.
	v = 9.5f;
	menuApply(1, c, MenuHover{0, 1, true}, true, true, 0, 0);
	CHECK(v == 10.0f);
	// Clamp at vmin.
	v = 0.5f;
	menuApply(1, c, MenuHover{0, 0, true}, true, true, 0, 0);
	CHECK(v == 0.0f);
}

TEST(menu, apply_stepper_hold_repeats) {
	MenuCategory c;
	float v = 0;
	MenuItem it;
	it.kind = MK_STEPPER;
	it.vmin = 0;
	it.vmax = 100;
	it.vstep = 1;
	it.get = [&] { return v; };
	it.set = [&](float x) { v = x; };
	c.items.push_back(it);

	// Press starts a 400ms repeat timer.
	menuApply(7, c, MenuHover{0, 1, true}, true, true, 0, 0);
	CHECK(v == 1.0f);
	// Holding within the delay does not repeat.
	menuApply(7, c, MenuHover{0, 1, true}, false, true, 0, 0);
	CHECK(v == 1.0f);
	// After the initial delay, holding steps again.
	std::this_thread::sleep_for(std::chrono::milliseconds(450));
	menuApply(7, c, MenuHover{0, 1, true}, false, true, 0, 0);
	CHECK(v == 2.0f);
	// Releasing clears the hold state.
	menuApply(7, c, MenuHover{0, 1, false}, false, false, 0, 0);
	menuApply(7, c, MenuHover{-1, 0, false}, false, false, 0, 0);
	CHECK(v == 2.0f);
}

TEST(menu, apply_fader_drag_and_commit) {
	MenuCategory c;
	float v = 0;
	int changes = 0, commits = 0;
	MenuItem it;
	it.kind = MK_FADER;
	it.vmin = 0;
	it.vmax = 100;
	it.get = [&] { return v; };
	it.set = [&](float x) { v = x; };
	it.onChange = [&] { ++changes; };
	it.onCommit = [&] { ++commits; };
	c.items.push_back(it);

	// Drag to the right edge -> frac 1.0 -> v = 100.
	menuApply(3, c, MenuHover{0, 0, true}, true, true, 0.40f, 0);
	CHECK(v == 100.0f);
	CHECK_EQ(changes, 1);
	// Release commits.
	menuApply(3, c, MenuHover{0, 0, false}, false, false, 0.40f, 0);
	CHECK_EQ(commits, 1);
	// Drag to left edge.
	menuApply(3, c, MenuHover{0, 0, true}, true, true, -0.40f, 0);
	CHECK(v == 0.0f);
	menuApply(3, c, MenuHover{0, 0, false}, false, false, -0.40f, 0);
	CHECK_EQ(commits, 2);
}

TEST(menu, apply_dropdown_open_select_close) {
	MenuCategory c;
	float sel = 0;
	int changes = 0;
	MenuItem it;
	it.kind = MK_DROPDOWN;
	it.options = {"x", "y", "z"};
	it.get = [&] { return sel; };
	it.set = [&](float x) { sel = x; };
	it.onChange = [&] { ++changes; };
	c.items.push_back(it);

	// Click header toggles open.
	menuApply(4, c, MenuHover{0, 0, true}, true, true, 0, 0);
	CHECK(c.items[0].dropOpen);
	// Click option 2 selects it and closes.
	menuApply(4, c, MenuHover{0, 102, true}, true, true, 0, 0);
	CHECK(sel == 2.0f);
	CHECK(!c.items[0].dropOpen);
	CHECK_EQ(changes, 1);
	// Header again reopens and re-closes.
	menuApply(4, c, MenuHover{0, 0, true}, true, true, 0, 0);
	CHECK(c.items[0].dropOpen);
	menuApply(4, c, MenuHover{0, 0, true}, true, true, 0, 0);
	CHECK(!c.items[0].dropOpen);
}

TEST(menu, hit_dropdown_items) {
	MenuCategory c;
	float sel = 0;
	MenuItem it;
	it.kind = MK_DROPDOWN;
	it.options = {"a", "b"};
	it.dropOpen = true;
	it.get = [&] { return sel; };
	it.set = [&](float x) { sel = x; };
	c.items.push_back(it);
	float yTop = menuRowTop(c, 0);
	// Option 0 centre: yTop - 0.055 - 0.075 - 0.060*0.5
	MenuHover h = menuHit(c, 0.0f, yTop - 0.055f - 0.075f - 0.030f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 100);
	// Header hit.
	h = menuHit(c, 0.0f, yTop - 0.055f - 0.0375f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 0);
	// Closed dropdown doesn't report option hits.
	c.items[0].dropOpen = false;
	h = menuHit(c, 0.0f, yTop - 0.055f - 0.075f - 0.030f);
	CHECK_EQ(h.item, -1);
}

TEST(menu, apply_ignores_out_of_range_hover) {
	MenuCategory c;
	float v = 0;
	c.items.push_back(toggle_item(v));
	menuApply(0, c, MenuHover{5, 0, true}, true, true, 0, 0);
	CHECK(v == 0.0f);
	menuApply(0, c, MenuHover{-1, 0, false}, true, false, 0, 0);
	CHECK(v == 0.0f);
}

TEST(menu, apply_disabled_ignores_input) {
	MenuCategory c;
	float v = 0;
	MenuItem it = toggle_item(v);
	it.disabled = true;
	c.items.push_back(it);
	menuApply(0, c, MenuHover{0, 0, true}, true, true, 0, 0);
	CHECK(v == 0.0f);
}

TEST(menu, custom_category_delegates) {
	MenuCategory c;
	c.custom = true;
	MenuItem it;
	it.kind = MK_CUSTOM;
	int built = 0, hits = 0, acts = 0;
	it.cBuild = [&](std::vector<float> &, const MenuHover &) { ++built; };
	it.cHit = [&](float, float, MenuHover & h) { ++hits; h.item = 0; };
	it.cAct = [&](const MenuHover &, bool, bool, float, float) { ++acts; };
	c.items.push_back(it);

	std::vector<float> vbuf;
	menuBuild(vbuf, c, MenuHover{});
	CHECK_EQ(built, 1);
	MenuHover h = menuHit(c, 0, 0);
	CHECK_EQ(hits, 1);
	CHECK_EQ(h.item, 0);
	menuApply(0, c, h, true, true, 0, 0);
	CHECK_EQ(acts, 1);
}

TEST(menu, value_sig_tracks_values) {
	MenuCategory c;
	float a = 0, b = 1;
	c.items.push_back(toggle_item(a));
	c.items.push_back(toggle_item(b));
	unsigned s1 = menuValueSig(c);
	CHECK_EQ(menuValueSig(c), s1); // deterministic
	a = 1;
	CHECK(menuValueSig(c) != s1); // value change flips the sig
}

TEST(menu, build_runs_without_crash) {
	// Exercises every build branch including disabled and open dropdown.
	MenuCategory c;
	float v = 0.5f, sel = 1;
	MenuItem t = toggle_item(v);
	c.items.push_back(t);
	MenuItem b;
	b.kind = MK_BUTTON;
	c.items.push_back(b);
	MenuItem f;
	f.kind = MK_FADER;
	f.get = [&] { return v; };
	f.valueText = [](char * buf, int n) { snprintf(buf, (size_t)n, "50%%"); };
	c.items.push_back(f);
	MenuItem f2; // fader without a valueText: plain-label branch
	f2.kind = MK_FADER;
	f2.get = [&] { return v; };
	c.items.push_back(f2);
	MenuItem s;
	s.kind = MK_STEPPER;
	s.get = [&] { return v; };
	c.items.push_back(s);
	MenuItem d;
	d.kind = MK_DROPDOWN;
	d.options = {"o1", "o2"};
	d.dropOpen = true;
	d.get = [&] { return sel; };
	c.items.push_back(d);
	MenuItem dis = toggle_item(v);
	dis.disabled = true;
	dis.kind = MK_STEPPER;
	c.items.push_back(dis);

	std::vector<float> vbuf;
	menuBuild(vbuf, c, MenuHover{2, 0, true});
	CHECK(!vbuf.empty());
	size_t n = vbuf.size();
	vbuf.clear();
	menuBuild(vbuf, c, MenuHover{-1, 0, false});
	CHECK_EQ(vbuf.size(), n); // same geometry regardless of hover
}
