// ui/server_list.cpp: server store, connection state, layout, hit test, clicks.
#include "test_framework.h"
#include "server_list.h"
#include "app_state.h"
#include "panel_geometry.h"

#include <cmath>

// Defined in stubs.cpp; observe what the UI pushers received.
extern std::vector<ServerInfo> gLastPushedServers;
extern int gPushServersCalls;
extern std::string gLastConnError;
extern int gPushConnErrorCalls;
extern int gLastConnecting;
extern int gPushConnectingCalls;

namespace
{
ServerInfo srv(const char *name, const char *host, int port, bool manual = false)
{
	ServerInfo s;
	s.name = name;
	s.hostname = host;
	s.port = port;
	s.manual = manual;
	return s;
}

struct Reset {
	Reset()
	{
		setServerList({});
		clearConnectionError();
		setConnecting(false);
		gOnServerConnect = nullptr;
		gOnServerRemove = nullptr;
		gOnServerAutoconnect = nullptr;
		gOnRefreshServers = nullptr;
		gOnConnectingChanged = nullptr;
	}
	~Reset()
	{
		setServerList({});
		clearConnectionError();
		setConnecting(false);
		gOnServerConnect = nullptr;
		gOnServerRemove = nullptr;
		gOnServerAutoconnect = nullptr;
		gOnRefreshServers = nullptr;
		gOnConnectingChanged = nullptr;
	}
};
} // namespace

TEST(srvlist, set_get_roundtrip_and_push) {
	Reset r;
	int pushes = gPushServersCalls;
	setServerList({srv("a", "h1", 1), srv("b", "h2", 2)});
	auto got = getServerList();
	CHECK_EQ(got.size(), 2u);
	CHECK_EQ(got[0].name, "a");
	CHECK_EQ(got[1].port, 2);
	CHECK_EQ(gPushServersCalls, pushes + 1);
	CHECK_EQ(gLastPushedServers.size(), 2u);
}

TEST(srvlist, update_autoconnect_targets_entry) {
	Reset r;
	setServerList({srv("a", "h1", 1), srv("b", "h1", 2)});
	updateAutoconnect("h1", 2, true);
	auto got = getServerList();
	CHECK(!got[0].autoconnect);
	CHECK(got[1].autoconnect);
	// Unknown host is a no-op.
	updateAutoconnect("nope", 9, true);
	got = getServerList();
	CHECK(!got[0].autoconnect);
}

TEST(srvlist, connection_error_state) {
	Reset r;
	setConnectionError("boom");
	CHECK_EQ(getConnectionError(), "boom");
	CHECK_EQ(gLastConnError, "boom");
	clearConnectionError();
	CHECK(getConnectionError().empty());
	CHECK(gLastConnError.empty());
}

TEST(srvlist, connecting_state_and_callback) {
	Reset r;
	int seen = -1;
	gOnConnectingChanged = [&](bool c) { seen = c ? 1 : 0; };
	gPinEntryRequested.store(true);

	setConnecting(true);
	CHECK(isConnecting());
	CHECK_EQ(seen, 1);
	CHECK_EQ(gLastConnecting, 1);
	CHECK(gPinEntryRequested.load()); // still set while connecting

	setConnecting(false);
	CHECK(!isConnecting());
	CHECK_EQ(seen, 0);
	CHECK(!gPinEntryRequested.load()); // cleared when the attempt ends
}

TEST(srvlist, content_height) {
	Reset r;
	// Empty list is just the hint text.
	CHECK(std::abs(serverContentHeight() - 0.10f) < 1e-6);

	setServerList({srv("a", "h", 1)});
	// one row + gap + refresh
	CHECK(std::abs(serverContentHeight() - (0.14f + 0.02f + 0.10f)) < 1e-6);

	setServerList({srv("a", "h", 1), srv("b", "h", 2)});
	CHECK(std::abs(serverContentHeight() - (2 * 0.14f + 0.02f + 0.02f + 0.10f)) < 1e-6);

	// Error banner adds its own height + gap.
	setConnectionError("x");
	float withErr = serverContentHeight();
	setConnectionError("");
	CHECK(std::abs(withErr - (2 * 0.14f + 0.02f + 0.06f + 0.02f + 0.02f + 0.10f)) < 1e-6);
}

TEST(srvlist, build_empty_and_populated) {
	Reset r;
	std::vector<float> v;
	float h = buildServerContent(v, 0.0f, -1, -1);
	CHECK(std::abs(h - 0.20f) < 1e-6);
	CHECK(!v.empty()); // hint text emitted

	v.clear();
	setServerList({srv("alpha", "host", 1234)});
	h = buildServerContent(v, 0.0f, -1, -1);
	CHECK(std::abs(h - serverContentHeight()) < 1e-6);
	CHECK(!v.empty());
}

TEST(srvlist, build_with_error_and_variants) {
	Reset r;
	ServerInfo s = srv("beta", "h", 5);
	s.discovered = true;
	s.autoconnect = true;
	ServerInfo m = srv("manual", "h2", 6, true);
	setServerList({s, m});
	setConnectionError("nope");
	std::vector<float> v;
	buildServerContent(v, 0.0f, 0, 0);
	CHECK(!v.empty());
	// Scroll pushes everything up; still builds.
	v.clear();
	buildServerContent(v, 0.5f, -1, -2);
	CHECK(!v.empty());
}

TEST(srvlist, hit_row_and_parts) {
	Reset r;
	setServerList({srv("a", "h", 1)});

	// X button centre: (0.305, 0.41) for the first row.
	SrvHover h = hitServerContent(0.305f, 0.41f, 0.0f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 3);
	CHECK(h.grab);

	// Connect button centre: (0.175, 0.41).
	h = hitServerContent(0.175f, 0.41f, 0.0f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 1);

	// Auto toggle centre: (-0.05, 0.41).
	h = hitServerContent(-0.05f, 0.41f, 0.0f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 2);

	// Row body away from the buttons.
	h = hitServerContent(-0.3f, 0.41f, 0.0f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 0);
	CHECK(!h.grab);

	// Refresh button: centred below the row.
	h = hitServerContent((kCtX0 + kCtX1Content) * 0.5f, 0.28f, 0.0f);
	CHECK_EQ(h.item, -1);
	CHECK_EQ(h.part, 4);

	// Way off the list.
	h = hitServerContent(-5.0f, -5.0f, 0.0f);
	CHECK_EQ(h.item, -1);
}

TEST(srvlist, hit_skips_auto_for_manual) {
	Reset r;
	setServerList({srv("m", "h", 1, true)});
	// Auto toggle area should fall through to the row body for manual servers.
	SrvHover h = hitServerContent(-0.05f, 0.41f, 0.0f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 0);
}

TEST(srvlist, hit_with_error_offsets_rows) {
	Reset r;
	setServerList({srv("a", "h", 1)});
	setConnectionError("err");
	// Row top is pushed down by the banner: first row now starts at
	// kCtTop - 0.08. Clicking the old row-Y now hits nothing or the banner area.
	SrvHover h = hitServerContent(-0.3f, 0.33f, 0.0f);
	CHECK_EQ(h.item, 0);
	CHECK_EQ(h.part, 0);
}

TEST(srvlist, click_connect_clears_error_and_fires) {
	Reset r;
	setServerList({srv("a", "h", 7)});
	setConnectionError("old");
	ServerInfo got;
	bool fired = false;
	gOnServerConnect = [&](const ServerInfo &s) { fired = true; got = s; };
	applyServerClick(SrvHover{0, 1, true}, true);
	CHECK(fired);
	CHECK_EQ(got.port, 7);
	CHECK(getConnectionError().empty());

	// No click edge -> nothing.
	setConnectionError("x");
	fired = false;
	applyServerClick(SrvHover{0, 1, true}, false);
	CHECK(!fired);
	CHECK_EQ(getConnectionError(), "x");
}

TEST(srvlist, click_autoconnect_and_remove) {
	Reset r;
	setServerList({srv("a", "h", 7)});
	std::string host;
	int port = 0;
	gOnServerAutoconnect = [&](const std::string &h, int p) { host = h; port = p; };
	applyServerClick(SrvHover{0, 2, true}, true);
	CHECK(getServerList()[0].autoconnect);
	CHECK_EQ(host, "h");
	CHECK_EQ(port, 7);

	bool removed = false;
	gOnServerRemove = [&](const std::string &h, int p) { removed = (h == "h" && p == 7); };
	applyServerClick(SrvHover{0, 3, true}, true);
	CHECK(removed);
}

TEST(srvlist, click_refresh_and_guards) {
	Reset r;
	setServerList({srv("a", "h", 1)});
	int refreshes = 0;
	gOnRefreshServers = [&] { ++refreshes; };
	applyServerClick(SrvHover{-1, 4, true}, true);
	CHECK_EQ(refreshes, 1);

	// Negative item or out-of-range index: safe no-ops.
	applyServerClick(SrvHover{-1, 0, false}, true);
	applyServerClick(SrvHover{9, 1, true}, true);
	CHECK_EQ(refreshes, 1);
}

TEST(srvlist, scroll_shifts_hit_test) {
	Reset r;
	setServerList({srv("a", "h", 1), srv("b", "h", 2)});
	// Scrolling down by one row+gap puts row 1's body at cy ~0.09.
	SrvHover h = hitServerContent(-0.3f, 0.09f, 0.16f);
	CHECK_EQ(h.item, 1);
}
