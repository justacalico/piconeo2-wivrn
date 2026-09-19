// wivrn/net/wivrn_client_pico.cpp: the full session handshake and post-connect
// packet I/O, driven against a loopback fake server that speaks the real wire
// protocol (same typed_socket/serialization/crypto stack the server uses).
#include "test_framework.h"
#include "faults.h"
#include "wivrn_client_pico.h"
#include "crypto.h"
#include "secrets.h"
#include "smp.h"
#include "wivrn_packets.h"

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <system_error>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
namespace from_headset = wivrn::from_headset;
namespace to_headset = wivrn::to_headset;
using crypto_state = to_headset::crypto_handshake::crypto_state;

// Server side sees the mirror image of the client's packet directions.
using srv_control_t = wivrn::typed_socket<wivrn::TCP, from_headset::packets, to_headset::packets>;
using srv_stream_t = wivrn::typed_socket<wivrn::UDP, from_headset::packets, to_headset::packets>;

namespace
{

int bound_port(int fd)
{
	sockaddr_in6 sa{};
	socklen_t n = sizeof(sa);
	getsockname(fd, (sockaddr *)&sa, &n);
	return ntohs(sa.sin6_port);
}

sockaddr_in6 loopback6(int port)
{
	sockaddr_in6 sa{};
	sa.sin6_family = AF_INET6;
	sa.sin6_port = htons(port);
	inet_pton(AF_INET6, "::ffff:127.0.0.1", &sa.sin6_addr);
	return sa;
}

sockaddr_in loopback4(int port)
{
	sockaddr_in sa{};
	sa.sin_family = AF_INET;
	sa.sin_port = htons(port);
	inet_pton(AF_INET, "127.0.0.1", &sa.sin_addr);
	return sa;
}

// Bind a server UDP stream socket on an ephemeral port.
srv_stream_t make_stream()
{
	srv_stream_t s;
	sockaddr_in6 a{};
	a.sin6_family = AF_INET6;
	a.sin6_addr = in6addr_any;
	s.bind(a);
	return s;
}

// typed_socket::receive() reads with MSG_DONTWAIT: poll first, then drain.
// Anything already buffered is served through receive_pending.
from_headset::packets srv_recv(srv_control_t & ctrl)
{
	while (true)
	{
		if (auto pkt = ctrl.receive_pending())
			return std::move(*pkt);

		pollfd pfd{ctrl.get_fd(), POLLIN, 0};
		int r = ::poll(&pfd, 1, 15000);
		if (r == 0)
			throw std::runtime_error("fake server: recv timeout");
		if (r < 0)
			throw std::system_error(errno, std::system_category());
		if (pfd.revents & (POLLHUP | POLLERR))
			throw std::runtime_error("fake server: control socket closed");
		if (pfd.revents & POLLIN)
		{
			if (auto pkt = ctrl.receive())
				return std::move(*pkt);
		}
	}
}

// First packet every client sends.
from_headset::crypto_handshake read_hello(srv_control_t & ctrl)
{
	return std::get<from_headset::crypto_handshake>(srv_recv(ctrl));
}

// Block until the client closes the control socket (session destruction) or a
// cap expires. Keeping the server's sockets alive this long means no ICMP
// unreachable can race a client resend and fail an otherwise good handshake.
void wait_peer_close(srv_control_t & ctrl)
{
	pollfd pfd{ctrl.get_fd(), POLLIN | POLLRDHUP, 0};
	::poll(&pfd, 1, 5000);
}

// Bounded accept: if the client's connect never lands (e.g. loopback quirks in
// a container), the server thread must still be joinable.
std::pair<srv_control_t, sockaddr_in6> srv_accept(wivrn::TCPListener & listener)
{
	pollfd pfd{listener.get_fd(), POLLIN, 0};
	if (::poll(&pfd, 1, 15000) <= 0)
		throw std::runtime_error("fake server: accept timeout");
	return listener.accept<srv_control_t>();
}

// Bounded stream read, same reason.
auto stream_recv(srv_stream_t & stream)
{
	pollfd pfd{stream.get_fd(), POLLIN, 0};
	if (::poll(&pfd, 1, 15000) <= 0)
		throw std::runtime_error("fake server: stream recv timeout");
	return stream.receive_from_raw();
}

// A server-side exception must end the thread quietly: the client reports the
// real failure through its own timeout or socket error, while join() still
// returns instead of hanging the whole suite.
template <typename F>
std::thread run_server(F && f)
{
	return std::thread([fn = std::forward<F>(f)] {
		try
		{
			fn();
		}
		catch (...)
		{
		}
	});
}

// Serialize a to_headset packet into a wire frame ([u32 size][payload]) so a
// test can write it onto the control fd in controlled pieces.
template <typename T>
std::vector<uint8_t> wire_frame(const T &packet)
{
	wivrn::serialization_packet p;
	srv_control_t::serialize(p, packet);
	auto &spans = (std::vector<std::span<uint8_t>> &)p;

	uint32_t size = 0;
	for (const auto &chunk : spans)
		size += chunk.size_bytes();

	std::vector<uint8_t> frame;
	uint8_t *szp = (uint8_t *)&size;
	frame.insert(frame.end(), szp, szp + sizeof(size));
	for (const auto &chunk : spans)
		frame.insert(frame.end(), chunk.begin(), chunk.end());
	return frame;
}

void expect_throws(std::function<void()> fn, const char *what_substr)
{
	try
	{
		fn();
	}
	catch (const std::exception &e)
	{
		if (what_substr)
			CHECK(std::string(e.what()).find(what_substr) != std::string::npos);
		return;
	}
	CHECK(false); // reached only when fn did not throw
}

} // namespace

TEST(client, encryption_disabled_udp_stream)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());
	std::atomic<bool> got_stream_handshake{false};

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::encryption_disabled});
		srv_recv(ctrl); // client's confirmation crypto_handshake{}

		auto stream = make_stream();
		ctrl.send(to_headset::handshake{.stream_port = bound_port(stream.get_fd())});

		auto [raw, from] = stream_recv(stream);
		auto pkt = raw.deserialize<from_headset::packets>();
		CHECK(std::holds_alternative<from_headset::handshake>(pkt));
		got_stream_handshake = true;

		stream.connect(from);
		stream.send(to_headset::handshake{});
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	uint64_t sent = 0, received = 0;
	{
		wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                         [](int) { return "000000"; }, shutdown);
		sent = session.bytes_sent();
		received = session.bytes_received();
	}
	srv.join();
	CHECK(got_stream_handshake.load());
	CHECK(sent > 0);
	CHECK(received > 0);
}

TEST(client, encryption_disabled_tcp_only)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::encryption_disabled});
		srv_recv(ctrl);
		ctrl.send(to_headset::handshake{.stream_port = 9757});

		// tcp_only client falls back to control for stream packets.
		auto pkt = srv_recv(ctrl);
		CHECK(std::holds_alternative<from_headset::handshake>(pkt));
		ctrl.send(to_headset::handshake{});
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	{
		wivrn_session_pico session(loopback4(port), true, kp, "test-headset",
		                         [](int) { return "000000"; }, shutdown);
	}
	srv.join();
}

TEST(client, pin_pairing_flow)
{
	const std::string PIN = "424242";
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());
	std::atomic<bool> pin_ok{false};
	std::atomic<int> pin_fd{-1};

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		auto hello = read_hello(ctrl);
		CHECK(hello.name == "test-headset");

		crypto::key srv_key = crypto::key::generate_x25519_keypair();
		ctrl.send(to_headset::crypto_handshake{
		        .public_key = srv_key.public_key(),
		        .state = crypto_state::pin_needed});

		crypto::smp server_smp;
		auto m1 = std::get<from_headset::pin_check_1>(srv_recv(ctrl)).message;
		auto m2 = server_smp.step2(m1, PIN);
		ctrl.send(to_headset::pin_check_2{.message = m2});

		auto m3 = std::get<from_headset::pin_check_3>(srv_recv(ctrl)).message;
		auto [m4, matched] = server_smp.step4(m3);
		ctrl.send(to_headset::pin_check_4{.message = m4});
		pin_ok = matched;

		crypto::key client_pub = crypto::key::from_public_key(hello.public_key);
		secrets s{srv_key, client_pub, PIN};
		ctrl.set_aes_key_and_ivs(s.control_key, s.control_iv_from_headset,
		                         s.control_iv_to_headset);
		srv_recv(ctrl); // encrypted crypto_handshake{}

		auto stream = make_stream();
		stream.set_aes_key_and_ivs(s.stream_key, s.stream_iv_header_from_headset,
		                         s.stream_iv_header_to_headset);
		ctrl.send(to_headset::handshake{.stream_port = bound_port(stream.get_fd())});

		auto [raw, from] = stream_recv(stream);
		auto pkt = raw.deserialize<from_headset::packets>();
		CHECK(std::holds_alternative<from_headset::handshake>(pkt));

		stream.connect(from);
		stream.send(to_headset::handshake{});
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	uint64_t sent = 0, received = 0;
	{
		wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                         [&](int fd) {
			                         pin_fd = fd;
			                         return PIN;
		                         },
		                         shutdown);
		sent = session.bytes_sent();
		received = session.bytes_received();
	}
	srv.join();
	CHECK(pin_ok.load());
	CHECK(pin_fd.load() >= 0);
	CHECK(sent > 0);
	CHECK(received > 0);
}

TEST(client, already_paired_tcp_only)
{
	const std::string PIN = "000000"; // the pin the client falls back to
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		auto hello = read_hello(ctrl);

		crypto::key srv_key = crypto::key::generate_x25519_keypair();
		ctrl.send(to_headset::crypto_handshake{
		        .public_key = srv_key.public_key(),
		        .state = crypto_state::client_already_paired});

		crypto::key client_pub = crypto::key::from_public_key(hello.public_key);
		secrets s{srv_key, client_pub, PIN};
		ctrl.set_aes_key_and_ivs(s.control_key, s.control_iv_from_headset,
		                         s.control_iv_to_headset);
		srv_recv(ctrl); // encrypted crypto_handshake{}
		ctrl.send(to_headset::handshake{.stream_port = -1});

		auto pkt = srv_recv(ctrl); // from_headset::handshake over control
		CHECK(std::holds_alternative<from_headset::handshake>(pkt));
		ctrl.send(to_headset::handshake{});
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	{
		wivrn_session_pico session(loopback6(port), true, kp, "test-headset",
		                         [](int) { return "999999"; }, shutdown);
	}
	srv.join();
}

TEST(client, wrong_pin_is_rejected)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		auto hello = read_hello(ctrl);

		crypto::key srv_key = crypto::key::generate_x25519_keypair();
		ctrl.send(to_headset::crypto_handshake{
		        .public_key = srv_key.public_key(),
		        .state = crypto_state::pin_needed});

		try
		{
			crypto::smp server_smp;
			auto m1 = std::get<from_headset::pin_check_1>(srv_recv(ctrl)).message;
			auto m2 = server_smp.step2(m1, "111111"); // server knows a different pin
			ctrl.send(to_headset::pin_check_2{.message = m2});

			auto m3 = std::get<from_headset::pin_check_3>(srv_recv(ctrl)).message;
			auto [m4, matched] = server_smp.step4(m3);
			ctrl.send(to_headset::pin_check_4{.message = m4});
		}
		catch (...)
		{
		}
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	expect_throws(
	        [&] {
		        wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                                 [](int) { return "000000"; }, shutdown);
	        },
	        "PIN");
	srv.join();
}

TEST(client, pairing_disabled_throws)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::pairing_disabled});
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	expect_throws(
	        [&] {
		        wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                                 [](int) { return "000000"; }, shutdown);
	        },
	        "Pairing is disabled");
	srv.join();
}

TEST(client, incompatible_version_throws)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::incompatible_version});
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	expect_throws(
	        [&] {
		        wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                                 [](int) { return "000000"; }, shutdown);
	        },
	        "Incompatible protocol");
	srv.join();
}

TEST(client, shutdown_flag_cancels_handshake)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		std::this_thread::sleep_for(300ms);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{true}; // cancelled before we even start
	expect_throws(
	        [&] {
		        wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                                 [](int) { return "000000"; }, shutdown);
	        },
	        "cancelled");
	srv.join();
}

TEST(client, silent_server_times_out)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		std::this_thread::sleep_for(11s); // outlive the client's 10s handshake timeout
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	expect_throws(
	        [&] {
		        wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                                 [](int) { return "000000"; }, shutdown);
	        },
	        "Timeout");
	srv.join();
}

TEST(client, poll_dispatch_pending_and_socket_errors)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());
	std::atomic<int> stage{0};

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::encryption_disabled});
		srv_recv(ctrl);
		ctrl.send(to_headset::handshake{.stream_port = -1});
		srv_recv(ctrl); // client's handshake
		ctrl.send(to_headset::handshake{}); // completes client handshake

		while (stage.load() == 0)
			std::this_thread::sleep_for(5ms);
		ctrl.send(to_headset::timesync_query{.query = 12345});
		while (stage.load() == 1)
			std::this_thread::sleep_for(5ms);
		// ctrl destructs with the thread -> client sees POLLHUP
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	wivrn_session_pico session(loopback6(port), true, kp, "test-headset",
	                         [](int) { return "000000"; }, shutdown);

	// No traffic: poll times out and reports 0.
	CHECK(session.poll([](const auto &) {}, 30ms) == 0);

	// Have the server push a packet; poll must dispatch it to the visitor.
	stage = 1;
	bool got_query = false;
	int r = session.poll(
	        [&](const auto &packet) {
		        using T = std::remove_cvref_t<decltype(packet)>;
		        if constexpr (std::is_same_v<T, to_headset::timesync_query>)
		        {
			        got_query = true;
			        CHECK(packet.query == 12345);
		        }
	        },
	        5000ms);
	CHECK(r > 0);
	CHECK(got_query);
	CHECK(session.bytes_received() > 0);

	// Killing the control socket makes poll() throw.
	stage = 2;
	expect_throws([&] { session.poll([](const auto &) {}, 5000ms); },
	              "control socket");
	srv.join();
}

TEST(client, send_paths_count_bytes)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::encryption_disabled});
		srv_recv(ctrl);
		ctrl.send(to_headset::handshake{.stream_port = -1});
		srv_recv(ctrl);
		ctrl.send(to_headset::handshake{});

		// Drain whatever the test sends, keep the socket alive a moment.
		for (int i = 0; i < 2; ++i)
			srv_recv(ctrl);
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	uint64_t sent = 0;
	{
		wivrn_session_pico session(loopback6(port), true, kp, "test-headset",
		                         [](int) { return "000000"; }, shutdown);

		uint64_t before = session.bytes_sent();
		session.send_control(from_headset::timesync_response{.query = 1, .response = 2});
		// tcp_only: send_stream falls back to the control socket.
		session.send_stream(from_headset::timesync_response{.query = 3, .response = 4});
		sent = session.bytes_sent();
		CHECK(sent > before);
	}
	srv.join();
	CHECK(sent > 0);
}

TEST(client, handshake_error_carries_message)
{
	handshake_error err("test message");
	CHECK(std::string(err.what()) == "test message");
}

TEST(client, ipv4_udp_stream_fails)
{
	// The UDP stream socket is AF_INET6; the IPv4 ctor can never connect it.
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::encryption_disabled});
		srv_recv(ctrl);
		ctrl.send(to_headset::handshake{.stream_port = 9757});
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	expect_throws(
	        [&] {
		        wivrn_session_pico session(loopback4(port), false, kp, "test-headset",
		                                 [](int) { return "000000"; }, shutdown);
	        },
	        nullptr);
	srv.join();
}

TEST(client, poll_failure_during_handshake)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		std::this_thread::sleep_for(300ms);
	});

	test_fault_only = "poll";
	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	expect_throws(
	        [&] {
		        wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                                 [](int) { return "000000"; }, shutdown);
	        },
	        nullptr);
	test_fault_only = nullptr;
	srv.join();
}

TEST(client, fragmented_packet_retries_receive)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);

		// Hand-serialize the crypto_handshake answer and push it onto the wire
		// in two chunks so the client's first receive() comes back incomplete.
		auto frame = wire_frame(to_headset::crypto_handshake{
		        .state = crypto_state::encryption_disabled});

		size_t first = sizeof(uint32_t) + 2; // header + a slice of payload
		::send(ctrl.get_fd(), frame.data(), first, MSG_NOSIGNAL);
		std::this_thread::sleep_for(60ms);
		::send(ctrl.get_fd(), frame.data() + first, frame.size() - first, MSG_NOSIGNAL);

		srv_recv(ctrl); // confirmation crypto_handshake{}
		ctrl.send(to_headset::handshake{.stream_port = -1});
		srv_recv(ctrl); // client's handshake
		ctrl.send(to_headset::handshake{});
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	bool ok = false;
	{
		wivrn_session_pico session(loopback6(port), true, kp, "test-headset",
		                         [](int) { return "000000"; }, shutdown);
		ok = session.is_handshake_ok();
	}
	CHECK(ok);
	srv.join();
}

TEST(client, hangup_event_aborts_handshake)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		std::this_thread::sleep_for(300ms); // never answers, never polls
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	test_poll_revents = POLLHUP; // error without POLLIN: unreachable via real TCP
	expect_throws(
	        [&] {
		        wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                                 [](int) { return "000000"; }, shutdown);
	        },
	        "Socket error during handshake");
	test_poll_revents = 0;
	srv.join();
}

TEST(client, reset_peer_aborts_handshake)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		// RST, not a clean FIN: SO_LINGER{1,0} then close.
		linger l{1, 0};
		setsockopt(ctrl.get_fd(), SOL_SOCKET, SO_LINGER, &l, sizeof(l));
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	expect_throws(
	        [&] {
		        wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                                 [](int) { return "000000"; }, shutdown);
	        },
	        nullptr);
	srv.join();
}

TEST(client, forged_smp_message_throws)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);

		crypto::key srv_key = crypto::key::generate_x25519_keypair();
		ctrl.send(to_headset::crypto_handshake{
		        .public_key = srv_key.public_key(),
		        .state = crypto_state::pin_needed});

		srv_recv(ctrl); // pin_check_1
		// A syntactically valid but cryptographically wrong msg2: the ZK
		// proofs in it can never verify, so the client raises smp_cheated.
		crypto::smp::msg2 forged{};
		for (auto &b : forged)
			b = crypto::bignum(2);
		ctrl.send(to_headset::pin_check_2{.message = forged});
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	expect_throws(
	        [&] {
		        wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                                 [](int) { return "000000"; }, shutdown);
	        },
	        "PIN");
	srv.join();
}

TEST(client, missing_second_handshake_is_not_fatal)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::encryption_disabled});
		srv_recv(ctrl);
		ctrl.send(to_headset::handshake{.stream_port = -1});
		srv_recv(ctrl); // client's handshake; never answer it
		std::this_thread::sleep_for(11s);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	// The ctor returns after its 10s wait with handshake_ok=false.
	wivrn_session_pico session(loopback6(port), true, kp, "test-headset",
	                         [](int) { return "000000"; }, shutdown);
	CHECK(!session.is_handshake_ok());
	srv.join();
}

TEST(client, stream_handshake_is_resent)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());
	std::atomic<int> stream_hellos{0};

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::encryption_disabled});
		srv_recv(ctrl);

		auto stream = make_stream();
		ctrl.send(to_headset::handshake{.stream_port = bound_port(stream.get_fd())});

		// Sit on the first hello long enough that the client resends it, then
		// answer so the handshake completes.
		auto [raw, from] = stream_recv(stream);
		stream.connect(from);
		stream_hellos++;
		std::this_thread::sleep_for(250ms);
		auto [raw2, from2] = stream_recv(stream);
		stream_hellos++;
		stream.send(to_headset::handshake{});
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	bool ok = false;
	{
		wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                         [](int) { return "000000"; }, shutdown);
		ok = session.is_handshake_ok();
	}
	srv.join();
	CHECK(stream_hellos.load() >= 2);
	CHECK(ok);
}

TEST(client, stream_socket_error_throws_on_poll)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());
	std::atomic<bool> kill_stream{false};

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::encryption_disabled});
		srv_recv(ctrl);

		auto stream = make_stream();
		ctrl.send(to_headset::handshake{.stream_port = bound_port(stream.get_fd())});
		auto [raw, from] = stream_recv(stream);
		stream.connect(from);
		stream.send(to_headset::handshake{});

		while (!kill_stream.load())
			std::this_thread::sleep_for(5ms);
		// Closing the stream socket makes the connected UDP fd report POLLERR
		// (ICMP port unreachable) once the client writes again.
		sockaddr_in6 peer_addr = from;
		stream = srv_stream_t{}; // fresh unbound socket; old one closes
		(void)peer_addr;
		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	{
		wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                         [](int) { return "000000"; }, shutdown);
		CHECK(session.is_handshake_ok());

		kill_stream = true;
		std::this_thread::sleep_for(50ms);
		bool threw = false;
		for (int i = 0; i < 20 && !threw; ++i)
		{
			session.send_stream(from_headset::handshake{});
			try
			{
				session.poll([](const auto &) {}, 50ms);
			}
			catch (const std::runtime_error &e)
			{
				threw = true;
				CHECK(std::string(e.what()).find("stream socket") != std::string::npos);
			}
		}
		CHECK(threw);
	}
	srv.join();
}

TEST(client, poll_drains_pending_packets)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());
	std::atomic<int> stage{0};

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::encryption_disabled});
		srv_recv(ctrl);

		auto stream = make_stream();
		ctrl.send(to_headset::handshake{.stream_port = bound_port(stream.get_fd())});
		auto [raw, from] = stream_recv(stream);
		stream.connect(from);
		stream.send(to_headset::handshake{});

		while (stage.load() == 0)
			std::this_thread::sleep_for(5ms);

		// Two control packets coalesced into a single TCP segment: the second
		// stays buffered and is served from receive_pending on the next poll.
		std::vector<uint8_t> blob;
		for (int i = 0; i < 2; ++i)
		{
			auto frame = wire_frame(to_headset::timesync_query{.query = 100 + i});
			blob.insert(blob.end(), frame.begin(), frame.end());
		}
		::send(ctrl.get_fd(), blob.data(), blob.size(), MSG_NOSIGNAL);

		// Two stream datagrams back to back: recvmmsg returns the first and
		// queues the second for receive_pending.
		stream.send(to_headset::timesync_query{.query = 200});
		stream.send(to_headset::timesync_query{.query = 201});

		wait_peer_close(ctrl);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	{
		wivrn_session_pico session(loopback6(port), false, kp, "test-headset",
		                         [](int) { return "000000"; }, shutdown);
		CHECK(session.is_handshake_ok());

		stage = 1;
		int seen = 0;
		for (int i = 0; i < 10 && seen < 4; ++i)
		{
			session.poll(
			        [&](const auto &packet) {
				        using T = std::remove_cvref_t<decltype(packet)>;
				        if constexpr (std::is_same_v<T, to_headset::timesync_query>)
					        ++seen;
			        },
			        300ms);
		}
		CHECK(seen == 4);
		stage = 2;
	}
	srv.join();
}

TEST(client, session_poll_failure_throws)
{
	wivrn::TCPListener listener(0);
	int port = bound_port(listener.get_fd());

	std::thread srv = run_server([&] {
		auto [ctrl, peer] = srv_accept(listener);
		read_hello(ctrl);
		ctrl.send(to_headset::crypto_handshake{.state = crypto_state::encryption_disabled});
		srv_recv(ctrl);
		ctrl.send(to_headset::handshake{.stream_port = -1});
		srv_recv(ctrl);
		ctrl.send(to_headset::handshake{});
		std::this_thread::sleep_for(500ms);
	});

	crypto::key kp = crypto::key::generate_x25519_keypair();
	std::atomic<bool> shutdown{false};
	wivrn_session_pico session(loopback6(port), true, kp, "test-headset",
	                         [](int) { return "000000"; }, shutdown);

	test_fault_only = "poll";
	expect_throws([&] { session.poll([](const auto &) {}, 10ms); }, nullptr);
	test_fault_only = nullptr;
	srv.join();
}
