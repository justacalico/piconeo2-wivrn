// wivrn/common/wivrn_sockets.cpp: UDP/TCP loopback round-trips, encryption,
// framing and error paths. Uses real sockets on 127.0.0.1 / ::1.
#include "test_framework.h"
#include "wivrn_sockets.h"

#include <arpa/inet.h>
#include <cstring>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
#include <variant>

using wivrn::deserialization_packet;
using wivrn::serialization_packet;

namespace
{
uint16_t bound_port(wivrn::fd_base & s)
{
	sockaddr_in6 addr{};
	socklen_t len = sizeof(addr);
	getsockname(s.get_fd(), (sockaddr *)&addr, &len);
	return ntohs(addr.sin6_port);
}

sockaddr_in6 loopback6(uint16_t port)
{
	sockaddr_in6 a{};
	a.sin6_family = AF_INET6;
	a.sin6_port = htons(port);
	inet_pton(AF_INET6, "::1", &a.sin6_addr);
	return a;
}

wivrn::UDP bound_udp()
{
	wivrn::UDP s;
	sockaddr_in6 a{};
	a.sin6_family = AF_INET6;
	a.sin6_addr = in6addr_any;
	s.bind(a);
	return s;
}

// Next framed TCP packet. receive_raw() throws EAGAIN when the socket is empty
// even if a complete packet is already buffered, so fall back to
// receive_pending() and keep polling until something lands.
deserialization_packet next_tcp(wivrn::TCP & s)
{
	deserialization_packet pkt = s.receive_pending();
	for (int tries = 0; tries < 10000 && pkt.empty(); ++tries) {
		try {
			pkt = s.receive_raw();
		} catch (const std::system_error &) {
			pkt = s.receive_pending();
		}
	}
	return pkt;
}
} // namespace

TEST(sockets, exceptions_have_messages) {
	wivrn::socket_shutdown se;
	CHECK(std::string(se.what()) == "Socket shutdown");
	wivrn::invalid_packet ie;
	CHECK(std::string(ie.what()) == "Invalid packet");
}

TEST(sockets, fd_base_move_and_close) {
	wivrn::UDP a;
	CHECK(a);
	int raw = a.get_fd();
	CHECK(raw >= 0);
	CHECK((int)a == raw);

	wivrn::UDP b(std::move(a));
	CHECK(b);
	CHECK(!a); // moved-from is empty
	CHECK(b.get_fd() == raw);
}

TEST(sockets, udp_roundtrip) {
	wivrn::UDP rx = bound_udp();
	uint16_t port = bound_port(rx);

	wivrn::UDP tx;
	tx.connect(loopback6(port));

	serialization_packet p;
	p.serialize(std::string("ping"));
	p.serialize(uint32_t(42));
	CHECK_EQ(tx.send_raw(std::move(p)), 2 + 4 + 4); // size u16 + "ping" + u32

	auto pkt = rx.receive_raw();
	CHECK(!pkt.empty());
	CHECK_EQ(pkt.deserialize<std::string>(), "ping");
	CHECK_EQ(pkt.deserialize<uint32_t>(), 42u);
}

TEST(sockets, udp_ipv4_connect) {
	wivrn::UDP rx = bound_udp();
	uint16_t port = bound_port(rx);

	wivrn::UDP tx;
	sockaddr_in a{};
	a.sin_family = AF_INET;
	a.sin_port = htons(port);
	inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
	tx.connect(a);

	serialization_packet p;
	p.serialize(uint8_t(7));
	tx.send_raw(std::move(p));
	auto pkt = rx.receive_raw();
	CHECK_EQ(pkt.deserialize<uint8_t>(), 7);
}

TEST(sockets, udp_receive_pending_queued) {
	wivrn::UDP rx = bound_udp();
	uint16_t port = bound_port(rx);
	wivrn::UDP tx;
	tx.connect(loopback6(port));

	for (uint32_t i = 0; i < 3; ++i) {
		serialization_packet p;
		p.serialize(i);
		tx.send_raw(std::move(p));
	}
	// First datagram via receive_raw; the rest queued for receive_pending.
	CHECK_EQ(rx.receive_raw().deserialize<uint32_t>(), 0u);
	CHECK_EQ(rx.receive_pending().deserialize<uint32_t>(), 1u);
	CHECK_EQ(rx.receive_pending().deserialize<uint32_t>(), 2u);
	CHECK(rx.receive_pending().empty());
}

TEST(sockets, udp_receive_from_reports_sender) {
	wivrn::UDP rx = bound_udp();
	uint16_t port = bound_port(rx);
	wivrn::UDP tx = bound_udp();
	tx.connect(loopback6(port));

	serialization_packet p;
	p.serialize(uint16_t(9));
	tx.send_raw(std::move(p));

	auto [pkt, from] = rx.receive_from_raw();
	CHECK_EQ(pkt.deserialize<uint16_t>(), 9);
	CHECK_EQ(ntohs(from.sin6_port), bound_port(tx));
	CHECK(IN6_IS_ADDR_V4MAPPED(&from.sin6_addr) || IN6_IS_ADDR_LOOPBACK(&from.sin6_addr));
}

TEST(sockets, udp_send_many) {
	wivrn::UDP rx = bound_udp();
	uint16_t port = bound_port(rx);
	wivrn::UDP tx;
	tx.connect(loopback6(port));

	std::vector<serialization_packet> pkts(3);
	for (uint32_t i = 0; i < 3; ++i)
		pkts[i].serialize(i + 10);
	size_t sent = tx.send_many_raw(pkts);
	CHECK_EQ(sent, 3 * 4u);

	for (uint32_t i = 0; i < 3; ++i)
		CHECK_EQ(rx.receive_raw().deserialize<uint32_t>(), i + 10);

	// Empty batch is a no-op.
	pkts.clear();
	CHECK_EQ(tx.send_many_raw(pkts), 0u);
}

TEST(sockets, udp_encrypted_roundtrip) {
	wivrn::UDP rx = bound_udp();
	uint16_t port = bound_port(rx);
	wivrn::UDP tx;
	tx.connect(loopback6(port));

	std::array<uint8_t, 16> key{};
	std::array<uint8_t, 8> iv_ab{}, iv_ba{};
	for (int i = 0; i < 16; ++i)
		key[i] = uint8_t(i);
	for (int i = 0; i < 8; ++i) {
		iv_ab[i] = uint8_t(0xa0 + i);
		iv_ba[i] = uint8_t(0xb0 + i);
	}
	tx.set_aes_key_and_ivs(key, /*recv*/ iv_ba, /*send*/ iv_ab);
	rx.set_aes_key_and_ivs(key, /*recv*/ iv_ab, /*send*/ iv_ba);

	serialization_packet p;
	p.serialize(std::string("secret"));
	tx.send_raw(std::move(p));

	auto pkt = rx.receive_raw();
	CHECK_EQ(pkt.deserialize<std::string>(), "secret");
}

TEST(sockets, udp_encrypted_short_packet_throws) {
	wivrn::UDP rx = bound_udp();
	uint16_t port = bound_port(rx);

	std::array<uint8_t, 16> key{};
	std::array<uint8_t, 8> iv{};
	rx.set_aes_key_and_ivs(key, iv, iv);

	// Send a raw 4-byte datagram (smaller than the 8-byte IV prefix).
	int raw = socket(AF_INET6, SOCK_DGRAM, 0);
	sockaddr_in6 dst = loopback6(port);
	uint8_t junk[4] = {1, 2, 3, 4};
	sendto(raw, junk, sizeof(junk), 0, (sockaddr *)&dst, sizeof(dst));
	close(raw);

	CHECK_THROWS_AS(rx.receive_raw(), std::runtime_error);
}

TEST(sockets, udp_short_packet_dropped_in_from_raw) {
	wivrn::UDP rx = bound_udp();
	uint16_t port = bound_port(rx);

	std::array<uint8_t, 16> key{};
	std::array<uint8_t, 8> iv{};
	rx.set_aes_key_and_ivs(key, iv, iv);

	int raw = socket(AF_INET6, SOCK_DGRAM, 0);
	sockaddr_in6 dst = loopback6(port);
	uint8_t junk[4] = {1, 2, 3, 4};
	sendto(raw, junk, sizeof(junk), 0, (sockaddr *)&dst, sizeof(dst));
	close(raw);

	// receive_from_raw drops short encrypted packets instead of throwing.
	auto [pkt, from] = rx.receive_from_raw();
	CHECK(pkt.empty());
}

TEST(sockets, udp_socket_options) {
	wivrn::UDP s = bound_udp();
	s.set_receive_buffer_size(1 << 20);
	s.set_send_buffer_size(1 << 20);
	s.set_tos(0x10);
	// Buffer sizes are best-effort; just verify the calls don't throw.
	CHECK(s);
}

TEST(sockets, udp_multicast_membership) {
	wivrn::UDP s = bound_udp();
	in6_addr grp{};
	inet_pton(AF_INET6, "ff02::114", &grp);
	bool joined = false;
	try {
		s.subscribe_multicast(grp);
		joined = true;
	} catch (const std::system_error &) {
	}
	// Membership may fail on hosts without multicast routing; either way the
	// socket stays usable only on the success path (failure closes the fd).
	if (joined) {
		CHECK(s);
		try {
			s.unsubscribe_multicast(grp);
		} catch (const std::system_error &) {
		}
	} else {
		CHECK(!s);
	}
}

TEST(sockets, tcp_roundtrip) {
	wivrn::TCPListener listener(0);
	uint16_t port = bound_port(listener);

	wivrn::TCP client(loopback6(port));
	auto [server, peer] = listener.accept<wivrn::TCP>();
	CHECK(server);

	serialization_packet p;
	p.serialize(std::string("hello tcp"));
	p.serialize(uint64_t(0x1122334455667788));
	client.send_raw(std::move(p));

	auto pkt = server.receive_raw();
	CHECK_EQ(pkt.deserialize<std::string>(), "hello tcp");
	CHECK_EQ(pkt.deserialize<uint64_t>(), 0x1122334455667788ull);
}

TEST(sockets, tcp_ipv4_connect) {
	wivrn::TCPListener listener(0);
	uint16_t port = bound_port(listener);

	sockaddr_in a{};
	a.sin_family = AF_INET;
	a.sin_port = htons(port);
	// v4-mapped loopback: the listener is on in6addr_any.
	inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);

	wivrn::TCP client(a);
	auto [server, peer] = listener.accept<wivrn::TCP>();

	serialization_packet p;
	p.serialize(uint8_t(5));
	client.send_raw(std::move(p));
	CHECK_EQ(server.receive_raw().deserialize<uint8_t>(), 5);
}

TEST(sockets, tcp_fragmented_and_pending) {
	wivrn::TCPListener listener(0);
	uint16_t port = bound_port(listener);
	wivrn::TCP client(loopback6(port));
	auto [server, peer] = listener.accept<wivrn::TCP>();

	// Two packets in flight: receive_raw drains the first, pending the second.
	for (uint32_t i = 0; i < 2; ++i) {
		serialization_packet p;
		p.serialize(i + 100);
		client.send_raw(std::move(p));
	}
	CHECK_EQ(server.receive_raw().deserialize<uint32_t>(), 100u);
	CHECK_EQ(next_tcp(server).deserialize<uint32_t>(), 101u);
	CHECK(server.receive_pending().empty());
}

TEST(sockets, tcp_send_many) {
	wivrn::TCPListener listener(0);
	uint16_t port = bound_port(listener);
	wivrn::TCP client(loopback6(port));
	auto [server, peer] = listener.accept<wivrn::TCP>();

	std::vector<serialization_packet> pkts(3);
	for (uint32_t i = 0; i < 3; ++i)
		pkts[i].serialize(i);
	size_t sent = client.send_many_raw(pkts);
	CHECK_EQ(sent, 3 * (4 + 4)); // size header + payload each

	for (uint32_t i = 0; i < 3; ++i)
		CHECK_EQ(next_tcp(server).deserialize<uint32_t>(), i);

	pkts.clear();
	CHECK_EQ(client.send_many_raw(pkts), 0u);
}

TEST(sockets, tcp_encrypted_roundtrip) {
	wivrn::TCPListener listener(0);
	uint16_t port = bound_port(listener);
	wivrn::TCP client(loopback6(port));
	auto [server, peer] = listener.accept<wivrn::TCP>();

	std::array<uint8_t, 16> key{}, iv_ab{}, iv_ba{};
	for (int i = 0; i < 16; ++i) {
		key[i] = uint8_t(i * 3);
		iv_ab[i] = uint8_t(0x40 + i);
		iv_ba[i] = uint8_t(0x70 + i);
	}
	client.set_aes_key_and_ivs(key, /*recv*/ iv_ba, /*send*/ iv_ab);
	server.set_aes_key_and_ivs(key, /*recv*/ iv_ab, /*send*/ iv_ba);

	serialization_packet p;
	p.serialize(std::string("encrypted tcp"));
	client.send_raw(std::move(p));

	auto pkt = server.receive_raw();
	CHECK_EQ(pkt.deserialize<std::string>(), "encrypted tcp");
}

TEST(sockets, tcp_zero_size_throws) {
	wivrn::TCPListener listener(0);
	uint16_t port = bound_port(listener);
	wivrn::TCP client(loopback6(port));
	auto [server, peer] = listener.accept<wivrn::TCP>();

	uint32_t zero = 0;
	send(client.get_fd(), &zero, sizeof(zero), MSG_NOSIGNAL);
	CHECK_THROWS_AS(server.receive_raw(), std::runtime_error);
}

TEST(sockets, tcp_oversized_header_throws) {
	wivrn::TCPListener listener(0);
	uint16_t port = bound_port(listener);
	wivrn::TCP client(loopback6(port));
	auto [server, peer] = listener.accept<wivrn::TCP>();

	// Advertise a payload larger than the 16MiB cap. First call buffers the
	// header and returns empty; the second sees the oversize and throws.
	uint32_t huge = 32 * 1024 * 1024;
	send(client.get_fd(), &huge, sizeof(huge), MSG_NOSIGNAL);
	server.receive_raw();
	CHECK_THROWS_AS(server.receive_raw(), std::runtime_error);
}

TEST(sockets, tcp_shutdown_raises) {
	wivrn::TCPListener listener(0);
	uint16_t port = bound_port(listener);
	wivrn::TCP client(loopback6(port));
	auto [server, peer] = listener.accept<wivrn::TCP>();
	{
		wivrn::TCP gone = std::move(client); // closes on scope exit
	}
	CHECK_THROWS_AS(server.receive_raw(), wivrn::socket_shutdown);
}

TEST(sockets, typed_socket_variant_roundtrip) {
	using Msg = std::variant<uint32_t, std::string>;
	using Sock = wivrn::typed_socket<wivrn::TCP, Msg, Msg>;

	wivrn::TCPListener listener(0);
	uint16_t port = bound_port(listener);
	Sock client(loopback6(port));
	auto [server_base, peer] = listener.accept<Sock>();
	Sock & server = server_base;

	client.send(uint32_t(77));
	client.send(std::string("typed"));

	auto m1 = server.receive();
	CHECK(m1.has_value());
	CHECK_EQ(std::get<uint32_t>(*m1), 77u);
	auto pkt = next_tcp(server);
	auto m2 = pkt.deserialize<Msg>();
	CHECK_EQ(m2.index(), 1u);
	CHECK_EQ(std::get<std::string>(m2), "typed");
}

TEST(sockets, typed_socket_udp) {
	using Msg = std::variant<uint32_t>;
	using Sock = wivrn::typed_socket<wivrn::UDP, Msg, Msg>;

	Sock rx;
	sockaddr_in6 a{};
	a.sin6_family = AF_INET6;
	a.sin6_addr = in6addr_any;
	rx.bind(a);
	uint16_t port = bound_port(rx);

	Sock tx;
	tx.connect(loopback6(port));
	tx.send(uint32_t(33));
	auto m = rx.receive();
	CHECK(m.has_value());
	CHECK_EQ(std::get<uint32_t>(*m), 33u);
}
