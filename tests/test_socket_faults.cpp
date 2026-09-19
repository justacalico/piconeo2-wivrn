// wivrn/common/wivrn_sockets.cpp: libc-level error paths via --wrap fault
// injection (socket/connect/recv/send failures) plus framing edge cases that
// need raw bytes on the wire.
#include "test_framework.h"
#include "faults.h"
#include "wivrn_sockets.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <vector>

using wivrn::deserialization_packet;
using wivrn::serialization_packet;

namespace
{
sockaddr_in6 loopback6(uint16_t port)
{
	sockaddr_in6 a{};
	a.sin6_family = AF_INET6;
	a.sin6_port = htons(port);
	inet_pton(AF_INET6, "::ffff:127.0.0.1", &a.sin6_addr);
	return a;
}

sockaddr_in loopback4(uint16_t port)
{
	sockaddr_in a{};
	a.sin_family = AF_INET;
	a.sin_port = htons(port);
	a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	return a;
}

uint16_t bound_port(wivrn::fd_base & s)
{
	sockaddr_in6 addr{};
	socklen_t len = sizeof(addr);
	getsockname(s.get_fd(), (sockaddr *)&addr, &len);
	return ntohs(addr.sin6_port);
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

// A connected TCP pair: listener + client + accepted server end.
struct tcp_pair
{
	wivrn::TCPListener listener;
	wivrn::TCP client;
	wivrn::TCP server;

	tcp_pair() :
	        listener(0),
	        client(loopback6(bound_port(listener))),
	        server(listener.accept<>().first)
	{}
};
} // namespace

TEST(socket_faults, udp_ctor_and_connect_failures)
{
	CHECK(exercise_faults([] { wivrn::UDP u; }) > 0);

	wivrn::UDP u;
	CHECK(exercise_faults([&] { u.connect(loopback6(9)); }) > 0);
	CHECK(exercise_faults([&] { u.connect(loopback4(9)); }) > 0);
	CHECK(exercise_faults([&] { u.bind(loopback6(0)); }) > 0);
}

TEST(socket_faults, tcp_ctor_and_listener_failures)
{
	// A port with no listener: connect fails fast on every swept iteration so
	// the op always throws and the sweep just exhausts its cap.
	uint16_t dead_port;
	{
		wivrn::TCPListener tmp(0);
		dead_port = bound_port(tmp);
	}

	CHECK(exercise_faults([&] { wivrn::TCP t(loopback6(dead_port)); }) > 0);
	CHECK(exercise_faults([&] { wivrn::TCP t(loopback4(dead_port)); }) > 0);
	CHECK(exercise_faults([&] { wivrn::TCPListener l(0); }) > 0);

	// accept() failing outright, and setsockopt failing inside TCP::init on an
	// otherwise successful accept.
	wivrn::TCPListener l2(0);
	wivrn::TCP pending(loopback6(bound_port(l2)));

	test_fault_in = 1;
	CHECK_THROWS_AS((void)l2.accept<>(), std::system_error);

	test_fault_in = 2;
	CHECK_THROWS_AS((void)l2.accept<>(), std::system_error);
	test_fault_in = -1;
}

TEST(socket_faults, udp_recvfrom_failures)
{
	wivrn::UDP rx = bound_udp();
	wivrn::UDP tx;

	auto send_datagram = [&] {
		serialization_packet p;
		p.serialize(uint32_t(0xdeadbeef));
		tx.send_raw(std::move(p));
	};
	tx.connect(loopback6(bound_port(rx)));

	// A pending datagram lets every swept call site run: peek fail, read fail,
	// then success.
	send_datagram();
	CHECK(exercise_faults([&] { (void)rx.receive_from_raw(); }) > 0);

	// recvmmsg failure (-1) and empty receive (0 -> socket_shutdown).
	send_datagram();
	CHECK(exercise_faults([&] { (void)rx.receive_raw(); }) > 0);
	send_datagram();
	test_fault_ret = 0;
	CHECK(exercise_faults([&] { (void)rx.receive_raw(); }) > 0);
	test_fault_ret = -1;
}

TEST(socket_faults, tcp_recv_and_sendmsg_failures)
{
	tcp_pair p;

	serialization_packet pkt;
	pkt.serialize(uint32_t(0x1234));
	p.client.send_raw(std::move(pkt));

	CHECK(exercise_faults([&] { (void)p.server.receive_raw(); }) > 0);

	// sendmsg returning 0 means the peer is gone -> socket_shutdown.
	test_fault_ret = 0;
	CHECK(exercise_faults([&] {
		serialization_packet q;
		q.serialize(uint32_t(1));
		p.client.send_raw(std::move(q));
	}) > 0);
	CHECK(exercise_faults([&] {
		std::array<serialization_packet, 2> batch;
		batch[0].serialize(uint32_t(1));
		batch[1].serialize(uint32_t(2));
		p.client.send_many_raw(batch);
	}) > 0);
	test_fault_ret = -1;

	// Hard sendmsg error also throws.
	CHECK(exercise_faults([&] {
		serialization_packet q;
		q.serialize(uint32_t(2));
		p.client.send_raw(std::move(q));
	}) > 0);
}

TEST(socket_faults, udp_sendmsg_and_sendmmsg_failures)
{
	wivrn::UDP rx = bound_udp();
	wivrn::UDP tx;
	tx.connect(loopback6(bound_port(rx)));

	CHECK(exercise_faults([&] {
		serialization_packet q;
		q.serialize(uint32_t(3));
		tx.send_raw(std::move(q));
	}) > 0);

	test_fault_ret = -1;
	CHECK(exercise_faults([&] {
		std::array<serialization_packet, 2> batch;
		batch[0].serialize(uint32_t(1));
		batch[1].serialize(uint32_t(2));
		tx.send_many_raw(batch);
	}) > 0);
}

TEST(socket_faults, tcp_zero_size_header_throws)
{
	tcp_pair p;
	uint32_t zero = 0;
	(void)::write(p.client.get_fd(), &zero, sizeof(zero));
	// Let the bytes land, then the next receive sees payload_size == 0.
	for (int i = 0; i < 1000; ++i)
	{
		try
		{
			(void)p.server.receive_raw();
		}
		catch (const std::system_error &)
		{
			usleep(1000);
			continue;
		}
		catch (const std::runtime_error & e)
		{
			CHECK(std::string(e.what()).find("0 size") != std::string::npos);
			// The bad header stays buffered: receive_pending rejects it too.
			CHECK_THROWS_AS(p.server.receive_pending(), std::runtime_error);
			return;
		}
	}
	CHECK(false);
}

TEST(socket_faults, tcp_partial_header_then_packet)
{
	tcp_pair p;

	// Send a framed packet as two segments: first carries the header plus part
	// of the payload so the parser takes the "header already buffered" branch.
	uint32_t payload = 0x01020304;
	uint32_t header = sizeof(payload);
	uint8_t wire[sizeof(header) + sizeof(payload)];
	memcpy(wire, &header, sizeof(header));
	memcpy(wire + sizeof(header), &payload, sizeof(payload));

	(void)::write(p.client.get_fd(), wire, sizeof(header) + 2);
	for (int i = 0; i < 1000; ++i)
	{
		try
		{
			(void)p.server.receive_raw();
			break;
		}
		catch (const std::system_error &)
		{
		}
		usleep(1000);
	}
	(void)::write(p.client.get_fd(), wire + sizeof(header) + 2, sizeof(payload) - 2);

	deserialization_packet pkt;
	for (int i = 0; i < 1000; ++i)
	{
		pkt = p.server.receive_pending();
		if (not pkt.empty())
			break;
		try
		{
			pkt = p.server.receive_raw();
			if (not pkt.empty())
				break;
		}
		catch (const std::system_error &)
		{
		}
		usleep(1000);
	}
	CHECK(not pkt.empty());
	if (pkt.empty())
		return;
	CHECK_EQ(pkt.deserialize<uint32_t>(), payload);
}
