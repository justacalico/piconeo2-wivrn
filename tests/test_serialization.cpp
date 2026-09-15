// wivrn/common/wivrn_serialization.h: round-trips, size accounting and error paths.
#include "test_framework.h"
#include "wivrn_serialization.h"
#include "smp.h"

#include <array>
#include <chrono>
#include <cstring>
#include <optional>
#include <variant>
#include <vector>

using wivrn::deserialization_packet;
using wivrn::deserialization_error;
using wivrn::serialization_error;
using wivrn::serialization_packet;
using wivrn::serialized_size;

namespace
{
template <typename T>
T roundtrip(const T & v)
{
	serialization_packet p;
	p.serialize(v);

	// Gather the (possibly scattered) spans into a contiguous buffer.
	std::vector<std::span<uint8_t>> & spans = p;
	size_t total = 0;
	for (auto s: spans)
		total += s.size();

	std::shared_ptr<uint8_t[]> mem(new uint8_t[total ? total : 1]);
	size_t off = 0;
	for (auto s: spans)
	{
		memcpy(mem.get() + off, s.data(), s.size());
		off += s.size();
	}
	CHECK_EQ(total, off);

	deserialization_packet dp(mem, std::span(mem.get(), total));
	return dp.deserialize<T>();
}

template <typename T>
size_t wire_size_of(const T & v)
{
	serialization_packet p;
	p.serialize(v);
	std::vector<std::span<uint8_t>> & spans = p;
	size_t total = 0;
	for (auto s: spans)
		total += s.size();
	return total;
}
} // namespace

TEST(ser, arithmetic_roundtrip) {
	CHECK_EQ(roundtrip<uint8_t>(0xab), 0xab);
	CHECK_EQ(roundtrip<int16_t>(-1234), -1234);
	CHECK_EQ(roundtrip<uint32_t>(0xdeadbeef), 0xdeadbeefu);
	CHECK_EQ(roundtrip<int64_t>(-42), -42);
	CHECK(roundtrip<float>(3.25f) == 3.25f);
	CHECK(roundtrip<double>(-1.5) == -1.5);
	CHECK_EQ(roundtrip<bool>(true), true);
	CHECK_EQ(roundtrip<bool>(false), false);
}

TEST(ser, serialized_size_matches_wire) {
	CHECK_EQ(serialized_size(uint32_t(7)), sizeof(uint32_t));
	CHECK_EQ(wire_size_of(uint32_t(7)), sizeof(uint32_t));

	std::string s = "hello";
	CHECK_EQ(serialized_size(s), wire_size_of(s));

	std::vector<int> v{1, 2, 3};
	CHECK_EQ(serialized_size(v), wire_size_of(v));

	std::optional<std::string> o = "abc";
	CHECK_EQ(serialized_size(o), wire_size_of(o));
	o.reset();
	CHECK_EQ(serialized_size(o), wire_size_of(o));
}

TEST(ser, enum_roundtrip) {
	enum class color : uint8_t { red, green, blue };
	CHECK(roundtrip(color::blue) == color::blue);
}

TEST(ser, aggregate_roundtrip) {
	struct point {
		float x, y, z;
	};
	point in{1.f, 2.f, 3.f};
	point out = roundtrip(in);
	CHECK(out.x == 1.f && out.y == 2.f && out.z == 3.f);
	CHECK_EQ(wire_size_of(in), sizeof(point));
}

TEST(ser, aggregate_with_nontrivial_member) {
	struct named {
		uint32_t id;
		std::string name;
	};
	named in{7, "seven"};
	named out = roundtrip(in);
	CHECK_EQ(out.id, 7u);
	CHECK_EQ(out.name, "seven");
}

TEST(ser, string_sizes) {
	CHECK_EQ(roundtrip(std::string()), "");

	std::string small(100, 'x');
	CHECK_EQ(roundtrip(small), small);
	// Small sizes use a single uint16.
	CHECK_EQ(wire_size_of(small), sizeof(uint16_t) + 100);

	// Over 0x7fff the size is encoded as two uint16s.
	std::string big(0x9000, 'y');
	CHECK_EQ(roundtrip(big), big);
	CHECK_EQ(wire_size_of(big), 2 * sizeof(uint16_t) + 0x9000);
}

TEST(ser, size_encoding_boundaries) {
	serialization_packet p;
	p.serialize_size(0x7ffe);
	CHECK_EQ(wire_size_of(uint8_t(0)) - 1 + 0, 0); // sanity on helper

	// 0x7fff and above take the two-word path.
	p.clear();
	p.serialize_size(0x7fff);
	{
		std::vector<std::span<uint8_t>> & spans = p;
		CHECK_EQ(spans.size(), 1u);
		CHECK_EQ(spans[0].size(), 4u);
	}
}

TEST(ser, serialize_size_overflow_throws) {
	serialization_packet p;
	CHECK_THROWS_AS(p.serialize_size(0x8000'0000), serialization_error);
}

TEST(ser, vector_trivial_roundtrip) {
	std::vector<uint32_t> in{1, 2, 3, 4, 5};
	CHECK(roundtrip(in) == in);
	CHECK(roundtrip(std::vector<uint32_t>{}) == std::vector<uint32_t>{});
}

TEST(ser, vector_nontrivial_roundtrip) {
	std::vector<std::string> in{"a", "bb", "ccc"};
	CHECK(roundtrip(in) == in);
}

TEST(ser, optional_roundtrip) {
	std::optional<uint32_t> some = 42;
	std::optional<uint32_t> none;
	CHECK(roundtrip(some) == some);
	CHECK(roundtrip(none) == none);

	std::optional<std::string> s = "hi";
	CHECK(roundtrip(s) == s);
}

TEST(ser, array_roundtrip) {
	std::array<int16_t, 4> in{1, -2, 3, -4};
	CHECK(roundtrip(in) == in);

	// Large trivial arrays go through the span path.
	std::array<uint8_t, 64> big{};
	for (size_t i = 0; i < big.size(); ++i)
		big[i] = uint8_t(i);
	CHECK(roundtrip(big) == big);

	std::array<std::string, 2> strs{"x", "yy"};
	CHECK(roundtrip(strs) == strs);
}

TEST(ser, variant_roundtrip) {
	using V = std::variant<uint32_t, std::string, bool>;
	CHECK(std::get<uint32_t>(roundtrip(V{uint32_t(9)})) == 9u);
	CHECK(std::get<std::string>(roundtrip(V{std::string("v")})) == "v");
	CHECK(std::get<bool>(roundtrip(V{true})) == true);
}

TEST(ser, variant_bad_index_throws) {
	// Manually craft a variant wire blob with an out-of-range index.
	serialization_packet p;
	p.serialize<uint8_t>(42); // index 42, variant has 3 alternatives
	p.serialize<uint32_t>(0);

	std::vector<std::span<uint8_t>> & spans = p;
	size_t total = 0;
	for (auto s: spans)
		total += s.size();
	std::shared_ptr<uint8_t[]> mem(new uint8_t[total]);
	size_t off = 0;
	for (auto s: spans)
	{
		memcpy(mem.get() + off, s.data(), s.size());
		off += s.size();
	}
	deserialization_packet dp(mem, std::span(mem.get(), total));
	CHECK_THROWS_AS((dp.deserialize<std::variant<uint32_t, std::string, bool>>()),
	                deserialization_error);
}

TEST(ser, duration_roundtrip) {
	using ns = std::chrono::nanoseconds;
	CHECK_EQ(roundtrip(ns{123456789}).count(), 123456789);
}

TEST(ser, span_roundtrip) {
	std::vector<uint8_t> data{10, 20, 30, 40};
	std::span<uint8_t> in(data);
	serialization_packet p;
	p.serialize(in);

	std::vector<std::span<uint8_t>> & spans = p;
	size_t total = 0;
	for (auto s: spans)
		total += s.size();
	std::shared_ptr<uint8_t[]> mem(new uint8_t[total]);
	size_t off = 0;
	for (auto s: spans)
	{
		memcpy(mem.get() + off, s.data(), s.size());
		off += s.size();
	}
	deserialization_packet dp(mem, std::span(mem.get(), total));
	auto out = dp.deserialize<std::span<uint8_t>>();
	CHECK_EQ(out.size(), data.size());
	CHECK(memcmp(out.data(), data.data(), data.size()) == 0);
}

TEST(ser, truncated_buffer_throws) {
	uint32_t v = 0x11223344;
	serialization_packet p;
	p.serialize(v);
	std::vector<std::span<uint8_t>> & spans = p;
	// Keep only 2 of the 4 bytes.
	std::shared_ptr<uint8_t[]> mem(new uint8_t[2]);
	memcpy(mem.get(), spans[0].data(), 2);
	deserialization_packet dp(mem, std::span(mem.get(), 2));
	CHECK_THROWS_AS(dp.deserialize<uint32_t>(), deserialization_error);
	CHECK(!dp.empty());
}

TEST(ser, truncated_string_throws) {
	// Header says 100 bytes but the buffer ends early.
	serialization_packet p;
	p.serialize_size(100);
	std::vector<std::span<uint8_t>> & spans = p;
	std::shared_ptr<uint8_t[]> mem(new uint8_t[spans[0].size()]);
	memcpy(mem.get(), spans[0].data(), spans[0].size());
	deserialization_packet dp(mem, std::span(mem.get(), spans[0].size()));
	CHECK_THROWS_AS(dp.deserialize<std::string>(), deserialization_error);
}

TEST(ser, bignum_roundtrip) {
	crypto::bignum in(0x12345);
	crypto::bignum out = roundtrip(in);
	CHECK(out.to_hex() == in.to_hex());
}

TEST(ser, packet_clear_and_reuse) {
	serialization_packet p;
	p.serialize(uint32_t(1));
	{
		std::vector<std::span<uint8_t>> & spans = p;
		CHECK_EQ(spans.size(), 1u);
	}
	p.clear();
	{
		std::vector<std::span<uint8_t>> & spans = p;
		CHECK(spans.empty());
	}
	p.serialize(uint16_t(2));
	CHECK(roundtrip(uint16_t(2)) == 2);
}

TEST(ser, type_hash_stability) {
	constexpr uint64_t h1 = wivrn::serialization_type_hash<uint32_t>(0);
	constexpr uint64_t h2 = wivrn::serialization_type_hash<uint32_t>(0);
	CHECK_EQ(h1, h2);
	CHECK(wivrn::serialization_type_hash<uint32_t>(0) != wivrn::serialization_type_hash<uint64_t>(0));
	CHECK(wivrn::serialization_type_hash<uint32_t>(0) != wivrn::serialization_type_hash<uint32_t>(1));
	CHECK(wivrn::serialization_type_hash<int32_t>(0) != wivrn::serialization_type_hash<uint32_t>(0));
}

TEST(ser, deserialize_into_existing_and_wire_size) {
	serialization_packet p;
	p.serialize(uint32_t(0xdeadbeef));
	std::vector<std::span<uint8_t>> & spans = p;
	size_t total = 0;
	for (auto s: spans)
		total += s.size();

	std::shared_ptr<uint8_t[]> mem(new uint8_t[total]);
	size_t off = 0;
	for (auto s: spans) {
		memcpy(mem.get() + off, s.data(), s.size());
		off += s.size();
	}

	deserialization_packet dp(mem, std::span(mem.get(), total));
	CHECK_EQ(dp.wire_size(), total);
	uint32_t out = 0;
	dp.deserialize(out);   // void overload writing into an existing object
	CHECK_EQ(out, 0xdeadbeefu);
}

TEST(ser, serialized_size_of_size_boundaries) {
	CHECK_EQ(wivrn::serialized_size_of_size(0), sizeof(uint16_t));
	CHECK_EQ(wivrn::serialized_size_of_size(0x7ffe), sizeof(uint16_t));
	CHECK_EQ(wivrn::serialized_size_of_size(0x7fff), 2 * sizeof(uint16_t));
	CHECK_EQ(wivrn::serialized_size_of_size(0x7fff'fffe), 2 * sizeof(uint16_t));
	// Volatile arg stops constexpr folding so the middle branch really runs.
	volatile size_t mid = 0x8000;
	CHECK_EQ(wivrn::serialized_size_of_size(mid), 2 * sizeof(uint16_t));
	CHECK_THROWS_AS(wivrn::serialized_size_of_size(0x7fff'ffff), serialization_error);
	CHECK_THROWS_AS(wivrn::serialized_size_of_size(0x8000'0000), serialization_error);
}

TEST(ser, hash_feeds_negative_ints) {
	// The hasher's signed-int path is only exercised by negative enum values.
	wivrn::details::hash_context h;
	uint64_t before = h.hash;
	CHECK(h.feed(int64_t(-7)) != before);
}
