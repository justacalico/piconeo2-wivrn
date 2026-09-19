// wivrn/common/smp.cpp: the Socialist Millionaire Protocol used for PIN
// pairing. Runs both parties (alice + bob) through the full exchange.
#include "test_framework.h"
#include "faults.h"
#include "smp.h"

#include <openssl/bn.h>

#include <atomic>
#include <thread>

using crypto::bignum;
using crypto::smp;
using crypto::smp_cheated;

// The comparison/subtraction operators live in smp.cpp but are not declared in
// smp.h; declare them here so tests exercise the real implementations.
namespace crypto {
bignum operator-(const bignum & a, const bignum & b);
std::strong_ordering operator<=>(const bignum & a, const bignum & b);
bool operator==(const bignum & a, const bignum & b);
bignum powm(const bignum & b, const bignum & e, const bignum & m);
} // namespace crypto

// Full 5-step exchange; returns bob's verdict.
static bool run_smp(const std::string &alice_secret, const std::string &bob_secret,
                    bool *alice_ok = nullptr) {
	smp alice, bob;
	auto m1 = alice.step1(alice_secret);
	auto m2 = bob.step2(m1, bob_secret);
	auto m3 = alice.step3(m2);
	auto [m4, bob_match] = bob.step4(m3);
	if (alice_ok)
		*alice_ok = alice.step5(m4);
	return bob_match;
}

TEST(smp, matching_secrets_pair) {
	bool alice_ok = false;
	bool bob_ok = run_smp("1234", "1234", &alice_ok);
	CHECK(bob_ok);
	CHECK(alice_ok);
}

TEST(smp, mismatched_secrets_reject) {
	bool alice_ok = true;
	bool bob_ok = run_smp("1234", "4321", &alice_ok);
	CHECK(!bob_ok);
	CHECK(!alice_ok);
}

TEST(smp, empty_vs_nonempty_secret) {
	CHECK(!run_smp("", "1234"));
}

TEST(smp, step2_rejects_bad_group_elements) {
	smp alice, bob;
	auto m1 = alice.step1("1234");

	// g2a = 0 -> check_group_elem fails.
	auto bad = m1;
	bad[0] = bignum(0);
	CHECK_THROWS_AS(bob.step2(bad, "1234"), smp_cheated);

	// g3a = 1 -> also rejected.
	bad = m1;
	bad[3] = bignum(1);
	CHECK_THROWS_AS(bob.step2(bad, "1234"), smp_cheated);
}

TEST(smp, step2_rejects_forged_proof) {
	smp alice, bob;
	auto m1 = alice.step1("1234");
	auto bad = m1;
	bad[2] = bignum(42); // corrupt d2: knowledge proof fails
	CHECK_THROWS_AS(bob.step2(bad, "1234"), smp_cheated);
}

TEST(smp, step3_rejects_forged_proof) {
	smp alice, bob;
	auto m1 = alice.step1("1234");
	auto m2 = bob.step2(m1, "1234");
	m2[8] = bignum(7); // corrupt cp: equal-coords proof fails
	CHECK_THROWS_AS(alice.step3(m2), smp_cheated);
}

TEST(smp, step3_rejects_forged_log_proof) {
	smp alice, bob;
	auto m1 = alice.step1("1234");
	auto m2 = bob.step2(m1, "1234");
	m2[1] = bignum(42); // corrupt c3: knowledge-of-log proof fails
	CHECK_THROWS_AS(alice.step3(m2), smp_cheated);
}

TEST(smp, step3_rejects_out_of_range) {
	smp alice, bob;
	auto m1 = alice.step1("1234");
	auto m2 = bob.step2(m1, "1234");
	auto bad = m2;
	bad[6] = bignum(0); // pb not a group element
	CHECK_THROWS_AS(alice.step3(bad), smp_cheated);
}

TEST(smp, step4_rejects_forged_proof) {
	smp alice, bob;
	auto m1 = alice.step1("1234");
	auto m2 = bob.step2(m1, "1234");
	auto m3 = alice.step3(m2);
	m3[6] = bignum(5); // corrupt cr: equal-logs proof fails
	CHECK_THROWS_AS(bob.step4(m3), smp_cheated);
}

TEST(smp, step4_rejects_bad_group_elements) {
	smp alice, bob;
	auto m1 = alice.step1("1234");
	auto m2 = bob.step2(m1, "1234");
	auto m3 = alice.step3(m2);

	auto bad = m3;
	bad[0] = bignum(0); // pa not a group element
	CHECK_THROWS_AS(bob.step4(bad), smp_cheated);

	bad = m3;
	bad[3] = bignum(1); // d6 not a valid exponent
	CHECK_THROWS_AS(bob.step4(bad), smp_cheated);
}

TEST(smp, step4_rejects_forged_coords_proof) {
	smp alice, bob;
	auto m1 = alice.step1("1234");
	auto m2 = bob.step2(m1, "1234");
	auto m3 = alice.step3(m2);
	m3[2] = bignum(42); // corrupt cp: equal-coords proof fails
	CHECK_THROWS_AS(bob.step4(m3), smp_cheated);
}

TEST(smp, step5_rejects_forged_proof) {
	smp alice, bob;
	auto m1 = alice.step1("1234");
	auto m2 = bob.step2(m1, "1234");
	auto m3 = alice.step3(m2);
	auto [m4, ok] = bob.step4(m3);
	CHECK(ok);
	m4[1] = bignum(9); // corrupt cr
	CHECK_THROWS_AS(alice.step5(m4), smp_cheated);
}

TEST(smp, step5_rejects_bad_group_elements) {
	smp alice, bob;
	auto m1 = alice.step1("1234");
	auto m2 = bob.step2(m1, "1234");
	auto m3 = alice.step3(m2);
	auto [m4, ok] = bob.step4(m3);
	CHECK(ok);

	auto bad = m4;
	bad[0] = bignum(0); // rb not a group element
	CHECK_THROWS_AS(alice.step5(bad), smp_cheated);

	bad = m4;
	bad[2] = bignum(1); // d7 not a valid exponent
	CHECK_THROWS_AS(alice.step5(bad), smp_cheated);
}

TEST(smp, bignum_conversions) {
	bignum a = bignum::from_hex("DEADBEEF0123456789");
	CHECK_EQ(a.to_hex(), "DEADBEEF0123456789");

	std::string data = a.to_data();
	bignum b = bignum::from_data(data);
	CHECK(a == b);

	std::string mpi = a.to_mpi();
	bignum c = bignum::from_mpi(mpi);
	CHECK(a == c);

	CHECK_EQ(a.data_size(), data.size());

	// A zero bignum serializes to empty data; BN_bn2bin returning 0 bytes is
	// not an error there.
	bignum zero(0);
	CHECK(zero.to_data().empty());
	CHECK_EQ(bignum::from_data(zero.to_data()), zero);
}

TEST(smp, bignum_from_mpi_garbage_throws) {
	// A 4-byte length prefix claiming a huge number -> BN_mpi2bn fails.
	CHECK_THROWS_AS(bignum::from_mpi(std::string("\xff\xff\xff\xff", 4)),
	                std::runtime_error);
}

TEST(smp, bignum_comparison_and_copy) {
	bignum a(5), b(9), c(5);
	CHECK(a < b);
	CHECK(b > a);
	CHECK(a == c);
	CHECK((a <=> b) == std::strong_ordering::less);
	CHECK((a <=> c) == std::strong_ordering::equal);

	bignum d = a;      // copy ctor
	bignum e;
	e = b;             // copy assign
	CHECK(d == a);
	CHECK(e == b);
	CHECK(d.is_valid());

	bignum f;          // default: invalid until used
	CHECK(!f.is_valid());
	f = std::move(d);  // move assign
	CHECK(f == a);
}

TEST(smp, bignum_subtraction) {
	bignum diff = bignum(10) - bignum(3);
	CHECK(diff == bignum(7));
}

TEST(smp, constants_well_formed) {
	// SM_ORDER must equal (p-1)/2.
	BIGNUM *q2 = BN_new();
	BN_copy(q2, *smp::SM_ORDER);
	BN_mul_word(q2, 2);
	BIGNUM *pm1 = BN_new();
	BN_copy(pm1, *smp::SM_MODULUS);
	BN_sub_word(pm1, 1);
	CHECK(BN_cmp(q2, pm1) == 0);
	BN_free(q2);
	BN_free(pm1);

	CHECK(smp::SM_GENERATOR == bignum(2));
	CHECK(smp::SM_MODULUS_MINUS_2 == smp::SM_MODULUS - bignum(2));
}

TEST(smp, bn_ctx_failure_throws) {
	// bn_ctx() is thread_local: a fresh thread calls BN_CTX_new again.
	test_fault_only = "BN_CTX_new";
	std::atomic<bool> threw{false};
	std::thread t([&] {
		try
		{
			crypto::powm(bignum(2), bignum(3), smp::SM_MODULUS);
		}
		catch (...)
		{
			threw = true;
		}
	});
	t.join();
	test_fault_only = nullptr;
	CHECK(threw);
}
