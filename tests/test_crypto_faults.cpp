// OpenSSL error paths in crypto.cpp / smp.cpp via --wrap fault injection.
// test_fault_in counts down over wrapped calls; when it hits 0 that call
// fails and the code under test should throw. Sweeping N covers every site.
#include "test_framework.h"
#include "faults.h"
#include "crypto.h"
#include "smp.h"

#include <openssl/evp.h>
#include <cstring>
#include <span>
#include <vector>

using crypto::bignum;

// File-local helpers in smp.cpp, declared here so fault sweeps can reach them.
namespace crypto {
bignum operator-(const bignum & a, const bignum & b);
bignum invm(const bignum & a, const bignum & n);
} // namespace crypto

TEST(crypto_faults, keygen_error_paths)
{
	CHECK(exercise_faults([] { crypto::key::generate_x25519_keypair(); }) > 0);
	CHECK(exercise_faults([] { crypto::key::generate_x448_keypair(); }) > 0);
	CHECK(exercise_faults([] { crypto::key::generate_rsa_keypair(2048); }) > 0);
}

TEST(crypto_faults, pem_read_write_error_paths)
{
	crypto::key k = crypto::key::generate_x25519_keypair();
	std::string pub = k.public_key();
	std::string priv = k.private_key();

	CHECK(exercise_faults([&] { crypto::key::from_public_key(pub); }) > 0);
	CHECK(exercise_faults([&] { crypto::key::from_private_key(priv); }) > 0);
	CHECK(exercise_faults([&] { (void)k.public_key(); }) > 0);
	CHECK(exercise_faults([&] { (void)k.private_key(); }) > 0);
}

TEST(crypto_faults, dh_and_kem_error_paths)
{
	crypto::key a = crypto::key::generate_x25519_keypair();
	crypto::key b = crypto::key::generate_x25519_keypair();
	CHECK(exercise_faults([&] { crypto::key::diffie_hellman(a, b); }) > 0);

	crypto::key rsa = crypto::key::generate_rsa_keypair(2048);
	auto ws = rsa.encapsulate();
	CHECK(exercise_faults([&] { (void)rsa.encapsulate(); }) > 0);
	CHECK(exercise_faults([&] { (void)rsa.decapsulate(ws.wrapped); }) > 0);
}

TEST(crypto_faults, cipher_error_paths)
{
	std::array<uint8_t, 16> key{}, iv{};
	std::vector<uint8_t> plain(64, 0x42);

	CHECK(exercise_faults([&] { crypto::encrypt_context{EVP_aes_128_ctr()}; }) > 0);
	CHECK(exercise_faults([&] { crypto::decrypt_context{EVP_aes_128_ctr()}; }) > 0);

	crypto::encrypt_context enc(EVP_aes_128_ctr());
	crypto::decrypt_context dec(EVP_aes_128_ctr());
	CHECK(exercise_faults([&] { enc.set_key(key); }) > 0);
	CHECK(exercise_faults([&] { enc.set_iv(iv); }) > 0);
	CHECK(exercise_faults([&] { enc.set_key_and_iv(key, iv); }) > 0);

	enc.set_key_and_iv(key, iv);
	dec.set_key_and_iv(key, iv);
	CHECK(exercise_faults([&] { (void)enc.encrypt(plain); }) > 0);
	CHECK(exercise_faults([&] { auto p = plain; enc.encrypt_in_place(std::span(p)); }) > 0);
	CHECK(exercise_faults([&] {
		uint8_t a[16] = {}, b[16] = {};
		std::span<uint8_t> ss[2] = {std::span(a), std::span(b)};
		enc.encrypt_in_place(std::span(ss));
	}) > 0);

	auto ct = enc.encrypt(plain);
	CHECK(exercise_faults([&] { (void)dec.decrypt(ct); }) > 0);
	CHECK(exercise_faults([&] { auto c = ct; dec.decrypt_in_place(std::span(c)); }) > 0);
	CHECK(exercise_faults([&] {
		uint8_t a[16] = {}, b[16] = {};
		std::span<uint8_t> ss[2] = {std::span(a), std::span(b)};
		dec.decrypt_in_place(std::span(ss));
	}) > 0);
}

TEST(crypto_faults, pbkdf2_error_paths)
{
	std::vector<uint8_t> secret(8, 1);
	CHECK(exercise_faults([&] {
		crypto::pbkdf2("pass", "salt", secret, 32);
	}) > 0);
}

TEST(crypto_faults, bignum_error_paths)
{
	CHECK(exercise_faults([] { (void)bignum(42); }) > 0);
	CHECK(exercise_faults([] { (void)bignum::from_hex("abcd"); }) > 0);
	CHECK(exercise_faults([] { (void)bignum::from_data("xyz"); }) > 0);
	std::string mpi = bignum(42).to_mpi();
	CHECK(exercise_faults([&] { (void)bignum::from_mpi(mpi); }) > 0);
	CHECK(exercise_faults([] {
		bignum a(9), b(4);
		(void)crypto::operator-(a, b);
	}) > 0);
	CHECK(exercise_faults([] {
		bignum a(3), n(17);
		(void)crypto::invm(a, n);
	}) > 0);
	CHECK(exercise_faults([] {
		bignum v = bignum::from_hex("1234");
		(void)v.to_mpi();
	}) > 0);
	CHECK(exercise_faults([] {
		bignum v = bignum::from_hex("1234");
		(void)v.to_data();
	}) > 0);
	CHECK(exercise_faults([] {
		bignum v = bignum::from_hex("1234");
		(void)v.to_hex();
	}) > 0);
}

TEST(crypto_faults, smp_error_paths)
{
	CHECK(exercise_faults([] {
		crypto::smp alice;
		alice.step1("1234");
	}) > 0);
	CHECK(exercise_faults([] {
		crypto::smp alice, bob;
		auto m1 = alice.step1("1234");
		bob.step2(m1, "1234");
	}) > 0);
	CHECK(exercise_faults([] {
		crypto::smp alice, bob;
		auto m1 = alice.step1("1234");
		auto m2 = bob.step2(m1, "1234");
		alice.step3(m2);
	}) > 0);
	CHECK(exercise_faults([] {
		crypto::smp alice, bob;
		auto m1 = alice.step1("1234");
		auto m2 = bob.step2(m1, "1234");
		auto m3 = alice.step3(m2);
		bob.step4(m3);
	}) > 0);
	CHECK(exercise_faults([] {
		crypto::smp alice, bob;
		auto m1 = alice.step1("1234");
		auto m2 = bob.step2(m1, "1234");
		auto m3 = alice.step3(m2);
		auto [m4, ok] = bob.step4(m3);
		alice.step5(m4);
	}) > 0);
}
