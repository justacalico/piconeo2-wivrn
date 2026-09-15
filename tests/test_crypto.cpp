// wivrn/common/crypto.cpp: key generation, PEM round-trips, DH, RSA KEM,
// AES-CTR contexts and PBKDF2.
#include "test_framework.h"
#include "crypto.h"

#include <openssl/evp.h>
#include <cstring>
#include <stdexcept>

TEST(crypto, x25519_keygen_and_pem_roundtrip) {
	crypto::key k = crypto::key::generate_x25519_keypair();
	CHECK(k);
	std::string pub = k.public_key();
	std::string priv = k.private_key();
	CHECK(pub.find("BEGIN PUBLIC KEY") != std::string::npos);
	CHECK(priv.find("BEGIN PRIVATE KEY") != std::string::npos);

	crypto::key pub2 = crypto::key::from_public_key(pub);
	crypto::key priv2 = crypto::key::from_private_key(priv);
	CHECK(pub2);
	CHECK(priv2);
	// Public half re-exported from the private key must match.
	CHECK_EQ(pub2.public_key(), priv2.public_key());
}

TEST(crypto, x448_keygen) {
	crypto::key k = crypto::key::generate_x448_keypair();
	CHECK(k);
	crypto::key pub2 = crypto::key::from_public_key(k.public_key());
	CHECK(pub2);
}

TEST(crypto, rsa_keygen) {
	crypto::key k = crypto::key::generate_rsa_keypair(1024);
	CHECK(k);
	CHECK(k.private_key().find("BEGIN PRIVATE KEY") != std::string::npos);
}

TEST(crypto, diffie_hellman_shared_secret) {
	crypto::key a = crypto::key::generate_x25519_keypair();
	crypto::key b = crypto::key::generate_x25519_keypair();

	crypto::key a_pub = crypto::key::from_public_key(a.public_key());
	crypto::key b_pub = crypto::key::from_public_key(b.public_key());

	auto s_ab = crypto::key::diffie_hellman(a, b_pub);
	auto s_ba = crypto::key::diffie_hellman(b, a_pub);
	CHECK_EQ(s_ab.size(), 32u);
	CHECK(s_ab == s_ba);

	// A third party gets a different secret.
	crypto::key c = crypto::key::generate_x25519_keypair();
	crypto::key c_pub = crypto::key::from_public_key(c.public_key());
	auto s_ac = crypto::key::diffie_hellman(a, c_pub);
	CHECK(s_ac != s_ab);
}

TEST(crypto, rsa_encapsulate_decapsulate) {
	crypto::key k = crypto::key::generate_rsa_keypair(1024);
	auto ws = k.encapsulate();
	CHECK(!ws.wrapped.empty());
	CHECK(ws.secret.size() > 0);

	crypto::key priv = crypto::key::from_private_key(k.private_key());
	auto secret = priv.decapsulate(ws.wrapped);
	CHECK(secret == ws.secret);
}

TEST(crypto, aes_ctr_roundtrip) {
	crypto::encrypt_context enc{EVP_aes_128_ctr()};
	CHECK(enc);
	CHECK_EQ(enc.key_length(), 16u);
	CHECK_EQ(enc.iv_length(), 16u);
	CHECK_EQ(enc.block_size(), 1u); // CTR is a stream mode

	std::vector<uint8_t> key(16, 0x42), iv(16, 0x07);
	enc.set_key_and_iv(key, iv);

	std::vector<uint8_t> plain = {'h', 'e', 'l', 'l', 'o', ' ', 'w', 'i', 'v', 'r', 'n'};
	auto ct = enc.encrypt(plain);
	CHECK(ct.size() >= plain.size());
	CHECK(ct != plain);

	crypto::decrypt_context dec{EVP_aes_128_ctr()};
	dec.set_key_and_iv(key, iv);
	auto pt = dec.decrypt(ct);
	pt.resize(plain.size());
	CHECK(pt == plain);
}

TEST(crypto, encrypt_in_place) {
	crypto::encrypt_context enc{EVP_aes_128_ctr()};
	std::vector<uint8_t> key(16, 0x11), iv(16, 0x22);
	enc.set_key_and_iv(key, iv);

	std::vector<uint8_t> buf(64);
	for (size_t i = 0; i < buf.size(); i++)
		buf[i] = (uint8_t)i;
	std::vector<uint8_t> orig = buf;

	enc.encrypt_in_place(std::span<uint8_t>(buf));
	CHECK(buf != orig);

	crypto::decrypt_context dec{EVP_aes_128_ctr()};
	dec.set_key_and_iv(key, iv);
	dec.decrypt_in_place(std::span<uint8_t>(buf));
	CHECK(buf == orig);
}

TEST(crypto, encrypt_in_place_multi_span) {
	crypto::encrypt_context enc{EVP_aes_128_ctr()};
	std::vector<uint8_t> key(16, 0x5a), iv(16, 0x33);
	enc.set_key_and_iv(key, iv);

	std::vector<uint8_t> a(16, 0xaa), b(16, 0xbb);
	std::vector<uint8_t> joined = a;
	joined.insert(joined.end(), b.begin(), b.end());

	std::vector<std::span<uint8_t>> spans = {std::span<uint8_t>(a), std::span<uint8_t>(b)};
	enc.encrypt_in_place(std::span<std::span<uint8_t>>(spans));

	crypto::decrypt_context dec{EVP_aes_128_ctr()};
	dec.set_key_and_iv(key, iv);
	std::vector<std::span<uint8_t>> back = {std::span<uint8_t>(a), std::span<uint8_t>(b)};
	dec.decrypt_in_place(std::span<std::span<uint8_t>>(back));

	std::vector<uint8_t> out = a;
	out.insert(out.end(), b.begin(), b.end());
	CHECK(out == joined);
}

TEST(crypto, block_cipher_rejects_in_place) {
	// AES-CBC has block_size > 1, encrypt_in_place must refuse.
	crypto::encrypt_context enc{EVP_aes_128_cbc()};
	std::vector<uint8_t> key(16, 0), iv(16, 0);
	enc.set_key_and_iv(key, iv);
	std::vector<uint8_t> buf(16);
	CHECK_THROWS_AS(enc.encrypt_in_place(std::span<uint8_t>(buf)), std::runtime_error);

	crypto::decrypt_context dec{EVP_aes_128_cbc()};
	dec.set_key_and_iv(key, iv);
	CHECK_THROWS_AS(dec.decrypt_in_place(std::span<uint8_t>(buf)), std::runtime_error);
}

TEST(crypto, wrong_key_or_iv_length_throws) {
	crypto::encrypt_context enc{EVP_aes_128_ctr()};
	std::vector<uint8_t> good_key(16, 0), good_iv(16, 0);
	std::vector<uint8_t> short_key(8, 0), short_iv(8, 0);

	CHECK_THROWS_AS(enc.set_key(short_key), std::invalid_argument);
	CHECK_THROWS_AS(enc.set_iv(short_iv), std::invalid_argument);
	CHECK_THROWS_AS(enc.set_key_and_iv(short_key, good_iv), std::invalid_argument);
	CHECK_THROWS_AS(enc.set_key_and_iv(good_key, short_iv), std::invalid_argument);
}

TEST(crypto, uninit_context_throws) {
	crypto::encrypt_context enc; // default: ctx == nullptr
	CHECK(!enc);
	std::vector<uint8_t> key(16, 0), iv(16, 0);
	CHECK_THROWS_AS(enc.set_key(key), std::invalid_argument);
	CHECK_THROWS_AS(enc.set_iv(iv), std::invalid_argument);
	CHECK_THROWS_AS(enc.set_key_and_iv(key, iv), std::invalid_argument);
}

TEST(crypto, pbkdf2_deterministic) {
	std::vector<uint8_t> secret(32, 0xab);
	auto a = crypto::pbkdf2("1234", "saltsalt", secret, 48);
	auto b = crypto::pbkdf2("1234", "saltsalt", secret, 48);
	CHECK_EQ(a.size(), 48u);
	CHECK(a == b);
	// Different PIN -> different output.
	auto c = crypto::pbkdf2("9999", "saltsalt", secret, 48);
	CHECK(a != c);
}

TEST(crypto, key_move_semantics) {
	crypto::key a = crypto::key::generate_x25519_keypair();
	std::string pub = a.public_key();
	crypto::key b = std::move(a);
	CHECK(b);
	CHECK_EQ(b.public_key(), pub);

	crypto::key c;
	c = std::move(b);
	CHECK(c);
	CHECK(!b);
}

TEST(crypto, bad_pem_throws) {
	CHECK_THROWS(crypto::key::from_public_key("not a pem"));
	CHECK_THROWS(crypto::key::from_private_key("not a pem"));
}

TEST(crypto, mismatched_dh_throws) {
	crypto::key a = crypto::key::generate_x25519_keypair();
	crypto::key rsa = crypto::key::generate_rsa_keypair(1024);
	// X25519 private + RSA public: derive must fail.
	CHECK_THROWS(crypto::key::diffie_hellman(a, rsa));
}

TEST(crypto, kem_on_wrong_key_type_throws) {
	crypto::key a = crypto::key::generate_x25519_keypair();
	CHECK_THROWS(a.encapsulate());
	CHECK_THROWS(a.decapsulate(std::span<uint8_t>()));
}

TEST(crypto, decapsulate_garbage_throws_or_rejects) {
	crypto::key k = crypto::key::generate_rsa_keypair(1024);
	std::vector<uint8_t> junk(4, 0xff); // way too short for an RSA-wrapped blob
	CHECK_THROWS(k.decapsulate(junk));
}

TEST(crypto, cbc_encrypt_decrypt_roundtrip) {
	crypto::encrypt_context enc{EVP_aes_128_cbc()};
	CHECK_EQ(enc.block_size(), 16u);
	std::vector<uint8_t> key(16, 0x31), iv(16, 0x77);
	enc.set_key_and_iv(key, iv);

	std::vector<uint8_t> plain(37, 0x5c); // not a block multiple: exercises padding
	auto ct = enc.encrypt(plain);
	CHECK(ct.size() >= plain.size());

	crypto::decrypt_context dec{EVP_aes_128_cbc()};
	dec.set_key_and_iv(key, iv);
	auto pt = dec.decrypt(ct);
	pt.resize(plain.size());
	CHECK(pt == plain);
}

TEST(crypto, block_cipher_rejects_multi_span_in_place) {
	crypto::encrypt_context enc(EVP_aes_128_cbc());
	std::array<uint8_t, 16> key{}, iv{};
	enc.set_key_and_iv(key, iv);

	uint8_t a[16] = {}, b[16] = {};
	std::span<uint8_t> spans[2] = {std::span(a), std::span(b)};
	CHECK_THROWS_AS(enc.encrypt_in_place(std::span(spans)), std::runtime_error);

	crypto::decrypt_context dec(EVP_aes_128_cbc());
	dec.set_key_and_iv(key, iv);
	CHECK_THROWS_AS(dec.decrypt_in_place(std::span(spans)), std::runtime_error);
}
