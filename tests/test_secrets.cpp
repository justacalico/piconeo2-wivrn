// wivrn/common/secrets.cpp: DH + PBKDF2 session-key derivation.
#include "test_framework.h"
#include "secrets.h"

TEST(secrets, both_sides_derive_same_keys) {
	crypto::key a = crypto::key::generate_x25519_keypair();
	crypto::key b = crypto::key::generate_x25519_keypair();
	crypto::key a_pub = crypto::key::from_public_key(a.public_key());
	crypto::key b_pub = crypto::key::from_public_key(b.public_key());

	// Alice: my=a, peer=b_pub. Bob: my=b, peer=a_pub.
	secrets sa(a, b_pub, "1234");
	secrets sb(b, a_pub, "1234");
	CHECK(sa.control_key == sb.control_key);
	CHECK(sa.stream_key == sb.stream_key);
	CHECK(sa.control_iv_to_headset == sb.control_iv_to_headset);
	CHECK(sa.stream_iv_header_from_headset == sb.stream_iv_header_from_headset);
}

TEST(secrets, different_pin_yields_different_keys) {
	crypto::key a = crypto::key::generate_x25519_keypair();
	crypto::key b = crypto::key::generate_x25519_keypair();
	crypto::key b_pub = crypto::key::from_public_key(b.public_key());

	secrets s1(a, b_pub, "1234");
	secrets s2(a, b_pub, "5678");
	CHECK(s1.control_key != s2.control_key);
	CHECK(s1.stream_key != s2.stream_key);
}

TEST(secrets, deterministic_for_same_inputs) {
	crypto::key a = crypto::key::generate_x25519_keypair();
	crypto::key b = crypto::key::generate_x25519_keypair();
	crypto::key b_pub = crypto::key::from_public_key(b.public_key());

	secrets s1(a, b_pub, "9999");
	secrets s2(a, b_pub, "9999");
	CHECK(s1.control_key == s2.control_key);
	CHECK(s1.stream_iv_header_to_headset == s2.stream_iv_header_to_headset);
}
