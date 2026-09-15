// packed_quaternion.h: smallest-three quaternion packing used on the wire.
#include "test_framework.h"
#include "packed_quaternion.h"

#include <cmath>

static float quat_err(const XrQuaternionf &a, const XrQuaternionf &b) {
	float d1 = std::fabs(a.x - b.x) + std::fabs(a.y - b.y) +
	           std::fabs(a.z - b.z) + std::fabs(a.w - b.w);
	float d2 = std::fabs(a.x + b.x) + std::fabs(a.y + b.y) +
	           std::fabs(a.z + b.z) + std::fabs(a.w + b.w);
	return std::min(d1, d2); // q and -q are the same rotation
}

static XrQuaternionf norm(XrQuaternionf q) {
	float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	return {q.x / n, q.y / n, q.z / n, q.w / n};
}

TEST(packed_quaternion, identity_roundtrip) {
	XrQuaternionf q{0, 0, 0, 1};
	XrQuaternionf r = pack(q);
	CHECK(quat_err(q, r) < 0.02f);
}

TEST(packed_quaternion, x_dominant) {
	XrQuaternionf q = norm({0.9f, 0.1f, 0.1f, 0.1f});
	packed_quaternion p = pack(q);
	CHECK_EQ(p.value >> 30, 0u); // largest component index stored in top bits
	CHECK(quat_err(q, p) < 0.02f);
}

TEST(packed_quaternion, y_dominant) {
	XrQuaternionf q = norm({0.1f, 0.9f, 0.1f, 0.1f});
	packed_quaternion p = pack(q);
	CHECK_EQ(p.value >> 30, 1u);
	CHECK(quat_err(q, p) < 0.02f);
}

TEST(packed_quaternion, z_dominant) {
	XrQuaternionf q = norm({0.1f, 0.1f, 0.9f, 0.1f});
	packed_quaternion p = pack(q);
	CHECK_EQ(p.value >> 30, 2u);
	CHECK(quat_err(q, p) < 0.02f);
}

TEST(packed_quaternion, w_dominant) {
	XrQuaternionf q = norm({0.1f, 0.1f, 0.1f, 0.9f});
	packed_quaternion p = pack(q);
	CHECK_EQ(p.value >> 30, 3u);
	CHECK(quat_err(q, p) < 0.02f);
}

TEST(packed_quaternion, random_rotations) {
	// Random-ish unit quaternions covering every dominant branch.
	const XrQuaternionf cases[] = {
	        norm({0.5f, 0.5f, 0.5f, 0.5f}),
	        norm({-0.7f, 0.2f, -0.1f, 0.3f}),
	        norm({0.2f, -0.8f, 0.4f, -0.1f}),
	        norm({0.3f, 0.2f, -0.75f, 0.5f}),
	        norm({-0.2f, -0.3f, -0.4f, -0.85f}),
	        norm({1.0f, 0.0f, 0.0f, 0.0f}),
	        norm({0.0f, 1.0f, 0.0f, 0.0f}),
	        norm({0.0f, 0.0f, 1.0f, 0.0f}),
	};
	for (const auto &q : cases) {
		XrQuaternionf r = pack(q);
		float len = r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w;
		CHECK_NEAR(len, 1.0f, 0.05f);
		CHECK(quat_err(q, r) < 0.02f);
	}
}
