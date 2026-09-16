// math3d.h: tiny column-major mat4 + Hamilton quaternion helpers.
#include "test_framework.h"
#include "math3d.h"

static bool mat_eq(const Mat4 &a, const Mat4 &b, float tol = 1e-5f) {
	for (int i = 0; i < 16; i++)
		if (std::fabs(a.m[i] - b.m[i]) > tol)
			return false;
	return true;
}

TEST(math3d, identity) {
	Mat4 m = mat4Identity();
	for (int i = 0; i < 16; i++)
		CHECK_EQ(m.m[i], (i % 5 == 0) ? 1.0f : 0.0f);
}

TEST(math3d, mul_identity) {
	Mat4 t = mat4Translate(1.5f, -2.0f, 3.25f);
	Mat4 i = mat4Identity();
	CHECK(mat_eq(mat4Mul(t, i), t));
	CHECK(mat_eq(mat4Mul(i, t), t));
}

TEST(math3d, mul_translate_compose) {
	// T(a) * T(b) should translate by a+b.
	Mat4 a = mat4Translate(1, 2, 3);
	Mat4 b = mat4Translate(10, -20, 5);
	Mat4 r = mat4Mul(a, b);
	CHECK_EQ(r.m[12], 11.0f);
	CHECK_EQ(r.m[13], -18.0f);
	CHECK_EQ(r.m[14], 8.0f);
}

TEST(math3d, translate_sets_column) {
	Mat4 t = mat4Translate(-4.5f, 0.25f, 9.0f);
	CHECK_EQ(t.m[12], -4.5f);
	CHECK_EQ(t.m[13], 0.25f);
	CHECK_EQ(t.m[14], 9.0f);
	// and the rest is identity
	CHECK_EQ(t.m[0], 1.0f);
	CHECK_EQ(t.m[15], 1.0f);
}

TEST(math3d, perspective_shape) {
	Mat4 p = mat4Perspective(1.5707963f /*90deg*/, 2.0f, 0.1f, 100.0f);
	// f = 1/tan(45deg) = 1
	CHECK_NEAR(p.m[0], 0.5f, 1e-4f);   // f/aspect
	CHECK_NEAR(p.m[5], 1.0f, 1e-4f);   // f
	CHECK_NEAR(p.m[10], (100.1f) / (0.1f - 100.0f), 1e-4f);
	CHECK_EQ(p.m[11], -1.0f);
	CHECK_NEAR(p.m[14], (2.0f * 100.0f * 0.1f) / (0.1f - 100.0f), 1e-4f);
}

TEST(math3d, quat_to_mat4_identity) {
	Mat4 r = quatToMat4(0, 0, 0, 1);
	CHECK(mat_eq(r, mat4Identity()));
}

TEST(math3d, quat_to_mat4_90z) {
	// 90 deg around +Z maps +X to +Y.
	float s = std::sin(3.14159265f / 4.0f), c = std::cos(3.14159265f / 4.0f);
	Mat4 r = quatToMat4(0, 0, s, c);
	// Column 0 = rotated +X basis vector.
	CHECK_NEAR(r.m[0], 0.0f, 1e-6f);
	CHECK_NEAR(r.m[1], 1.0f, 1e-6f);
	CHECK_NEAR(r.m[2], 0.0f, 1e-6f);
	// Column 1 = rotated +Y -> -X.
	CHECK_NEAR(r.m[4], -1.0f, 1e-6f);
	CHECK_NEAR(r.m[5], 0.0f, 1e-6f);
}

TEST(math3d, transpose3x3_inverts_rotation) {
	float s = std::sin(0.4f), c = std::cos(0.4f);
	Mat4 r = quatToMat4(0, s, 0, c); // some Y rotation
	Mat4 inv = mat4Transpose3x3(r);
	Mat4 prod = mat4Mul(r, inv);
	CHECK(mat_eq(prod, mat4Identity(), 1e-5f));
}

TEST(math3d, quat_conjugate) {
	Quat q{0.1f, -0.2f, 0.3f, 0.9f};
	Quat c = quatConj(q);
	CHECK_EQ(c.x, -0.1f);
	CHECK_EQ(c.y, 0.2f);
	CHECK_EQ(c.z, -0.3f);
	CHECK_EQ(c.w, 0.9f);
	// q * conj(q) = identity
	Quat p = quatMul(quatNorm(q), quatConj(quatNorm(q)));
	CHECK_NEAR(p.x, 0, 1e-5f);
	CHECK_NEAR(p.y, 0, 1e-5f);
	CHECK_NEAR(p.z, 0, 1e-5f);
	CHECK_NEAR(p.w, 1, 1e-5f);
}

TEST(math3d, quat_mul_associativity_and_basis) {
	// Rotating 90deg about Z then 90deg about X is a known composite.
	float h = std::sin(3.14159265f / 4.0f), hw = std::cos(3.14159265f / 4.0f);
	Quat z90{0, 0, h, hw};
	Quat x90{h, 0, 0, hw};
	Quat both = quatMul(x90, z90); // apply z90 first, then x90
	// Rotate +X: z90 sends it to +Y, x90 sends +Y to +Z => +X -> +Z
	float v[3] = {1, 0, 0}, out[3];
	quatRotateVec(both, v, out);
	CHECK_NEAR(out[0], 0, 1e-5f);
	CHECK_NEAR(out[1], 0, 1e-5f);
	CHECK_NEAR(out[2], 1, 1e-5f);
}

TEST(math3d, quat_to_mat3_matches_mat4) {
	Quat q = quatNorm({0.3f, -0.4f, 0.2f, 0.8f});
	float m3[9];
	quatToMat3(q, m3);
	Mat4 m4 = quatToMat4(q.x, q.y, q.z, q.w);
	CHECK_NEAR(m3[0], m4.m[0], 1e-6f);
	CHECK_NEAR(m3[4], m4.m[5], 1e-6f);
	CHECK_NEAR(m3[8], m4.m[10], 1e-6f);
	CHECK_NEAR(m3[1], m4.m[1], 1e-6f);
	CHECK_NEAR(m3[3], m4.m[4], 1e-6f);
}

TEST(math3d, quat_norm) {
	Quat n = quatNorm({1, 2, 3, 4});
	float len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z + n.w * n.w);
	CHECK_NEAR(len, 1.0f, 1e-6f);
	// degenerate -> identity
	Quat z = quatNorm({0, 0, 0, 0});
	CHECK_EQ(z.w, 1.0f);
	CHECK_EQ(z.x, 0.0f);
}

TEST(math3d, quat_rotate_vec_aliasing) {
	// out may alias v
	float v[3] = {0, 1, 0};
	Quat z90{0, 0, std::sin(3.14159265f / 4.0f), std::cos(3.14159265f / 4.0f)};
	quatRotateVec(z90, v, v);
	CHECK_NEAR(v[0], -1.0f, 1e-5f);
	CHECK_NEAR(v[1], 0.0f, 1e-5f);
	CHECK_NEAR(v[2], 0.0f, 1e-5f);
}

TEST(math3d, quat_scale_angle) {
	// k=1 returns the same rotation; k=0 identity; k=2 doubles the angle.
	float h = std::sin(0.3f), hw = std::cos(0.3f); // 0.6 rad about Y
	Quat q{0, h, 0, hw};
	Quat same = quatScaleAngle(q, 1.0f);
	float v[3] = {0, 0, -1}, a[3], b[3];
	quatRotateVec(quatNorm(same), v, a);
	quatRotateVec(q, v, b);
	CHECK_NEAR(a[0], b[0], 1e-4f);
	CHECK_NEAR(a[2], b[2], 1e-4f);

	Quat id = quatScaleAngle(q, 0.0f);
	quatRotateVec(id, v, a);
	CHECK_NEAR(a[0], v[0], 1e-4f);
	CHECK_NEAR(a[2], v[2], 1e-4f);

	// Negative-w quaternion (same rotation) scales identically.
	Quat qn{-q.x, -q.y, -q.z, -q.w};
	Quat s2 = quatScaleAngle(qn, 1.0f);
	quatRotateVec(s2, v, a);
	CHECK_NEAR(a[0], b[0], 1e-4f);

	// Near-identity quat (tiny s) must not produce NaN.
	Quat tiny = quatNorm({1e-9f, 0, 0, 1.0f});
	Quat r = quatScaleAngle(tiny, 5.0f);
	CHECK(std::isfinite(r.w));
	CHECK_NEAR(r.w, 1.0f, 1e-4f);
}
