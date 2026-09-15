// core/foveation.cpp: JSON extraction of the six server foveation params.
#include "test_framework.h"
#include "foveation.h"

// Defined in stubs.cpp; feeds alvr_get_settings_json_bounded.
void test_set_settings_json(const std::string &json);

TEST(fov, defaults_on_empty_json) {
	test_set_settings_json("");
	float out[6] = {-1, -1, -1, -1, -1, -1};
	readFoveationParams(out);
	CHECK(out[0] == 0.66f);
	CHECK(out[1] == 0.6f);
	CHECK(out[2] == 0.4f);
	CHECK(out[3] == 0.1f);
	CHECK(out[4] == 6.0f);
	CHECK(out[5] == 6.0f);
}

TEST(fov, parses_all_six) {
	test_set_settings_json(
	        "{\"video\":{\"foveated_encoding\":{\"center_size_x\":0.5,"
	        "\"center_size_y\":0.45,\"center_shift_x\":0.3,\"center_shift_y\":0.2,"
	        "\"edge_ratio_x\":4.0,\"edge_ratio_y\":5.0}}}");
	float out[6];
	readFoveationParams(out);
	CHECK(out[0] == 0.5f);
	CHECK(out[1] == 0.45f);
	CHECK(out[2] == 0.3f);
	CHECK(out[3] == 0.2f);
	CHECK(out[4] == 4.0f);
	CHECK(out[5] == 5.0f);
}

TEST(fov, partial_json_mixes_parsed_and_defaults) {
	test_set_settings_json("{\"center_size_x\":0.9,\"edge_ratio_y\":3.0}");
	float out[6];
	readFoveationParams(out);
	CHECK(out[0] == 0.9f);
	CHECK(out[1] == 0.6f);  // default
	CHECK(out[5] == 3.0f);
	CHECK(out[4] == 6.0f);  // default
}

TEST(fov, reordered_keys_use_fallback_scan) {
	// Keys after the cursor fall back to a full scan from the top.
	test_set_settings_json(
	        "{\"edge_ratio_y\":7.0,\"center_size_x\":0.8,\"center_size_y\":0.7}");
	float out[6];
	readFoveationParams(out);
	CHECK(out[0] == 0.8f);
	CHECK(out[1] == 0.7f);
	CHECK(out[5] == 7.0f);
}

TEST(fov, negative_and_zero_values_parse) {
	test_set_settings_json(
	        "{\"center_size_x\":0.0,\"center_shift_x\":-0.5}");
	float out[6];
	readFoveationParams(out);
	CHECK(out[0] == 0.0f);
	CHECK(out[2] == -0.5f);
}
