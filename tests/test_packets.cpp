// wivrn/common/wivrn_packets.h: every variant alternative round-trips through
// the serializer unchanged. Also covers to_pose_flags and xr::to_string.
#include "test_framework.h"
#include "wivrn_packets.h"
#include "wivrn_serialization.h"

#include <cstring>
#include <span>
#include <vector>

using wivrn::deserialization_packet;
using wivrn::serialization_packet;

namespace
{
std::vector<uint8_t> wire_bytes(const auto & v)
{
	serialization_packet p;
	p.serialize(v);
	std::vector<uint8_t> out;
	for (auto s: std::vector<std::span<uint8_t>>(p))
		out.insert(out.end(), s.begin(), s.end());
	return out;
}

// Variant alternative -> wire -> variant -> wire. Byte equality proves the
// aggregate round-tripped losslessly.
template <typename V, typename T>
void variant_rt(const T & v)
{
	V var{v};
	auto bytes = wire_bytes(var);

	std::shared_ptr<uint8_t[]> mem(new uint8_t[bytes.size() ? bytes.size() : 1]);
	memcpy(mem.get(), bytes.data(), bytes.size());
	deserialization_packet dp(mem, std::span(mem.get(), bytes.size()));
	V var2 = dp.deserialize<V>();

	CHECK_EQ(wire_bytes(var2), bytes);
	CHECK_EQ(var2.index(), var.index());
}
} // namespace

using namespace wivrn;
namespace fh = wivrn::from_headset;
namespace th = wivrn::to_headset;

TEST(packets, from_headset_handshake_and_crypto)
{
	fh::crypto_handshake ch{};
	ch.protocol_version = 0x1234;
	ch.public_key = "-----BEGIN PUBLIC KEY-----";
	ch.name = "test-headset";
	variant_rt<fh::packets>(ch);

	fh::pin_check_1 p1{};
	p1.message.fill(crypto::bignum(7));
	variant_rt<fh::packets>(p1);
	fh::pin_check_3 p3{};
	p3.message.fill(crypto::bignum(9));
	variant_rt<fh::packets>(p3);
	variant_rt<fh::packets>(fh::handshake{});
}

TEST(packets, from_headset_headset_info)
{
	fh::headset_info_packet hi{};
	hi.render_eye_width = 1440;
	hi.render_eye_height = 1600;
	hi.stream_eye_width = 720;
	hi.stream_eye_height = 800;
	hi.available_refresh_rates = {60.f, 72.f, 90.f};
	hi.settings.preferred_refresh_rate = 90.f;
	hi.settings.bitrate_bps = 40'000'000;
	hi.settings.fps_divider = 2;
	hi.speaker = fh::headset_info_packet::audio_description{2, 48000};
	hi.microphone = std::nullopt;
	hi.fov[0] = {-0.9f, 0.9f, -1.0f, 1.0f};
	hi.hand_tracking = true;
	hi.eye_gaze = true;
	hi.face_tracking = fh::face_type::htc;
	hi.body_tracking = fh::body_type::meta;
	hi.num_generic_trackers = 3;
	hi.supported_codecs = {video_codec::h264, video_codec::h265};
	hi.bit_depth = 10;
	hi.system_name = "Pico Neo 2";
	hi.language = "en";
	hi.country = "US";
	hi.variant = "eye";
	variant_rt<fh::packets>(hi);

	// And once more with audio both ways empty and no bit depth.
	fh::headset_info_packet hi2{};
	variant_rt<fh::packets>(hi2);
}

TEST(packets, from_headset_feedback_and_state)
{
	fh::feedback fb{};
	fb.frame_index = 42;
	fb.stream_index = 1;
	fb.encode_begin = 1;
	fb.encode_end = 2;
	fb.send_begin = 3;
	fb.send_end = 4;
	fb.received_first_packet = 5;
	fb.received_last_packet = 6;
	fb.sent_to_decoder = 7;
	fb.received_from_decoder = 8;
	fb.blitted = 9;
	fb.displayed = 10;
	fb.times_displayed = 3;
	variant_rt<fh::packets>(fb);

	variant_rt<fh::packets>(fh::settings_changed{.preferred_refresh_rate = 72.f,
	                                           .minimum_refresh_rate = 60.f,
	                                           .bitrate_bps = 12345,
	                                           .mirror_gamepad = true,
	                                           .enabled_body_parts = 0xff});

	audio_data ad{};
	variant_rt<fh::packets>(ad);

	fh::timesync_response ts{};
	ts.query = 111;
	ts.response = 222;
	variant_rt<fh::packets>(ts);

	variant_rt<fh::packets>(fh::battery{.charge = 0.5f, .present = true, .charging = true});
	variant_rt<fh::packets>(fh::refresh_rate_changed{.from = 60.f, .to = 90.f});
	variant_rt<fh::packets>(fh::session_state_changed{.state = XR_SESSION_STATE_FOCUSED});
	variant_rt<fh::packets>(fh::user_presence_changed{.present = true, .change_time = 77});
	variant_rt<fh::packets>(fh::stream_tab_changed{.tab = stream_tab::settings});
	variant_rt<fh::packets>(fh::override_foveation_center{.enabled = true, .pitch = 0.3f, .distance = 1.5f});
}

TEST(packets, from_headset_tracking_all_face_variants)
{
	fh::tracking t{};
	t.production_timestamp = 1;
	t.timestamp = 2;
	t.state_flags = fh::tracking::recentered;
	t.view_flags = XR_VIEW_STATE_ORIENTATION_VALID_BIT;
	t.interaction_profiles[0] = interaction_profile::oculus_touch_controller;
	t.views[0].pose = {{1, 2, 3, 4}, {5, 6, 7}};
	t.views[1].fov = {-1.f, 1.f, -1.f, 1.f};

	fh::tracking::pose dp{};
	dp.pose = {{0, 0, 0, 1}, {1, 2, 3}};
	dp.linear_velocity = {1, 0, 0};
	dp.device = device_id::HEAD;
	dp.flags = 0x3f;
	t.device_poses = {dp};

	t.face = fh::tracking::android_face{};
	variant_rt<fh::packets>(t);
	t.face = fh::tracking::fb_face2{};
	variant_rt<fh::packets>(t);
	t.face = fh::tracking::htc_face{};
	variant_rt<fh::packets>(t);
	t.face = std::monostate{};
	variant_rt<fh::packets>(t);
}

TEST(packets, from_headset_body_tracking)
{
	variant_rt<fh::packets>(fh::derived_pose{.source = device_id::HEAD, .target = device_id::LEFT_HAND, .relation = {}});

	fh::hand_tracking ht{};
	ht.hand = fh::hand_tracking::right;
	ht.timestamp = 9;
	variant_rt<fh::packets>(ht);   // joints absent

	fh::hand_tracking::pose jp{};
	jp.position = {1, 2, 3};
	jp.radius = 42;
	jp.flags = 0x3f;
	std::array<fh::hand_tracking::pose, XR_HAND_JOINT_COUNT_EXT> joints;
	joints.fill(jp);
	ht.joints = joints;
	variant_rt<fh::packets>(ht);   // joints present

	fh::meta_body mb{};
	mb.confidence = 0.9f;
	mb.joints = std::monostate{};
	variant_rt<fh::packets>(mb);
	mb.joints = fh::meta_body::fb_joints{};
	variant_rt<fh::packets>(mb);
	mb.joints = fh::meta_body::meta_joints{};
	variant_rt<fh::packets>(mb);

	fh::meta_body_skeleton sk{};
	sk.skeleton = fh::meta_body_skeleton::fb_skeleton{};
	variant_rt<fh::packets>(sk);
	sk.skeleton = fh::meta_body_skeleton::meta_skeleton{};
	variant_rt<fh::packets>(sk);

	fh::bd_body bd{};
	bd.all_tracked = true;
	variant_rt<fh::packets>(bd);

	fh::htc_body hb{};
	hb.timestamp = 5;
	variant_rt<fh::packets>(hb);
}

TEST(packets, from_headset_inputs_and_apps)
{
	fh::inputs in{};
	in.values = {{device_id::LEFT_TRIGGER_VALUE, 0.5f, 10}, {device_id::A_CLICK, 1.0f, 20}};
	variant_rt<fh::packets>(in);

	variant_rt<fh::packets>(fh::hid::input{fh::hid::button_down{7}});
	variant_rt<fh::packets>(fh::hid::input{fh::hid::button_up{7}});
	variant_rt<fh::packets>(fh::hid::input{fh::hid::mouse_move{1.5f, -2.5f}});
	variant_rt<fh::packets>(fh::hid::input{fh::hid::mouse_scroll{0.1f, 0.2f}});
	variant_rt<fh::packets>(fh::hid::input{fh::hid::key_down{65}});
	variant_rt<fh::packets>(fh::hid::input{fh::hid::key_up{65}});

	fh::visibility_mask_changed vm{};
	vm.view_index = 1;
	vm.data[0].vertices = {{0, 0}, {1, 0}, {0, 1}};
	vm.data[0].indices = {0, 1, 2};
	variant_rt<fh::packets>(vm);

	fh::get_application_list gal{};
	gal.language = "fr";
	gal.country = "FR";
	gal.variant = "x";
	variant_rt<fh::packets>(gal);

	fh::start_app sa{};
	sa.app_id = "com.example.app";
	variant_rt<fh::packets>(sa);
	variant_rt<fh::packets>(fh::get_running_applications{});
	variant_rt<fh::packets>(fh::set_active_application{.id = 9});
	variant_rt<fh::packets>(fh::stop_application{.id = 9});
}

TEST(packets, to_headset_handshake_and_control)
{
	th::crypto_handshake ch{};
	ch.public_key = "pem";
	ch.state = th::crypto_handshake::crypto_state::pin_needed;
	variant_rt<th::packets>(ch);

	th::pin_check_2 p2{};
	p2.message.fill(crypto::bignum(3));
	variant_rt<th::packets>(p2);
	th::pin_check_4 p4{};
	p4.message.fill(crypto::bignum(5));
	variant_rt<th::packets>(p4);
	variant_rt<th::packets>(th::handshake{.stream_port = 9757});
	variant_rt<th::packets>(th::server_message{.kind = th::server_message::kind::error, .msg = "oops"});
	variant_rt<th::packets>(th::server_message{.kind = th::server_message::kind::toast_urgent, .msg = "hi"});

	th::audio_stream_description asd{};
	asd.speaker = th::audio_stream_description::device{2, 44100};
	variant_rt<th::packets>(asd);
	asd.microphone = th::audio_stream_description::device{1, 16000};
	variant_rt<th::packets>(asd);

	th::video_stream_description vsd{};
	vsd.width = 1440;
	vsd.height = 1600;
	vsd.codec = {video_codec::h265, video_codec::h265, video_codec::raw};
	vsd.frame_rate = 90.f;
	vsd.refresh_rate = 90.f;
	variant_rt<th::packets>(vsd);
}

TEST(packets, to_headset_shards_and_haptics)
{
	th::video_stream_data_shard sh{};
	sh.stream_item_idx = 0;
	sh.frame_idx = 99;
	sh.shard_idx = 3;
	variant_rt<th::packets>(sh);   // no view/timing info

	sh.view_info = th::video_stream_data_shard::view_info_t{};
	sh.view_info->display_time = 12345;
	sh.view_info->alpha = true;
	sh.view_info->foveation[0].x = {1, 4, 5};
	sh.view_info->foveation[1].y = {2, 3};
	sh.timing_info = th::video_stream_data_shard::timing_info_t{1, 2, 3, 4};
	variant_rt<th::packets>(sh);

	th::haptics h{};
	h.id = device_id::LEFT_HAND;
	h.duration = std::chrono::nanoseconds(50'000'000);
	h.frequency = 120.f;
	h.amplitude = 0.7f;
	variant_rt<th::packets>(h);

	variant_rt<th::packets>(th::timesync_query{.query = 31337});

	th::tracking_control tc{};
	tc.pattern = {{device_id::HEAD, 1000}, {device_id::RIGHT_AIM, 2000}};
	tc.motions_to_photons = 30'000'000;
	variant_rt<th::packets>(tc);

	variant_rt<th::packets>(th::feature_control{.f = th::feature_control::microphone, .state = true});
	variant_rt<th::packets>(th::feature_control{.f = th::feature_control::hid_input, .state = false});
	variant_rt<th::packets>(th::refresh_rate_change{.hz = 72.f});
	variant_rt<th::packets>(th::stream_tab_change{.tab = stream_tab::stats});
}

TEST(packets, to_headset_applications)
{
	th::application_list al{};
	al.language = "en";
	al.country = "US";
	al.variant = "v";
	al.applications = {{"id1", "App One"}, {"id2", "App Two"}};
	variant_rt<th::packets>(al);

	th::application_icon ai{};
	ai.id = "id1";
	ai.image = {std::byte{0x89}, std::byte{0x50}, std::byte{0x4e}, std::byte{0x47}};
	variant_rt<th::packets>(ai);

	th::running_applications ra{};
	ra.applications = {{"App One", 1, false, true}, {"Overlay", 2, true, false}};
	variant_rt<th::packets>(ra);
}

TEST(packets, pose_flags_from_space_flags)
{
	CHECK_EQ(fh::to_pose_flags(0), 0);
	CHECK_EQ(fh::to_pose_flags(XR_SPACE_LOCATION_ORIENTATION_VALID_BIT), fh::pose_flags::orientation_valid);
	CHECK_EQ(fh::to_pose_flags(XR_SPACE_LOCATION_POSITION_VALID_BIT), fh::pose_flags::position_valid);
	CHECK_EQ(fh::to_pose_flags(XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT), fh::pose_flags::orientation_tracked);
	CHECK_EQ(fh::to_pose_flags(XR_SPACE_LOCATION_POSITION_TRACKED_BIT), fh::pose_flags::position_tracked);
	CHECK_EQ(fh::to_pose_flags(0, XR_SPACE_VELOCITY_LINEAR_VALID_BIT), fh::pose_flags::linear_velocity_valid);
	CHECK_EQ(fh::to_pose_flags(0, XR_SPACE_VELOCITY_ANGULAR_VALID_BIT), fh::pose_flags::angular_velocity_valid);
	CHECK_EQ(fh::to_pose_flags(XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT,
	                     XR_SPACE_VELOCITY_LINEAR_VALID_BIT | XR_SPACE_VELOCITY_ANGULAR_VALID_BIT),
	         0x0f);
}
