#pragma once

#include "wivrn_client_pico.h"
#include "pico_blit.h"
#include "pico_decoder.h"
#include "pico_audio.h"
#include "wivrn_packets.h"
#include "crypto.h"
#include "pico_tracking.h"

#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <android/hardware_buffer.h>
#include <jni.h>
#include <openxr/openxr.h>

#include <atomic>
#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

struct streaming_client
{
	JavaVM * vm = nullptr;
	jobject activity = nullptr;

	std::atomic<bool> shutdown{false};

	std::unique_ptr<wivrn_session_pico> session;

	pico_blit_pipeline blit_pipeline;
	std::atomic<int> eye_width{1664};
	std::atomic<int> eye_height{1756};
	std::atomic<int> stream_eye_width{1664};
	std::atomic<int> stream_eye_height{1756};
	std::atomic<bool> resolution_dirty{false};
	std::atomic<bool> streaming{false};
	std::atomic<bool> stream_ui_visible{false};
	std::atomic<bool> stream_stalled{false};

	std::atomic<int64_t> last_shard_ns{0};
	std::atomic<int64_t> connected_ns{0};
	std::atomic<uint64_t> last_displayed_frame{0};
	std::atomic<int64_t> frame_first_displayed_ns{0};

	uint64_t stats_bytes_rx = 0;
	uint64_t stats_bytes_tx = 0;
	int64_t stats_last_time = 0;
	int stats_frame_count = 0;
	int stats_fps = 0;
	float stats_bandwidth_rx = 0;
	float stats_bandwidth_tx = 0;
	int64_t stats_last_encode_begin = 0;

	float stats_cpu_time_ms = 0;
	float stats_gpu_time_ms = 0;
	float stats_encode_ms = 0;
	float stats_send_ms = 0;
	float stats_network_ms = 0;
	float stats_decode_ms = 0;
	float stats_render_wait_ms = 0;
	float stats_blit_ms = 0;
	float stats_total_latency_ms = 0;

	static constexpr float k_pico_fov_half = 101.0f * 0.5f * 0.01745329252f;
	XrFovf eye_fov[2]{
		XrFovf{-k_pico_fov_half, k_pico_fov_half, k_pico_fov_half, -k_pico_fov_half},
		XrFovf{-k_pico_fov_half, k_pico_fov_half, k_pico_fov_half, -k_pico_fov_half},
	};

	std::mutex tracking_mutex;
	wivrn::to_headset::tracking_control tracking_control;
	bool tracking_control_received = false;

	std::mutex video_mutex;
	std::optional<wivrn::to_headset::video_stream_description> video_desc;
	bool video_ready = false;

	std::unique_ptr<pico_video_decoder> decoders[3];

	struct shard_set
	{
		uint64_t frame_index = 0;
		std::vector<std::optional<wivrn::to_headset::video_stream_data_shard>> shards;
		size_t min_for_reconstruction = 0;
		wivrn::from_headset::feedback feedback{};
		wivrn::to_headset::video_stream_data_shard::view_info_t view_info{};
		bool has_view_info = false;
		int64_t t_first_shard_ns = 0;
		int64_t t_last_shard_ns = 0;
		bool has_server_timing = false;
		int64_t encode_begin = 0;
		int64_t encode_end = 0;
		int64_t send_begin = 0;
		int64_t send_end = 0;

		void reset(uint64_t idx)
		{
			frame_index = idx;
			shards.clear();
			min_for_reconstruction = 0;
			has_view_info = false;
			t_first_shard_ns = 0;
			t_last_shard_ns = 0;
			has_server_timing = false;
		}
	};
	shard_set current_shards[3];
	shard_set next_shards[3];

	static constexpr size_t FRAME_BUFFER_SIZE = 3;

	std::mutex decoded_frame_mutex;
	std::array<std::shared_ptr<pico_decoded_frame>, FRAME_BUFFER_SIZE> decoded_frame_buffers[3];
	std::atomic<uint64_t> latest_decoded_frame_index{0};
	std::atomic<uint64_t> latest_decoded_frame_index_per_stream[3]{};

	std::shared_ptr<pico_decoded_frame> get_frame(uint64_t frame_index, int stream) const
	{
		auto &buf = decoded_frame_buffers[stream];
		auto &slot = buf[frame_index % buf.size()];
		if (slot && slot->valid && slot->frame_index == frame_index)
			return slot;
		return nullptr;
	}

	std::shared_ptr<pico_decoded_frame> get_latest_frame(int stream) const
	{
		uint64_t idx = latest_decoded_frame_index_per_stream[stream].load(std::memory_order_acquire);
		if (idx == 0)
			return nullptr;
		auto &buf = decoded_frame_buffers[stream];
		auto &slot = buf[idx % buf.size()];
		if (slot && slot->valid && slot->frame_index == idx)
			return slot;
		return nullptr;
	}

	GLuint eye_textures[3]{0, 0, 0};
	EGLImageKHR eye_egl_images[3]{EGL_NO_IMAGE_KHR, EGL_NO_IMAGE_KHR, EGL_NO_IMAGE_KHR};
	EGLImageKHR eye_prev_egl_images[3]{EGL_NO_IMAGE_KHR, EGL_NO_IMAGE_KHR, EGL_NO_IMAGE_KHR};
	AHardwareBuffer * last_hb[3]{nullptr, nullptr, nullptr};

	GLuint stream_fbo = 0;

	std::mutex audio_mutex;
	std::optional<wivrn::to_headset::audio_stream_description> audio_desc;
	std::unique_ptr<pico_audio> audio_handle;

	std::atomic<bool> microphone_enabled{false};
	std::atomic<bool> auto_reconnect{false};
	std::atomic<int64_t> time_offset_ns{0};
	std::atomic<int> bitrate_mbps{50};
	std::atomic<int> max_bitrate_mbps{50};
	std::atomic<int> current_bitrate_mbps{50};

	// Native copies of app/running-app data for the 3D streaming UI.
	struct AppEntry { std::string id; std::string name; };
	struct RunningApp { std::string name; uint32_t id; bool overlay; bool active; };
	std::mutex app_mutex;
	std::vector<AppEntry>    available_apps;
	std::vector<RunningApp>  running_apps;
	bool app_list_requested = false;
	std::string launching_app_id;   // app currently being launched (empty = idle)

	std::string server_host;
	int server_port = 0;
	bool tcp_only = false;
	std::string pairing_pin;
	std::string last_error;

	crypto::key headset_keypair;

	std::promise<std::string> pin_promise;

	std::thread network_thread;
	std::thread connect_thread;
	std::mutex connect_mutex;
	std::atomic<bool> user_initiated{false};  // true when the user manually clicked Connect

	pico_native_tracker tracker;

	streaming_client();
	~streaming_client();

	void setup_decoders();
	void setup_audio();
	bool connect_to_server();
	void try_connect();
	void run_connect_loop();
	void send_headset_info();
	void network_loop();
	void reset_stream_state();
	void send_bitrate_change(int mbps);
	// Push the current eye-foveation preference to the server as an
	// override_foveation_center packet. enabled=false tells the server to
	// track the live EYE_GAZE pose; enabled=true pins the center to the
	// supplied pitch/distance. No-op when no session is connected.
	void send_eye_foveation_override();

	void handle_packet(wivrn::to_headset::packets & packet);
	void handle_video_shard(wivrn::to_headset::video_stream_data_shard && shard);

	int64_t get_timestamp_ns()
	{
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
	}

	XrTime to_xr_time(int64_t headset_ns)
	{
		return headset_ns + time_offset_ns.load();
	}
};

extern streaming_client * g_stream;

extern PFNEGLCREATEIMAGEKHRPROC g_eglCreateImageKHR;
extern PFNEGLDESTROYIMAGEKHRPROC g_eglDestroyImageKHR;
extern PFNEGLGETNATIVECLIENTBUFFERANDROIDPROC g_eglGetNativeClientBufferANDROID;
extern PFNGLEGLIMAGETARGETTEXTURE2DOESPROC g_glEGLImageTargetTexture2DOES;
void load_egl_procs();
