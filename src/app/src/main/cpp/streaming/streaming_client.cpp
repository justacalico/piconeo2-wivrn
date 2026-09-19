#include "streaming_client.h"
#include "pico_stutter.h"
#include "latency_tracker.h"
#include "eye_tracking.h"
#include "app_state.h"   // gManualLobby, requestPinEntryUI, gOnPinSubmit
#include "input.h"       // queueHaptic
#include "server_list.h" // setConnectionError / setConnecting for lobby UI

#include <spdlog/spdlog.h>
#include <spdlog/sinks/android_sink.h>

#include <android/log.h>
#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO, "wivrn-native", __VA_ARGS__)

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fstream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
using namespace wivrn;

stutter_detector g_stutter;
latency_tracker g_latency;

streaming_client * g_stream = nullptr;

PFNEGLCREATEIMAGEKHRPROC g_eglCreateImageKHR = nullptr;
PFNEGLDESTROYIMAGEKHRPROC g_eglDestroyImageKHR = nullptr;
PFNEGLGETNATIVECLIENTBUFFERANDROIDPROC g_eglGetNativeClientBufferANDROID = nullptr;
PFNGLEGLIMAGETARGETTEXTURE2DOESPROC g_glEGLImageTargetTexture2DOES = nullptr;

void load_egl_procs()
{
	if (g_eglCreateImageKHR) return;
	g_eglCreateImageKHR = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
	g_eglDestroyImageKHR = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
	g_eglGetNativeClientBufferANDROID = (PFNEGLGETNATIVECLIENTBUFFERANDROIDPROC)eglGetProcAddress("eglGetNativeClientBufferANDROID");
	g_glEGLImageTargetTexture2DOES = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
	spdlog::info("EGL procs: create={} destroy={} getNative={} glImage={}",
		(void*)g_eglCreateImageKHR, (void*)g_eglDestroyImageKHR,
		(void*)g_eglGetNativeClientBufferANDROID, (void*)g_glEGLImageTargetTexture2DOES);
}

static void init_spdlog()
{
	if (spdlog::default_logger_raw()) return;
	ALOGI("init_spdlog: creating android logger");
	auto logger = spdlog::android_logger_mt("wivrn", "wivrn-native");
	spdlog::set_default_logger(logger);
	spdlog::set_level(spdlog::level::debug);
	ALOGI("init_spdlog: done, default_logger_raw=%p", (void*)spdlog::default_logger_raw());
}

streaming_client::streaming_client()
{
	ALOGI("streaming_client ctor: start");
	init_spdlog();
	ALOGI("streaming_client ctor: spdlog done");
	load_egl_procs();
	ALOGI("streaming_client ctor: egl done");
}

streaming_client::~streaming_client()
{
	shutdown = true;
	tracker.session = nullptr;
	tracker.stop();

	if (connect_thread.joinable())
		connect_thread.join();
	if (network_thread.joinable())
		network_thread.join();
}

void streaming_client::setup_decoders()
{
	if (!video_desc || decoders[0])
		return;

	for (uint8_t i = 0; i < 3; i++)
	{
		if (video_desc->codec[i] == video_codec::raw)
		{
			decoders[i].reset();
			continue;
		}

		decoders[i] = std::make_unique<pico_video_decoder>(
			*video_desc, i,
			[this, i](std::shared_ptr<pico_decoded_frame> frame) {
				uint64_t fi = frame->frame_index;
				g_stutter.on_frame_decoded(fi, i);
				g_latency.on_frame_decoded(fi, i);
				std::lock_guard lock(decoded_frame_mutex);
				auto &buf = decoded_frame_buffers[i];
				buf[fi % buf.size()] = std::move(frame);
				latest_decoded_frame_index.store(fi);
				latest_decoded_frame_index_per_stream[i].store(fi, std::memory_order_release);
			});
		spdlog::warn("Created decoder for stream {}", i);
	}

	for (int i = 0; i < 3; i++)
	{
		current_shards[i].reset(0);
		next_shards[i].reset(1);
	}
}

void streaming_client::setup_audio()
{
	if (!audio_desc)
		return;

	audio_handle = std::make_unique<pico_audio>(*audio_desc, *session);
	if (microphone_enabled.load())
		audio_handle->set_mic_state(true);
	spdlog::info("Audio initialized (mic=%d)", microphone_enabled.load() ? 1 : 0);
}

void streaming_client::handle_video_shard(to_headset::video_stream_data_shard && shard)
{
	last_shard_ns.store(get_timestamp_ns());

	static std::atomic<int> shard_count{0};
	int sc = ++shard_count;
	if (sc <= 5 || (sc % 100 == 0))
		ALOGI("video shard #%d: stream=%u frame=%lu size=%zu",
			sc, (unsigned)shard.stream_item_idx,
			(unsigned long)shard.frame_idx, shard.payload.size());

	if (shard.timing_info && shard.stream_item_idx == 0)
		stats_last_encode_begin = shard.timing_info->encode_begin;

	if (!streaming.load())
	{
		streaming = true;
		stream_ui_visible = false;
		setEyeTrackingStreaming(true);
		ALOGI("Streaming started, hiding lobby UI");
		spdlog::info("Streaming started, hiding lobby UI");
	}

	uint8_t idx = shard.stream_item_idx;
	if (idx >= 3 || !decoders[idx])
	{
		if (sc <= 5)
			ALOGI("video shard dropped: idx=%u decoders=%d", (unsigned)idx, decoders[idx] ? 1 : 0);
		return;
	}

	uint64_t frame_idx = shard.frame_idx;
	shard_set * target = nullptr;
	if (frame_idx == current_shards[idx].frame_index)
		target = &current_shards[idx];
	else if (frame_idx == next_shards[idx].frame_index)
		target = &next_shards[idx];
	else if (frame_idx > current_shards[idx].frame_index)
	{
		if (next_shards[idx].frame_index == frame_idx)
		{
			current_shards[idx] = std::move(next_shards[idx]);
		}
		else
		{
			current_shards[idx].reset(frame_idx);
		}
		next_shards[idx].reset(frame_idx + 1);
		target = &current_shards[idx];
	}
	else
	{
		spdlog::warn("Stale shard for frame {} on stream {}", frame_idx, idx);
		return;
	}

	if (shard.view_info)
	{
		target->view_info = *shard.view_info;
		target->has_view_info = true;
	}

	if (shard.shard_idx >= target->shards.size())
		target->shards.resize(shard.shard_idx + 1);

	bool is_last_shard = shard.timing_info.has_value();
	bool is_first_shard = (shard.shard_idx == 0);

	// Track shard arrival timestamps for the decoder thread to use
	int64_t now_ns = get_timestamp_ns();
	if (is_first_shard)
		target->t_first_shard_ns = now_ns;
	if (is_last_shard)
	{
		target->t_last_shard_ns = now_ns;
		if (shard.timing_info)
		{
			target->has_server_timing = true;
			target->encode_begin = shard.timing_info->encode_begin;
			target->encode_end = shard.timing_info->encode_end;
			target->send_begin = shard.timing_info->send_begin;
			target->send_end = shard.timing_info->send_end;
		}
	}

	target->shards[shard.shard_idx] = std::move(shard);

	if (!is_last_shard)
		return;

	size_t filled = 0;
	for (auto & s : target->shards)
		if (s) filled++;

	if (filled < target->min_for_reconstruction)
		return;

	bool complete = true;
	for (auto & s : target->shards)
		if (!s) { complete = false; break; }

	if (!complete)
		return;

	std::vector<std::span<const uint8_t>> data;
	for (auto & s : target->shards)
	{
		if (s)
			data.push_back(s->payload);
	}

	wivrn::from_headset::feedback feedback{};
	feedback.frame_index = frame_idx;
	feedback.stream_index = idx;

	int64_t fb_now = get_timestamp_ns();
	feedback.received_first_packet = to_xr_time(fb_now);
	feedback.received_last_packet = to_xr_time(fb_now);
	feedback.sent_to_decoder = to_xr_time(fb_now);

	// Push to decoder with timing info; latency/stutter tracking
	// happens on the decoder's input thread, not here.
	decoders[idx]->push_data(data, frame_idx, false,
		target->t_first_shard_ns, target->t_last_shard_ns,
		target->has_server_timing,
		target->encode_begin, target->encode_end,
		target->send_begin, target->send_end);

	feedback.received_from_decoder = to_xr_time(get_timestamp_ns());

	if (target->has_view_info)
		decoders[idx]->frame_completed(feedback, target->view_info);

	if (session)
		session->send_stream(feedback);
}

void streaming_client::handle_packet(to_headset::packets & packet)
{
	std::visit([this](auto && p) {
		using T = std::decay_t<decltype(p)>;

		if constexpr (std::is_same_v<T, to_headset::video_stream_description>)
		{
			std::lock_guard lock(video_mutex);
			video_desc = p;
			video_ready = true;
			stream_eye_width.store(p.width);
			stream_eye_height.store(p.height);
			ALOGI("Received video stream description: %dx%d, fps=%f, codecs=[%d,%d,%d]",
				p.width, p.height, p.frame_rate,
				(int)p.codec[0], (int)p.codec[1], (int)p.codec[2]);
			spdlog::warn("Received video stream description: {}x{}, fps={}",
				p.width, p.height, p.frame_rate);
			// App launched, clear the launching indicator
			{
				std::lock_guard<std::mutex> lk(app_mutex);
				launching_app_id.clear();
			}
			setup_decoders();
		}
		else if constexpr (std::is_same_v<T, to_headset::audio_stream_description>)
		{
			std::lock_guard lock(audio_mutex);
			audio_desc = p;
			ALOGI("Received audio stream description: speaker=%s mic=%s",
				p.speaker ? "yes" : "no",
				p.microphone ? "yes" : "no");
			spdlog::info("Received audio stream description: speaker={} mic={}",
				p.speaker ? "yes" : "no",
				p.microphone ? "yes" : "no");
			setup_audio();
		}
		else if constexpr (std::is_same_v<T, to_headset::tracking_control>)
		{
			std::lock_guard lock(tracking_mutex);
			tracking_control = p;
			tracking_control_received = true;
			ALOGI("Received tracking control: %zu samples, m2p=%lldns",
				p.pattern.size(), (long long)p.motions_to_photons);
			spdlog::info("Received tracking control: {} samples, m2p={}ns",
				p.pattern.size(), p.motions_to_photons);

			int64_t head_pred = 0;
			for (const auto & s : p.pattern)
			{
				if (s.device == device_id::HEAD)
				{
					head_pred = std::max(head_pred, s.prediction_ns);
				}
			}
			// The server-reported motions-to-photons is the total pipeline latency estimate.
			// Use it even when the HEAD pattern gives a smaller per-device prediction.
			if (p.motions_to_photons > 0)
				head_pred = std::max(head_pred, p.motions_to_photons);
			if (head_pred == 0)
				head_pred = 30000000; // 30ms default prediction

			tracker.set_prediction_ns(head_pred);
			spdlog::info("Head prediction: base={}ns m2p={}ns pattern={}ns",
				head_pred, p.motions_to_photons,
				[&]{ int64_t m=0; for (auto&s:p.pattern) if(s.device==device_id::HEAD) m=std::max(m,s.prediction_ns); return m; }());
		}
		else if constexpr (std::is_same_v<T, to_headset::haptics>)
		{
			spdlog::info("HAPTICS packet received: id={} amp={} dur={}ns freq={}",
				(int)p.id, p.amplitude, p.duration.count(), p.frequency);
			int hand = -1;
			switch (p.id)
			{
				case device_id::LEFT_CONTROLLER_HAPTIC:
				case device_id::LEFT_TRIGGER_HAPTIC:
				case device_id::LEFT_THUMB_HAPTIC:
					hand = 0;
					break;
				case device_id::RIGHT_CONTROLLER_HAPTIC:
				case device_id::RIGHT_TRIGGER_HAPTIC:
				case device_id::RIGHT_THUMB_HAPTIC:
					hand = 1;
					break;
				default:
					break;
			}
			if (hand >= 0)
				queueHaptic(hand, p.amplitude, p.duration.count());
		}
		else if constexpr (std::is_same_v<T, to_headset::timesync_query>)
		{
			int64_t now = get_timestamp_ns();
			session->send_control(from_headset::timesync_response{
				.query = p.query,
				.response = now,
			});
		}
		else if constexpr (std::is_same_v<T, to_headset::feature_control>)
		{
			if (p.f == to_headset::feature_control::microphone)
			{
				microphone_enabled = p.state;
				if (audio_handle)
					audio_handle->set_mic_state(p.state);
			}
		}
		else if constexpr (std::is_same_v<T, to_headset::refresh_rate_change>)
		{
			spdlog::info("Refresh rate change requested: {} Hz (not supported)", p.hz);
		}
		else if constexpr (std::is_same_v<T, to_headset::server_message>)
		{
			spdlog::info("Server message: {}", p.msg);
		}
		else if constexpr (std::is_same_v<T, to_headset::video_stream_data_shard>)
		{
			handle_video_shard(std::move(p));
		}
		else if constexpr (std::is_same_v<T, wivrn::audio_data>)
		{
			if (audio_handle)
				(*audio_handle)(std::move(p));
		}
		else if constexpr (std::is_same_v<T, to_headset::crypto_handshake> ||
			std::is_same_v<T, to_headset::pin_check_2> ||
			std::is_same_v<T, to_headset::pin_check_4> ||
			std::is_same_v<T, to_headset::handshake>)
		{
		}
		else if constexpr (std::is_same_v<T, to_headset::application_list>)
		{
			spdlog::info("Received application list: {} apps", p.applications.size());
			std::vector<std::pair<std::string, std::string>> apps;
			apps.reserve(p.applications.size());
			for (auto & app : p.applications)
				apps.emplace_back(app.id, app.name);
			{
				std::lock_guard<std::mutex> lk(app_mutex);
				available_apps.clear();
				for (auto & app : p.applications)
					available_apps.push_back({app.id, app.name});
				app_list_requested = true;
			}
		}
		else if constexpr (std::is_same_v<T, to_headset::application_icon>)
		{
			spdlog::info("Received application icon for id={} ({} bytes)", p.id, p.image.size());
		}
		else if constexpr (std::is_same_v<T, to_headset::running_applications>)
		{
			spdlog::info("Received running applications: {} apps", p.applications.size());
			{
				std::lock_guard<std::mutex> lk(app_mutex);
				running_apps.clear();
				for (auto & app : p.applications)
					running_apps.push_back({app.name, app.id, app.overlay, app.active});
				// Clear launching state if any app is now active
				if (!launching_app_id.empty()) {
					bool any_active = false;
					for (auto & ra : running_apps)
						if (ra.active) { any_active = true; break; }
					if (any_active) launching_app_id.clear();
				}
			}
		}
		else
		{
			ALOGI("handle_packet: unhandled/unknown packet type");
		}
	}, packet);
}

void streaming_client::reset_stream_state()
{
	spdlog::info("reset_stream_state: streaming=false");
	setEyeTrackingStreaming(false);
	streaming = false;
	stream_ui_visible = false;
	stream_stalled = false;
	last_displayed_frame = 0;
	frame_first_displayed_ns = 0;
	video_ready = false;
	g_latency.reset();
	{
		std::lock_guard<std::mutex> lk(app_mutex);
		available_apps.clear();
		running_apps.clear();
		app_list_requested = false;
		launching_app_id.clear();
	}
	stats_bytes_rx = 0;
	stats_bytes_tx = 0;
	stats_bandwidth_rx = 0;
	stats_bandwidth_tx = 0;
	spdlog::info("reset_stream_state: resetting video_desc");
	video_desc.reset();
	spdlog::info("reset_stream_state: resetting audio_desc");
	audio_desc.reset();
	spdlog::info("reset_stream_state: resetting audio_handle");
	audio_handle.reset();
	spdlog::info("reset_stream_state: resetting decoders");
	for (int i = 0; i < 3; i++)
	{
		decoders[i].reset();
		current_shards[i].reset(0);
		next_shards[i].reset(1);
		{
			std::lock_guard lock(decoded_frame_mutex);
			for (auto &slot : decoded_frame_buffers[i])
				slot.reset();
		}
	}
	latest_decoded_frame_index.store(0);
	last_shard_ns.store(0);
	int max_b = max_bitrate_mbps.load();
	current_bitrate_mbps.store(max_b);
	bitrate_mbps.store(max_b);
	spdlog::info("reset_stream_state: done");
}

void streaming_client::send_bitrate_change(int mbps)
{
	if (!session)
		return;

	from_headset::settings_changed sc{};
	sc.preferred_refresh_rate = 72.0f;
	sc.minimum_refresh_rate = 72.0f;
	sc.fps_divider = 1;
	sc.bitrate_bps = (uint32_t)mbps * 1'000'000;
	session->send_control(sc);
	spdlog::info("send_bitrate_change: {} Mbps", mbps);
}

void streaming_client::send_eye_foveation_override()
{
	if (!session)
		return;

	// Only meaningful on EYE hardware. On non-EYE units there is no gaze to
	// track and the server already falls back to a fixed center, so skip the
	// packet to avoid confusing the server's override state.
	if (!gEyeSupported.load())
		return;

	bool want_eye = gWivrnEyeFoveation.load();
	from_headset::override_foveation_center pkt{};
	pkt.enabled = !want_eye;   // server: enabled=true => use the fixed center we send
	// Defaults mirror the WiVRn server's natural-gaze fallback (see
	// wivrn_foveation.cpp: 10 deg below horizontal, 1 m convergence). Sent
	// only when the override is active; the server ignores them otherwise.
	pkt.pitch    = -10.0f * (float)M_PI / 180.0f;
	pkt.distance = 1.0f;
	session->send_control(pkt);
	ALOGI("send_eye_foveation_override: eye_tracked=%d (server override=%s)",
	      want_eye ? 1 : 0, pkt.enabled ? "fixed" : "gaze");
	spdlog::info("send_eye_foveation_override: eye_tracked={} (server override={})",
	             want_eye, pkt.enabled ? "fixed" : "gaze");
}

void streaming_client::network_loop()
{
	constexpr int64_t video_timeout_ns = 10'000'000'000LL;
	ALOGI("network_loop: started");

	int64_t last_state_ping_ns = get_timestamp_ns();

	while (!shutdown && session)
	{
		try
		{
			session->poll([this](to_headset::packets && packet) {
				handle_packet(packet);
			}, 500ms);

			int64_t now = get_timestamp_ns();
			int64_t last = last_shard_ns.load();

			// Re-send session state every 2s so the server updates
			// visibility/focus for newly connected OpenXR apps.
			// When the in-stream lobby overlay (gManualLobby) is open the
			// session is VISIBLE, not FOCUSED, so the server and SteamVR
			// overlays know the user is interacting with a menu.
			if (now - last_state_ping_ns > 2'000'000'000LL)
			{
				last_state_ping_ns = now;
				session->send_control(from_headset::session_state_changed{
					.state = gManualLobby.load() ? XR_SESSION_STATE_VISIBLE
					                             : XR_SESSION_STATE_FOCUSED,
				});
			}

			if (last > 0 && (now - last) > video_timeout_ns)
			{
				last_error = fmt::format("No video for {}s", video_timeout_ns / 1'000'000'000);
				spdlog::warn("{}", last_error);
				break;
			}
		}
		catch (std::exception & e)
		{
			last_error = e.what();
			spdlog::error("Network error: {}", e.what());
			break;
		}
		catch (...)
		{
			last_error = "Unknown network error";
			spdlog::error("Network error: unknown exception");
			break;
		}
	}

	spdlog::info("network_loop ended, cleaning up session");
	if (auto_reconnect.load())
		spdlog::info("network_loop: auto_reconnect set, will retry");
	else
		spdlog::info("network_loop: disconnected: {}", last_error.empty() ? "Disconnected" : last_error);
	if (!last_error.empty() && !auto_reconnect.load() && user_initiated.load())
		setConnectionError(last_error);
	spdlog::info("network_loop: setting tracker.session=nullptr");
	tracker.session = nullptr;
	spdlog::info("network_loop: calling tracker.stop()");
	tracker.stop();
	spdlog::info("network_loop: tracker stopped, resetting session");
	session.reset();
	spdlog::info("network_loop: session reset, resetting stream state");
	reset_stream_state();
	spdlog::info("network_loop: done, exiting");
}

bool streaming_client::connect_to_server()
{
	ALOGI("connect_to_server: enter, host=%s port=%d", server_host.c_str(), server_port);
	spdlog::info("Connecting to server {}:{}", server_host, server_port);
	ALOGI("connect_to_server: after spdlog");
	last_error.clear();

	try
	{
		const char * key_dir = "/data/data/org.meumeu.wivrn.neo2.pvr/files";
		const char * key_path = "/data/data/org.meumeu.wivrn.neo2.pvr/files/private_key.pem";
		ALOGI("connect_to_server: loading keypair from %s", key_path);
		spdlog::info("connect: loading keypair from {}", key_path);
		bool key_loaded = false;
		{
			std::ifstream f{key_path};
			if (f.good())
			{
				std::string key_str{(std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>()};
				ALOGI("connect_to_server: key_str len=%zu", key_str.size());
				if (!key_str.empty())
				{
					headset_keypair = crypto::key::from_private_key(key_str);
					ALOGI("connect_to_server: loaded existing keypair");
					spdlog::info("connect: loaded existing keypair");
					key_loaded = true;
				}
			}
		}
		if (!key_loaded)
		{
			ALOGI("connect_to_server: generating new keypair");
			spdlog::info("connect: generating new keypair");
			mkdir(key_dir, 0700);
			headset_keypair = crypto::key::generate_x448_keypair();
			ALOGI("connect_to_server: keypair generated, writing");
			std::ofstream f{key_path};
			f << headset_keypair.private_key();
			f.close();
			ALOGI("connect_to_server: keypair written");
		}
		std::string model_name = "Pico Neo2";
		ALOGI("connect_to_server: model_name=%s, pin_empty=%d", model_name.c_str(), (int)pairing_pin.empty());

		auto pin_enter = [this](int fd) -> std::string {
			if (!pairing_pin.empty())
			{
				spdlog::warn("PIN entry requested - using PIN from URI: {}", pairing_pin);
				return pairing_pin;
			}

			pin_promise = std::promise<std::string>();
			auto future = pin_promise.get_future();

			// Wire the PIN pad submit callback to fulfill our promise.
			gOnPinSubmit = [this](const std::string &pin) {
				pairing_pin = pin;
				try { pin_promise.set_value(pin); } catch (...) {}
			};
			requestPinEntryUI();
			spdlog::info("PIN entry requested from UI");

			for (int i = 0; i < 1200; ++i)
			{
				if (shutdown)
					return "000000";
				if (!pairing_pin.empty())
					return pairing_pin;

				pollfd pfd{};
				pfd.fd = fd;
				pfd.events = 0;
				int r = ::poll(&pfd, 1, 0);
				if (r < 0 || (pfd.revents & (POLLHUP | POLLERR | POLLRDHUP)))
				{
					spdlog::warn("PIN entry: control socket closed by server");
					throw std::runtime_error("Server disconnected during PIN entry");
				}

				auto status = future.wait_for(std::chrono::milliseconds(100));
				if (status == std::future_status::ready)
				{
					std::string pin = future.get();
					if (pin.empty())
						return "000000";
					return pin;
				}
			}
			return "000000";
		};

		struct addrinfo hints{};
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;

		struct addrinfo * result = nullptr;
		ALOGI("connect_to_server: calling getaddrinfo(%s, %d)", server_host.c_str(), server_port);
		int ret = getaddrinfo(server_host.c_str(), std::to_string(server_port).c_str(), &hints, &result);
		ALOGI("connect_to_server: getaddrinfo returned %d", ret);
		if (ret != 0)
		{
			std::string local_host = server_host + ".local";
			ALOGI("connect_to_server: retrying with %s", local_host.c_str());
			ret = getaddrinfo(local_host.c_str(), std::to_string(server_port).c_str(), &hints, &result);
			ALOGI("connect_to_server: getaddrinfo(.local) returned %d", ret);
		}
		if (ret != 0)
		{
			ALOGI("connect_to_server: getaddrinfo failed: %s", gai_strerror(ret));
			spdlog::error("getaddrinfo failed: {}", gai_strerror(ret));
			return false;
		}

		for (struct addrinfo * rp = result; rp; rp = rp->ai_next)
		{
			if (shutdown)
			{
				ALOGI("connect_to_server: shutdown requested, aborting");
				spdlog::info("connect: shutdown requested, aborting");
				freeaddrinfo(result);
				return false;
			}
			try
			{
				if (rp->ai_family == AF_INET)
				{
					auto * addr = (struct sockaddr_in *)rp->ai_addr;
					ALOGI("connect_to_server: trying IPv4 %s", inet_ntoa(addr->sin_addr));
					spdlog::warn("connect: trying IPv4 {}", inet_ntoa(addr->sin_addr));

					try
					{
						ALOGI("connect_to_server: creating wivrn_session_pico (IPv4)");
						session = std::make_unique<wivrn_session_pico>(
							*addr, tcp_only, headset_keypair, model_name, pin_enter, shutdown);
						ALOGI("connect_to_server: session created, handshake_ok=%d", (int)(session && session->is_handshake_ok()));
					}
					catch (std::exception & e)
					{
						ALOGI("connect_to_server: session creation failed: %s", e.what());
						spdlog::warn("connect: session creation failed: {}", e.what()); last_error = e.what();
					}
					if (session && session->is_handshake_ok())
					{
						freeaddrinfo(result);
						ALOGI("connect_to_server: Connected to server (IPv4)");
						spdlog::info("Connected to server (IPv4)");
						return true;
					}
					if (session)
						session.reset();
				}
				else if (rp->ai_family == AF_INET6)
				{
					auto * addr = (struct sockaddr_in6 *)rp->ai_addr;
					char ip6str[INET6_ADDRSTRLEN];
				 inet_ntop(AF_INET6, &addr->sin6_addr, ip6str, sizeof(ip6str));
					ALOGI("connect_to_server: trying IPv6 %s", ip6str);
					try
					{
						ALOGI("connect_to_server: creating wivrn_session_pico (IPv6)");
						session = std::make_unique<wivrn_session_pico>(
							*addr, tcp_only, headset_keypair, model_name, pin_enter, shutdown);
						ALOGI("connect_to_server: session created (IPv6), handshake_ok=%d", (int)(session && session->is_handshake_ok()));
					}
					catch (std::exception & e)
					{
						ALOGI("connect_to_server: IPv6 session creation failed: %s", e.what());
						spdlog::warn("connect: IPv6 session creation failed: {}", e.what()); last_error = e.what();
					}
					if (session && session->is_handshake_ok())
					{
						freeaddrinfo(result);
						ALOGI("connect_to_server: Connected to server (IPv6)");
						spdlog::info("Connected to server (IPv6)");
						return true;
					}
					if (session)
						session.reset();
				}
			}
			catch (std::exception & e)
			{
				ALOGI("connect_to_server: Connection attempt failed: %s", e.what());
				spdlog::warn("Connection attempt failed: {}", e.what()); last_error = e.what();
			}
		}

		freeaddrinfo(result);
		ALOGI("connect_to_server: Failed to connect to server");
		spdlog::error("Failed to connect to server");
		return false;
	}
	catch (std::exception & e)
	{
		spdlog::error("connect_to_server error: {}", e.what());
		last_error = e.what();
		return false;
	}
	catch (...)
	{
		spdlog::error("connect_to_server error: unknown exception");
		last_error = "Unknown error";
		return false;
	}
}

void streaming_client::send_headset_info()
{
	if (!session)
		return;

	from_headset::headset_info_packet info{};

	info.render_eye_width = eye_width.load();
	info.render_eye_height = eye_height.load();
	info.stream_eye_width = stream_eye_width.load();
	info.stream_eye_height = stream_eye_height.load();

	info.available_refresh_rates.push_back(72.0f);
	info.settings.preferred_refresh_rate = 72.0f;
	info.settings.minimum_refresh_rate = 72.0f;
	info.settings.fps_divider = 1;
	info.settings.bitrate_bps = (uint32_t)bitrate_mbps.load() * 1000000;

	pico_audio::get_audio_description(info);

	if (!microphone_enabled.load())
		info.microphone.reset();

	ALOGI("send_headset_info: microphone_enabled=%d, info.microphone=%s, info.speaker=%s",
		microphone_enabled.load() ? 1 : 0,
		info.microphone ? "yes" : "no",
		info.speaker ? "yes" : "no");

	info.fov[0] = eye_fov[0];
	info.fov[1] = eye_fov[1];

	info.hand_tracking = false;
	info.eye_gaze = gEyeSupported.load();
	info.palm_pose = false;
	info.user_presence = true;
	info.passthrough = false;
	info.face_tracking = gEyeSupported.load() ? from_headset::face_type::fb2 : from_headset::face_type::none;
	info.body_tracking = from_headset::body_type::none;
	info.num_generic_trackers = 0;

	// Request 10-bit encoding for better quality (eliminates banding
	// artifacts in gradients). The server will fall back to 8-bit if
	// the GPU or codec doesn't support 10-bit.
	info.bit_depth = 10;

	{
		std::vector<wivrn::video_codec> codecs;
		pico_video_decoder::supported_codecs(codecs);
		info.supported_codecs = codecs;
	}
	if (info.supported_codecs.empty())
		info.supported_codecs = {h264, h265};

	info.system_name = "Pico Neo2";
	info.language = "en";
	info.country = "US";
	info.variant = "";

	session->send_control(info);
	ALOGI("Sent headset info: %dx%d, %zu codecs, bit_depth=%d",
		info.render_eye_width, info.render_eye_height,
		info.supported_codecs.size(),
		info.bit_depth ? (int)*info.bit_depth : -1);
	spdlog::warn("Sent headset info: {}x{}, {} codecs, bit_depth={}",
		info.render_eye_width, info.render_eye_height,
		info.supported_codecs.size(),
		info.bit_depth ? (int)*info.bit_depth : -1);

	session->send_control(from_headset::session_state_changed{
		.state = XR_SESSION_STATE_FOCUSED,
	});

	session->send_control(from_headset::stream_tab_changed{
		.tab = wivrn::stream_tab::hidden,
	});
}

void streaming_client::try_connect()
{
	ALOGI("try_connect: enter, server_host='%s'", server_host.c_str());
	if (server_host.empty())
	{
		ALOGI("try_connect: server_host is empty");
		spdlog::warn("try_connect: server_host is empty");
		return;
	}

	ALOGI("try_connect: locking connect_mutex");
	std::lock_guard lock(connect_mutex);
	ALOGI("try_connect: locked, connect_thread.joinable=%d", (int)connect_thread.joinable());
	if (connect_thread.joinable())
	{
		ALOGI("try_connect: stopping previous connection");
		spdlog::info("try_connect: stopping previous connection");
		shutdown = true;
		if (session)
		{
			int fd = session->get_control_fd();
			::shutdown(fd, SHUT_RDWR);
		}
		std::thread([this] {
			spdlog::info("detached: waiting for connect_mutex");
			std::lock_guard lock(connect_mutex);
			spdlog::info("detached: got connect_mutex, joining threads");
			if (connect_thread.joinable())
				connect_thread.join();
			spdlog::info("detached: connect_thread joined");
			if (network_thread.joinable())
				network_thread.join();
			spdlog::info("detached: network_thread joined");
			session.reset();
			reset_stream_state();
			shutdown = false;
			spdlog::info("detached: starting new connect_thread");
			connect_thread = std::thread([this] {
				run_connect_loop();
			});
		}).detach();
		return;
	}

	ALOGI("try_connect: creating connect_thread");
	connect_thread = std::thread([this] {
		ALOGI("connect_thread lambda: starting, this=%p", (void*)this);
		run_connect_loop();
	});
	ALOGI("try_connect: connect_thread created, returning");
}

void streaming_client::run_connect_loop()
{
	ALOGI("run_connect_loop: entered, this=%p", (void*)this);
	spdlog::info("Connection attempt");
	ALOGI("run_connect_loop: after spdlog info");
	setConnecting(true);

	if (connect_to_server())
	{
		try
		{
			int fd = session->get_control_fd();
			int keepalive = 1;
			setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
			int idle = 3;
			setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
			int intvl = 1;
			setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
			int cnt = 3;
			setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));

			send_headset_info();
			connected_ns.store(get_timestamp_ns());
			last_shard_ns.store(0);
			clearConnectionError();
			network_thread = std::thread([this] { network_loop(); });
			tracker.session = session.get();
			tracker.start();

			network_thread.join();
			spdlog::info("run_connect_loop: network_thread joined, auto_reconnect={}", auto_reconnect.load());
		}
		catch (std::exception & e)
		{
			spdlog::error("Failed to start client: {}", e.what());
			last_error = e.what();
			if (user_initiated.load())
				setConnectionError(e.what());
			if (network_thread.joinable())
				network_thread.join();
			tracker.session = nullptr;
			tracker.stop();
			session.reset();
			reset_stream_state();
		}
	}
	else
	{
		if (session)
			session.reset();
		if (!shutdown)
		{
			spdlog::warn("Connection failed: {}", last_error);
			if (user_initiated.load())
				setConnectionError(last_error);
		}
	}
	setConnecting(false);
	user_initiated.store(false);

	while (auto_reconnect.load() && !shutdown.load())
	{
		spdlog::info("Auto-reconnect: waiting 2s before retrying...");
		for (int i = 0; i < 20 && !shutdown.load(); i++)
			std::this_thread::sleep_for(100ms);
		if (shutdown.load())
			break;

		spdlog::info("Auto-reconnect: retrying connection to {}:{}", server_host, server_port);

		if (connect_to_server())
		{
			try
			{
				int fd = session->get_control_fd();
				int keepalive = 1;
				setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
				int idle = 3;
				setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
				int intvl = 1;
				setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
				int cnt = 3;
				setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));

				send_headset_info();
				connected_ns.store(get_timestamp_ns());
				last_shard_ns.store(0);
				network_thread = std::thread([this] { network_loop(); });
				tracker.session = session.get();
				tracker.start();

				network_thread.join();
			}
			catch (std::exception & e)
			{
				spdlog::error("Auto-reconnect: failed to start client: {}", e.what());
				last_error = e.what();
				if (network_thread.joinable())
					network_thread.join();
				tracker.session = nullptr;
				tracker.stop();
				session.reset();
				reset_stream_state();
			}
		}
		else
		{
			if (session)
				session.reset();
			spdlog::warn("Auto-reconnect: connection failed: {}", last_error);
		}
	}

	if (!auto_reconnect.load())
	{
		spdlog::info("Connection thread exiting: auto_reconnect is false");
	}
	else if (shutdown.load())
	{
		spdlog::info("Connection thread exiting: shutdown requested");
	}
	else
	{
		spdlog::info("Connection thread exiting");
	}
}
