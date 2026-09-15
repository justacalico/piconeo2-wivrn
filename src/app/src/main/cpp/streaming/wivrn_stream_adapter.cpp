#include "wivrn_stream_adapter.h"
#include "streaming_client.h"
#include "pico_decoder.h"
#include "log.h"
#include "latency_tracker.h"

#include <android/hardware_buffer.h>
#include <mutex>

extern PFNEGLCREATEIMAGEKHRPROC g_eglCreateImageKHR;
extern PFNEGLDESTROYIMAGEKHRPROC g_eglDestroyImageKHR;
extern PFNEGLGETNATIVECLIENTBUFFERANDROIDPROC g_eglGetNativeClientBufferANDROID;
extern PFNGLEGLIMAGETARGETTEXTURE2DOESPROC g_glEGLImageTargetTexture2DOES;

static GLuint g_eye_textures[2] = {0, 0};
static EGLImageKHR g_eye_images[2] = {EGL_NO_IMAGE_KHR, EGL_NO_IMAGE_KHR};
static AHardwareBuffer * g_last_hb[2] = {nullptr, nullptr};

// Server render poses from the last synchronized blit, for the render
// thread's warp baseline. Written by wivrn_blit_frame_pair, read by the
// render thread when setting up the warp pose.
XrPosef gLastServerPoses[2] = {};
std::mutex gServerPoseMutex;

bool wivrn_streaming()
{
    return g_stream && g_stream->streaming.load();
}

bool wivrn_stream_ready()
{
    if (!g_stream) return false;
    return g_stream->streaming.load() && g_stream->video_ready;
}

bool wivrn_stream_resolution(int *w, int *h)
{
    if (!g_stream || !g_stream->video_desc) return false;
    std::lock_guard lock(g_stream->video_mutex);
    if (!g_stream->video_desc) return false;
    *w = g_stream->video_desc->width;
    *h = g_stream->video_desc->height;
    return true;
}

float wivrn_stream_framerate()
{
    if (!g_stream || !g_stream->video_desc) return 0.0f;
    std::lock_guard lock(g_stream->video_mutex);
    if (!g_stream->video_desc) return 0.0f;
    float r = g_stream->video_desc->refresh_rate;
    return r > 0.0f ? r : g_stream->video_desc->frame_rate;
}

bool wivrn_get_synced_frames(std::shared_ptr<pico_decoded_frame> out_frames[2],
                             XrPosef out_server_pose[2])
{
    if (!g_stream) return false;

    uint64_t idx0 = g_stream->latest_decoded_frame_index_per_stream[0].load(std::memory_order_acquire);
    uint64_t idx1 = g_stream->latest_decoded_frame_index_per_stream[1].load(std::memory_order_acquire);

    if (idx0 > 0 && idx1 > 0)
    {
        uint64_t chosen = std::min(idx0, idx1);
        out_frames[0] = g_stream->get_frame(chosen, 0);
        out_frames[1] = g_stream->get_frame(chosen, 1);
        if (!out_frames[0] || !out_frames[1] || !out_frames[0]->valid || !out_frames[1]->valid)
        {
            out_frames[0] = g_stream->get_latest_frame(0);
            out_frames[1] = g_stream->get_latest_frame(1);
        }
    }
    else
    {
        out_frames[0] = g_stream->get_latest_frame(0);
        out_frames[1] = g_stream->get_latest_frame(1);
    }

    for (int e = 0; e < 2; e++)
    {
        if (out_frames[e] && out_frames[e]->valid)
            out_server_pose[e] = out_frames[e]->server_pose[e];
        else
            out_server_pose[e] = {};
    }
    return true;
}

bool wivrn_blit_eye_frame(int eye, const std::shared_ptr<pico_decoded_frame> & frame,
                          int viewport_w, int viewport_h, XrPosef * out_pose)
{
    if (!frame || !frame->valid || !frame->hardware_buffer)
        return false;

    if (out_pose)
        *out_pose = frame->server_pose[eye];

    if (g_eye_textures[eye] == 0)
    {
        glGenTextures(1, &g_eye_textures[eye]);
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, g_eye_textures[eye]);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0);
    }

    if (frame->hardware_buffer != g_last_hb[eye])
    {
        if (g_eye_images[eye] != EGL_NO_IMAGE_KHR)
        {
            g_eglDestroyImageKHR(eglGetCurrentDisplay(), g_eye_images[eye]);
            g_eye_images[eye] = EGL_NO_IMAGE_KHR;
        }

        EGLClientBuffer client_buffer = g_eglGetNativeClientBufferANDROID(frame->hardware_buffer);
        if (client_buffer)
        {
            EGLint attrs[] = {EGL_IMAGE_PRESERVED_KHR, EGL_TRUE, EGL_NONE};
            g_eye_images[eye] = g_eglCreateImageKHR(
                eglGetCurrentDisplay(), EGL_NO_CONTEXT, EGL_NATIVE_BUFFER_ANDROID,
                client_buffer, attrs);
        }

        if (g_eye_images[eye] != EGL_NO_IMAGE_KHR)
        {
            glBindTexture(GL_TEXTURE_EXTERNAL_OES, g_eye_textures[eye]);
            g_glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, g_eye_images[eye]);
            glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0);
        }
        g_last_hb[eye] = frame->hardware_buffer;
    }

    if (g_eye_images[eye] == EGL_NO_IMAGE_KHR)
        return false;

    if (!g_stream->blit_pipeline.is_initialized())
        g_stream->blit_pipeline.init(viewport_w, viewport_h);
    g_stream->blit_pipeline.set_resolution(viewport_w, viewport_h);

    g_stream->blit_pipeline.draw(eye, g_eye_textures[eye],
                                frame->foveation[eye], frame->width, frame->height);

    g_latency.on_frame_rendered(frame->frame_index, eye);
    return true;
}

bool wivrn_blit_eye(int eye, int viewport_w, int viewport_h, XrPosef * out_pose)
{
    if (!g_stream) return false;
    auto frame = g_stream->get_latest_frame(eye);
    return wivrn_blit_eye_frame(eye, frame, viewport_w, viewport_h, out_pose);
}

void wivrn_blit_frame_pair(GLuint fbo, const GLuint dst_tex[2])
{
    if (!g_stream)
        return;

    const int eyeW = g_stream->eye_width.load();
    const int eyeH = g_stream->eye_height.load();
    if (eyeW <= 0 || eyeH <= 0)
        return;

    // If stream 1 falls far behind stream 0, flush its decoder so it can
    // catch up. This prevents the right eye from showing stale frames.
    {
        uint64_t idx0 = g_stream->latest_decoded_frame_index_per_stream[0].load(std::memory_order_acquire);
        uint64_t idx1 = g_stream->latest_decoded_frame_index_per_stream[1].load(std::memory_order_acquire);
        if (idx0 > 0 && idx1 > 0)
        {
            int64_t gap = (int64_t)idx0 - (int64_t)idx1;
            if (gap > 15 && g_stream->decoders[1])
            {
                static int flush_log_count = 0;
                if (flush_log_count++ % 100 == 0)
                    LOGI("stream 1 behind by %lld frames, flushing", (long long)gap);
                g_stream->decoders[1]->flush();
            }
        }
    }

    // Get synchronized frames for both eyes (same frame index) to prevent
    // the right eye from stuttering when stream 1 lags behind stream 0.
    std::shared_ptr<pico_decoded_frame> frames[2];
    XrPosef poses[2];
    wivrn_get_synced_frames(frames, poses);

    for (int eye = 0; eye < 2; eye++) {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, dst_tex[eye], 0);
        glViewport(0, 0, static_cast<GLsizei>(eyeW), static_cast<GLsizei>(eyeH));
        wivrn_blit_eye_frame(eye, frames[eye], eyeW, eyeH, &poses[eye]);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Store the exact server poses from the frames that were blitted.
    {
        std::lock_guard<std::mutex> lk(gServerPoseMutex);
        gLastServerPoses[0] = poses[0];
        gLastServerPoses[1] = poses[1];
    }
}

bool wivrn_get_server_pose(XrPosef out[2])
{
    std::lock_guard<std::mutex> lk(gServerPoseMutex);
    out[0] = gLastServerPoses[0];
    out[1] = gLastServerPoses[1];
    // Validate by squared norm so valid 180° orientations like (0,1,0,0)
    // return true. The old (w != 0 || x != 0) check rejected those.
    const auto & o = gLastServerPoses[0].orientation;
    float n2 = o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w;
    return n2 > 1e-6f;
}
