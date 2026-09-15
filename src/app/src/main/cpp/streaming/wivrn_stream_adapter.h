#pragma once

#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <openxr/openxr.h>
#include <memory>
#include <mutex>

// Returns true if the WiVRn stream client exists and is actively streaming.
bool wivrn_streaming();

// Returns true if the stream is active AND the decoders are ready to produce
// frames. When this becomes true the render loop should create/resize the
// stream swapchain.
bool wivrn_stream_ready();

// Writes the current stream resolution into *w and *h. Returns true when the
// video description has been received and a resolution is known.
bool wivrn_stream_resolution(int *w, int *h);

// Negotiated stream refresh rate from the video description, or 0 when unknown.
float wivrn_stream_framerate();

// Blit the latest decoded WiVRn frame for one eye into the currently bound
// FBO. The caller must have bound the destination framebuffer and viewport.
// Returns true if a frame was available and blit.
// If out_pose is non-null, receives the server render pose from the blitted frame.
bool wivrn_blit_eye(int eye, int viewport_w, int viewport_h, XrPosef * out_pose = nullptr);

// Get synchronized frames for both eyes (same frame index). Returns the
// frames and their server poses. The caller is responsible for binding the
// FBO per eye and calling wivrn_blit_eye_frame.
struct pico_decoded_frame;
bool wivrn_get_synced_frames(std::shared_ptr<pico_decoded_frame> out_frames[2],
                             XrPosef out_server_pose[2]);

// Blit a specific frame for one eye into the currently bound FBO.
bool wivrn_blit_eye_frame(int eye,
                          const std::shared_ptr<pico_decoded_frame> & frame,
                          int viewport_w, int viewport_h,
                          XrPosef * out_pose = nullptr);

// Blit the newest synchronized frame pair (same frame index for both eyes)
// into dst_tex[0]/dst_tex[1] via `fbo`, at eye resolution. Stores the server
// poses of the blitted frames for wivrn_get_server_pose.
void wivrn_blit_frame_pair(GLuint fbo, const GLuint dst_tex[2]);

// Server render poses from the last synchronized blit (set by the render
// path, read by the render thread's warp setup).
extern XrPosef gLastServerPoses[2];
extern std::mutex gServerPoseMutex;

// Returns the server render pose from the last synchronized blit.
bool wivrn_get_server_pose(XrPosef out[2]);
