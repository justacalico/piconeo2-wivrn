#include <jni.h>
#include <android/log.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>          // sched_setaffinity / SCHED_FIFO
#include <sys/resource.h>   // setpriority fallback
#include <dirent.h>         // scan /proc/self/task to find the SDK warp thread
#include <errno.h>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <cstdio>
#include <time.h>

#include "alvr_client_core.h"
#include "alvr_ext.h"    // fork-only ALVR C API additions
#include "log.h"         // TAG / LOGI / LOGE / nowNs()
#include "pico_sdk.h"    // Pico native SDK prototypes + config accessors + render events
#include "streaming/streaming_client.h"  // g_stream (for tracker recenter flag)
#include "wivrn/core/latency_tracker.h"  // g_latency
#include "math3d.h"      // Mat4 / Quat helpers
#include "gl_util.h"     // compile()
#include "eye_tracking.h"// readEyeGazes() + gaze/openness state
#include "device_info.h" // IP / status / model strings + readers
#include "android_ui.h"  // Android View-based UI (replaces ImGui)
#include "ui_kit.h"      // font + appendTextLine/Quad for reticle/crosshair
#include "eq_panel.h"    // EQ state for config persistence
#include "server_list.h"     // server list data layer
#include "app_state.h"   // shared lobby/render knobs: IPD, input edges, toggles, diag
#include "lobby.h"       // 3D lobby environment
#include "passthrough.h" // passthrough camera background for the lobby
#include "simple_lobby.h"// simple 3D lobby environment (floor grid + sky)
#include "input.h"       // controller + head-pose shared state
#include "foveation.h"   // readFoveationParams() from the settings JSON
#include "render_thread.h"// shared render-thread lifetime/window/sleep state
#include "streaming/wivrn_stream_adapter.h"

// Sleep until an ABSOLUTE CLOCK_MONOTONIC deadline. Avoids relative usleep oversleep
// drift (1-4ms under scheduler load). A past deadline returns immediately.
static inline void sleepUntilMonoNs(uint64_t deadlineNs) {
    struct timespec t;
    t.tv_sec  = (time_t)(deadlineNs / 1000000000ULL);
    t.tv_nsec = (long)(deadlineNs % 1000000000ULL);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, nullptr);
}

// Bounded copy into a fixed-size status/text buffer. Today every caller passes
// a string literal, but the helper future-proofs the pattern against later
// server/user-controlled input landing in the same buffers.
static inline void setStrBounded(char *dst, const char *src, size_t cap) {
    if (cap == 0) return;
    size_t i = 0;
    while (i + 1 < cap && src[i]) { dst[i] = src[i]; i++; }
    dst[i] = 0;
}

static const char *alvrEventName(AlvrEvent_Tag t) {
    switch (t) {
        case ALVR_EVENT_HUD_MESSAGE_UPDATED: return "HUD_MESSAGE_UPDATED";
        case ALVR_EVENT_STREAMING_STARTED:   return "STREAMING_STARTED";
        case ALVR_EVENT_STREAMING_STOPPED:   return "STREAMING_STOPPED";
        case ALVR_EVENT_HAPTICS:             return "HAPTICS";
        case ALVR_EVENT_DECODER_CONFIG:      return "DECODER_CONFIG";
        case ALVR_EVENT_REAL_TIME_CONFIG:    return "REAL_TIME_CONFIG";
        default: return "?";
    }
}

JavaVM   *gVM       = nullptr;
jobject   gActivity = nullptr;
jclass    gVrClass  = nullptr;
pthread_t gThread;
std::atomic<bool> gRunning{false};   // render-thread lifetime

// Dedicated fixed-rate tracking thread. Decoupling pose read + ALVR uplink from the
// render thread's frame-paced spin keeps velocity filters rate-stable and bounds
// the tracking packet rate over Wi-Fi.
static pthread_t        gTrackThread;
static std::atomic<bool> gTrackRunning{false};   // tracking-thread lifetime
// Pause flag: when true, the tracking thread sleeps instead of calling
// Pvr_GetMainSensorState_. The SDK's internal render-thread state (that
// Pvr_GetMainSensorState_ dereferences) is null/unstable during a surface
// teardown -> recreate cycle, causing a null-deref crash in
// Pvr_GetPredictedDisplayTime_. Pausing the tracker for the duration of
// the surface swap avoids the race.
static std::atomic<bool> gTrackPaused{false};

// ALVR path ids, file scope so both the render thread and tracking thread can
// see them. Populated once after alvr_initialize().
struct BtnIds { uint64_t trigVal, gripVal, thumbX, thumbY, thumbClick, thumbTouch, menu, face1, face2; };
static uint64_t alvrHeadId = 0;
static uint64_t alvrHandId[2] = { 0, 0 };
static BtnIds   alvrBtn[2] = {};

// Stale-controller suppression: a broken/missing controller may report conn=1
// (cached CV service state) but never produce real tracking, leaving its pose
// stuck at the exact default (origin + identity quat). After kStaleThreshold
// frames of that, we skip forwarding it so the server doesn't render a ghost
// controller parked at the floor. Written by the tracking thread, read by the
// render thread (lobby pointer / model draw). Benign race: at worst the render
// thread sees a one-frame-stale decision.
static int  staleFrames[2] = { 0, 0 };
static bool staleSkip[2]   = { false, false };
static constexpr int kStaleThreshold = 150;  // ~0.5s at 300Hz before suppressing

// Window handed in by the SurfaceView callbacks. The render thread owns one
// long-lived EGL display+context; it (re)creates only the window surface as
// the SurfaceView surface is destroyed/recreated.
std::mutex     gWinMutex;
ANativeWindow *gPendingWindow = nullptr;   // latest window (or null)
std::atomic<bool> gWindowDirty{false};     // a change is pending (see render_thread.h)

pico_lobby * gLobby = new pico_lobby();
pico_passthrough * gPassthrough = new pico_passthrough();
static simple_lobby * gSimpleLobby = nullptr;

// Shared "position + per-vertex colour" shader: grid, HUD text, and eye-gaze
// marker all reuse this same program (gProg) + uMVP.
static const char *kVtxSrc =
    "#version 300 es\n"
    "layout(location=0) in vec3 aPos;\n"
    "layout(location=1) in vec3 aColor;\n"
    "uniform mat4 uMVP;\n"
    "out vec3 vColor;\n"
    "void main(){ vColor=aColor; gl_Position=uMVP*vec4(aPos,1.0); }\n";

static const char *kFrgSrc =
    "#version 300 es\n"
    "precision mediump float;\n"
    "in vec3 vColor; out vec4 oColor;\n"
    "void main(){ oColor=vec4(vColor,1.0); }\n";

static GLuint gProg = 0;
static GLint  gMvpLoc = -1;

// Build the shared pos+colour program (gProg).
static void buildGraphics() {
    gProg = glCreateProgram();
    GLuint vs = compile(GL_VERTEX_SHADER, kVtxSrc);
    GLuint fs = compile(GL_FRAGMENT_SHADER, kFrgSrc);
    glAttachShader(gProg, vs); glAttachShader(gProg, fs);
    glLinkProgram(gProg);
    GLint ok=0; glGetProgramiv(gProg, GL_LINK_STATUS, &ok);
    if (!ok) { char log[512]; glGetProgramInfoLog(gProg, 512, nullptr, log); LOGE("link: %s", log); }
    gMvpLoc = glGetUniformLocation(gProg, "uMVP");
    LOGI("graphics built prog=%u", gProg);
}

// ALVR's storage uses app_dirs2, which needs $HOME (unset on Android) -> it
// panics. Point HOME at the app's private files dir before initializing ALVR.
void setHomeFromFilesDir(JNIEnv *env, jobject activity) {
    jclass cls = env->GetObjectClass(activity);
    jmethodID m = env->GetMethodID(cls, "getFilesDir", "()Ljava/io/File;");
    jobject file = env->CallObjectMethod(activity, m);
    jclass fcls = env->GetObjectClass(file);
    jmethodID gp = env->GetMethodID(fcls, "getAbsolutePath", "()Ljava/lang/String;");
    jstring jpath = (jstring) env->CallObjectMethod(file, gp);
    const char *path = env->GetStringUTFChars(jpath, nullptr);
    setenv("HOME", path, 1);
    LOGI("set HOME=%s", path);
    env->ReleaseStringUTFChars(jpath, path);
}

// ---------- ALVR stream presentation (swapchain -> SDK warp) ----------------
// The forked stream shader writes final present-domain bytes (warm WB + sRGB
// encode + dither) directly into plain RGBA8 swapchain textures, handed to the
// SDK DIATW warp with no separate sRGB-encode blit pass.
// Ring depth: the same ring is both ALVR's render target AND the texture submitted
// to the warp. The warp keeps up to 4 submitted entries + 1 slot we're rendering
// into = 5.
static const int kSwapLen = 5;
GLuint gSwap[2][kSwapLen] = {{0},{0}};
GLuint gStreamFbo = 0;   // reusable FBO for the diag HUD overlay into gSwap
int    gSwapIdx = 0;     // render write-head into the ring
// Per-slot fence + the render pose that slot was rendered with.
static GLsync gSwapFence[kSwapLen] = {0};
static AlvrViewParams gSwapVP[kSwapLen][2] = {};
static uint64_t gSwapFrameTs[kSwapLen] = {0};
static int    gPrevSwapIdx = -1;
static bool   gPrevSwapValid = false;
// Async present state: the SDK DIATW compositor owns the window and re-projects
// the last decoded eye textures to a fresh pose every vsync; we just keep feeding
// it the newest frame (so head motion stays smooth even below video framerate).
// The +1-frame pipeline state is gPrevSwapIdx/gPrevSwapValid (above).

// The SDK warp thread owns the window: we hand it eye textures (HW lens
// distortion + async reprojection + direct present) and never self-present.
static std::atomic<bool>   gWarpToWindow{false};  // warp thread was given the real window surface
static std::atomic<bool>   gAtwEnabled{false};
uint32_t gStreamW = 0, gStreamH = 0;
// Foveation params currently APPLIED to the de-foveation pipeline. Cached so we
// can detect a server-side foveation change mid-session and re-sync.
static bool  gFoveOn = false;
static float gFovParams[6] = {0,0,0,0,0,0};   // csx,csy,shx,shy,erx,ery (currently APPLIED)
// Foveation re-sync debounce. A re-sync rebuilds the de-foveation swapchain
// (multi-MB GPU realloc), so a burst of REAL_TIME_CONFIG events coalesces into
// one rebuild once params settle ~250ms.
static float    gFovPending[6] = {0,0,0,0,0,0};
static uint64_t gFovPendingNs = 0;
static bool     gFovResyncPending = false;
static const uint64_t kFovDebounceNs = 250000000ULL;   // 250ms settle window
// Proximity sleep: Java sets gSleepReq true after the headset is OFF the head for
// the timeout, false on don. The render thread pauses ALVR (server stops sending,
// decoder torn down) to save power, then resumes on wear. alvr_pause/resume must
// run on THIS thread where the connection lives.
std::atomic<bool> gSleepReq{false};
static std::atomic<bool> gSlept{false};
// Play-area extents (meters) from the Pico boundary; forwarded to SteamVR as
// chaperone on each STREAMING_STARTED. 0 = none configured.
static float gPlayspaceW = 0.0f, gPlayspaceD = 0.0f;
// Negotiated stream refresh rate; fed to the decoder as frame-rate so the Venus
// driver stops logging "Unable to convey fps info".
static float gRefreshHint = 72.0f;
// Stream-lifecycle flags. Read/written from the render loop and the ALVR
// event handler thread, so they are atomic for cross-thread visibility.
static std::atomic<bool>   gStreaming{false};
static std::atomic<bool>   gDecoderReady{false};
static std::atomic<bool>   gAlvrGlReady{false};
// Set by STREAMING_STARTED, consumed by the video submit path: reset the frame
// pacer + per-second video counters at the start of each stream.
static bool   gResetPacer = false;


// HW-compositor lobby: a per-eye ring of textures we render the lobby into and
// hand to the SDK warp. RGBA8 holding display colours; the warp presents them as-is.
// Ring DEPTH: 5 = the warp's 4 held entries + 1 we're rendering into never alias
// (matches the video ring's kSwapLen reasoning).
static const int kLobbySz = 1536;   // per-eye lobby render target (~1.5x linear res)
static const int kLobbyRing = 5;
static GLuint gLobbyEye[2][kLobbyRing] = {{0},{0}};
static bool   gLobbyEyeReady = false;
static GLuint gLobbyFbo = 0, gLobbyDepth = 0;
// Per-slot GPU fence + the pose each slot was rendered at, so the lobby submit can
// pipeline like the video path: hand the warp the previous frame's GPU-complete slot.
static GLsync gLobbyFence[kLobbyRing] = {0};
static Quat   gLobbyPoseQ[kLobbyRing];
static float  gLobbyPoseP[kLobbyRing][3];
static void buildLobbyTarget() {
    for (int e = 0; e < 2; e++) {
        glGenTextures(kLobbyRing, gLobbyEye[e]);
        for (int i = 0; i < kLobbyRing; i++) {
            glBindTexture(GL_TEXTURE_2D, gLobbyEye[e][i]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kLobbySz, kLobbySz, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
    }
    gLobbyEyeReady = true;
    glGenRenderbuffers(1, &gLobbyDepth);
    glBindRenderbuffer(GL_RENDERBUFFER, gLobbyDepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, kLobbySz, kLobbySz);
    glGenFramebuffers(1, &gLobbyFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, gLobbyFbo);
    // Attach ring slot [0][0] for the completeness check; the render loop
    // re-attaches the live per-eye ring texture each frame.
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gLobbyEye[0][0], 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, gLobbyDepth);
    bool ok = (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    LOGI("lobby target built %dx%d ok=%d", kLobbySz, kLobbySz, ok);
}

// Release the lobby eye-texture ring + FBO/depth (~94 MB). Called when a stream
// starts so that memory isn't parked while in-game; buildLobbyTarget re-runs
// lazily the next time the lobby is shown. Requires a current context.
static void freeLobbyTarget() {
    if (!gLobbyEyeReady) return;
    for (int e = 0; e < 2; e++) {
        glDeleteTextures(kLobbyRing, gLobbyEye[e]);
        for (int i = 0; i < kLobbyRing; i++) gLobbyEye[e][i] = 0;
    }
    if (gLobbyFbo)   { glDeleteFramebuffers(1, &gLobbyFbo);   gLobbyFbo = 0; }
    if (gLobbyDepth) { glDeleteRenderbuffers(1, &gLobbyDepth); gLobbyDepth = 0; }
    for (int i = 0; i < kLobbyRing; i++) if (gLobbyFence[i]) { glDeleteSync(gLobbyFence[i]); gLobbyFence[i] = 0; }
    gLobbyEyeReady = false;
    LOGI("lobby target freed (~%d MB reclaimed)", (2*kLobbyRing*kLobbySz*kLobbySz*4)/(1024*1024));
}

// Eye-gaze debug marker: a small bright-green filled disc (triangle fan as
// GL_TRIANGLES) in the XY plane. Reuses gProg. Drawn in the lobby at the gaze
// point, billboard to the head. Only shown on a Neo 2 EYE (gated by gEyeOnline).
static GLuint gGazeVao = 0, gGazeVbo = 0;
static int    gGazeVertCount = 0;
static void buildGazeMarker() {
    const int   kSeg = 28;
    const float kR   = 0.05f;     // ~5cm disc at the gaze point
    std::vector<float> v;
    for (int i = 0; i < kSeg; i++) {
        float a0 = (float)i       / kSeg * 2.0f * (float)M_PI;
        float a1 = (float)(i + 1) / kSeg * 2.0f * (float)M_PI;
        // center, then two rim points -> one triangle per segment
        const float g[3] = { 0.2f, 0.55f, 1.0f };   // WiVRn blue
        v.insert(v.end(), { 0,0,0, g[0],g[1],g[2] });
        v.insert(v.end(), { cosf(a0)*kR, sinf(a0)*kR, 0, g[0],g[1],g[2] });
        v.insert(v.end(), { cosf(a1)*kR, sinf(a1)*kR, 0, g[0],g[1],g[2] });
    }
    gGazeVertCount = (int)(v.size() / 6);
    glGenVertexArrays(1, &gGazeVao);
    glBindVertexArray(gGazeVao);
    glGenBuffers(1, &gGazeVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gGazeVbo);
    glBufferData(GL_ARRAY_BUFFER, v.size()*sizeof(float), v.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);
    LOGI("gaze marker built verts=%d", gGazeVertCount);
}

// ---------------------------------------------------------------------------
// Controller wireframe models from the Neo 2 system assets. The meshes live on
// /system as plain Wavefront OBJ (centimetres, centred at origin, long axis = Z).
// We read v + f, emit each face's edges as line segments, scale to metres.
// 0 = left, 1 = right.
// ---------------------------------------------------------------------------
static GLuint gCtrlVao[2] = {0,0}, gCtrlVbo[2] = {0,0};
static int    gCtrlVertCount[2] = {0,0};
// Vertex format: pos.xyz + uv.xy + shade.rgb = 8 floats
static GLuint gCtrlTex[5] = {0,0,0,0,0};  // idle, app, home, touchpad, trigger
static GLuint gCtrlProg = 0;
static GLint  gCtrlMvpLoc = 0, gCtrlTexLoc = 0;
enum CtrlTex { TEX_IDLE=0, TEX_APP=1, TEX_HOME=2, TEX_TOUCH=3, TEX_TRIG=4 };
// Controller OBJ models bundled in assets, extracted to HOME/controller/.
// r.obj reads right on the LEFT hand, controller2s.obj on the RIGHT.
static std::string kCtrlObjPath[2];
static std::string kCtrlTexPath[5];
static void initCtrlObjPaths() {
    const char *home = getenv("HOME");
    if (!home) home = "/data/data/org.meumeu.wivrn.neo2.pvr/files";
    std::string base = std::string(home) + "/controller/";
    kCtrlObjPath[0] = base + "r.obj";
    kCtrlObjPath[1] = base + "controller2s.obj";
    kCtrlTexPath[TEX_IDLE]  = base + "controller2s_idle.png";
    kCtrlTexPath[TEX_APP]   = base + "controller2s_app.png";
    kCtrlTexPath[TEX_HOME]  = base + "controller2s_home.png";
    kCtrlTexPath[TEX_TOUCH] = base + "controller2s_touchpad.png";
    kCtrlTexPath[TEX_TRIG]  = base + "controller2s_trigger.png";
}
// Load OBJ with UVs. Output: interleaved pos.xyz + uv.xy per vertex (5 floats),
// triangulated. Positions converted cm -> m.
static void loadCtrlObjTextured(const char *path, std::vector<float> &outPos, std::vector<float> &outUv) {
    outPos.clear(); outUv.clear();
    FILE *f = fopen(path, "r");
    if (!f) { LOGE("ctrl obj missing: %s", path); return; }
    std::vector<float> vx, vy, vz, vt_u, vt_v;
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        if (line[0]=='v' && line[1]==' ') {
            float x,y,z;
            if (sscanf(line+2, "%f %f %f", &x,&y,&z)==3) { vx.push_back(x); vy.push_back(y); vz.push_back(z); }
        } else if (line[0]=='v' && line[1]=='t' && line[2]==' ') {
            float u,v;
            if (sscanf(line+3, "%f %f", &u,&v)==2) { vt_u.push_back(u); vt_v.push_back(v); }
        } else if (line[0]=='f' && (line[1]==' '||line[1]=='\t')) {
            int vi[16], ti[16]; int n=0;
            char *tok = strtok(line+2, " \t\r\n");
            while (tok && n<16) {
                int v_idx=0, t_idx=0;
                // format: v/vt/vn or v/vt or v
                v_idx = atoi(tok);
                const char *slash1 = strchr(tok, '/');
                if (slash1) t_idx = atoi(slash1+1);
                if (v_idx != 0) {
                    vi[n] = (v_idx > 0) ? v_idx-1 : (int)vx.size()+v_idx;
                    ti[n] = (t_idx > 0) ? t_idx-1 : (t_idx < 0) ? (int)vt_u.size()+t_idx : -1;
                    n++;
                }
                tok = strtok(nullptr, " \t\r\n");
            }
            for (int i=1; i+1<n; i++) {
                int a=vi[0], b=vi[i], c=vi[i+1];
                int ta=ti[0], tb=ti[i], tc=ti[i+1];
                if (a<0||b<0||c<0||a>=(int)vx.size()||b>=(int)vx.size()||c>=(int)vx.size()) continue;
                outPos.insert(outPos.end(), { vx[a]*0.01f,vy[a]*0.01f,vz[a]*0.01f });
                outPos.insert(outPos.end(), { vx[b]*0.01f,vy[b]*0.01f,vz[b]*0.01f });
                outPos.insert(outPos.end(), { vx[c]*0.01f,vy[c]*0.01f,vz[c]*0.01f });
                auto uv = [&](int idx) {
                    if (idx>=0 && idx<(int)vt_u.size())
                        outUv.insert(outUv.end(), { vt_u[idx], vt_v[idx] });
                    else
                        outUv.insert(outUv.end(), { 0.0f, 0.0f });
                };
                uv(ta); uv(tb); uv(tc);
            }
        }
    }
    fclose(f);
    LOGI("ctrl obj %s -> %d tri verts (UVs: %s)", path, (int)(outPos.size()/3),
         vt_u.empty() ? "no" : "yes");
}
// Load a PNG from file into a GL texture (RGBA). Uses stb_image if available,
// otherwise falls back to a raw RGBA loader. Here we use a minimal PNG decoder
// via the Android BitmapFactory through JNI is overkill; instead we use stb_image.
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
static GLuint loadCtrlTexture(const char *path) {
    int w, h, ch;
    stbi_set_flip_vertically_on_load(1); // PNG is top-down, GL is bottom-up
    unsigned char *data = stbi_load(path, &w, &h, &ch, 4);
    if (!data) { LOGE("ctrl tex load failed: %s", path); return 0; }
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
    LOGI("ctrl tex %s -> %d (%dx%d)", path, tex, w, h);
    stbi_image_free(data);
    return tex;
}
static void buildCtrlShader() {
    if (gCtrlProg) return;
    const char *vs = R"(#version 300 es
        layout(location=0) in vec3 aPos;
        layout(location=1) in vec2 aUV;
        layout(location=2) in vec3 aShade;
        uniform mat4 uMVP;
        out vec2 vUV;
        out vec3 vShade;
        void main() {
            gl_Position = uMVP * vec4(aPos, 1.0);
            vUV = aUV;
            vShade = aShade;
        })";
    const char *fs = R"(#version 300 es
        precision mediump float;
        in vec2 vUV;
        in vec3 vShade;
        uniform sampler2D uTex;
        out vec4 frag;
        void main() {
            vec4 tex = texture(uTex, vUV);
            frag = vec4(tex.rgb * vShade, 1.0);
        })";
    gCtrlProg = glCreateProgram();
    GLuint s1 = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(s1, 1, &vs, nullptr); glCompileShader(s1);
    GLint ok; glGetShaderiv(s1, GL_COMPILE_STATUS, &ok);
    if (!ok) { char log[1024]; glGetShaderInfoLog(s1, 1024, nullptr, log); LOGE("ctrl VS: %s", log); }
    GLuint s2 = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(s2, 1, &fs, nullptr); glCompileShader(s2);
    glGetShaderiv(s2, GL_COMPILE_STATUS, &ok);
    if (!ok) { char log[1024]; glGetShaderInfoLog(s2, 1024, nullptr, log); LOGE("ctrl FS: %s", log); }
    glAttachShader(gCtrlProg, s1); glAttachShader(gCtrlProg, s2);
    glLinkProgram(gCtrlProg);
    glGetProgramiv(gCtrlProg, GL_LINK_STATUS, &ok);
    if (!ok) { char log[1024]; glGetProgramInfoLog(gCtrlProg, 1024, nullptr, log); LOGE("ctrl link: %s", log); }
    glDeleteShader(s1); glDeleteShader(s2);
    gCtrlMvpLoc = glGetUniformLocation(gCtrlProg, "uMVP");
    gCtrlTexLoc = glGetUniformLocation(gCtrlProg, "uTex");
    LOGI("ctrl shader prog=%u", gCtrlProg);
}
static std::vector<float> gCtrlPosData[2], gCtrlUvData[2];
static void buildControllerMeshes() {
    if (kCtrlObjPath[0].empty()) initCtrlObjPaths();
    buildCtrlShader();
    for (int t = 0; t < 5; t++)
        if (!gCtrlTex[t] && !kCtrlTexPath[t].empty())
            gCtrlTex[t] = loadCtrlTexture(kCtrlTexPath[t].c_str());
    // Light direction (normalized) - from upper front left
    const float lx = -0.4f, ly = 0.6f, lz = -0.7f;
    float llen = sqrtf(lx*lx + ly*ly + lz*lz);
    float lxn = lx/llen, lyn = ly/llen, lzn = lz/llen;
    const float ambient = 0.4f;
    for (int h=0; h<2; h++) {
        if (gCtrlPosData[h].empty())
            loadCtrlObjTextured(kCtrlObjPath[h].c_str(), gCtrlPosData[h], gCtrlUvData[h]);
        // Build 8-float vertices: pos.xyz + uv.xy + shade.rgb
        std::vector<float> v;
        v.reserve(gCtrlPosData[h].size() / 3 * 8);
        for (size_t i = 0; i + 9 <= gCtrlPosData[h].size(); i += 9) {
            float ax = gCtrlPosData[h][i],   ay = gCtrlPosData[h][i+1], az = gCtrlPosData[h][i+2];
            float bx = gCtrlPosData[h][i+3], by = gCtrlPosData[h][i+4], bz = gCtrlPosData[h][i+5];
            float cx = gCtrlPosData[h][i+6], cy = gCtrlPosData[h][i+7], cz = gCtrlPosData[h][i+8];
            // face normal
            float ex1 = bx-ax, ey1 = by-ay, ez1 = bz-az;
            float ex2 = cx-ax, ey2 = cy-ay, ez2 = cz-az;
            float nx = ey1*ez2 - ez1*ey2;
            float ny = ez1*ex2 - ex1*ez2;
            float nz = ex1*ey2 - ey1*ex2;
            float nl = sqrtf(nx*nx + ny*ny + nz*nz);
            if (nl > 1e-6f) { nx /= nl; ny /= nl; nz /= nl; }
            float diff = nx*lxn + ny*lyn + nz*lzn;
            if (diff < 0) diff = 0;
            float shade = ambient + (1.0f - ambient) * diff;
            float sR = shade, sG = shade, sB = shade;
            size_t uvBase = i / 3 * 2;  // matching UV index
            v.insert(v.end(), { ax,ay,az, gCtrlUvData[h][uvBase],   gCtrlUvData[h][uvBase+1],   sR,sG,sB });
            v.insert(v.end(), { bx,by,bz, gCtrlUvData[h][uvBase+2], gCtrlUvData[h][uvBase+3], sR,sG,sB });
            v.insert(v.end(), { cx,cy,cz, gCtrlUvData[h][uvBase+4], gCtrlUvData[h][uvBase+5], sR,sG,sB });
        }
        gCtrlVertCount[h] = (int)(v.size()/8);
        if (!gCtrlVao[h]) {
            glGenVertexArrays(1,&gCtrlVao[h]);
            glBindVertexArray(gCtrlVao[h]);
            glGenBuffers(1,&gCtrlVbo[h]);
            glBindBuffer(GL_ARRAY_BUFFER,gCtrlVbo[h]);
            // pos.xyz
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,8*sizeof(float),(void*)0);
            // uv.xy
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1,2,GL_FLOAT,GL_FALSE,8*sizeof(float),(void*)(3*sizeof(float)));
            // shade.rgb
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,8*sizeof(float),(void*)(5*sizeof(float)));
            glBindVertexArray(0);
        }
        glBindBuffer(GL_ARRAY_BUFFER, gCtrlVbo[h]);
        glBufferData(GL_ARRAY_BUFFER, v.size()*sizeof(float), v.data(), GL_STATIC_DRAW);
    }
}

// 3D HUD text + lobby UI kit (font, appendTextLine/Quad, ui* widgets) live in
// ui_kit.h/.cpp. The dynamic VBOs they fill are still owned here.
static bool gServersOpen = false;              // server list panel shown
static GLuint gTextVao = 0, gTextVbo = 0;
static GLuint gSliderVao = 0, gSliderVbo = 0;   // lobby SETTINGS panel (dynamic)
static GLuint gSrvVao = 0, gSrvVbo = 0;         // lobby SERVER LIST panel (dynamic)
static GLuint gReticleVao = 0, gReticleVbo = 0; // head-gaze crosshair (static "+")
static int    gReticleVertCount = 0;
static GLuint gEqVao = 0, gEqVbo = 0;           // lobby 16-band audio EQ panel (dynamic)
static GLuint gLaserVao = 0, gLaserVbo = 0;     // controller laser beam (dynamic, world-space)
static GLuint gCursorVao = 0, gCursorVbo = 0;   // UI pointer cursor ring (dynamic, panel-local)
static GLuint gDiagVao = 0, gDiagVbo = 0;       // streaming diagnostics overlay (dynamic, NDC)
static GLuint gWarnVao = 0, gWarnVbo = 0;       // low-battery warning pop-up (dynamic)
static GLuint gTestVao = 0, gTestVbo = 0;         // simple 2D box + text test overlay
static int    gTestVertCount = 0;
static void buildTextBuffers() {
    glGenVertexArrays(1, &gTextVao);
    glBindVertexArray(gTextVao);
    glGenBuffers(1, &gTextVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gTextVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // slider panel
    glGenVertexArrays(1, &gSliderVao);
    glBindVertexArray(gSliderVao);
    glGenBuffers(1, &gSliderVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gSliderVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // server list panel
    glGenVertexArrays(1, &gSrvVao);
    glBindVertexArray(gSrvVao);
    glGenBuffers(1, &gSrvVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gSrvVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // 16-band EQ panel
    glGenVertexArrays(1, &gEqVao);
    glBindVertexArray(gEqVao);
    glGenBuffers(1, &gEqVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gEqVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // controller laser beam (world-space line)
    glGenVertexArrays(1, &gLaserVao);
    glBindVertexArray(gLaserVao);
    glGenBuffers(1, &gLaserVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gLaserVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // UI pointer cursor ring (panel-local, dynamic)
    glGenVertexArrays(1, &gCursorVao);
    glBindVertexArray(gCursorVao);
    glGenBuffers(1, &gCursorVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gCursorVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // diagnostics overlay
    glGenVertexArrays(1, &gDiagVao);
    glBindVertexArray(gDiagVao);
    glGenBuffers(1, &gDiagVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gDiagVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // low-battery warning pop-up
    glGenVertexArrays(1, &gWarnVao);
    glBindVertexArray(gWarnVao);
    glGenBuffers(1, &gWarnVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gWarnVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // simple 2D box + text test overlay
    glGenVertexArrays(1, &gTestVao);
    glBindVertexArray(gTestVao);
    glGenBuffers(1, &gTestVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gTestVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);
}

// Cached video decoder config so we can recreate the decoder after a proximity-sleep
// teardown without waiting for a reconnect.
static int      gDecCodec   = 0;
static uint8_t  gDecCfg[16384];
static uint64_t gDecCfgLen  = 0;
static bool     gHaveDecCfg = false;
// The decoder output-pump thread is created + destroyed with each decoder, so
// the big-core pin must be re-armed every time. createVideoDecoder clears these.
static bool     gDecoderPinned = false;
static int      gDecoderPinTries = 0;
// Same for the fork's video receive thread ("AlvrVideoRecv"), created per connection.
static bool     gVideoRecvPinned = false;
static int      gVideoRecvPinTries = 0;

// Push Software IPD + FOV to the server. Called at STREAMING_STARTED and whenever
// the IPD changes mid-stream.
static void sendViewParams() {
    // Per-eye HALF-FOV in radians. The stock server renders to exactly this, so
    // lowering gStreamFovDeg packs more pixels/degree. MUST match writeSdkFov()
    // or the server's image is stretched across the warp's map.
    const float hh = gStreamFovDeg.load() * 0.5f * (float) M_PI / 180.0f;
    AlvrViewParams vp[2];
    for (int e = 0; e < 2; e++) {
        vp[e].pose.orientation = { 0, 0, 0, 1 };
        vp[e].pose.position[0] = (e == 0 ? -softIpdM()*0.5f : softIpdM()*0.5f);
        vp[e].pose.position[1] = 0; vp[e].pose.position[2] = 0;
        vp[e].fov = { -hh, hh, hh, -hh };
    }
    alvr_send_view_params(vp);
    gSentIpdMm.store(gSoftIpdMm.load());
}

// Report HMD + controller battery levels to the server so the SteamVR dashboard
// shows real charge. HMD level from Android's power_supply sysfs; controllers
// report 0..100 via the CV service (keys[10]).
static void sendBatteryReports() {
    long cap = -1; bool plugged = false;
    FILE *f = fopen("/sys/class/power_supply/battery/capacity", "r");
    if (f) { if (fscanf(f, "%ld", &cap) != 1) cap = -1; fclose(f); }
    f = fopen("/sys/class/power_supply/battery/status", "r");
    if (f) { char s[32] = {0}; if (fgets(s, sizeof(s), f)) plugged = (strstr(s, "Charging") || strstr(s, "Full")); fclose(f); }
    if (cap >= 0) alvr_send_battery(alvrHeadId, (float) cap / 100.0f, plugged);

    // WiVRn protocol battery packet for the HMD.
    if (cap >= 0 && g_stream && g_stream->session) {
        from_headset::battery b{};
        b.charge = (float)cap / 100.0f;
        b.present = true;
        b.charging = plugged;
        g_stream->session->send_control(b);
    }

    int  cbat[2] = { -1, -1 };
    bool cconn[2] = { false, false };
    { std::lock_guard<std::mutex> lk(gCtrlMutex);
      for (int h = 0; h < 2; h++) {
          if (gCtrl[h].conn != 1) continue;
          // Treat origin+identity as not actually connected (ghost from SDK)
          if (gCtrl[h].pos[0] == 0.0f && gCtrl[h].pos[1] == 0.0f && gCtrl[h].pos[2] == 0.0f &&
              gCtrl[h].q[0] == 0.0f && gCtrl[h].q[1] == 0.0f && gCtrl[h].q[2] == 0.0f && gCtrl[h].q[3] == 1.0f)
              continue;
          cconn[h] = true;
          if (gCtrl[h].keyCount > 10) cbat[h] = gCtrl[h].keys[10];
      } }
    for (int h = 0; h < 2; h++)
        if (cbat[h] >= 0) alvr_send_battery(alvrHandId[h], (float) cbat[h] / 100.0f, false);

    // Push to Java UI
    androidUiPushBattery((int)cap, cbat[0], cconn[0], cbat[1], cconn[1]);
}

// Forward declarations for sysfs helpers (defined below readSysfsLong).
static bool diagReadLong(const char *path, long *out);
static float diagZoneTempC(const char *needle);
static float diagCpuClockPct(int lo, int hi);
static float diagCpuBusyPct();

// Push diagnostics data (pipeline + system telemetry) to the Java UI for the
// diag HUD overlay. Throttled to ~3Hz to match the old 3D overlay cadence.
static void sendDiagData() {
    int mode = gDiagHudMode.load();
    if (mode == 0) return;
    static uint64_t lastDiagPush = 0;
    uint64_t now = nowNs();
    if (now - lastDiagPush < 333000000ULL) return;
    lastDiagPush = now;

    float pipeline[12] = {0};
    float system[12] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

    // Pipeline stats from streaming_client + frame timing globals
    if (mode == 1) {
        if (g_stream) {
            pipeline[0] = g_stream->stats_total_latency_ms;
            pipeline[1] = g_stream->stats_decode_ms;
            pipeline[2] = 0;  // queue (not separately tracked)
            pipeline[3] = g_stream->stats_render_wait_ms;
        }
        pipeline[4] = g_stream ? (float)g_stream->stats_fps : 0;
        pipeline[5] = (float)gVidDecoded.load();
        pipeline[6] = (float)gVidSubmit.load();
        pipeline[7] = (float)gVidDropped.load();
        pipeline[8]  = gGapMsX10.load() / 10.0f;
        pipeline[9]  = gEncMsX10.load() / 10.0f;
        pipeline[10] = gEnqMsX10.load() / 10.0f;
        pipeline[11] = (float)gFenceTimeouts.load();
    }

    // System telemetry from sysfs
    if (mode == 2) {
        system[0] = diagCpuBusyPct();
        system[1] = diagCpuClockPct(0, 3);
        system[2] = diagCpuClockPct(4, 7);
        long gl = 0;
        system[3] = diagReadLong("/sys/class/kgsl/kgsl-3d0/devfreq/gpu_load", &gl) ? (float)gl : -1.0f;
        long gc = 0, gm = 0;
        bool okc = diagReadLong("/sys/class/kgsl/kgsl-3d0/devfreq/cur_freq", &gc);
        bool okm = diagReadLong("/sys/class/kgsl/kgsl-3d0/devfreq/max_freq", &gm);
        system[4] = (okc && okm && gm > 0) ? 100.0f * (float)gc / (float)gm : -1.0f;
        system[5] = okc ? (float)gc / 1e6f : -1.0f;
        system[6] = diagZoneTempC("cpu");
        system[7] = diagZoneTempC("gpu");
        system[8] = diagZoneTempC("ddr");
        system[9] = diagZoneTempC("aoss");
        { std::lock_guard<std::mutex> lk(gCtrlMutex);
          if (gCtrl[0].conn == 1 && gCtrl[0].keyCount > 10) system[10] = (float)gCtrl[0].keys[10];
          if (gCtrl[1].conn == 1 && gCtrl[1].keyCount > 10) system[11] = (float)gCtrl[1].keys[10]; }
    }

    androidUiPushDiag(mode, pipeline, system);
}


// (Re)create the MediaCodec video decoder from the cached stream config.
static void createVideoDecoder() {
    if (!gHaveDecCfg) return;
    // gRefreshHint is set by STREAMING_STARTED, which fires before this. If we
    // reach here with !gStreaming the ordering broke and gRefreshHint may be stale.
    if (!gStreaming)
        LOGE("createVideoDecoder: !gStreaming -- gRefreshHint(%.1f) may be stale/default", gRefreshHint);
    AlvrDecoderConfig dc = {};
    dc.codec = (AlvrCodec) gDecCodec;
    dc.force_software_decoder   = false;
    // 1.5-frame jitter buffer: lower QUEUE latency than 2.0 with no starvation.
    dc.max_buffering_frames     = 1.5f;
    dc.buffering_history_weight = 0.90f;
    int fps = (int)(gRefreshHint > 1.0f ? gRefreshHint + 0.5f : 72.0f);
    // Local (not static): alvr_create_decoder copies the options into an owned Vec
    // during the call, so the array only needs to live until then.
    AlvrMediacodecOption decOpts[5] = {};
    decOpts[0].key = "vendor.qti-ext-dec-low-latency.enable";
    decOpts[0].ty = ALVR_MEDIACODEC_PROP_TYPE_INT32; decOpts[0].value.int32 = 1;
    decOpts[1].key = "low-latency";
    decOpts[1].ty = ALVR_MEDIACODEC_PROP_TYPE_INT32; decOpts[1].value.int32 = 1;
    decOpts[2].key = "frame-rate";
    decOpts[2].ty = ALVR_MEDIACODEC_PROP_TYPE_INT32; decOpts[2].value.int32 = fps;
    // Give the Venus driver an explicit OPERATING RATE (= refresh) for a stable
    // clock, and PRIORITY 0 (realtime). operating-rate = target fps, NOT a max-clock
    // request (SHORT_MAX), that would only add heat on this thermally limited SoC.
    decOpts[3].key = "operating-rate";
    decOpts[3].ty = ALVR_MEDIACODEC_PROP_TYPE_INT32; decOpts[3].value.int32 = fps;
    decOpts[4].key = "priority";
    decOpts[4].ty = ALVR_MEDIACODEC_PROP_TYPE_INT32; decOpts[4].value.int32 = 0;
    dc.options = decOpts; dc.options_count = 5;
    dc.config_buffer = gDecCfg;
    dc.config_buffer_size = gDecCfgLen;
    alvr_create_decoder(dc);
    gDecoderReady = true;
    gDecoderPinned = false; gDecoderPinTries = 0;   // re-arm big-core pin for new pump thread
    LOGI("video decoder created codec=%d config=%llu bytes buffering=%.2f fps/operating-rate=%d (refreshHint=%.1f) opts=5",
         gDecCodec, (unsigned long long)gDecCfgLen, dc.max_buffering_frames, fps, gRefreshHint);
}

// Head-gaze crosshair: a small "+" centred at the origin (XY plane, in metres).
static void buildReticle() {
    std::vector<float> v;
    appendQuad(v, -0.020f,  0.0025f, 0.020f, -0.0025f, 0.85f, 0.90f, 1.0f);  // horizontal arm
    appendQuad(v, -0.0025f, 0.020f,  0.0025f, -0.020f, 0.85f, 0.90f, 1.0f);  // vertical arm
    gReticleVertCount = (int)(v.size() / 6);
    glGenVertexArrays(1, &gReticleVao);
    glBindVertexArray(gReticleVao);
    glGenBuffers(1, &gReticleVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gReticleVbo);
    glBufferData(GL_ARRAY_BUFFER, v.size()*sizeof(float), v.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);
}

// Simple 2D box + text test overlay, head-locked at a fixed depth.
static void buildTestOverlay() {
    std::vector<float> v;
    const char *msg = "NI HAO WO SHI ZHENXI";
    const float px = 0.004f;
    int n = (int)strlen(msg);
    float lineW = (n * 6 - 1) * px;
    float x0 = -lineW * 0.5f;
    float y0 = 0.05f;             // slightly above centre
    float pad = 0.02f;
    appendQuad(v, x0 - pad, y0 + pad * 1.5f, x0 + lineW + pad, y0 - 7 * px - pad,
               0.10f, 0.10f, 0.10f);
    appendTextLine(v, msg, y0, px, 0.95f, 0.95f, 0.95f);
    gTestVertCount = (int)(v.size() / 6);
    glGenVertexArrays(1, &gTestVao);
    glBindVertexArray(gTestVao);
    glGenBuffers(1, &gTestVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gTestVbo);
    glBufferData(GL_ARRAY_BUFFER, v.size()*sizeof(float), v.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);
}

// Push a per-eye FULL FOV (degrees) into the SDK warp's texture/projection mapping
// (fEyeTextureFov0/1 + GlobalConfig). Does NOT rebuild the warp mesh, the caller
// re-points the warp (EV_InitRenderThread) when a live change must take effect.
static void writeSdkFov(float fullDeg) {
    fEyeTextureFov0 = fullDeg;
    fEyeTextureFov1 = fullDeg;
    Pvr_SetProjectionFov(fullDeg, fullDeg);
    LOGI("writeSdkFov(%.2f): fEyeTextureFov0/1 + GlobalConfig FovDegrees set", fullDeg);
}

// Poll headset battery (~1Hz, self-throttled) and arm the low-battery popup on a
// downward crossing of 15% / 5%.
static void pollBatteryWarn() {
    static uint64_t sNext = 0;
    static int sLastBatt = -1;
    uint64_t now = nowNs();
    if (now < sNext) return;
    sNext = now + 1000000000ULL;
    long cap = -1;
    FILE *bf = fopen("/sys/class/power_supply/battery/capacity", "r");
    if (bf) { if (fscanf(bf, "%ld", &cap) != 1) cap = -1; fclose(bf); }
    if (cap < 0 || cap > 100) return;
    if (sLastBatt >= 0) {
        const int th[2] = { 15, 5 };
        for (int i = 0; i < 2; i++)
            if (sLastBatt > th[i] && cap <= th[i]) {
                gBattWarnPct.store((int) cap);
                gBattWarnStartNs.store(nowNs());
                LOGI("BATTERY: %ld%% -- low-battery warning (crossed %d%%)", cap, th[i]);
            }
    }
    sLastBatt = (int) cap;
}

// Draw ONE eye of the low-battery popup. Head-locked card that slides up, holds,
// then slides back down across the 5s window. No-op when inactive. Every render
// path calls this LAST so it layers over the diagnostics HUD. Projected at the
// warp's current texture FOV so its on-lens position is independent of the FOV setting.
static void drawBatteryWarn(int eye) {
    uint64_t bwStart = gBattWarnStartNs.load();
    if (bwStart == 0) return;
    uint64_t bwNow = nowNs();
    if (bwNow - bwStart >= kBattWarnDurNs) return;

    static std::vector<float> sWarnV;
    static int sWarnCount = 0;
    static uint64_t sWarnKey = 0;
    if (sWarnKey != bwStart) {   // rebuild geometry once per activation
        sWarnV.clear();
        // buildBatteryWarn removed - battery warning now handled by Android UI
        sWarnCount = 0;
        sWarnKey = bwStart;
    }
    if (sWarnCount <= 0) return;

    // Vertical slide: smoothstep up, hold, smoothstep down.
    float phase = (float)(bwNow - bwStart) / 1e9f;
    const float slideDur = 0.45f, yTravel = 0.55f;
    const float dur = (float)kBattWarnDurNs / 1e9f;
    float off;
    if (phase < slideDur)            { float f=phase/slideDur;        float e=f*f*(3.0f-2.0f*f); off=-(1.0f-e)*yTravel; }
    else if (phase > dur - slideDur) { float f=(dur-phase)/slideDur; if(f<0)f=0; float e=f*f*(3.0f-2.0f*f); off=-(1.0f-e)*yTravel; }
    else                              off=0.0f;

    float hudFovRad = (fEyeTextureFov0 > 1.0f ? fEyeTextureFov0 : 101.0f) * 0.01745329f;
    Mat4 proj = mat4Perspective(hudFovRad, 1.0f, 0.05f, 50.0f);
    const float a = -30.0f * 0.01745329f;
    float ca = cosf(a), sa = sinf(a);
    Mat4 rx = mat4Identity(); rx.m[5]=ca; rx.m[6]=sa; rx.m[9]=-sa; rx.m[10]=ca;
    Mat4 sc = mat4Identity(); sc.m[0]=1.125f; sc.m[5]=1.125f; sc.m[10]=1.125f;
    Mat4 model = mat4Mul(mat4Mul(mat4Translate(0.0f, -0.22f + off, -1.0f), rx), sc);
    float exh = (eye == 0 ? -softIpdM()*0.5f : softIpdM()*0.5f);
    Mat4 mvp = mat4Mul(proj, mat4Mul(mat4Translate(-exh,0,0), model));

    glDisable(GL_DEPTH_TEST); glDisable(GL_CULL_FACE); glDisable(GL_SCISSOR_TEST);
    glUseProgram(gProg);
    glBindVertexArray(gWarnVao);
    glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, mvp.m);
    glDrawArrays(GL_TRIANGLES, 0, sWarnCount);
    glBindVertexArray(0);
}

static void destroyStreamSwapchain() {
    for (int e = 0; e < 2; e++) {
        if (gSwap[e][0] != 0) glDeleteTextures(kSwapLen, gSwap[e]);
        for (int i = 0; i < kSwapLen; i++) gSwap[e][i] = 0;
    }
    // Invalidate the pipeline state: ring textures are gone, so the stale
    // "previous" index must not be handed to the warp after recreation.
    for (int i = 0; i < kSwapLen; i++) {
        if (gSwapFence[i]) { glDeleteSync(gSwapFence[i]); gSwapFence[i] = 0; }
    }
    // Also clear the stashed per-slot render poses. gPrevSwapValid=false already
    // forces the first frame onto the current slot, but a zeroed gSwapVP keeps
    // the invariant safe (a non-unit quat from stale memory would be rejected
    // by the warp's SelectRT).
    memset(gSwapVP, 0, sizeof(gSwapVP));
    memset(gSwapFrameTs, 0, sizeof(gSwapFrameTs));
    gSwapIdx = 0;
    gPrevSwapIdx = -1; gPrevSwapValid = false;
    if (gStreamFbo != 0) { glDeleteFramebuffers(1, &gStreamFbo); gStreamFbo = 0; }
}

static void createStreamSwapchain(uint32_t w, uint32_t h) {
    destroyStreamSwapchain();   // free any prior textures (reconnect / res change)
    for (int e = 0; e < 2; e++) {
        glGenTextures(kSwapLen, gSwap[e]);
        for (int i = 0; i < kSwapLen; i++) {
            glBindTexture(GL_TEXTURE_2D, gSwap[e][i]);
            // Plain RGBA8 (NOT sRGB-storage): the forked stream shader writes
            // already sRGB-encoded + dithered bytes. An sRGB-format texture would
            // re-encode on sample and the warp would show a double-encoded image.
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
    }
    glGenFramebuffers(1, &gStreamFbo);   // diag HUD overlay draws into gSwap via this
    glBindTexture(GL_TEXTURE_2D, 0);
    LOGI("created stream swapchain %ux%u x%d/eye", w, h, kSwapLen);
}

// Apply a foveation re-sync: rebuild the de-foveation swapchain + restart the
// stream renderer with new FFR params. Debounced by the caller. Caller guarantees
// gStreaming && gAlvrGlReady && gFoveOn && gStreamW>0 and GL context is current.
static void applyFoveationResync(const float np[6]) {
    memcpy(gFovParams, np, sizeof(gFovParams));
    uint32_t swapW = gStreamW, swapH = gStreamH;
    if (g_stream) {
        int ew = g_stream->eye_width.load();
        int eh = g_stream->eye_height.load();
        if (ew > 0 && eh > 0) { swapW = ew; swapH = eh; }
    }
    createStreamSwapchain(swapW, swapH);
    const uint32_t *swapArr[2] = { gSwap[0], gSwap[1] };
    AlvrStreamConfig sc = {};
    sc.view_resolution_width  = swapW;
    sc.view_resolution_height = swapH;
    sc.swapchain_textures = (const uint32_t **) swapArr;
    sc.swapchain_length   = kSwapLen;
    sc.enable_foveation        = gFoveOn;
    sc.foveation_center_size_x = np[0];
    sc.foveation_center_size_y = np[1];
    sc.foveation_center_shift_x= np[2];
    sc.foveation_center_shift_y= np[3];
    sc.foveation_edge_ratio_x  = np[4];
    sc.foveation_edge_ratio_y  = np[5];
    sc.enable_upscaling   = false;
    alvr_start_stream_opengl(sc);
    gSwapIdx = 0;   // createStreamSwapchain already reset the pipeline
    LOGI("REAL_TIME_CONFIG: de-foveation re-synced center=(%.3f,%.3f) shift=(%.3f,%.3f) edge=(%.2f,%.2f)",
         np[0],np[1],np[2],np[3],np[4],np[5]);
}

static void callVrStatic(JNIEnv *env, const char *name) {
    if (gVrClass == nullptr) return;   // firmware without VrActivity (see nativeStart)
    if (env->ExceptionCheck()) { env->ExceptionDescribe(); env->ExceptionClear(); }
    jmethodID m = env->GetStaticMethodID(gVrClass, name, "()V");
    if (m == nullptr) { env->ExceptionClear(); LOGE("no static %s", name); return; }
    env->CallStaticVoidMethod(gVrClass, m);
    if (env->ExceptionCheck()) { env->ExceptionDescribe(); env->ExceptionClear(); }
    LOGI("called VrActivity.%s", name);
}

// Upload `v` into a DYNAMIC vbo while reusing its storage. Re-specs (orphans)
// the buffer only when the data outgrows the current allocation. `cap` is the
// caller-owned tracker of the vbo's byte size.
static void uploadDynamicVbo(GLuint vbo, const std::vector<float> &v, GLsizeiptr &cap) {
    GLsizeiptr bytes = (GLsizeiptr)(v.size() * sizeof(float));
    if (bytes == 0) return;
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    if (bytes > cap) {
        cap = bytes + bytes / 2 + 256;   // grow with slack so re-spec is rare
        glBufferData(GL_ARRAY_BUFFER, cap, nullptr, GL_DYNAMIC_DRAW);
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, v.data());
}

// ===== Shared lobby panel interaction + draw helpers ====================
// Used by both the lobby path (pre-stream) and the streaming overlay path
// (manual lobby toggled mid-stream). Extracted so the two paths can't drift.

struct PanelInteract {
    // inputs
    float ptrOx, ptrOy, ptrOz;
    float ptrDx, ptrDy, ptrDz;
    bool  ptrFromController;
    bool  ptrGrab;
    float ptrStickY;
    bool  clickEdge;
    bool  grabEdge;
    // outputs
    int   sliderVertCount = 0;
    float cursorLx = 0, cursorLy = 0;
    bool  cursorOnPanel = false;
    bool  cursorPressed = false;
    bool  lobbyHover = false;
    float laserLen = 100.0f;
};

static bool rayPanelHit(const Mat4 &W,
                        float ptrOx, float ptrOy, float ptrOz,
                        float ptrDx, float ptrDy, float ptrDz,
                        bool ptrFromController, float &laserLen,
                        float &lx, float &ly, float &t)
{
    float Rx=W.m[0],Ry=W.m[1],Rz=W.m[2];
    float Ux=W.m[4],Uy=W.m[5],Uz=W.m[6];
    float Nx=W.m[8],Ny=W.m[9],Nz=W.m[10];
    float Qx=W.m[12],Qy=W.m[13],Qz=W.m[14];
    float denom = ptrDx*Nx + ptrDy*Ny + ptrDz*Nz;
    if (fabsf(denom) <= 1e-5f) return false;
    t = ((Qx-ptrOx)*Nx + (Qy-ptrOy)*Ny + (Qz-ptrOz)*Nz) / denom;
    if (t <= 0.0f) return false;
    float hx=ptrOx+ptrDx*t, hy=ptrOy+ptrDy*t, hz=ptrOz+ptrDz*t;
    float rx=hx-Qx, ry=hy-Qy, rz=hz-Qz;
    lx = rx*Rx + ry*Ry + rz*Rz;
    ly = rx*Ux + ry*Uy + rz*Uz;
    if (ptrFromController && t < laserLen) laserLen = t - 0.01f;
    return true;
}

static void updateLobbyPanel(PanelInteract &pi, const Mat4 &settingsWorld)
{
    // ImGui owns all UI input and rendering. We only need the ray-panel
    // hit test for cursor position tracking (used by the laser/cursor
    // visual and fed to ImGui via gImGui.setInput).
    float lx, ly, t;
    bool onPanel = rayPanelHit(settingsWorld, pi.ptrOx, pi.ptrOy, pi.ptrOz,
                               pi.ptrDx, pi.ptrDy, pi.ptrDz,
                               pi.ptrFromController, pi.laserLen, lx, ly, t);
    if (onPanel) {
        pi.cursorLx = lx;
        pi.cursorLy = ly;
        pi.cursorOnPanel = true;
        pi.cursorPressed = pi.ptrGrab;
        pi.lobbyHover = true;
    }
    pi.sliderVertCount = 0;
}

static void drawLaserBeam(float ox, float oy, float oz,
                          float dx, float dy, float dz, float len,
                          const Mat4 &sproj, const Mat4 &sview)
{
    float exr = ox + dx*len, eyr = oy + dy*len, ezr = oz + dz*len;
    float lv[12] = { ox,oy,oz, 0.70f,0.82f,1.00f,
                     exr,eyr,ezr, 0.70f,0.82f,1.00f };
    glBindBuffer(GL_ARRAY_BUFFER, gLaserVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(lv), lv, GL_DYNAMIC_DRAW);
    Mat4 mvp = mat4Mul(sproj, sview);
    glUseProgram(gProg);
    glBindVertexArray(gLaserVao);
    glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, mvp.m);
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);
}

static void drawSettingsPanelVerts(int sliderVertCount, const Mat4 &sproj, const Mat4 &sview, const Mat4 &settingsWorld)
{
    // 3D settings panel rendering removed - UI now composited from Android View texture
    (void)sliderVertCount; (void)sproj; (void)sview; (void)settingsWorld;
}

static void drawPointerCursor(float cursorLx, float cursorLy, bool cursorPressed,
                              const Mat4 &sproj, const Mat4 &sview, const Mat4 &settingsWorld)
{
    const int kSegs = 24;
    const float r = 0.018f;
    const float rz = 0.005f;
    float cr = 0.85f, cg = 0.92f, cb = 1.0f;
    if (cursorPressed) { cr = 0.3f; cg = 0.6f; cb = 1.0f; }
    float cv[kSegs * 2 * 6 * 3];
    int vi = 0;
    if (cursorPressed) {
        for (int i = 0; i < kSegs; i++) {
            float a0 = (float)i       / kSegs * 2.0f * (float)M_PI;
            float a1 = (float)(i + 1) / kSegs * 2.0f * (float)M_PI;
            cv[vi++] = cursorLx;            cv[vi++] = cursorLy;            cv[vi++] = rz; cv[vi++] = cr; cv[vi++] = cg; cv[vi++] = cb;
            cv[vi++] = cursorLx+cosf(a0)*r; cv[vi++] = cursorLy+sinf(a0)*r; cv[vi++] = rz; cv[vi++] = cr; cv[vi++] = cg; cv[vi++] = cb;
            cv[vi++] = cursorLx+cosf(a1)*r; cv[vi++] = cursorLy+sinf(a1)*r; cv[vi++] = rz; cv[vi++] = cr; cv[vi++] = cg; cv[vi++] = cb;
        }
    } else {
        const float ri = r * 0.6f;
        for (int i = 0; i < kSegs; i++) {
            float a0 = (float)i       / kSegs * 2.0f * (float)M_PI;
            float a1 = (float)(i + 1) / kSegs * 2.0f * (float)M_PI;
            float ox0 = cosf(a0)*r,  oy0 = sinf(a0)*r;
            float ox1 = cosf(a1)*r,  oy1 = sinf(a1)*r;
            float ix0 = cosf(a0)*ri, iy0 = sinf(a0)*ri;
            float ix1 = cosf(a1)*ri, iy1 = sinf(a1)*ri;
            cv[vi++] = cursorLx+ox0; cv[vi++] = cursorLy+oy0; cv[vi++] = rz; cv[vi++] = cr; cv[vi++] = cg; cv[vi++] = cb;
            cv[vi++] = cursorLx+ox1; cv[vi++] = cursorLy+oy1; cv[vi++] = rz; cv[vi++] = cr; cv[vi++] = cg; cv[vi++] = cb;
            cv[vi++] = cursorLx+ix1; cv[vi++] = cursorLy+iy1; cv[vi++] = rz; cv[vi++] = cr; cv[vi++] = cg; cv[vi++] = cb;
            cv[vi++] = cursorLx+ox0; cv[vi++] = cursorLy+oy0; cv[vi++] = rz; cv[vi++] = cr; cv[vi++] = cg; cv[vi++] = cb;
            cv[vi++] = cursorLx+ix1; cv[vi++] = cursorLy+iy1; cv[vi++] = rz; cv[vi++] = cr; cv[vi++] = cg; cv[vi++] = cb;
            cv[vi++] = cursorLx+ix0; cv[vi++] = cursorLy+iy0; cv[vi++] = rz; cv[vi++] = cr; cv[vi++] = cg; cv[vi++] = cb;
        }
    }
    int cursorVerts = vi / 6;
    glBindBuffer(GL_ARRAY_BUFFER, gCursorVbo);
    glBufferData(GL_ARRAY_BUFFER, vi * sizeof(float), cv, GL_DYNAMIC_DRAW);
    Mat4 mvp = mat4Mul(sproj, mat4Mul(sview, settingsWorld));
    glUseProgram(gProg);
    glBindVertexArray(gCursorVao);
    glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, mvp.m);
    glDrawArrays(GL_TRIANGLES, 0, cursorVerts);
    glBindVertexArray(0);
}

// Pin the render/submit thread to big cores + elevated priority so a background
// task can't preempt it between frame-ready and warp submit. SD845 = cpu0-3 little,
// cpu4-7 big; we pin to the top half. SCHED_FIFO usually EPERMs for a normal app,
// so we fall back to the most favourable nice value. The thread sleeps when idle,
// so an RT class here can't starve a core.
static int pinSubmitThreadForLowLatency() {
    int reservedCpu = -1;   // a big core kept OFF our mask, reserved for the warp
    long n = sysconf(_SC_NPROCESSORS_CONF);
    if (n >= 2) {
        cpu_set_t set; CPU_ZERO(&set);
        long lo = n / 2, hi = n - 1;
        // Leave the TOP big core free for the SDK warp thread so our render/submit
        // work never shares a core with the warp's per-vsync present.
        if (hi - lo >= 1) { reservedCpu = (int)hi; hi -= 1; }
        for (long c = lo; c <= hi; c++) CPU_SET((int)c, &set);
        if (sched_setaffinity(0, sizeof(set), &set) == 0)
            LOGI("submit thread pinned to big cores [%ld..%ld] (cpu%d reserved for warp)", lo, hi, reservedCpu);
        else
            LOGI("sched_setaffinity failed (errno=%d)", errno);
    }
    struct sched_param sp; sp.sched_priority = 2;   // low RT prio is plenty
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == 0) {
        LOGI("submit thread SCHED_FIFO prio=2");
    } else {
        // Normal apps can't get SCHED_FIFO -> best-effort nice (urgent-display ~ -8).
        if (setpriority(PRIO_PROCESS, 0, -8) == 0)
            LOGI("SCHED_FIFO denied (errno=%d); set nice=-8", errno);
        else
            LOGI("SCHED_FIFO denied + setpriority denied (errno=%d)", errno);
    }
    return reservedCpu;
}

// Find the first thread in THIS process whose comm == name (or 0). The
// /proc/self/task walk is why callers throttle their retries.
static pid_t findTidByComm(const char *name) {
    DIR *d = opendir("/proc/self/task");
    if (!d) return 0;
    pid_t tid = 0;
    struct dirent *e;
    while ((e = readdir(d)) != nullptr) {
        if (e->d_name[0] < '0' || e->d_name[0] > '9') continue;
        char path[80]; snprintf(path, sizeof(path), "/proc/self/task/%s/comm", e->d_name);
        FILE *f = fopen(path, "r");
        if (!f) continue;
        char comm[64] = {0};
        if (fgets(comm, sizeof(comm), f)) {
            char *nl = strchr(comm, '\n'); if (nl) *nl = 0;
            if (strcmp(comm, name) == 0) tid = (pid_t) atoi(e->d_name);
        }
        fclose(f);
        if (tid) break;
    }
    closedir(d);
    return tid;
}

// CPU set of the big-core half [n/2 .. n-1] MINUS exceptCpu. Falls back to the
// full big half if excluding would leave it empty.
static cpu_set_t bigCoreSetExcept(int exceptCpu, long &loOut, long &hiOut) {
    long n = sysconf(_SC_NPROCESSORS_CONF);
    long lo = (n >= 2) ? n / 2 : 0, hi = n - 1;
    cpu_set_t set; CPU_ZERO(&set);
    for (long c = lo; c <= hi; c++) if ((int)c != exceptCpu) CPU_SET((int)c, &set);
    if (CPU_COUNT(&set) == 0) for (long c = lo; c <= hi; c++) CPU_SET((int)c, &set);
    loOut = lo; hiOut = hi;
    return set;
}

// Raise the SDK's DIATW warp thread priority and discover which big core it runs
// on, so the caller can keep our submit + decoder threads off it. The warp must
// finish its reproject+present within every ~13.3ms refresh. The SDK re-affinitizes
// its own warp thread AFTER any affinity we set, so we READ where it landed and
// reserve THAT core. Returns the warp's big core, or -1 if not up yet.
static int pinWarpThreadForLowLatency(int guessReservedCpu) {
    pid_t warpTid = findTidByComm("WarpThread");
    if (!warpTid) return -1;   // warp not up / not named yet -> caller retries

    // Priority is separate from affinity and is NOT overridden by the SDK,
    // so still raise it above our submit thread's prio 2.
    struct sched_param sp; sp.sched_priority = 3;
    if (sched_setscheduler(warpTid, SCHED_FIFO, &sp) == 0)
        LOGI("WarpThread tid=%d SCHED_FIFO prio=3", (int)warpTid);
    else if (setpriority(PRIO_PROCESS, warpTid, -10) == 0)
        LOGI("WarpThread SCHED_FIFO denied (errno=%d); set nice=-10", errno);
    else
        LOGI("WarpThread prio elevation denied (errno=%d)", errno);

    long n = sysconf(_SC_NPROCESSORS_CONF);
    long bigLo = (n >= 2) ? n / 2 : 0, bigHi = n - 1;
    int warpCore = guessReservedCpu;
    cpu_set_t cur; CPU_ZERO(&cur);
    if (sched_getaffinity(warpTid, sizeof(cur), &cur) == 0) {
        int bigSet = 0, highBig = -1;
        for (long c = bigLo; c <= bigHi; c++) if (CPU_ISSET((int)c, &cur)) { bigSet++; highBig = (int)c; }
        if (bigSet == 1) {
            warpCore = highBig;   // SDK pinned it here -> respect it, reserve this core
            LOGI("WarpThread tid=%d is SDK-pinned to big core %d -> reserving it", (int)warpTid, warpCore);
        } else {
            // Floating across >1 big core: the SDK didn't pin it, so we pin it
            // to the top big core and reserve that.
            warpCore = (bigHi >= bigLo) ? (int)bigHi : guessReservedCpu;
            cpu_set_t one; CPU_ZERO(&one); CPU_SET(warpCore, &one);
            if (sched_setaffinity(warpTid, sizeof(one), &one) == 0)
                LOGI("WarpThread tid=%d was floating (%d big cores) -> pinned to dedicated core %d",
                     (int)warpTid, bigSet, warpCore);
            else
                LOGI("WarpThread tid=%d float-pin to cpu%d failed (errno=%d); reserving it anyway",
                     (int)warpTid, warpCore, errno);
        }
    }
    return warpCore;
}

// Pin the fork's decoder output-pump thread ("AlvrDecOutput") to the big cores +
// raise its priority. It loops dequeue_output_buffer + release_output_buffer, pacing
// how fast decoded frames land in the ImageReader. reservedCpu = the warp's dedicated
// big core, kept OUT of the decoder mask. Returns true once found + pinned.
static bool pinDecoderThreadForLowLatency(int reservedCpu) {
    pid_t decTid = findTidByComm("AlvrDecOutput");
    if (!decTid) return false;   // pump not up / not named yet -> caller retries

    long lo = 0, hi = 0;
    cpu_set_t set = bigCoreSetExcept(reservedCpu, lo, hi);   // big half minus the warp's core
    if (sched_setaffinity(decTid, sizeof(set), &set) == 0)
        LOGI("decoder pump tid=%d pinned to big cores [%ld..%ld] minus warp core %d",
             (int)decTid, lo, hi, reservedCpu);
    else
        LOGI("decoder pump affinity failed (errno=%d)", errno);
    // prio just below the warp (3), match the submit thread's 2.
    struct sched_param sp; sp.sched_priority = 2;
    if (sched_setscheduler(decTid, SCHED_FIFO, &sp) == 0)
        LOGI("decoder pump SCHED_FIFO prio=2");
    else if (setpriority(PRIO_PROCESS, decTid, -8) == 0)
        LOGI("decoder pump SCHED_FIFO denied (errno=%d); set nice=-8", errno);
    else
        LOGI("decoder pump prio elevation denied (errno=%d)", errno);
    return true;
}

// Pin the fork's video receive thread ("AlvrVideoRecv") to the big cores minus the
// warp's core, mirroring the decoder pump. Light thread, so priority is left below
// the pump/submit (nice fallback only), affinity is the win here. Returns true
// once found + pinned.
static bool pinVideoRecvThreadForLowLatency(int reservedCpu) {
    pid_t recvTid = findTidByComm("AlvrVideoRecv");
    if (!recvTid) return false;   // recv thread not up / not named yet -> caller retries

    long lo = 0, hi = 0;
    cpu_set_t set = bigCoreSetExcept(reservedCpu, lo, hi);   // big half minus the warp's core
    if (sched_setaffinity(recvTid, sizeof(set), &set) == 0)
        LOGI("video recv tid=%d pinned to big cores [%ld..%ld] minus warp core %d",
             (int)recvTid, lo, hi, reservedCpu);
    else
        LOGI("video recv affinity failed (errno=%d)", errno);
    // Elevated nice so a background task can't preempt the frame's first hop,
    // but below the pump/submit RT band.
    if (setpriority(PRIO_PROCESS, recvTid, -6) == 0)
        LOGI("video recv nice=-6");
    else
        LOGI("video recv prio elevation denied (errno=%d)", errno);
    return true;
}

// Pin the SoC CPU/GPU perf level via the Pico/QVR perf service. Always on, so we
// don't depend on the headset's Power Profile for a stable floor during streaming.
// Level range 0..5 (5 = max, 0 = system default). Thermal governor still clamps
// the GPU when hot. Root-free (Pico perf service, not sysfs).
static const int kPerfLevelMax = 5;   // max valid perf level (hardware ceiling)
// GPU is the bottleneck: a perf sweep showed the pipeline is GPU-bound. We pin GPU
// one below max (4 -> ~675MHz) to avoid the boost/throttle sawtooth at level 5.
// CPU is pinned lower (3 -> ~2092MHz) since it barely moves FPS, shedding thermal
// budget for the GPU. Thermal step-down still applies (pin-1 when hot).
static const int kGpuPerfPin = 4;
static const int kCpuPerfPin = 3;
// Baseline perf levels captured BEFORE we ever change them (the system default we
// restore to when releasing the floor).
static bool gPerfBaseCaptured = false;
static int  gPerfBaseCpu = 0, gPerfBaseGpu = 0;
static void capturePerfBaseline() {
    if (gPerfBaseCaptured) return;
    bool cs = false, gs = false; int cl = 0, gl = 0;
    GetCpuLevel(cl, cs); GetGpuLevel(gl, gs);
    gPerfBaseCpu = cl; gPerfBaseGpu = gl; gPerfBaseCaptured = true;
    LOGI("perf: baseline levels cpu=%d gpu=%d (restored when floor released)", cl, gl);
}

// Highest on-die GPU temperature in Celsius, or -1 if no GPU thermal zone found.
// Scans /sys/class/thermal for a zone whose `type` contains "gpu".
static float gpuZoneTempC() {
    // Scan for "gpu" zones ONCE and cache their indices; subsequent calls only
    // read the temp nodes.
    static int  sGpuZones[8];
    static int  sGpuN = -1;
    if (sGpuN < 0) {
        sGpuN = 0;
        for (int z = 0; z < 40 && sGpuN < 8; z++) {
            char p[128], type[64] = {0};
            snprintf(p, sizeof(p), "/sys/class/thermal/thermal_zone%d/type", z);
            FILE *f = fopen(p, "r");
            if (!f) continue;                   // gap in numbering -> keep scanning
            bool got = fgets(type, sizeof(type), f) != nullptr;
            fclose(f);
            if (got && strstr(type, "gpu")) sGpuZones[sGpuN++] = z;
        }
    }
    long best = -1;
    for (int i = 0; i < sGpuN; i++) {
        char p[128];
        snprintf(p, sizeof(p), "/sys/class/thermal/thermal_zone%d/temp", sGpuZones[i]);
        FILE *f = fopen(p, "r");
        if (!f) continue;
        long t = 0; if (fscanf(f, "%ld", &t) == 1 && t > best) best = t;
        fclose(f);
    }
    return best < 0 ? -1.0f : (float)best / 1000.0f;
}

// THERMAL-ADAPTIVE perf level. We normally hard-pin CPU/GPU, but on this thermally
// limited SD845 sustained heavy worlds drive the GPU into the high 80s/90s where the
// governor hard-clamps the clock anyway. When the GPU crosses kHotC we drop the
// requested floor one notch; we restore to the pin once it falls below kCoolC.
// Wide hysteresis prevents level flapping.
static const float kHotC  = 88.0f;   // step down above this GPU temp
static const float kCoolC = 82.0f;   // restore to the pin below this (hysteresis)
// Thermal backoff state: true once the GPU crosses kHotC, until it falls below kCoolC.
static bool perfBackedOff() {
    static bool   sBackedOff = false;
    static uint64_t sLastSample = 0;
    uint64_t now = nowNs();
    if (now - sLastSample > 1000000000ULL) {     // sample at ~1Hz; sysfs is cheap but not free
        sLastSample = now;
        float t = gpuZoneTempC();
        if (t > 0.0f) {
            if (!sBackedOff && t >= kHotC) {
                sBackedOff = true;
                LOGI("perf: GPU %.1fC >= %.1fC -- backing off perf floor to pin-1 (thermal)", t, kHotC);
            } else if (sBackedOff && t <= kCoolC) {
                sBackedOff = false;
                LOGI("perf: GPU %.1fC <= %.1fC -- restoring pinned perf floor", t, kCoolC);
            }
        }
    }
    return sBackedOff;
}
// Desired level for a domain given its base pin, applying the shared thermal step-down.
static int desiredPerfLevel(int basePin) { return perfBackedOff() ? (basePin - 1) : basePin; }

// Apply a perf level (>=0 pins CPU+GPU to it; -1 releases to baseline). Returns
// true once the perf service accepted it; SetCpuLevel returns -1 until the QVR
// service client is bound, so the caller retries.
static bool applyCpuLevel(int level) {
    capturePerfBaseline();
    bool on = (level >= 0);
    if (level > kPerfLevelMax) level = kPerfLevelMax;     // defensive clamp to valid range
    int want = on ? level : gPerfBaseCpu;
    int rc = SetCpuLevel(want, on);      // 2nd arg = sustained/static floor while pinned
    if (rc < 0) {                        // perf service client not bound yet -> retry
        static bool warned = false;
        if (!warned) { warned = true; LOGI("perf: SetCpuLevel not applied yet (rc=%d) -- service client null; retrying", rc); }
        return false;
    }
    int cl = -1; bool cs = false; GetCpuLevel(cl, cs);
    LOGI("perf: CPU target=%d (%s) -> want=%d (rc=%d; readback cpu=%d)", level, on ? "pin" : "release", want, rc, cl);
    return true;
}
static bool applyGpuLevel(int level) {
    capturePerfBaseline();
    bool on = (level >= 0);
    if (level > kPerfLevelMax) level = kPerfLevelMax;
    int want = on ? level : gPerfBaseGpu;
    int rg = SetGpuLevel(want, on);
    if (rg < 0) {
        static bool warned = false;
        if (!warned) { warned = true; LOGI("perf: SetGpuLevel not applied yet (rg=%d) -- service client null; retrying", rg); }
        return false;
    }
    int gl = -1; bool gs = false; GetGpuLevel(gl, gs);
    LOGI("perf: GPU target=%d (%s) -> want=%d (rg=%d; readback gpu=%d)", level, on ? "pin" : "release", want, rg, gl);
    return true;
}

// Read a single integer from a sysfs node (-1 on failure).
static long readSysfsLong(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    long v = -1; if (fscanf(f, "%ld", &v) != 1) v = -1;
    fclose(f);
    return v;
}

static bool diagReadLong(const char *path, long *out) {
    FILE *f = fopen(path, "r");
    if (!f) return false;
    long v = 0; int n = fscanf(f, "%ld", &v); fclose(f);
    if (n != 1) return false;
    *out = v; return true;
}

static float diagZoneTempC(const char *needle) {
    struct Cache { char needle[16]; int zones[8]; int n; };
    static Cache sCache[6];
    static int   sCacheN = 0;
    Cache *c = nullptr;
    for (int i = 0; i < sCacheN; i++)
        if (strncmp(sCache[i].needle, needle, sizeof(sCache[i].needle) - 1) == 0) { c = &sCache[i]; break; }
    if (!c && sCacheN < (int)(sizeof(sCache) / sizeof(sCache[0]))) {
        c = &sCache[sCacheN++];
        snprintf(c->needle, sizeof(c->needle), "%s", needle);
        c->n = 0;
        for (int z = 0; z < 40 && c->n < 8; z++) {
            char p[128], type[64] = {0};
            snprintf(p, sizeof(p), "/sys/class/thermal/thermal_zone%d/type", z);
            FILE *f = fopen(p, "r");
            if (!f) continue;
            if (!fgets(type, sizeof(type), f)) { fclose(f); continue; }
            fclose(f);
            if (strstr(type, needle)) c->zones[c->n++] = z;
        }
    }
    if (!c) return -1.0f;
    long best = -1;
    for (int i = 0; i < c->n; i++) {
        char p[128];
        snprintf(p, sizeof(p), "/sys/class/thermal/thermal_zone%d/temp", c->zones[i]);
        long t = 0; if (diagReadLong(p, &t) && t > best) best = t;
    }
    return best < 0 ? -1.0f : (float)best / 1000.0f;
}

static float diagCpuClockPct(int lo, int hi) {
    static long sMaxFreq[8] = {0};
    float sum = 0; int n = 0;
    for (int c = lo; c <= hi && c < 8; c++) {
        char p[128]; long cur = 0;
        if (sMaxFreq[c] == 0) {
            snprintf(p, sizeof(p), "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", c);
            if (!diagReadLong(p, &sMaxFreq[c]) || sMaxFreq[c] <= 0) { sMaxFreq[c] = -1; }
        }
        if (sMaxFreq[c] <= 0) continue;
        snprintf(p, sizeof(p), "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", c);
        if (!diagReadLong(p, &cur)) continue;
        sum += 100.0f * (float)cur / (float)sMaxFreq[c]; n++;
    }
    return n ? sum / n : -1.0f;
}

static float diagCpuBusyPct() {
    FILE *f = fopen("/proc/stat", "r");
    if (!f) return -1.0f;
    char cpu[8]; unsigned long long u=0,ni=0,s=0,id=0,io=0,irq=0,sirq=0,st=0;
    int n = fscanf(f, "%7s %llu %llu %llu %llu %llu %llu %llu %llu",
                   cpu,&u,&ni,&s,&id,&io,&irq,&sirq,&st);
    fclose(f);
    if (n < 5) return -1.0f;
    unsigned long long idle = id + io;
    unsigned long long total = u+ni+s+id+io+irq+sirq+st;
    static unsigned long long pIdle = 0, pTotal = 0;
    unsigned long long dT = total - pTotal, dI = idle - pIdle;
    pIdle = idle; pTotal = total;
    if (dT == 0) return -1.0f;
    float pct = 100.0f * (float)(dT - dI) / (float)dT;
    return pct < 0 ? 0 : (pct > 100 ? 100 : pct);
}

// Log the real GPU + per-cluster CPU clocks the SoC settled at for the perf sweep.
static void logActualClocks(const char *ctx) {
    long gcur = readSysfsLong("/sys/class/kgsl/kgsl-3d0/gpuclk");
    long gmin = readSysfsLong("/sys/class/kgsl/kgsl-3d0/devfreq/min_freq");
    long gmax = readSysfsLong("/sys/class/kgsl/kgsl-3d0/devfreq/max_freq");
    long c0   = readSysfsLong("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
    long c0m  = readSysfsLong("/sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq");
    long c4   = readSysfsLong("/sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq");
    long c4m  = readSysfsLong("/sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq");
    LOGI("perf-sweep %s: GPU cur=%ld min=%ld max=%ld MHz | little cur=%ld min=%ld MHz | big cur=%ld min=%ld MHz",
         ctx,
         gcur > 0 ? gcur / 1000000 : -1, gmin > 0 ? gmin / 1000000 : -1, gmax > 0 ? gmax / 1000000 : -1,
         c0 > 0 ? c0 / 1000 : -1, c0m > 0 ? c0m / 1000 : -1,
         c4 > 0 ? c4 / 1000 : -1, c4m > 0 ? c4m / 1000 : -1);
}

// Fixed-rate tracking/uplink thread. Reads head pose, derives filtered velocity,
// reads eye gaze + controllers, and pushes the tracking frame to ALVR, decoupled
// from the render thread so rate-stable filters hold their time constants and the
// uplink fires at a constant ~300Hz. No JNI, so it does NOT attach to the JVM.
static void *trackingThread(void *) {
    // Keep this thread on the LITTLE cores, off the gold cores the render + warp
    // threads contend for. It's only ~300Hz of light pose work.
    {
        long n = sysconf(_SC_NPROCESSORS_CONF);
        if (n >= 2) {
            cpu_set_t set; CPU_ZERO(&set);
            for (long c = 0; c < n / 2; c++) CPU_SET((int)c, &set);   // little-core half
            if (sched_setaffinity(0, sizeof(set), &set) == 0)
                LOGI("tracking thread pinned to little cores [0..%ld]", n/2 - 1);
            else
                LOGI("tracking affinity failed (errno=%d)", errno);
        }
    }
    const long  kPeriodNs = 1000000000L / 300;   // ~300Hz fixed cadence
    // Decouple the UPLINK packet rate from the FILTER step rate. The EMA filters
    // must step at the full 300Hz to hold their tuned tau's, but the server only
    // needs ~150Hz for extrapolation to 72Hz display. Step every loop but only SEND
    // every kUplinkDiv-th iteration, halving tracking packets over the Wi-Fi link.
    const int   kUplinkDiv = 2;                  // 300Hz / 2 = ~150Hz uplink
    int  tframe = 0;
    // head-velocity filter state
    Quat     prevQ   = { 0, 0, 0, 1 };
    float    prevP[3] = { 0, 0, 0 };
    uint64_t prevTs  = 0;
    bool     havePrev = false;
    float    fLin[3] = { 0, 0, 0 };   // EMA-filtered linear velocity (m/s)
    float    fAng[3] = { 0, 0, 0 };   // EMA-filtered angular velocity (rad/s, world)
    float    ctrlOrigin[3] = { 0.0f, 1.6f, 0.0f };
    bool     ctrlOriginSet = false;
    // controller button edge-detect state (only emit deltas; see render loop note)
    bool  lastBin[2][5]  = {};
    bool  binInit[2]     = { false, false };
    float lastScal[2][4] = {};
    bool  scalInit[2]    = { false, false };
    // Controller linear-velocity filter state (per hand). The CV service has no
    // hardware linear velocity, so we differentiate the world position and EMA-smooth
    // it. Stepped ONLY on a fresh CV sample to avoid dt-noise from duplicate reads.
    float    ctrlPrevP[2][3]  = {};     // last world pos used for differencing
    uint64_t ctrlPrevTs[2]    = { 0, 0 };
    bool     ctrlHavePrev[2]  = { false, false };
    float    ctrlRawP[2][3]   = {};     // last raw sample seen (new-sample detect)
    float    ctrlFLin[2][3]   = {};     // EMA-filtered CENTER linear velocity (m/s, pose frame)
    // Eye-gaze read cadence. Pvr_GetEyeTrackingData is heavy, so call it at ~100Hz
    // (every 3rd 300Hz tick) and reuse the cached gaze between ticks.
    XrPosef eyeGazeCache[2] = {};
    bool     eyeVLCache = false, eyeVRCache = false;

    // Pace the loop against an ABSOLUTE deadline so scheduling jitter + oversleep
    // don't accumulate and drift the real rate below 300Hz.
    auto tsAddNs = [](struct timespec &t, long ns){
        t.tv_nsec += ns;
        while (t.tv_nsec >= 1000000000L) { t.tv_nsec -= 1000000000L; t.tv_sec += 1; }
    };
    struct timespec nextTick;
    clock_gettime(CLOCK_MONOTONIC, &nextTick);

    while (gTrackRunning.load()) {
        // The PVR SDK nulls its internal render-thread state during a surface
        // swap, and Pvr_GetMainSensorState_ dereferences it without a null
        // check. Sleep while the render thread is doing the surface teardown +
        // warp re-point to avoid the crash.
        if (gTrackPaused.load()) { usleep(2000); continue; }
        float qx=0,qy=0,qz=0,qw=1, px=0,py=0,pz=0, vfov=90, hfov=90; int viewNumber=0;
        Pvr_GetMainSensorState(&qx,&qy,&qz,&qw,&px,&py,&pz,&vfov,&hfov,&viewNumber);
        // Publish head pose for the Java controller poller and the render thread's
        // lobby head transform.
        {
            std::lock_guard<std::mutex> lk(gHeadMutex);
            gHeadData[0]=qx; gHeadData[1]=qy; gHeadData[2]=qz; gHeadData[3]=qw;
            gHeadData[4]=px; gHeadData[5]=py; gHeadData[6]=pz;
        }
        if (!ctrlOriginSet && py > 0.5f) {
            ctrlOrigin[0] = px; ctrlOrigin[1] = py; ctrlOrigin[2] = pz;
            ctrlOriginSet = true;
            LOGI("controller world origin captured (%.2f,%.2f,%.2f)", px, py, pz);
        }

        // All alvr_send_* functions are no-op stubs in the WiVRn streaming path.
        // The pico_native_tracker handles all tracking uplink. Skip the useless ALVR
        // velocity filtering, motion building, and button edge detection. The head
        // pose is already published to gHeadData above.
        // BUT we still need to read eye tracking data here so that gGazeLocal /
        // gGazeValid / gEyeOnline get populated for the WiVRn tracker (pico_tracking
        // reads them via pollEyeGaze) and the debug overlay.
        {
            Quat headQ = quatNorm({ qx, qy, qz, qw });
            const int kEyeDiv = 3;   // 300Hz / 3 = ~100Hz
            if ((tframe % kEyeDiv) == 0) {
                XrPosef eg[2]; bool vL=false, vR=false;
                if (!readEyeGazes(eg, &vL, &vR, tframe, headQ))
                    vL = vR = false;
                eyeGazeCache[0] = eg[0]; eyeGazeCache[1] = eg[1];
                eyeVLCache = vL; eyeVRCache = vR;
            }
        }
        tframe++;
        tsAddNs(nextTick, kPeriodNs);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTick, nullptr);
        continue;

        // ---- Below: dead code (alvr_send_* are stubs) kept for reference ----
        AlvrDeviceMotion hmd = {};
        hmd.device_id = alvrHeadId;
        hmd.pose.orientation = { qx, qy, qz, qw };
        hmd.pose.position[0] = px; hmd.pose.position[1] = py; hmd.pose.position[2] = pz;

        const uint64_t ts = nowNs();

        // ---- Head-velocity prediction (server-side extrapolation) -----------
        {
            Quat curQ = quatNorm({ qx, qy, qz, qw });
            if (havePrev) {
                double dt = (double)(ts - prevTs) * 1e-9;
                if (dt > 0.002 && dt < 0.05) {        // 2ms..50ms; else skip this sample
                    // dt-normalized EMA (rate-stable): tau=0.66s, correct at the
                    // ~300Hz this thread runs at (independent of step rate).
                    const float a = 1.0f - expf(-(float)dt / 0.66f);
                    float lvx = (float)((px - prevP[0]) / dt);
                    float lvy = (float)((py - prevP[1]) / dt);
                    float lvz = (float)((pz - prevP[2]) / dt);
                    Quat dq = quatNorm(quatMul(curQ, quatConj(prevQ)));
                    if (dq.w < 0) { dq.x=-dq.x; dq.y=-dq.y; dq.z=-dq.z; dq.w=-dq.w; }
                    float w   = dq.w > 1.0f ? 1.0f : dq.w;
                    float ang = 2.0f * acosf(w);
                    float s   = sqrtf(1.0f - w*w);
                    float avx = 0, avy = 0, avz = 0;
                    if (s > 1e-6f) {
                        float k = (ang / s) / (float)dt;
                        avx = dq.x * k; avy = dq.y * k; avz = dq.z * k;
                    }
                    fLin[0] += (lvx - fLin[0]) * a; fLin[1] += (lvy - fLin[1]) * a; fLin[2] += (lvz - fLin[2]) * a;
                    fAng[0] += (avx - fAng[0]) * a; fAng[1] += (avy - fAng[1]) * a; fAng[2] += (avz - fAng[2]) * a;
                }
            }
            prevQ = curQ; prevP[0]=px; prevP[1]=py; prevP[2]=pz; prevTs = ts; havePrev = true;
            // Position: scale the reported linear velocity by the same predict factor
            // the controllers use (kBasePredict), so the head's positional extrapolation
            // reach matches the hands' instead of running at full 1.0. Rotation is left
            // unscaled, the DIATW warp already reprojects orientation to the live pose
            // every vsync locally, so the server-side angular reach isn't the lever here.
            float hp = kBasePredict;
            if (hp < 0.0f) hp = 0.0f; else if (hp > 1.0f) hp = 1.0f;
            hmd.linear_velocity[0]  = fLin[0] * hp; hmd.linear_velocity[1]  = fLin[1] * hp; hmd.linear_velocity[2]  = fLin[2] * hp;
            hmd.angular_velocity[0] = fAng[0]; hmd.angular_velocity[1] = fAng[1]; hmd.angular_velocity[2] = fAng[2];
        }

        // Eye gaze (Neo 2 EYE); no-op on non-Eye units (returns false). Read at
        // ~100Hz and reuse the cache between reads (the in-between uplinks still carry
        // the latest gaze). The cached pose is global-space (composed with the headQ
        // at read time); ~10ms of head motion between reads is negligible for gaze OSC.
        XrPosef eyeGaze[2];
        const XrPosef *eyeGazePtr[2] = { nullptr, nullptr };
        const XrPosef *const *eyeGazes = nullptr;
        const int kEyeDiv = 3;   // 300Hz / 3 = ~100Hz
        if ((tframe % kEyeDiv) == 0) {
            if (!readEyeGazes(eyeGazeCache, &eyeVLCache, &eyeVRCache, tframe, quatNorm({ qx, qy, qz, qw })))
                eyeVLCache = eyeVRCache = false;   // blink/invalid -> don't forward stale gaze
        }
        eyeGaze[0] = eyeGazeCache[0]; eyeGaze[1] = eyeGazeCache[1];
        bool vL = eyeVLCache, vR = eyeVRCache;
        if (vL) eyeGazePtr[0] = &eyeGaze[0];
        if (vR) eyeGazePtr[1] = &eyeGaze[1];
        if (vL || vR) eyeGazes = eyeGazePtr;

        // ---- Controllers: forward 6DoF (+touchpad) to ALVR (Neo 2 EYE) -------
        AlvrDeviceMotion motions[3];
        motions[0] = hmd;
        int motionCount = 1;
        CtrlState cs[2];
        { std::lock_guard<std::mutex> lk(gCtrlMutex); cs[0] = gCtrl[0]; cs[1] = gCtrl[1]; }
        // Stale-controller detection: a broken/missing controller may report
        // conn=1 (cached CV service state) but never produce real tracking,
        // leaving its pose stuck at the exact default (origin + identity quat).
        // If that holds for a sustained window, skip forwarding it so the server
        // doesn't render a ghost controller parked at the floor. The moment real
        // tracking arrives (pose deviates from default), the hand re-arms.
        for (int h = 0; h < 2; h++) {
            bool isDefault = cs[h].conn == 1 &&
                cs[h].pos[0] == 0.0f && cs[h].pos[1] == 0.0f && cs[h].pos[2] == 0.0f &&
                cs[h].q[0] == 0.0f && cs[h].q[1] == 0.0f && cs[h].q[2] == 0.0f && cs[h].q[3] == 1.0f;
            if (isDefault) {
                if (staleFrames[h] < kStaleThreshold) staleFrames[h]++;
                if (staleFrames[h] >= kStaleThreshold && !staleSkip[h]) {
                    LOGI("CTRL[%d] stale (conn=1 but pose at origin for >%.1fs) -> suppressing ghost",
                         h, kStaleThreshold / 300.0f);
                    staleSkip[h] = true;
                }
            } else {
                if (staleSkip[h]) LOGI("CTRL[%d] tracking resumed -> forwarding", h);
                staleSkip[h] = false;
                staleFrames[h] = 0;
            }
        }
        // In the lobby (manual lobby over a live stream) suppress ALL controller
        // input to the server, motions AND buttons, so interacting with our
        // menu doesn't fire ghost actions in the SteamVR scene. HMD pose + eye data
        // still flow (not "actions"). Only the HMD motion is forwarded.
        bool inLobby = gManualLobby.load();
        for (int h = 0; h < 2 && !inLobby; h++) {
            if (!cs[h].fresh || cs[h].conn != 1 || staleSkip[h]) continue;
            AlvrDeviceMotion &m = motions[motionCount++];
            m = {};
            m.device_id = alvrHandId[h];
            Quat oq = { -cs[h].q[0], -cs[h].q[1], cs[h].q[2], cs[h].q[3] };
            float kYawOutDeg = kBaseYawDeg;   // baked baseline
            float yaw = (h == 0 ? +kYawOutDeg : -kYawOutDeg) * 0.01745329f;
            Quat qYaw = { 0.0f, sinf(yaw*0.5f), 0.0f, cosf(yaw*0.5f) };
            oq = quatNorm(quatMul(oq, qYaw));
            // Controller rotation offset, baked from the ALVR dashboard "Left
            // controller rotation offset" (XYZ euler degrees). The server applies it
            // as a final LOCAL post-multiply (Quat::from_euler(XYZ); see server_core
            // tracking/mod.rs) with the RIGHT hand mirrored on Y and Z (X unchanged).
            // We replicate it here so it lives in client code, keep the dashboard
            // values at 0 to avoid double-applying. On TOP of the kYawOutDeg term.
            float kRotOffXDeg = kBaseRotXDeg;   // baseline; left X (right uses same)
            float kRotOffYDeg = kBaseRotYDeg;   // baseline; left Y (right negated)
            static const float kRotOffZDeg =  0.0f;    // left Z (right negated)
            float rx = kRotOffXDeg * 0.01745329f;
            float ry = (h == 0 ? kRotOffYDeg : -kRotOffYDeg) * 0.01745329f;
            float rz = (h == 0 ? kRotOffZDeg : -kRotOffZDeg) * 0.01745329f;
            Quat qrx = { sinf(rx*0.5f), 0.0f, 0.0f, cosf(rx*0.5f) };
            Quat qry = { 0.0f, sinf(ry*0.5f), 0.0f, cosf(ry*0.5f) };
            Quat qrz = { 0.0f, 0.0f, sinf(rz*0.5f), cosf(rz*0.5f) };
            Quat qRotOff = quatMul(quatMul(qrx, qry), qrz);   // from_euler(XYZ)
            oq = quatNorm(quatMul(oq, qRotOff));
            m.pose.orientation = { oq.x, oq.y, oq.z, oq.w };
            // Grip offset = a FIXED point on the physical controller, so it lives in
            // the controller's LOCAL frame and must rotate with it. (Was applied in
            // world space, which kept the offset world-aligned while the controller
            // rotated -> the reported origin drifted off the real one along whatever
            // axis you pitched/rolled.) Rotate the local offset by the SAME reported
            // orientation, then add to the raw tracked position. Local axes (in the
            // controller's own frame): X = sideways, Y = up/down, Z = front/back.
            //   +X = toward the controller's RIGHT side   / -X = toward its LEFT
            //   +Y = up toward the buttons (top face)      / -Y = down toward the trigger
            //   +Z = back toward the wrist                 / -Z = forward toward the tip
            // kGripSideLocal is mirrored for the right hand (the two controllers are
            // mirror images), like kYawOutDeg above; the other two apply to both hands.
            float kGripSideMm = kBaseGripSideMm, kGripUpMmV = kBaseGripUpMm, kGripBackMmV = kBaseGripBackMm;
            float kGripSideLocal = kGripSideMm * 0.001f;  // baseline; X
            float kGripUpLocal   = kGripUpMmV  * 0.001f;  // Y
            float kGripBackLocal = kGripBackMmV * 0.001f; // Z
            float sideSign = (h == 0) ? +1.0f : -1.0f;    // mirror X on the right hand
            float gripLocal[3] = { kGripSideLocal * sideSign, kGripUpLocal, kGripBackLocal };
            // ROT SWING: split the offset into an ORBITING part (rotated by the full
            // controller orientation -> swings with pitch/roll, the felt pivot sits
            // at the SDK tracked point) and a LEVEL part (rotated by heading ONLY ->
            // stays gravity-level, so the controller spins more in place). swing=1 is
            // fully rigid (old behavior); swing=0 removes pitch/roll orbit. At a level,
            // forward pose the two frames coincide, so the rest position is unchanged.
            float swing = kBaseRotSwing;   // baseline
            if (swing < 0.0f) swing = 0.0f; else if (swing > 1.0f) swing = 1.0f;
            // Heading-only quaternion = twist of oq about the up (Y) axis (swing-twist).
            float yn = sqrtf(oq.y*oq.y + oq.w*oq.w);
            Quat oqYaw = (yn > 1e-6f) ? Quat{ 0.0f, oq.y/yn, 0.0f, oq.w/yn }
                                      : Quat{ 0.0f, 0.0f, 0.0f, 1.0f };
            float gripOrbit[3] = { gripLocal[0]*swing,        gripLocal[1]*swing,        gripLocal[2]*swing };
            float gripLevel[3] = { gripLocal[0]*(1.0f-swing), gripLocal[1]*(1.0f-swing), gripLocal[2]*(1.0f-swing) };
            float wOrbit[3], wLevel[3];
            quatRotateVec(oq,    gripOrbit, wOrbit);
            quatRotateVec(oqYaw, gripLevel, wLevel);
            float gripWorld[3] = { wOrbit[0]+wLevel[0], wOrbit[1]+wLevel[1], wOrbit[2]+wLevel[2] };
            float wpx = cs[h].pos[0]*0.001f + gripWorld[0];
            float wpy = cs[h].pos[1]*0.001f + gripWorld[1];
            float wpz = cs[h].pos[2]*0.001f + gripWorld[2];
            m.pose.position[0] = wpx;
            m.pose.position[1] = wpy;
            m.pose.position[2] = wpz;
            // Angular velocity: hardware gyro, re-expressed into the converted pose
            // frame. The pose orientation is conjugated by a 180deg Z flip (the
            // (-x,-y,z,w) mapping), so a world-frame angular-velocity vector maps the
            // same way: (wx,wy,wz) -> (-wx,-wy,wz). Forwarding it raw would fight the
            // converted pose during fast turns.
            // Prediction strength: the server extrapolates pose + velocity*dt, so
            // scaling the reported velocities scales the prediction reach. 0.75 backs
            // off ~25%, the raw rates overshoot on quick stops/flicks.
            float kCtrlPredict = kBasePredict;   // baseline
            if (kCtrlPredict < 0.0f) kCtrlPredict = 0.0f; else if (kCtrlPredict > 1.0f) kCtrlPredict = 1.0f;
            // Angular velocity: hardware gyro re-expressed into the converted pose
            // frame (-wx,-wy,wz to match the (-x,-y,z,w) orientation flip). This SAME
            // omega also drives the grip lever's linear velocity below, so position
            // and rotation extrapolate by exactly the same amount (no shear).
            float omega[3] = { -cs[h].angVel[0], -cs[h].angVel[1], cs[h].angVel[2] };
            m.angular_velocity[0] = omega[0] * kCtrlPredict;
            m.angular_velocity[1] = omega[1] * kCtrlPredict;
            m.angular_velocity[2] = omega[2] * kCtrlPredict;
            // Linear velocity = velocity of the Pico CENTER + omega x lever. The
            // center term is differentiated (smoothed); the lever-swing term is
            // computed ANALYTICALLY from the same omega we send, so the rotation
            // prediction and the lever's positional prediction stay locked together
            // (numerically differentiating the whole position incl. the lever would
            // lag the crisp gyro rotation and shear the rigid body during fast turns).
            float leverVel[3] = {
                omega[1]*wOrbit[2] - omega[2]*wOrbit[1],
                omega[2]*wOrbit[0] - omega[0]*wOrbit[2],
                omega[0]*wOrbit[1] - omega[1]*wOrbit[0],
            };
            float cx = cs[h].pos[0]*0.001f, cy = cs[h].pos[1]*0.001f, cz = cs[h].pos[2]*0.001f;
            bool ctrlNew = !ctrlHavePrev[h] ||
                cs[h].pos[0]!=ctrlRawP[h][0] || cs[h].pos[1]!=ctrlRawP[h][1] ||
                cs[h].pos[2]!=ctrlRawP[h][2];
            if (ctrlNew) {
                if (ctrlHavePrev[h]) {
                    double dt = (double)(ts - ctrlPrevTs[h]) * 1e-9;
                    if (dt > 0.002 && dt < 0.05) {
                        // tau=0.05s: light smoothing (~one CV sample period) on the
                        // center translation, the lever term is unfiltered (analytic).
                        const float a = 1.0f - expf(-(float)dt / 0.05f);
                        float lvx = (float)((cx - ctrlPrevP[h][0]) / dt);
                        float lvy = (float)((cy - ctrlPrevP[h][1]) / dt);
                        float lvz = (float)((cz - ctrlPrevP[h][2]) / dt);
                        ctrlFLin[h][0] += (lvx - ctrlFLin[h][0]) * a;
                        ctrlFLin[h][1] += (lvy - ctrlFLin[h][1]) * a;
                        ctrlFLin[h][2] += (lvz - ctrlFLin[h][2]) * a;
                    }
                }
                ctrlPrevP[h][0]=cx; ctrlPrevP[h][1]=cy; ctrlPrevP[h][2]=cz;
                ctrlPrevTs[h]=ts; ctrlHavePrev[h]=true;
                ctrlRawP[h][0]=cs[h].pos[0]; ctrlRawP[h][1]=cs[h].pos[1]; ctrlRawP[h][2]=cs[h].pos[2];
            }
            m.linear_velocity[0] = (ctrlFLin[h][0] + leverVel[0]) * kCtrlPredict;
            m.linear_velocity[1] = (ctrlFLin[h][1] + leverVel[1]) * kCtrlPredict;
            m.linear_velocity[2] = (ctrlFLin[h][2] + leverVel[2]) * kCtrlPredict;
        }
        if ((tframe % 38) == 0) {
            for (int h = 0; h < 2; h++) {
                if (!cs[h].fresh || cs[h].conn != 1) continue;
                LOGI("CTRLALIGN head p=(%.3f,%.3f,%.3f) q=(%.3f,%.3f,%.3f,%.3f) | hand%d RAW pmm=(%.0f,%.0f,%.0f) q=(%.3f,%.3f,%.3f,%.3f)",
                     px, py, pz, qx, qy, qz, qw,
                     h, cs[h].pos[0], cs[h].pos[1], cs[h].pos[2],
                     cs[h].q[0], cs[h].q[1], cs[h].q[2], cs[h].q[3]);
            }
        }

        // Send pose+gaze every kUplinkDiv-th step (~150Hz); the filters above
        // already stepped this iteration regardless, so decimating only the SEND
        // keeps them rate-stable while halving the uplink packet rate.
        bool doUplink = (tframe % kUplinkDiv) == 0;

        // Forward eye blink (per-eye openness) once eye tracking has data. The
        // openness is latched to be sent with the NEXT alvr_send_tracking, so only
        // push it on uplink steps (the smoothing itself still runs every iteration).
        if (gEyeHaveOpen) {
            // dt-normalized blink smoothing (tau=0.011s ~= alpha 0.70 at 72Hz).
            static uint64_t sBlinkPrevTs = 0;
            float kBlinkAlpha = 1.0f;
            if (sBlinkPrevTs != 0) {
                double bdt = (double)(ts - sBlinkPrevTs) * 1e-9;
                if (bdt > 0.0 && bdt < 0.05) kBlinkAlpha = 1.0f - expf(-(float)bdt / 0.011f);
            }
            sBlinkPrevTs = ts;
            gEyeOpenSmooth[0] += (gEyeOpen[0] - gEyeOpenSmooth[0]) * kBlinkAlpha;
            gEyeOpenSmooth[1] += (gEyeOpen[1] - gEyeOpenSmooth[1]) * kBlinkAlpha;
            if (doUplink) alvr_send_eye_openness(gEyeOpenSmooth[0], gEyeOpenSmooth[1]);
        }
        if (doUplink) alvr_send_tracking(ts, motions, motionCount, nullptr,
                                         (const struct AlvrPose *const *)eyeGazes);

        // ---- Controller buttons: send only on CHANGE (edge) ------------------
        auto sendBinEdge = [&](int h, int i, uint64_t id, bool v) {
            if (binInit[h] && lastBin[h][i] == v) return;
            lastBin[h][i] = v;
            AlvrButtonValue b; b.tag = ALVR_BUTTON_VALUE_BINARY; b.binary = v;
            alvr_send_button(id, b);
        };
        auto sendScalEdge = [&](int h, int i, uint64_t id, float v) {
            if (scalInit[h] && lastScal[h][i] == v) return;
            lastScal[h][i] = v;
            AlvrButtonValue b; b.tag = ALVR_BUTTON_VALUE_SCALAR; b.scalar = v;
            alvr_send_button(id, b);
        };
        // Thumbstick response curve. The Neo 2 stick reports LINEARLY, which feels
        // touchy: a small deflection already commands a big value, so locomotion
        // ramps up fast. Reshape with an expo (blend of cubic + linear) so small
        // inputs are gentle while full throw still reaches 1.0. Plus a center
        // deadzone (resting drift) AND an outer saturation band: the stick can't
        // physically reach its reported max, so the last kStickOuter of travel is
        // dead, clamp anything past (1 - kStickOuter) to full 1.0 and rescale the
        // ramp into the usable [kStickDead, 1 - kStickOuter] band. Tuning:
        //   kStickExpo  0=linear .. 1=pure cubic (more = more relaxed near center)
        //   kStickDead  fraction of travel ignored at center
        //   kStickOuter fraction of travel at the edge that all reads as 100%
        static const float kStickExpo  = 0.6f;
        static const float kStickDead  = 0.06f;
        static const float kStickOuter = 0.20f;
        // Inferred-touch threshold: no capacitive pad on these sticks, so treat any
        // deflection past this radius as "finger on the stick". Kept just under
        // kStickDead so touch always latches before locomotion starts moving.
        static const float kStickTouch = 0.05f;
        auto stickCurve = [](float x) -> float {
            float s = (x < 0.0f) ? -1.0f : 1.0f;
            float a = fabsf(x); if (a > 1.0f) a = 1.0f;
            if (a <= kStickDead) return 0.0f;
            float top = 1.0f - kStickOuter;
            if (a >= top) return s * 1.0f;                 // outer band -> full deflection
            a = (a - kStickDead) / (top - kStickDead);     // rescale usable travel to [0,1]
            float curved = kStickExpo * (a*a*a) + (1.0f - kStickExpo) * a;
            return s * curved;
        };
        for (int h = 0; h < 2 && !inLobby; h++) {
            if (!cs[h].fresh || cs[h].conn != 1 || cs[h].keyCount < 10 || staleSkip[h]) continue;
            const int *k = cs[h].keys;
            if (k[2]||k[3]||k[4]||k[5]||k[6]||k[7])
                LOGI("BTN[%d] trig=%d grip=%d joyClk=%d menu=%d A/X=%d B/Y=%d (joy %d,%d trigA=%d gripA=%d)",
                     h, k[2],k[3],k[4],k[5],k[6],k[7], k[0],k[1],k[8],k[9]);
            // The stick reports a SQUARE range: a full diagonal gives x~1 AND y~1,
            // a vector of radius sqrt(2). SteamVR expects a CIRCULAR stick (radius
            // <= 1), so trim the square corners to the unit circle, clamp the
            // post-curve vector's magnitude to 1 (cardinals untouched; only the
            // diagonal corner is pulled in to the rim). The per-axis response curve
            // above (deadzone/expo/outer band) is preserved.
            float stx = stickCurve((k[0] - 128) / 128.0f);
            float sty = stickCurve((k[1] - 128) / 128.0f);
            float mag2 = stx*stx + sty*sty;
            if (mag2 > 1.0f) { float inv = 1.0f / sqrtf(mag2); stx *= inv; sty *= inv; }
            sendScalEdge(h, 0, alvrBtn[h].thumbX, stx);
            sendScalEdge(h, 1, alvrBtn[h].thumbY, sty);
            float trig = k[8] / 255.0f; if (trig < 0) trig = 0; if (trig > 1) trig = 1;
            sendScalEdge(h, 2, alvrBtn[h].trigVal, trig);
            float grip = k[9] / 255.0f; if (grip < 0) grip = 0; if (grip > 1) grip = 1;
            if (grip == 0.0f && k[3]) grip = 1.0f;
            sendScalEdge(h, 3, alvrBtn[h].gripVal, grip);
            sendBinEdge(h, 0, alvrBtn[h].thumbClick, k[4] != 0);
            // Inferred stick touch: deflection past kStickTouch, or a click (which is
            // physically a press, so the finger is certainly on it). Uses the RAW
            // radius, not the post-curve stx/sty, so the deadzone doesn't hide it.
            float rx = (k[0] - 128) / 128.0f, ry = (k[1] - 128) / 128.0f;
            bool  stickTouched = (rx*rx + ry*ry) > (kStickTouch * kStickTouch) || k[4] != 0;
            sendBinEdge(h, 4, alvrBtn[h].thumbTouch, stickTouched);
            sendBinEdge(h, 1, alvrBtn[h].menu,       k[5] != 0);
            sendBinEdge(h, 2, alvrBtn[h].face1,      k[6] != 0);
            sendBinEdge(h, 3, alvrBtn[h].face2,      k[7] != 0);
            binInit[h] = true;
            scalInit[h] = true;
        }

        tframe++;
        // Advance the absolute deadline by exactly one period and sleep to it. If
        // we overran (deadline already past), re-anchor to now instead of firing a
        // catch-up burst of zero-length iterations (which would spike the uplink rate
        // and inject tiny-dt samples into the EMA filters).
        tsAddNs(nextTick, kPeriodNs);
        struct timespec nowT;
        clock_gettime(CLOCK_MONOTONIC, &nowT);
        if (nowT.tv_sec > nextTick.tv_sec ||
            (nowT.tv_sec == nextTick.tv_sec && nowT.tv_nsec > nextTick.tv_nsec)) {
            nextTick = nowT;   // overran -> re-anchor; no sleep this iteration
        } else {
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTick, nullptr);
        }
    }
    LOGI("tracking thread exit at tframe %d", tframe);
    return nullptr;
}

// Apply HMD panel brightness from a 0..1 slider fraction. Primary path is the Pico
// SDK backlight control; falls back to Android window-brightness API if that fails.
static void applyHmdBrightness(float frac, JNIEnv *env) {
    if (frac < 0.0f) frac = 0.0f; else if (frac > 1.0f) frac = 1.0f;
    int level = kBrightMin + (int)(frac * (float)(kBrightMax - kBrightMin) + 0.5f);
    SetHmdScreenBrightness(level);
    int rb = -1; GetHmdScreenBrightness(rb);
    int diff = rb - level; if (diff < 0) diff = -diff;
    bool pvrOk = (rb >= 0 && diff <= 32);
    if (!pvrOk && env && gActivity) {   // Pico path inert -> Android window brightness
        jclass cls = env->GetObjectClass(gActivity);
        jmethodID m = cls ? env->GetMethodID(cls, "setWindowBrightness", "(F)V") : nullptr;
        if (m) env->CallVoidMethod(gActivity, m, (jfloat) frac);
        if (cls) env->DeleteLocalRef(cls);
        if (env->ExceptionCheck()) env->ExceptionClear();
        LOGI("brightness: Pvr readback=%d (wanted %d) -> Android fallback %.2f", rb, level, frac);
    } else {
        LOGI("brightness: Pvr set level=%d (readback=%d)", level, rb);
    }
}

void *renderThread(void *) {
    // Reset file-scope statics that survive a nativeStop->nativeStart in the same
    // process: a relaunch builds a fresh EGL context, so any GL object name or
    // "already done" latch left from the previous context is stale and must be
    // zeroed. Without this, guards like `if (gCtrlProg) return;` skip re-creation
    // and the render loop draws with invalid GL names -> missing faces, dead UI.
    gWarpToWindow = false;
    gAtwEnabled = false;
    gPerfBaseCaptured = false;

    // GL programs
    gProg = 0;
    gMvpLoc = -1;
    gCtrlProg = 0;
    gCtrlMvpLoc = 0;
    gCtrlTexLoc = 0;

    // Controller mesh GL resources
    for (int h = 0; h < 2; h++) {
        gCtrlVao[h] = 0;
        gCtrlVbo[h] = 0;
        gCtrlVertCount[h] = 0;
    }
    for (int t = 0; t < 5; t++) gCtrlTex[t] = 0;
    // Force OBJ reload so VBOs get rebuilt with fresh vertex data.
    for (int h = 0; h < 2; h++) {
        gCtrlPosData[h].clear();
        gCtrlUvData[h].clear();
    }

    // Lobby eye-texture ring + FBO
    for (int e = 0; e < 2; e++)
        for (int i = 0; i < kLobbyRing; i++) gLobbyEye[e][i] = 0;
    gLobbyFbo = 0;
    gLobbyDepth = 0;
    gLobbyEyeReady = false;
    for (int i = 0; i < kLobbyRing; i++) gLobbyFence[i] = 0;

    // Stream swapchain
    for (int e = 0; e < 2; e++)
        for (int i = 0; i < kSwapLen; i++) gSwap[e][i] = 0;
    gStreamFbo = 0;
    gSwapIdx = 0;
    gPrevSwapIdx = -1;
    gPrevSwapValid = false;
    for (int i = 0; i < kSwapLen; i++) gSwapFence[i] = 0;
    gStreamW = 0;
    gStreamH = 0;

    // UI VAOs/VBOs
    gTextVao = 0; gTextVbo = 0;
    gSliderVao = 0; gSliderVbo = 0;
    gSrvVao = 0; gSrvVbo = 0;
    gReticleVao = 0; gReticleVbo = 0;
    gReticleVertCount = 0;
    gEqVao = 0; gEqVbo = 0;
    gLaserVao = 0; gLaserVbo = 0;
    gCursorVao = 0; gCursorVbo = 0;
    gDiagVao = 0; gDiagVbo = 0;
    gWarnVao = 0; gWarnVbo = 0;
    gTestVao = 0; gTestVbo = 0;
    gTestVertCount = 0;

    // Eye-gaze marker
    gGazeVao = 0; gGazeVbo = 0;
    gGazeVertCount = 0;

    // Stream / foveation state
    gFoveOn = false;
    gFovResyncPending = false;
    gResetPacer = false;
    gStreaming = false;
    gDecoderReady = false;
    gAlvrGlReady = false;
    gSlept = false;

    // Passthrough and simple_lobby: their init() guards on a bool that survives
    // across launches. Delete and recreate so they build fresh GL resources.
    if (gSimpleLobby) { delete gSimpleLobby; gSimpleLobby = nullptr; }
    if (gPassthrough) { delete gPassthrough; gPassthrough = new pico_passthrough(); }
    // Android View UI: global object with init guard.
    gAndroidUi.reset();
    JNIEnv *env = nullptr;
    gVM->AttachCurrentThread(&env, nullptr);
    int reservedCpu = pinSubmitThreadForLowLatency();   // big-core affinity + prio; returns core reserved for warp
    bool warpPinned = false;          // warp thread pinned once it exists

    EGLDisplay dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    eglInitialize(dpy, nullptr, nullptr);
    const EGLint cfgAttribs[] = {
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 24, EGL_NONE
    };
    EGLConfig cfg; EGLint n = 0;
    eglChooseConfig(dpy, cfgAttribs, &cfg, 1, &n);
    const EGLint ctxAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };
    // EGL_IMG_context_priority: the GPU scheduler preempts in favour of higher-
    // priority contexts. The async warp present is time-critical, so it gets HIGH;
    // our render/encode work gets LOW so the GPU yields to the warp. Best-effort:
    // the driver may clamp the level, but Adreno honours these.
    #ifndef EGL_CONTEXT_PRIORITY_LEVEL_IMG
    #define EGL_CONTEXT_PRIORITY_LEVEL_IMG 0x3100
    #define EGL_CONTEXT_PRIORITY_HIGH_IMG  0x3101
    #define EGL_CONTEXT_PRIORITY_MEDIUM_IMG 0x3102
    #define EGL_CONTEXT_PRIORITY_LOW_IMG   0x3103
    #endif
    const EGLint ctxAttribsLow[] = { EGL_CONTEXT_CLIENT_VERSION, 3,
        EGL_CONTEXT_PRIORITY_LEVEL_IMG, EGL_CONTEXT_PRIORITY_LOW_IMG, EGL_NONE };
    const EGLint ctxAttribsHigh[] = { EGL_CONTEXT_CLIENT_VERSION, 3,
        EGL_CONTEXT_PRIORITY_LEVEL_IMG, EGL_CONTEXT_PRIORITY_HIGH_IMG, EGL_NONE };
    EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctxAttribs);

    // SDK init + GL resources are created once on a tiny pbuffer so the GL context
    // is valid before the first window surface exists. They survive surface
    // destroy/recreate because the context itself is never destroyed.
    const EGLint pbufAttribs[] = { EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE };
    EGLSurface pbuf = eglCreatePbufferSurface(dpy, cfg, pbufAttribs);
    eglMakeCurrent(dpy, pbuf, pbuf, ctx);

    Pvr_SetInitActivity((void *) gActivity, (void *) gVrClass);
    Pvr_Enable6DofModule(true);                 // enable 6DoF before init/start
    int initRc = Pvr_Init(0);
    int sensRc = InitSensor();                  // select sensor type (else identity pose)
    int startRc = Pvr_StartSensor(0);
    LOGI("Pvr_Enable6DofModule(true); Pvr_Init=%d InitSensor=%d Pvr_StartSensor=%d",
         initRc, sensRc, startRc);
    // Detect Neo 2 EYE support but leave IR illuminators OFF for now. We only light
    // them up once a stream connects with the server's Face Tracking eye source
    // enabled. See applyServerEyeTracking() on STREAMING_STARTED/STOPPED.
    initEyeTrackingMode();
    refreshDeviceIp();   // prime the lobby HUD IP (also re-read periodically in lobby)
    readHeadsetModel(env);
    LOGI("device IP = %s, model = %s", gIpText, gModelText);

    // Pick the tracking origin to mirror the server's tracking space:
    //  - Guardian configured -> StageLevel (2): room-scale, pair with server
    //    recentering = Disabled.
    //  - No guardian -> FloorLevel (1): seated, pair with server's LocalFloor.
    // Either way py includes the floor height, so you never spawn in the floor.
    bool guardian = Pvr_BoundaryGetConfigured();
    int  originType = guardian ? 2 /* StageLevel */ : 1 /* FloorLevel */;
    bool originRc = Pvr_SetTrackingOriginType(originType);
    LOGI("Pvr_SetTrackingOriginType(%s)=%d floorHeight=%.3f guardian=%d",
         guardian ? "StageLevel" : "FloorLevel", originRc, Pvr_GetFloorHeight(),
         guardian);

    // With FloorLevel/StageLevel origin, the Pico SDK reports Y relative to the
    // real-world floor. Mark the tracker floor-relative so it sends raw floor height
    // instead of adding a forced 1.5m standing-height offset.
    if (g_stream)
        g_stream->tracker.floor_relative.store(true);

    // Capture the play-area extents (meters) before tearing the boundary system
    // down, forwarded to SteamVR as the chaperone once a stream starts.
    if (guardian) {
        float bx = 0, by = 0, bz = 0;
        int n = Pvr_BoundaryGetDimensions(&bx, &by, &bz, true /* play area */);
        if (n > 0 && bx > 0.05f && bz > 0.05f) {
            gPlayspaceW = bx; gPlayspaceD = bz;
        }
        LOGI("boundary configured: dims=%.2fx%.2f (n=%d) -> playspace %.2fx%.2f",
             bx, bz, n, gPlayspaceW, gPlayspaceD);
    } else {
        LOGI("boundary not configured; no playspace to forward");
    }

    Pvr_DisableBoundary();
    Pvr_ShutdownSDKBoundary();

    // Render IPD = world-scale knob, user-adjustable in the lobby (gSoftIpdMm,
    // persisted). softIpdM() is the live value used by the lobby render, the warp
    // submit pose, and the ALVR view_params.
    LOGI("render IPD = %.4f m (software, adjustable); device reports %.4f",
         softIpdM(), Pvr_GetIPD());

    // ---- ALVR client_core init -------------------------------------------
    // Context must be the Activity; JavaVM from JNI. After this the client
    // searches for an ALVR PC server on the LAN (mDNS) and connects.
    setHomeFromFilesDir(env, gActivity);   // before any ALVR storage access
    loadAllConfig();                        // restore ALL persisted settings
    // Apply the persisted STREAM FOV to the SDK before the warp thread is created
    // (at the first surface's EV_InitRenderThread), so the warp builds its
    // distortion mesh with the right texture FOV. Runs after Pvr_Init so it isn't
    // clobbered.
    writeSdkFov(gStreamFovDeg.load());
    if (gBrightnessSaved.load()) {
        applyHmdBrightness(gBrightnessFrac.load(), env);   // re-apply the saved level
    } else {
        // No saved value: seed the slider from the panel's current backlight so it
        // reflects reality, and leave the brightness untouched.
        int cur = -1; GetHmdScreenBrightness(cur);
        if (cur >= 0) {
            float fr = (float)(cur - kBrightMin) / (float)(kBrightMax - kBrightMin);
            if (fr < 0.0f) fr = 0.0f; else if (fr > 1.0f) fr = 1.0f;
            gBrightnessFrac.store(fr);
            LOGI("brightness: panel default level=%d -> slider %.2f", cur, fr);
        }
    }
    pushEqGains();                          // apply the restored EQ to the DSP
    LOGI("alvr: calling initialize_logging"); alvr_initialize_logging();
    LOGI("alvr: calling initialize_android_context");
    alvr_initialize_android_context((void *) gVM, (void *) gActivity);
    LOGI("alvr: android_context done");
    // RECORD_AUDIO is a runtime permission on Android 10; without it the mic-to-PC
    // stream stays silent on a fresh install.
    alvr_try_get_permission("android.permission.RECORD_AUDIO");
    // Pico Neo 2 panel scans out at exactly 72Hz (HWComposer vsync = 13888888 ns).
    // There is NO 75Hz mode in the display stack. Advertise only the real native rate.
    const float kRefreshRates[] = { 72.0f };
    AlvrClientCapabilities caps = {};
    // SQUARE per-eye render -> matches the Pico's square eye buffer (1664x1664)
    // and our square per-eye FOV (101deg H == V). A non-square buffer with a
    // square FOV squishes content horizontally. The distortion pipeline assumes a
    // ~square per-eye buffer; reduce decode load via foveated encoding, NOT raw
    // resolution.
    caps.default_view_width  = 1664;
    caps.default_view_height = 1664;
    caps.refresh_rates       = kRefreshRates;
    caps.refresh_rates_count = 1;
    caps.foveated_encoding   = gFoveationEnabled.load();
    caps.encoder_high_profile= true;
    caps.encoder_10_bits     = false;
    caps.encoder_av1         = false;   // SD845 Venus: H.264/HEVC only, no AV1
    caps.prefer_10bit        = false;
    caps.prefer_full_range   = true;
    caps.preferred_encoding_gamma = 1.0f;
    caps.prefer_hdr          = false;
    LOGI("alvr: calling initialize"); alvr_initialize(caps);
    LOGI("alvr: calling resume"); alvr_resume();
    alvrHeadId = alvr_path_string_to_id("/user/head");
    alvrHandId[0] = alvr_path_string_to_id("/user/hand/left");
    alvrHandId[1] = alvr_path_string_to_id("/user/hand/right");
    LOGI("ALVR initialized + resumed; head id=%llu", (unsigned long long) alvrHeadId);
    // Controller input path ids (per hand). Neo 2 CV2 = thumbstick wand: trigger,
    // grip, thumbstick, menu, and two face buttons (A/B right, X/Y left). Path
    // strings MUST match the ALVR v20 server's Paths.cpp or the button is dropped.
    const char *kHand[2] = { "/user/hand/left", "/user/hand/right" };
    for (int h = 0; h < 2; h++) {
        char p[128];
        #define PID(f, sfx) snprintf(p,sizeof(p),"%s%s",kHand[h],sfx); alvrBtn[h].f = alvr_path_string_to_id(p)
        // Paths must match the ALVR server's Quest source profile or the button
        // is "not mapped" and dropped:
        //  - grip: squeeze/VALUE only (value auto-derives click via hysteresis).
        //  - trigger: value only (auto-derives click).
        //  - menu: menu/click (system/click is never read as a source).
        PID(trigVal,    "/input/trigger/value");
        PID(gripVal,    "/input/squeeze/value");
        PID(thumbX,     "/input/thumbstick/x");
        PID(thumbY,     "/input/thumbstick/y");
        PID(thumbClick, "/input/thumbstick/click");
        // These wands have no capacitive stick, so the runtime never gets a touch
        // event. Infer touch from deflection (see the send below).
        PID(thumbTouch, "/input/thumbstick/touch");
        PID(menu,       "/input/menu/click");
        // Face buttons: right = a/b, left = x/y (pick per hand).
        PID(face1, h == 1 ? "/input/a/click" : "/input/x/click");
        PID(face2, h == 1 ? "/input/b/click" : "/input/y/click");
        #undef PID
    }

    // ALVR's GL renderer (wgpu) creates its OWN separate EGL context inside
    // alvr_initialize_opengl(). It is NOT our context, so GL objects are invisible
    // across the boundary. Fix: recreate OUR context as a SHARE context of ALVR's,
    // so texture names are common to both.
    alvr_initialize_opengl();           // creates + makes current ALVR's GL context
    EGLContext alvrCtx = eglGetCurrentContext();
    EGLDisplay alvrDpy = eglGetCurrentDisplay();
    if (alvrCtx == EGL_NO_CONTEXT) {
        LOGE("alvr_initialize_opengl left no current context!");
    } else {
        if (alvrDpy != EGL_NO_DISPLAY && alvrDpy != dpy) {
            LOGI("ALVR uses a different EGLDisplay; adopting it for sharing");
            dpy = alvrDpy;
            pbuf = eglCreatePbufferSurface(dpy, cfg, pbufAttribs);
        }
        EGLContext shared = eglCreateContext(dpy, cfg, alvrCtx, ctxAttribsLow);
        if (shared == EGL_NO_CONTEXT) {
            LOGE("eglCreateContext(shared with ALVR) failed 0x%x", eglGetError());
        } else {
            eglMakeCurrent(dpy, pbuf, pbuf, shared);
            eglDestroyContext(dpy, ctx);   // drop the bootstrap context
            ctx = shared;
            LOGI("created shared GL context %p (shares ALVR ctx %p)", shared, alvrCtx);
        }
    }
    gAlvrGlReady = true;
    LOGI("alvr_initialize_opengl done");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    buildGraphics();
    buildLobbyTarget();      // per-eye ring for the SDK warp
    buildGazeMarker();       // eye-gaze debug disc (Neo 2 EYE only)
    buildTextBuffers();      // dynamic VBOs for lobby HUD text + slider
    buildReticle();          // head-gaze crosshair
    buildControllerMeshes(); // Neo 2 controller wireframes
    // Initialize Android View UI + GL texture + composite shader.
    gAndroidUi.init();
    // Passthrough camera background replaces the dark-void environment.
    // The lobby UI panels composite on top of the live camera feed.
    if (gPassthrough) {
        gPassthrough->init();
        if (gWivrnPassthrough.load())
            gPassthrough->start();
    }

    gSimpleLobby = new simple_lobby();
    gSimpleLobby->init();

    RenderEventFunc re = (RenderEventFunc) GetRenderEventFunc();
    LOGI("GetRenderEventFunc=%p", (void *) re);

    ANativeWindow *curWin = nullptr;            // window this surface was made from
    EGLSurface     sfc    = EGL_NO_SURFACE;
    int  winW = 0, winH = 0;
    bool vrStarted = false;
    bool rtInited  = false;                      // SDK render thread init (tracking pipe)
    EGLContext warpCtx = EGL_NO_CONTEXT;         // persistent: warp thread's ctx (re-pointed on resume)
    int  frame = 0, framesWithSurface = 0;
    // Lobby HUD anchor: WORLD-locked panel. Captured once on lobby (re)entry --
    // planted in front of wherever the user is looking, so it stays put.
    bool hudAnchored = false;
    Mat4 hudWorld = mat4Identity();
    Mat4 settingsWorld = mat4Identity(); // unified SETTINGS window, anchored in front (overlays the info HUD)
    int  lobbyEyeIdx = 0;   // HW-compositor lobby eye-texture ring index
    // Render-on-demand: re-render the lobby eyes only when content changes; the
    // warp reprojects the last frame to live head pose in between. lastRenderedIdx
    // = -1 forces a first render.
    int  lastRenderedIdx = -1;   // slot rendered LAST frame -> warp's submit target this frame
    int  lobbySubmitIdx  = 0;    // slot currently handed to the warp
    // Deferred free of the lobby eye-texture ring after a stream resumes. The warp
    // thread free-runs and is still re-sampling the last lobby submit's textures;
    // deleting them out from under it corrupts the warp ring and crashes. Wait until
    // the warp ring (4 entries) has cycled to fresh stream textures before freeing.
    int  lobbyFreeDelay = -1;

    // Spin up the fixed-rate tracking thread now that the SDK + ALVR are
    // initialized and the path ids are populated.
    gTrackRunning.store(true);
    pthread_create(&gTrackThread, nullptr, trackingThread, nullptr);
    LOGI("tracking thread started (~300Hz fixed-rate uplink)");

    while (gRunning.load()) {
        // Wall-clock at the top of EVERY iteration, used by the video submit path's
        // trailing floor sleep to bound submit-to-submit to >= one vsync.
        uint64_t tLoopStart = nowNs();
        // --- react to surface changes (create / destroy) coming from Java ---
        if (gWindowDirty.load()) {
            ANativeWindow *newWin;
            {
                std::lock_guard<std::mutex> lk(gWinMutex);
                newWin = gPendingWindow;
                gWindowDirty.store(false);
            }
            if (newWin != curWin) {
                // Pause the tracking thread: Pvr_GetMainSensorState_ dereferences
                // the SDK's internal render-thread state which gets nulled during
                // the surface swap. Without this, repeated close/reopen cycles
                // crash with a null-deref in Pvr_GetPredictedDisplayTime_.
                gTrackPaused.store(true);
                // RESUME FIX (HW mode): the warp thread captured this window surface
                // at EV_InitRenderThread and presents to it forever. Before we destroy
                // it, tell the warp to PAUSE so it stops presenting to a dying surface.
                if (gWarpToWindow && re && warpCtx != EGL_NO_CONTEXT) {
                    re(EV_Pause);
                    LOGI("HW compositor: warp paused for surface teardown");
                }
                // tear down old surface
                eglMakeCurrent(dpy, pbuf, pbuf, ctx);
                if (sfc != EGL_NO_SURFACE) { eglDestroySurface(dpy, sfc); sfc = EGL_NO_SURFACE; }
                if (curWin) { ANativeWindow_release(curWin); curWin = nullptr; }

                if (newWin) {
                    curWin = newWin;            // ownership transferred to us
                    sfc = eglCreateWindowSurface(dpy, cfg, curWin, nullptr);
                    if (sfc == EGL_NO_SURFACE) {
                        LOGE("eglCreateWindowSurface failed 0x%x", eglGetError());
                    } else {
                        eglMakeCurrent(dpy, sfc, sfc, ctx);
                        eglQuerySurface(dpy, sfc, EGL_WIDTH, &winW);
                        eglQuerySurface(dpy, sfc, EGL_HEIGHT, &winH);
                        // Reset the surfaced-frame counter on every (re)creation
                        // so a fresh surface re-settles before the logo removal.
                        framesWithSurface = 0;
                        LOGI("window surface ready %dx%d", winW, winH);
                        // First real surface: wire up the SDK render thread so the
                        // tracking data pipe is established (live head pose).
                        // NB(pico): EV_InitRenderThread spins up the SDK render
                        // thread (the DIATW TimeWarp compositor) and establishes the
                        // tracking pipe. The two branches below decide what the warp
                        // presents to: HW path = the real window; self-present path
                        // = a private throwaway pbuffer (parked, so it can't steal
                        // our window) while WE present.
                        if (!rtInited && re) {
                            // The SDK warp thread (TimeWarpLocal::ReadSurface, run
                            // inside InitRenderThread) captures whatever EGL surface
                            // is CURRENT at this instant and presents to it forever.
                            // Give the warp thread its OWN context + pbuffer to
                            // capture, so it never touches our context/window.
                            // Note: these are the PANEL per-eye scanout dims
                            // (3840/2 x 2160 = 1920x2160), NOT our square render
                            // targets. Do NOT change to a square value, this sizes
                            // the SDK's own single-pass depth buffer, which our path
                            // never uses (we submit finished colour textures via
                            // PVR_CameraEndFrame with no depth).
                            Pvr_SetSinglePassDepthBufferWidthHeight(winW / 2, winH);
                            {
                                // HW COMPOSITOR PATH: let the warp thread own the
                                // WINDOW (direct low-latency present) but on its OWN
                                // context that SHARES our (and ALVR's) textures, so
                                // it can sample the eye textures we feed it.
                                warpCtx = eglCreateContext(dpy, cfg, ctx, ctxAttribsHigh);  // SHARE ours, HIGH prio
                                { EGLint pr = -1; eglQueryContext(dpy, warpCtx, EGL_CONTEXT_PRIORITY_LEVEL_IMG, &pr);
                                  LOGI("warpCtx priority level = 0x%x (HIGH=0x%x)", pr, EGL_CONTEXT_PRIORITY_HIGH_IMG); }
                                eglMakeCurrent(dpy, sfc, sfc, warpCtx);
                                re(EV_InitRenderThread);             // warp captures warpCtx + WINDOW
                                eglMakeCurrent(dpy, pbuf, pbuf, ctx);// our ctx -> offscreen pbuffer
                                gWarpToWindow = true;
                                LOGI("HW compositor: warp owns window via shared ctx %p; we render offscreen", warpCtx);
                            }
                            rtInited = true;
                        } else if (rtInited && gWarpToWindow && re && warpCtx != EGL_NO_CONTEXT) {
                            // RESUME RE-POINT (HW mode): the warp thread is alive but
                            // bound to the OLD (destroyed) window surface. Make warpCtx
                            // current on the NEW surface and re-issue EV_InitRenderThread
                            // to re-capture it, then EV_Resume to unpause.
                            eglMakeCurrent(dpy, sfc, sfc, warpCtx);
                            re(EV_InitRenderThread);   // re-capture the new window surface
                            re(EV_Resume);             // unpause the warp
                            eglMakeCurrent(dpy, pbuf, pbuf, ctx);  // our ctx -> offscreen
                            // EV_InitRenderThread resets the SDK's internal ATW state,
                            // so force re-enable on the next submit.
                            gAtwEnabled = false;
                            LOGI("HW compositor: warp RE-POINTED to new surface (resume)");
                        }
                    }
                } else {
                    LOGI("window surface destroyed -> pausing render");
                }
            }
            // Surface swap complete (or surface destroyed): the SDK's internal
            // state is stable again, so the tracking thread can safely resume
            // calling Pvr_GetMainSensorState_.
            gTrackPaused.store(false);
        }

        // --- proximity power-sleep (don/doff) --------------------------------
        // Off-head for the timeout -> pause the connection so the server stops
        // streaming and we tear the decoder down. On don, resume. Edge-triggered.
        {
            bool wantSleep = gSleepReq.load();
            if (wantSleep && !gSlept) {
                LOGI("proximity: off-head timeout -> pausing stream (power save)");
                if (gDecoderReady) { alvr_destroy_decoder(); gDecoderReady = false; }
                gStreaming = false;
                alvr_pause();
                if (gPassthrough) gPassthrough->stop();
                gSlept = true;
                if (g_stream && g_stream->session)
                    g_stream->session->send_control(from_headset::user_presence_changed{
                            .present = false,
                            .change_time = g_stream->to_xr_time(g_stream->get_timestamp_ns())});
            } else if (!wantSleep && gSlept) {
                LOGI("proximity: headset donned -> resuming stream");
                alvr_resume();
                if (gPassthrough) gPassthrough->start();
                gSlept = false;
                if (g_stream && g_stream->session)
                    g_stream->session->send_control(from_headset::user_presence_changed{
                            .present = true,
                            .change_time = g_stream->to_xr_time(g_stream->get_timestamp_ns())});
            }
        }

        // --- recover a lost EGL surface (display sleep/wake or doff/don) ------
        // The Pico blanks the panel off-head, invalidating our EGL surface often
        // WITHOUT a Java surface callback. As long as we still hold the ANativeWindow,
        // just recreate the surface from it. Throttled.
        if (sfc == EGL_NO_SURFACE && curWin != nullptr && (frame % 8) == 0) {
            EGLSurface ns = eglCreateWindowSurface(dpy, cfg, curWin, nullptr);
            if (ns != EGL_NO_SURFACE) {
                sfc = ns;
                eglMakeCurrent(dpy, sfc, sfc, ctx);
                eglQuerySurface(dpy, sfc, EGL_WIDTH, &winW);
                eglQuerySurface(dpy, sfc, EGL_HEIGHT, &winH);
                framesWithSurface = 0;
                LOGI("window surface RECOVERED %dx%d", winW, winH);
            }
        }

        // Once the SDK warp thread is up, pin it to the reserved big core + raise
        // its priority. Retried because the thread may not be named the instant init
        // returns; stop after ~150 frames. Throttled to every 8th frame.
        if (rtInited && !warpPinned && reservedCpu >= 0 && (frame % 8) == 0) {
            int warpCore = pinWarpThreadForLowLatency(reservedCpu);
            if (warpCore >= 0) {
                // The SDK may place the warp on a different big core than our
                // initial guess. Adopt the warp's real core and move THIS (submit)
                // thread off it.
                if (warpCore != reservedCpu) {
                    reservedCpu = warpCore;
                    long lo = 0, hi = 0;
                    cpu_set_t s = bigCoreSetExcept(reservedCpu, lo, hi);
                    if (sched_setaffinity(0, sizeof(s), &s) == 0)
                        LOGI("submit thread re-pinned to big cores [%ld..%ld] minus warp core %d",
                             lo, hi, reservedCpu);
                    else
                        LOGI("submit thread re-pin (minus warp core %d) failed (errno=%d)", reservedCpu, errno);
                }
                warpPinned = true;
            } else if (frame > 150) {
                warpPinned = true;   // give up scanning if the warp never shows / is renamed
            }
        }

        // Hold GPU and CPU pinned while streaming. applyGpuLevel/applyCpuLevel
        // return false until the QVR perf service client binds, so retry each frame.
        if (rtInited) {
            static int cpuApplied = -2, gpuApplied = -2;  // sentinel: != any level (0..5) or -1
            // Only hold the floor while streaming. In the lobby or when doffed/asleep,
            // RELEASE to baseline so the SoC can downclock.
            bool release = (!gStreaming || gSleepReq.load());
            int wantGpu = release ? -1 : desiredPerfLevel(kGpuPerfPin);
            int wantCpu = release ? -1 : desiredPerfLevel(kCpuPerfPin);
            if (wantGpu != gpuApplied && applyGpuLevel(wantGpu)) {
                gpuApplied = wantGpu;
                logActualClocks("gpu");
            }
            if (wantCpu != cpuApplied && applyCpuLevel(wantCpu)) {
                cpuApplied = wantCpu;
                logActualClocks("cpu");
            }

            // Report battery to the server + UI (~1/sec).
            {
                static uint64_t sLastBatt = 0;
                uint64_t nb = nowNs();
                if (nb - sLastBatt > 1000000000ULL) { sLastBatt = nb; sendBatteryReports(); }
            }
            // Push diag HUD data to Java UI (~3/sec, throttled inside).
            sendDiagData();
            // Push running apps to Java UI (~1/sec).
            {
                static uint64_t sLastApps = 0;
                uint64_t na = nowNs();
                if (na - sLastApps > 1000000000ULL && g_stream) {
                    sLastApps = na;
                    std::vector<std::string> names;
                    std::vector<int> ids;
                    std::vector<bool> actives;
                    std::vector<std::string> appIds;
                    std::vector<std::string> appNames;
                    {
                        std::lock_guard<std::mutex> lk(g_stream->app_mutex);
                        for (auto &a : g_stream->running_apps) {
                            names.push_back(a.name);
                            ids.push_back((int)a.id);
                            actives.push_back(a.active);
                        }
                        for (auto &a : g_stream->available_apps) {
                            appIds.push_back(a.id);
                            appNames.push_back(a.name);
                        }
                    }
                    androidUiPushRunningApps(names, ids, actives);
                    if (!appIds.empty())
                        androidUiPushAvailableApps(appIds, appNames);
                }
            }
        }

        // Once a decoder exists, find + pin its output-pump thread. The pump spawns
        // synchronously inside alvr_create_decoder, but pthread_setname lands a beat
        // later, so retry the /proc scan. Re-armed per decoder. Throttled to every 8th
        // frame; ~25 attempts before giving up.
        if (gStreaming && gDecoderReady && !gDecoderPinned && (frame % 8) == 0) {
            if (pinDecoderThreadForLowLatency(reservedCpu) || ++gDecoderPinTries > 25)
                gDecoderPinned = true;
        }

        // Same for the video receive thread (exists once connected, before the decoder).
        // Re-armed on STREAMING_STARTED; scanned on the same throttled cadence.
        if (gStreaming && !gVideoRecvPinned && (frame % 8) == 0) {
            if (pinVideoRecvThreadForLowLatency(reservedCpu) || ++gVideoRecvPinTries > 25)
                gVideoRecvPinned = true;
        }

        // ---- Head pose read-back (produced by the tracking thread) -----------
        // The render thread reads the pose the tracking thread publishes into
        // gHeadData for the lobby head transform + heartbeat log.
        float qx=0,qy=0,qz=0,qw=1, px=0,py=0,pz=0;
        {
            std::lock_guard<std::mutex> lk(gHeadMutex);
            qx=gHeadData[0]; qy=gHeadData[1]; qz=gHeadData[2]; qw=gHeadData[3];
            px=gHeadData[4]; py=gHeadData[5]; pz=gHeadData[6];
        }
        if ((frame % 120) == 0)
            LOGI("frame %d q=(%.3f,%.3f,%.3f,%.3f) p=(%.3f,%.3f,%.3f) vr=%d surf=%d",
                 frame, qx,qy,qz,qw, px,py,pz, vrStarted, (sfc != EGL_NO_SURFACE));

        const uint64_t ts = nowNs();

        // Persist a changed Software IPD once it settles (~0.5s after the last
        // adjustment) so we don't hammer the file during a slider sweep.
        if (gIpdDirty.load() && ts - gIpdChangeNs.load() > 500000000ULL) {
            saveSoftIpd();
            gIpdDirty.store(false);
        }
        // If the IPD changed while streaming, push fresh view_params so the server
        // re-renders at the new eye separation without needing a reconnect.
        if (gStreaming && gSentIpdMm.load() != gSoftIpdMm.load()) {
            sendViewParams();
            if (g_stream) g_stream->tracker.soft_ipd.store(softIpdM());
            LOGI("Software IPD changed mid-stream -> resent view_params (%.1f mm)", gSoftIpdMm.load());
        }

        // ---- FIELD OF VIEW committed (slider released) ----------------------
        // Apply the FOV lever. Writing the SDK globals alone doesn't rebuild the
        // warp's distortion mesh, so re-point the warp (Pause/Init/Resume) to make
        // the change take effect live; then resend view_params so the server renders
        // the new cone at the same FOV the warp now maps. Only fires on slider release.
        if (gFovDirty.exchange(false)) {
            float eff = gStreamFovDeg.load();
            writeSdkFov(eff);
            if (rtInited && gWarpToWindow && re && warpCtx != EGL_NO_CONTEXT && sfc != EGL_NO_SURFACE) {
                re(EV_Pause);
                eglMakeCurrent(dpy, sfc, sfc, warpCtx);
                re(EV_InitRenderThread);   // rebuild the warp mesh at the new FOV
                re(EV_Resume);
                eglMakeCurrent(dpy, pbuf, pbuf, ctx);
                gAtwEnabled = false;       // re-enable ATW on the next submit
                LOGI("FIELD OF VIEW: warp re-pointed at %.1f deg", eff);
            }
            if (gStreaming) sendViewParams();   // server renders the new cone
            saveStreamFov();
            LOGI("FIELD OF VIEW applied: %.1f deg", eff);
        }

        // Low-battery watch: self-throttled to ~1Hz, arms the popup on a 15%/5%
        // crossing. Covers both the stream + lobby paths.
        pollBatteryWarn();

        // ---- ALVR: drain events --------------------------------------------
        AlvrEvent ev;
        while (alvr_poll_event(&ev)) {
            if (ev.tag == ALVR_EVENT_HUD_MESSAGE_UPDATED) {
                char msg[1024] = {0};
                alvr_hud_message_bounded(msg, sizeof(msg));   // never overflows; NUL-terminated within cap
                LOGI("ALVR event HUD_MESSAGE_UPDATED: %s", msg);
                // Map ALVR's HUD message to a short lobby status. Prefix-match
                // the whole word so "Disconnected" doesn't trip "Connect".
                auto startsWith = [](const char *s, const char *pre) {
                    while (*pre) { if (*s++ != *pre++) return false; } return true;
                };
                if (startsWith(msg, "Connect"))     setStrBounded(gStatusText, "Connecting", sizeof(gStatusText));
                else if (startsWith(msg, "Search")) setStrBounded(gStatusText, "Searching", sizeof(gStatusText));
                else if (msg[0] == 0)               setStrBounded(gStatusText, "Disconnected", sizeof(gStatusText));
                // ALVR prints "hostname: XXXX.client", parse it for the lobby HUD.
                const char *hn = strstr(msg, "hostname:");
                if (hn) {
                    hn += 9;
                    while (*hn == ' ' || *hn == '\t') hn++;
                    int i = 0;
                    while (*hn && *hn != '\n' && *hn != '\r' && i < (int)sizeof(gHostnameText)-1)
                        gHostnameText[i++] = *hn++;
                    gHostnameText[i] = 0;
                }
            } else if (ev.tag == ALVR_EVENT_STREAMING_STARTED) {
                gStreamW = ev.STREAMING_STARTED.view_width;
                gStreamH = ev.STREAMING_STARTED.view_height;
                gRefreshHint = ev.STREAMING_STARTED.refresh_rate_hint;
                LOGI("ALVR STREAMING_STARTED %ux%u @%.0fHz fov-enc=%d hdr=%d",
                     gStreamW, gStreamH, ev.STREAMING_STARTED.refresh_rate_hint,
                     ev.STREAMING_STARTED.enable_foveated_encoding,
                     ev.STREAMING_STARTED.enable_hdr);
                // Read the server's foveation params from settings JSON so our
                // de-foveation matches the server. Gated on enable_foveated_encoding.
                bool foveOn = ev.STREAMING_STARTED.enable_foveated_encoding;
                float foCsx=0, foCsy=0, foShx=0, foShy=0, foErx=0, foEry=0;
                if (foveOn) {
                    float fp[6]; readFoveationParams(fp);
                    foCsx=fp[0]; foCsy=fp[1]; foShx=fp[2]; foShy=fp[3]; foErx=fp[4]; foEry=fp[5];
                    LOGI("foveation ON: center=(%.3f,%.3f) shift=(%.3f,%.3f) edge=(%.2f,%.2f)",
                         foCsx, foCsy, foShx, foShy, foErx, foEry);
                }
                // Cache the params so a later mid-session foveation change can be
                // detected + re-synced (see REAL_TIME_CONFIG).
                gFoveOn = foveOn;
                gFovParams[0]=foCsx; gFovParams[1]=foCsy; gFovParams[2]=foShx;
                gFovParams[3]=foShy; gFovParams[4]=foErx; gFovParams[5]=foEry;
                gFovResyncPending = false;   // fresh stream -> drop any stale pending re-sync
                // Defensive: if a reconnect arrives without a STREAMING_STOPPED,
                // drop the stale decoder so the DECODER_CONFIG rebuilds it fresh.
                if (gDecoderReady) { alvr_destroy_decoder(); gDecoderReady = false; }
                // createStreamSwapchain below allocates new textures + resets the
                // pipeline, so no stale slot is handed to the warp on reconnect.
                if (gAlvrGlReady && gStreamW > 0) {
                    // Create the swapchain at EYE dimensions, not the server's stream
                    // dimensions. The PVR warp's distortion mesh is built for the eye
                    // buffer resolution; a larger swapchain leaves black borders.
                    uint32_t swapW = gStreamW, swapH = gStreamH;
                    if (g_stream) {
                        int ew = g_stream->eye_width.load();
                        int eh = g_stream->eye_height.load();
                        if (ew > 0 && eh > 0) { swapW = ew; swapH = eh; }
                    }
                    createStreamSwapchain(swapW, swapH);
                    const uint32_t *swapArr[2] = { gSwap[0], gSwap[1] };
                    AlvrStreamConfig sc = {};
                    sc.view_resolution_width  = swapW;
                    sc.view_resolution_height = swapH;
                    sc.swapchain_textures = (const uint32_t **) swapArr;
                    sc.swapchain_length   = kSwapLen;
                    sc.enable_foveation        = foveOn;
                    sc.foveation_center_size_x = foCsx;
                    sc.foveation_center_size_y = foCsy;
                    sc.foveation_center_shift_x= foShx;
                    sc.foveation_center_shift_y= foShy;
                    sc.foveation_edge_ratio_x  = foErx;
                    sc.foveation_edge_ratio_y  = foEry;
                    sc.enable_upscaling   = false;
                    alvr_start_stream_opengl(sc);
                    // Tell the server our per-eye FOV (~101deg) + current Software IPD.
                    sendViewParams();
                    gSwapIdx = 0;
                    gStreaming = true;
                    // Sync the tracker's soft_ipd from gSoftIpdMm so the server and
                    // PVR warp use the same IPD (avoids stereo mismatch).
                    if (g_stream) g_stream->tracker.soft_ipd.store(softIpdM());
                    // Stream owns the eye buffers now, stop the passthrough camera.
                    if (gPassthrough) gPassthrough->stop();
                    gResetPacer = true;          // fresh stream -> reset pacer + video counters
                    gVideoRecvPinned = false; gVideoRecvPinTries = 0;   // re-arm recv-thread pin
                    gManualLobby.store(false);   // start in the stream, not the manual lobby
                    // Announce a custom interaction profile per hand with the exact
                    // button set we drive. Includes the RIGHT hand's menu/click,
                    // which the stock Quest profile lacks, the binder maps right
                    // menu -> right SYSTEM (SteamVR dashboard).
                    // NOTE: a custom profile rebuilds the GLOBAL button mapping
                    // manager from exactly the ids it carries, so it must be sent
                    // ONCE with BOTH hands' buttons, sending per-hand makes the
                    // second call wipe the first hand's mappings.
                    {
                        uint64_t ids[16] = {
                            alvrBtn[0].trigVal, alvrBtn[0].gripVal,
                            alvrBtn[0].thumbX,  alvrBtn[0].thumbY, alvrBtn[0].thumbClick,
                            alvrBtn[0].menu,    alvrBtn[0].face1,  alvrBtn[0].face2,
                            alvrBtn[1].trigVal, alvrBtn[1].gripVal,
                            alvrBtn[1].thumbX,  alvrBtn[1].thumbY, alvrBtn[1].thumbClick,
                            alvrBtn[1].menu,    alvrBtn[1].face1,  alvrBtn[1].face2,
                        };
                        alvr_send_custom_interaction_profile(alvrHandId[0], ids, 16);
                    }
                    LOGI("sent custom interaction profile (both hands, incl. right menu->system)");
                    setStrBounded(gStatusText, "Connected", sizeof(gStatusText));
                    LOGI("stream renderer ready (%ux%u)", gStreamW, gStreamH);
                    // Settings JSON is now live: light up the EYE illuminators only if
                    // the server's Face Tracking eye source is on.
                    applyServerEyeTracking(true);
                    // Push the eye-foveation preference once the stream is up so
                    // the server picks gaze-tracked or fixed-center foveation
                    // from the start (no extra round-trip after first frame).
                    if (g_stream) g_stream->send_eye_foveation_override();
                    // Forward the Pico play area to SteamVR as the chaperone.
                    // Only if a guardian was set up.
                    if (gPlayspaceW > 0.0f && gPlayspaceD > 0.0f) {
                        alvr_send_playspace(gPlayspaceW, gPlayspaceD);
                        LOGI("sent playspace %.2fx%.2f m", gPlayspaceW, gPlayspaceD);
                    }
                }
            } else if (ev.tag == ALVR_EVENT_DECODER_CONFIG) {
                if (!gDecoderReady) {
                    // Cache the config (codec + NAL) so we can rebuild the decoder
                    // after the manual-lobby pause without a reconnect.
                    // Bounded copy: alvr_get_decoder_config_bounded never writes past
                    // sizeof(gDecCfg) and returns the FULL length. An oversized NAL
                    // is corrupt, REJECT it (don't feed a truncated SPS/PPS to MediaCodec).
                    uint64_t n = alvr_get_decoder_config_bounded(
                            (char *) gDecCfg, sizeof(gDecCfg));
                    if (n > sizeof(gDecCfg)) {
                        LOGE("decoder config %llu B exceeds %zu B buffer -- rejecting",
                             (unsigned long long) n, sizeof(gDecCfg));
                    } else {
                        gDecCodec = ev.DECODER_CONFIG.codec;
                        gDecCfgLen = n;
                        gHaveDecCfg = true;
                        createVideoDecoder();
                    }
                }
            } else if (ev.tag == ALVR_EVENT_STREAMING_STOPPED) {
                // Tear the stream down fully so a reconnect rebuilds the decoder +
                // swapchain from the new config. Without this, the stale decoder
                // (old SPS/PPS, maybe old resolution) decodes the new stream -> garbled.
                gStreaming = false;
                gManualLobby.store(false);   // back to the normal disconnected lobby
                androidUiPushDiagOverlayOnly(false);  // diag overlay off when not streaming
                gHaveDecCfg = false;         // stale config; next stream sends a fresh one
                setStrBounded(gStatusText, "Disconnected", sizeof(gStatusText));
                if (gDecoderReady) { alvr_destroy_decoder(); gDecoderReady = false; }
                destroyStreamSwapchain();   // also resets the pipeline (no stale slot)
                gFovResyncPending = false;   // drop any pending re-sync for the dead stream
                applyServerEyeTracking(false);       // stream gone -> turn the IR off
                // Back in the lobby, restart the passthrough camera.
                if (gPassthrough && gWivrnPassthrough.load()) gPassthrough->start();
                LOGI("ALVR STREAMING_STOPPED -> decoder+swapchain torn down");
            } else if (ev.tag == ALVR_EVENT_REAL_TIME_CONFIG) {
                // The server changed a live setting mid-session. Check for a
                // foveation change: our de-foveation params are baked at
                // StreamingStarted, so if the server re-foveated without a full
                // restart the eyes misalign (stale FFR map). Re-read the params and
                // rebuild the de-foveation pipeline in place if they moved.
                if (gStreaming && gAlvrGlReady && gFoveOn && gStreamW > 0) {
                    float np[6]; readFoveationParams(np);
                    bool changedVsApplied = false;
                    for (int i = 0; i < 6; i++)
                        if (fabsf(np[i] - gFovParams[i]) > 1e-4f) changedVsApplied = true;
                    if (changedVsApplied) {
                        // DEFER the rebuild, stash the latest params + reset the
                        // settle timer. The loop applies one rebuild once these stop
                        // changing for kFovDebounceNs, so a slider drag's burst of
                        // events = a single swapchain realloc.
                        memcpy(gFovPending, np, sizeof(gFovPending));
                        gFovPendingNs = nowNs();
                        gFovResyncPending = true;
                        LOGI("REAL_TIME_CONFIG: foveation change pending center=(%.3f,%.3f) "
                             "shift=(%.3f,%.3f) edge=(%.2f,%.2f) -> debounced %llums",
                             np[0],np[1],np[2],np[3],np[4],np[5],
                             (unsigned long long)(kFovDebounceNs/1000000ULL));
                    } else {
                        // Matches what's applied -> cancel any pending rebuild.
                        gFovResyncPending = false;
                        LOGI("REAL_TIME_CONFIG: no foveation change");
                    }
                } else {
                    LOGI("ALVR event REAL_TIME_CONFIG (not streaming / no foveation)");
                }
            } else if (ev.tag == ALVR_EVENT_HAPTICS) {
                // Server-driven controller rumble. Map device_id -> hand and park
                // the pulse; the Java ControllerClient poller drains it.
                uint64_t did = ev.HAPTICS.device_id;
                int hand = (did == alvrHandId[1]) ? 1 : (did == alvrHandId[0]) ? 0 : -1;
                if (hand >= 0)
                    queueHaptic(hand, ev.HAPTICS.amplitude, ev.HAPTICS.frequency,
                                ev.HAPTICS.duration_s);
            } else {
                LOGI("ALVR event %s", alvrEventName(ev.tag));
            }
        }

        // Apply a debounced foveation re-sync once the pending params have settled
        // (no further REAL_TIME_CONFIG change for kFovDebounceNs). Coalesces a slider
        // drag's burst of events into a single swapchain rebuild. Runs here, right
        // after the event drain, before the per-frame GL work, so our context is
        // current.
        if (gFovResyncPending && gStreaming && gAlvrGlReady && gFoveOn && gStreamW > 0 &&
            nowNs() - gFovPendingNs > kFovDebounceNs) {
            applyFoveationResync(gFovPending);
            gFovResyncPending = false;
        }

        // Handle resolution change from the settings slider: recreate the
        // swapchain at the new eye dimensions and restart the stream renderer.
        if (g_stream && g_stream->resolution_dirty.exchange(false) &&
            gStreaming && gAlvrGlReady && gStreamW > 0) {
            int ew = g_stream->eye_width.load();
            int eh = g_stream->eye_height.load();
            if (ew > 0 && eh > 0) {
                createStreamSwapchain(ew, eh);
                const uint32_t *swapArr[2] = { gSwap[0], gSwap[1] };
                AlvrStreamConfig sc = {};
                sc.view_resolution_width  = ew;
                sc.view_resolution_height = eh;
                sc.swapchain_textures = (const uint32_t **) swapArr;
                sc.swapchain_length   = kSwapLen;
                sc.enable_foveation        = gFoveOn;
                sc.foveation_center_size_x = gFovParams[0];
                sc.foveation_center_size_y = gFovParams[1];
                sc.foveation_center_shift_x= gFovParams[2];
                sc.foveation_center_shift_y= gFovParams[3];
                sc.foveation_edge_ratio_x  = gFovParams[4];
                sc.foveation_edge_ratio_y  = gFovParams[5];
                sc.enable_upscaling   = false;
                alvr_start_stream_opengl(sc);
                gSwapIdx = 0;
                gPrevSwapIdx = -1; gPrevSwapValid = false;
                sendViewParams();
                LOGI("resolution change: swapchain recreated at %dx%d", ew, eh);
            }
        }

        // No surface (panel asleep / doffed): nothing to draw. Idle at ~20Hz
        // instead of busy-spinning to save power.
        if (sfc == EGL_NO_SURFACE) { frame++; usleep(50000); continue; }

        // Remove the platform logo a few frames after we first have a surface.
        // NB(pico): we do NOT call the Java startVRModel() entry point here --
        // that lets DIATW grab the panel without our shared-context handoff.
        // Our compositor is started above via EV_InitRenderThread with a shared
        // warp context.
        // Use >= (not ==) so the one-shot removal still fires if framesWithSurface
        // skips the exact value 3. The !vrStarted guard makes it fire exactly once.
        if (!vrStarted && framesWithSurface >= 3) {
            callVrStatic(env, "removePlatformLogo");
            LOGI("VR mode: warp compositor active (startVRModel not used)");
            vrStarted = true;
        }

        const int halfW = winW / 2;

        // ---- Toggle the lobby overlay mid-stream (stream stays alive) --------
        // Three gestures flip gManualLobby (render the lobby WITHOUT tearing down
        // the stream, connection/decoder stay alive):
        //   (a) hold the headset SIDE button (keycode 1001) for 2s, or
        //   (b) DOUBLE-TAP the RIGHT controller app/menu button, or
        //   (c) click BOTH thumbsticks simultaneously.
        // Only active during streaming so stale controller data can't open the lobby.
        auto toggleManualLobby = [&](const char *why) {
            bool nowLobby = !gManualLobby.load();
            gManualLobby.store(nowLobby);
            if (nowLobby) {
                hudAnchored = false;
                // Reposition the lobby panel in front of where the user is facing
                // so the overlay opens at a natural viewing angle.
                if (gLobby) {
                    Mat4 hr = quatToMat4(qx, qy, qz, qw);
                    float fx = -hr.m[8], fz = -hr.m[10];
                    float hp[3] = {px, py, pz};
                    gLobby->recenter_facing(hp, fx, fz);
                }
            }
            // Notify the server that the user is interacting with the in-stream
            // lobby overlay. When open, session drops to VISIBLE so the server
            // and SteamVR overlays know the user is in a menu; when closed,
            // restore FOCUSED + hidden tab.
            if (g_stream && g_stream->session) {
                try {
                    g_stream->session->send_control(
                        from_headset::session_state_changed{
                            .state = nowLobby ? XR_SESSION_STATE_VISIBLE
                                              : XR_SESSION_STATE_FOCUSED,
                        });
                    g_stream->session->send_control(
                        from_headset::stream_tab_changed{
                            .tab = nowLobby ? stream_tab::applications
                                            : stream_tab::hidden,
                        });
                } catch (std::exception & e) {
                    LOGI("toggleManualLobby: failed to notify server: %s", e.what());
                }
            }
            gOkClick.store(false);               // swallow any pending click
            LOGI("%s -> manual lobby = %d (stream stays alive)", why, (int)nowLobby);
        };
        const bool canToggle = gStreaming && gDecoderReady;
        {
            static uint64_t sideHoldStart = 0;
            if (canToggle && gSideHeld.load() && !gEqGrabbing) {
                if (sideHoldStart == 0) sideHoldStart = nowNs();
                else if (nowNs() - sideHoldStart > 2000000000ULL) {   // 2s hold
                    toggleManualLobby("side button 2s hold");
                    sideHoldStart = 0;                                 // consume this hold
                }
            } else {
                sideHoldStart = 0;
            }
        }
        // Right controller app/menu double-tap (two rising edges within 400ms).
        {
            static bool menuPrev = false; static uint64_t lastTapNs = 0;
            bool menuNow = false;
            { std::lock_guard<std::mutex> lk(gCtrlMutex);
              menuNow = (gCtrl[1].conn==1 && gCtrl[1].keyCount>5 && gCtrl[1].keys[5]!=0); }
            if (menuNow && !menuPrev) {           // rising edge
                uint64_t now = nowNs();
                if (canToggle && lastTapNs != 0 && now - lastTapNs < 400000000ULL) {
                    toggleManualLobby("right menu double-tap");
                    lastTapNs = 0;
                } else {
                    lastTapNs = now;
                }
            }
            menuPrev = menuNow;
        }
        // Both thumbsticks clicked simultaneously.
        {
            static bool prev_stick[2] = {false, false};
            bool stick[2] = {false, false};
            { std::lock_guard<std::mutex> lk(gCtrlMutex);
              for (int h = 0; h < 2; h++)
                  stick[h] = (gCtrl[h].conn==1 && gCtrl[h].keyCount>4 && gCtrl[h].keys[4]!=0); }
            bool both_now = stick[0] && stick[1];
            bool both_prev = prev_stick[0] && prev_stick[1];
            if (canToggle && both_now && !both_prev)
                toggleManualLobby("both thumbsticks clicked");
            prev_stick[0] = stick[0]; prev_stick[1] = stick[1];
        }

        // ---- Manual-lobby overlay: video keeps playing, UI draws on top ------
        // The overlay keeps the decoder running and renders the lobby UI on top
        // of the live video. No pause, no drain, no IDR request on exit.

        // ---- ALVR video path: async TimeWarp (present every refresh) --------
        // The overlay (gManualLobby) draws on top of the video below.
        if (gStreaming && gDecoderReady) {
            gStreamingMode.store(true);
            // Servers tab is hidden while streaming; fall back to Settings.
            if (gSettingsCat == 0) gSettingsCat = 1;
            // Re-anchor the lobby HUD next time we return to the non-streaming
            // lobby. Don't reset while the manual lobby overlay is open, or the
            // panel re-anchors in front of the head every frame and follows the
            // HMD instead of staying world-locked.
            if (!gManualLobby.load())
                hudAnchored = false;
            // Free the lobby eye-texture ring (~94 MB) while streaming; rebuilt
            // lazily when we next return to the lobby. DEFER the free, arm a
            // countdown, then free only once the warp ring has been refilled with
            // stream textures, so we never glDeleteTextures slots the free-running
            // warp is still sampling. Re-entering the lobby cancels a pending free.
            if (gLobbyEyeReady && lobbyFreeDelay < 0) {
                lobbyFreeDelay = kSwapLen + 2;   // wait out the 4-entry warp ring + margin
            }
            if (lobbyFreeDelay == 0) {
                eglMakeCurrent(dpy, pbuf, pbuf, ctx);
                freeLobbyTarget();
                lastRenderedIdx = -1; lobbyEyeIdx = 0;   // force a fresh render on rebuild
                lobbyFreeDelay = -1;
            } else if (lobbyFreeDelay > 0) {
                lobbyFreeDelay--;
            }
            // (0/1) ADAPTIVE PHASE-LOCK + UPDATE SOURCE (HW path).
            // The warp free-runs reprojection every vsync; the beat comes from
            // the AGE of the video frame we submit jittering as the PC's frame
            // arrival drifts past a fixed capture gate, when drift brings
            // arrival onto the gate, intervals alternate 0/2 new frames =
            // drop/double = stutter. We TRACK where frames actually arrive (EMA
            // of the vsync phase at arrival) and place the submit gate a fixed
            // margin after it. The gate chases the drift, so it never sits on
            // the arrival boundary.
            uint64_t ts = 0; void *hwbuf = nullptr;
            uint64_t fts = 0; void *fbuf = nullptr; int drained = 0;
            // BLOCK on the decoder for the first frame instead of busy-polling.
            // alvr_get_frame_timeout sleeps until a freshly decoded frame arrives
            // (or the cap elapses), so we wake ~once per frame instead of spinning.
            // Cap = 1.5 frame intervals: under healthy streaming a frame always
            // lands within 1, so we wake on the frame; the cap only bounds how
            // often top-of-loop housekeeping runs when the stream stalls.
            // Once the first frame is in hand, drain the rest non-blocking to coalesce.
            //
            // BUFFER RELEASE: coalescing here does NOT leak or starve the decoder.
            // Each alvr_get_frame pop_front()s the PREVIOUS call's image first (Rust's
            // Image::Drop returns that AHardwareBuffer to the ImageReader) then hands
            // back the next. So every intermediate frame in this drain loop is released
            // by the very next call; pop_front() IS the release. The final coalesced
            // hwbuf stays in_use until the NEXT iteration's first dequeue, by which
            // point alvr_render_stream_opengl below has already consumed it.
            uint64_t blockNs = (uint64_t)(1.5e9f / (gRefreshHint > 1.0f ? gRefreshHint : 72.0f));
            // ORDERING ASSUMPTION (B3): each alvr_get_frame releases the PREVIOUS
            // call's image back to the decoder immediately, but the GPU read of that
            // image is only fenced, not waited. Safe because the Venus decoder takes
            // >= one frame to recycle+refill a released buffer, longer than our
            // render takes to drain its GPU work. If this drain is ever reworked to
            // hold multiple images or the render overruns a frame, add an explicit
            // fence wait before release.
            if (alvr_get_frame_timeout(&fts, &fbuf, blockNs)) {
                ts = fts; hwbuf = fbuf; drained++;
                while (alvr_get_frame(&fts, &fbuf)) { ts = fts; hwbuf = fbuf; drained++; }
            }

            // SUBMIT EVERY DECODED FRAME. The warp thread free-runs reprojection
            // to the live predicted pose every vsync, so we just need to keep the
            // latest decoded frame in front of it. Do NOT add an adaptive vsync
            // submit gate: it submits at most once per vsync, so a frame landing
            // after the gate is overwritten before submit for no benefit.
            bool doSubmit = (drained > 0);

            // ADAPTIVE FRAME PACING (anti-judder). At 72Hz this only observes
            // cadence to decide when to back off, it never sleeps and never
            // re-drains, so a healthy stream submits exactly as before with no
            // added latency. Once the device can't keep up, it presents at an
            // irregular 55-67 fps that beats the 72Hz scanout, causing violent
            // shake. Dropping CLEANLY to an evenly-spaced lower rate (every
            // 2nd/3rd vsync) is lower fps but smooth. sPaceDiv moves with
            // asymmetric hysteresis (back off fast, speed up only after a long
            // clean spell) so it settles instead of oscillating.
            static int      sPaceDiv = 1;          // 1=72Hz, 2=36Hz, 3=24Hz
            static int64_t  sLastSubmitVs = 0;
            static int      sWinMiss = 0;
            static uint64_t sWinT0 = 0, sLastMissNs = 0;
            // Fresh stream: clear stale pacer state (don't clear gResetPacer yet --
            // the counters block below consumes it). Otherwise a stream that ended
            // at 24Hz starts the next one throttled.
            if (gResetPacer) {
                sPaceDiv = 1; sLastSubmitVs = 0; sWinMiss = 0; sWinT0 = 0; sLastMissNs = 0;
            }
            if (doSubmit) {
                const double interval = 1e9 / 72.0;         // panel vsync period (ns)
                int64_t cur = (int64_t) floor(PVR::GetFractionalVsync());
                if (sPaceDiv > 1) {
                    // Reduced rate: phase-lock the present to an even sPaceDiv-vsync beat.
                    int64_t target = sLastSubmitVs + sPaceDiv;
                    if (cur < target) {
                        double waitVs = (double) target - PVR::GetFractionalVsync();
                        if (waitVs > 0.0 && waitVs < 4.0)   // guard a bogus oracle read
                            // Absolute-deadline sleep (no usleep oversleep drift): the
                            // wait duration in ns from now to the target vsync beat.
                            sleepUntilMonoNs(nowNs() + (uint64_t)(waitVs * interval));
                        cur = (int64_t) floor(PVR::GetFractionalVsync());
                    }
                    // Grab any fresher frame that arrived during the wait, so pacing
                    // only regularises WHEN we present, no extra latency.
                    while (alvr_get_frame(&fts, &fbuf)) { ts = fts; hwbuf = fbuf; drained++; }
                }
                // A miss = this submit ran past the current rate's vsync budget AND
                // the decoder had a backlog (drained>1 => our fault, not a slow
                // source: a late submit with one frame in hand is sub-72 server fps
                // or a network gap, which must not drop the rate).
                int64_t spacing = (sLastSubmitVs != 0) ? (cur - sLastSubmitVs) : sPaceDiv;
                sLastSubmitVs = cur;
                bool ourMiss = (spacing - sPaceDiv > 0 && drained > 1);

                // Act on the FREQUENCY of misses, never on a single one. A lone
                // stall (keyframe-decode or GC spike) must NOT step the rate down:
                // the frame already hitched, so dropping the rate can't un-hitch it
                // and only pumps 72<->24. Only a SUSTAINED beat of misses in a short
                // window backs off.
                uint64_t pnow = nowNs();
                if (ourMiss) { sWinMiss++; sLastMissNs = pnow; }
                if (sWinT0 == 0) sWinT0 = pnow;
                if (pnow - sWinT0 >= 250000000ULL) {        // short window -> quick onset
                    if (sWinMiss >= 2 && sPaceDiv < 3) {
                        sPaceDiv++;
                        LOGI("pace: shake guard -> %dHz (%d misses/0.25s)", 72 / sPaceDiv, sWinMiss);
                    } else if (sPaceDiv > 1 && pnow - sLastMissNs >= 3000000000ULL) {
                        // Step back up only after a sustained clean spell (~3s),
                        // one level at a time.
                        sPaceDiv--;
                        LOGI("pace: recovered -> %dHz", 72 / sPaceDiv);
                    }
                    sWinMiss = 0; sWinT0 = pnow;
                }
            }
            // STUTTER DIAGNOSTIC: per-stage timing, per-second max (ms). gap =
            // submit-to-submit period; render/enc = our GPU-issue cost; enq =
            // the cost of handing the frame to the SDK warp (incl. vsync
            // backpressure, NOT the warp's own compute). These probes feed ONLY
            // the diag HUD's pipeline page; with no HUD shown there's no
            // consumer, so skip the clock reads on the shipping path. The
            // per-second VIDEO / VIDEO-LATENCY logs stay ALWAYS.
            bool diagTiming = gDiagHudMode.load() != 0;
            static uint64_t _lastStart=0, _mGap=0, _mRender=0, _mEnc=0, _mEnq=0;
            uint64_t _tStart = diagTiming ? nowNs() : 0, _tRender = _tStart, _tEnc = _tStart;
            if (doSubmit) {
                if (diagTiming) {
                    if (_lastStart) { uint64_t g = _tStart - _lastStart; if (g > _mGap) _mGap = g; }
                    _lastStart = _tStart;
                }
                AlvrViewParams outVP[2] = {};
                // Use the current sensor pose as a placeholder for
                // alvr_report_compositor_start. The actual render pose (from the
                // server) is read AFTER the blit and stored into gSwapVP.
                for (int e = 0; e < 2; e++) {
                    outVP[e].pose.orientation = { qx, qy, qz, qw };
                    outVP[e].pose.position[0] = px;
                    outVP[e].pose.position[1] = py;
                    outVP[e].pose.position[2] = pz;
                }
                alvr_report_compositor_start(ts, outVP);

                if (!gAtwEnabled) { Pvr_SetAsyncTimeWarp(1); gAtwEnabled = true;
                    LOGI("HW compositor: async TimeWarp enabled; cfg8(asyncMode)=%d cfg0x19=%d",
                         cfgI(8), cfgI(0x19)); }

                // Hand one ring slot to the SDK warp thread: wait its (frame-old)
                // fence so it's GPU-complete, set both eyes' textures + the render
                // pose they were drawn at, then enqueue. The warp reprojects that
                // baseline to the live predicted pose every vsync.
                static int sFenceTimeouts = 0;   // per-second tally (reset below), render-thread-only
                auto submitSlot = [&](int p, uint64_t fenceWaitNs) {
                    if (gSwapFence[p]) {
                        GLenum w = glClientWaitSync(gSwapFence[p], GL_SYNC_FLUSH_COMMANDS_BIT, fenceWaitNs);
                        if (w == GL_TIMEOUT_EXPIRED) sFenceTimeouts++;
                    }
                    PVR_CameraEndFrame(0, gSwap[0][p]);
                    PVR_CameraEndFrame(1, gSwap[1][p]);
                    if (gSwapFrameTs[p]) {
                        uint64_t fi = (gSwapFrameTs[p] + 5000) / 10000;
                        g_latency.on_frame_submitted(fi, 0, nowNs());
                    }
                    // Render-pose baseline for BOTH eyes. Normalize the quat; reuse
                    // last good if degenerate (the warp's SelectRT rejects non-unit
                    // quats). Position is the per-eye IPD offset, NOT the server
                    // head position, the PVR warp only does rotational time warp,
                    // so feeding it a moving head position causes stutter.
                    static Quat sLastGoodQ[2] = { {0,0,0,1}, {0,0,0,1} };
                    float ipd = softIpdM();
                    for (int e = 0; e < 2; e++) {
                        Quat q = { gSwapVP[p][e].pose.orientation.x, gSwapVP[p][e].pose.orientation.y,
                                   gSwapVP[p][e].pose.orientation.z, gSwapVP[p][e].pose.orientation.w };
                        float n2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
                        if (n2 > 1e-6f) { q = quatNorm(q); sLastGoodQ[e] = q; }
                        else            { q = sLastGoodQ[e]; }
                        float eye_offset = (e == 0 ? -ipd * 0.5f : ipd * 0.5f);
                        PvrPoseBlk blk; memset(&blk, 0, sizeof(blk));
                        blk.v[0] = q.x; blk.v[1] = q.y; blk.v[2] = q.z; blk.v[3] = q.w;
                        blk.v[4] = eye_offset;
                        blk.v[5] = 0;
                        blk.v[6] = 0;
                        PVR_ChangeRenderPose(e, 0, blk);
                    }
                    PVR_TimeWarpEvent(0);
                };

                // SUBMIT THE CURRENT frame right after rendering it, not the
                // previous frame's slot. This eliminates the +1 frame of
                // world-content latency (~14ms at 72Hz) the old pipelined
                // approach added. Trade-off: we must glClientWaitSync on the
                // current frame's fence instead of a frame-old fence that's
                // already signalled. Under healthy streaming the blit is fast
                // (<5ms GPU) so the wait returns well within budget.
                bool firstFrame = !gPrevSwapValid;
                uint64_t _tEnqStart = diagTiming ? nowNs() : 0;
                if (diagTiming) { uint64_t e = nowNs(); if (e - _tEnqStart > _mEnq) _mEnq = e - _tEnqStart; }

                AlvrStreamViewParams svp[2];
                for (int e = 0; e < 2; e++) {
                    svp[e].swapchain_index = (uint32_t) gSwapIdx;
                    svp[e].reprojection_rotation = { 0, 0, 0, 1 };
                    svp[e].fov = outVP[e].fov;
                }
                uint64_t _tRenderStart = diagTiming ? nowNs() : 0;
                alvr_render_stream_opengl(hwbuf, svp);   // -> gSwap[e][gSwapIdx] (leaves wgpu ctx current)
                if (diagTiming) { _tRender = nowNs(); if (_tRender - _tRenderStart > _mRender) _mRender = _tRender - _tRenderStart; }

                {
                    // The video is rendered in ALVR's wgpu context. Flush to kick
                    // off the GPU work, then fence. The submitSlot below does
                    // glClientWaitSync on the fence, which is the actual GPU
                    // completion wait. Using glFlush + fence instead of glFinish
                    // lets the CPU continue (read server poses, set up overlay)
                    // while the GPU blit runs in parallel.
                    glFlush();
                    GLsync alvrFence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
                    // wgpu made its ctx current; restore ours (offscreen pbuffer; the
                    // warp owns the window). Textures are shared across the group.
                    eglMakeCurrent(dpy, pbuf, pbuf, ctx);
                    // PicoNeo2 fork: NO encode blit. The forked stream shader
                    // already wrote the final present-domain bytes directly into
                    // gSwap[e][gSwapIdx], so we feed gSwap straight to the warp.
                    // Diag HUD is now rendered by the Android UI (WivrnLobbyView).
                    // Low-battery popup active window (5s after a 15%/5% crossing).
                    // Draws into gSwap on OUR ctx, shares the FBO-bind / fence handling.
                    bool warnActive = gBattWarnStartNs.load() != 0 &&
                                      (nowNs() - gBattWarnStartNs.load()) < kBattWarnDurNs;
                    if (warnActive) {
                        if (alvrFence) { glWaitSync(alvrFence, 0, GL_TIMEOUT_IGNORED); glDeleteSync(alvrFence); alvrFence = 0; }
                        glBindFramebuffer(GL_FRAMEBUFFER, gStreamFbo);
                        glDisable(GL_DEPTH_TEST); glDisable(GL_CULL_FACE); glDisable(GL_SCISSOR_TEST);
                        {
                            int ew = g_stream ? g_stream->eye_width.load() : 0;
                            int eh = g_stream ? g_stream->eye_height.load() : 0;
                            glViewport(0, 0, (GLsizei)(ew > 0 ? ew : gStreamW),
                                              (GLsizei)(eh > 0 ? eh : gStreamH));
                        }
                        // Low-battery popup, per-eye.
                        for (int e = 0; e < 2; e++) {
                            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                                   GL_TEXTURE_2D, gSwap[e][gSwapIdx], 0);
                            drawBatteryWarn(e);
                        }
                        glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    }
                    // ---- Stream overlay: draw lobby UI on top of the live video ----
                    // When gManualLobby is toggled mid-stream, composite the lobby
                    // UI directly on top of the video in gSwap (overlay mode: no
                    // color clear, only depth). The video keeps playing underneath.
                    if (gManualLobby.load()) {
                        gStreamingMode.store(true);   // show streaming tabs in sidebar
                        // The PVR warp only does rotational time warp: it reprojects
                        // the frame from the server's render orientation (gSwapVP) to
                        // the live display orientation. The overlay must be drawn at
                        // the SERVER's orientation so the warp's reprojection keeps it
                        // world-locked. Drawing it at the current head orientation
                        // causes the warp to shift it every frame -> stutter.
                        // Position uses the current head pose since the warp doesn't
                        // reproject position. Anchoring still uses the current head
                        // orientation to place the panel in front of the user.
                        XrPosef srvPoses[2];
                        bool haveSrv = wivrn_get_server_pose(srvPoses);
                        float vqx = qx, vqy = qy, vqz = qz, vqw = qw;
                        if (haveSrv) {
                            vqx = srvPoses[0].orientation.x;
                            vqy = srvPoses[0].orientation.y;
                            vqz = srvPoses[0].orientation.z;
                            vqw = srvPoses[0].orientation.w;
                        }
                        Mat4 hRot = quatToMat4(qx, qy, qz, qw);  // current head orient for anchoring
                        Mat4 invRot = mat4Transpose3x3(quatToMat4(vqx, vqy, vqz, vqw));  // server orient for view
                        Mat4 viewBase = mat4Mul(invRot, mat4Translate(-px, -py, -pz));
                        float lobbyFovDeg = (fEyeTextureFov0 > 1.0f) ? fEyeTextureFov0 : 101.0f;
                        float fovy = lobbyFovDeg * (float)M_PI / 180.0f;
                        Mat4 sproj = mat4Perspective(fovy, 1.0f, 0.05f, 250.0f);

                        // Anchor the panel in front of the user (same logic as the
                        // lobby path). hudAnchored is reset when the lobby is toggled.
                        if (!hudAnchored) {
                            float fx = -hRot.m[8], fz = -hRot.m[10];
                            float fn = sqrtf(fx*fx + fz*fz);
                            if (fn > 1e-5f) { fx /= fn; fz /= fn; } else { fx = 0; fz = -1; }
                            const float kHudDist = 2.0f;
                            float ax = px + fx * kHudDist, ay = py, az = pz + fz * kHudDist;
                            float cphi = -fz, sphi = -fx;
                            Mat4 m = mat4Identity();
                            m.m[0] = cphi; m.m[2] = -sphi;
                            m.m[8] = sphi; m.m[10] = cphi;
                            m.m[12] = ax;  m.m[13] = ay; m.m[14] = az;
                            settingsWorld = m;
                            hudAnchored = true;
                        }

                        // Controller state for pointer interaction.
                        float ptrOx=px, ptrOy=py, ptrOz=pz;
                        float ptrDx=-hRot.m[8], ptrDy=-hRot.m[9], ptrDz=-hRot.m[10];
                        bool  ptrFromController = false;
                        bool  ptrGrab = gOkHeld.load();
                        float ptrStickY = 0.0f;
                        bool  recenterDown = false;
                        {
                            CtrlState cc[2];
                            { std::lock_guard<std::mutex> lk(gCtrlMutex); cc[0]=gCtrl[0]; cc[1]=gCtrl[1]; }
                            for (int hh=0; hh<2; hh++)
                                if (cc[hh].conn==1 && cc[hh].keyCount>5 && cc[hh].keys[5]!=0) recenterDown = true;
                            auto trig = [](const CtrlState &s){
                                return (s.keyCount>2 && s.keys[2]!=0) || (s.keyCount>8 && s.keys[8]>40);
                            };
                            for (int hh=0; hh<2; hh++)
                                if (hh != gDominantHand && cc[hh].conn==1 && trig(cc[hh])) gDominantHand.store(hh);
                            int h = (cc[gDominantHand.load()].conn==1) ? gDominantHand.load()
                                  : (cc[0].conn==1 ? 0 : (cc[1].conn==1 ? 1 : -1));
                            if (h >= 0) {
                                Quat oq = quatNorm({ -cc[h].q[0], -cc[h].q[1], cc[h].q[2], cc[h].q[3] });
                                float qx2=oq.x, qy2=oq.y, qz2=oq.z, qw2=oq.w;
                                ptrDx = -2.0f*(qx2*qz2 + qw2*qy2);
                                ptrDy = -2.0f*(qy2*qz2 - qw2*qx2);
                                ptrDz = -(1.0f - 2.0f*(qx2*qx2 + qy2*qy2));
                                float ux = 2.0f*(qx2*qy2 - qw2*qz2);
                                float uy = 1.0f - 2.0f*(qx2*qx2 + qz2*qz2);
                                float uz = 2.0f*(qy2*qz2 + qw2*qx2);
                const float kFrontOff = 0.075f;
                const float kUpOff    = -0.005f;
                                ptrOx = cc[h].pos[0]*0.001f + ptrDx*kFrontOff + ux*kUpOff;
                                ptrOy = cc[h].pos[1]*0.001f + ptrDy*kFrontOff + uy*kUpOff;
                                ptrOz = cc[h].pos[2]*0.001f + ptrDz*kFrontOff + uz*kUpOff;
                                ptrGrab = trig(cc[h]);
                                ptrFromController = true;
                                if (cc[h].keyCount > 1) ptrStickY = (cc[h].keys[1] - 128) / 127.0f;
                            }
                        }

                        // Recenter on app/menu button rising edge.
                        {
                            static bool recenterPrev = false;
                            if (recenterDown && !recenterPrev) {
                                hudAnchored = false;
                                if (gLobby) {
                                    float fx = -hRot.m[8], fz = -hRot.m[10];
                                    float fn = sqrtf(fx*fx + fz*fz);
                                    if (fn > 1e-5f) { fx /= fn; fz /= fn; } else { fx = 0; fz = -1; }
                                    float yaw = atan2f(-fx, -fz);
                                    float hp[3] = {px, py, pz};
                                    gLobby->recenter(hp, yaw);
                                }
                            }
                            recenterPrev = recenterDown;
                        }
                        if (gWivrnRecenterReq.exchange(false)) {
                            hudAnchored = false;
                            if (gLobby) {
                                float fx = -hRot.m[8], fz = -hRot.m[10];
                                float fn = sqrtf(fx*fx + fz*fz);
                                if (fn > 1e-5f) { fx /= fn; fz /= fn; } else { fx = 0; fz = -1; }
                                float yaw = atan2f(-fx, -fz);
                                float hp[3] = {px, py, pz};
                                gLobby->recenter(hp, yaw);
                            }
                        }

                        // Pointer ray vs panel plane for hit-testing.
                        bool okClicked = gOkClick.exchange(false);
                        static bool sPtrGrabPrev = false;
                        bool grabEdge = ptrGrab && !sPtrGrabPrev;
                        bool clickEdge = ptrFromController ? grabEdge : okClicked;

                        PanelInteract pi;
                        pi.ptrOx = ptrOx; pi.ptrOy = ptrOy; pi.ptrOz = ptrOz;
                        pi.ptrDx = ptrDx; pi.ptrDy = ptrDy; pi.ptrDz = ptrDz;
                        pi.ptrFromController = ptrFromController;
                        pi.ptrGrab = ptrGrab;
                        pi.ptrStickY = ptrStickY;
                        pi.clickEdge = clickEdge;
                        pi.grabEdge = grabEdge;
                        updateLobbyPanel(pi, settingsWorld);
                        sPtrGrabPrev = ptrGrab;

                        // Apply menu-requested side effects.
                        if (gEyeTrackReapply.exchange(false)) applyServerEyeTracking(gStreaming);
                        if (gBrightnessApply.exchange(false)) applyHmdBrightness(gBrightnessFrac.load(), env);
                        if (gEyeFoveationDirty.exchange(false) && g_stream)
                            g_stream->send_eye_foveation_override();

                        // Draw the lobby overlay into gSwap (on top of the video).
                        if (alvrFence) { glWaitSync(alvrFence, 0, GL_TIMEOUT_IGNORED); glDeleteSync(alvrFence); alvrFence = 0; }
                        glBindFramebuffer(GL_FRAMEBUFFER, gStreamFbo);
                        glDisable(GL_DEPTH_TEST); glDisable(GL_CULL_FACE); glDisable(GL_SCISSOR_TEST);

                        // Set input + upload pixels from Java's View rendering.
                        gAndroidUi.setInput(pi.cursorLx, pi.cursorLy, pi.cursorPressed, pi.cursorOnPanel, pi.clickEdge);
                        if (pi.cursorOnPanel)
                            androidUiPushTouch(AndroidUi::mToPxX(pi.cursorLx), AndroidUi::mToPxY(pi.cursorLy),
                                               pi.cursorPressed, pi.clickEdge, pi.ptrStickY);
                        else
                            androidUiPushTouch(-1, -1, false, false, pi.ptrStickY);
                        androidUiFetchAndUpload();

                        int ovW = 0, ovH = 0;
                        {
                            int ew = g_stream ? g_stream->eye_width.load() : 0;
                            int eh = g_stream ? g_stream->eye_height.load() : 0;
                            ovW = (ew > 0) ? ew : (int)gStreamW;
                            ovH = (eh > 0) ? eh : (int)gStreamH;
                        }
                        for (int e = 0; e < 2; e++) {
                            float ex = (e == 0 ? -softIpdM()*0.5f : softIpdM()*0.5f);
                            Mat4 eyeShift = mat4Translate(-ex, 0, 0);
                            Mat4 view = mat4Mul(eyeShift, viewBase);
                            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                                   GL_TEXTURE_2D, gSwap[e][gSwapIdx], 0);
                            glViewport(0, 0, ovW, ovH);

                            if (ptrFromController)
                                drawLaserBeam(ptrOx, ptrOy, ptrOz, ptrDx, ptrDy, ptrDz, pi.laserLen, sproj, view);
                            drawSettingsPanelVerts(pi.sliderVertCount, sproj, view, settingsWorld);

                            // Composite Android UI texture on top of the panel.
                            Mat4 mvp = mat4Mul(sproj, mat4Mul(view, settingsWorld));
                            glEnable(GL_BLEND);
                            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                            gAndroidUi.draw(mvp.m);
                            glDisable(GL_BLEND);

                            // Pointer cursor: ring when hovering, filled disc when pressed.
                            if (pi.cursorOnPanel)
                                drawPointerCursor(pi.cursorLx, pi.cursorLy, pi.cursorPressed, sproj, view, settingsWorld);
                        }
                        glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    }
                    // ---- Diag-only overlay: when diag HUD is on but lobby is closed,
                    // composite just the diag overlay (transparent bg) on the video. ----
                    {
                        int dm = gDiagHudMode.load();
                        bool wantDiagOnly = (dm != 0) && !gManualLobby.load();
                        static bool sDiagOnlyState = false;
                        if (wantDiagOnly != sDiagOnlyState) {
                            sDiagOnlyState = wantDiagOnly;
                            androidUiPushDiagOverlayOnly(wantDiagOnly);
                        }
                        if (wantDiagOnly) {
                            if (alvrFence) { glWaitSync(alvrFence, 0, GL_TIMEOUT_IGNORED); glDeleteSync(alvrFence); alvrFence = 0; }
                            glBindFramebuffer(GL_FRAMEBUFFER, gStreamFbo);
                            glDisable(GL_DEPTH_TEST); glDisable(GL_CULL_FACE); glDisable(GL_SCISSOR_TEST);
                            {
                                int ew = g_stream ? g_stream->eye_width.load() : 0;
                                int eh = g_stream ? g_stream->eye_height.load() : 0;
                                glViewport(0, 0, (GLsizei)(ew > 0 ? ew : gStreamW),
                                                  (GLsizei)(eh > 0 ? eh : gStreamH));
                            }
                            androidUiFetchAndUpload();
                            Mat4 hRot = quatToMat4(qx, qy, qz, qw);
                            Mat4 invRot = mat4Transpose3x3(hRot);
                            Mat4 viewBase = mat4Mul(invRot, mat4Translate(-px, -py, -pz));
                            float lobbyFovDeg = (fEyeTextureFov0 > 1.0f) ? fEyeTextureFov0 : 101.0f;
                            float fovy = lobbyFovDeg * (float)M_PI / 180.0f;
                            Mat4 sproj = mat4Perspective(fovy, 1.0f, 0.05f, 250.0f);
                            if (!hudAnchored) {
                                float fx = -hRot.m[8], fz = -hRot.m[10];
                                float fn = sqrtf(fx*fx + fz*fz);
                                if (fn > 1e-5f) { fx /= fn; fz /= fn; } else { fx = 0; fz = -1; }
                                const float kHudDist = 2.0f;
                                float ax = px + fx * kHudDist, ay = py, az = pz + fz * kHudDist;
                                float cphi = -fz, sphi = -fx;
                                Mat4 m = mat4Identity();
                                m.m[0] = cphi; m.m[2] = -sphi;
                                m.m[8] = sphi; m.m[10] = cphi;
                                m.m[12] = ax;  m.m[13] = ay;  m.m[14] = az;
                                settingsWorld = m;
                                hudAnchored = true;
                            }
                            int doW = 0, doH = 0;
                            {
                                int ew = g_stream ? g_stream->eye_width.load() : 0;
                                int eh = g_stream ? g_stream->eye_height.load() : 0;
                                doW = (ew > 0) ? ew : (int)gStreamW;
                                doH = (eh > 0) ? eh : (int)gStreamH;
                            }
                            for (int e = 0; e < 2; e++) {
                                float ex = (e == 0 ? -softIpdM()*0.5f : softIpdM()*0.5f);
                                Mat4 eyeShift = mat4Translate(-ex, 0, 0);
                                Mat4 view = mat4Mul(eyeShift, viewBase);
                                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                                       GL_TEXTURE_2D, gSwap[e][gSwapIdx], 0);
                                glViewport(0, 0, doW, doH);
                                Mat4 mvp = mat4Mul(sproj, mat4Mul(view, settingsWorld));
                                glEnable(GL_BLEND);
                                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                                gAndroidUi.draw(mvp.m);
                                glDisable(GL_BLEND);
                            }
                            glBindFramebuffer(GL_FRAMEBUFFER, 0);
                        }
                    }
                    // PIPELINE: fence THIS slot and flush so the GPU starts it, but
                    // do NOT block. The warp samples this slot a frame from now, by
                    // which point the fence is long-signalled -> no torn texture.
                    uint64_t _tEncStart = diagTiming ? nowNs() : 0;
                    if (gSwapFence[gSwapIdx]) glDeleteSync(gSwapFence[gSwapIdx]);
                    if (warnActive || gManualLobby.load() || (gDiagHudMode.load() != 0 && !gManualLobby.load())) {
                        // HUD / battery-popup path: extra work was issued in our ctx,
                        // ordered after ALVR via glWaitSync; a fresh fence covers it all.
                        gSwapFence[gSwapIdx] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
                        glFlush();
                    } else {
                        // No HUD -> our ctx issued NO work this frame, so the ALVR
                        // render fence IS this slot's fence directly. Skips the
                        // per-frame glWaitSync + a redundant second fence. (Sync
                        // objects are shared, so next frame's glClientWaitSync on
                        // our ctx still works.)
                        gSwapFence[gSwapIdx] = alvrFence; alvrFence = 0;
                    }
                    if (alvrFence) glDeleteSync(alvrFence);   // safety (unreached paths)
                    // Stash this frame's render pose with its ring slot. Read the
                    // server poses AFTER the blit (alvr_render_stream_opengl updates
                    // gLastServerPoses from the frames it just drew). Using poses
                    // from before the blit would be one frame behind the actual
                    // frame content, causing the warp to misreproject.
                    XrPosef serverPoses[2];
                    if (wivrn_get_server_pose(serverPoses)) {
                        for (int e = 0; e < 2; e++) {
                            outVP[e].pose.orientation = {
                                serverPoses[e].orientation.x,
                                serverPoses[e].orientation.y,
                                serverPoses[e].orientation.z,
                                serverPoses[e].orientation.w };
                            outVP[e].pose.position[0] = serverPoses[e].position.x;
                            outVP[e].pose.position[1] = serverPoses[e].position.y;
                            outVP[e].pose.position[2] = serverPoses[e].position.z;
                        }
                    }
                    gSwapVP[gSwapIdx][0] = outVP[0];
                    gSwapVP[gSwapIdx][1] = outVP[1];
                    gSwapFrameTs[gSwapIdx] = ts;
                    if (diagTiming) { _tEnc = nowNs(); if (_tEnc - _tEncStart > _mEnc) _mEnc = _tEnc - _tEncStart; }

                    // Submit the CURRENT frame's slot now that rendering is done.
                    // The fence was just issued, so wait with a generous budget.
                    // This is the zero-pipeline path: the frame we just rendered
                    // is the one handed to the warp, eliminating the +1 frame latency.
                    submitSlot(gSwapIdx, 20000000ULL /* ~20ms: render in flight */);
                    gPrevSwapIdx = gSwapIdx; gPrevSwapValid = true;
                }
                // Report the submit->present queue time. GetFractionalVsync()'s
                // fractional part = progress through the current refresh interval,
                // so time-to-next-vsync = (1-frac)*interval. Fall back to one
                // interval if the oracle reads out of range.
                float interval = 1e9f / (gRefreshHint > 1.0f ? gRefreshHint : 72.0f);
                double fv = PVR::GetFractionalVsync();
                double frac = fv - floor(fv);
                float timeToVsync = (frac >= 0.0 && frac <= 1.0) ? (float)((1.0 - frac) * interval)
                                                                 : interval;
                // No +1-frame pipeline: the texture we just rendered for `ts` IS
                // the one handed to the warp this iteration, so its submit->photon
                // queue is just the time to the next vsync.
                float vsyncQ = timeToVsync;
                alvr_report_submit(ts, (uint64_t) vsyncQ);
                gSwapIdx = (gSwapIdx + 1) % kSwapLen;
                // Measure actual decoder output. submits = loop iterations that
                // presented a fresh frame; decoded = TRUE count of frames pulled
                // from the decoder (sum of `drained`); dropped = frames discarded
                // because >1 arrived in one iteration. decoded is the real decoder
                // FPS; if decoded~72 but submits<72 we're coalescing, not starving.
                static int sSubmits = 0, sDecoded = 0, sDropped = 0; static uint64_t vT0 = 0;
                // Fresh stream: clear stale per-second counters + the diag gap anchor,
                // then consume the reset flag (last user of it this iteration).
                if (gResetPacer) {
                    sSubmits = 0; sDecoded = 0; sDropped = 0; vT0 = 0;
                    _lastStart = 0;
                    gResetPacer = false;
                }
                sSubmits++;
                sDecoded += drained;
                if (drained > 1) sDropped += (drained - 1);
                uint64_t vNow = nowNs();
                if (vT0 == 0) vT0 = vNow;
                if (vNow - vT0 >= 1000000000ULL) {
                    float dtS = (vNow - vT0) * 1e-9f;
                    LOGI("VIDEO: decoded=%d submits=%d dropped(coalesced)=%d  (panel 72Hz, negotiated %.0fHz)",
                         sDecoded, sSubmits, sDropped, gRefreshHint);
                    if (diagTiming)
                        LOGI("VIDEO-TIMING(ms,max): gap=%.1f render=%.1f enc=%.1f enq=%.1f  "
                             "(gap=submit-to-submit incl tracking/sleep; enq=submit to SDK warp, "
                             "incl vsync backpressure -- not warp compute; >13.3 = a coalesce)",
                             _mGap/1e6, _mRender/1e6, _mEnc/1e6, _mEnq/1e6);

                    // Feed the Java stats tab with detailed per-second data.
                    if (g_stream && g_stream->session) {
                        uint64_t rx = g_stream->session->bytes_received();
                        uint64_t tx = g_stream->session->bytes_sent();
                        float bw_rx = (float)(rx - g_stream->stats_bytes_rx) / dtS;
                        float bw_tx = (float)(tx - g_stream->stats_bytes_tx) / dtS;
                        g_stream->stats_bytes_rx = rx;
                        g_stream->stats_bytes_tx = tx;
                        g_stream->stats_bandwidth_rx = 0.8f * g_stream->stats_bandwidth_rx + 0.2f * bw_rx;
                        g_stream->stats_bandwidth_tx = 0.8f * g_stream->stats_bandwidth_tx + 0.2f * bw_tx;

                        auto bd = g_latency.get_avg_breakdown_ms();
                        int64_t total_ns = g_latency.get_avg_total_latency_ns();
                        float total_ms = total_ns / 1e6f;
                        float fps = (float)sDecoded / dtS;

                        g_stream->stats_fps = (int)lroundf(fps);
                        g_stream->stats_total_latency_ms = total_ms;
                        g_stream->stats_encode_ms   = bd[0];
                        g_stream->stats_send_ms     = bd[1];
                        g_stream->stats_network_ms  = bd[2];
                        g_stream->stats_decode_ms   = bd[3];
                        g_stream->stats_render_wait_ms = bd[4];
                        g_stream->stats_blit_ms     = bd[5];

                        LOGI("LATENCY: total=%.1fms enc=%.1f send=%.1f net=%.1f dec=%.1f wait=%.1f blit=%.1f fps=%.1f",
                             total_ms, bd[0], bd[1], bd[2], bd[3], bd[4], bd[5], fps);

                        // Push stats to Java UI for the streaming stats tab.
                        androidUiPushStats(
                            g_stream->stats_fps,
                            g_stream->stats_total_latency_ms,
                            g_stream->stats_bandwidth_rx,
                            g_stream->stats_bandwidth_tx,
                            g_stream->stats_cpu_time_ms,
                            g_stream->stats_gpu_time_ms,
                            g_stream->stats_encode_ms,
                            g_stream->stats_send_ms,
                            g_stream->stats_network_ms,
                            g_stream->stats_decode_ms,
                            g_stream->stats_render_wait_ms,
                            g_stream->stats_blit_ms,
                            gWivrnBitrateMbps.load(),
                            g_stream->eye_width.load(),
                            g_stream->eye_height.load(),
                            gWivrnMicrophone.load());
                    }

                    gVidDecoded.store(sDecoded); gVidSubmit.store(sSubmits); gVidDropped.store(sDropped);
                    gFenceTimeouts.store(sFenceTimeouts);
                    if (sFenceTimeouts > 0)
                        LOGI("%d warp-submit fence timeout(s) this second -- a slot wasn't "
                             "GPU-complete within budget (render overrun / thermal stall) -> possible tear",
                             sFenceTimeouts);
                    if (diagTiming) {
                        gGapMsX10.store((int)(_mGap/1e5)); gRenderMsX10.store((int)(_mRender/1e5));
                        gEncMsX10.store((int)(_mEnc/1e5)); gEnqMsX10.store((int)(_mEnq/1e5));
                    }
                    sSubmits = 0; sDecoded = 0; sDropped = 0; vT0 = vNow;
                    sFenceTimeouts = 0;
                    _mGap = _mRender = _mEnc = _mEnq = 0;
                }
            }
            // alvr_get_frame_timeout blocks on the decoder for up to ~2 frame
            // intervals, so under healthy streaming the loop is paced by frame
            // arrival. But when the device falls behind, the NEXT
            // alvr_get_frame_timeout returns a coalesced burst instantly and the
            // loop would spin faster than 72Hz. Two warp submits then land inside
            // one refresh, and the legacy DIATW latches each eye SEPARATELY ->
            // per-eye pose desync (the video detaches and swims on head turn).
            // Pace each iteration to >= one vsync so two submits can never share
            // a refresh. Only bites in the burst case; costs nothing under healthy
            // streaming. Do NOT target above 72Hz or drop the floor to 0.
            {
                const uint64_t kVsyncNs = (uint64_t)(1e9 / 72.0);   // 72Hz panel (do NOT exceed)
                uint64_t now = nowNs();
                uint64_t iterTime = now - tLoopStart;
                // If the iteration already consumed >= one vsync (frame wait +
                // render), don't add another vsync sleep -- that would double the
                // cycle to ~28ms (36Hz). Go straight to the next
                // alvr_get_frame_timeout which blocks until the next frame anyway.
                if (iterTime < kVsyncNs) {
                    // Iteration was faster than one vsync: sleep the remainder to
                    // prevent spinning faster than 72Hz in the burst case.
                    float interval = 1e9f / (gRefreshHint > 1.0f ? gRefreshHint : 72.0f);
                    double fv = PVR::GetFractionalVsync();
                    double frac = fv - floor(fv);
                    uint64_t sleepTarget;
                    if (frac >= 0.0 && frac <= 1.0) {
                        sleepTarget = now + (uint64_t)((1.0 - frac) * interval);
                    } else {
                        sleepTarget = tLoopStart + kVsyncNs;
                    }
                    uint64_t minTarget = tLoopStart + kVsyncNs;
                    if (sleepTarget < minTarget) sleepTarget = minTarget;
                    sleepUntilMonoNs(sleepTarget);
                }
            }
            frame++; framesWithSurface++;
            continue;
        }

        // The streaming video path above ends with `continue`, so this lobby
        // path only runs pre-stream / between streams. The in-stream overlay
        // draws on top of the video in the path above.
        // Keep streaming tabs visible when connected to the server even with
        // no video playing, so the user can launch apps from the headset.
        bool wivrnConnected = g_stream && g_stream->session && g_stream->connected_ns.load() > 0;
        gStreamingMode.store(wivrnConnected);
        // Auto-switch to the Launch tab when we first connect.
        static bool wasConnected = false;
        if (wivrnConnected != wasConnected) {
            androidUiPushStreaming(wivrnConnected);
            if (wivrnConnected)
                gSettingsCat = 4;  // Launch tab
        }
        wasConnected = wivrnConnected;
        // If we were on a streaming-only tab but lost connection, fall back to Settings.
        if (!gStreamingMode.load()) {
            if (gSettingsCat >= 2)  // streaming-only tabs are 2+
                gSettingsCat = 1;   // Settings
        }

        // When streaming has started but the decoder isn't ready yet (first
        // frame hasn't arrived), show BLACK instead of the lobby UI. The lobby
        // render block still runs (it owns the warp submit path), but we skip
        // drawLobbyScene so the eye textures stay cleared to black.
        bool showBlack = gStreaming && !gDecoderReady;

        // ---- lobby (pre-stream / between streams): world-locked IP/status/model
        // HUD + floor grid + eye-gaze marker (Neo 2 EYE). Rendered in BOTH render
        // modes: HW compositor renders each eye into a ring texture fed to the SDK
        // warp; self-present does its own distortion + window present. ----
        // View from head pose: inverse head rotation, then -head position.
        uint64_t tIterStart = nowNs();   // pace this iteration's submit cadence (see trailing sleep)
        Mat4 headRot  = quatToMat4(qx, qy, qz, qw);
        Mat4 invRot   = mat4Transpose3x3(headRot);
        Mat4 viewBase = mat4Mul(invRot, mat4Translate(-px, -py, -pz));

        // Render at the SDK's render FOV (square target) so the distortion maps
        // correctly, matching the video path.
        float lobbyFovDeg = (fEyeTextureFov0 > 1.0f) ? fEyeTextureFov0 : 101.0f;
        float fovy = lobbyFovDeg * (float)M_PI / 180.0f;
        Mat4 proj  = mat4Perspective(fovy, 1.0f, 0.05f, 250.0f);   // square target -> aspect 1

        // ---- Unified lobby panel (wiVRn-style floating window) ----
        // A single world-locked panel with sidebar tabs (SERVERS, CONNECT,
        // VIDEO, AUDIO, INPUT, SYSTEM, DEBUG, LOBBY, ABOUT, EXIT) and a
        // content area. Always visible. No separate HUD or buttons.
        if ((frame % 120) == 0 || !(gIpText[0] >= '0' && gIpText[0] <= '9')) refreshDeviceIp();
        const float kHudDist = 2.0f;         // panel distance in front of the user

        int textVertCount = 0;   // no HUD text anymore

        // World-anchor the panel once per lobby entry: plant it kHudDist in front
        // of the current head, at head height, facing back toward the user, using
        // YAW ONLY (no pitch/roll) so it stands upright. Captured into hudWorld
        // and reused every frame thereafter -> the panel is fixed in space.
        if (!hudAnchored) {
            // head forward = local -Z in world (3rd column of the rotation, negated)
            float fx = -headRot.m[8], fz = -headRot.m[10];
            float fn = sqrtf(fx*fx + fz*fz);
            if (fn > 1e-5f) { fx /= fn; fz /= fn; } else { fx = 0; fz = -1; }  // flat forward
            float ax = px + fx * kHudDist, ay = py, az = pz + fz * kHudDist;   // anchor position
            // yaw so the panel's local +Z (its readable face) points back at the
            // user (-forward): RotY(phi) maps +Z -> (sin phi, 0, cos phi) = (-fx,0,-fz)
            float cphi = -fz, sphi = -fx;        // already unit-length
            Mat4 m = mat4Identity();
            m.m[0] = cphi; m.m[2] = -sphi;       // column-major yaw about +Y
            m.m[8] = sphi; m.m[10] = cphi;
            m.m[12] = ax;  m.m[13] = ay; m.m[14] = az;   // translation
            hudWorld = m;

            // Unified lobby panel: same facing, same distance.
            settingsWorld = m;

            hudAnchored = true;
        }

        // ===== UNIFIED LOBBY POINTER =========================================
        // One ray drives every interactable element (settings panel controls and
        // the EQ): a controller LASER when any controller is connected, otherwise
        // the head-gaze ray. The gaze reticle is shown ONLY when there's no
        // controller. "grab" = controller trigger held / OK button held; a single
        // "click" = trigger press-edge (controller) or OK click (gaze).
        float ptrOx=px, ptrOy=py, ptrOz=pz;
        float ptrDx=-headRot.m[8], ptrDy=-headRot.m[9], ptrDz=-headRot.m[10];
        bool  ptrFromController = false;   // pointer is a controller laser (vs head gaze)
        bool  ptrGrab = gOkHeld.load();
        float ptrStickY = 0.0f;            // dominant-hand thumbstick Y (-1..1, up = +)
        bool  recenterDown = false;        // app/menu button (either hand) -> re-anchor panels
        // Per-hand world pose for drawing the controller models (0=L,1=R).
        bool  ctrlConn[2] = {false,false};
        float ctrlPos[2][3] = {{0,0,0},{0,0,0}};
        float ctrlQuat[2][4] = {{0,0,0,1},{0,0,0,1}};
        int   ctrlKeys[2][16] = {{0},{0}};
        int   ctrlKeyCount[2] = {0,0};
        {
            CtrlState cc[2];
            { std::lock_guard<std::mutex> lk(gCtrlMutex); cc[0]=gCtrl[0]; cc[1]=gCtrl[1]; }
            for (int hh=0; hh<2; hh++)
                if (cc[hh].conn==1 && cc[hh].keyCount>5 && cc[hh].keys[5]!=0) recenterDown = true;
            auto trig = [](const CtrlState &s){
                return (s.keyCount>2 && s.keys[2]!=0) || (s.keyCount>8 && s.keys[8]>40);
            };
            // Capture both hands' world pose for the controller-model draw.
            // Same conversion as the laser path: pos*0.001, quat = (-x,-y,z,w).
            for (int hh=0; hh<2; hh++) if (cc[hh].conn==1) {
                ctrlConn[hh] = true;
                ctrlPos[hh][0]=cc[hh].pos[0]*0.001f; ctrlPos[hh][1]=cc[hh].pos[1]*0.001f; ctrlPos[hh][2]=cc[hh].pos[2]*0.001f;
                Quat cq = quatNorm({ -cc[hh].q[0], -cc[hh].q[1], cc[hh].q[2], cc[hh].q[3] });
                ctrlQuat[hh][0]=cq.x; ctrlQuat[hh][1]=cq.y; ctrlQuat[hh][2]=cq.z; ctrlQuat[hh][3]=cq.w;
                ctrlKeyCount[hh] = cc[hh].keyCount;
                for (int k=0; k<cc[hh].keyCount && k<16; k++) ctrlKeys[hh][k] = cc[hh].keys[k];
            }
            // Off-hand dominance: pulling the trigger on a connected NON-dominant
            // controller claims the laser.
            for (int hh=0; hh<2; hh++)
                if (hh != gDominantHand && cc[hh].conn==1 && trig(cc[hh])) gDominantHand.store(hh);
            int h = (cc[gDominantHand.load()].conn==1) ? gDominantHand.load()
                  : (cc[0].conn==1 ? 0 : (cc[1].conn==1 ? 1 : -1));
            if (h >= 0) {
                // Same conversion as the streaming controller path (CV service
                // returns a world-frame pose): pos*0.001 and (-x,-y,z,w) directly.
                Quat oq = quatNorm({ -cc[h].q[0], -cc[h].q[1], cc[h].q[2], cc[h].q[3] });
                float qx2=oq.x, qy2=oq.y, qz2=oq.z, qw2=oq.w;
                ptrDx = -2.0f*(qx2*qz2 + qw2*qy2);
                ptrDy = -2.0f*(qy2*qz2 - qw2*qx2);
                ptrDz = -(1.0f - 2.0f*(qx2*qx2 + qy2*qy2));
                float fwd0x = ptrDx, fwd0y = ptrDy, fwd0z = ptrDz;   // untilted local forward
                // controller local up axis (+Y)
                float ux = 2.0f*(qx2*qy2 - qw2*qz2);
                float uy = 1.0f - 2.0f*(qx2*qx2 + qz2*qz2);
                float uz = 2.0f*(qy2*qz2 + qw2*qx2);
                // Emit from a FIXED point on the model, the front-top tip near
                // the ring/trigger, expressed in the controller's OWN axes
                // (untilted forward + local up), so it stays anchored there in
                // every orientation.
                const float kFrontOff = 0.075f;    // toward the front tip (pointing axis)
                const float kUpOff    = -0.005f;   // down the front face toward the tip
                ptrOx = cc[h].pos[0]*0.001f + fwd0x*kFrontOff + ux*kUpOff;
                ptrOy = cc[h].pos[1]*0.001f + fwd0y*kFrontOff + uy*kUpOff;
                ptrOz = cc[h].pos[2]*0.001f + fwd0z*kFrontOff + uz*kUpOff;
                ptrGrab = trig(cc[h]);
                ptrFromController = true;
                // thumbstick Y for menu page-scrolling (keys[1]: center 128, 0..255)
                if (cc[h].keyCount > 1) ptrStickY = (cc[h].keys[1] - 128) / 127.0f;
            }
        }

        // Recenter: the app/menu button re-anchors the lobby panels in front of the
        // head on its rising edge (like the streaming recenter).
        {
            static bool recenterPrev = false;
            if (recenterDown && !recenterPrev) {
                hudAnchored = false;
                if (gLobby) {
                    float fx = -headRot.m[8], fz = -headRot.m[10];
                    float fn = sqrtf(fx*fx + fz*fz);
                    if (fn > 1e-5f) { fx /= fn; fz /= fn; } else { fx = 0; fz = -1; }
                    float yaw = atan2f(-fx, -fz);
                    float hp[3] = {px, py, pz};
                    gLobby->recenter(hp, yaw);
                }
            }
            recenterPrev = recenterDown;
        }

        // Full recenter from controller home long-press or settings button:
        // tracker sets lobby_recenter_requested, render thread does the panel
        // re-anchor (needs head pose). Sensor reset + height calibration already
        // done by tracker.
        if (g_stream && g_stream->tracker.lobby_recenter_requested.exchange(false))
        {
            hudAnchored = false;
            if (gLobby) {
                float fx = -headRot.m[8], fz = -headRot.m[10];
                float fn = sqrtf(fx*fx + fz*fz);
                if (fn > 1e-5f) { fx /= fn; fz /= fn; } else { fx = 0; fz = -1; }
                float yaw = atan2f(-fx, -fz);
                float hp[3] = {px, py, pz};
                gLobby->recenter(hp, yaw);
                LOGI("lobby recentered by controller home long-press");
            }
        }

        // Settings panel RECENTER button: full recenter (tracker + sensor + lobby).
        if (gWivrnRecenterReq.exchange(false))
        {
            LOGI("recenter triggered by settings button");
            if (g_stream) {
                g_stream->tracker.recenter_height();
                g_stream->tracker.recenter_requested.store(true);
            }
            Pvr_ResetSensorAll();
            svrRecenterOrientation();
            recenterHeadTrackerAW();
            hudAnchored = false;
            if (gLobby) {
                float fx = -headRot.m[8], fz = -headRot.m[10];
                float fn = sqrtf(fx*fx + fz*fz);
                if (fn > 1e-5f) { fx /= fn; fz /= fn; } else { fx = 0; fz = -1; }
                float yaw = atan2f(-fx, -fz);
                float hp[3] = {px, py, pz};
                gLobby->recenter(hp, yaw);
            }
        }

        bool okClicked = gOkClick.exchange(false);
        static bool sPtrGrabPrev = false;
        bool grabEdge = ptrGrab && !sPtrGrabPrev;
        bool clickEdge = ptrFromController ? grabEdge : okClicked;   // unified single-click

        PanelInteract pi;
        pi.ptrOx = ptrOx; pi.ptrOy = ptrOy; pi.ptrOz = ptrOz;
        pi.ptrDx = ptrDx; pi.ptrDy = ptrDy; pi.ptrDz = ptrDz;
        pi.ptrFromController = ptrFromController;
        pi.ptrGrab = ptrGrab;
        pi.ptrStickY = ptrStickY;
        pi.clickEdge = clickEdge;
        pi.grabEdge = grabEdge;
        updateLobbyPanel(pi, settingsWorld);
        if (!ptrGrab && gEqGrabbing) { gEqGrabbing = false; gEqActiveBand = -1; }

        sPtrGrabPrev = ptrGrab;
        bool showReticle = (!ptrFromController && pi.lobbyHover);

        // ---- apply menu-requested side effects ----
        // The data-driven menu has no GL context / JNIEnv / locomotion access, so
        // its callbacks raise flags; perform the real effect here.
        if (gEyeTrackReapply.exchange(false)) applyServerEyeTracking(gStreaming);
        if (gBrightnessApply.exchange(false)) applyHmdBrightness(gBrightnessFrac.load(), env);
        // Eye-foveation toggle changed (or streaming just started): push the
        // override_foveation_center packet so the server tracks gaze or pins
        // the center accordingly. Skipped while not streaming so a lobby flip
        // doesn't spam a disconnected session.
        if (gEyeFoveationDirty.exchange(false) && gStreaming && g_stream)
            g_stream->send_eye_foveation_override();

        // Render the lobby scene (passthrough background + world-locked HUD +
        // gaze disc) into the currently-bound FBO. Shared by the HW-compositor
        // and self-present paths.
        auto drawLobbyScene = [&](const Mat4 &sproj, const Mat4 &sview, const Mat4 &sEyeShift, int eyeIdx) {
            // Simple 3D lobby environment when passthrough is off.
            if (!(gPassthrough && gPassthrough->is_camera_on()) && gSimpleLobby)
                gSimpleLobby->draw(sproj, sview);
            // Passthrough camera background. Drawn first as a fullscreen quad so
            // everything else composites on top.
            if (gPassthrough && gPassthrough->is_camera_on())
                gPassthrough->draw(eyeIdx);
            glDisable(GL_DEPTH_TEST); glDisable(GL_CULL_FACE);
            if (ptrFromController)
                drawLaserBeam(ptrOx, ptrOy, ptrOz, ptrDx, ptrDy, ptrDz, pi.laserLen, sproj, sview);
            // Controller models: textured solid meshes attached to each live
            // controller pose. Texture swaps based on button state.
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LEQUAL);
            for (int h = 0; h < 2; h++) {
                if (!ctrlConn[h] || gCtrlVertCount[h] <= 0) continue;
                Mat4 M = quatToMat4(ctrlQuat[h][0], ctrlQuat[h][1], ctrlQuat[h][2], ctrlQuat[h][3]);
                M.m[12] = ctrlPos[h][0]; M.m[13] = ctrlPos[h][1]; M.m[14] = ctrlPos[h][2];
                Mat4 cMvp = mat4Mul(sproj, mat4Mul(sview, M));
                // Pick texture based on button state.
                // k[8]=trigger(0-255), k[4]=touchpad click, k[6]=app, k[5]=home
                int texIdx = TEX_IDLE;
                const int *k = ctrlKeys[h];
                int kc = ctrlKeyCount[h];
                if (kc > 8 && k[8] > 40)       texIdx = TEX_TRIG;
                else if (kc > 4 && k[4] != 0)  texIdx = TEX_TOUCH;
                else if (kc > 6 && k[6] != 0)  texIdx = TEX_APP;
                else if (kc > 5 && k[5] != 0)  texIdx = TEX_HOME;
                GLuint tex = gCtrlTex[texIdx] ? gCtrlTex[texIdx] : gCtrlTex[TEX_IDLE];
                glUseProgram(gCtrlProg);
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, tex);
                glUniform1i(gCtrlTexLoc, 0);
                glBindVertexArray(gCtrlVao[h]);
                glUniformMatrix4fv(gCtrlMvpLoc, 1, GL_FALSE, cMvp.m);
                glDrawArrays(GL_TRIANGLES, 0, gCtrlVertCount[h]);
                glBindVertexArray(0);
            }
            glDisable(GL_DEPTH_TEST);
            // Eye-gaze debug marker: the green disc at the live RAW Tobii gaze
            // point. Gated by the persisted EYE DEBUG toggle.
            if (gEyeDebugOn.load() && gEyeOnline.load() && gGazeValid.load() && gGazeVertCount > 0) {
                const float gd = 1.6f;
                float g0=gGazeLocal[0].load(), g1=gGazeLocal[1].load(), g2=gGazeLocal[2].load();
                float dx = headRot.m[0]*g0 + headRot.m[4]*g1 + headRot.m[8]*g2;
                float dy = headRot.m[1]*g0 + headRot.m[5]*g1 + headRot.m[9]*g2;
                float dz = headRot.m[2]*g0 + headRot.m[6]*g1 + headRot.m[10]*g2;
                float dn = sqrtf(dx*dx+dy*dy+dz*dz);
                if (dn > 1e-5f) { dx/=dn; dy/=dn; dz/=dn; }
                // Orient the disc TANGENT to the gaze hemisphere: its +Z normal
                // points from the marker back toward the head, so the dot sits
                // flush on a half-sphere and always faces the user.
                float nx=-dx, ny=-dy, nz=-dz;                 // marker -> head
                float rx = 1.0f*nz - 0.0f*ny;                 // right = worldUp(0,1,0) x n
                float ry = 0.0f*nx - 0.0f*nz;
                float rz = 0.0f*ny - 1.0f*nx;
                float rn = sqrtf(rx*rx+ry*ry+rz*rz);
                if (rn < 1e-4f) { rx=1; ry=0; rz=0; rn=1; }   // gaze ~vertical: stable fallback
                rx/=rn; ry/=rn; rz/=rn;
                float ux = ny*rz - nz*ry;                     // up = n x right
                float uy = nz*rx - nx*rz;
                float uz = nx*ry - ny*rx;
                Mat4 face = mat4Identity();
                face.m[0]=rx; face.m[1]=ry; face.m[2]=rz;
                face.m[4]=ux; face.m[5]=uy; face.m[6]=uz;
                face.m[8]=nx; face.m[9]=ny; face.m[10]=nz;
                Mat4 mk = mat4Mul(mat4Translate(px+dx*gd, py+dy*gd, pz+dz*gd), face);
                Mat4 mkMvp = mat4Mul(sproj, mat4Mul(sview, mk));
                glUseProgram(gProg);
                glBindVertexArray(gGazeVao);
                glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, mkMvp.m);
                glDrawArrays(GL_TRIANGLES, 0, gGazeVertCount);
                glBindVertexArray(0);
            }
            // Native 3D lobby UI: unified floating panel (wiVRn-style).
            drawSettingsPanelVerts(pi.sliderVertCount, sproj, sview, settingsWorld);
            // ImGui overlay: composite the offscreen UI texture as a 3D quad.
            {
                Mat4 mvp = mat4Mul(sproj, mat4Mul(sview, settingsWorld));
                glDisable(GL_DEPTH_TEST);
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                gAndroidUi.draw(mvp.m);
                glDisable(GL_BLEND);
            }
            // Pointer cursor: ring when hovering, filled disc when pressed.
            // Drawn on top of the ImGui quad so the user sees where the ray hits.
            if (pi.cursorOnPanel)
                drawPointerCursor(pi.cursorLx, pi.cursorLy, pi.cursorPressed, sproj, sview, settingsWorld);

            // PIN pad overlay removed - PIN entry now handled by Android UI.

            glEnable(GL_DEPTH_TEST); glEnable(GL_CULL_FACE);
        };

        // ===== HW-COMPOSITOR LOBBY: render each eye into its ring texture and
        // hand them to the SDK warp (same submit path as the video). The warp
        // owns the window, applies lens distortion + async reprojection, and
        // presents. RENDER EVERY FRAME: do NOT gate on a content-dirty signature
        //, this lobby submit path does NOT async-reproject a re-submitted stale
        // frame, so between redraws the world-locked view would freeze and head
        // motion judder. The lobby is only shown pre-stream/between streams, so
        // drawing it every frame costs no in-game thermal headroom. =====
        {
            // We're back in the lobby, cancel any pending deferred free of the
            // eye-texture ring so we keep reusing the live ring.
            lobbyFreeDelay = -1;
            if (!gLobbyEyeReady && rtInited) {   // rebuild after a stream freed it
                eglMakeCurrent(dpy, pbuf, pbuf, ctx);
                buildLobbyTarget();
                lastRenderedIdx = -1; lobbyEyeIdx = 0;
            }
            if (gLobbyEyeReady && rtInited) {
                {
                    eglMakeCurrent(dpy, pbuf, pbuf, ctx);   // stay offscreen; warp owns the window
                    if (gGridThemeDirty.exchange(false)) { buildControllerMeshes(); }   // recolour controllers (grid floor removed, passthrough replaces it)
                    // Set input + fetch pixels from Java's View rendering once per frame.
                    gAndroidUi.setInput(pi.cursorLx, pi.cursorLy, pi.cursorPressed, pi.cursorOnPanel, pi.clickEdge);
                    if (pi.cursorOnPanel)
                        androidUiPushTouch(AndroidUi::mToPxX(pi.cursorLx), AndroidUi::mToPxY(pi.cursorLy),
                                           pi.cursorPressed, pi.clickEdge, pi.ptrStickY);
                    else
                        androidUiPushTouch(-1, -1, false, false, pi.ptrStickY);
                    androidUiFetchAndUpload();
                    for (int eye = 0; eye < 2; eye++) {
                        float ex = (eye == 0 ? -softIpdM()*0.5f : softIpdM()*0.5f);
                        Mat4 eyeShift = mat4Translate(-ex, 0, 0);
                        Mat4 view = mat4Mul(eyeShift, viewBase);
                        glBindFramebuffer(GL_FRAMEBUFFER, gLobbyFbo);
                        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                               GL_TEXTURE_2D, gLobbyEye[eye][lobbyEyeIdx], 0);
                        // Re-attach depth every frame: the Adreno driver drops FBO
                        // attachments after EGL surface destroy/recreate cycles.
                        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                                  GL_RENDERBUFFER, gLobbyDepth);
                        glDisable(GL_SCISSOR_TEST);
                        glViewport(0, 0, kLobbySz, kLobbySz);
                        // Always clear colour to black first. If the passthrough
                        // camera has a frame, drawLobbyScene overwrites it. If
                        // not, we get a clean black frame instead of smearing the
                        // previous frame's content.
                        glClearColor(0, 0, 0, 1);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        if (!showBlack) {
                            drawLobbyScene(proj, view, eyeShift, eye);
                            drawBatteryWarn(eye);   // low-battery popup, layered over lobby content
                        }
                    }
                    glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    // PIPELINE: fence THIS slot + glFlush so the GPU starts it,
                    // but DON'T block. We hand the warp the slot we rendered LAST
                    // frame (below), GPU-complete a full frame ago, which
                    // avoids a per-frame glClientWaitSync stall. Same +1-frame
                    // submit pipeline the video path uses; the lobby is shown only
                    // pre/between streams, so the one extra frame of latency is
                    // invisible and both eyes still submit together.
                    if (gLobbyFence[lobbyEyeIdx]) glDeleteSync(gLobbyFence[lobbyEyeIdx]);
                    gLobbyFence[lobbyEyeIdx] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
                    glFlush();
                    gLobbyPoseQ[lobbyEyeIdx] = quatNorm({ qx, qy, qz, qw });   // pose this slot was rendered at
                    gLobbyPoseP[lobbyEyeIdx][0] = px; gLobbyPoseP[lobbyEyeIdx][1] = py; gLobbyPoseP[lobbyEyeIdx][2] = pz;
                    int thisIdx = lobbyEyeIdx;
                    lobbyEyeIdx = (lobbyEyeIdx + 1) % kLobbyRing;
                    // Submit the PREVIOUS frame's slot; the very first frame has
                    // none, so fall back to the slot we just rendered.
                    // When the passthrough camera is on, skip the pipeline and
                    // submit THIS frame's slot instead. The 1-frame pipeline
                    // latency is invisible for the world-locked lobby UI, but
                    // the passthrough camera image is screen-locked and the
                    // extra frame of staleness shows up as visible judder when
                    // the head moves. The glClientWaitSync stall below is
                    // acceptable because the lobby only runs pre/between streams.
                    bool cam_on = gPassthrough && gPassthrough->is_camera_on();
                    lobbySubmitIdx = (cam_on || lastRenderedIdx < 0) ? thisIdx : lastRenderedIdx;
                    lastRenderedIdx = thisIdx;
                    // Make sure the slot we're about to hand the warp is
                    // GPU-complete. The pipelined slot's fence was created last
                    // frame -> already signalled; the first-frame fallback waits
                    // on a fresh render under a wide one-off budget.
                    if (gLobbyFence[lobbySubmitIdx]) {
                        uint64_t budget = (lobbySubmitIdx == thisIdx) ? 50000000ULL : 5000000ULL;
                        glClientWaitSync(gLobbyFence[lobbySubmitIdx], GL_SYNC_FLUSH_COMMANDS_BIT, budget);
                    }
                }
                if (!gAtwEnabled) { Pvr_SetAsyncTimeWarp(1); gAtwEnabled = true; }
                PVR_CameraEndFrame(0, gLobbyEye[0][lobbySubmitIdx]);
                PVR_CameraEndFrame(1, gLobbyEye[1][lobbySubmitIdx]);
                // Submit the RENDER pose (not the live pose) so the warp's
                // reprojection delta is correct for the pipelined/reused slot too.
                Quat sQ = gLobbyPoseQ[lobbySubmitIdx];
                Mat4 rRot = quatToMat4(sQ.x, sQ.y, sQ.z, sQ.w);
                for (int e = 0; e < 2; e++) {
                    float ex = (e == 0 ? -softIpdM()*0.5f : softIpdM()*0.5f);
                    PvrPoseBlk blk; memset(&blk, 0, sizeof(blk));
                    blk.v[0] = sQ.x; blk.v[1] = sQ.y; blk.v[2] = sQ.z; blk.v[3] = sQ.w;
                    blk.v[4] = gLobbyPoseP[lobbySubmitIdx][0] + rRot.m[0]*ex;
                    blk.v[5] = gLobbyPoseP[lobbySubmitIdx][1] + rRot.m[1]*ex;
                    blk.v[6] = gLobbyPoseP[lobbySubmitIdx][2] + rRot.m[2]*ex;
                    PVR_ChangeRenderPose(e, 0, blk);
                }
                PVR_TimeWarpEvent(0);
            }
            // PACING: trailing sleep AFTER the submit, sized to hold a steady
            // ONE-VSYNC cadence. period = work + sleep; we sleep the remainder
            // of one 72Hz interval when there's slack. CRITICAL invariant: period
            // is ALWAYS >= one vsync, never target above the 72Hz panel and
            // always leave a floor gap, so two submits can never land in one
            // refresh and desync the per-eye DIATW latch.
            const uint64_t kVsyncNs  = (uint64_t)(1e9 / 72.0);   // 72Hz panel native (do NOT exceed)
            const uint64_t kMinGapNs = 3000000ULL;               // always yield >=3ms so the warp thread runs
            uint64_t work = nowNs() - tIterStart;
            uint64_t sleepNs = (work < kVsyncNs) ? (kVsyncNs - work) : 0;
            if (sleepNs < kMinGapNs) sleepNs = kMinGapNs;        // floor -> period stays >= one vsync
            usleep((useconds_t)(sleepNs / 1000));
            frame++; framesWithSurface++;
            continue;
        }

    }

    LOGI("render loop exit at frame %d", frame);
    // Stop + join the tracking thread before tearing down GL/JVM.
    gTrackRunning.store(false);
    pthread_join(gTrackThread, nullptr);
    // Tear down the ALVR core in stock order so a later nativeStart re-inits
    // a clean client: pause the stream, free the GL-side resources while the
    // context is still current, then destroy the core.
    alvr_pause();
    alvr_destroy_opengl();
    alvr_destroy();
    eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (sfc != EGL_NO_SURFACE) eglDestroySurface(dpy, sfc);
    if (pbuf != EGL_NO_SURFACE) eglDestroySurface(dpy, pbuf);
    if (curWin) ANativeWindow_release(curWin);
    if (ctx != EGL_NO_CONTEXT) eglDestroyContext(dpy, ctx);
    eglTerminate(dpy);
    gVM->DetachCurrentThread();
    return nullptr;
}

// Called from Java surface callbacks: hand the render thread a new window (or
// null when the surface is destroyed).
void setWindow(ANativeWindow *win) {
    std::lock_guard<std::mutex> lk(gWinMutex);
    // If a previous window is STILL pending (the render thread hasn't consumed
    // it yet), overwriting gPendingWindow would drop that ANativeWindow ref
    // without releasing it -> one leak per unconsumed surface callback. Release
    // it first. (Only safe while dirty: once consumed, curWin owns that ref.)
    if (gWindowDirty.load() && gPendingWindow && gPendingWindow != win)
        ANativeWindow_release(gPendingWindow);
    gPendingWindow = win;
    gWindowDirty.store(true);
}
