#include "eye_tracking.h"
#include "pico_sdk.h"
#include "log.h"
#include "app_state.h"          // gEyeDebugOn
#include <cmath>
#include <cstring>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

std::atomic<bool>  gEyeOnline{false};
// Per-element braces (not `= {...}`): copy-init form selects atomic's deleted
// copy ctor on libc++.
std::atomic<float> gGazeLocal[3] = { {0.0f}, {0.0f}, {-1.0f} };
std::atomic<bool>  gGazeValid{false};
float gEyeOpen[2] = {1,1};
bool  gEyeHaveOpen = false;

std::atomic<bool> gEyeSupported{false};
bool gServerEyeEnabled = false;
static std::atomic<bool> gEyeIrOn{false};   // EYE bit currently set (server stream OR debug toggle)

// Pico tracking-mode bits (ROTATION=0x1, POSITION=0x2, EYE=0x4). The EYE bit gates
// the IR illuminators + the eye service feeding Pvr_GetEyeTrackingData.
static const int MODE_POSITION = 0x2, MODE_EYE = 0x4;

// WiVRn has no server-side eye-tracking settings channel; always on when HW supports it.
static bool userEyeEnabled() {
    return true;
}

void initEyeTrackingMode() {
    int supported = Pvr_GetTrackingMode();
    gEyeSupported.store((supported & MODE_EYE) != 0);
    // Boot with IR off (POSITION-only); lit up later only if a stream needs gaze.
    bool setRc = Pvr_SetTrackingMode(MODE_POSITION);
    // The eye-mode worker is a process-lifetime singleton that survives
    // nativeStop/nativeStart. Reset eye state here so a fresh tracking thread
    // can't read stale eye data before applyServerEyeTracking re-evaluates.
    gEyeIrOn.store(false);
    gEyeOnline.store(false);
    gGazeValid.store(false);
    LOGI("eye: Pvr_GetTrackingMode supported=0x%x (EYE=%d); boot POSITION-only (IR off) set=%d",
         supported, gEyeSupported.load() ? 1 : 0, setRc ? 1 : 0);
}

// Pvr_SetTrackingMode BLOCKS for tens of ms; must never run on the render thread.
static void applyEyeModeNow(bool streaming) {
    gServerEyeEnabled = (streaming && gEyeSupported.load()) ? userEyeEnabled() : false;
    bool debugWants = gEyeDebugOn.load();
    bool wantEye = gEyeSupported.load() && (gServerEyeEnabled || debugWants);
    int mode = MODE_POSITION | (wantEye ? MODE_EYE : 0);
    bool setRc = Pvr_SetTrackingMode(mode);
    gEyeIrOn.store(wantEye);
    if (!wantEye) { gEyeOnline.store(false); gGazeValid.store(false); }   // forget stale gaze when IR is off
    LOGI("eye: tracking mode -> %s (streaming=%d supported=%d userEye=%d debug=%d) set=%d",
         wantEye ? "POSITION|EYE (IR on)" : "POSITION-only (IR off)",
         streaming ? 1 : 0, gEyeSupported.load() ? 1 : 0, gServerEyeEnabled ? 1 : 0,
         debugWants ? 1 : 0, setRc ? 1 : 0);
}

// Worker thread so the blocking SetTrackingMode never stalls the render thread.
// Requests coalesce to the latest desired state.
static std::mutex              gEyeReqMtx;
static std::condition_variable gEyeReqCv;
static bool                    gEyeReqPending = false;
static bool                    gEyeReqStreaming = false;
static bool                    gEyeWorkerStarted = false;

static void eyeModeWorker() {
    for (;;) {
        bool streaming;
        {
            std::unique_lock<std::mutex> lk(gEyeReqMtx);
            gEyeReqCv.wait(lk, [] { return gEyeReqPending; });
            gEyeReqPending = false;
            streaming = gEyeReqStreaming;
        }
        applyEyeModeNow(streaming);   // blocking SetTrackingMode, off the render thread
    }
}

void applyServerEyeTracking(bool streaming) {
    std::lock_guard<std::mutex> lk(gEyeReqMtx);
    if (!gEyeWorkerStarted) {
        gEyeWorkerStarted = true;
        std::thread(eyeModeWorker).detach();   // lives for the process; never joined
    }
    gEyeReqStreaming = streaming;   // coalesce to the latest desired state
    gEyeReqPending = true;
    gEyeReqCv.notify_one();
}

// Read Neo 2 EYE tracking and build OpenXR eye-gaze poses. out[0]=left, out[1]=right.
// gEyeOnline latches true when real eye data first arrives (only Eye units service
// Pvr_GetEyeTrackingData), gating all eye-related behaviour.
bool readEyeGazes(XrPosef out[2], bool *vL, bool *vR, int frame, Quat headQ) {
    *vL = *vR = false;
    if (!gEyeIrOn.load()) return false;
    int ls=0, rs=0, cs=0;
    float lp[3]={0}, rp[3]={0}, cp[3]={0};
    float lv[3]={0}, rv[3]={0}, cv[3]={0};
    float lo=0, ro=0, lpd=0, rpd=0;
    float lg[3]={0}, rg[3]={0}, fg[3]={0}; int fgs=0;
    bool ok = Pvr_GetEyeTrackingData(&ls,&rs,&cs,
        &lp[0],&lp[1],&lp[2], &rp[0],&rp[1],&rp[2], &cp[0],&cp[1],&cp[2],
        &lv[0],&lv[1],&lv[2], &rv[0],&rv[1],&rv[2], &cv[0],&cv[1],&cv[2],
        &lo,&ro, &lpd,&rpd,
        &lg[0],&lg[1],&lg[2], &rg[0],&rg[1],&rg[2],
        &fg[0],&fg[1],&fg[2], &fgs);
    if (!ok) return false;
    // NOTE: do NOT negate Z. The SDK negates Z for Unity (+Z forward); OpenXR
    // wants -Z forward = the native raw convention.
    // Per-eye gaze vectors (lv/rv) are unpopulated on Neo 2 EYE; only the COMBINED
    // gaze (cv) carries real direction. Capture openness before the gaze-validity
    // gate (a blink invalidates gaze but is when openness->0 matters).
    if (ls > 0 || rs > 0 || cs > 0) {
        gEyeOpen[0] = lo; gEyeOpen[1] = ro; gEyeHaveOpen = true;
    }
    float cmag = sqrtf(cv[0]*cv[0] + cv[1]*cv[1] + cv[2]*cv[2]);
    bool valid = (cs > 0 && cmag > 0.5f && cmag < 2.0f);
    if (!valid) {
        gGazeValid.store(false);
        static bool warned = false;
        if (!warned) {
            warned = true;
            LOGI("eye service responds but no valid combined gaze (cs=%d |cv|=%.2f) -- "
                 "not forwarding gaze (non-EYE unit or no eyes detected)", cs, cmag);
        }
        return false;
    }
    if (!gEyeOnline.load()) {
        gEyeOnline.store(true);
        LOGI("EYE TRACKING ONLINE: status L=%d R=%d C=%d", ls, rs, cs);
    }
    // Drive both eyes from the combined gaze. Decompose into decoupled spherical
    // pitch/yaw (q = Rx(pitch)*Ry(yaw)) so it round-trips through the server's
    // to_euler(XYZ) as exactly (pitch, yaw, 0) without cross-coupling.
    float gx = cv[0], gy = cv[1], gz = cv[2];
    float gn = sqrtf(gx*gx + gy*gy + gz*gz);
    if (gn > 1e-6f) { gx /= gn; gy /= gn; gz /= gn; }
    float yaw   = atan2f(-gx, sqrtf(gy*gy + gz*gz));
    float pitch = atan2f(gy, -gz);
    // Debug visualizer uses the RAW Tobii gaze vector directly.
    gGazeLocal[0].store(cv[0]); gGazeLocal[1].store(cv[1]); gGazeLocal[2].store(cv[2]);
    Quat qP = { sinf(pitch * 0.5f), 0.0f, 0.0f, cosf(pitch * 0.5f) };
    Quat qY = { 0.0f, sinf(yaw * 0.5f), 0.0f, cosf(yaw * 0.5f) };
    Quat q = quatNorm(quatMul(qP, qY));   // head-LOCAL gaze (for the debug marker via cv)
    // Server eye_gazes are in global tracking space, not head-relative. Compose
    // with head orientation so the gaze is world-aligned.
    q = quatNorm(quatMul(headQ, q));
    out[0] = {}; out[1] = {};
    out[0].orientation = { q.x, q.y, q.z, q.w };
    out[1].orientation = { q.x, q.y, q.z, q.w };
    *vL = *vR = true;
    gGazeValid.store(true);
    if ((frame % 60) == 0) {
        LOGI("eye: yaw=%.1f pitch=%.1f deg Cv=(%.3f,%.3f,%.3f) open=(%.2f,%.2f) status=(%d,%d,%d)",
             yaw*57.2957795f, pitch*57.2957795f, cv[0],cv[1],cv[2], lo,ro, ls,rs,cs);
    }
    return true;
}
