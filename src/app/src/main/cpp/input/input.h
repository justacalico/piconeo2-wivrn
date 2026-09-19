#pragma once
// Controller + head-pose state shared between JNI input sinks and the render thread.
#include <cstdint>
#include <mutex>

// ---------- Pico controllers (data pushed from Java ControllerClient) -------
// hand: 0=left, 1=right. Java polls per-hand state and pushes it via
// nativeControllerState(); render thread reads under gCtrlMutex.
struct CtrlState {
    int   conn = 0;                 // 1 = connected
    float q[4] = {0,0,0,1};         // orientation (Pico frame, raw)
    float pos[3] = {0,0,0};         // position (Pico frame, raw)
    float angVel[3] = {0,0,0};
    int   keys[16] = {0};
    int   keyCount = 0;
    bool  fresh = false;            // got at least one update
};
extern std::mutex gCtrlMutex;
extern CtrlState  gCtrl[2];

// Latest head pose (qx,qy,qz,qw,px,py,pz), published by the render thread for
// the Java controller poller to feed the SDK's head-aligned getControllerSensorState.
extern std::mutex gHeadMutex;
extern float      gHeadData[7];

// ---------- haptics (server -> controller rumble) ---------------------------
// The network thread parks rumble requests here via queueHaptic(); the Java
// controller poller drains them each tick through nativeDrainHaptic() and calls
// vibrateCV2ControllerStrength(). hand 0=L/1=R.
// Coalesce (keep strongest pulse) rather than queue: between two ~11ms polls at
// most one rumble matters.
struct PendingHaptic {
    float amplitude  = 0.0f;
    int   durationMs = 0;
    bool  pending    = false;
};
extern std::mutex     gHapticMutex;
extern PendingHaptic  gHaptic[2];

// Enqueue a rumble pulse. amplitude is clamped to (0,1]; duration_ns is
// truncated to ms and clamped to [12,1000]. While a pulse is pending the
// strongest amplitude and longest duration win.
void queueHaptic(int hand, float amplitude, int64_t duration_ns);
