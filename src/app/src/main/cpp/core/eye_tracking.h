#pragma once
// Neo 2 EYE eye-tracking: read the Pico eye service and build OpenXR eye-gaze poses.
#include <openxr/openxr.h>      // XrPosef
#include "math3d.h"             // Quat
#include <atomic>

// Atomic: written on the tracking thread and eye-mode worker, read on the render
// thread. Per-field atomics are enough (the marker is cosmetic, inter-field tear
// is irrelevant).
// gEyeOnline latches true only when real eye data first arrives (Eye units only).
extern std::atomic<bool>  gEyeOnline;
// Latest COMBINED gaze direction in head-local space + validity (lobby marker).
extern std::atomic<float> gGazeLocal[3];
extern std::atomic<bool>  gGazeValid;
// Per-eye openness (0=closed..1=open), captured even mid-blink. Tracking-thread-only.
extern float gEyeOpen[2];
extern bool  gEyeHaveOpen;

// Read Neo 2 EYE tracking and build OpenXR eye-gaze poses. out[0]=left, out[1]=right.
// headQ = current head orientation (for global-space gaze compose).
bool readEyeGazes(XrPosef out[2], bool *vL, bool *vR, int frame, Quat headQ);

// ---- server-driven eye-tracking power gating ----
// IR illuminators only burn power while the EYE tracking-mode bit is set; keep it
// off unless the server is actually consuming gaze.
extern std::atomic<bool> gEyeSupported;  // EYE bit in Pvr_GetTrackingMode()
extern bool gServerEyeEnabled;   // server eye source on (eye-worker-only)
// Boot: detect EYE support but leave IR off (POSITION-only).
void initEyeTrackingMode();
// (Re)evaluate tracking mode: enable EYE bit only if supported AND streaming AND
// server wants gaze. Call on stream start / stop.
void applyServerEyeTracking(bool streaming);

// ---- pico_oxr / pico_native compatibility shims ----

extern std::atomic<float> gGazeQuat[4];
extern std::atomic<float> gEyeOpenness[2];
extern std::atomic<bool>  gEyeOpennessValid;
extern std::atomic<float> gGazePitch;
extern std::atomic<float> gGazeYaw;

inline void initEyeTracking() { initEyeTrackingMode(); }
inline void setEyeTrackingStreaming(bool streaming) { applyServerEyeTracking(streaming); }
void pollEyeGaze();
