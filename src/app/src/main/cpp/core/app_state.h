#pragma once
// Cross-cutting shared state: lobby/render knobs touched by the render loop,
// lobby panels, and JNI input handlers. Atomics where producer and consumer
// threads differ.
#include <atomic>
#include <cstdint>
#include <functional>

// Exit callback (set by JNI, triggered by the EXIT tab)
extern std::function<void()> gOnExit;

// ---- Software IPD (lobby-adjustable, persisted) ----------------------------
// Per-eye optical-centre offset for lobby render, warp submit pose, and the
// tracking uplink. Stored in millimetres.
constexpr float kIpdMin = 58.0f, kIpdMax = 72.0f, kIpdStep = 0.5f;
extern std::atomic<float>    gSoftIpdMm;
extern std::atomic<bool>     gIpdDirty;     // value changed, needs persisting
extern std::atomic<uint64_t> gIpdChangeNs;  // last-change time (debounce save)
extern std::atomic<float>    gSentIpdMm;    // IPD last pushed to the server via the tracker
inline float softIpdM() { return gSoftIpdMm.load() * 0.001f; }

// ---- lobby input edges (set by the JNI key handler, consumed by render loop) -
extern std::atomic<bool> gOkClick;   // OK/select pressed (edge)
extern std::atomic<bool> gOkHeld;    // OK/select held (gaze drag of the EQ)
extern std::atomic<bool> gSideHeld;  // headset SIDE button (keycode 1001) held

// Show the lobby mid-stream WITHOUT disconnecting.
extern std::atomic<bool> gManualLobby;

// ---- persisted toggles -----------------------------------------------------
extern std::atomic<bool> gEyeDebugOn;   // eye-gaze debug marker on/off
extern std::atomic<int>  gDiagHudMode;  // streaming diagnostics HUD: 0=off, 1=pipeline, 2=system (CPU/GPU/heat)
extern std::atomic<float> gBrightnessFrac; // HMD panel brightness 0..1 (VIDEO tab slider)
extern std::atomic<bool>  gBrightnessSaved; // a brightness value was persisted (vs. panel default)

// ---- wivrn client option placeholders (server-controlled or not yet wired) --
extern std::atomic<bool>   gWivrnTcpOnly;
extern std::atomic<int>    gWivrnCodec;        // 0=h264, 1=h265, 2=av1
extern std::atomic<int>    gWivrnFoveation;    // 0=off, 1=low, 2=medium, 3=high
extern std::atomic<float>  gWivrnBitrateMbps;  // 0..200
extern std::atomic<float>  gWivrnResolutionScale; // 0.5..2.0
extern std::atomic<bool>   gWivrnHandTracking;
extern std::atomic<bool>   gWivrnEyeTracking;
extern std::atomic<bool>   gWivrnPassthrough;
extern std::atomic<bool>   gWivrnMicrophone;
extern std::atomic<float>  gWivrnCtrlVibration; // 0..1
extern std::atomic<bool>   gWivrnRecenterReq;   // one-shot button flag

// Eye-tracked foveation: when true the client tells the server to derive the
// foveation center from the live EYE_GAZE pose instead of a fixed forward
// center. Only meaningful on Neo 2 EYE hardware (gEyeSupported) and requires
// the server to have foveation enabled.
extern std::atomic<bool>   gWivrnEyeFoveation;
// Raised by the settings callback / streaming start-stop to request the render
// thread re-send the override_foveation_center packet.
extern std::atomic<bool>   gEyeFoveationDirty;

// Foveated encoding master switch. When false, caps.foveated_encoding is set
// to false so the server sends full-resolution frames (no edge stretching).
// Takes effect on the next stream connection.
extern std::atomic<bool>   gFoveationEnabled;

// ---- Stream FOV (higher-DPI lever) -----------------------------------------
// The server renders into a fixed 1664^2 per-eye buffer at whatever FOV the
// client commands. Shrinking the commanded FOV packs the same pixels into fewer
// degrees = higher PPD at identical decoder/thermal cost. The warp texture FOV
// must shrink in lockstep (Pvr_SetProjectionFov + fEyeTextureFov0/1) to avoid
// magnification. 101 = SDK native per-eye FOV (headset max); lower to trade
// edge FOV for pixel density.
constexpr float kFovMin = 70.0f, kFovMax = 101.0f;   // full per-eye FOV range, degrees
extern std::atomic<float> gStreamFovDeg;   // full per-eye FOV, degrees (persisted)
extern std::atomic<bool>  gFovDirty;       // changed -> re-apply (server view_params + warp mesh)
void saveStreamFov();

// ---- render-thread side-effect requests ------------------------------------
// Menu callbacks raise these flags; the render loop polls + clears them and
// performs the actual effect (menu_model has no GL context/JNIEnv access).
extern std::atomic<bool> gGridThemeDirty;    // rebuild floor VBO (theme changed)
extern std::atomic<bool> gEyeTrackReapply;   // re-evaluate the Pico eye-tracking mode
extern std::atomic<bool> gBrightnessApply;   // push gBrightnessFrac to the HMD panel

// ---- per-second diagnostics published for the HUD --------------------------
extern std::atomic<int> gVidDecoded, gVidSubmit, gVidDropped;
extern std::atomic<int> gGapMsX10, gRenderMsX10, gEncMsX10, gEnqMsX10;
// Per-second count of warp-submit fence waits that timed out (slot wasn't
// GPU-complete within budget -> possible tear). >0 flags a render overrun.
extern std::atomic<int> gFenceTimeouts;

// ---- low-battery warning ---------------------------------------------------
// On a downward crossing of 15% / 5% the render loop stamps gBattWarnStartNs +
// the crossed percentage; the draw path shows a 5-second head-locked popup.
extern std::atomic<uint64_t> gBattWarnStartNs;   // 0 = inactive; else popup start (CLOCK_MONOTONIC ns)
extern std::atomic<int>      gBattWarnPct;        // percentage to display in the popup
constexpr uint64_t kBattWarnDurNs = 5000000000ULL;  // popup lifetime (5 s)

// ---- streaming / lobby mode ------------------------------------------------
extern std::atomic<bool> gStreamingMode;     // true while connected to a server
extern int gSettingsCat;                     // legacy 3D settings category index (stubbed)

// ---- PIN entry -------------------------------------------------------------
extern std::atomic<bool> gPinEntryRequested;  // server requested PIN pairing
extern std::atomic<int>  gDominantHand;       // 0=left, 1=right
#include <functional>
#include <string>
extern std::function<void(const std::string &)> gOnPinSubmit;
void requestPinEntryUI();
void submitPin(const std::string &pin);

// ---- EQ state (moved from eq_panel.h) --------------------------------------
extern std::atomic<bool> gEqGrabbing;
extern std::atomic<int>  gEqActiveBand;

// ---- persistence (single $HOME/config.txt) ---------------------------------
// loadAllConfig() restores every setting at boot; saveAllConfig() rewrites the
// whole file. saveX() wrappers just call saveAllConfig().
void loadAllConfig();
void saveAllConfig();
void saveSoftIpd();
void saveEyeDebug();
void saveDiagHud();
void saveBrightness();
