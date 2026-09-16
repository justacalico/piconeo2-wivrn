#pragma once
// Prototypes for the prebuilt Pico (legacy Neo 2) native SDK we link against
// (libPvr_UnitySDK.so). Most are reverse-engineered ABI contracts, see the
// per-function notes. This header is the single place that documents them.

extern "C" {
    int   Pvr_Init(int index);
    int   Pvr_StartSensor(int index);
    int   Pvr_Enable6DofModule(bool enable);
    int   InitSensor();   // sets the sensor-TYPE selector (svr/falcon/usb...) from config;
                          // without it Pvr_GetMainSensorState returns identity.
    void  Pvr_SetInitActivity(void *activity, void *vrActivityClass);
    int   Pvr_GetMainSensorState(float *x, float *y, float *z, float *w,
                                 float *px, float *py, float *pz,
                                 float *vfov, float *hfov, int *viewNumber);
    // Controller tracking (Neo 2: 3DoF controllers + arm model). RE'd from
    // libPvr_UnitySDK: fills outQuat[4] (xyzw) + outPos[3]. No hand index --
    // returns the SDK's "current" controller. Data is fed via the Java controller
    // service (setControllerData); if that module is broken values stay idle/zero.
    void  Pvr_GetControllerTrackingData(float *outQuat, float *outPos);
    void  Pvr_GetPointerPose(float *outQuat, float *outPos);  // arm-model pointer
    // Eye tracking (Neo 2 EYE only; returns false on non-Eye units).
    // 35 out pointers: per-eye + combined gaze point/vector, eye openness,
    // pupil dilation, eye position guide, foveated gaze. We forward only the
    // per-eye gaze vector (consumed via the tracking uplink's eye_gazes).
    bool  Pvr_GetEyeTrackingData(
        int *lStatus, int *rStatus, int *cStatus,
        float *lPx, float *lPy, float *lPz,   // left gaze point
        float *rPx, float *rPy, float *rPz,   // right gaze point
        float *cPx, float *cPy, float *cPz,   // combined gaze point
        float *lVx, float *lVy, float *lVz,   // left gaze vector (direction)
        float *rVx, float *rVy, float *rVz,   // right gaze vector
        float *cVx, float *cVy, float *cVz,   // combined gaze vector
        float *lOpen, float *rOpen,           // eyelid openness
        float *lPupil, float *rPupil,         // pupil dilation
        float *lGuideX, float *lGuideY, float *lGuideZ,   // eye position guide
        float *rGuideX, float *rGuideY, float *rGuideZ,
        float *fovGazeX, float *fovGazeY, float *fovGazeZ, // foveated gaze dir
        int *fovGazeState);
    bool  Pvr_GetEyeTrackingAutoIPD(float *autoIPD);   // measured IPD (meters)
    // Tracking-mode bitmask: ROTATION=0x1, POSITION=0x2, EYE=0x4. Eye tracking
    // is OFF until we set POSITION|EYE (mirrors SDK's SetEyeTrackingMode).
    bool  Pvr_SetTrackingMode(int trackingMode);
    int   Pvr_GetTrackingMode();
    void  Pvr_DisableBoundary();
    void  Pvr_ShutdownSDKBoundary();
    // Tracking origin: 0=EyeLevel, 1=FloorLevel, 2=StageLevel. SteamVR
    // expects FLOOR origin; without it the head sits at y=0 and you spawn in the floor.
    bool  Pvr_SetTrackingOriginType(int trackingOriginType);
    // Recentering: Pvr_ResetSensorAll = full position+orientation reset.
    // svrRecenterOrientation resets full orientation (pitch+yaw+roll) for 3DoF.
    int   Pvr_ResetSensor(int resetType);
    int   Pvr_ResetSensorAll();
    void  recenterHeadTrackerAW();
    void  pvr_RecenterYaw();
    void  svrRecenterOrientation();
    void  svrRecenterPose();
#define PXR_RESET_ALL 3
    float Pvr_GetFloorHeight();   // device floor calibration height (diagnostic)
    // Play-area extents in meters: x=width, z=depth (y unused). isPlayArea=true
    // returns the inner play area, false the outer boundary. Must be read BEFORE
    // Pvr_ShutdownSDKBoundary(). Returns count of geometry points (0 if unset).
    int   Pvr_BoundaryGetDimensions(float *x, float *y, float *z, bool isPlayArea);
    bool  Pvr_BoundaryGetConfigured();
    // --- See-through / passthrough camera (RE'd from libPvr_UnitySDK) ---
    // The Neo 2 has a front-facing tracking camera. StartCameraPreview kicks the
    // frame loop; SetCameraImageRect sets delivery resolution. GetCameraData_Ext
    // returns a pointer to the latest raw RGBA frame buffer.
    // BoundaryGetSeeThroughData is the per-eye variant (cameraIndex 0=left, 1=right).
    // BoundarySetSeeThroughVisible gates runtime compositing of the camera layer.
    void  PVR_StartCameraPreview(int mode);
    void  PVR_SetCameraImageRect(int width, int height);
    unsigned char *Pvr_GetCameraData_Ext();
    unsigned char *Pvr_BoundaryGetSeeThroughData(int cameraIndex, bool *valid,
                                                  unsigned int *width, unsigned int *height,
                                                  unsigned int *count, long long *timestamp);
    void  Pvr_BoundarySetSeeThroughVisible(bool visible);
    void  Pvr_BoundarySeeThroughSetVisible_(bool visible);
    int   Pvr_GetSeeThroughState();
    // Distance thresholds that gate SeeThrough activation. Setting
    // fDstcToShowSeeThrough very large forces always-on (the system thinks
    // the user is always "close to boundary").
    extern float fDstcToShowSeeThrough;
    extern float fDstcToShowSeeThroughComp;
    void *GetRenderEventFunc();
    bool  Pvr_SetSinglePassDepthBufferWidthHeight(int width, int height);
    void  Pvr_GetFOV(float *outA, float *outB);
    // Set per-eye projection FOV in DEGREES. RE'd: stores into GlobalConfig
    // (offsets +20/+24). Under armeabi-v7a softfp, floats arrive in r0/r1.
    // The warp's distortion/projection is built from this at mesh-build time
    // (EV_InitRenderThread), so a live change needs the warp re-pointed.
    // This is the "higher DPI" lever: shrink FOV so the fixed eye buffer packs
    // more pixels into the visible lens cone.
    void  Pvr_SetProjectionFov(float fovXDeg, float fovYDeg);
    float Pvr_GetIPD();   // device IPD in METERS (Neo 2: fixed ~0.065)
    // --- HW compositor (DIATW) eye-buffer submit path ---
    // Feed eye textures to the SDK warp thread for HW lens distortion + async
    // reprojection + direct low-latency present.
    void  Pvr_SetCurrentRenderTexture(unsigned int texId);   // stores current eye tex
    void  PvrBeginEyeEvent();                                  // register tex into warp ring
    void  PvrEndEyeEvent();
    void  PVR_CameraEndFrame(unsigned int eye, unsigned int texId);  // store eye tex slot
    // pose block: 88 bytes (22 floats) passed BY VALUE starting in r2 (eye=r0, pad=r1).
    // Copied into ctx+eye*0x128+0x8ce0, then into the warp source by PVR_TimeWarpEvent.
    struct PvrPoseBlk { float v[22]; };
    void  PVR_ChangeRenderPose(unsigned int eye, unsigned int pad, struct PvrPoseBlk blk);
    void  Pvr_SetAsyncTimeWarp(unsigned char enable);
    // Per-frame submit: reads eye textures + pose -> builds warp source -> pvr_WarpSwap.
    void  PVR_TimeWarpEvent(unsigned int eye);
    // SDK data globals: per-eye texture FOV the distortion mesh expects, and
    // eye-buffer FOV in X/Y. Populated at Pvr_Init.
    extern float fEyeTextureFov0;
    extern float fEyeTextureFov1;
    extern float gEyeBufferFovX;
    extern float gEyeBufferFovY;
}

// ---- CPU/GPU performance-level pinning (Qualcomm VR perf service) ----------
// SetCpuLevel/SetGpuLevel push a perf LEVEL + sustained flag into the QVR perf
// service; return >=0 when applied, -1 if not bound yet (retry).
// C++-mangled (NOT extern "C"): _Z11SetCpuLevelib / _Z11SetGpuLevelib etc.
int  SetCpuLevel(int level, bool sustained);
int  SetGpuLevel(int level, bool sustained);
void GetCpuLevel(int &level, bool &sustained);
void GetGpuLevel(int &level, bool &sustained);

// ---- HMD panel backlight (lobby brightness slider + off-head auto-dim) ---
// Raw panel backlight level (0..255 on Neo 2 LCD). C++-mangled T symbols:
// _Z22SetHmdScreenBrightnessi / _Z22GetHmdScreenBrightnessRi.
// Falls back to Android window-brightness API if the SDK path doesn't take.
void SetHmdScreenBrightness(int brightness);
void GetHmdScreenBrightness(int &brightness);
constexpr int kBrightMin = 8, kBrightMax = 255;   // slider floor (avoid full black) .. panel max

// Vsync phase oracle (C++ symbol _ZN3PVR18GetFractionalVsyncEv). Returns fractional
// vsync number: integer part = count, fractional = progress through current refresh
// interval. Used to phase-lock drain+submit so the warp gets the freshest frame at
// a consistent content-age.
namespace PVR { double GetFractionalVsync(); }

// Internal Pico config accessors (C++-mangled, NOT the remapped public Pvr_Get*Config
// which only expose ~7 presets). These read arbitrary DECRYPTED config indices the
// SDK loaded at Pvr_Init, including lens distortion polynomial + chromatic aberration.
enum GlobalFloatConfigs : int {};
enum GlobalIntConfigs   : int {};
float  psmvr_GetFloatConfig(GlobalFloatConfigs idx);   // returns float (r0), NOT double
int    psmvr_GetIntConfig(GlobalIntConfigs idx);
static inline float cfgF(int i) { return psmvr_GetFloatConfig((GlobalFloatConfigs) i); }
static inline int   cfgI(int i) { return psmvr_GetIntConfig((GlobalIntConfigs) i); }

// SDK render-event ids (issued via GetRenderEventFunc). We only use InitRenderThread
// (wires up the tracking data pipe). We never issue TimeWarp (that path needs Unity
// and aborts); we present the eyes ourselves.
enum { EV_InitRenderThread = 1024, EV_Pause = 1025, EV_Resume = 1026 };
typedef void (*RenderEventFunc)(int);
