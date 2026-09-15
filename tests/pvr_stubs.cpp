// Host stubs for the handful of libPvr_UnitySDK entry points the eye-tracking
// code calls. Everything is driven through the test_pvr_* globals below.
#include <cstring>

int test_pvr_tracking_mode = 0x2;   // what Pvr_GetTrackingMode reports
int test_pvr_last_set_mode = -1;    // last mode passed to Pvr_SetTrackingMode
bool test_pvr_set_mode_rc = true;
int test_pvr_set_mode_calls = 0;

// Filled into Pvr_GetEyeTrackingData's out-params when it returns true.
struct test_eye_data
{
	int ls = 0, rs = 0, cs = 0;
	float cv[3] = {0, 0, -1};
	float lo = 1.0f, ro = 1.0f;
};

test_eye_data test_pvr_eye;
bool test_pvr_eye_ok = false;

extern "C" {

int Pvr_GetTrackingMode()
{
	return test_pvr_tracking_mode;
}

bool Pvr_SetTrackingMode(int trackingMode)
{
	test_pvr_last_set_mode = trackingMode;
	test_pvr_set_mode_calls++;
	return test_pvr_set_mode_rc;
}

bool Pvr_GetEyeTrackingData(
        int *lStatus, int *rStatus, int *cStatus,
        float *lPx, float *lPy, float *lPz,
        float *rPx, float *rPy, float *rPz,
        float *cPx, float *cPy, float *cPz,
        float *lVx, float *lVy, float *lVz,
        float *rVx, float *rVy, float *rVz,
        float *cVx, float *cVy, float *cVz,
        float *lOpen, float *rOpen,
        float *lPupil, float *rPupil,
        float *lGuideX, float *lGuideY, float *lGuideZ,
        float *rGuideX, float *rGuideY, float *rGuideZ,
        float *fovGazeX, float *fovGazeY, float *fovGazeZ,
        int *fovGazeState)
{
	if (!test_pvr_eye_ok)
		return false;
	*lStatus = test_pvr_eye.ls;
	*rStatus = test_pvr_eye.rs;
	*cStatus = test_pvr_eye.cs;
	*lPx = *lPy = *lPz = 0;
	*rPx = *rPy = *rPz = 0;
	*cPx = *cPy = *cPz = 0;
	*lVx = *lVy = *lVz = 0;
	*rVx = *rVy = *rVz = 0;
	*cVx = test_pvr_eye.cv[0];
	*cVy = test_pvr_eye.cv[1];
	*cVz = test_pvr_eye.cv[2];
	*lOpen = test_pvr_eye.lo;
	*rOpen = test_pvr_eye.ro;
	*lPupil = *rPupil = 0;
	*lGuideX = *lGuideY = *lGuideZ = 0;
	*rGuideX = *rGuideY = *rGuideZ = 0;
	*fovGazeX = *fovGazeY = *fovGazeZ = 0;
	*fovGazeState = 0;
	return true;
}
}
