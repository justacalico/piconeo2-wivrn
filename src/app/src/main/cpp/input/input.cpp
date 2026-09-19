#include "input.h"

std::mutex gCtrlMutex;
CtrlState  gCtrl[2];

std::mutex gHeadMutex;
float      gHeadData[7] = {0,0,0,1,0,0,0};

std::mutex    gHapticMutex;
PendingHaptic gHaptic[2];

void queueHaptic(int hand, float amplitude, int64_t duration_ns) {
    if (hand < 0 || hand > 1 || amplitude <= 0.0f || duration_ns <= 0)
        return;
    if (amplitude > 1.0f)
        amplitude = 1.0f;
    int ms = static_cast<int>(duration_ns / 1000000);
    if (ms < 12)   ms = 12;
    if (ms > 1000) ms = 1000;

    std::lock_guard<std::mutex> lk(gHapticMutex);
    PendingHaptic &p = gHaptic[hand];
    if (!p.pending || amplitude > p.amplitude)
        p.amplitude = amplitude;
    if (!p.pending || ms > p.durationMs)
        p.durationMs = ms;
    p.pending = true;
}
