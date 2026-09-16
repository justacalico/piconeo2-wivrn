#include "input.h"

std::mutex gCtrlMutex;
CtrlState  gCtrl[2];

std::mutex gHeadMutex;
float      gHeadData[7] = {0,0,0,1,0,0,0};

std::mutex    gHapticMutex;
PendingHaptic gHaptic[2];
