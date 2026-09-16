#pragma once
// Stream swapchain state: the WiVRn stream adapter blits decoded frames into
// the same ring the render thread submits to the SDK warp.
#include <GLES3/gl3.h>
#include <stdint.h>

extern GLuint gSwap[2][5];
extern GLuint gStreamFbo;
extern int    gSwapIdx;
extern uint32_t gStreamW;
extern uint32_t gStreamH;
