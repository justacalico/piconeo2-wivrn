#pragma once
// Lobby HUD device strings: LAN IPv4, client status, server hostname, headset model.
#include <jni.h>

extern char gIpText[24];        // "Check Wi-Fi" until a LAN address is found
extern char gStatusText[32];    // Disconnected / Searching / Connecting / Connected
extern char gModelText[32];     // e.g. "Pico Neo 2 Eye"
extern char gHostnameText[24];  // connected server hostname
extern bool gIsEyeHw;           // true on Neo 2 EYE (pxr.vendorhw.eye == 1)

void toUpperAscii(char *s);
void readHeadsetModel(JNIEnv *env);
void refreshDeviceIp();
