#include "app_state.h"
#include "eq_panel.h"   // EQ state folded into the unified config file
#include "log.h"
#include <cstdio>
#include <cstdlib>   // getenv
#include <cstring>   // strncmp, strstr

std::function<void()> gOnExit;

// Moved from settings_panel.h / pin_pad.h
std::atomic<bool> gStreamingMode{false};
int gSettingsCat = 1;  // default to Settings tab
std::atomic<bool> gPinEntryRequested{false};
std::atomic<int>  gDominantHand{1};
std::function<void(const std::string &)> gOnPinSubmit;
std::atomic<bool> gEqGrabbing{false};
std::atomic<int>  gEqActiveBand{-1};

void requestPinEntryUI() { gPinEntryRequested.store(true); }
void submitPin(const std::string &pin) {
    gPinEntryRequested.store(false);
    if (gOnPinSubmit) gOnPinSubmit(pin);
}

std::atomic<float>    gSoftIpdMm{65.0f};
std::atomic<bool>     gIpdDirty{false};
std::atomic<uint64_t> gIpdChangeNs{0};
std::atomic<float>    gSentIpdMm{-1.0f};

std::atomic<bool> gOkClick{false};
std::atomic<bool> gOkHeld{false};
std::atomic<bool> gSideHeld{false};
std::atomic<bool> gManualLobby{false};

std::atomic<bool> gEyeDebugOn{false};
std::atomic<int>  gDiagHudMode{0};
std::atomic<float> gBrightnessFrac{1.0f};
std::atomic<bool>  gBrightnessSaved{false};

std::atomic<bool>   gWivrnTcpOnly{false};
std::atomic<int>    gWivrnCodec{0};
std::atomic<int>    gWivrnFoveation{0};
std::atomic<float>  gWivrnBitrateMbps{50.0f};
std::atomic<float>  gWivrnResolutionScale{1.0f};
std::atomic<bool>   gWivrnHandTracking{false};
std::atomic<bool>   gWivrnEyeTracking{true};
std::atomic<bool>   gWivrnPassthrough{false};
std::atomic<bool>   gWivrnMicrophone{false};
std::atomic<float>  gWivrnCtrlVibration{1.0f};
std::atomic<bool>   gWivrnRecenterReq{false};

std::atomic<bool>   gWivrnEyeFoveation{true};
std::atomic<bool>   gEyeFoveationDirty{false};
std::atomic<bool>   gFoveationEnabled{true};

std::atomic<float> gStreamFovDeg{101.0f};   // full per-eye FOV; 101 = SDK native (no change)
std::atomic<bool>  gFovDirty{false};

std::atomic<bool> gGridThemeDirty{false};
std::atomic<bool> gEyeTrackReapply{false};
std::atomic<bool> gBrightnessApply{false};

std::atomic<int> gVidDecoded{0}, gVidSubmit{0}, gVidDropped{0};
std::atomic<int> gGapMsX10{0}, gRenderMsX10{0}, gEncMsX10{0}, gEnqMsX10{0};
std::atomic<int> gFenceTimeouts{0};   // warp-submit fence-wait timeouts/sec

std::atomic<uint64_t> gBattWarnStartNs{0};
std::atomic<int>      gBattWarnPct{0};

// Unified persistence: one $HOME/config.txt in tagged key=value format with a
// version header. saveAllConfig() rewrites the whole file; saveX() wrappers
// just call it. Unknown keys on load are skipped so new fields can be inserted
// anywhere without breaking older files, and a missing key keeps the default.
static const int kConfigVersion = 2;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static bool configPath(char *out, size_t n) {
    const char *home = getenv("HOME");
    if (!home) return false;
    snprintf(out, n, "%s/config.txt", home);
    return true;
}

void saveAllConfig() {
    char path[512];
    if (!configPath(path, sizeof(path))) return;
    FILE *f = fopen(path, "w");
    if (!f) { LOGE("config: save failed (%s)", path); return; }
    fprintf(f, "version=%d\n", kConfigVersion);
    fprintf(f, "softIpd=%.2f\n", gSoftIpdMm.load());
    fprintf(f, "eyeDebug=%d\n",   gEyeDebugOn.load() ? 1 : 0);
    fprintf(f, "diagHud=%d\n",   gDiagHudMode.load());
    fprintf(f, "brightness=%.3f\n", gBrightnessSaved.load() ? gBrightnessFrac.load() : -1.0f);
    fprintf(f, "eqPreset=%d\n",   gEqPresetIdx);
    fprintf(f, "eqCustom1=");
    for (int i = 0; i < kEqBands; i++) fprintf(f, "%.2f%c", gEqCustoms[0][i], i == kEqBands - 1 ? '\n' : ' ');
    fprintf(f, "eqCustom2=");
    for (int i = 0; i < kEqBands; i++) fprintf(f, "%.2f%c", gEqCustoms[1][i], i == kEqBands - 1 ? '\n' : ' ');
    fprintf(f, "streamFov=%.2f\n", gStreamFovDeg.load());
    fprintf(f, "wivrnTcpOnly=%d\n", gWivrnTcpOnly.load() ? 1 : 0);
    fprintf(f, "wivrnResolutionScale=%.2f\n", gWivrnResolutionScale.load());
    fprintf(f, "wivrnBitrateMbps=%.0f\n", gWivrnBitrateMbps.load());
    fprintf(f, "wivrnMicrophone=%d\n", gWivrnMicrophone.load() ? 1 : 0);
    fprintf(f, "wivrnCtrlVibration=%.2f\n", gWivrnCtrlVibration.load());
    fprintf(f, "wivrnEyeTracking=%d\n", gWivrnEyeTracking.load() ? 1 : 0);
    fprintf(f, "wivrnPassthrough=%d\n", gWivrnPassthrough.load() ? 1 : 0);
    fprintf(f, "wivrnEyeFoveation=%d\n", gWivrnEyeFoveation.load() ? 1 : 0);
    fprintf(f, "wivrnFoveationEnabled=%d\n", gFoveationEnabled.load() ? 1 : 0);
    fclose(f);
}

// One-time migration: read old per-setting files into the live atomics, then
// delete every legacy file (including dead leftovers from removed features).
// Runs only when config.txt is absent. Returns true if any migratable file existed.
static bool migrateLegacyConfig() {
    const char *home = getenv("HOME");
    if (!home) return false;
    char path[512];
    FILE *f;
    bool any = false;
    auto openOld = [&](const char *name) -> FILE * {
        snprintf(path, sizeof(path), "%s/%s", home, name);
        FILE *ff = fopen(path, "r");
        if (ff) any = true;
        return ff;
    };
    if ((f = openOld("software_ipd.txt"))) { float v; if (fscanf(f, "%f", &v) == 1) gSoftIpdMm.store(clampf(v, kIpdMin, kIpdMax)); fclose(f); }
    if ((f = openOld("eye_debug.txt")))    { int v;   if (fscanf(f, "%d", &v) == 1) gEyeDebugOn.store(v != 0); fclose(f); }
    if ((f = openOld("diag_hud.txt")))     { int v;   if (fscanf(f, "%d", &v) == 1) { if (v < 0) v = 0; if (v > 2) v = 2; gDiagHudMode.store(v); } fclose(f); }
    if ((f = openOld("brightness.txt")))   { float v; if (fscanf(f, "%f", &v) == 1) { gBrightnessFrac.store(clampf(v, 0.0f, 1.0f)); gBrightnessSaved.store(true); } fclose(f); }
    if ((f = openOld("eq_profile.txt"))) {   // old format: idx line, then 2x16 gains (one per line)
        int idx = 0; if (fscanf(f, "%d", &idx) != 1) idx = 0; if (idx < 0 || idx >= kEqNumPresets) idx = 0; gEqPresetIdx = idx;
        for (int s = 0; s < kEqNumPresets; s++)
            for (int i = 0; i < kEqBands; i++) { float g; if (fscanf(f, "%f", &g) != 1) break; gEqCustoms[s][i] = clampf(g, -kEqGainMax, kEqGainMax); }
        fclose(f);
    }
    // Sweep away every legacy file: migrated ones plus dead leftovers from
    // removed features (power toggles, decoder buffer A/B, eye_calib).
    static const char *kLegacy[] = {
        "software_ipd.txt", "eye_debug.txt", "diag_hud.txt", "theme.txt", "brightness.txt",
        "eq_profile.txt",
        "maxpower.txt", "adaptive_power.txt", "low_latency.txt", "buffer_frames.txt", "eye_calib.txt",
    };
    for (const char *name : kLegacy) { snprintf(path, sizeof(path), "%s/%s", home, name); remove(path); }
    return any;
}

// Parse a "key=value\n" line. Returns the value pointer (inside `line`) and
// writes the key length to *keyLen, or returns nullptr if no '=' found.
static const char *parseKeyValue(char *line, const char **keyOut, size_t *keyLen) {
    char *eq = strchr(line, '=');
    if (!eq) return nullptr;
    *eq = 0;
    // strip trailing \n/\r from the value
    char *v = eq + 1;
    char *nl = v + strlen(v);
    while (nl > v && (nl[-1] == '\n' || nl[-1] == '\r')) *--nl = 0;
    *keyOut = line;
    *keyLen = eq - line;
    return v;
}

static bool keyIs(const char *key, size_t keyLen, const char *name) {
    return strlen(name) == keyLen && strncmp(key, name, keyLen) == 0;
}

// Parse the legacy positional config.txt (pre-version-2). Reads the same line
// order saveAllConfig used to write, then returns true so the caller re-saves
// in the new key=value format.
static bool loadLegacyPositionalConfig(FILE *f) {
    char ln[1024];
    float fv; int iv;
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%f", &fv) == 1) gSoftIpdMm.store(clampf(fv, kIpdMin, kIpdMax));
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) gEyeDebugOn.store(iv != 0);
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) { if (iv < 0) iv = 0; if (iv > 2) iv = 2; gDiagHudMode.store(iv); }
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%f", &fv) == 1 && fv >= 0.0f) {
        gBrightnessFrac.store(clampf(fv, 0.0f, 1.0f)); gBrightnessSaved.store(true);
    }
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) { if (iv < 0 || iv >= kEqNumPresets) iv = 0; gEqPresetIdx = iv; }
    for (int slot = 0; slot < 2; slot++) {
        if (!fgets(ln, sizeof(ln), f)) break;
        const char *p = ln;
        for (int i = 0; i < kEqBands; i++) {
            float g; int adv = 0;
            if (sscanf(p, " %f%n", &g, &adv) != 1) break;
            gEqCustoms[slot][i] = clampf(g, -kEqGainMax, kEqGainMax);
            p += adv;
        }
    }
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%f", &fv) == 1)
        gStreamFovDeg.store(clampf(fv, kFovMin, kFovMax));
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) gWivrnTcpOnly.store(iv != 0);
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%f", &fv) == 1) gWivrnResolutionScale.store(clampf(fv, 0.5f, 2.0f));
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%f", &fv) == 1) gWivrnBitrateMbps.store(clampf(fv, 5.0f, 200.0f));
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) gWivrnMicrophone.store(iv != 0);
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%f", &fv) == 1) gWivrnCtrlVibration.store(clampf(fv, 0.0f, 1.0f));
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) gWivrnEyeTracking.store(iv != 0);
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) gWivrnPassthrough.store(iv != 0);
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) gWivrnEyeFoveation.store(iv != 0);
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) gFoveationEnabled.store(iv != 0);
    return true;
}

void loadAllConfig() {
    char path[512];
    if (!configPath(path, sizeof(path))) return;
    FILE *f = fopen(path, "r");
    if (!f) {
        // No unified file yet: migrate old per-setting files, then bake into config.txt.
        if (migrateLegacyConfig()) { saveAllConfig(); LOGI("config: migrated legacy files -> %s", path); }
        else                       { LOGI("config: no saved file, using defaults"); }
    } else {
        // Peek the first line: if it starts with "version=" it's the new key=value
        // format, otherwise it's the legacy positional format and we migrate it.
        char first[64] = {0};
        if (!fgets(first, sizeof(first), f)) { fclose(f); LOGI("config: empty file, using defaults"); }
        else if (strncmp(first, "version=", 8) == 0) {
            // key=value format: parse each line, skip unknown keys.
            char ln[1024];
            // first already consumed the version line; read the rest.
            while (fgets(ln, sizeof(ln), f)) {
                const char *key; size_t keyLen;
                const char *v = parseKeyValue(ln, &key, &keyLen);
                if (!v) continue;   // blank or malformed line
                float fv; int iv;
                if      (keyIs(key, keyLen, "softIpd"))              { if (sscanf(v, "%f", &fv) == 1) gSoftIpdMm.store(clampf(fv, kIpdMin, kIpdMax)); }
                else if (keyIs(key, keyLen, "eyeDebug"))             { if (sscanf(v, "%d", &iv) == 1) gEyeDebugOn.store(iv != 0); }
                else if (keyIs(key, keyLen, "diagHud"))              { if (sscanf(v, "%d", &iv) == 1) { if (iv < 0) iv = 0; if (iv > 2) iv = 2; gDiagHudMode.store(iv); } }
                else if (keyIs(key, keyLen, "brightness"))           { if (sscanf(v, "%f", &fv) == 1 && fv >= 0.0f) { gBrightnessFrac.store(clampf(fv, 0.0f, 1.0f)); gBrightnessSaved.store(true); } }
                else if (keyIs(key, keyLen, "eqPreset"))             { if (sscanf(v, "%d", &iv) == 1) { if (iv < 0 || iv >= kEqNumPresets) iv = 0; gEqPresetIdx = iv; } }
                else if (keyIs(key, keyLen, "eqCustom1") || keyIs(key, keyLen, "eqCustom2")) {
                    int slot = (key[keyLen - 1] == '2') ? 1 : 0;
                    const char *p = v;
                    for (int i = 0; i < kEqBands; i++) {
                        float g; int adv = 0;
                        if (sscanf(p, " %f%n", &g, &adv) != 1) break;
                        gEqCustoms[slot][i] = clampf(g, -kEqGainMax, kEqGainMax);
                        p += adv;
                    }
                }
                else if (keyIs(key, keyLen, "streamFov"))            { if (sscanf(v, "%f", &fv) == 1) gStreamFovDeg.store(clampf(fv, kFovMin, kFovMax)); }
                else if (keyIs(key, keyLen, "wivrnTcpOnly"))         { if (sscanf(v, "%d", &iv) == 1) gWivrnTcpOnly.store(iv != 0); }
                else if (keyIs(key, keyLen, "wivrnResolutionScale")) { if (sscanf(v, "%f", &fv) == 1) gWivrnResolutionScale.store(clampf(fv, 0.5f, 2.0f)); }
                else if (keyIs(key, keyLen, "wivrnBitrateMbps"))     { if (sscanf(v, "%f", &fv) == 1) gWivrnBitrateMbps.store(clampf(fv, 5.0f, 200.0f)); }
                else if (keyIs(key, keyLen, "wivrnMicrophone"))      { if (sscanf(v, "%d", &iv) == 1) gWivrnMicrophone.store(iv != 0); }
                else if (keyIs(key, keyLen, "wivrnCtrlVibration"))   { if (sscanf(v, "%f", &fv) == 1) gWivrnCtrlVibration.store(clampf(fv, 0.0f, 1.0f)); }
                else if (keyIs(key, keyLen, "wivrnEyeTracking"))     { if (sscanf(v, "%d", &iv) == 1) gWivrnEyeTracking.store(iv != 0); }
                else if (keyIs(key, keyLen, "wivrnPassthrough"))     { if (sscanf(v, "%d", &iv) == 1) gWivrnPassthrough.store(iv != 0); }
                else if (keyIs(key, keyLen, "wivrnEyeFoveation"))    { if (sscanf(v, "%d", &iv) == 1) gWivrnEyeFoveation.store(iv != 0); }
                else if (keyIs(key, keyLen, "wivrnFoveationEnabled")){ if (sscanf(v, "%d", &iv) == 1) gFoveationEnabled.store(iv != 0); }
                // unknown key -> skip (forward-compatible)
            }
            fclose(f);
            LOGI("config: loaded %s (v%d key=value)", path, kConfigVersion);
        } else {
            // Legacy positional format: rewind and parse the old way, then
            // re-save in the new format so future loads are tagged.
            fclose(f);
            f = fopen(path, "r");
            if (f) {
                loadLegacyPositionalConfig(f);
                fclose(f);
                saveAllConfig();
                LOGI("config: migrated legacy positional file -> %s (v%d)", path, kConfigVersion);
            }
        }
    }
    // mirror the active EQ slot into the live gain set
    if (gEqPresetIdx < 0 || gEqPresetIdx >= kEqNumPresets) gEqPresetIdx = 0;
    for (int i = 0; i < kEqBands; i++) gEqGains[i] = gEqCustoms[gEqPresetIdx][i];
}

// Thin wrappers: every settings change rewrites the whole config.txt.
void saveSoftIpd()    { saveAllConfig(); }
void saveEyeDebug()   { saveAllConfig(); }
void saveDiagHud()    { saveAllConfig(); }
void saveStreamFov()  { saveAllConfig(); }
void saveBrightness() { gBrightnessSaved.store(true); saveAllConfig(); }
