# Contributing

Thanks for wanting to help. This doc covers the bits that aren't obvious from reading the code.

## Dev environment

Same as the README's Prerequisites section:

- Android SDK with NDK **26.3.11579264**
- JDK **17**
- CMake **3.22.1**
- The Pico Neo 2 native SDK `.so` files in `src/app/src/main/jniLibs/armeabi-v7a/` and `pvr_classes.jar` in `src/app/libs/` (see README - the build fails with a clear message if these are missing)

```bash
export ANDROID_HOME=/path/to/Android/sdk
export ANDROID_SDK_ROOT=$ANDROID_HOME
cd src
./gradlew assembleDebug
```

APK ends up in `app/build/outputs/apk/debug/`.

## Build variants

`assembleDebug` and `assembleRelease` both produce an optimized build. The CMakeLists forces `-O2 -DNDEBUG` regardless of Gradle variant because the compositor is perf-critical and a default `-O0` debug build is unusable on the headset. If you need a true unoptimized build for stepping in a debugger, override `CMAKE_CXX_FLAGS_DEBUG` locally and don't commit the change.

Release builds are signed only when `keystore.properties` exists one directory above `src/` (see README). Without it, `assembleRelease` produces an unsigned APK.

## Code layout

```
src/app/src/main/cpp/
├── wivrn/              vendored WiVRn shared sources (audio, video, net, tracking, render, core)
├── streaming/          WiVRn adapter for the Pico compositor:
│                       streaming_client.cpp      - connection + decoder lifecycle
│                       pvr_renderer.cpp          - eye-texture blit into the warp swapchain
│                       wivrn_stream_adapter.cpp  - thin glue between wivrn/ and the render loop
├── render_thread.cpp   the main compositor + render loop (~3600 lines, the heart of the client)
├── app_state.{cpp,h}   persisted settings + cross-thread atomic knobs
├── ui_kit.{cpp,h}      immediate-mode lobby UI widgets (box, text, button, toggle, fader)
├── settings_panel.cpp  the SETTINGS window (data-driven menu of toggles/sliders)
├── lobby.cpp           ported WiVRn lobby UI panel rendered to a texture
├── passthrough.cpp     Camera2 NDK passthrough background for the lobby
├── eye_tracking.cpp    Tobii/Pico eye tracking bridge
├── device_info.cpp     headset model + IP strings for the lobby HUD
├── input.cpp           controller + head-pose shared state
├── eq_panel.cpp        16-band audio EQ panel
├── pico_sdk.h          Pico SDK function declarations (the SDK ships no header)
└── third_party/        boost, fmt, magic_enum, spdlog (header-only)
```

The render loop talks to WiVRn directly through `wivrn_stream_adapter.h` and `streaming_client.h`. There is no compatibility layer; call the WiVRn APIs from the render thread as needed.

## Adding a setting

1. Declare an `std::atomic<T> gWivrnFoo{default}` in `app_state.h` (atomic because most settings are read from the render loop and written from the JNI input handler).
2. Add a `key=value` line to `saveAllConfig()` in `app_state.cpp`.
3. Add a `keyIs(key, keyLen, "wivrnFoo")` branch to `loadAllConfig()` that parses the value into the atomic.
4. Add a UI control in `settings_panel.cpp` (a `uiToggle` / `uiHFader` / `uiDropdownItem`). The settings panel is data-driven - look at how `wivrnToggle("PASSTHROUGH", gWivrnPassthrough)` is wired.
5. Bump `kConfigVersion` in `app_state.cpp` if you change the file format in a way that should invalidate older files. Adding a new key does not need a bump - unknown keys are skipped on load and missing keys keep the default.

The config file is `$HOME/config.txt` on the headset, tagged `key=value` with a `version=N` header. Old positional-format files auto-migrate on first load.

## Adding a lobby UI element

The lobby UI is an immediate-mode layer on top of `ui_kit`. Each frame, the render loop rebuilds vertex buffers from the live state:

- `uiBox(v, rect, color)` - filled quad
- `uiTextC(v, s, cx, yTop, px, ...)` / `uiTextL(...)` - centered / left-anchored text
- `uiButton(v, rect, label, hot, disabled)` - button (hot = hovered/selected)
- `uiToggle(v, rect, label, on, hot, ...)` - toggle row
- `uiHFader(v, rect, frac, hot, ...)` - horizontal slider
- `uiDropdownHeader` / `uiDropdownItem` - collapsible section

`v` is a `std::vector<float>` of interleaved position + color + UV verts that gets uploaded to a VBO and drawn with `gProg` at the end of the frame. Hit-testing is `uiHit(rect, lx, ly)` against the controller/head-gaze pointer position. Look at `settings_panel.cpp` and `eq_panel.cpp` for working examples.

## Branches and merges

Feature branches off `main`, named by area: `fix/...`, `cleanup/...`, `docs/...`, `feature/...`. Merge via GitLab MR with a merge commit (not squash). Keep branches short-lived.

## Testing

There are no automated tests today. You test by building the APK, installing on the headset (`adb install -r app/build/outputs/apk/debug/app-debug.apk`), and exercising the change. `adb logcat | grep wivrn` is the main feedback loop.

When automated tests eventually land they'll be gated behind `WIVRN_BUILD_TEST=ON` so they don't pollute the headset build.
