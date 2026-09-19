# Pico Neo 2 WiVRn

Native WiVRn client for the Pico Neo 2 family. Stream PC VR games to your headset over Wi-Fi or USB with low latency.

> [!NOTE]
> This project is in **Beta**. It works but expect some rough edges.

## Supported headsets

| Headset | Status |
|---------|--------|
| Pico Neo 2 | Working |
| Pico Neo 2 Eye | Untested |
| Pico G2 | Untested |
| Pico Neo Lite | Working |

## Features

- 6DOF and 3DOF head tracking
- Controller inputs with 3D model rendering
- Head-gaze laser interaction for the lobby UI (no controllers needed)
- Audio streaming
- Overlay application support

## WiVRn Version

This client is compatible with WiVRn server version **26.9**.

## Prerequisites

### Get the Pico SDK

The Pico Neo 2 SDK is not bundled due to licensing. A helper script downloads and extracts it from the [Internet Archive](https://archive.org/details/pico-neo-2-sdks-exes.-7z) bundle.

```bash
./tools/fetch_pico_sdk.sh
```

### Environment

- Android SDK with NDK 26.3.11579264
- JDK 17
- CMake 3.22.1
- Pico Neo 2 with USB debugging enabled

## Building

```bash
export ANDROID_HOME=/path/to/Android/sdk
export ANDROID_SDK_ROOT=$ANDROID_HOME
export ANDROID_NDK_ROOT=$ANDROID_HOME/ndk/26.3.11579264
export JAVA_HOME=/path/to/jdk-17
export PATH="$JAVA_HOME/bin:$PATH"

./tools/fetch_pico_sdk.sh
cd src
./gradlew assembleDebug
```

The APK will be in `app/build/outputs/apk/debug/` (or `release/` for signed release builds).

### Signed release builds

Release signing is configured via `keystore.properties` placed one directory above the gradle project (i.e. at the repo root):

```properties
storeFile=../pico2-wivrn-release.jks
storePassword=your_store_password
keyAlias=pico2-wivrn
keyPassword=your_key_password
```

The keystore file path is relative to `src/`. If the file is absent, release builds are unsigned.

## Installing

```bash
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

Or grab a pre-built signed APK from the [GitLab releases](https://gitlab.com/HttpAnimations/piconeo2-wivrn/-/releases).

## Usage

1. Install the WiVRn server on your PC ([WiVRn server setup](https://github.com/WiVRn/WiVRn))
2. Launch the app on your headset
3. Pair with your PC by entering the PIN shown on screen
4. Stream VR games over Wi-Fi or USB

## Community

Got questions or want to contribute? Join the Discord: [https://discord.gg/RQ9nSpmtfU](https://discord.gg/RQ9nSpmtfU)

Listed on [NeoRevived](https://openlyst.github.io/neorevived/#/entry/streaming/gitlab.HttpAnimations.piconeo2-wivrn).

## Acknowledgments

- [WiVRn](https://github.com/WiVRn/WiVRn) | The upstream project this client is based on
- [ALVR Pico Legacy](https://github.com/Juspertinry/alvr-pico-legacy) | Rendering reference for the Pico Neo 2
- [Pico SDK](https://developer.picoxr.com/) | Pico Neo 2 SDK
