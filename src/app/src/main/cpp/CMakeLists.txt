cmake_minimum_required(VERSION 3.22.1)
project(p2wivrn)

# C++20 is required by the WiVRn shared sources.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This is a perf-critical VR client. The Gradle `debug` variant otherwise compiles
# the native lib at -O0 (CMAKE_*_FLAGS_DEBUG default) which leaves the whole ~6k-line
# compositor unoptimized. Force -O2 -DNDEBUG regardless of variant. -DNDEBUG also
# compiles out the verbose #ifndef NDEBUG dev logging in jni.cpp.
set(CMAKE_C_FLAGS_DEBUG   "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -DNDEBUG")

find_library(log-lib log)
find_library(egl-lib EGL)
find_library(gles-lib GLESv3)
find_library(android-lib android)
find_library(jnigraphics-lib jnigraphics)
find_library(media-lib mediandk)
find_library(camera2-lib camera2ndk)
find_library(aaudio-lib aaudio)

# Import the prebuilt Pico SDK native library so we can link against its
# exported C symbols (Pvr_Init, Pvr_GetMainSensorState, GetRenderEventFunc, ...).
add_library(Pvr_UnitySDK SHARED IMPORTED)
set_target_properties(Pvr_UnitySDK PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../jniLibs/${ANDROID_ABI}/libPvr_UnitySDK.so)

# Self-contained WiVRn/PVR client. All shared sources are vendored under
# wivrn/ so this project is self-contained.
set(WIVRN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/wivrn)
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# spdlog / fmt are header-only and vendored here.
set(SPDLOG_INCLUDE_DIR ${THIRD_PARTY_DIR})
set(BOOST_INCLUDE_DIR ${THIRD_PARTY_DIR})

# OpenSSL for the WiVRn crypto handshake.
set(OPENSSL_INSTALL_DIR ${CMAKE_BINARY_DIR}/openssl-install)
set(OPENSSL_INCLUDE_DIR ${OPENSSL_INSTALL_DIR}/include)
set(OPENSSL_CRYPTO_LIBRARY ${OPENSSL_INSTALL_DIR}/lib/libcrypto.a)

string(REGEX REPLACE "android-([0-9]+)" "\\1" ANDROID_API_LEVEL ${ANDROID_PLATFORM})
set(NDK_TOOLCHAIN ${ANDROID_NDK}/toolchains/llvm/prebuilt/${ANDROID_HOST_TAG})
set(NDK_CC ${NDK_TOOLCHAIN}/bin/armv7a-linux-androideabi${ANDROID_API_LEVEL}-clang)
include(ExternalProject)
ExternalProject_Add(openssl_ext
    URL https://github.com/openssl/openssl/releases/download/openssl-3.4.1/openssl-3.4.1.tar.gz
    URL_HASH SHA256=002a2d6b30b58bf4bea46c43bdd96365aaf8daa6c428782aa4feee06da197df3
    CONFIGURE_COMMAND
        ${CMAKE_COMMAND} -E env
        ANDROID_NDK_ROOT=${ANDROID_NDK}
        ANDROID_NDK_HOME=${ANDROID_NDK}
        PATH=${NDK_TOOLCHAIN}/bin:$ENV{PATH}
        ./Configure android-arm no-shared no-tests no-asm
        --prefix=${OPENSSL_INSTALL_DIR}
        -D__ANDROID_API__=${ANDROID_API_LEVEL}
    BUILD_COMMAND make -j${CMAKE_BUILD_PARALLEL_LEVEL}
        CROSS_COMPILE=
        CC=${NDK_CC}
        AR=${NDK_TOOLCHAIN}/bin/llvm-ar
        RANLIB=${NDK_TOOLCHAIN}/bin/llvm-ranlib
    INSTALL_COMMAND make install_sw
    BUILD_IN_SOURCE true
    BUILD_BYPRODUCTS ${OPENSSL_CRYPTO_LIBRARY}
)
file(MAKE_DIRECTORY ${OPENSSL_INCLUDE_DIR})
add_library(OpenSSL::Crypto STATIC IMPORTED)
set_target_properties(OpenSSL::Crypto PROPERTIES
    IMPORTED_LOCATION ${OPENSSL_CRYPTO_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${OPENSSL_INCLUDE_DIR}
)
add_dependencies(OpenSSL::Crypto openssl_ext)

# Vulkan headers (needed by wivrn-common).
set(VULKAN_INCLUDE_DIR /usr/include)

# --- wivrn-common (crypto, pairing, sockets) ---
configure_file(${WIVRN_DIR}/common/wivrn_config.h.in ${CMAKE_BINARY_DIR}/wivrn_config.h)

file(WRITE ${CMAKE_BINARY_DIR}/version.cpp
    "#include \"version.h\"\n"
    "namespace wivrn {\n"
    "const char git_commit[] = \"unknown\";\n"
    "const bool is_tag = false;\n"
    "const char * display_version() { return \"standalone\"; }\n"
    "}\n"
)

add_library(wivrn-common STATIC EXCLUDE_FROM_ALL
    ${WIVRN_DIR}/common/crypto.cpp
    ${WIVRN_DIR}/common/smp.cpp
    ${WIVRN_DIR}/common/secrets.cpp
    ${WIVRN_DIR}/common/wivrn_sockets.cpp
    ${CMAKE_BINARY_DIR}/version.cpp
)
target_include_directories(wivrn-common PUBLIC
    ${WIVRN_DIR}/common
    ${CMAKE_BINARY_DIR}
    ${WIVRN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${THIRD_PARTY_DIR}/magic_enum
)
target_include_directories(wivrn-common SYSTEM PUBLIC ${VULKAN_INCLUDE_DIR} ${SPDLOG_INCLUDE_DIR} ${BOOST_INCLUDE_DIR})
target_compile_definitions(wivrn-common PUBLIC VMA_VULKAN_VERSION=1001000 VULKAN_HPP_NO_STRUCT_CONSTRUCTORS SPDLOG_FMT_EXTERNAL FMT_HEADER_ONLY)
target_compile_options(wivrn-common PUBLIC -fconstexpr-steps=268435456 -fconstexpr-depth=2048)
target_compile_options(wivrn-common PUBLIC "-include${CMAKE_CURRENT_SOURCE_DIR}/core/openxr_ext_stubs.h" "-include${WIVRN_DIR}/common/openxr_body_compat.h")
target_link_libraries(wivrn-common PUBLIC OpenSSL::Crypto)

# --- p2wivrn (PVR client with WiVRn streaming) ---
add_library(p2wivrn SHARED
    core/jni.cpp
    core/app_state.cpp
    core/device_info.cpp
    core/eye_tracking.cpp
    render/render_thread.cpp
    render/gl_util.cpp
    render/passthrough.cpp
    render/menu_model.cpp
    render/lobby.cpp
    input/input.cpp
    ui/ui_kit.cpp
    ui/eq_panel.cpp
    ui/server_list.cpp
    ui/simple_lobby.cpp
    ui/android_ui.cpp
    streaming/streaming_client.cpp
    streaming/pvr_renderer.cpp
    streaming/pvr_eye_bridge.cpp
    streaming/wivrn_stream_adapter.cpp
    ${WIVRN_DIR}/net/wivrn_client_pico.cpp
    ${WIVRN_DIR}/video/pico_decoder.cpp
    ${WIVRN_DIR}/audio/pico_audio.cpp
    ${WIVRN_DIR}/render/pico_blit.cpp
    ${WIVRN_DIR}/tracking/pico_tracking.cpp
)

# Thin LTO on OUR translation units + link only (imported prebuilt libs are
# unaffected -- they're IMPORTED, never compiled here).
target_compile_options(p2wivrn PRIVATE -flto=thin)
target_link_options(p2wivrn PRIVATE -flto=thin)

target_include_directories(p2wivrn PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/render
    ${CMAKE_CURRENT_SOURCE_DIR}/input
    ${CMAKE_CURRENT_SOURCE_DIR}/ui
    ${CMAKE_CURRENT_SOURCE_DIR}/streaming
    ${CMAKE_CURRENT_SOURCE_DIR}/openxr
    ${WIVRN_DIR}
    ${WIVRN_DIR}/core
    ${WIVRN_DIR}/net
    ${WIVRN_DIR}/video
    ${WIVRN_DIR}/audio
    ${WIVRN_DIR}/render
    ${WIVRN_DIR}/tracking
    ${WIVRN_DIR}/common
    ${CMAKE_BINARY_DIR}
)

target_include_directories(p2wivrn SYSTEM PRIVATE ${SPDLOG_INCLUDE_DIR} ${BOOST_INCLUDE_DIR} ${THIRD_PARTY_DIR})

target_compile_definitions(p2wivrn PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG SPDLOG_FMT_EXTERNAL FMT_HEADER_ONLY)

target_link_libraries(p2wivrn
    Pvr_UnitySDK
    wivrn-common
    ${gles-lib}
    ${egl-lib}
    ${android-lib}
    ${jnigraphics-lib}
    ${media-lib}
    ${camera2-lib}
    ${aaudio-lib}
    ${log-lib})
