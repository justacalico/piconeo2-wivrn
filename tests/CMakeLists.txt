cmake_minimum_required(VERSION 3.22)
project(p2wivrn_tests CXX)

# Host-side unit tests for the parts of the client that don't need the Pico
# SDK / GLES / JNI. Everything under src/app/src/main/cpp that is portable gets
# compiled into libp2wivrn_testable with coverage instrumentation; the actual
# test cases live next to this file.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src/app/src/main/cpp)
set(WIVRN_DIR ${CPP_DIR}/wivrn)
set(THIRD_PARTY_DIR ${CPP_DIR}/third_party)

find_package(OpenSSL REQUIRED)
# Only the headers and libjvm are needed; AWT is absent from headless JDKs.
find_package(JNI REQUIRED COMPONENTS JVM)
find_package(Threads REQUIRED)

# The app build generates this at configure time; same contents here.
file(WRITE ${CMAKE_BINARY_DIR}/version.cpp
    "#include \"version.h\"\n"
    "namespace wivrn {\n"
    "const char git_commit[] = \"unknown\";\n"
    "const bool is_tag = false;\n"
    "const char * display_version() { return \"standalone\"; }\n"
    "}\n"
)

# Sources under test: every cpp that builds without Pico SDK / GL / real JNI.
add_library(p2wivrn_testable STATIC
    ${WIVRN_DIR}/common/crypto.cpp
    ${WIVRN_DIR}/common/smp.cpp
    ${WIVRN_DIR}/common/secrets.cpp
    ${WIVRN_DIR}/common/wivrn_sockets.cpp
    ${WIVRN_DIR}/net/wivrn_client_pico.cpp
    ${CPP_DIR}/core/app_state.cpp
    ${CPP_DIR}/core/device_info.cpp
    ${CPP_DIR}/core/eye_tracking.cpp
    ${CPP_DIR}/input/input.cpp
    ${CPP_DIR}/ui/ui_kit.cpp
    ${CPP_DIR}/ui/eq_panel.cpp
    ${CPP_DIR}/ui/server_list.cpp
    ${CPP_DIR}/render/menu_model.cpp
    ${CPP_DIR}/streaming/pvr_eye_bridge.cpp
    ${CMAKE_BINARY_DIR}/version.cpp
)

target_compile_options(p2wivrn_testable PUBLIC
    -O0 -g --coverage
    -fconstexpr-ops-limit=268435456 -fconstexpr-depth=2048
    "-include${CPP_DIR}/core/openxr_ext_stubs.h"
    "-include${WIVRN_DIR}/common/openxr_body_compat.h"
)

target_include_directories(p2wivrn_testable PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/shims   # <android/log.h>, <sys/system_properties.h>
    ${CMAKE_CURRENT_SOURCE_DIR}          # test helpers + stub declarations
    ${CPP_DIR}
    ${CPP_DIR}/core
    ${CPP_DIR}/input
    ${CPP_DIR}/render
    ${CPP_DIR}/streaming
    ${CPP_DIR}/ui
    ${WIVRN_DIR}
    ${WIVRN_DIR}/common
    ${WIVRN_DIR}/core
    ${WIVRN_DIR}/net
    ${WIVRN_DIR}/tracking
    ${CMAKE_BINARY_DIR}                  # wivrn_config.h / version.cpp location
    ${JNI_INCLUDE_DIRS}
)

target_include_directories(p2wivrn_testable SYSTEM PUBLIC
    ${THIRD_PARTY_DIR}
    ${THIRD_PARTY_DIR}/magic_enum
)

target_compile_definitions(p2wivrn_testable PUBLIC
    SPDLOG_FMT_EXTERNAL FMT_HEADER_ONLY
    VMA_VULKAN_VERSION=1001000 VULKAN_HPP_NO_STRUCT_CONSTRUCTORS
)

target_link_libraries(p2wivrn_testable PUBLIC
    OpenSSL::Crypto
    Threads::Threads
    gcov
)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)

add_executable(p2wivrn_tests ${TEST_SOURCES} main.cpp stubs.cpp pvr_stubs.cpp openssl_wrap.cpp libc_wrap.cpp)
target_link_libraries(p2wivrn_tests PRIVATE p2wivrn_testable)

# Route the OpenSSL calls under test through the fault-injection wrappers.
foreach(sym
        BIO_new BIO_new_mem_buf
        PEM_read_bio_PUBKEY PEM_read_bio_PrivateKey
        PEM_write_bio_PUBKEY PEM_write_bio_PrivateKey
        EVP_PKEY_CTX_new EVP_PKEY_CTX_new_id
        EVP_PKEY_keygen_init EVP_PKEY_keygen
        EVP_PKEY_CTX_set_rsa_keygen_bits EVP_PKEY_CTX_set_kem_op
        EVP_PKEY_derive_init EVP_PKEY_derive_set_peer EVP_PKEY_derive
        EVP_PKEY_encapsulate_init EVP_PKEY_encapsulate
        EVP_PKEY_decapsulate_init EVP_PKEY_decapsulate
        EVP_CIPHER_CTX_new EVP_CipherInit_ex2
        EVP_EncryptInit_ex2 EVP_EncryptUpdate EVP_EncryptFinal_ex
        EVP_DecryptInit EVP_DecryptInit_ex2 EVP_DecryptUpdate EVP_DecryptFinal_ex
        EVP_KDF_fetch EVP_KDF_CTX_new EVP_KDF_derive
        BN_new BN_CTX_new BN_dup BN_mpi2bn BN_bn2mpi BN_bn2bin BN_bn2hex
        BN_bin2bn BN_hex2bn BN_sub BN_mod_exp BN_mod_mul BN_mod_sub
        BN_mod_inverse BN_rand BN_set_word
        EVP_MD_CTX_new EVP_DigestInit EVP_DigestUpdate EVP_DigestFinal
        socket bind connect listen accept setsockopt
        recvfrom recv recvmmsg sendmsg sendmmsg writev poll
        opendir fopen
        sched_setscheduler sched_setaffinity sched_getaffinity
        setpriority sysconf)
    target_link_options(p2wivrn_tests PRIVATE "-Wl,--wrap=${sym}")
endforeach()
target_compile_options(p2wivrn_tests PRIVATE -O0 -g --coverage)

enable_testing()
add_test(NAME p2wivrn_tests COMMAND p2wivrn_tests)
set_tests_properties(p2wivrn_tests PROPERTIES TIMEOUT 600)

# Coverage report over the library sources only (not the tests themselves).
find_program(GCOV gcov REQUIRED)
add_custom_target(coverage
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/coverage.sh ${CMAKE_BINARY_DIR} ${CPP_DIR}
    DEPENDS p2wivrn_tests
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Per-file line coverage of tested sources"
)
