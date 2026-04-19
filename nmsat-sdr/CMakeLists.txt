cmake_minimum_required(VERSION 3.16)
project(nmsat-sdr C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(Threads REQUIRED)

find_library(RTLSDR_LIB rtlsdr REQUIRED)
find_library(BLADERF_LIB bladeRF REQUIRED)
find_library(MATH_LIB    m      REQUIRED)

# ── Core library (src/) ─────────────────────────────────────────────────────
add_library(nmsat_core STATIC
    src/sdr.c
    src/blade.c
    src/process.c
)

target_include_directories(nmsat_core PUBLIC src)

target_link_libraries(nmsat_core PUBLIC
    ${RTLSDR_LIB}
    ${BLADERF_LIB}
    ${MATH_LIB}
    Threads::Threads
)

# ── Test executable (tests/) ────────────────────────────────────────────────
add_executable(nmsat_test
    tests/main.c
)

target_link_libraries(nmsat_test PRIVATE nmsat_core)

# ── Peek utility ─────────────────────────────────────────────────────────────
add_executable(nmsat_peek
    tests/peek_signal.c
)

target_link_libraries(nmsat_peek PRIVATE nmsat_core)
