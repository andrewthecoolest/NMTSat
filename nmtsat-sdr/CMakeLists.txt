cmake_minimum_required(VERSION 3.16)
project(nmtsat-sdr C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(Threads REQUIRED)

find_library(RTLSDR_LIB rtlsdr REQUIRED)
find_library(BLADERF_LIB bladeRF REQUIRED)
find_library(MATH_LIB    m      REQUIRED)

# ── Core library (src/) ─────────────────────────────────────────────────────
add_library(nmtsat_core STATIC
    src/sdr.c
    src/blade.c
    src/process.c
)

target_include_directories(nmtsat_core PUBLIC src)

target_link_libraries(nmtsat_core PUBLIC
    ${RTLSDR_LIB}
    ${BLADERF_LIB}
    ${MATH_LIB}
    Threads::Threads
)

# ── Sync test ────────────────────────────────────────────────────────────────
add_executable(nmtsat_sync
    tests/sync_test.c
)

target_link_libraries(nmtsat_sync PRIVATE nmtsat_core)

# ── Longevity monitor ────────────────────────────────────────────────────────
add_executable(nmtsat_longevity
    tests/longevity.c
)

target_link_libraries(nmtsat_longevity PRIVATE nmtsat_core)
