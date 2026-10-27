cmake_minimum_required(VERSION 3.16)
project(BulletNEXT LANGUAGES CXX)

# Bullet NEXT must stay C++14 so it can be wired into the 2.79b engine.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Embedded mode (F1, C1.11): added to the game engine with add_subdirectory(). Only the two
# backends are built there; the engine's build type and flags are left alone.
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(BN_EMBEDDED OFF)
else()
  set(BN_EMBEDDED ON)
endif()

if(NOT BN_EMBEDDED AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Strict floating point: no fast-math, no FMA contraction (determinism rule).
if(MSVC)
  add_compile_options(/fp:precise /W3 /wd4244 /wd4267 /wd4305 /wd4996)
else()
  add_compile_options(-fno-fast-math -ffp-contract=off -w)
endif()

if(BN_EMBEDDED)
  set(_bn_dev OFF)
else()
  set(_bn_dev ON)
endif()
option(BN_BUILD_ORIG "Build the unrenamed Bullet 2.84 (Blender 2.79b) copy for fidelity tests" ${_bn_dev})
option(BN_BUILD_LEGACY "Build the frozen, renamed legacy backend" ON)
option(BN_BUILD_NEXT "Build Bullet NEXT (next/, upstream base + F1 changes)" ON)
option(BN_BUILD_UPSTREAM "Build pristine upstream for comparison" ${_bn_dev})
option(BN_BUILD_HARNESS "Build the golden harness" ${_bn_dev})

function(bn_add_bullet_lib target root)
  if(EXISTS "${root}/SOURCES.txt")
    file(STRINGS "${root}/SOURCES.txt" _rel)
    set(_srcs "")
    foreach(f ${_rel})
      list(APPEND _srcs "${root}/${f}")
    endforeach()
  else()
    file(GLOB_RECURSE _srcs
      "${root}/LinearMath/*.cpp"
      "${root}/BulletCollision/*.cpp"
      "${root}/BulletDynamics/*.cpp"
      "${root}/BulletSoftBody/*.cpp")
    list(FILTER _srcs EXCLUDE REGEX ".*All\\.cpp$")
  endif()
  add_library(${target} STATIC ${_srcs})
  target_include_directories(${target} PUBLIC "${root}")
endfunction()

if(BN_BUILD_ORIG)
  bn_add_bullet_lib(bn_bullet284_orig "${CMAKE_CURRENT_SOURCE_DIR}/third_party/bullet284_orig")
endif()
if(BN_BUILD_LEGACY AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/legacy/SOURCES.txt")
  bn_add_bullet_lib(bn_bullet_legacy "${CMAKE_CURRENT_SOURCE_DIR}/legacy")
endif()
if(BN_BUILD_NEXT)
  bn_add_bullet_lib(bn_bullet_next "${CMAKE_CURRENT_SOURCE_DIR}/next")
  # F1 C1.9: thread-safe build with Bullet's own task scheduler (no OpenMP/TBB dependency).
  target_compile_definitions(bn_bullet_next PUBLIC BT_THREADSAFE=1)
  if(NOT EMSCRIPTEN)  # the browser build has one thread
    find_package(Threads REQUIRED)
    target_link_libraries(bn_bullet_next PUBLIC Threads::Threads)
  endif()
  if(EMSCRIPTEN)
    # Browser build (FL, CL.10): Bullet NEXT's SSE code through WebAssembly SIMD, as on the PC.
    # PUBLIC, so every file that includes Bullet NEXT sees the same btVector3.
    target_compile_options(bn_bullet_next PUBLIC -msimd128 -msse2)
  endif()
endif()
# Portable float math (FL, CL.10): Bullet NEXT's sin, cos, atan2, exp, log and pow come from its
# own copy of musl's math, so a scene gives the same results on Windows, Linux and in the browser.
option(BN_PORTABLE_MATH "Bullet NEXT: the same float math on every system (musl copy)" ON)
if(BN_BUILD_NEXT AND BN_PORTABLE_MATH)
  enable_language(C)
  file(GLOB _bn_musl "${CMAKE_CURRENT_SOURCE_DIR}/third_party/musl_math/*.c")
  add_library(bn_musl_math STATIC ${_bn_musl})
  set_target_properties(bn_musl_math PROPERTIES C_STANDARD 99 C_EXTENSIONS ON)
  target_compile_definitions(bn_bullet_next PUBLIC BN_PORTABLE_MATH)
  target_link_libraries(bn_bullet_next PUBLIC bn_musl_math)
endif()
if(BN_BUILD_NEXT)
  # Bullet's .bullet file reader (upstream Extras/Serialize, same pinned commit) on Bullet NEXT,
  # for programs that load serialized collision shapes (HARFANG's mesh colliders, FL).
  set(_ser "${CMAKE_CURRENT_SOURCE_DIR}/third_party/bullet3_master/Extras/Serialize")
  add_library(bn_bullet_next_serialize STATIC
    ${_ser}/BulletFileLoader/bChunk.cpp ${_ser}/BulletFileLoader/bDNA.cpp ${_ser}/BulletFileLoader/bFile.cpp
    ${_ser}/BulletFileLoader/btBulletFile.cpp
    ${_ser}/BulletWorldImporter/btBulletWorldImporter.cpp ${_ser}/BulletWorldImporter/btWorldImporter.cpp
    ${_ser}/BulletWorldImporter/btMultiBodyWorldImporter.cpp)
  target_include_directories(bn_bullet_next_serialize PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/third_party/bullet3_master/Extras")
  target_link_libraries(bn_bullet_next_serialize PUBLIC bn_bullet_next)
endif()
# Double precision (FL, host profile blender53-rb): Blender 5.3 builds its Bullet copy with
# BT_USE_DOUBLE_PRECISION, so Bullet NEXT's place in Blender's rigid body world is double too.
option(BN_BUILD_DOUBLE "Build Blender 5.3's Bullet copy and Bullet NEXT in double precision" ${_bn_dev})
if(BN_BUILD_DOUBLE)
  bn_add_bullet_lib(bn_bullet_blender53 "${CMAKE_CURRENT_SOURCE_DIR}/third_party/blender53/bullet2")
  target_compile_definitions(bn_bullet_blender53 PUBLIC BT_USE_DOUBLE_PRECISION)
  if(BN_BUILD_NEXT)
    bn_add_bullet_lib(bn_bullet_nextd "${CMAKE_CURRENT_SOURCE_DIR}/next")
    target_compile_definitions(bn_bullet_nextd PUBLIC BT_USE_DOUBLE_PRECISION BT_THREADSAFE=1)
    target_link_libraries(bn_bullet_nextd PUBLIC Threads::Threads)
  endif()
endif()
if(BN_BUILD_UPSTREAM)
  bn_add_bullet_lib(bn_bullet_upstream "${CMAKE_CURRENT_SOURCE_DIR}/third_party/bullet3_master")
endif()

# The Lab (FL): scene files, the Lab engines and tools. On by default when Bullet NEXT is built on
# its own; a host that wants it (the HARFANG Lab build) turns it on, the BGE engine leaves it off.
if(BN_EMBEDDED)
  option(BN_BUILD_LAB "Build the Lab: scene files, Lab engines (Jolt included) and tools" OFF)
else()
  option(BN_BUILD_LAB "Build the Lab: scene files, Lab engines (Jolt included) and tools" ON)
endif()
if(BN_BUILD_LAB)
  add_subdirectory(lab)
endif()

if(BN_BUILD_HARNESS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/harness/CMakeLists.txt")
  add_subdirectory(harness)
endif()

# What the game engine needs to build its two copies of the physics module (C1.11).
set(BN_NEXT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/next" CACHE INTERNAL "Bullet NEXT headers")
set(BN_LEGACY_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/legacy" CACHE INTERNAL "Legacy backend headers")
set(BN_C_API_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/blender_c_api" CACHE INTERNAL "Blender C API shim")
set(BN_RENAME_TOOL "${CMAKE_CURRENT_SOURCE_DIR}/tools/rename_legacy.py" CACHE INTERNAL "Legacy rename tool")
