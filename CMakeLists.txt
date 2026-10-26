cmake_minimum_required(VERSION 3.16)
project(BulletNEXT LANGUAGES CXX)

# Bullet NEXT must stay C++14 so it can be wired into the 2.79b engine.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Strict floating point: no fast-math, no FMA contraction (determinism rule).
if(MSVC)
  add_compile_options(/fp:precise /W3 /wd4244 /wd4267 /wd4305 /wd4996)
else()
  add_compile_options(-fno-fast-math -ffp-contract=off -w)
endif()

option(BN_BUILD_ORIG "Build the unrenamed Bullet 2.84 (Blender 2.79b) copy for fidelity tests" ON)
option(BN_BUILD_LEGACY "Build the frozen, renamed legacy backend" ON)
option(BN_BUILD_NEXT "Build Bullet NEXT (next/, upstream base + F1 changes)" ON)
option(BN_BUILD_UPSTREAM "Build pristine upstream for comparison" ON)
option(BN_BUILD_HARNESS "Build the golden harness" ON)

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
  find_package(Threads REQUIRED)
  target_compile_definitions(bn_bullet_next PUBLIC BT_THREADSAFE=1)
  target_link_libraries(bn_bullet_next PUBLIC Threads::Threads)
endif()
if(BN_BUILD_UPSTREAM)
  bn_add_bullet_lib(bn_bullet_upstream "${CMAKE_CURRENT_SOURCE_DIR}/third_party/bullet3_master")
endif()

if(BN_BUILD_HARNESS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/harness/CMakeLists.txt")
  add_subdirectory(harness)
endif()
