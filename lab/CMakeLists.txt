# Bullet NEXT Lab (FL): scene files, and the Lab engines - the harness World behind one
# backend-neutral interface (lab/core/bnl_engine.h), built once per Bullet copy:
#   orig     - Blender 2.79b's Bullet 2.84, unrenamed (fidelity reference)
#   legacy   - the frozen, renamed backend (harness sources renamed by tools/rename_legacy.py)
#   next     - Bullet NEXT, features off
#   nextf    - Bullet NEXT with the F1 features (what BGE NEXT uses)
#   upstream - pristine upstream Bullet
# Each copy of the harness code lives in its own namespace (bn::<tag>), so legacy, next and nextf
# link into one program; orig and upstream use the plain bt names and stay in their own programs.
find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_library(bnl_core STATIC core/bnl_scene.cpp core/bnl_scene.h core/bnl_engine.h core/bnl_mesh.cpp core/bnl_mesh.h
            core/bnl_recording.cpp core/bnl_recording.h)
target_include_directories(bnl_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/core" "${PROJECT_SOURCE_DIR}/third_party")
set(BN_LAB_SCENES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/scenes" CACHE INTERNAL "Lab scene files")

set(BN_HSRC ${PROJECT_SOURCE_DIR}/harness/src)
set(BNL_ENGINE_SRC ${BN_HSRC}/bn_world.cpp ${BN_HSRC}/bn_record.cpp ${BN_HSRC}/bn_load.cpp ${BN_HSRC}/bn_engine.cpp
    ${BN_HSRC}/bn_fuzz.cpp)
set(BNL_HARNESS_HDR ${BN_HSRC}/bn_common.h ${BN_HSRC}/bn_world.h ${BN_HSRC}/bn_record.h ${BN_HSRC}/bn_load.h)
set(BNL_MAIN_SRC ${BN_HSRC}/bn_main.cpp)
set(BN_RENAME ${PROJECT_SOURCE_DIR}/tools/rename_legacy.py)

# Renamed copy of the harness for the legacy backend (engine sources and the runner).
set(BNL_GEN_LEGACY ${CMAKE_CURRENT_BINARY_DIR}/gen_legacy CACHE INTERNAL "Renamed harness sources")
set(_gen "")
foreach(f ${BNL_ENGINE_SRC} ${BNL_MAIN_SRC} ${BNL_HARNESS_HDR})
  get_filename_component(n ${f} NAME)
  list(APPEND _gen ${BNL_GEN_LEGACY}/${n})
endforeach()
add_custom_command(
  OUTPUT ${_gen}
  COMMAND ${Python3_EXECUTABLE} ${BN_RENAME} files ${BNL_GEN_LEGACY} ${BNL_ENGINE_SRC} ${BNL_MAIN_SRC} ${BNL_HARNESS_HDR}
  DEPENDS ${BNL_ENGINE_SRC} ${BNL_MAIN_SRC} ${BNL_HARNESS_HDR} ${BN_RENAME} ${PROJECT_SOURCE_DIR}/tools/legacy_extra_symbols.txt
  COMMENT "Renaming harness sources for the legacy backend")
add_custom_target(bnl_gen_legacy DEPENDS ${_gen})
set(_gen_engine "")
foreach(f ${BNL_ENGINE_SRC})
  get_filename_component(n ${f} NAME)
  list(APPEND _gen_engine ${BNL_GEN_LEGACY}/${n})
endforeach()

set(BNL_ENGINE_TAGS "" CACHE INTERNAL "Lab engines built")
function(bnl_engine tag lib backend)
  set(target bnl_engine_${tag})
  add_library(${target} STATIC ${ARGN})
  target_include_directories(${target} PUBLIC ${BN_HSRC})
  target_link_libraries(${target} PUBLIC ${lib} bnl_core)
  # Private: several engines link into one program. The harness runner repeats them (BNL_DEFS).
  set(defs BN_TAG=${tag} BN_BACKEND_NAME="${backend}" BN_SCENES_DIR="${BN_LAB_SCENES_DIR}")
  if(tag STREQUAL "next" OR tag STREQUAL "nextf" OR tag STREQUAL "nextd")
    list(APPEND defs BN_NEXT_LIB=1)
  endif()
  if(tag STREQUAL "nextf" OR tag STREQUAL "nextd")
    list(APPEND defs BN_NEXT_FEATURES=1)
  endif()
  target_compile_definitions(${target} PRIVATE ${defs})
  set_target_properties(${target} PROPERTIES BNL_DEFS "${defs}")
  set(BNL_ENGINE_TAGS ${BNL_ENGINE_TAGS} ${tag} CACHE INTERNAL "Lab engines built")
endfunction()

if(TARGET bn_bullet284_orig)
  bnl_engine(orig bn_bullet284_orig orig ${BNL_ENGINE_SRC})
endif()
if(TARGET bn_bullet_legacy)
  bnl_engine(legacy bn_bullet_legacy legacy ${_gen_engine})
  target_include_directories(bnl_engine_legacy BEFORE PUBLIC ${BNL_GEN_LEGACY})
  add_dependencies(bnl_engine_legacy bnl_gen_legacy)
endif()
if(TARGET bn_bullet_next)
  bnl_engine(next bn_bullet_next next ${BNL_ENGINE_SRC})
  bnl_engine(nextf bn_bullet_next nextf ${BNL_ENGINE_SRC})
endif()
if(TARGET bn_bullet_upstream)
  bnl_engine(upstream bn_bullet_upstream upstream ${BNL_ENGINE_SRC})
endif()
# Double precision, as Blender 5.3 builds Bullet (host profile blender53-rb): Blender's own copy
# and Bullet NEXT with its features. Separate programs: both use the plain bt names.
if(TARGET bn_bullet_blender53)
  bnl_engine(blender53 bn_bullet_blender53 blender53 ${BNL_ENGINE_SRC})
  # TL.5 reference: Blender 5.3's own rb_bullet_api.cpp driven as BKE_rigidbody does.
  add_executable(bnl_rbref reference/bnl_rbref.cpp ${PROJECT_SOURCE_DIR}/third_party/blender53/rigidbody/rb_bullet_api.cpp)
  target_include_directories(bnl_rbref PRIVATE ${PROJECT_SOURCE_DIR}/third_party/blender53/rigidbody ${BN_HSRC})
  target_link_libraries(bnl_rbref PRIVATE bn_bullet_blender53 bnl_core)
  target_compile_definitions(bnl_rbref PRIVATE BN_SCENES_DIR="${BN_LAB_SCENES_DIR}")
endif()
if(TARGET bn_bullet_nextd)
  bnl_engine(nextd bn_bullet_nextd nextd ${BNL_ENGINE_SRC})
endif()

# Jolt Physics as a Lab engine, the yardstick (CL.6): third_party/jolt (v5.6.0), C++17, built
# apart from Bullet NEXT. Cross-platform deterministic, no GPU compute, no installation targets.
option(BN_LAB_JOLT "Build the Jolt Lab engine" ON)
if(BN_LAB_JOLT)
  set(PHYSICS_REPO_ROOT "${PROJECT_SOURCE_DIR}/third_party/jolt")
  foreach(o JPH_USE_DX12 JPH_USE_VK JPH_USE_MTL JPH_USE_CPU_COMPUTE ENABLE_OBJECT_STREAM ENABLE_INSTALL
            ENABLE_ALL_WARNINGS INTERPROCEDURAL_OPTIMIZATION OVERRIDE_CXX_FLAGS FLOATING_POINT_EXCEPTIONS_ENABLED
            USE_STATIC_MSVC_RUNTIME_LIBRARY DEBUG_RENDERER_IN_DEBUG_AND_RELEASE PROFILER_IN_DEBUG_AND_RELEASE
            GENERATE_DEBUG_SYMBOLS)
    set(${o} OFF CACHE BOOL "" FORCE)
  endforeach()
  set(CROSS_PLATFORM_DETERMINISTIC ON CACHE BOOL "" FORCE)
  if(EMSCRIPTEN)
    # HARFANG's browser build links with exception catching on, which Emscripten refuses to mix
    # with -fno-exceptions code.
    set(CPP_EXCEPTIONS_ENABLED ON CACHE BOOL "" FORCE)
  endif()
  add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/jolt/Build ${CMAKE_CURRENT_BINARY_DIR}/jolt EXCLUDE_FROM_ALL)
  add_library(bnl_engine_jolt STATIC engines/bnl_jolt.cpp)
  set_target_properties(bnl_engine_jolt PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
  # Jolt is built without RTTI; classes derived from its types must be too.
  target_compile_options(bnl_engine_jolt PRIVATE $<IF:$<CXX_COMPILER_ID:MSVC>,/GR-,-fno-rtti>)
  target_link_libraries(bnl_engine_jolt PUBLIC Jolt bnl_core)
endif()

# The engines one program can hold together, behind bnl::createEngine(name).
add_library(bnl_engines STATIC core/bnl_registry.cpp)
target_link_libraries(bnl_engines PUBLIC bnl_core)
foreach(tag legacy next nextf jolt)
  if(TARGET bnl_engine_${tag})
    target_link_libraries(bnl_engines PUBLIC bnl_engine_${tag})
    string(TOUPPER ${tag} TAGU)
    target_compile_definitions(bnl_engines PRIVATE BNL_HAVE_${TAGU}=1)
  endif()
endforeach()

# Several engines in one program, stepped in turn (CL.2 check, gate tooling).
# Random collision pairs on the Bullet engines (CL.11, TL.7).
add_executable(bnl_fuzz tools/bnl_fuzz.cpp)
target_link_libraries(bnl_fuzz PRIVATE bnl_engines)

# Heightfield edge data: worked out against baked, and their cost (Stage 1: T1S.1, T1S.3, B1S.1).
if(TARGET bn_bullet_next AND NOT EMSCRIPTEN)
  add_executable(bnl_edgecheck tools/bnl_edgecheck.cpp)
  target_link_libraries(bnl_edgecheck PRIVATE bn_bullet_next bnl_core)
endif()

add_executable(bnl_run tools/bnl_run.cpp)
target_link_libraries(bnl_run PRIVATE bnl_engines)
target_compile_definitions(bnl_run PRIVATE BN_SCENES_DIR="${BN_LAB_SCENES_DIR}")
if(EMSCRIPTEN)
  # Browser build (CL.10): under Node the runner reads the scene files straight from disk.
  # Bullet keeps large arrays on the stack (GJK, EPA); Emscripten gives 64 KB unless told.
  target_link_options(bnl_run PRIVATE -sNODERAWFS=1 -sALLOW_MEMORY_GROWTH=1 -sEXIT_RUNTIME=1 -sSTACK_SIZE=4MB)
endif()
