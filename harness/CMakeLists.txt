# Bullet NEXT golden harness (F0; scene files and Lab engines since FL).
# One runner per backend: harness/src/bn_main.cpp linked with that backend's Lab engine library
# (lab/CMakeLists.txt), which holds the World, the recorder and the scene-file loader.
function(bn_harness target tag main)
  if(NOT TARGET bnl_engine_${tag})
    return()
  endif()
  add_executable(${target} ${main})
  target_link_libraries(${target} PRIVATE bnl_engine_${tag})
  get_target_property(defs bnl_engine_${tag} BNL_DEFS)
  target_compile_definitions(${target} PRIVATE ${defs})
  if(tag STREQUAL "legacy")
    add_dependencies(${target} bnl_gen_legacy)
  endif()
endfunction()

bn_harness(bn_harness_orig orig ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_main.cpp)
set_source_files_properties(${BNL_GEN_LEGACY}/bn_main.cpp PROPERTIES GENERATED TRUE)  # made in lab/
bn_harness(bn_harness_legacy legacy ${BNL_GEN_LEGACY}/bn_main.cpp)
bn_harness(bn_harness_next next ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_main.cpp)
bn_harness(bn_harness_nextf nextf ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_main.cpp)
bn_harness(bn_harness_upstream upstream ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_main.cpp)
bn_harness(bn_harness_blender53 blender53 ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_main.cpp)
bn_harness(bn_harness_nextd nextd ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_main.cpp)

add_executable(bn_compare ${CMAKE_CURRENT_SOURCE_DIR}/compare/bn_compare.cpp)

# T0.2 symbol isolation: one program that links the legacy and next libraries together.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(BN_RENAME ${PROJECT_SOURCE_DIR}/tools/rename_legacy.py)
set(BN_GEN ${CMAKE_CURRENT_BINARY_DIR}/gen_symcheck)
if(TARGET bn_bullet_legacy AND TARGET bn_bullet_next)
  add_custom_command(
    OUTPUT ${BN_GEN}/symcheck_world_legacy.cpp
    COMMAND ${Python3_EXECUTABLE} ${BN_RENAME} files ${BN_GEN}/symtmp ${CMAKE_CURRENT_SOURCE_DIR}/symcheck/symcheck_world.cpp
    COMMAND ${CMAKE_COMMAND} -E copy ${BN_GEN}/symtmp/symcheck_world.cpp ${BN_GEN}/symcheck_world_legacy.cpp
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/symcheck/symcheck_world.cpp ${BN_RENAME})
  add_library(bn_symcheck_legacy OBJECT ${BN_GEN}/symcheck_world_legacy.cpp)
  target_include_directories(bn_symcheck_legacy PRIVATE ${PROJECT_SOURCE_DIR}/legacy)
  target_compile_definitions(bn_symcheck_legacy PRIVATE BN_SYM_FN=bn_symcheck_legacy_world)
  add_library(bn_symcheck_next OBJECT ${CMAKE_CURRENT_SOURCE_DIR}/symcheck/symcheck_world.cpp)
  target_include_directories(bn_symcheck_next PRIVATE ${PROJECT_SOURCE_DIR}/next)
  target_compile_definitions(bn_symcheck_next PRIVATE BN_SYM_FN=bn_symcheck_next_world BT_THREADSAFE=1)
  add_executable(bn_symcheck ${CMAKE_CURRENT_SOURCE_DIR}/symcheck/symcheck_main.cpp
    $<TARGET_OBJECTS:bn_symcheck_legacy> $<TARGET_OBJECTS:bn_symcheck_next>)
  target_link_libraries(bn_symcheck PRIVATE bn_bullet_legacy bn_bullet_next)
endif()

# One-time export of the F0/F1 C++ scenes to scene files (CL.5), against the 2.84 copy.
if(TARGET bn_bullet284_orig AND EXISTS ${PROJECT_SOURCE_DIR}/lab/tools/bnl_export_f0.cpp)
  add_executable(bnl_export_f0 ${PROJECT_SOURCE_DIR}/lab/tools/bnl_export_f0.cpp)
  target_link_libraries(bnl_export_f0 PRIVATE bn_bullet284_orig bnl_core)
endif()
