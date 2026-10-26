# Bullet NEXT golden harness (F0).
# The same harness sources are built against three Bullet copies:
#   orig   - Blender 2.79b's Bullet 2.84, unrenamed (fidelity reference)
#   legacy - the frozen, renamed backend (sources renamed by tools/rename_legacy.py)
#   next   - Bullet NEXT (upstream master base)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(BN_HARNESS_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_world.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_record.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_scenes.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_main.cpp)
set(BN_HARNESS_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_common.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_world.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_record.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bn_scenes.h)
set(BN_RENAME ${PROJECT_SOURCE_DIR}/tools/rename_legacy.py)

# Renamed copy of the harness for the legacy backend.
set(BN_GEN ${CMAKE_CURRENT_BINARY_DIR}/gen_legacy)
set(BN_GEN_SRC "")
foreach(f ${BN_HARNESS_SRC} ${BN_HARNESS_HDR})
  get_filename_component(n ${f} NAME)
  list(APPEND BN_GEN_SRC ${BN_GEN}/${n})
endforeach()
add_custom_command(
  OUTPUT ${BN_GEN_SRC}
  COMMAND ${Python3_EXECUTABLE} ${BN_RENAME} files ${BN_GEN} ${BN_HARNESS_SRC} ${BN_HARNESS_HDR}
  DEPENDS ${BN_HARNESS_SRC} ${BN_HARNESS_HDR} ${BN_RENAME} ${PROJECT_SOURCE_DIR}/tools/legacy_extra_symbols.txt
  COMMENT "Renaming harness sources for the legacy backend")
list(FILTER BN_GEN_SRC INCLUDE REGEX ".*\\.cpp$")

function(bn_harness target lib backend)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} PRIVATE ${lib})
  target_compile_definitions(${target} PRIVATE BN_BACKEND_NAME="${backend}")
endfunction()

if(TARGET bn_bullet284_orig)
  bn_harness(bn_harness_orig bn_bullet284_orig orig ${BN_HARNESS_SRC})
endif()
if(TARGET bn_bullet_legacy)
  bn_harness(bn_harness_legacy bn_bullet_legacy legacy ${BN_GEN_SRC})
endif()
if(TARGET bn_bullet_next)
  bn_harness(bn_harness_next bn_bullet_next next ${BN_HARNESS_SRC})
  # Bullet NEXT with the F1 features switched on (what BGE NEXT will use).
  bn_harness(bn_harness_nextf bn_bullet_next nextf ${BN_HARNESS_SRC})
  target_compile_definitions(bn_harness_nextf PRIVATE BN_NEXT_FEATURES=1)
endif()
if(TARGET bn_bullet_upstream)
  bn_harness(bn_harness_upstream bn_bullet_upstream upstream ${BN_HARNESS_SRC})
endif()

add_executable(bn_compare ${CMAKE_CURRENT_SOURCE_DIR}/compare/bn_compare.cpp)

# T0.2 symbol isolation: one program that links the legacy and next libraries together.
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
