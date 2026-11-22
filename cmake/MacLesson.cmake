# Standalone lesson entry point. The root Ninja build owns the shared libraries.
get_filename_component(lesson "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
string(MAKE_C_IDENTIFIER "lesson_${lesson}" lesson_target)
project(${lesson_target} LANGUAGES NONE)

get_filename_component(root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(shared_build_dir "${root}/build/macos")
set(deps "${root}/build/deps")
if(NOT EXISTS "${deps}/bin/ninja")
  message(FATAL_ERROR "Run ./build_3rdparty.sh in ${root} first.")
endif()
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env "PATH=${deps}/bin:$ENV{PATH}"
          "${CMAKE_COMMAND}" -S "${root}" -B "${shared_build_dir}" -G Ninja
          "-DCMAKE_PREFIX_PATH=${deps}" -DCMAKE_BUILD_TYPE=Release
  COMMAND_ERROR_IS_FATAL ANY)

add_custom_target(${lesson_target} ALL
  COMMAND "${CMAKE_COMMAND}" --build "${shared_build_dir}" --target "${lesson_target}"
  USES_TERMINAL VERBATIM)
if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
  configure_file("${CMAKE_CURRENT_LIST_DIR}/lesson.ninja.in" build.ninja @ONLY)
endif()
