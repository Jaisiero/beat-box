cmake_minimum_required(VERSION 3.21)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/deps.cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(beat-box VERSION 1.0)

file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.hpp)

message("src files:")
foreach(file ${SRCS})
    message(STATUS ${file})
endforeach()

add_executable(${PROJECT_NAME} ${SRCS})

# RigidBodyManager::create() instantiates dozens of large task objects as locals in a
# single function, producing a very large stack frame. The default 1 MB stack overflows
# on entry (STATUS_STACK_OVERFLOW). Give the main thread generous headroom.
# Linux needs no flag here: the main-thread stack is 8 MB by default (vs Windows' 1 MB) and the
# frame fits -- verified by a full scene_7 run on the RTX 4090. Do NOT re-add -Wl,-z,stacksize:
# that is a Solaris linker directive, GNU ld silently ignores it and prints
# "warning: -z stacksize=33554432 ignored", so it was pure noise. If a future refactor does
# overflow 8 MB, raise it at launch (ulimit -s 65536) rather than pretending a link flag works.
if (MSVC)
    target_link_options(${PROJECT_NAME} PRIVATE /STACK:33554432) # 32 MB
endif()

target_include_directories(${PROJECT_NAME} PUBLIC include)

find_package(glfw3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)

set(DAXA_ENABLE_UTILS_MEM ON CACHE BOOL "" FORCE)
set(DAXA_ENABLE_UTILS_IMGUI ON CACHE BOOL "" FORCE)
set(DAXA_ENABLE_UTILS_PIPELINE_MANAGER_SLANG ON CACHE BOOL "" FORCE)
set(DAXA_ENABLE_UTILS_TASK_GRAPH ON CACHE BOOL "" FORCE)
set(DAXA_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(DAXA_ENABLE_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory("${PROJECT_SOURCE_DIR}/lib/Daxa" "${PROJECT_BINARY_DIR}/Daxa")
target_link_libraries(${PROJECT_NAME} PRIVATE daxa::daxa)
if (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        "/external:I${PROJECT_SOURCE_DIR}/lib/Daxa/include"
        /external:W0
    )
endif()

if (WIN32)
    FetchContent_GetProperties(slang)
    # Automatically copy all Slang bin files (DLLs, executables, and standard library) to the target executable directory
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${slang_SOURCE_DIR}/bin"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
        COMMENT "Copying Slang compiler binaries and standard libraries to target directory..."
    )
endif()

# Synchronize runtime sources on every build, including shader-only edits. Preserve
# timestamps of identical files for Daxa's SPIR-V cache and remove stale shadow files.
add_custom_target(${PROJECT_NAME}_runtime_sources ALL
    COMMAND ${CMAKE_COMMAND}
        "-DSOURCE_TREE=${PROJECT_SOURCE_DIR}/include"
        "-DDESTINATION_TREE=$<TARGET_FILE_DIR:${PROJECT_NAME}>/include"
        -P "${PROJECT_SOURCE_DIR}/cmake/sync_runtime_sources.cmake"
    COMMAND ${CMAKE_COMMAND}
        "-DSOURCE_TREE=${PROJECT_SOURCE_DIR}/src/shaders"
        "-DDESTINATION_TREE=$<TARGET_FILE_DIR:${PROJECT_NAME}>/src/shaders"
        -P "${PROJECT_SOURCE_DIR}/cmake/sync_runtime_sources.cmake"
    COMMENT "Synchronizing runtime shader sources and includes"
    VERBATIM
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_runtime_sources)

# Copy assets to the build output directory so they are available when running from there
if (EXISTS "${PROJECT_SOURCE_DIR}/assets")
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/assets"
        COMMENT "Copying assets to target directory..."
    )
endif()

find_package(glm CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE glm::glm)

# Host-side unit tests for the C++/Slang-shared math (include/math.hpp). Pure CPU — no GPU,
# no window — so they run anywhere (incl. CI). Run with: ctest --test-dir build/Release -C RelWithDebInfo
enable_testing()
add_executable(math_tests tests/math_tests.cpp)
target_include_directories(math_tests PRIVATE include)
target_link_libraries(math_tests PRIVATE daxa::daxa glm::glm)
if (MSVC)
    target_compile_options(math_tests PRIVATE
        "/external:I${PROJECT_SOURCE_DIR}/lib/Daxa/include"
        /external:W0
    )
endif()
add_test(NAME math_tests COMMAND math_tests)

# D1-offline: opt-in step to warm the Slang->SPIR-V cache. Building all ~69 pipelines is the ~100s
# cold-start cost; Daxa caches the SPIR-V in spirv_cache/ next to the exe, so a launch that warms it
# ahead of time starts in ~2s. This runs the exe with BB_COMPILE_ONLY=1 (compile all pipelines, then
# exit before the render loop). It is a SEPARATE, MANUAL target (not an automatic POST_BUILD) so it
# never breaks a GPU-less CI build — run it on a machine with a GPU:
#     cmake --build build/Release --config RelWithDebInfo --target warm_shader_cache
add_custom_target(warm_shader_cache
    COMMAND ${CMAKE_COMMAND} -E env BB_COMPILE_ONLY=1 "$<TARGET_FILE:${PROJECT_NAME}>"
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    DEPENDS ${PROJECT_NAME}
    COMMENT "Warming the Slang->SPIR-V cache (spirv_cache/) so the first launch is fast"
    USES_TERMINAL
)

add_executable(runtime_diagnostics_tests tests/runtime_diagnostics_tests.cpp)
target_include_directories(runtime_diagnostics_tests PRIVATE src)
add_test(NAME runtime_diagnostics_tests COMMAND runtime_diagnostics_tests)

add_executable(path_tracing_tests tests/path_tracing_tests.cpp)
target_include_directories(path_tracing_tests PRIVATE include)
target_link_libraries(path_tracing_tests PRIVATE daxa::daxa glm::glm)
add_test(NAME path_tracing_tests COMMAND path_tracing_tests)
