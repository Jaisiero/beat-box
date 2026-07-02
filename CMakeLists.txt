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
if (MSVC)
    target_link_options(${PROJECT_NAME} PRIVATE /STACK:33554432) # 32 MB
else()
    target_link_options(${PROJECT_NAME} PRIVATE -Wl,-z,stacksize=33554432)
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

# Copy runtime shader sources/includes to the build output directory so the
# pipeline manager can compile shaders when the executable is launched there.
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${PROJECT_SOURCE_DIR}/include"
    "$<TARGET_FILE_DIR:${PROJECT_NAME}>/include"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${PROJECT_SOURCE_DIR}/src/shaders"
    "$<TARGET_FILE_DIR:${PROJECT_NAME}>/src/shaders"
    COMMENT "Copying runtime shader sources and includes to target directory..."
)

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
