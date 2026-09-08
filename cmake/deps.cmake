
# Ensure Daxa is available
# NOTE: This is temporary, and should really be consumed from the official vcpkg port of Daxa.
# Daxa is in active development, so for the sake of the tutorial, we're using a newer version
# of Daxa than is available through vcpkg directly.
if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/../lib/Daxa/CMakeLists.txt")
    find_package(Git REQUIRED)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../lib")
    execute_process(COMMAND ${GIT_EXECUTABLE} clone https://github.com/Ipotrick/Daxa
        WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../lib"
        COMMAND_ERROR_IS_FATAL ANY)
    execute_process(COMMAND ${GIT_EXECUTABLE} checkout 3.6
        WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../lib/Daxa"
        COMMAND_ERROR_IS_FATAL ANY)
endif()

# lib/Daxa is gitignored and checked out at tag 3.6 above, but this project REQUIRES the local
# patch (TInlineTask move/copy-assignment + uses() fix, and the bundled-Slang 2025.11 -> 2026.10
# bump; see patches/daxa-3.6/README.md) or it does not compile. Apply it idempotently on every
# configure: if it applies cleanly -> apply; if it reverse-applies -> already patched, skip;
# otherwise the local Daxa tree diverged -> warn (do not hard-fail a working local setup).
foreach(BB_DAXA_PATCH_NAME local-changes.patch compute-occupancy.patch)
set(BB_DAXA_PATCH "${CMAKE_CURRENT_LIST_DIR}/../patches/daxa-3.6/${BB_DAXA_PATCH_NAME}")
if(EXISTS "${BB_DAXA_PATCH}")
    find_package(Git REQUIRED)
    execute_process(COMMAND ${GIT_EXECUTABLE} apply --check "${BB_DAXA_PATCH}"
        WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../lib/Daxa"
        RESULT_VARIABLE BB_DAXA_PATCH_NEEDED OUTPUT_QUIET ERROR_QUIET)
    if(BB_DAXA_PATCH_NEEDED EQUAL 0)
        execute_process(COMMAND ${GIT_EXECUTABLE} apply "${BB_DAXA_PATCH}"
            WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../lib/Daxa"
            COMMAND_ERROR_IS_FATAL ANY)
        message(STATUS "beat-box: applied patches/daxa-3.6/${BB_DAXA_PATCH_NAME} to lib/Daxa")
    else()
        execute_process(COMMAND ${GIT_EXECUTABLE} apply --check --reverse "${BB_DAXA_PATCH}"
            WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../lib/Daxa"
            RESULT_VARIABLE BB_DAXA_PATCH_REVERSED OUTPUT_QUIET ERROR_QUIET)
        if(NOT BB_DAXA_PATCH_REVERSED EQUAL 0)
            message(WARNING "beat-box: lib/Daxa matches neither a clean 3.6 checkout nor the patched "
                "state (patches/daxa-3.6/${BB_DAXA_PATCH_NAME}). The build may fail to compile; see "
                "patches/daxa-3.6/README.md to reconcile the tree.")
        endif()
    endif()
endif()

endforeach()

# If the user has set a toolchain file, we'll want to chainload it via vcpkg
if(NOT (CMAKE_TOOLCHAIN_FILE MATCHES "/scripts/buildsystems/vcpkg.cmake") AND DEFINED CMAKE_TOOLCHAIN_FILE)
    set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_FILE}" CACHE UNINITIALIZED "")
endif()

# Check if vcpkg is installed globally. Otherwise, clone vcpkg
if(EXISTS "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    file(TO_CMAKE_PATH $ENV{VCPKG_ROOT} VCPKG_ROOT)
    set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
else()
    if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/../lib/vcpkg/scripts/buildsystems/vcpkg.cmake")
        find_package(Git REQUIRED)
        execute_process(COMMAND ${GIT_EXECUTABLE} clone https://github.com/Microsoft/vcpkg
            WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../lib"
            COMMAND_ERROR_IS_FATAL ANY)
    endif()
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/../lib/vcpkg/scripts/buildsystems/vcpkg.cmake")
endif()
