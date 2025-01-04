# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.

cmake_minimum_required(VERSION 3.15)

# conan
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan_provider.cmake")
    message(
        STATUS
        "Downloading conan_provider.cmake from https://github.com/conan-io/cmake-conan"
    )
    file(
        DOWNLOAD
            "https://raw.githubusercontent.com/conan-io/cmake-conan/refs/heads/develop2/conan_provider.cmake"
            "${CMAKE_BINARY_DIR}/conan_provider.cmake"
        TLS_VERIFY ON
    )
endif()

set(CMAKE_PROJECT_TOP_LEVEL_INCLUDES ${CMAKE_BINARY_DIR}/conan_provider.cmake)

project("AlfBFF" LANGUAGES CXX)

# global configuration
enable_testing()

# windows defaults
if(WIN32)
    add_definitions(-D_WIN32_WINNT=0x0A00)
endif()

# compiler
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/MP)
endif()

find_package(asio)
find_package(asio)
find_package(spdlog)
find_package(catch2)
find_package(toml11)

# Include sub-projects.
add_subdirectory("AltBFF")
add_subdirectory("BFFCLAPI")
add_subdirectory("Utils")
add_subdirectory("FSUIPC")
