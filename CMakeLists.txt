# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.

cmake_minimum_required (VERSION 3.15)

project ("AlfBFF")

# global configuration
enable_testing()

# conan
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

# windows defaults
add_definitions(-D_WIN32_WINNT=0x0A00)

# compiler

# Include sub-projects.
add_subdirectory("AltBFF")
add_subdirectory("BFFCLAPI")
add_subdirectory("Utils")
add_subdirectory("promise-cpp")
add_subdirectory("FSUIPC")
add_subdirectory("CSV")
