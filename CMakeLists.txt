# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.

cmake_minimum_required (VERSION 3.15)

project ("AlfBFF" LANGUAGES CXX)

# global configuration
enable_testing()

# conan
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
  message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
  file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/develop/conan.cmake"
                "${CMAKE_BINARY_DIR}/conan.cmake"
                TLS_VERIFY ON)
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_configure(REQUIRES 
                        asio/1.32.0
                        spdlog/1.14.1
                        catch2/2.12.1
                        toml11/4.2.0
                      GENERATORS cmake)

conan_cmake_autodetect(settings)
conan_cmake_install(PATH_OR_REFERENCE .
                    BUILD missing
                    SETTINGS ${settings})

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

# windows defaults
add_definitions(-D_WIN32_WINNT=0x0A00)

# compiler
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (MSVC)
    add_compile_options(/MP)  
endif()

# Include sub-projects.
add_subdirectory("AltBFF")
add_subdirectory("BFFCLAPI")
add_subdirectory("Utils")
add_subdirectory("FSUIPC")
add_subdirectory("CSV")
