# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.

cmake_minimum_required (VERSION 3.15)

project ("AlfBFF" LANGUAGES CXX)

# global configuration
enable_testing()

# conan
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
  message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
  file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/v0.16.1/conan.cmake"
                "${CMAKE_BINARY_DIR}/conan.cmake"
                TLS_VERIFY ON)
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_configure(REQUIRES 
                        boost/1.71.0@conan/stable
                        spdlog/1.4.2@bincrafters/stable
                        catch2/2.12.1
                      GENERATORS cmake_multi)

set(CMAKE_CONFIGURATION_TYPES  Debug Release)

get_property(is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG )

if(is_multi_config)
    message("Multi config generator detected")
    set(conan_configs_to_create ${CMAKE_CONFIGURATION_TYPES})
    set(conan_cmake_file conanbuildinfo_multi.cmake)
else()
     message("Single config generator detected")
    set(conan_configs_to_create ${CMAKE_BUILD_TYPE})
    set(conan_cmake_file conanbuildinfo.cmake)
endif()

foreach(config ${conan_configs_to_create})
    set(CMAKE_BUILD_TYPE ${config})
    conan_cmake_autodetect(settings)
    conan_cmake_install(PATH_OR_REFERENCE .
                        BUILD missing
                        SETTINGS ${settings})
    set(CMAKE_BUILD_TYPE "")
endforeach()

include(${CMAKE_BINARY_DIR}/${conan_cmake_file})
conan_basic_setup(TARGETS)

# windows defaults
add_definitions(-D_WIN32_WINNT=0x0A00)

# compiler
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include sub-projects.
add_subdirectory("AltBFF")
add_subdirectory("BFFCLAPI")
add_subdirectory("Utils")
add_subdirectory("promise-cpp")
add_subdirectory("FSUIPC")
add_subdirectory("CSV")
