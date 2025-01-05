# CMakeList.txt : CMake project for AlfBFF, include source and define
# project specific logic here.
#
cmake_minimum_required(VERSION 3.15)

project(Utils)

# Add source to this project's library.
add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME} PUBLIC)

target_include_directories(${PROJECT_NAME} INTERFACE "include")

target_sources(
    ${PROJECT_NAME}
    PRIVATE
        "include/Utils/Accumulators.h"
        "include/Utils/Common.h"
        "include/Utils/PID.h"
        "include/Utils/RateLimiter.h"
        "include/Utils/PeriodicTimer.h"
)
