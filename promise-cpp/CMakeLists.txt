# CMakeList.txt : CMake project for AlfBFF, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.15)

project(promise-cpp)

# Add source to this project's executable.
add_library (${PROJECT_NAME} INTERFACE)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

target_include_directories(${PROJECT_NAME} 
	INTERFACE "include")

