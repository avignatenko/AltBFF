# CMakeList.txt : CMake project for AlfBFF, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.15)

project(Utils)

# Add source to this project's library.
add_library (${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}  PUBLIC CONAN_PKG::boost)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} 
	PUBLIC "include" 
	PRIVATE "include/Utils")

target_sources(${PROJECT_NAME} 
	PRIVATE 
		"src/DispatchQueue.cpp"
		"src/IRunner.cpp"
		"include/Utils/DispatchQueue.h"
		"include/Utils/IRunner.h"
		"include/Utils/Timer.h")
