# CMakeList.txt : CMake project for AlfBFF, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.15)

project(BFFCLAPI)

# Add source to this project's executable.
add_library (${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}  PUBLIC CONAN_PKG::boost promise-cpp CONAN_PKG::spdlog)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} 
	PUBLIC "include" 
	PRIVATE "include/BFFCLAPI")

target_sources(${PROJECT_NAME} 
	PRIVATE 
		"src/UDPClient.cpp"
		"src/ClientReceiver.cpp"
		"src/ClientSender.cpp"
		"include/BFFCLAPI/UDPClient.h"
		"include/BFFCLAPI/CLStructures.h")
