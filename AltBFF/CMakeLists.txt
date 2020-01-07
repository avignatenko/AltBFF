# CMakeList.txt : CMake project for AlfBFF, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.15)

project(AltBFF)

# Add source to this project's executable.
add_executable (${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}  PRIVATE BFFCLAPI Utils)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_sources(${PROJECT_NAME} PRIVATE 
	"src/AltBFF.cpp"
	"src/AltBFF.h")
