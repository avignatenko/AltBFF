# CMakeList.txt : CMake project for AlfBFF, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.15)

project(AltBFF)

# Add source to this project's executable.
add_executable (${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}  PRIVATE BFFCLAPI Utils FSUIPC CONAN_PKG::spdlog CONAN_PKG::catch2)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)


target_sources(${PROJECT_NAME} PRIVATE 
	"src/Main.cpp"
	"src/Model.cpp"
	"src/Model.h"
	"src/Sim.cpp"
	"src/Sim.h"
	"src/Tests.cpp" 
	"src/A2ASTec30AP.cpp")


set (settingsfile_source "${CMAKE_CURRENT_SOURCE_DIR}/resources/settings.ini")
set (settingsfile_destination "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/settings.ini")

add_custom_command(
 COMMAND ${CMAKE_COMMAND} -E copy ${settingsfile_source} ${settingsfile_destination} 
 DEPENDS ${settingsfile_source}
 OUTPUT ${settingsfile_destination}
 COMMENT "Copy settings file ${settingsfile_source} => ${settingsfile_destination}"
)

add_custom_target(CopyResources ALL DEPENDS "${settingsfile_destination}")