cmake_minimum_required(VERSION 3.15)

project(AltBFF)

# Add source to this project's executable.
add_executable(${PROJECT_NAME})

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE BFFCLAPI Utils spdlog::spdlog Catch2::Catch2 toml11::toml11
)

target_sources(
    ${PROJECT_NAME}
    PRIVATE
        "src/Main.cpp"
        "src/Model.cpp"
        "src/Model.h"
        "src/Sim.cpp"
        "src/Sim.h"
        "src/Tests.cpp"
        "src/A2ASTec30AP.h"
        "src/A2ASTec30AP.cpp"
        "src/ReadSettings.cpp"
        "src/ReadSettings.h"
        "src/ControlLoop.cpp"
        "src/ControlLoop.h"
        "src/SimImpl.h"
        "src/SimDummy.h"
        "src/PeriodicLoop.h"
)

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE FSUIPC)

    target_sources(
        ${PROJECT_NAME}
        PRIVATE "src/SimFSUIPC.cpp" "src/SimFSUIPC.h"
    )
endif()

set(settingsfile_source "${CMAKE_CURRENT_SOURCE_DIR}/resources/")

add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_directory ${settingsfile_source}
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
)
