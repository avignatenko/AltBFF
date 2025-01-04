cmake_minimum_required(VERSION 3.15)

project(CSV)

# Add source to this project's executable.
add_library(${PROJECT_NAME})

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC "include"
    PRIVATE "include/CSV"
)

target_sources(
    ${PROJECT_NAME}
    PRIVATE
        "src/CSVread.cpp"
        "src/CSVwrite.cpp"
        "src/strerror.cpp"
        "src/libcsv.c"
)
