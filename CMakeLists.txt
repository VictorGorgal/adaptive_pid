cmake_minimum_required(VERSION 3.10)

project(adaptive_pid)

# Specify the source files
set(SOURCES
    src/adaptive_pid.c
)

# Create the library
add_library(adaptive_pid STATIC ${SOURCES})

# Specify include directories
target_include_directories(adaptive_pid PUBLIC include)

# Link the necessary libraries
target_link_libraries(adaptive_pid
    pico_stdlib
)
