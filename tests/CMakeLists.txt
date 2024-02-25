cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tests)

add_executable(${PROJECT_NAME}
    tests.cpp
    ../miniaudio.h
    ../miniaudio.hpp
)

# Set Visual Studio working directory to find tests files properly
set_target_properties(${PROJECT_NAME} PROPERTIES
  VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ..
    PUBLIC gtest
    PUBLIC gtest/include
)
