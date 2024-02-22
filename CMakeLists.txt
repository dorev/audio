cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tests)

add_executable(${PROJECT_NAME}
    tests.cpp
    miniaudio.h
    miniaudio.hpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC gtest
    PUBLIC gtest/include
)