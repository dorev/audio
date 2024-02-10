cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tests)

add_executable(${PROJECT_NAME}
    main.cpp
    tests.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC gtest
    PUBLIC gtest/include
    PUBLIC gmock
    PUBLIC gmock/include
)