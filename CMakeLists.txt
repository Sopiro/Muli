cmake_minimum_required(VERSION 3.13)

set(PROJECT_NAME test)
set(CMAKE_CXX_STANDARD 17)

project(${PROJECT_NAME})
add_executable(${PROJECT_NAME} src/main.cpp)
