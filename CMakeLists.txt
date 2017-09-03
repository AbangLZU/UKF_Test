cmake_minimum_required(VERSION 3.7)
project(UKF_Test)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp ukf.cpp ukf.h)
add_executable(UKF_Test ${SOURCE_FILES})