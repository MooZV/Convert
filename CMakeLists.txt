cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(convert)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(convert convert.cpp)
