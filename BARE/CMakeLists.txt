cmake_minimum_required(VERSION 3.20)

project("BARE" C CXX ASM)

include(cmake/st-project.cmake)
include_directories(Inc)
include_directories(Debug)
include_directories(Src)
#include_directories(build\debug\build)

add_executable(${PROJECT_NAME})
add_st_target_properties(${PROJECT_NAME})