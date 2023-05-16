cmake_minimum_required(VERSION 3.24.2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE TRUE)

project(tercom CXX)

option(BUILD_EXAMPLES "Build all examples")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
source_group(Source FILES ${SOURCES})
add_library(lib-tercom STATIC ${SOURCES})

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif(BUILD_EXAMPLES)
