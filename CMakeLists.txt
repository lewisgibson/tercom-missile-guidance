cmake_minimum_required(VERSION 3.24.2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE TRUE)

project(tercom CXX)

option(BUILD_STATIC "Build static library")
option(BUILD_EXAMPLES "Build examples")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
source_group(Source FILES ${SOURCES})

add_library(tercom STATIC ${SOURCES})
add_library(tercomstatic SHARED ${SOURCES})
set_target_properties(tercomstatic PROPERTIES OUTPUT_NAME tercom)

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif(BUILD_EXAMPLES)
