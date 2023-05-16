cmake_minimum_required(VERSION 3.24)

# The release pipeline stamps the published version (1.0.<run_number>) so the
# installed find_package version matches the tarball; local builds default to 1.0.0.
if(NOT DEFINED TERCOM_VERSION)
    set(TERCOM_VERSION 1.0.0)
endif()

project(
    tercom
    VERSION ${TERCOM_VERSION}
    DESCRIPTION "Terrain-referenced navigation and software-in-the-loop guidance library"
    LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Language standard and global build settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Place build products predictably so the example/tool steps and the
# documentation can reference a stable layout.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# A sensible default for single-configuration generators.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
include(CompilerWarnings)
include(Sanitizers)
include(StaticAnalysis)
include(CodeCoverage)
include(Optimization)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(TERCOM_BUILD_EXAMPLES "Build the example programs" ON)
option(TERCOM_BUILD_TESTS "Build the unit tests" ON)
option(TERCOM_BUILD_TOOLS "Build the command-line tools" ON)
option(TERCOM_BUILD_BENCHMARKS "Build the performance benchmarks" OFF)
option(TERCOM_ENABLE_SANITIZERS "Build first-party targets with Address/UB sanitizers" OFF)
option(TERCOM_ENABLE_COVERAGE "Build first-party targets with coverage instrumentation" OFF)
option(TERCOM_ENABLE_CLANG_TIDY "Run clang-tidy as part of the build" OFF)
option(TERCOM_ENABLE_CPPCHECK "Run cppcheck as part of the build" OFF)
option(TERCOM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# ---------------------------------------------------------------------------
# Test framework (fetched). Declared before static analysis is switched on so
# the dependency itself is never passed through clang-tidy or cppcheck.
# ---------------------------------------------------------------------------
if(TERCOM_BUILD_TESTS)
    enable_testing()
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2)
    # Do not install GoogleTest alongside tercom, and reuse the parent's CRT.
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# Switch on the static analysers for everything compiled from here down. The
# helper exports CMAKE_CXX_CLANG_TIDY / CMAKE_CXX_CPPCHECK into this scope, so it
# must run after the fetched dependency and before the first-party subdirectories.
tercom_enable_static_analysis()

add_subdirectory(src)

if(TERCOM_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
if(TERCOM_BUILD_TOOLS)
    add_subdirectory(tools)
endif()
if(TERCOM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
if(TERCOM_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# ---------------------------------------------------------------------------
# Install + export so downstream projects can find_package(tercom)
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
    TARGETS tercom
    EXPORT tercomTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    EXPORT tercomTargets
    FILE tercomTargets.cmake
    NAMESPACE tercom::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tercom)

configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/tercomConfig.cmake.in
    ${CMAKE_BINARY_DIR}/tercomConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tercom)

write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/tercomConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

install(FILES ${CMAKE_BINARY_DIR}/tercomConfig.cmake ${CMAKE_BINARY_DIR}/tercomConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tercom)
