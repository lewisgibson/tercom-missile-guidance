include(GoogleTest)

# Treat the fetched GoogleTest headers as system headers so its macro expansions
# do not trip the strict first-party warning set (-Wconversion, -Wold-style-cast,
# and friends). Without this, warnings-as-errors would fail on code we do not own.
foreach(gtest_target gtest gtest_main gmock gmock_main)
    if(TARGET ${gtest_target})
        get_target_property(gtest_includes ${gtest_target} INTERFACE_INCLUDE_DIRECTORIES)
        if(gtest_includes)
            set_target_properties(${gtest_target} PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${gtest_includes}")
        endif()
    endif()
endforeach()

# One executable per module. Listed explicitly so the suite never drifts silently.
set(TERCOM_TESTS
    linear_test
    matrix_test
    attitude_test
    geodesy_test
    height_field_test
    ascii_grid_test
    synthetic_terrain_test
    contour_map_test
    strapdown_ins_test
    error_state_filter_test
    terrain_navigator_test
    route_test
    guidance_test
    guidance_system_test
    nmea_test
    serial_gnss_test
    simulated_sensors_test
    correlator_test
    sitl_test)

foreach(test_name ${TERCOM_TESTS})
    add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE tercom::tercom GTest::gtest_main)
    tercom_target_warnings(${test_name})
    tercom_target_sanitizers(${test_name})
    tercom_target_coverage(${test_name})
    # Static analysis is not run on test translation units: GoogleTest's macros
    # expand to code that trips many clang-tidy/cppcheck checks, and analysing the
    # assertion scaffolding adds noise without protecting the shipped library.
    set_target_properties(${test_name} PROPERTIES CXX_CLANG_TIDY "" CXX_CPPCHECK "")
    gtest_discover_tests(${test_name})
endforeach()

# The SITL golden test shares the canonical scenario header (build-time support,
# not an installed header) with the example and the benchmark.
target_include_directories(sitl_test PRIVATE ${CMAKE_SOURCE_DIR}/scenarios)
