add_executable(sitl-simulation src/main.cpp)
target_link_libraries(sitl-simulation PRIVATE tercom::tercom)
# The shared golden-scenario header (build-time support, not an installed header).
target_include_directories(sitl-simulation PRIVATE ${CMAKE_SOURCE_DIR}/scenarios)
tercom_target_warnings(sitl-simulation)
tercom_target_sanitizers(sitl-simulation)
tercom_target_coverage(sitl-simulation)
tercom_target_optimization(sitl-simulation)
