add_executable(tercom-benchmark src/main.cpp)
target_link_libraries(tercom-benchmark PRIVATE tercom::tercom)
# The shared golden-scenario header (build-time support, not an installed header).
target_include_directories(tercom-benchmark PRIVATE ${CMAKE_SOURCE_DIR}/scenarios)
tercom_target_warnings(tercom-benchmark)
tercom_target_sanitizers(tercom-benchmark)
tercom_target_coverage(tercom-benchmark)
tercom_target_optimization(tercom-benchmark)
