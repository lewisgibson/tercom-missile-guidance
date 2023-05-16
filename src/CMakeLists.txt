# The tercom library. Sources are listed explicitly rather than globbed so that
# adding or removing a translation unit is a deliberate, reviewable change and the
# build never silently drifts from the tree.
add_library(
    tercom
    geo/ascii_grid.cpp
    geo/geodesy.cpp
    geo/height_field.cpp
    geo/synthetic_terrain.cpp
    guidance/guidance_law.cpp
    nav/error_state_filter.cpp
    nav/strapdown_ins.cpp
    nav/terrain_navigator.cpp
    sensors/nmea.cpp
    sensors/serial_gnss.cpp
    sensors/simulated_sensors.cpp
    simulation/simulator.cpp
    tercom.cpp
    terrain/contour_map.cpp
    terrain/tercom_correlator.cpp
    trajectory/route.cpp)

add_library(tercom::tercom ALIAS tercom)

target_include_directories(
    tercom
    PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

target_compile_features(tercom PUBLIC cxx_std_20)

tercom_target_warnings(tercom)
tercom_target_sanitizers(tercom)
tercom_target_coverage(tercom)
tercom_target_optimization(tercom)

set_target_properties(tercom PROPERTIES VERSION ${PROJECT_VERSION} EXPORT_NAME tercom)
