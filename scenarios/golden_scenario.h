#pragma once

// The single, canonical definition of the tuned "golden" software-in-the-loop
// scenario: the procedural terrain, the route, the truth start state, and the run
// configuration that the SITL example, the golden regression test, and the
// benchmark all share. Centralising it means a re-tune updates every consumer at
// once and they can never drift apart.
//
// This header is deliberately NOT part of the installed public API: it lives under
// scenarios/ (added to the include path of the in-repo executables only), not under
// include/, so it ships with the source but never installs alongside the library.

#include <cmath>
#include <utility>
#include <vector>

#include "tercom/geo/height_field.h"
#include "tercom/geo/synthetic_terrain.h"
#include "tercom/math/linear.h"
#include "tercom/nav/nav_state.h"
#include "tercom/simulation/simulator.h"
#include "tercom/trajectory/route.h"
#include "tercom/trajectory/waypoint.h"

namespace TERCOM { namespace Scenarios {

    // Cruise altitude (metres) and target ground speed (metres per second) held
    // across every leg of the golden route.
    inline constexpr double kGoldenAltitudeM = 1100.0;
    inline constexpr double kGoldenSpeedMps = 200.0;

    // A ~70 km square of procedural terrain at 100 m posts, elevations ranging
    // roughly 340-860 m, i.e. 600 +/- 260 (520 m peak-to-trough relief). The fixed
    // seed pins one specific, reproducible landscape.
    inline Geo::HeightField MakeGoldenTerrain() {
        Geo::FractalTerrainConfiguration configuration;
        configuration.cols = 700u;
        configuration.rows = 700u;
        configuration.cell_size_m = 100.0;
        configuration.base_elevation_m = 600.0;
        configuration.relief_m = 520.0;
        configuration.feature_wavelength_m = 2500.0;
        configuration.octaves = 7;
        configuration.seed = 2024u;
        return Geo::GenerateFractalTerrain(configuration);
    }

    // The three-leg route (four waypoints) flown across the terrain at cruise.
    inline Trajectory::Route MakeGoldenRoute() {
        std::vector<Trajectory::Waypoint> waypoints;
        waypoints.push_back(Trajectory::Waypoint{Math::Vec3{5000.0, 5000.0, kGoldenAltitudeM}, kGoldenSpeedMps});
        waypoints.push_back(Trajectory::Waypoint{Math::Vec3{25000.0, 15000.0, kGoldenAltitudeM}, kGoldenSpeedMps});
        waypoints.push_back(Trajectory::Waypoint{Math::Vec3{45000.0, 40000.0, kGoldenAltitudeM}, kGoldenSpeedMps});
        waypoints.push_back(Trajectory::Waypoint{Math::Vec3{60000.0, 60000.0, kGoldenAltitudeM}, kGoldenSpeedMps});
        return Trajectory::Route{std::move(waypoints)};
    }

    // Truth starts on the first waypoint, heading toward the second at cruise speed.
    inline Nav::NavState MakeGoldenTruth() {
        const double leg_east = 25000.0 - 5000.0;
        const double leg_north = 15000.0 - 5000.0;
        const double leg_length = std::sqrt((leg_east * leg_east) + (leg_north * leg_north));
        Nav::NavState truth;
        truth.position_enu_m = Math::Vec3{5000.0, 5000.0, kGoldenAltitudeM};
        truth.velocity_enu_mps = Math::Vec3{kGoldenSpeedMps * leg_east / leg_length, kGoldenSpeedMps * leg_north / leg_length, 0.0};
        return truth;
    }

    // The run configuration: only the duration is moved off the library defaults.
    inline Simulation::Configuration MakeGoldenConfiguration() {
        Simulation::Configuration configuration;
        configuration.duration_s = 400.0;
        return configuration;
    }

}} // namespace TERCOM::Scenarios
