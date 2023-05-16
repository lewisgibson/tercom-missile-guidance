#include <gtest/gtest.h>

#include <vector>

#include "tercom/geo/synthetic_terrain.h"
#include "tercom/guidance/guidance_law.h"
#include "tercom/nav/nav_state.h"
#include "tercom/sensors/measurements.h"
#include "tercom/tercom.h"
#include "tercom/trajectory/correction.h"
#include "tercom/trajectory/route.h"
#include "tercom/trajectory/waypoint.h"

namespace {

    using namespace TERCOM;

    GuidanceSystem MakeSystem() {
        Geo::FractalTerrainConfiguration parameters;
        parameters.cols = 256;
        parameters.rows = 256;
        parameters.cell_size_m = 100.0;
        parameters.seed = 5;
        Geo::HeightField terrain = Geo::GenerateFractalTerrain(parameters);

        std::vector<Trajectory::Waypoint> waypoints = {
            {{2000.0, 2000.0, 1000.0}, 200.0},
            {{12000.0, 2000.0, 1000.0}, 200.0},
        };
        Trajectory::Route route(waypoints);

        Nav::NavState initial;
        initial.position_enu_m = {2000.0, 2000.0, 1000.0};
        initial.velocity_enu_mps = {200.0, 0.0, 0.0};
        return GuidanceSystem(Terrain::ContourMap(terrain), initial, std::move(route));
    }

    TEST(GuidanceSystem, ExposesStateRouteAndUncertainty) {
        GuidanceSystem system = MakeSystem();

        EXPECT_FALSE(system.ReachedDestination());
        EXPECT_EQ(system.Route().Waypoints().size(), 2u);
        EXPECT_GT(system.HorizontalStdM(), 0.0);
        EXPECT_GT(system.AltitudeStdM(), 0.0);
        EXPECT_NEAR(system.EstimatedState().position_enu_m.x, 2000.0, 1e-9);
        // Navigator accessor is reachable and starts with no fixes.
        EXPECT_EQ(system.Navigator().AcceptedFixCount(), 0u);
    }

    TEST(GuidanceSystem, CalculateCorrectionProducesFiniteCommands) {
        GuidanceSystem system = MakeSystem();
        const Trajectory::Correction correction = system.CalculateCorrection();
        EXPECT_TRUE(std::isfinite(correction.roll_command_rad));
        EXPECT_TRUE(std::isfinite(correction.pitch_command_rad));
        EXPECT_GE(correction.throttle, 0.0);
        EXPECT_LE(correction.throttle, 1.0);
    }

    TEST(GuidanceSystem, CalculateCorrectionAdvancesRouteToCompletion) {
        // Place the initial estimate on the final waypoint so a single
        // CalculateCorrection captures it through the facade and the route reports
        // complete. This exercises the facade's route advancement and the
        // ReachedDestination()==true path that the other tests never reach.
        Geo::FractalTerrainConfiguration parameters;
        parameters.cols = 256;
        parameters.rows = 256;
        parameters.cell_size_m = 100.0;
        parameters.seed = 5;
        Geo::HeightField terrain = Geo::GenerateFractalTerrain(parameters);

        std::vector<Trajectory::Waypoint> waypoints = {
            {{2000.0, 2000.0, 1000.0}, 200.0},
            {{12000.0, 2000.0, 1000.0}, 200.0},
        };
        Trajectory::Route route(waypoints);

        Nav::NavState initial;
        initial.position_enu_m = {12000.0, 2000.0, 1000.0}; // sitting on the final waypoint
        initial.velocity_enu_mps = {200.0, 0.0, 0.0};
        GuidanceSystem system(Terrain::ContourMap(terrain), initial, std::move(route));

        EXPECT_FALSE(system.ReachedDestination());
        system.CalculateCorrection();
        EXPECT_TRUE(system.ReachedDestination());
    }

    TEST(GuidanceSystem, CalculateCorrectionMatchesStandaloneGuidanceForOffTrackState) {
        // An off-track estimate must yield a correcting bank, and that bank must match
        // a standalone GuidanceLaw fed the same estimate and an equivalent route. This
        // proves CalculateCorrection feeds the live navigator estimate into the
        // facade's own route, not stale or default state.
        Geo::FractalTerrainConfiguration parameters;
        parameters.cols = 256;
        parameters.rows = 256;
        parameters.cell_size_m = 100.0;
        parameters.seed = 5;
        Geo::HeightField terrain = Geo::GenerateFractalTerrain(parameters);

        const std::vector<Trajectory::Waypoint> waypoints = {
            {{2000.0, 2000.0, 1000.0}, 200.0},
            {{12000.0, 2000.0, 1000.0}, 200.0},
        };
        Trajectory::Route route(waypoints);

        Nav::NavState initial;
        initial.position_enu_m = {4000.0, 2300.0, 1000.0}; // 300 m north of the eastbound leg
        initial.velocity_enu_mps = {200.0, 0.0, 0.0};
        GuidanceSystem system(Terrain::ContourMap(terrain), initial, route);

        const Nav::NavState estimate = system.EstimatedState();
        const Trajectory::Correction actual = system.CalculateCorrection();

        Guidance::GuidanceLaw reference{Guidance::GuidanceConfiguration{}};
        Trajectory::Route reference_route(waypoints);
        const Trajectory::Correction expected = reference.Compute(estimate, reference_route);

        EXPECT_NEAR(actual.roll_command_rad, expected.roll_command_rad, 1e-9);
        EXPECT_GT(actual.roll_command_rad, 0.0); // north offset -> positive correcting bank
    }

    TEST(GuidanceSystem, FeedingSamplesAdvancesTheEstimate) {
        GuidanceSystem system = MakeSystem();

        // A short burst of straight, level IMU samples advances time and the INS.
        const Sensors::ImuSample level{{0.0, 0.0, 0.0}, {0.0, 0.0, 9.80665}, 0.0};
        for (int i = 1; i <= 10; ++i) {
            Sensors::ImuSample sample = level;
            sample.time_s = static_cast<double>(i) * 0.01;
            system.OnImu(sample);
        }
        EXPECT_GT(system.EstimatedState().time_s, 0.0);
    }

    TEST(GuidanceSystem, IgnoresInvalidBarometerAndRadar) {
        GuidanceSystem system = MakeSystem();

        Sensors::BarometricSample bad_baro;
        bad_baro.valid = false;
        system.OnBarometer(bad_baro); // early return, no update

        Sensors::RadarAltitudeSample bad_radar;
        bad_radar.valid = false;
        system.OnRadarAltimeter(bad_radar); // early return

        // A valid radar sample before any barometer is also ignored (no baseline).
        Sensors::RadarAltitudeSample radar;
        radar.valid = true;
        radar.clearance_m = 400.0;
        radar.time_s = 0.1;
        system.OnRadarAltimeter(radar);

        EXPECT_EQ(system.Navigator().FixAttemptCount(), 0u);
        EXPECT_EQ(system.Navigator().AcceptedFixCount(), 0u);
    }

} // namespace
