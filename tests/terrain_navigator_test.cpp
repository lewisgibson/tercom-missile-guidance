#include <gtest/gtest.h>

#include <cstddef>
#include <optional>

#include "tercom/geo/height_field.h"
#include "tercom/geo/synthetic_terrain.h"
#include "tercom/math/linear.h"
#include "tercom/nav/nav_state.h"
#include "tercom/nav/terrain_navigator.h"
#include "tercom/sensors/measurements.h"
#include "tercom/terrain/contour_map.h"

namespace {

    using namespace TERCOM;

    // A small rough reference map for the navigator under test.
    Geo::HeightField MakeTerrain() {
        Geo::FractalTerrainConfiguration parameters;
        parameters.cols = 256;
        parameters.rows = 256;
        parameters.cell_size_m = 100.0;
        parameters.seed = 5;
        return Geo::GenerateFractalTerrain(parameters);
    }

    Nav::NavState MakeInitial() {
        Nav::NavState state;
        state.position_enu_m = {5000.0, 5000.0, 1000.0};
        state.velocity_enu_mps = {200.0, 0.0, 0.0};
        return state;
    }

    // A relief-rich reference map (the correlator tests' proven parameters) so a
    // recovered fix clears the roughness and confidence gates.
    Geo::HeightField MakeRoughTerrain() {
        Geo::FractalTerrainConfiguration configuration;
        configuration.cols = 400;
        configuration.rows = 400;
        configuration.cell_size_m = 100.0;
        configuration.base_elevation_m = 500.0;
        configuration.relief_m = 400.0;
        configuration.seed = 0xC0FFEEULL;
        return Geo::GenerateFractalTerrain(configuration);
    }

    // Feed one straight, level, constant-velocity IMU step at the given time. The
    // specific force is the upward reaction to gravity and the angular rate is zero,
    // so the INS advances along track at its current velocity without turning.
    void StepLevelFlight(Nav::TerrainReferencedNavigator& navigator, double time_s) {
        Sensors::ImuSample imu;
        imu.time_s = time_s;
        imu.angular_rate_radps = Math::Vec3{0.0, 0.0, 0.0};
        imu.specific_force_mps2 = Math::Vec3{0.0, 0.0, Math::kStandardGravity};
        navigator.OnImu(imu);
    }

    // A radar sample whose clearance reproduces the map elevation directly beneath
    // the navigator's current INS position, so the measured strip matches the map at
    // a zero horizontal offset (which the correlator recovers).
    Sensors::RadarAltitudeSample MatchingRadarSample(
        const Nav::TerrainReferencedNavigator& navigator, const Geo::HeightField& terrain, double barometric_altitude_m, double time_s) {
        const Math::Vec3 position = navigator.EstimatedState().position_enu_m;
        const std::optional<double> elevation = terrain.ElevationAt(position.x, position.y);
        EXPECT_TRUE(elevation.has_value());
        Sensors::RadarAltitudeSample radar;
        radar.valid = true;
        radar.time_s = time_s;
        radar.clearance_m = barometric_altitude_m - elevation.value_or(0.0);
        return radar;
    }

    TEST(TerrainReferencedNavigator, InitialDiagnosticsAreSane) {
        // The navigator is non-movable, so construct it in place.
        const Nav::TerrainReferencedNavigator navigator(Terrain::ContourMap(MakeTerrain()), MakeInitial());

        EXPECT_EQ(navigator.FixAttemptCount(), 0u);
        EXPECT_EQ(navigator.AcceptedFixCount(), 0u);
        EXPECT_EQ(navigator.DownweightedFixCount(), 0u);
        EXPECT_EQ(navigator.ProfileSize(), 0u);
        EXPECT_GT(navigator.HorizontalStdM(), 0.0);
        EXPECT_GT(navigator.AltitudeStdM(), 0.0);
    }

    TEST(TerrainReferencedNavigator, BarometerUpdateShrinksAltitudeStd) {
        Nav::TerrainReferencedNavigator navigator(Terrain::ContourMap(MakeTerrain()), MakeInitial());

        const double before = navigator.AltitudeStdM();
        Sensors::BarometricSample barometer;
        barometer.valid = true;
        barometer.altitude_m = 1000.0;
        barometer.time_s = 0.1;
        navigator.OnBarometer(barometer);

        EXPECT_LT(navigator.AltitudeStdM(), before);
    }

    TEST(TerrainReferencedNavigator, BarometerWithVerticalErrorMovesEstimateTowardMeasurement) {
        Nav::TerrainReferencedNavigator navigator(Terrain::ContourMap(MakeTerrain()), MakeInitial());

        const double initial_z = navigator.EstimatedState().position_enu_m.z; // 1000.0
        constexpr double kMeasuredAltitude = 1040.0;                          // 40 m above the INS estimate

        Sensors::BarometricSample barometer;
        barometer.valid = true;
        barometer.altitude_m = kMeasuredAltitude;
        barometer.time_s = 0.1;
        navigator.OnBarometer(barometer);

        const double after_z = navigator.EstimatedState().position_enu_m.z;
        // The estimate must move up toward the higher measurement (correct innovation
        // sign and the correction is actually applied)...
        EXPECT_GT(after_z, initial_z);
        // ...but a single update only closes a fraction of the gap, so no overshoot.
        EXPECT_LT(after_z, kMeasuredAltitude);
    }

    TEST(TerrainReferencedNavigator, RadarSamplesAccumulateIntoProfile) {
        Nav::TerrainReferencedNavigator navigator(Terrain::ContourMap(MakeTerrain()), MakeInitial());

        // A barometer reading is required before radar samples are used (the radar
        // path turns ground clearance into terrain elevation using the baro altitude).
        Sensors::BarometricSample barometer;
        barometer.valid = true;
        barometer.altitude_m = 1000.0;
        barometer.time_s = 0.1;
        navigator.OnBarometer(barometer);

        Sensors::RadarAltitudeSample radar;
        radar.valid = true;
        radar.clearance_m = 400.0;
        for (int i = 0; i < 5; ++i) {
            radar.time_s = 0.1 + (0.05 * static_cast<double>(i));
            navigator.OnRadarAltimeter(radar);
        }

        // Fewer than one batch of samples, so they are still buffered, not yet correlated.
        EXPECT_EQ(navigator.ProfileSize(), 5u);
        EXPECT_EQ(navigator.FixAttemptCount(), 0u);
    }

    TEST(TerrainReferencedNavigator, NonAdvancingImuMovesClockButDoesNotGrowCovariance) {
        Nav::TerrainReferencedNavigator navigator(Terrain::ContourMap(MakeTerrain()), MakeInitial());

        // First IMU step (dt > 0) advances both the INS clock and the filter
        // covariance, since Predict runs.
        StepLevelFlight(navigator, 0.1);
        const double std_after_first = navigator.HorizontalStdM();
        EXPECT_GT(std_after_first, 0.0);

        // Duplicate timestamp: dt == 0, so Predict is skipped. The clock is re-stamped
        // but the covariance must not grow.
        StepLevelFlight(navigator, 0.1);
        EXPECT_EQ(navigator.HorizontalStdM(), std_after_first);
        EXPECT_DOUBLE_EQ(navigator.EstimatedState().time_s, 0.1);

        // Out-of-order timestamp: dt < 0, so Predict is again skipped. The clock moves
        // to the earlier value but the covariance still must not grow.
        StepLevelFlight(navigator, 0.05);
        EXPECT_EQ(navigator.HorizontalStdM(), std_after_first);
        EXPECT_DOUBLE_EQ(navigator.EstimatedState().time_s, 0.05);
    }

    TEST(TerrainReferencedNavigator, FullBatchOverRecoverableTerrainAcceptsFixAndResetsProfile) {
        const Geo::HeightField terrain = MakeRoughTerrain();

        Nav::NavState initial;
        initial.position_enu_m = {8000.0, 8000.0, 1000.0};
        initial.velocity_enu_mps = {150.0, 150.0, 0.0}; // diagonal cruise, wings level
        Nav::TerrainReferencedNavigator navigator(Terrain::ContourMap(terrain), initial);

        constexpr double kBarometricAltitude = 1000.0;
        Sensors::BarometricSample barometer;
        barometer.valid = true;
        barometer.altitude_m = kBarometricAltitude;
        barometer.time_s = 0.0;
        navigator.OnBarometer(barometer);

        // Drive one full correlation batch: advance the INS, then take a radar sample
        // matching the terrain beneath it. The strip matches the map at zero offset.
        const Nav::NavigatorConfiguration defaults;
        constexpr double kStep = 0.5;
        for (std::size_t i = 0; i < defaults.correlation_batch_samples; ++i) {
            const double time_s = kStep * static_cast<double>(i + 1);
            StepLevelFlight(navigator, time_s);
            navigator.OnRadarAltimeter(MatchingRadarSample(navigator, terrain, kBarometricAltitude, time_s));
        }

        // The full strip was gathered, correlated, accepted, folded in, and reset.
        EXPECT_EQ(navigator.FixAttemptCount(), 1u);
        EXPECT_EQ(navigator.AcceptedFixCount(), 1u);
        EXPECT_EQ(navigator.DownweightedFixCount(), 0u);
        EXPECT_EQ(navigator.ProfileSize(), 0u);
        EXPECT_TRUE(navigator.LastFix().accepted);

        // A further radar sample begins a fresh strip.
        const double next_time = kStep * static_cast<double>(defaults.correlation_batch_samples + 1);
        StepLevelFlight(navigator, next_time);
        navigator.OnRadarAltimeter(MatchingRadarSample(navigator, terrain, kBarometricAltitude, next_time));
        EXPECT_EQ(navigator.ProfileSize(), 1u);
    }

    TEST(TerrainReferencedNavigator, InconsistentFixIsDownWeightedNotRejected) {
        const Geo::HeightField terrain = MakeRoughTerrain();

        Nav::NavState initial;
        initial.position_enu_m = {8000.0, 8000.0, 1000.0};
        initial.velocity_enu_mps = {150.0, 150.0, 0.0};

        // A practically-zero soft gate forces any accepted fix carrying a non-zero
        // innovation through the down-weight path rather than the clean-accept path.
        Nav::NavigatorConfiguration configuration;
        configuration.fix_gate_chi2 = 1.0e-6;
        Nav::TerrainReferencedNavigator navigator(Terrain::ContourMap(terrain), initial, configuration);

        constexpr double kBarometricAltitude = 1000.0;
        Sensors::BarometricSample barometer;
        barometer.valid = true;
        barometer.altitude_m = kBarometricAltitude;
        barometer.time_s = 0.0;
        navigator.OnBarometer(barometer);

        // Sample the terrain 100 m off the INS track on each axis so the correlator
        // recovers a confident fix displaced from the INS position. The fix passes the
        // correlator's gates (so it is accepted) but its non-zero innovation trips the
        // tiny chi-square gate, so it is folded in down-weighted.
        constexpr double kOffset = 100.0;
        constexpr double kStep = 0.5;
        for (std::size_t i = 0; i < configuration.correlation_batch_samples; ++i) {
            const double time_s = kStep * static_cast<double>(i + 1);
            StepLevelFlight(navigator, time_s);
            const Math::Vec3 position = navigator.EstimatedState().position_enu_m;
            const std::optional<double> elevation = terrain.ElevationAt(position.x + kOffset, position.y + kOffset);
            ASSERT_TRUE(elevation.has_value());
            Sensors::RadarAltitudeSample radar;
            radar.valid = true;
            radar.time_s = time_s;
            radar.clearance_m = kBarometricAltitude - elevation.value_or(0.0);
            navigator.OnRadarAltimeter(radar);
        }

        EXPECT_EQ(navigator.FixAttemptCount(), 1u);
        EXPECT_EQ(navigator.AcceptedFixCount(), 1u);
        EXPECT_GT(navigator.DownweightedFixCount(), 0u);
        EXPECT_TRUE(navigator.LastFix().accepted);
    }

    TEST(TerrainReferencedNavigator, FullBatchOverFlatProfileAttemptsButRejectsFix) {
        const Geo::HeightField terrain = MakeRoughTerrain();

        Nav::NavState initial;
        initial.position_enu_m = {8000.0, 8000.0, 1000.0};
        initial.velocity_enu_mps = {0.0, 0.0, 0.0}; // stationary: every sample at one point
        Nav::TerrainReferencedNavigator navigator(Terrain::ContourMap(terrain), initial);

        constexpr double kBarometricAltitude = 1000.0;
        Sensors::BarometricSample barometer;
        barometer.valid = true;
        barometer.altitude_m = kBarometricAltitude;
        barometer.time_s = 0.0;
        navigator.OnBarometer(barometer);

        // Without INS motion every sample lands on the same spot, so the strip is
        // flat. The batch still triggers a correlation attempt, but the correlator
        // rejects it for insufficient relief, so no fix is folded in.
        const Nav::NavigatorConfiguration defaults;
        const Sensors::RadarAltitudeSample flat = MatchingRadarSample(navigator, terrain, kBarometricAltitude, 0.0);
        for (std::size_t i = 0; i < defaults.correlation_batch_samples; ++i) {
            Sensors::RadarAltitudeSample radar = flat;
            radar.time_s = 0.1 * static_cast<double>(i + 1);
            navigator.OnRadarAltimeter(radar);
        }

        EXPECT_EQ(navigator.FixAttemptCount(), 1u);
        EXPECT_EQ(navigator.AcceptedFixCount(), 0u);
        EXPECT_EQ(navigator.ProfileSize(), 0u);
        EXPECT_FALSE(navigator.LastFix().accepted);
    }

} // namespace
