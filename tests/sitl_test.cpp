#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "golden_scenario.h"
#include "tercom/math/attitude.h"
#include "tercom/math/linear.h"
#include "tercom/nav/nav_state.h"
#include "tercom/simulation/simulator.h"
#include "tercom/trajectory/route.h"
#include "tercom/trajectory/waypoint.h"

namespace {

    // Build the tuned golden scenario from its single canonical definition (shared
    // with the example and the benchmark). Each call regenerates the deterministic
    // terrain, so the result is self-contained and bit-for-bit identical run to run.
    TERCOM::Simulation::Simulator MakeGoldenSimulator() {
        return TERCOM::Simulation::Simulator(
            TERCOM::Scenarios::MakeGoldenTerrain(), TERCOM::Scenarios::MakeGoldenTruth(), TERCOM::Scenarios::MakeGoldenRoute(), TERCOM::Scenarios::MakeGoldenConfiguration());
    }

} // namespace

TEST(SitlGoldenScenario, ConvergesToTunedThresholds) {
    TERCOM::Simulation::Simulator simulator = MakeGoldenSimulator();
    const TERCOM::Simulation::Result result = simulator.Run();

    // The simulation must have produced a trajectory.
    EXPECT_FALSE(result.steps.empty());

    // The terrain-referenced filter should accept a healthy stream of fixes.
    EXPECT_GT(result.accepted_fixes, 50u);

    // Tuned build yields final ~16.5 m, rms_after_first_fix ~60.5 m.
    EXPECT_LT(result.final_horizontal_error_m, 60.0);
    EXPECT_LT(result.rms_error_after_first_fix_m, 75.0);
}

TEST(SitlGoldenScenario, SteadyStateErrorBounded) {
    TERCOM::Simulation::Simulator simulator = MakeGoldenSimulator();
    const TERCOM::Simulation::Result result = simulator.Run();

    ASSERT_FALSE(result.steps.empty());

    // After the inertial solution has had time to settle (t > 60 s), the
    // horizontal error should stay small in both RMS and peak terms.
    double sum_squared = 0.0;
    std::size_t count = 0;
    double peak = 0.0;
    for (const TERCOM::Simulation::Step& step : result.steps) {
        if (step.time_s > 60.0) {
            sum_squared += step.horizontal_error_m * step.horizontal_error_m;
            ++count;
            if (step.horizontal_error_m > peak) {
                peak = step.horizontal_error_m;
            }
        }
    }

    ASSERT_GT(count, 0u);
    const double steady_state_rms = std::sqrt(sum_squared / static_cast<double>(count));
    EXPECT_LT(steady_state_rms, 60.0);
    EXPECT_LT(peak, 150.0);
}

TEST(SitlGoldenScenario, IsDeterministic) {
    TERCOM::Simulation::Simulator simulator_a = MakeGoldenSimulator();
    const TERCOM::Simulation::Result result_a = simulator_a.Run();

    TERCOM::Simulation::Simulator simulator_b = MakeGoldenSimulator();
    const TERCOM::Simulation::Result result_b = simulator_b.Run();

    // Two independently constructed, identically configured runs must agree
    // bit-for-bit: the terrain generator and the whole pipeline are deterministic.
    EXPECT_DOUBLE_EQ(result_a.final_horizontal_error_m, result_b.final_horizontal_error_m);
    EXPECT_DOUBLE_EQ(result_a.rms_horizontal_error_m, result_b.rms_horizontal_error_m);
    EXPECT_EQ(result_a.accepted_fixes, result_b.accepted_fixes);
}

TEST(SitlGoldenScenario, FlightModelAttitudeForwardMatchesVelocity) {
    // The simulator builds truth attitude as QuaternionFromEuler(bank, -fpa,
    // pi/2 - heading) from a compass heading. This verifies that convention yields a
    // body forward axis (+x) aligned with the velocity direction for a representative
    // climbing, banked, north-easterly heading -- i.e. the stored attitude is the
    // true FLU-to-ENU orientation, not a self-consistent placeholder. (The navigation
    // results held either way, but the attitude field must mean what it says.)
    const double heading = TERCOM::Math::DegToRad(30.0);           // compass: 30 deg east of north
    const double flight_path_angle = TERCOM::Math::DegToRad(10.0); // climbing
    const double bank = TERCOM::Math::DegToRad(15.0);
    const double speed = 200.0;

    const double horizontal = speed * std::cos(flight_path_angle);
    const TERCOM::Math::Vec3 velocity{horizontal * std::sin(heading), horizontal * std::cos(heading), speed * std::sin(flight_path_angle)};

    const TERCOM::Math::Quaternion attitude = TERCOM::Math::QuaternionFromEuler(bank, -flight_path_angle, (TERCOM::Math::kPi / 2.0) - heading);
    const TERCOM::Math::Vec3 forward = TERCOM::Math::Rotate(attitude, TERCOM::Math::Vec3{1.0, 0.0, 0.0});

    // Forward (body +x) must be parallel to the velocity direction (normalised dot
    // product of 1). Bank is a roll about that forward axis, so it does not move it.
    const TERCOM::Math::Vec3 velocity_direction = velocity * (1.0 / TERCOM::Math::Norm(velocity));
    EXPECT_NEAR(TERCOM::Math::Dot(forward, velocity_direction), 1.0, 1e-9);
}

TEST(SitlGoldenScenario, StepAndSummaryRecordsAreConsistent) {
    TERCOM::Simulation::Simulator simulator = MakeGoldenSimulator();
    const TERCOM::Simulation::Result result = simulator.Run();

    ASSERT_FALSE(result.steps.empty());
    // The diagnostics below only matter if fixes were actually applied.
    ASSERT_GT(result.accepted_fixes, 0u);

    std::size_t fix_applied_count = 0;
    double expected_max = 0.0;
    bool saw_positive_diagnostics = false;
    for (const TERCOM::Simulation::Step& step : result.steps) {
        // The recorded scalar errors must match the recorded ENU vectors exactly
        // (computed the same way the simulator does), catching a truth/estimate swap
        // or a sign flip on either channel.
        const double error_east = step.estimated_position_enu_m.x - step.truth_position_enu_m.x;
        const double error_north = step.estimated_position_enu_m.y - step.truth_position_enu_m.y;
        EXPECT_DOUBLE_EQ(step.horizontal_error_m, std::sqrt((error_east * error_east) + (error_north * error_north)));
        EXPECT_DOUBLE_EQ(step.altitude_error_m, step.estimated_position_enu_m.z - step.truth_position_enu_m.z);

        // The filter's reported horizontal one-sigma is always strictly positive
        // (the square root of averaged East/North variances of a positive-definite
        // covariance), guarding against a regression wiring the wrong source (zero, a
        // constant, or AltitudeStdM()) into the recorded field.
        EXPECT_GT(step.estimated_horizontal_std_m, 0.0);

        // The TERCOM diagnostics stay in their valid ranges on every step.
        EXPECT_GE(step.tercom_mad_m, 0.0);
        EXPECT_GE(step.tercom_confidence, 0.0);
        EXPECT_LE(step.tercom_confidence, 1.0);

        if (step.tercom_fix_applied) {
            ++fix_applied_count;
            // An applied fix carries real diagnostics, not the zero defaults.
            if (step.tercom_mad_m > 0.0 && step.tercom_confidence > 0.0) {
                saw_positive_diagnostics = true;
            }
        }
        expected_max = std::max(expected_max, step.horizontal_error_m);
    }

    // Each accepted fix is flagged on exactly one step.
    EXPECT_EQ(fix_applied_count, result.accepted_fixes);
    EXPECT_TRUE(saw_positive_diagnostics);
    // The summary maximum equals the per-step maximum.
    EXPECT_DOUBLE_EQ(result.max_horizontal_error_m, expected_max);
    // accepted + rejected must reconstruct the attempt total exactly. rejected_fixes
    // is the unsigned difference fix_attempts - accepted_fixes; had it underflowed it
    // would wrap near SIZE_MAX and break this equality (and blow past the step count).
    EXPECT_EQ(result.accepted_fixes + result.rejected_fixes, result.fix_attempts);
    EXPECT_GE(result.fix_attempts, result.accepted_fixes);
    EXPECT_LE(result.fix_attempts, result.steps.size());
}

TEST(SitlGoldenScenario, NoFixFallbackUsesOverallRms) {
    // A run too short for any radar batch to complete accepts no fix, so the
    // "after first fix" RMS must fall back to the overall RMS rather than divide by
    // zero or report garbage.
    TERCOM::Simulation::Configuration configuration = TERCOM::Scenarios::MakeGoldenConfiguration();
    configuration.duration_s = 0.5; // far shorter than one correlation batch
    TERCOM::Simulation::Simulator simulator(TERCOM::Scenarios::MakeGoldenTerrain(), TERCOM::Scenarios::MakeGoldenTruth(), TERCOM::Scenarios::MakeGoldenRoute(), configuration);
    const TERCOM::Simulation::Result result = simulator.Run();

    ASSERT_FALSE(result.steps.empty());
    EXPECT_EQ(result.accepted_fixes, 0u);
    EXPECT_DOUBLE_EQ(result.rms_error_after_first_fix_m, result.rms_horizontal_error_m);
}

TEST(SitlGoldenScenario, OffMapFlightSkipsRadarAndStillCompletes) {
    // A route that leaves the modelled terrain (0..69900 m) exercises the simulator's
    // off-map radar skip: where the ground elevation is undefined, no radar sample is
    // taken, but the run must still complete and record every IMU step rather than
    // dereferencing the empty optional or feeding the navigator a bogus clearance.
    TERCOM::Simulation::Configuration configuration = TERCOM::Scenarios::MakeGoldenConfiguration();
    configuration.duration_s = 200.0;

    std::vector<TERCOM::Trajectory::Waypoint> waypoints;
    waypoints.push_back(TERCOM::Trajectory::Waypoint{TERCOM::Math::Vec3{60000.0, 60000.0, 1100.0}, 200.0});
    waypoints.push_back(TERCOM::Trajectory::Waypoint{TERCOM::Math::Vec3{120000.0, 120000.0, 1100.0}, 200.0}); // far off the map
    TERCOM::Trajectory::Route route{std::move(waypoints)};

    TERCOM::Nav::NavState truth;
    truth.position_enu_m = {60000.0, 60000.0, 1100.0};
    const double component = 200.0 / std::sqrt(2.0); // 200 m/s heading north-east
    truth.velocity_enu_mps = {component, component, 0.0};

    TERCOM::Simulation::Simulator simulator(TERCOM::Scenarios::MakeGoldenTerrain(), truth, std::move(route), configuration);
    const TERCOM::Simulation::Result result = simulator.Run();

    ASSERT_FALSE(result.steps.empty());
    EXPECT_EQ(result.steps.size(), static_cast<std::size_t>(std::llround(configuration.duration_s * configuration.imu_rate_hz)));
}

TEST(SitlGoldenScenario, HighSensorRateClampsInterval) {
    // A sensor rate at or above the IMU rate would round the read interval to zero;
    // the clamp keeps it at one read per IMU step, so the run completes (no modulo by
    // zero) and produces exactly one step per IMU tick.
    TERCOM::Simulation::Configuration configuration = TERCOM::Scenarios::MakeGoldenConfiguration();
    configuration.duration_s = 1.0;
    configuration.imu_rate_hz = 100.0;
    configuration.barometer_rate_hz = 200.0; // exceeds the IMU rate: interval clamps to 1
    configuration.radar_rate_hz = 100.0;     // equals the IMU rate: interval stays 1
    TERCOM::Simulation::Simulator simulator(TERCOM::Scenarios::MakeGoldenTerrain(), TERCOM::Scenarios::MakeGoldenTruth(), TERCOM::Scenarios::MakeGoldenRoute(), configuration);

    const TERCOM::Simulation::Result result = simulator.Run();
    EXPECT_EQ(result.steps.size(), 100u); // 1 s at 100 Hz
}
