// Real-time demonstration of the terrain-referenced guidance API.
//
// A ground-truth vehicle cruises straight and level along a route leg. On every
// tick the program synthesises the inertial, barometric, and radar-altimeter
// measurements that vehicle would produce, feeds them to a GuidanceSystem, and
// prints the estimated state, the horizontal error against truth, and the control
// correction the guidance law returns. The estimate starts with a deliberate
// position offset; watch the terrain fixes pull it back onto the truth track.
//
// This is the small end-to-end loop. For rigorous, repeatable navigation
// statistics see the sitl-simulation example, which drives Simulation::Simulator.

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <optional>
#include <thread>
#include <vector>

#include "tercom/geo/synthetic_terrain.h"
#include "tercom/nav/nav_state.h"
#include "tercom/sensors/simulated_sensors.h"
#include "tercom/tercom.h"
#include "tercom/trajectory/route.h"

namespace {
    // A signal handler can only safely touch a volatile sig_atomic_t flag, which
    // must have static storage; the non-const-global check does not apply here.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    volatile std::sig_atomic_t g_stop = 0;
    void HandleSignal(int /*signal*/) {
        g_stop = 1;
    }
    // Standard gravity magnitude (the shared library constant), m/s^2.
    constexpr double kGravity = TERCOM::Math::kStandardGravity;
} // namespace

int main() {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    using namespace TERCOM;

    // Onboard reference map and the world flown over are the same terrain here.
    Geo::FractalTerrainConfiguration terrain_parameters;
    terrain_parameters.cols = 500;
    terrain_parameters.rows = 500;
    terrain_parameters.cell_size_m = 100.0;
    terrain_parameters.base_elevation_m = 500.0;
    terrain_parameters.relief_m = 400.0;
    terrain_parameters.feature_wavelength_m = 2500.0;
    terrain_parameters.octaves = 7;
    terrain_parameters.seed = 7;
    const Geo::HeightField terrain = Geo::GenerateFractalTerrain(terrain_parameters);

    // Straight, level cruise eastward at 200 m/s and 1000 m altitude.
    constexpr double kSpeed = 200.0;
    constexpr double kAltitude = 1000.0;
    const std::vector<Trajectory::Waypoint> waypoints = {
        {{5000.0, 12000.0, kAltitude}, kSpeed},
        {{45000.0, 12000.0, kAltitude}, kSpeed},
    };
    const Trajectory::Route route(waypoints);

    Nav::NavState truth;
    truth.position_enu_m = {5000.0, 12000.0, kAltitude};
    truth.velocity_enu_mps = {kSpeed, 0.0, 0.0};
    // Heading east. The library yaw is right-handed (counter-clockwise from east),
    // so a compass bearing of 90 degrees converts to yaw = pi/2 - 90 deg = 0, the
    // identity attitude, whose body forward axis points along ENU +x (east) to match
    // the eastward velocity above.
    truth.attitude = Math::QuaternionFromEuler(0.0, 0.0, (Math::kPi / 2.0) - Math::DegToRad(90.0));

    // The estimator starts offset from truth and ignorant of the true biases.
    Nav::NavState initial_estimate = truth;
    initial_estimate.position_enu_m.x += 100.0;
    initial_estimate.position_enu_m.y -= 80.0;

    GuidanceSystem guidance(Terrain::ContourMap(terrain), initial_estimate, route);

    Sensors::SimulatedImu imu;
    Sensors::SimulatedBarometer barometer;
    Sensors::SimulatedRadarAltimeter radar;

    // For straight, level, constant-velocity flight the specific force is purely
    // the reaction to gravity and the angular rate is zero.
    const Math::Vec3 specific_force_body{0.0, 0.0, kGravity};
    const Math::Vec3 angular_rate_body{0.0, 0.0, 0.0};

    constexpr double kImuRateHz = 100.0;
    constexpr double kDt = 1.0 / kImuRateHz;
    constexpr int kBarometerEvery = 10; // 10 Hz
    constexpr int kRadarEvery = 5;      // 20 Hz
    constexpr double kDurationS = 180.0;
    const auto total_steps = static_cast<int>(kDurationS * kImuRateHz);

    std::printf("%8s %13s %13s %15s %15s %9s %8s %8s %9s\n", "time_s", "truth_east_m", "truth_north_m", "estimate_east_m", "estimate_north_m", "error_m", "roll_rad", "pitch_rad",
        "throttle");

    std::size_t last_fix_count = 0;
    for (int step = 0; step < total_steps && g_stop == 0; ++step) {
        const double time = static_cast<double>(step + 1) * kDt;

        // Advance ground truth (straight, level, constant velocity).
        truth.position_enu_m += truth.velocity_enu_mps * kDt;

        imu.SetTruth(angular_rate_body, specific_force_body, time);
        guidance.OnImu(imu.Read());

        if (step % kBarometerEvery == 0) {
            barometer.SetTruth(truth.position_enu_m.z, time);
            guidance.OnBarometer(barometer.Read());
        }
        if (step % kRadarEvery == 0) {
            const std::optional<double> ground = terrain.ElevationAt(truth.position_enu_m.x, truth.position_enu_m.y);
            if (ground.has_value()) {
                radar.SetTruth(truth.position_enu_m.z - *ground, time);
                guidance.OnRadarAltimeter(radar.Read());
            }
        }

        // Print once a second.
        if (step % static_cast<int>(kImuRateHz) == 0) {
            const Nav::NavState& estimate = guidance.EstimatedState();
            const Trajectory::Correction correction = guidance.CalculateCorrection();
            const double error_east = estimate.position_enu_m.x - truth.position_enu_m.x;
            const double error_north = estimate.position_enu_m.y - truth.position_enu_m.y;
            const double horizontal_error = std::sqrt((error_east * error_east) + (error_north * error_north));
            const std::size_t fixes = guidance.Navigator().AcceptedFixCount();
            const char fix_marker = (fixes > last_fix_count) ? '*' : ' ';
            last_fix_count = fixes;

            std::printf("%8.0f %13.0f %13.0f %15.0f %15.0f %8.1f%c %8.3f %8.3f %9.2f\n", time, truth.position_enu_m.x, truth.position_enu_m.y, estimate.position_enu_m.x,
                estimate.position_enu_m.y, horizontal_error, fix_marker, correction.roll_command_rad, correction.pitch_command_rad, correction.throttle);

            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        if (guidance.ReachedDestination()) {
            std::printf("Route complete.\n");
            break;
        }
    }

    std::printf("Accepted %zu terrain fixes over the run ('*' marks a fix).\n", guidance.Navigator().AcceptedFixCount());
    return 0;
}
