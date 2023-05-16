// Micro-benchmarks for the performance-critical navigation paths.
//
// This is a deliberately small, dependency-free harness (just <chrono>): it runs
// each operation many times, reports the average wall-clock cost, and for the full
// software-in-the-loop run also reports the real-time factor (how many seconds of
// simulated flight are computed per second of CPU time). Build it with an optimized
// preset, for example:
//
//   cmake --preset release -DTERCOM_BUILD_BENCHMARKS=ON
//   cmake --build build/release
//   ./build/release/bin/tercom-benchmark
//
// The numbers are indicative, not a statistical benchmark framework; they exist to
// catch large performance regressions and to document where the time goes.

#include <chrono>
#include <cstdio>
#include <vector>

#include "golden_scenario.h"
#include "tercom/geo/height_field.h"
#include "tercom/nav/error_state_filter.h"
#include "tercom/nav/strapdown_ins.h"
#include "tercom/simulation/simulator.h"
#include "tercom/terrain/tercom_correlator.h"

namespace {

    using Clock = std::chrono::steady_clock;

    // Average nanoseconds per call of the callable over the given number of
    // repetitions. Taken by const reference because it is invoked repeatedly (a
    // forwarding reference would be wrong: the callable cannot be moved-from N times).
    template <class Function>
    double AverageNanoseconds(const Function& function, int repetitions) {
        const auto start = Clock::now();
        for (int i = 0; i < repetitions; ++i) {
            function();
        }
        const auto stop = Clock::now();
        return std::chrono::duration<double, std::nano>(stop - start).count() / static_cast<double>(repetitions);
    }

    // The tuned demonstration terrain, shared by several benchmarks: the same
    // canonical golden terrain the example and the regression test use.
    TERCOM::Geo::HeightField MakeTerrain() {
        return TERCOM::Scenarios::MakeGoldenTerrain();
    }

} // namespace

int main() {
    using namespace TERCOM;

    std::printf("%-44s %14s %16s\n", "operation", "average", "throughput");
    std::printf("%-44s %14s %16s\n", "---------", "-------", "----------");

    // Terrain generation: one-off cost when a synthetic map is created.
    const double terrain_ns = AverageNanoseconds(
        [] {
            volatile auto terrain = MakeTerrain();
            (void) terrain;
        },
        5);
    std::printf("%-44s %11.2f ms\n", "generate terrain (700x700, 7 octaves)", terrain_ns / 1e6);

    const Geo::HeightField terrain = MakeTerrain();

    // Strapdown INS integration of one IMU sample.
    {
        Nav::NavState initial;
        initial.velocity_enu_mps = {200.0, 0.0, 0.0};
        Nav::StrapdownIns ins(initial);
        Sensors::ImuSample sample;
        sample.specific_force_mps2 = {0.0, 0.0, Math::kStandardGravity};
        double time = 0.0;
        const double step_ns = AverageNanoseconds(
            [&] {
                time += 0.01;
                sample.time_s = time;
                ins.Step(sample);
            },
            1000000);
        std::printf("%-44s %11.1f ns %12.2f M/s\n", "strapdown INS step", step_ns, 1000.0 / step_ns);
    }

    // Error-state Kalman filter covariance prediction (the per-IMU-sample cost).
    {
        Nav::ErrorStateFilter filter;
        const Math::Mat3 body_to_nav = Math::Mat3::Identity();
        const double predict_ns = AverageNanoseconds(
            [&] {
                filter.Predict(0.01, body_to_nav);
            },
            1000000);
        std::printf("%-44s %11.1f ns %12.2f M/s\n", "ESKF covariance prediction", predict_ns, 1000.0 / predict_ns);
    }

    // TERCOM correlation: a bounded search (converged filter) and a full-radius search.
    {
        Terrain::TercomCorrelator correlator(terrain);
        std::vector<Terrain::TerrainProfileSample> profile;
        for (int i = 0; i < 60; ++i) {
            const Math::Vec3 position{15000.0 + (static_cast<double>(i) * 100.0), 15000.0, 0.0};
            const std::optional<double> elevation = terrain.ElevationAt(position.x + 80.0, position.y - 50.0);
            profile.push_back({position, elevation.value_or(0.0)});
        }
        const Math::Vec3 predicted{15000.0, 15000.0, 0.0};
        const double bounded_ns = AverageNanoseconds(
            [&] {
                volatile auto fix = correlator.Correlate(profile, predicted, 30.0);
                (void) fix;
            },
            2000);
        const double full_ns = AverageNanoseconds(
            [&] {
                volatile auto fix = correlator.Correlate(profile, predicted);
                (void) fix;
            },
            1000);
        std::printf("%-44s %11.3f ms\n", "TERCOM correlate (bounded search)", bounded_ns / 1e6);
        std::printf("%-44s %11.3f ms\n", "TERCOM correlate (full-radius search)", full_ns / 1e6);
    }

    // The full closed-loop software-in-the-loop run.
    {
        // The same canonical golden scenario the example and regression test run.
        const Trajectory::Route route = Scenarios::MakeGoldenRoute();
        const Nav::NavState truth = Scenarios::MakeGoldenTruth();
        const Simulation::Configuration configuration = Scenarios::MakeGoldenConfiguration();

        const double run_ns = AverageNanoseconds(
            [&] {
                Simulation::Simulator simulator(terrain, truth, route, configuration);
                volatile auto result = simulator.Run();
                (void) result;
            },
            5);
        const double run_ms = run_ns / 1e6;
        std::printf("%-44s %11.2f ms %9.0f x real time\n", "full SITL run (400 s, 40000 steps)", run_ms, configuration.duration_s * 1000.0 / run_ms);
    }

    return 0;
}
