// Software-in-the-loop simulation of the full terrain-referenced navigation loop.
//
// It flies a ground-truth vehicle along a multi-leg route over procedurally
// generated terrain, corrupts the synthesised IMU, barometer, and radar-altimeter
// measurements with seeded bias and noise, and runs an independent GuidanceSystem
// on them. The estimator starts with a deliberate horizontal position error; the
// run shows TERCOM fixes bounding the inertial drift and converging the estimate.
//
// A summary is printed and the full time history is written to a CSV so the
// convergence can be plotted. The scenario is the tuned, known-good configuration
// and is fully deterministic: the same build produces the same numbers every run.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "golden_scenario.h"
#include "tercom/simulation/simulator.h"

int main(int argc, char** argv) {
    using namespace TERCOM;

    const std::string csv_path = (argc > 1) ? argv[1] : "sitl-convergence.csv";

    // The tuned, known-good golden scenario from its single canonical definition
    // (shared with the regression test and the benchmark): ~70 km of procedural
    // terrain, a three-leg cruise route, the truth start state, and a 400 s run.
    const Simulation::Configuration configuration = Scenarios::MakeGoldenConfiguration();
    Simulation::Simulator simulator(Scenarios::MakeGoldenTerrain(), Scenarios::MakeGoldenTruth(), Scenarios::MakeGoldenRoute(), configuration);
    const Simulation::Result result = simulator.Run();

    // Steady-state statistics, taken after the initial alignment has converged.
    constexpr double kSettleTimeS = 60.0;
    double settled_sum_squared = 0.0;
    double settled_max = 0.0;
    std::size_t settled_count = 0;
    for (const Simulation::Step& step : result.steps) {
        if (step.time_s > kSettleTimeS) {
            settled_sum_squared += step.horizontal_error_m * step.horizontal_error_m;
            settled_max = std::max(settled_max, step.horizontal_error_m);
            ++settled_count;
        }
    }
    const double settled_rms = (settled_count > 0) ? std::sqrt(settled_sum_squared / static_cast<double>(settled_count)) : 0.0;

    std::ofstream csv(csv_path);
    if (!csv) {
        std::fprintf(stderr, "error: could not open %s for writing\n", csv_path.c_str());
        return 1;
    }
    csv << "time_s,truth_east_m,truth_north_m,est_east_m,est_north_m,horizontal_error_m,altitude_error_m,est_horizontal_std_m,tercom_fix,tercom_mad_m,tercom_confidence\n";
    for (const Simulation::Step& step : result.steps) {
        csv << step.time_s << ',' << step.truth_position_enu_m.x << ',' << step.truth_position_enu_m.y << ',' << step.estimated_position_enu_m.x << ','
            << step.estimated_position_enu_m.y << ',' << step.horizontal_error_m << ',' << step.altitude_error_m << ',' << step.estimated_horizontal_std_m << ','
            << (step.tercom_fix_applied ? 1 : 0) << ',' << step.tercom_mad_m << ',' << step.tercom_confidence << '\n';
    }

    const std::size_t total_attempts = result.accepted_fixes + result.rejected_fixes;
    const double accept_pct = (total_attempts > 0) ? 100.0 * static_cast<double>(result.accepted_fixes) / static_cast<double>(total_attempts) : 0.0;

    std::printf("TERCOM software-in-the-loop result\n");
    std::printf("  duration                 %.0f s over %zu steps\n", configuration.duration_s, result.steps.size());
    std::printf("  terrain fixes            %zu accepted / %zu attempts (%.0f%%)\n", result.accepted_fixes, total_attempts, accept_pct);
    std::printf("  final horizontal error   %.1f m\n", result.final_horizontal_error_m);
    std::printf("  RMS horizontal error     %.1f m (whole run)\n", result.rms_horizontal_error_m);
    std::printf("  RMS after first fix      %.1f m\n", result.rms_error_after_first_fix_m);
    std::printf("  steady-state RMS (>%.0fs)  %.1f m\n", kSettleTimeS, settled_rms);
    std::printf("  steady-state peak        %.1f m\n", settled_max);
    std::printf("  peak error (with init)   %.1f m\n", result.max_horizontal_error_m);
    std::printf("  time history written to  %s\n", csv_path.c_str());
    return 0;
}
