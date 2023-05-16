#pragma once

// Software-in-the-loop simulator. It flies a ground-truth vehicle along the route
// with a coordinated-turn flight model, synthesises IMU, barometer, and radar
// altimeter measurements from the truth, and feeds them to an independent
// GuidanceSystem. Truth follows the route on its own state, so the recorded error
// (estimate minus truth) isolates the navigation performance: inertial drift
// growing between fixes and snapping back at each accepted TERCOM correction.
//
// The truth generator derives its IMU from the same mechanisation the INS
// integrates, so with the noise and bias disabled the estimate reproduces truth.

#include <cstddef>
#include <vector>

#include "tercom/geo/height_field.h"
#include "tercom/guidance/guidance_law.h"
#include "tercom/math/linear.h"
#include "tercom/nav/nav_state.h"
#include "tercom/sensors/simulated_sensors.h"
#include "tercom/tercom.h"
#include "tercom/trajectory/route.h"

namespace TERCOM { namespace Simulation {

    struct Configuration {
        // How long to fly, in seconds. 400 s is the reference run length used by the
        // bundled SITL example, the golden test, and the benchmark; it is long enough
        // for the inertial drift to grow and for several TERCOM fixes to pull it back.
        double duration_s = 400.0;
        // Sensor sample rates in hertz. The IMU runs fastest and sets the
        // integration step (dt = 1 / imu_rate_hz); the barometer and radar are
        // read every Nth IMU step. 100 Hz IMU, 10 Hz barometer, 20 Hz radar are
        // typical rates for a small inertial unit and its aiding sensors.
        double imu_rate_hz = 100.0;
        double barometer_rate_hz = 10.0;
        double radar_rate_hz = 20.0;

        // Deliberate horizontal offset added to the navigator's starting position,
        // in metres (east and north), while truth starts exactly on the route. This
        // seeds the run with a known position error so the recorded error curve
        // shows inertial drift and the TERCOM fixes correcting it. The values are an
        // arbitrary few-hundred-metre offset, not from any standard.
        double initial_position_error_east_m = 120.0;
        double initial_position_error_north_m = -90.0;

        // Truth flight-model first-order response time constants, in seconds. The
        // vehicle does not snap instantly to a commanded bank, climb, or speed; each
        // approaches its command exponentially with the time constant below (about
        // 63 percent of the way per time constant). Roll is quickest, speed slowest,
        // which is realistic for an aircraft-like vehicle. The values are tuning
        // choices that shape how aggressively truth manoeuvres, not physical
        // constants.
        double roll_time_constant_s = 0.7;
        double pitch_time_constant_s = 1.5;
        double speed_time_constant_s = 5.0;

        Sensors::SimulatedImuConfiguration imu;
        Sensors::SimulatedBarometerConfiguration barometer;
        Sensors::SimulatedRadarAltimeterConfiguration radar;
        GuidanceSystemConfiguration guidance_system;
        Guidance::GuidanceConfiguration truth_autopilot;
    };

    // One recorded sample of the run, captured at every IMU step. Holds both the
    // truth and the estimate so a caller can plot the trajectories and the error.
    struct Step {
        double time_s = 0.0;
        Math::Vec3 truth_position_enu_m;     // where the vehicle really is
        Math::Vec3 estimated_position_enu_m; // where the navigator thinks it is
        // Horizontal distance between estimate and truth (metres), the headline
        // navigation error; and the signed vertical error (estimate minus truth).
        double horizontal_error_m = 0.0;
        double altitude_error_m = 0.0;
        // The filter's own one-sigma horizontal position uncertainty (metres): what
        // the navigator believes its error is, independent of the true error above.
        double estimated_horizontal_std_m = 0.0;
        // Set on the step where a TERCOM terrain match was accepted and folded in.
        bool tercom_fix_applied = false;
        // Diagnostics from the most recent TERCOM match: mean-absolute-difference of
        // the best terrain alignment (metres, lower is a better match) and a derived
        // confidence in [0, 1]. See the correlator for how these are computed.
        double tercom_mad_m = 0.0;
        double tercom_confidence = 0.0;
    };

    // Summary statistics over the whole run, plus the full per-step record.
    struct Result {
        std::vector<Step> steps;
        // Horizontal error at the last step (metres): how close the navigator ended.
        double final_horizontal_error_m = 0.0;
        // Root-mean-square horizontal error over every step (metres): the typical
        // error magnitude across the whole flight, dominated by the early
        // pre-convergence period.
        double rms_horizontal_error_m = 0.0;
        // Largest horizontal error seen at any step (metres), usually right at the
        // start before the first fix.
        double max_horizontal_error_m = 0.0;
        // Root-mean-square horizontal error counting only steps after the first
        // accepted fix (metres): the steady-state accuracy once TERCOM is correcting,
        // which is the figure of merit for the navigator.
        double rms_error_after_first_fix_m = 0.0;
        // Counts of TERCOM correlation attempts: the total, and how many were
        // accepted versus rejected. fix_attempts == accepted_fixes + rejected_fixes.
        std::size_t fix_attempts = 0;
        std::size_t accepted_fixes = 0;
        std::size_t rejected_fixes = 0;
    };

    class Simulator {
      public:
        // truth_terrain is both the real world flown over and (copied) the onboard
        // reference map. truth_initial_state is the true starting state; route is the
        // plan. The (terrain, state, route, configuration) order matches the sibling
        // GuidanceSystem and TerrainReferencedNavigator constructors.
        Simulator(Geo::HeightField truth_terrain, const Nav::NavState& truth_initial_state, Trajectory::Route route, const Configuration& configuration = {});

        Result Run();

      private:
        Geo::HeightField truth_terrain_;
        Nav::NavState truth_initial_state_;
        Trajectory::Route route_;
        Configuration configuration_;
    };

}} // namespace TERCOM::Simulation
