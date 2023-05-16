#include "tercom/simulation/simulator.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "tercom/math/attitude.h"

namespace TERCOM { namespace Simulation {

    using Math::Quaternion;
    using Math::Vec3;

    namespace {
        // Standard gravity as a scalar magnitude for the coordinated-turn formula
        // (the shared Math::kStandardGravity; the INS uses the vector kGravityEnu,
        // which has the same magnitude).
        constexpr double kGravity = Math::kStandardGravity;

        // How many IMU steps elapse between reads of a slower sensor, rounded to the
        // nearest whole step and clamped to at least one. For example a 100 Hz IMU
        // and a 20 Hz radar give an interval of 5 (read radar every fifth step).
        std::size_t RateInterval(double imu_rate_hz, double sensor_rate_hz) {
            const auto interval = static_cast<std::size_t>(std::llround(imu_rate_hz / sensor_rate_hz));
            return std::max<std::size_t>(1, interval);
        }

        // Build the true FLU-body-to-ENU attitude from the flight model's compass
        // heading (clockwise from north), climb (flight-path) angle, and bank. The
        // math module's yaw is right-handed (counter-clockwise from east), so the
        // compass bearing converts as yaw = pi/2 - heading; and a positive climb is
        // nose-up, which is a negative pitch in this frame. The result therefore
        // satisfies Rotate(attitude, forward) == the unit velocity direction, so the
        // stored attitude is the vehicle's genuine orientation (not just a
        // self-consistent placeholder for the IMU synthesis).
        Quaternion AttitudeFromFlightModel(double bank_rad, double flight_path_angle_rad, double heading_rad) {
            return Math::QuaternionFromEuler(bank_rad, -flight_path_angle_rad, (Math::kPi / 2.0) - heading_rad);
        }
    } // namespace

    Simulator::Simulator(Geo::HeightField truth_terrain, const Nav::NavState& truth_initial_state, Trajectory::Route route, const Configuration& configuration)
        : truth_terrain_(std::move(truth_terrain)),
          truth_initial_state_(truth_initial_state),
          route_(std::move(route)),
          configuration_(configuration) {
    }

    Result Simulator::Run() {
        // Integration step (seconds) and the number of steps to run. The IMU rate
        // sets the step, so a 100 Hz IMU over 400 s is 40000 steps of 0.01 s each.
        const double dt = 1.0 / configuration_.imu_rate_hz;
        const auto total_steps = static_cast<std::size_t>(std::llround(configuration_.duration_s * configuration_.imu_rate_hz));
        const std::size_t barometer_interval = RateInterval(configuration_.imu_rate_hz, configuration_.barometer_rate_hz);
        const std::size_t radar_interval = RateInterval(configuration_.imu_rate_hz, configuration_.radar_rate_hz);

        // Truth flight-model scalar state, seeded from the initial velocity. The
        // truth vehicle is flown as a point mass described by four numbers: total
        // speed, heading (compass bearing about the up axis), flight-path angle
        // (climb angle above horizontal), and bank (roll). Heading here is measured
        // clockwise from north, hence atan2(east, north) rather than atan2(y, x).
        double speed = Math::Norm(truth_initial_state_.velocity_enu_mps);
        const double horizontal_speed0 = std::sqrt((truth_initial_state_.velocity_enu_mps.x * truth_initial_state_.velocity_enu_mps.x)
                                                   + (truth_initial_state_.velocity_enu_mps.y * truth_initial_state_.velocity_enu_mps.y));
        double heading = std::atan2(truth_initial_state_.velocity_enu_mps.x, truth_initial_state_.velocity_enu_mps.y);
        double flight_path_angle = std::atan2(truth_initial_state_.velocity_enu_mps.z, horizontal_speed0);
        double bank = 0.0;

        Nav::NavState truth = truth_initial_state_;
        // Set truth attitude so the body forward axis lines up with the initial
        // velocity (see AttitudeFromFlightModel for the compass-to-right-handed
        // conversion).
        truth.attitude = AttitudeFromFlightModel(bank, flight_path_angle, heading);

        // The estimator starts from a copy of truth but with a deliberate horizontal
        // position error injected, and with its bias estimates zeroed. Zeroing the
        // biases means the navigator does not know the true sensor biases up front;
        // the filter must learn the accelerometer bias online and the gyroscope bias is
        // left small enough to ignore (see SimulatedImuConfiguration). This is what makes
        // the recorded error meaningful: it is the gap the navigator has to close.
        Nav::NavState nav_initial = truth;
        nav_initial.position_enu_m.x += configuration_.initial_position_error_east_m;
        nav_initial.position_enu_m.y += configuration_.initial_position_error_north_m;
        nav_initial.accelerometer_bias_mps2 = Vec3{};
        nav_initial.gyroscope_bias_radps = Vec3{};

        // The navigator under test. It is given its own copy of the terrain as the
        // onboard reference map, so the only differences from truth are the injected
        // position error and the sensor noise and bias, not a different map.
        GuidanceSystem guidance_system(Terrain::ContourMap(truth_terrain_), nav_initial, route_, configuration_.guidance_system);

        // A second, independent guidance law that flies the truth vehicle along the
        // route from the truth state. It is separate from the navigator's own
        // guidance so truth motion is never contaminated by estimation error.
        // truth_route is truth's own copy of the plan, advanced as truth passes
        // waypoints, independent of the navigator's route progress.
        const Guidance::GuidanceLaw truth_autopilot(configuration_.truth_autopilot);
        Trajectory::Route truth_route = route_;

        Sensors::SimulatedImu imu(configuration_.imu);
        Sensors::SimulatedBarometer barometer(configuration_.barometer);
        Sensors::SimulatedRadarAltimeter radar(configuration_.radar);

        Result result;
        result.steps.reserve(total_steps);

        // Running accumulators for the summary statistics computed after the loop.
        // sum_squared_error feeds the overall RMS; the "after fix" accumulators feed
        // the steady-state RMS and only start once first_fix_seen becomes true.
        // previous_accepted lets us detect, step to step, when a new fix was folded
        // in (the accepted-fix count went up).
        double sum_squared_error = 0.0;
        double sum_squared_error_after_fix = 0.0;
        std::size_t samples_after_fix = 0;
        bool first_fix_seen = false;
        std::size_t previous_accepted = 0;

        for (std::size_t step = 0; step < total_steps; ++step) {
            const double time = static_cast<double>(step + 1) * dt;

            // Truth autopilot: follow the route from the truth state.
            const Trajectory::Correction command = truth_autopilot.Compute(truth, truth_route);
            const double speed_command = truth_route.ActiveWaypoint().speed_mps;

            // Move each scalar a fraction dt/tau of the way toward its command, the
            // discrete form of a first-order lag (an exponential approach with time
            // constant tau). The fraction is clamped to 1 so a step larger than the
            // time constant cannot overshoot the command.
            bank += (command.roll_command_rad - bank) * std::min(1.0, dt / configuration_.roll_time_constant_s);
            flight_path_angle += (command.pitch_command_rad - flight_path_angle) * std::min(1.0, dt / configuration_.pitch_time_constant_s);
            speed += (speed_command - speed) * std::min(1.0, dt / configuration_.speed_time_constant_s);

            // Coordinated-turn kinematics: a banked vehicle turns at rate
            // g * tan(bank) / speed (radians per second). This is the standard
            // level-turn relation, where the horizontal component of lift supplies
            // the centripetal force. The speed is floored at 1 m/s to avoid dividing
            // by a near-zero speed. WrapToPi keeps heading in (-pi, pi].
            const double turn_rate = kGravity * std::tan(bank) / std::max(speed, 1.0);
            heading = Math::WrapToPi(heading + (turn_rate * dt));

            // Rebuild the velocity vector from the updated speed, heading, and climb,
            // then derive acceleration and the new position. acceleration_nav is the
            // average over the step (change in velocity divided by dt); position uses
            // the constant-acceleration update x += v*dt + 0.5*a*dt^2, matching the
            // INS integrator exactly so the two stay in lockstep.
            const double horizontal_speed = speed * std::cos(flight_path_angle);
            const Vec3 velocity_new{horizontal_speed * std::sin(heading), horizontal_speed * std::cos(heading), speed * std::sin(flight_path_angle)};
            const Vec3 velocity_old = truth.velocity_enu_mps;
            const Vec3 acceleration_nav = (velocity_new - velocity_old) / dt;
            const Vec3 position_new = truth.position_enu_m + (velocity_old * dt) + (acceleration_nav * (0.5 * dt * dt));
            const Quaternion attitude_old = truth.attitude;
            const Quaternion attitude_new = AttitudeFromFlightModel(bank, flight_path_angle, heading);

            // Derive the ideal IMU reading by running the INS mechanisation backwards.
            // The INS computes acceleration as Rotate(attitude, specific_force) +
            // gravity; inverting that gives the specific force the IMU must report.
            // "Specific force" is what an accelerometer actually senses: total
            // acceleration minus gravity (so an accelerometer at rest reads +g
            // upward, not zero). Here gravity is subtracted and the result rotated
            // into the body frame using the conjugate (inverse) of the attitude.
            // The gyroscope reading is the body-frame rotation from old to new attitude,
            // recovered as a rotation vector and divided by dt to get rate.
            // Because this is the exact inverse of what StrapdownIns::Step does, a
            // noise-free, bias-free IMU integrated by the INS reproduces this truth
            // trajectory to machine precision.
            const Vec3 specific_force_body = Math::Rotate(attitude_old.Conjugate(), acceleration_nav - Nav::kGravityEnu);
            const Vec3 angular_rate_body = Math::RotationVectorFromQuaternion(attitude_old.Conjugate() * attitude_new) / dt;

            truth.position_enu_m = position_new;
            truth.velocity_enu_mps = velocity_new;
            truth.attitude = attitude_new;
            truth.time_s = time;

            // Feed the ideal IMU sample to the simulated IMU (which adds noise and
            // bias) and hand the corrupted reading to the navigator every step.
            imu.SetTruth(angular_rate_body, specific_force_body, time);
            guidance_system.OnImu(imu.Read());

            // Barometer reads true altitude (the ENU up coordinate) on its slower
            // schedule.
            if (step % barometer_interval == 0) {
                barometer.SetTruth(truth.position_enu_m.z, time);
                guidance_system.OnBarometer(barometer.Read());
            }

            // Radar altimeter measures clearance: height of the vehicle above the
            // terrain directly below, i.e. altitude minus ground elevation. Skip the
            // sample where the terrain is undefined (off the map edge).
            if (step % radar_interval == 0) {
                const std::optional<double> terrain = truth_terrain_.ElevationAt(truth.position_enu_m.x, truth.position_enu_m.y);
                if (terrain.has_value()) {
                    radar.SetTruth(truth.position_enu_m.z - *terrain, time);
                    guidance_system.OnRadarAltimeter(radar.Read());
                }
            }

            // A new TERCOM fix was folded in this step if the navigator's accepted
            // count rose since the previous step.
            const std::size_t accepted = guidance_system.Navigator().AcceptedFixCount();
            const bool fix_applied = accepted > previous_accepted;
            previous_accepted = accepted;

            const Nav::NavState& estimate = guidance_system.EstimatedState();
            const double error_east = estimate.position_enu_m.x - truth.position_enu_m.x;
            const double error_north = estimate.position_enu_m.y - truth.position_enu_m.y;
            const double horizontal_error = std::sqrt((error_east * error_east) + (error_north * error_north));

            Step record;
            record.time_s = time;
            record.truth_position_enu_m = truth.position_enu_m;
            record.estimated_position_enu_m = estimate.position_enu_m;
            record.horizontal_error_m = horizontal_error;
            record.altitude_error_m = estimate.position_enu_m.z - truth.position_enu_m.z;
            record.estimated_horizontal_std_m = guidance_system.HorizontalStdM();
            record.tercom_fix_applied = fix_applied;
            record.tercom_mad_m = guidance_system.Navigator().LastFix().mad_m;
            record.tercom_confidence = guidance_system.Navigator().LastFix().confidence;
            result.steps.push_back(record);

            // Accumulate squared error for the overall RMS. The "after first fix"
            // sum only starts once a fix has been seen on a prior step, so the
            // current step is counted from the step after the first fix onward.
            sum_squared_error += horizontal_error * horizontal_error;
            if (first_fix_seen) {
                sum_squared_error_after_fix += horizontal_error * horizontal_error;
                ++samples_after_fix;
            }
            if (fix_applied) {
                first_fix_seen = true;
            }
            result.max_horizontal_error_m = std::max(result.max_horizontal_error_m, horizontal_error);
        }

        // Finalise the statistics. RMS is sqrt(mean of squared errors). If no fix was
        // ever accepted, the post-fix RMS falls back to the overall RMS so callers
        // always get a usable number.
        if (!result.steps.empty()) {
            result.final_horizontal_error_m = result.steps.back().horizontal_error_m;
            result.rms_horizontal_error_m = std::sqrt(sum_squared_error / static_cast<double>(result.steps.size()));
        }
        result.rms_error_after_first_fix_m = (samples_after_fix > 0) ? std::sqrt(sum_squared_error_after_fix / static_cast<double>(samples_after_fix))
                                                                     : result.rms_horizontal_error_m;
        result.fix_attempts = guidance_system.Navigator().FixAttemptCount();
        result.accepted_fixes = guidance_system.Navigator().AcceptedFixCount();
        // Rejected = every attempt that did not pass the acceptance gate.
        result.rejected_fixes = result.fix_attempts - result.accepted_fixes;
        return result;
    }

}} // namespace TERCOM::Simulation
