#include "tercom/nav/strapdown_ins.h"

namespace TERCOM { namespace Nav {

    using Math::Quaternion;
    using Math::QuaternionFromRotationVector;
    using Math::Rotate;
    using Math::Vec3;

    StrapdownIns::StrapdownIns(const NavState& initial_state) : state_(initial_state) {
    }

    void StrapdownIns::Step(const Sensors::ImuSample& sample) {
        const double dt = sample.time_s - state_.time_s;
        if (dt <= 0.0) {
            // Out-of-order or duplicate sample: only advance the clock.
            state_.time_s = sample.time_s;
            return;
        }

        // Remove the current bias estimates from the raw measurements. An IMU
        // reports rotation rate and force with a slowly varying offset (bias);
        // the filter estimates that offset and we subtract it here.
        const Vec3 angular_rate = sample.angular_rate_radps - state_.gyroscope_bias_radps;
        const Vec3 specific_force_body = sample.specific_force_mps2 - state_.accelerometer_bias_mps2;

        // Specific force is what an accelerometer actually measures: total
        // acceleration minus gravity (a sensor in free fall reads zero, one at
        // rest reads +g upward). Rotate it from the body frame into the navigation
        // frame using the attitude at the start of the interval, then add gravity
        // back in to recover inertial acceleration expressed in ENU.
        const Vec3 specific_force_nav = Rotate(state_.attitude, specific_force_body);
        const Vec3 acceleration_nav = specific_force_nav + kGravityEnu;

        // Integrate over the interval: position with the constant-acceleration rule
        // x += v*dt + 0.5*a*dt^2, velocity with the rectangular rule v += a*dt.
        state_.position_enu_m += (state_.velocity_enu_mps * dt) + (acceleration_nav * (0.5 * dt * dt));
        state_.velocity_enu_mps += acceleration_nav * dt;

        // Propagate attitude: turn the body-frame angle swept over the interval
        // (rate * dt, a rotation vector) into a small rotation quaternion and
        // compose it onto the current attitude, renormalising to stay a unit
        // quaternion (numerical drift would otherwise scale rotated vectors).
        const Quaternion delta = QuaternionFromRotationVector(angular_rate * dt);
        state_.attitude = (state_.attitude * delta).Normalized();

        state_.time_s = sample.time_s;
    }

    // Closed-loop feedback: the error-state filter estimates how far the INS has
    // drifted and passes back corrections (already negated to be additive). Adding
    // the bias correction here means subsequent samples are de-biased better, so
    // the drift between terrain fixes shrinks rather than just snapping back.
    void StrapdownIns::ApplyCorrection(const Vec3& position_correction_m, const Vec3& velocity_correction_mps, const Vec3& accelerometer_bias_correction_mps2) {
        state_.position_enu_m += position_correction_m;
        state_.velocity_enu_mps += velocity_correction_mps;
        state_.accelerometer_bias_mps2 += accelerometer_bias_correction_mps2;
    }

}} // namespace TERCOM::Nav
