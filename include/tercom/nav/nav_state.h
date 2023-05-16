#pragma once

// The full navigation state carried by the strapdown INS and corrected by the
// terrain-referenced filter. Position and velocity are in the local ENU frame;
// attitude rotates body (FLU) vectors into ENU; the biases are the filter's
// current estimate of the inertial sensor errors.

#include "tercom/math/attitude.h"
#include "tercom/math/linear.h"

namespace TERCOM { namespace Nav {

    struct NavState {
        double time_s = 0.0;                // Vehicle clock at which this state is valid, seconds.
        Math::Vec3 position_enu_m;          // Position in the local ENU frame, metres.
        Math::Vec3 velocity_enu_mps;        // Velocity in the local ENU frame, metres per second.
        Math::Quaternion attitude;          // Body (FLU) to navigation (ENU) rotation; see math/attitude.h.
        Math::Vec3 accelerometer_bias_mps2; // Estimated accelerometer bias subtracted from raw samples, m/s^2.
        Math::Vec3 gyroscope_bias_radps;    // Estimated gyroscope bias subtracted from raw samples, rad/s.
    };

    // Standard gravity in the ENU navigation frame: the shared Math::kStandardGravity
    // magnitude placed on the Up (z) axis with a minus sign so it points down. The
    // strapdown INS adds this to the measured specific force to recover inertial
    // acceleration.
    inline constexpr Math::Vec3 kGravityEnu{0.0, 0.0, -Math::kStandardGravity};

}} // namespace TERCOM::Nav
