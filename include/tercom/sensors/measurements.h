#pragma once

// Plain measurement records produced by the sensor hardware-abstraction layer.
// Timestamps are seconds on the vehicle clock. Frames follow the library
// convention: body axes are Forward-Left-Up (FLU), the local navigation frame is
// East-North-Up.

#include "tercom/geo/geo_point.h"
#include "tercom/math/linear.h"

namespace TERCOM { namespace Sensors {

    // Strapdown inertial measurement: body-frame angular rate and specific force.
    // Angular rate is how fast the body is rotating about its own axes (the gyroscope
    // output), in radians per second. Specific force is what an accelerometer
    // actually senses: the non-gravitational acceleration, equal to true
    // acceleration minus gravity. A unit at rest reads roughly +9.81 m/s^2 upward,
    // because the ground pushes up on it; it is not the same as acceleration, and
    // the navigator must add gravity back to recover motion.
    struct ImuSample {
        Math::Vec3 angular_rate_radps;  // body angular rate, radians per second
        Math::Vec3 specific_force_mps2; // body specific force, metres per second squared
        double time_s = 0.0;
    };

    // Pressure altitude reduced to height above the local ENU reference datum (the
    // tangent plane at the geodetic origin), plus the constant bias it carries.
    // valid is false when the sensor has no reading.
    struct BarometricSample {
        double altitude_m = 0.0;
        double time_s = 0.0;
        bool valid = false;
    };

    // Radar altimeter ground clearance (height above the terrain directly below).
    // valid is false on dropout. On real hardware that can be excessive bank, a
    // return beyond maximum range, or a specular surface giving no echo; the
    // simulated backend models the beyond-maximum-range case (and a missing or
    // negative-clearance reading).
    struct RadarAltitudeSample {
        double clearance_m = 0.0;
        double time_s = 0.0;
        bool valid = false;
    };

    // Optional satellite fix, used only for initialisation or cross-checks: the
    // terrain-referenced path exists precisely for when this is unavailable.
    struct GnssFix {
        Geo::GeoPoint position;
        double time_s = 0.0;
        int satellites = 0;
        bool valid = false;
    };

}} // namespace TERCOM::Sensors
