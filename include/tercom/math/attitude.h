#pragma once

// Unit-quaternion attitude, Hamilton convention. The quaternion rotates a vector
// from the body frame to the local navigation frame: v_nav = q * v_body.
//
// Frame convention used throughout the library:
//   navigation frame = East-North-Up (ENU), z points up
//   body frame       = Forward-Left-Up (FLU), right-handed, pairs with ENU
//   Euler order      = ZYX (yaw about up, then pitch, then roll), applied as a
//                      body 3-2-1 sequence.
//
// The simulated IMU and the strapdown INS share this convention exactly, so an
// ideal (noise-free, bias-free) IMU integrated by the INS reproduces the truth
// trajectory to machine precision.

#include <cmath>

#include "tercom/math/linear.h"

namespace TERCOM { namespace Math {

    // A quaternion is a four-number way to store a 3D rotation. Think of it as a
    // rotation axis plus an angle packed into (w, x, y, z): w = cos(angle/2) and
    // (x, y, z) = axis * sin(angle/2). Compared with Euler angles it has no
    // gimbal lock, and compared with a 3x3 matrix it is cheaper to keep
    // normalised over many integration steps, which is why the INS carries
    // attitude as a quaternion. The default below is the identity (no rotation).
    struct Quaternion {
        double w = 1.0; // scalar (real) part, equals cos(angle/2)
        double x = 0.0; // vector part i component, equals axis_x * sin(angle/2)
        double y = 0.0; // vector part j component, equals axis_y * sin(angle/2)
        double z = 0.0; // vector part k component, equals axis_z * sin(angle/2)

        constexpr Quaternion() = default;

        constexpr Quaternion(double w_in, double x_in, double y_in, double z_in) : w(w_in), x(x_in), y(y_in), z(z_in) {
        }

        // Hamilton product: composes two rotations. (this * other) means apply
        // other first, then this, just like multiplying rotation matrices. The
        // four lines below are the standard expansion of that product.
        constexpr Quaternion operator*(const Quaternion& q) const {
            return {
                (w * q.w) - (x * q.x) - (y * q.y) - (z * q.z),
                (w * q.x) + (x * q.w) + (y * q.z) - (z * q.y),
                (w * q.y) - (x * q.z) + (y * q.w) + (z * q.x),
                (w * q.z) + (x * q.y) - (y * q.x) + (z * q.w),
            };
        }

        // Conjugate: negate the vector part. For a unit quaternion this is the
        // inverse rotation, so q.Conjugate() undoes q.
        constexpr Quaternion Conjugate() const {
            return {w, -x, -y, -z};
        }

        // Length of the four-vector. A valid rotation quaternion has norm 1.
        double Norm() const {
            return std::sqrt((w * w) + (x * x) + (y * y) + (z * z));
        }

        // Rescale to unit length. Floating-point integration slowly drifts the
        // norm away from 1, so callers renormalise to keep it a pure rotation.
        // Falls back to the identity if the quaternion is degenerate (zero norm).
        Quaternion Normalized() const {
            const double n = Norm();
            if (n == 0.0) {
                return {1.0, 0.0, 0.0, 0.0};
            }
            return {w / n, x / n, y / n, z / n};
        }
    };

    // Build a body-to-navigation quaternion from roll, pitch, yaw (radians),
    // applied as a yaw-pitch-roll (ZYX) sequence. Yaw is measured about the up
    // axis.
    inline Quaternion QuaternionFromEuler(double roll_rad, double pitch_rad, double yaw_rad) {
        const double half_roll = roll_rad * 0.5;
        const double half_pitch = pitch_rad * 0.5;
        const double half_yaw = yaw_rad * 0.5;

        // A quaternion encodes half-angles (see the struct comment), so every
        // term below uses the cosine and sine of half each Euler angle. cr/sr are
        // cos/sin of half the roll, cp/sp of half the pitch, cy/sy of half the
        // yaw.
        const double cr = std::cos(half_roll);
        const double sr = std::sin(half_roll);
        const double cp = std::cos(half_pitch);
        const double sp = std::sin(half_pitch);
        const double cy = std::cos(half_yaw);
        const double sy = std::sin(half_yaw);

        // Standard ZYX (yaw-pitch-roll) combination of the three single-axis
        // quaternions, multiplied out into closed form.
        return Quaternion{
            (cr * cp * cy) + (sr * sp * sy),
            (sr * cp * cy) - (cr * sp * sy),
            (cr * sp * cy) + (sr * cp * sy),
            (cr * cp * sy) - (sr * sp * cy),
        }
            .Normalized();
    }

    // Body-to-navigation rotation matrix equivalent to the quaternion, so that
    // R * v_body == Rotate(q, v_body).
    inline Mat3 RotationMatrixFromQuaternion(const Quaternion& q_in) {
        const Quaternion q = q_in.Normalized();
        // Precompute the products of components that appear in the standard
        // quaternion-to-matrix formula below, then assemble the nine entries.
        const double xx = q.x * q.x;
        const double yy = q.y * q.y;
        const double zz = q.z * q.z;
        const double xy = q.x * q.y;
        const double xz = q.x * q.z;
        const double yz = q.y * q.z;
        const double wx = q.w * q.x;
        const double wy = q.w * q.y;
        const double wz = q.w * q.z;
        return Mat3({
            1.0 - (2.0 * (yy + zz)),
            2.0 * (xy - wz),
            2.0 * (xz + wy),
            2.0 * (xy + wz),
            1.0 - (2.0 * (xx + zz)),
            2.0 * (yz - wx),
            2.0 * (xz - wy),
            2.0 * (yz + wx),
            1.0 - (2.0 * (xx + yy)),
        });
    }

    // Rotate a body-frame vector into the navigation frame. The vector is wrapped
    // as a quaternion with zero scalar part, sandwiched as q * v * conjugate(q),
    // which is the textbook way to apply a quaternion rotation to a vector.
    inline Vec3 Rotate(const Quaternion& q, const Vec3& v) {
        const Quaternion p{0.0, v.x, v.y, v.z};
        const Quaternion r = (q * p) * q.Conjugate();
        return {r.x, r.y, r.z};
    }

    // Quaternion representing a small body-frame rotation given a rotation vector
    // (axis * angle, radians). A rotation vector packs an axis and an angle into
    // one Vec3: its direction is the axis, its length is the angle. This is the
    // exponential map (the "exp" that turns an angular increment into a rotation)
    // and is how the INS applies the small turn measured by the gyroscope each step.
    inline Quaternion QuaternionFromRotationVector(const Vec3& rotation) {
        const double angle = Norm(rotation);
        // Below this angle (radians) sin(half)/angle would lose precision or
        // divide by near-zero; 1e-12 rad is far smaller than any real gyroscope step,
        // so the first-order approximation (then renormalised) is exact enough.
        if (angle < 1e-12) {
            // Small-angle limit: q ~= (1, axis*angle/2), i.e. half the rotation
            // vector in the vector part, then rescaled to unit length.
            return Quaternion{1.0, rotation.x * 0.5, rotation.y * 0.5, rotation.z * 0.5}.Normalized();
        }
        const double half = angle * 0.5;
        // Distribute sin(angle/2) across the unit axis = rotation / angle, so the
        // vector part becomes axis * sin(angle/2) as the definition requires.
        const double scale = std::sin(half) / angle;
        return {std::cos(half), rotation.x * scale, rotation.y * scale, rotation.z * scale};
    }

    // Inverse of QuaternionFromRotationVector (the logarithm map): recover the
    // rotation vector (axis * angle, radians) from a quaternion. The error-state
    // filter uses this to express an attitude difference as three small numbers.
    // Returns the shortest rotation (angle at most pi).
    inline Vec3 RotationVectorFromQuaternion(const Quaternion& quaternion) {
        const Quaternion q = quaternion.Normalized();
        Vec3 axis{q.x, q.y, q.z};
        double scalar = q.w;
        // q and -q represent the same rotation; forcing the scalar part positive
        // picks the hemisphere whose recovered angle is the shorter way round.
        if (scalar < 0.0) {
            scalar = -scalar;
            axis = -axis;
        }
        const double axis_norm = Norm(axis);
        // Vector part near zero means the rotation is essentially the identity;
        // 1e-12 guards the divide by axis_norm below. Return no rotation.
        if (axis_norm < 1e-12) {
            return {0.0, 0.0, 0.0};
        }
        // angle = 2 * atan2(|vector part|, scalar) recovers the full rotation
        // angle robustly; dividing by axis_norm turns the vector part back into a
        // unit axis before scaling it by that angle.
        const double angle = 2.0 * std::atan2(axis_norm, scalar);
        return axis * (angle / axis_norm);
    }

    // Extract roll, pitch, yaw (radians) from a body-to-navigation quaternion.
    inline Vec3 EulerFromQuaternion(const Quaternion& q) {
        const double sin_roll_cos_pitch = 2.0 * ((q.w * q.x) + (q.y * q.z));
        const double cos_roll_cos_pitch = 1.0 - (2.0 * ((q.x * q.x) + (q.y * q.y)));
        const double roll = std::atan2(sin_roll_cos_pitch, cos_roll_cos_pitch);

        double pitch = 0.0;
        const double sin_pitch = 2.0 * ((q.w * q.y) - (q.z * q.x));
        // At straight up or straight down sin_pitch can round slightly past +-1,
        // where asin is undefined; clamp to +-90 degrees (pi/2 rad) with the sign
        // of sin_pitch. This is the gimbal-lock pole of the ZYX convention.
        if (std::fabs(sin_pitch) >= 1.0) {
            pitch = std::copysign(kPi / 2.0, sin_pitch);
        } else {
            pitch = std::asin(sin_pitch);
        }

        const double sin_yaw_cos_pitch = 2.0 * ((q.w * q.z) + (q.x * q.y));
        const double cos_yaw_cos_pitch = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)));
        const double yaw = std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch);

        return {roll, pitch, yaw};
    }

}} // namespace TERCOM::Math
