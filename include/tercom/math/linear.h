#pragma once

// Minimal double-precision linear algebra for navigation maths.
// Deliberately small and dependency-free: a 3-vector, a 3x3 matrix, and the
// handful of operations the geodesy, inertial, and guidance code needs.

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace TERCOM { namespace Math {

    // The mathematical constant pi (ratio of a circle's circumference to its
    // diameter). All angles in the library are radians, where a full turn is
    // 2*pi, so this appears in every angle conversion and wrap below.
    inline constexpr double kPi = std::numbers::pi;

    // Standard gravity in metres per second squared: the conventional sea-level
    // free-fall acceleration (CGPM 1901 / ISO 80000-3). The single source of this
    // value for the whole library; the navigation frame uses the vector form
    // kGravityEnu, and the flight model and guidance use this scalar magnitude.
    inline constexpr double kStandardGravity = 9.80665;

    // Degrees to radians: 180 degrees equals pi radians, so multiply by pi/180.
    inline constexpr double DegToRad(double degrees) {
        return degrees * (kPi / 180.0);
    }

    // Radians to degrees: the inverse of the above, so multiply by 180/pi.
    inline constexpr double RadToDeg(double radians) {
        return radians * (180.0 / kPi);
    }

    // Wrap an angle in radians into the half-open interval (-pi, pi]. Headings
    // and yaw are periodic, so two angles that differ by a full turn are equal;
    // wrapping keeps a single canonical value and makes angle differences small.
    inline double WrapToPi(double radians) {
        // Shift by +pi, take the remainder over one full turn (2*pi), then shift
        // back by -pi. The branch below corrects fmod's sign so the result never
        // falls outside (-pi, pi] (fmod can return a negative remainder).
        double wrapped = std::fmod(radians + kPi, 2.0 * kPi);
        if (wrapped <= 0.0) {
            wrapped += 2.0 * kPi;
        }
        return wrapped - kPi;
    }

    // A 3-component vector of doubles. Depending on context it holds a position,
    // a velocity, an acceleration, or a rotation axis; the meaning of x, y, z is
    // set by the frame the caller is working in (for example east-north-up).
    struct Vec3 {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        constexpr Vec3() = default;

        constexpr Vec3(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {
        }

        constexpr Vec3 operator+(const Vec3& other) const {
            return {x + other.x, y + other.y, z + other.z};
        }

        constexpr Vec3 operator-(const Vec3& other) const {
            return {x - other.x, y - other.y, z - other.z};
        }

        constexpr Vec3 operator-() const {
            return {-x, -y, -z};
        }

        constexpr Vec3 operator*(double scalar) const {
            return {x * scalar, y * scalar, z * scalar};
        }

        constexpr Vec3 operator/(double scalar) const {
            return {x / scalar, y / scalar, z / scalar};
        }

        constexpr Vec3& operator+=(const Vec3& other) {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        constexpr Vec3& operator-=(const Vec3& other) {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }
    };

    inline constexpr Vec3 operator*(double scalar, const Vec3& vector) {
        return vector * scalar;
    }

    // Dot (scalar) product: sum of element-wise products. Equals
    // |a| * |b| * cos(angle between them), so it is zero for perpendicular
    // vectors and is also used below to compute a vector's length.
    inline constexpr double Dot(const Vec3& a, const Vec3& b) {
        return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
    }

    // Euclidean length (magnitude) of the vector: sqrt(x^2 + y^2 + z^2).
    inline double Norm(const Vec3& v) {
        return std::sqrt(Dot(v, v));
    }

    // Scale the vector to unit length (length 1) while keeping its direction.
    // Returns the zero vector unchanged rather than dividing by zero.
    inline Vec3 Normalized(const Vec3& v) {
        const double length = Norm(v);
        if (length == 0.0) {
            return v;
        }
        return v / length;
    }

    // Row-major 3x3 matrix. Used for coordinate-frame rotations.
    struct Mat3 {
        std::array<double, 9> m{};

        constexpr Mat3() = default;

        constexpr explicit Mat3(const std::array<double, 9>& values) : m(values) {
        }

        static constexpr Mat3 Identity() {
            return Mat3({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        }

        // Read element (row, col). Row-major storage means row r, column c lives
        // at flat index r*3 + c in the nine-element array.
        constexpr double At(std::size_t row, std::size_t col) const {
            return m[(row * 3) + col];
        }

        // Matrix-times-vector: rotates (or otherwise transforms) v. Each output
        // component is the dot product of one matrix row with v.
        constexpr Vec3 operator*(const Vec3& v) const {
            return {
                (At(0, 0) * v.x) + (At(0, 1) * v.y) + (At(0, 2) * v.z),
                (At(1, 0) * v.x) + (At(1, 1) * v.y) + (At(1, 2) * v.z),
                (At(2, 0) * v.x) + (At(2, 1) * v.y) + (At(2, 2) * v.z),
            };
        }

        // Transpose: reflect across the diagonal, swapping rows and columns. For
        // a rotation matrix the transpose is also its inverse (the reverse
        // rotation), which is how the geodesy code converts between frames.
        constexpr Mat3 Transposed() const {
            return Mat3({At(0, 0), At(1, 0), At(2, 0), At(0, 1), At(1, 1), At(2, 1), At(0, 2), At(1, 2), At(2, 2)});
        }
    };

}} // namespace TERCOM::Math
