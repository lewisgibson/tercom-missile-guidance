#include <gtest/gtest.h>

#include <tercom/math/linear.h>

#include <array>
#include <cmath>

namespace {

    using TERCOM::Math::DegToRad;
    using TERCOM::Math::Dot;
    using TERCOM::Math::kPi;
    using TERCOM::Math::Mat3;
    using TERCOM::Math::Norm;
    using TERCOM::Math::Normalized;
    using TERCOM::Math::RadToDeg;
    using TERCOM::Math::Vec3;
    using TERCOM::Math::WrapToPi;

    constexpr double kTol = 1e-12;

} // namespace

TEST(Vec3, DefaultConstructorIsZero) {
    const Vec3 v;
    EXPECT_NEAR(v.x, 0.0, kTol);
    EXPECT_NEAR(v.y, 0.0, kTol);
    EXPECT_NEAR(v.z, 0.0, kTol);
}

TEST(Vec3, ComponentConstructor) {
    const Vec3 v(1.0, 2.0, 3.0);
    EXPECT_NEAR(v.x, 1.0, kTol);
    EXPECT_NEAR(v.y, 2.0, kTol);
    EXPECT_NEAR(v.z, 3.0, kTol);
}

TEST(Vec3, Addition) {
    const Vec3 a(1.0, 2.0, 3.0);
    const Vec3 b(4.0, -5.0, 6.0);
    const Vec3 c = a + b;
    EXPECT_NEAR(c.x, 5.0, kTol);
    EXPECT_NEAR(c.y, -3.0, kTol);
    EXPECT_NEAR(c.z, 9.0, kTol);
}

TEST(Vec3, Subtraction) {
    const Vec3 a(1.0, 2.0, 3.0);
    const Vec3 b(4.0, -5.0, 6.0);
    const Vec3 c = a - b;
    EXPECT_NEAR(c.x, -3.0, kTol);
    EXPECT_NEAR(c.y, 7.0, kTol);
    EXPECT_NEAR(c.z, -3.0, kTol);
}

TEST(Vec3, UnaryNegation) {
    const Vec3 a(1.0, -2.0, 3.0);
    const Vec3 n = -a;
    EXPECT_NEAR(n.x, -1.0, kTol);
    EXPECT_NEAR(n.y, 2.0, kTol);
    EXPECT_NEAR(n.z, -3.0, kTol);
}

TEST(Vec3, ScalarMultiplyOnRight) {
    const Vec3 a(1.0, -2.0, 3.0);
    const Vec3 s = a * 2.0;
    EXPECT_NEAR(s.x, 2.0, kTol);
    EXPECT_NEAR(s.y, -4.0, kTol);
    EXPECT_NEAR(s.z, 6.0, kTol);
}

TEST(Vec3, ScalarMultiplyOnLeft) {
    const Vec3 a(1.0, -2.0, 3.0);
    const Vec3 s = 2.0 * a;
    EXPECT_NEAR(s.x, 2.0, kTol);
    EXPECT_NEAR(s.y, -4.0, kTol);
    EXPECT_NEAR(s.z, 6.0, kTol);
}

TEST(Vec3, ScalarMultiplyCommutes) {
    const Vec3 a(3.0, 4.0, 5.0);
    const Vec3 left = 1.5 * a;
    const Vec3 right = a * 1.5;
    EXPECT_NEAR(left.x, right.x, kTol);
    EXPECT_NEAR(left.y, right.y, kTol);
    EXPECT_NEAR(left.z, right.z, kTol);
}

TEST(Vec3, ScalarDivision) {
    const Vec3 a(2.0, -4.0, 6.0);
    const Vec3 d = a / 2.0;
    EXPECT_NEAR(d.x, 1.0, kTol);
    EXPECT_NEAR(d.y, -2.0, kTol);
    EXPECT_NEAR(d.z, 3.0, kTol);
}

TEST(Vec3, PlusEqualsAccumulates) {
    Vec3 a(1.0, 2.0, 3.0);
    Vec3& ref = (a += Vec3(10.0, 20.0, 30.0));
    EXPECT_NEAR(a.x, 11.0, kTol);
    EXPECT_NEAR(a.y, 22.0, kTol);
    EXPECT_NEAR(a.z, 33.0, kTol);
    EXPECT_EQ(&ref, &a);
}

TEST(Vec3, MinusEqualsSubtracts) {
    Vec3 a(11.0, 22.0, 33.0);
    Vec3& ref = (a -= Vec3(1.0, 2.0, 3.0));
    EXPECT_NEAR(a.x, 10.0, kTol);
    EXPECT_NEAR(a.y, 20.0, kTol);
    EXPECT_NEAR(a.z, 30.0, kTol);
    EXPECT_EQ(&ref, &a);
}

TEST(Vec3, DotProduct) {
    const Vec3 a(1.0, 2.0, 3.0);
    const Vec3 b(4.0, -5.0, 6.0);
    EXPECT_NEAR(Dot(a, b), (1.0 * 4.0) + (2.0 * -5.0) + (3.0 * 6.0), kTol);
}

TEST(Vec3, DotOfOrthogonalVectorsIsZero) {
    const Vec3 x(1.0, 0.0, 0.0);
    const Vec3 y(0.0, 1.0, 0.0);
    EXPECT_NEAR(Dot(x, y), 0.0, kTol);
}

TEST(Vec3, DotWithSelfIsNormSquared) {
    const Vec3 a(3.0, 4.0, 12.0);
    EXPECT_NEAR(Dot(a, a), 169.0, kTol);
}

TEST(Vec3, NormOfThreeFourTwelve) {
    const Vec3 a(3.0, 4.0, 12.0);
    EXPECT_NEAR(Norm(a), 13.0, kTol);
}

TEST(Vec3, NormOfZeroIsZero) {
    const Vec3 a;
    EXPECT_NEAR(Norm(a), 0.0, kTol);
}

TEST(Vec3, NormalizedHasUnitLength) {
    const Vec3 a(3.0, 4.0, 12.0);
    const Vec3 u = Normalized(a);
    EXPECT_NEAR(Norm(u), 1.0, kTol);
    EXPECT_NEAR(u.x, 3.0 / 13.0, kTol);
    EXPECT_NEAR(u.y, 4.0 / 13.0, kTol);
    EXPECT_NEAR(u.z, 12.0 / 13.0, kTol);
}

TEST(Vec3, NormalizedZeroVectorReturnsZero) {
    const Vec3 a;
    const Vec3 u = Normalized(a);
    EXPECT_NEAR(u.x, 0.0, kTol);
    EXPECT_NEAR(u.y, 0.0, kTol);
    EXPECT_NEAR(u.z, 0.0, kTol);
}

TEST(Vec3, NormalizedPreservesDirection) {
    const Vec3 a(0.0, -5.0, 0.0);
    const Vec3 u = Normalized(a);
    EXPECT_NEAR(u.x, 0.0, kTol);
    EXPECT_NEAR(u.y, -1.0, kTol);
    EXPECT_NEAR(u.z, 0.0, kTol);
}

TEST(Angles, DegToRadKnownValues) {
    EXPECT_NEAR(DegToRad(0.0), 0.0, kTol);
    EXPECT_NEAR(DegToRad(180.0), kPi, kTol);
    EXPECT_NEAR(DegToRad(90.0), kPi / 2.0, kTol);
    EXPECT_NEAR(DegToRad(-90.0), -kPi / 2.0, kTol);
}

TEST(Angles, RadToDegKnownValues) {
    EXPECT_NEAR(RadToDeg(0.0), 0.0, kTol);
    EXPECT_NEAR(RadToDeg(kPi), 180.0, kTol);
    EXPECT_NEAR(RadToDeg(kPi / 2.0), 90.0, kTol);
    EXPECT_NEAR(RadToDeg(-kPi / 2.0), -90.0, kTol);
}

TEST(Angles, DegRadRoundTrip) {
    for (double degrees = -360.0; degrees <= 360.0; degrees += 17.5) {
        EXPECT_NEAR(RadToDeg(DegToRad(degrees)), degrees, 1e-9);
    }
}

TEST(Angles, RadDegRoundTrip) {
    for (double radians = -6.0; radians <= 6.0; radians += 0.5) {
        EXPECT_NEAR(DegToRad(RadToDeg(radians)), radians, 1e-9);
    }
}

TEST(Angles, WrapToPiLeavesInteriorAnglesUnchanged) {
    EXPECT_NEAR(WrapToPi(0.0), 0.0, kTol);
    EXPECT_NEAR(WrapToPi(1.0), 1.0, kTol);
    EXPECT_NEAR(WrapToPi(-1.0), -1.0, kTol);
}

TEST(Angles, WrapToPiMapsPositivePiToPi) {
    EXPECT_NEAR(WrapToPi(kPi), kPi, 1e-9);
}

TEST(Angles, WrapToPiMapsNegativePiToPositivePi) {
    // The interval is half-open (-pi, pi], so -pi must wrap to +pi.
    EXPECT_NEAR(WrapToPi(-kPi), kPi, 1e-9);
}

TEST(Angles, WrapToPiHandlesThreePi) {
    EXPECT_NEAR(WrapToPi(3.0 * kPi), kPi, 1e-9);
}

TEST(Angles, WrapToPiHandlesNegativeThreePi) {
    EXPECT_NEAR(WrapToPi(-3.0 * kPi), kPi, 1e-9);
}

TEST(Angles, WrapToPiHandlesTwoPiToZero) {
    EXPECT_NEAR(WrapToPi(2.0 * kPi), 0.0, 1e-9);
    EXPECT_NEAR(WrapToPi(-2.0 * kPi), 0.0, 1e-9);
}

TEST(Angles, WrapToPiResultAlwaysInHalfOpenInterval) {
    for (double radians = -20.0; radians <= 20.0; radians += 0.3) {
        const double wrapped = WrapToPi(radians);
        EXPECT_GT(wrapped, -kPi);
        EXPECT_LE(wrapped, kPi + 1e-12);
    }
}

TEST(Angles, WrapToPiPreservesAngleModuloTwoPi) {
    for (double radians = -15.0; radians <= 15.0; radians += 0.7) {
        const double wrapped = WrapToPi(radians);
        const double difference = std::remainder(radians - wrapped, 2.0 * kPi);
        EXPECT_NEAR(difference, 0.0, 1e-9);
    }
}

TEST(Mat3, DefaultConstructorIsAllZero) {
    const Mat3 m;
    for (std::size_t row = 0; row < 3u; ++row) {
        for (std::size_t col = 0; col < 3u; ++col) {
            EXPECT_NEAR(m.At(row, col), 0.0, kTol);
        }
    }
}

TEST(Mat3, IdentityHasOnesOnDiagonal) {
    const Mat3 m = Mat3::Identity();
    for (std::size_t row = 0; row < 3u; ++row) {
        for (std::size_t col = 0; col < 3u; ++col) {
            const double expected = (row == col) ? 1.0 : 0.0;
            EXPECT_NEAR(m.At(row, col), expected, kTol);
        }
    }
}

TEST(Mat3, AtReadsRowMajorLayout) {
    const Mat3 m(std::array<double, 9>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0});
    EXPECT_NEAR(m.At(0, 0), 1.0, kTol);
    EXPECT_NEAR(m.At(0, 1), 2.0, kTol);
    EXPECT_NEAR(m.At(0, 2), 3.0, kTol);
    EXPECT_NEAR(m.At(1, 0), 4.0, kTol);
    EXPECT_NEAR(m.At(1, 1), 5.0, kTol);
    EXPECT_NEAR(m.At(1, 2), 6.0, kTol);
    EXPECT_NEAR(m.At(2, 0), 7.0, kTol);
    EXPECT_NEAR(m.At(2, 1), 8.0, kTol);
    EXPECT_NEAR(m.At(2, 2), 9.0, kTol);
}

TEST(Mat3, IdentityTimesVectorIsUnchanged) {
    const Mat3 m = Mat3::Identity();
    const Vec3 v(2.0, -3.0, 7.0);
    const Vec3 result = m * v;
    EXPECT_NEAR(result.x, v.x, kTol);
    EXPECT_NEAR(result.y, v.y, kTol);
    EXPECT_NEAR(result.z, v.z, kTol);
}

TEST(Mat3, RotationAboutZByNinetyDegreesMapsXToY) {
    // Rotation by +90 degrees about the z axis: x -> y, y -> -x.
    const double c = std::cos(kPi / 2.0);
    const double s = std::sin(kPi / 2.0);
    const Mat3 rotation(std::array<double, 9>{c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0});
    const Vec3 x(1.0, 0.0, 0.0);
    const Vec3 rotated = rotation * x;
    EXPECT_NEAR(rotated.x, 0.0, 1e-9);
    EXPECT_NEAR(rotated.y, 1.0, 1e-9);
    EXPECT_NEAR(rotated.z, 0.0, 1e-9);
}

TEST(Mat3, RotationAboutZByNinetyDegreesMapsYToNegativeX) {
    const double c = std::cos(kPi / 2.0);
    const double s = std::sin(kPi / 2.0);
    const Mat3 rotation(std::array<double, 9>{c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0});
    const Vec3 y(0.0, 1.0, 0.0);
    const Vec3 rotated = rotation * y;
    EXPECT_NEAR(rotated.x, -1.0, 1e-9);
    EXPECT_NEAR(rotated.y, 0.0, 1e-9);
    EXPECT_NEAR(rotated.z, 0.0, 1e-9);
}

TEST(Mat3, GeneralMatrixVectorProduct) {
    const Mat3 m(std::array<double, 9>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0});
    const Vec3 v(1.0, 0.0, -1.0);
    const Vec3 result = m * v;
    EXPECT_NEAR(result.x, (1.0 * 1.0) + (2.0 * 0.0) + (3.0 * -1.0), kTol);
    EXPECT_NEAR(result.y, (4.0 * 1.0) + (5.0 * 0.0) + (6.0 * -1.0), kTol);
    EXPECT_NEAR(result.z, (7.0 * 1.0) + (8.0 * 0.0) + (9.0 * -1.0), kTol);
}

TEST(Mat3, TransposedSwapsOffDiagonalEntries) {
    const Mat3 m(std::array<double, 9>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0});
    const Mat3 t = m.Transposed();
    for (std::size_t row = 0; row < 3u; ++row) {
        for (std::size_t col = 0; col < 3u; ++col) {
            EXPECT_NEAR(t.At(row, col), m.At(col, row), kTol);
        }
    }
}

TEST(Mat3, TransposeOfTransposeIsOriginal) {
    const Mat3 m(std::array<double, 9>{1.0, -2.0, 3.5, 4.0, 5.0, -6.0, 0.5, 8.0, 9.0});
    const Mat3 round = m.Transposed().Transposed();
    for (std::size_t row = 0; row < 3u; ++row) {
        for (std::size_t col = 0; col < 3u; ++col) {
            EXPECT_NEAR(round.At(row, col), m.At(row, col), kTol);
        }
    }
}

TEST(Mat3, TransposeOfIdentityIsIdentity) {
    const Mat3 m = Mat3::Identity().Transposed();
    for (std::size_t row = 0; row < 3u; ++row) {
        for (std::size_t col = 0; col < 3u; ++col) {
            const double expected = (row == col) ? 1.0 : 0.0;
            EXPECT_NEAR(m.At(row, col), expected, kTol);
        }
    }
}

TEST(Mat3, OrthonormalRotationTransposeIsInverse) {
    // For a pure rotation, R^T * (R * v) recovers v.
    const double c = std::cos(0.7);
    const double s = std::sin(0.7);
    const Mat3 rotation(std::array<double, 9>{c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0});
    const Vec3 v(1.3, -2.7, 4.1);
    const Vec3 recovered = rotation.Transposed() * (rotation * v);
    EXPECT_NEAR(recovered.x, v.x, 1e-9);
    EXPECT_NEAR(recovered.y, v.y, 1e-9);
    EXPECT_NEAR(recovered.z, v.z, 1e-9);
}
