#include <gtest/gtest.h>

#include <cmath>

#include "tercom/math/attitude.h"
#include "tercom/math/linear.h"

namespace {

    using TERCOM::Math::EulerFromQuaternion;
    using TERCOM::Math::Mat3;
    using TERCOM::Math::Norm;
    using TERCOM::Math::Quaternion;
    using TERCOM::Math::QuaternionFromEuler;
    using TERCOM::Math::QuaternionFromRotationVector;
    using TERCOM::Math::Rotate;
    using TERCOM::Math::RotationMatrixFromQuaternion;
    using TERCOM::Math::RotationVectorFromQuaternion;
    using TERCOM::Math::Vec3;

    constexpr double kTol = 1e-9;
    constexpr double kPi = TERCOM::Math::kPi;

    void ExpectQuaternionNear(const Quaternion& a, const Quaternion& b, double tol) {
        EXPECT_NEAR(a.w, b.w, tol);
        EXPECT_NEAR(a.x, b.x, tol);
        EXPECT_NEAR(a.y, b.y, tol);
        EXPECT_NEAR(a.z, b.z, tol);
    }

    void ExpectVec3Near(const Vec3& a, const Vec3& b, double tol) {
        EXPECT_NEAR(a.x, b.x, tol);
        EXPECT_NEAR(a.y, b.y, tol);
        EXPECT_NEAR(a.z, b.z, tol);
    }

} // namespace

TEST(AttitudeQuaternion, DefaultIsIdentity) {
    const Quaternion q;
    EXPECT_NEAR(q.w, 1.0, kTol);
    EXPECT_NEAR(q.x, 0.0, kTol);
    EXPECT_NEAR(q.y, 0.0, kTol);
    EXPECT_NEAR(q.z, 0.0, kTol);
}

TEST(AttitudeQuaternion, ComponentConstructor) {
    const Quaternion q{0.5, -0.25, 0.75, -0.125};
    EXPECT_NEAR(q.w, 0.5, kTol);
    EXPECT_NEAR(q.x, -0.25, kTol);
    EXPECT_NEAR(q.y, 0.75, kTol);
    EXPECT_NEAR(q.z, -0.125, kTol);
}

TEST(AttitudeQuaternion, ConjugateNegatesVectorPart) {
    const Quaternion q{0.5, 0.1, -0.2, 0.3};
    const Quaternion c = q.Conjugate();
    EXPECT_NEAR(c.w, 0.5, kTol);
    EXPECT_NEAR(c.x, -0.1, kTol);
    EXPECT_NEAR(c.y, 0.2, kTol);
    EXPECT_NEAR(c.z, -0.3, kTol);
}

TEST(AttitudeQuaternion, NormOfKnownQuaternion) {
    const Quaternion q{1.0, 2.0, 2.0, 4.0};
    // sqrt(1 + 4 + 4 + 16) = sqrt(25) = 5.
    EXPECT_NEAR(q.Norm(), 5.0, kTol);
}

TEST(AttitudeQuaternion, ProductWithConjugateIsIdentity) {
    // A general non-unit quaternion, normalized so q * conj(q) == identity.
    const Quaternion q = Quaternion{0.6, 0.1, -0.3, 0.4}.Normalized();
    const Quaternion product = q * q.Conjugate();
    ExpectQuaternionNear(product, Quaternion{1.0, 0.0, 0.0, 0.0}, kTol);
}

TEST(AttitudeQuaternion, IdentityTimesIdentityIsIdentity) {
    const Quaternion id{1.0, 0.0, 0.0, 0.0};
    const Quaternion product = id * id;
    ExpectQuaternionNear(product, id, kTol);
}

TEST(AttitudeQuaternion, HamiltonProductIsNonCommutative) {
    // i * j = k, but j * i = -k under the Hamilton convention.
    const Quaternion i{0.0, 1.0, 0.0, 0.0};
    const Quaternion j{0.0, 0.0, 1.0, 0.0};
    const Quaternion ij = i * j;
    const Quaternion ji = j * i;
    ExpectQuaternionNear(ij, Quaternion{0.0, 0.0, 0.0, 1.0}, kTol);
    ExpectQuaternionNear(ji, Quaternion{0.0, 0.0, 0.0, -1.0}, kTol);
}

TEST(AttitudeQuaternion, NormalizedHasUnitNorm) {
    const Quaternion q = Quaternion{2.0, -1.0, 3.0, 0.5}.Normalized();
    EXPECT_NEAR(q.Norm(), 1.0, kTol);
}

TEST(AttitudeQuaternion, NormalizedPreservesDirection) {
    const Quaternion q{2.0, 0.0, 0.0, 0.0};
    const Quaternion n = q.Normalized();
    ExpectQuaternionNear(n, Quaternion{1.0, 0.0, 0.0, 0.0}, kTol);
}

TEST(AttitudeQuaternion, NormalizedZeroReturnsIdentity) {
    const Quaternion q{0.0, 0.0, 0.0, 0.0};
    const Quaternion n = q.Normalized();
    ExpectQuaternionNear(n, Quaternion{1.0, 0.0, 0.0, 0.0}, kTol);
}

TEST(AttitudeRotate, IdentityLeavesVectorUnchanged) {
    const Quaternion id{1.0, 0.0, 0.0, 0.0};
    const Vec3 v{1.5, -2.5, 3.0};
    const Vec3 r = Rotate(id, v);
    ExpectVec3Near(r, v, kTol);
}

TEST(AttitudeRotate, NinetyDegreesAboutZMapsXToY) {
    // Right-handed 90-degree yaw about up (z) sends body-x into nav-y.
    const Quaternion q = QuaternionFromEuler(0.0, 0.0, kPi / 2.0);
    const Vec3 r = Rotate(q, Vec3{1.0, 0.0, 0.0});
    ExpectVec3Near(r, Vec3{0.0, 1.0, 0.0}, kTol);
}

TEST(AttitudeRotate, NinetyDegreesAboutZMapsYToNegativeX) {
    const Quaternion q = QuaternionFromEuler(0.0, 0.0, kPi / 2.0);
    const Vec3 r = Rotate(q, Vec3{0.0, 1.0, 0.0});
    ExpectVec3Near(r, Vec3{-1.0, 0.0, 0.0}, kTol);
}

TEST(AttitudeRotate, RotationPreservesLength) {
    const Quaternion q = QuaternionFromEuler(0.3, -0.4, 1.1);
    const Vec3 v{1.0, -2.0, 3.0};
    const Vec3 r = Rotate(q, v);
    EXPECT_NEAR(Norm(r), Norm(v), kTol);
}

TEST(AttitudeRotate, RotationAboutZAxisLeavesZAxisFixed) {
    const Quaternion q = QuaternionFromEuler(0.0, 0.0, 0.7);
    const Vec3 r = Rotate(q, Vec3{0.0, 0.0, 1.0});
    ExpectVec3Near(r, Vec3{0.0, 0.0, 1.0}, kTol);
}

TEST(AttitudeEuler, IdentityQuaternionGivesZeroEuler) {
    const Vec3 euler = EulerFromQuaternion(Quaternion{1.0, 0.0, 0.0, 0.0});
    ExpectVec3Near(euler, Vec3{0.0, 0.0, 0.0}, kTol);
}

TEST(AttitudeEuler, RoundTripSmallAngles) {
    const double roll = 0.05;
    const double pitch = -0.03;
    const double yaw = 0.08;
    const Quaternion q = QuaternionFromEuler(roll, pitch, yaw);
    const Vec3 euler = EulerFromQuaternion(q);
    EXPECT_NEAR(euler.x, roll, kTol);
    EXPECT_NEAR(euler.y, pitch, kTol);
    EXPECT_NEAR(euler.z, yaw, kTol);
}

TEST(AttitudeEuler, RoundTripModerateAngles) {
    const double roll = 0.6;
    const double pitch = -0.4;
    const double yaw = 1.2;
    const Quaternion q = QuaternionFromEuler(roll, pitch, yaw);
    const Vec3 euler = EulerFromQuaternion(q);
    EXPECT_NEAR(euler.x, roll, kTol);
    EXPECT_NEAR(euler.y, pitch, kTol);
    EXPECT_NEAR(euler.z, yaw, kTol);
}

TEST(AttitudeEuler, RoundTripNegativeYaw) {
    const double roll = -0.2;
    const double pitch = 0.35;
    const double yaw = -2.5;
    const Quaternion q = QuaternionFromEuler(roll, pitch, yaw);
    const Vec3 euler = EulerFromQuaternion(q);
    EXPECT_NEAR(euler.x, roll, kTol);
    EXPECT_NEAR(euler.y, pitch, kTol);
    EXPECT_NEAR(euler.z, yaw, kTol);
}

TEST(AttitudeEuler, GimbalLockPitchSaturatesAtPositiveHalfPi) {
    // sin_pitch >= 1.0 should clamp pitch to +pi/2.
    const Quaternion q = QuaternionFromEuler(0.0, kPi / 2.0, 0.0);
    const Vec3 euler = EulerFromQuaternion(q);
    EXPECT_NEAR(euler.y, kPi / 2.0, 1e-6);
}

TEST(AttitudeEuler, GimbalLockPitchSaturatesAtNegativeHalfPi) {
    const Quaternion q = QuaternionFromEuler(0.0, -kPi / 2.0, 0.0);
    const Vec3 euler = EulerFromQuaternion(q);
    EXPECT_NEAR(euler.y, -kPi / 2.0, 1e-6);
}

TEST(AttitudeRotationMatrix, IdentityQuaternionGivesIdentityMatrix) {
    const Mat3 r = RotationMatrixFromQuaternion(Quaternion{1.0, 0.0, 0.0, 0.0});
    const Mat3 id = Mat3::Identity();
    for (std::size_t i = 0u; i < 3u; ++i) {
        for (std::size_t j = 0u; j < 3u; ++j) {
            EXPECT_NEAR(r.At(i, j), id.At(i, j), kTol);
        }
    }
}

TEST(AttitudeRotationMatrix, MatrixTimesVectorMatchesRotate) {
    const Quaternion q = QuaternionFromEuler(0.4, -0.25, 0.9);
    const Mat3 r = RotationMatrixFromQuaternion(q);

    const Vec3 vectors[] = {
        Vec3{1.0, 0.0, 0.0},
        Vec3{0.0, 1.0, 0.0},
        Vec3{0.0, 0.0, 1.0},
        Vec3{1.0, -2.0, 3.0},
        Vec3{-4.0, 5.0, -6.0},
    };
    for (const Vec3& v : vectors) {
        const Vec3 by_matrix = r * v;
        const Vec3 by_quat = Rotate(q, v);
        ExpectVec3Near(by_matrix, by_quat, kTol);
    }
}

TEST(AttitudeRotationMatrix, IsOrthonormal) {
    // R * R^T should be the identity for a unit quaternion.
    const Quaternion q = QuaternionFromEuler(0.7, 0.2, -1.3);
    const Mat3 r = RotationMatrixFromQuaternion(q);
    const Mat3 rt = r.Transposed();
    for (std::size_t i = 0u; i < 3u; ++i) {
        for (std::size_t j = 0u; j < 3u; ++j) {
            double sum = 0.0;
            for (std::size_t k = 0u; k < 3u; ++k) {
                sum += r.At(i, k) * rt.At(k, j);
            }
            const double expected = (i == j) ? 1.0 : 0.0;
            EXPECT_NEAR(sum, expected, kTol);
        }
    }
}

TEST(AttitudeRotationMatrix, NormalizesNonUnitInput) {
    // Passing a non-unit quaternion should still yield an orthonormal matrix
    // equivalent to the normalized rotation.
    const Quaternion scaled{2.0, 0.0, 0.0, 0.0};
    const Mat3 r = RotationMatrixFromQuaternion(scaled);
    const Mat3 id = Mat3::Identity();
    for (std::size_t i = 0u; i < 3u; ++i) {
        for (std::size_t j = 0u; j < 3u; ++j) {
            EXPECT_NEAR(r.At(i, j), id.At(i, j), kTol);
        }
    }
}

TEST(AttitudeRotationVector, RoundTripKnownVector) {
    const Vec3 rotation{0.3, -0.5, 0.7};
    const Quaternion q = QuaternionFromRotationVector(rotation);
    const Vec3 recovered = RotationVectorFromQuaternion(q);
    ExpectVec3Near(recovered, rotation, kTol);
}

TEST(AttitudeRotationVector, RoundTripPureZAxis) {
    const Vec3 rotation{0.0, 0.0, kPi / 3.0};
    const Quaternion q = QuaternionFromRotationVector(rotation);
    const Vec3 recovered = RotationVectorFromQuaternion(q);
    ExpectVec3Near(recovered, rotation, kTol);
}

TEST(AttitudeRotationVector, ProducesUnitQuaternion) {
    const Vec3 rotation{0.2, 0.4, -0.1};
    const Quaternion q = QuaternionFromRotationVector(rotation);
    EXPECT_NEAR(q.Norm(), 1.0, kTol);
}

TEST(AttitudeRotationVector, SmallAngleBranchReturnsNearIdentity) {
    // Magnitude well below the 1e-12 threshold exercises the small-angle limit.
    const Vec3 tiny{1e-15, -1e-15, 1e-15};
    const Quaternion q = QuaternionFromRotationVector(tiny);
    ExpectQuaternionNear(q, Quaternion{1.0, 0.0, 0.0, 0.0}, 1e-12);
    EXPECT_NEAR(q.Norm(), 1.0, kTol);
}

TEST(AttitudeRotationVector, ZeroRotationGivesIdentityQuaternion) {
    const Quaternion q = QuaternionFromRotationVector(Vec3{0.0, 0.0, 0.0});
    ExpectQuaternionNear(q, Quaternion{1.0, 0.0, 0.0, 0.0}, kTol);
}

TEST(AttitudeRotationVector, ZeroQuaternionAxisGivesZeroVector) {
    // Identity quaternion has zero vector part, so the recovered rotation is zero.
    const Vec3 v = RotationVectorFromQuaternion(Quaternion{1.0, 0.0, 0.0, 0.0});
    ExpectVec3Near(v, Vec3{0.0, 0.0, 0.0}, kTol);
}

TEST(AttitudeRotationVector, ShortestRotationFromNegativeHemisphere) {
    // A quaternion with negative scalar describes the same rotation as its
    // negation; the recovered vector should pick the shortest (positive-w) one.
    const Vec3 rotation{0.1, 0.2, 0.3};
    const Quaternion q = QuaternionFromRotationVector(rotation);
    const Quaternion negated{-q.w, -q.x, -q.y, -q.z};
    const Vec3 recovered = RotationVectorFromQuaternion(negated);
    ExpectVec3Near(recovered, rotation, kTol);
}

TEST(AttitudeRotationVector, NinetyDegreesAboutZRotatesXToY) {
    const Vec3 rotation{0.0, 0.0, kPi / 2.0};
    const Quaternion q = QuaternionFromRotationVector(rotation);
    const Vec3 r = Rotate(q, Vec3{1.0, 0.0, 0.0});
    ExpectVec3Near(r, Vec3{0.0, 1.0, 0.0}, kTol);
}

TEST(AttitudeRotationVector, RoundTripPiRotationAboutX) {
    // At a half-turn the scalar part w -> 0, the boundary of the log map. The
    // recovered vector has magnitude pi about the same axis; the axis SIGN is a
    // genuine ambiguity at exactly pi, so assert magnitude and the physical effect
    // (a pi rotation about x flips the two perpendicular axes, fixes x) rather than
    // raw signed components.
    const Vec3 rotation{kPi, 0.0, 0.0};
    const Quaternion q = QuaternionFromRotationVector(rotation);

    const Vec3 recovered = RotationVectorFromQuaternion(q);
    EXPECT_NEAR(Norm(recovered), kPi, 1e-9);
    EXPECT_NEAR(std::fabs(recovered.x), kPi, 1e-9);
    EXPECT_NEAR(recovered.y, 0.0, 1e-9);
    EXPECT_NEAR(recovered.z, 0.0, 1e-9);

    ExpectVec3Near(Rotate(q, Vec3{1.0, 0.0, 0.0}), Vec3{1.0, 0.0, 0.0}, kTol);
    ExpectVec3Near(Rotate(q, Vec3{0.0, 1.0, 0.0}), Vec3{0.0, -1.0, 0.0}, kTol);
    ExpectVec3Near(Rotate(q, Vec3{0.0, 0.0, 1.0}), Vec3{0.0, 0.0, -1.0}, kTol);
}

TEST(AttitudeComposition, EulerQuaternionMatchesRotationMatrixRotate) {
    // Full chain: Euler -> quaternion -> matrix; matrix and quaternion agree.
    const Quaternion q = QuaternionFromEuler(0.15, 0.6, -0.45);
    const Mat3 r = RotationMatrixFromQuaternion(q);
    const Vec3 v{0.5, 1.5, -2.0};
    ExpectVec3Near(r * v, Rotate(q, v), kTol);
}
