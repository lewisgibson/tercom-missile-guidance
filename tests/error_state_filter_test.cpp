#include <gtest/gtest.h>

#include "tercom/math/attitude.h"
#include "tercom/math/linear.h"
#include "tercom/math/matrix.h"
#include "tercom/nav/error_state_filter.h"

#include <cmath>

namespace {

    using TERCOM::Math::kPi;
    using TERCOM::Math::Mat3;
    using TERCOM::Math::Matrix;
    using TERCOM::Math::QuaternionFromEuler;
    using TERCOM::Math::RotationMatrixFromQuaternion;
    using TERCOM::Math::Vec3;
    using TERCOM::Nav::ErrorStateFilter;
    using TERCOM::Nav::ErrorStateFilterConfiguration;

    // Build a 2x2 covariance with the given diagonal variances and zero
    // off-diagonal correlation.
    Matrix<2, 2> DiagonalCovariance(double var_east, double var_north) {
        Matrix<2, 2> covariance{};
        covariance(0, 0) = var_east;
        covariance(1, 1) = var_north;
        return covariance;
    }

    TEST(ErrorStateFilter, FreshFilterHasZeroCorrections) {
        ErrorStateFilter filter;

        const Vec3 position = filter.PositionCorrection();
        const Vec3 velocity = filter.VelocityCorrection();
        const Vec3 bias = filter.AccelerometerBiasCorrection();

        EXPECT_NEAR(position.x, 0.0, 1e-12);
        EXPECT_NEAR(position.y, 0.0, 1e-12);
        EXPECT_NEAR(position.z, 0.0, 1e-12);
        EXPECT_NEAR(velocity.x, 0.0, 1e-12);
        EXPECT_NEAR(velocity.y, 0.0, 1e-12);
        EXPECT_NEAR(velocity.z, 0.0, 1e-12);
        EXPECT_NEAR(bias.x, 0.0, 1e-12);
        EXPECT_NEAR(bias.y, 0.0, 1e-12);
        EXPECT_NEAR(bias.z, 0.0, 1e-12);
    }

    TEST(ErrorStateFilter, FreshFilterReportsConfiguredStds) {
        ErrorStateFilterConfiguration configuration;
        configuration.initial_position_std_m = 150.0;
        ErrorStateFilter filter(configuration);

        // Horizontal std averages the two horizontal position variances, both of
        // which start at initial_position_std_m^2, so the std equals it exactly.
        EXPECT_NEAR(filter.HorizontalStdM(), 150.0, 1e-9);
        EXPECT_NEAR(filter.AltitudeStdM(), 150.0, 1e-9);
    }

    TEST(ErrorStateFilter, PredictGrowsHorizontalStd) {
        ErrorStateFilter filter;

        const double initial_std = filter.HorizontalStdM();
        filter.Predict(1.0, Mat3::Identity());
        const double grown_std = filter.HorizontalStdM();

        EXPECT_GT(grown_std, initial_std);
    }

    TEST(ErrorStateFilter, PredictWithNonPositiveDtIsNoOp) {
        ErrorStateFilter filter;

        const double before = filter.HorizontalStdM();
        filter.Predict(0.0, Mat3::Identity());
        EXPECT_NEAR(filter.HorizontalStdM(), before, 1e-12);

        filter.Predict(-5.0, Mat3::Identity());
        EXPECT_NEAR(filter.HorizontalStdM(), before, 1e-12);
    }

    TEST(ErrorStateFilter, UpdateHorizontalPositionPullsTowardMeasurement) {
        ErrorStateFilter filter;

        const double innovation_east = 30.0;
        const double innovation_north = -20.0;
        const Matrix<2, 2> covariance = DiagonalCovariance(25.0, 25.0);

        filter.UpdateHorizontalPosition(innovation_east, innovation_north, covariance);

        const Vec3 correction = filter.PositionCorrection();

        // The Kalman gain drives the error state toward the innovation, and the
        // correction is the negated error state, so the correction is opposite in
        // sign to the innovation.
        EXPECT_LT(correction.x, 0.0);
        EXPECT_GT(correction.y, 0.0);

        // The correction magnitude is a fraction of the innovation magnitude: the
        // gain is between zero and one because the measurement has non-zero noise.
        EXPECT_LT(std::abs(correction.x), std::abs(innovation_east));
        EXPECT_LT(std::abs(correction.y), std::abs(innovation_north));
        EXPECT_GT(std::abs(correction.x), 0.0);
        EXPECT_GT(std::abs(correction.y), 0.0);

        // The vertical channel is unobserved by a horizontal fix.
        EXPECT_NEAR(correction.z, 0.0, 1e-9);
    }

    TEST(ErrorStateFilter, PositionFixCouplesIntoVelocityAndBiasViaCrossCovariance) {
        // Estimating accelerometer bias is the entire reason it is a filter state: a
        // position fix must also sharpen the velocity and bias estimates through the
        // cross-covariance that Predict builds between them. Propagate first so those
        // off-diagonal terms are non-zero, then fold in an east-only horizontal fix.
        ErrorStateFilter filter;
        for (int i = 0; i < 50; ++i) {
            filter.Predict(0.1, Mat3::Identity());
        }
        filter.UpdateHorizontalPosition(50.0, 0.0, DiagonalCovariance(1.0, 1.0));

        const Vec3 velocity = filter.VelocityCorrection();
        const Vec3 bias = filter.AccelerometerBiasCorrection();

        // The east fix moves the east velocity and east accelerometer-bias estimates,
        // which are correlated with east position after prediction.
        EXPECT_GT(std::abs(velocity.x), 0.0);
        EXPECT_GT(std::abs(bias.x), 0.0);
        // With an identity body-to-nav rotation the east and north channels are
        // decoupled and the north innovation is zero, so the north velocity/bias
        // channels do not move, and a horizontal fix never touches the vertical.
        EXPECT_NEAR(velocity.y, 0.0, 1e-9);
        EXPECT_NEAR(bias.y, 0.0, 1e-9);
        EXPECT_NEAR(velocity.z, 0.0, 1e-9);
        EXPECT_NEAR(bias.z, 0.0, 1e-9);
    }

    TEST(ErrorStateFilter, YawedPredictRotatesBiasCouplingAxis) {
        // The accelerometer-bias-to-velocity coupling uses the body-to-nav rotation R.
        // With an identity rotation an east position fix couples into the body-x bias
        // (see PositionFixCouplesIntoVelocityAndBiasViaCrossCovariance). A +90 deg yaw
        // makes R asymmetric (R != R^T), so the coupled bias channel must move to
        // body-y. An implementation that dropped R or transposed it would keep the
        // coupling on body-x, which this test catches.
        const Mat3 yaw90 = RotationMatrixFromQuaternion(QuaternionFromEuler(0.0, 0.0, kPi / 2.0));

        ErrorStateFilter filter;
        for (int i = 0; i < 50; ++i) {
            filter.Predict(0.1, yaw90);
        }
        filter.UpdateHorizontalPosition(50.0, 0.0, DiagonalCovariance(1.0, 1.0));

        const Vec3 bias = filter.AccelerometerBiasCorrection();
        EXPECT_GT(std::abs(bias.y), 0.0);
        EXPECT_NEAR(bias.x, 0.0, 1e-9);
        EXPECT_NEAR(bias.z, 0.0, 1e-9);
    }

    TEST(ErrorStateFilter, UpdateHorizontalPositionShrinksStd) {
        ErrorStateFilter filter;

        const double before = filter.HorizontalStdM();
        filter.UpdateHorizontalPosition(30.0, -20.0, DiagonalCovariance(25.0, 25.0));
        const double after = filter.HorizontalStdM();

        EXPECT_LT(after, before);
    }

    TEST(ErrorStateFilter, TighterFixGivesLargerCorrection) {
        ErrorStateFilter tight;
        ErrorStateFilter loose;

        const double innovation_east = 30.0;
        const double innovation_north = -20.0;

        tight.UpdateHorizontalPosition(innovation_east, innovation_north, DiagonalCovariance(1.0, 1.0));
        loose.UpdateHorizontalPosition(innovation_east, innovation_north, DiagonalCovariance(10000.0, 10000.0));

        // A more confident measurement (smaller covariance) yields a larger gain
        // and therefore a larger correction toward it.
        EXPECT_GT(std::abs(tight.PositionCorrection().x), std::abs(loose.PositionCorrection().x));
        EXPECT_GT(std::abs(tight.PositionCorrection().y), std::abs(loose.PositionCorrection().y));
    }

    TEST(ErrorStateFilter, UpdateAltitudeMovesZAndShrinksAltitudeStd) {
        ErrorStateFilter filter;

        const double before_std = filter.AltitudeStdM();
        filter.UpdateAltitude(40.0, 9.0);
        const double after_std = filter.AltitudeStdM();

        const Vec3 correction = filter.PositionCorrection();

        // Up-innovation drives the up error state positive, so the correction (its
        // negation) is negative, and a fraction of the innovation magnitude.
        EXPECT_LT(correction.z, 0.0);
        EXPECT_LT(std::abs(correction.z), 40.0);
        EXPECT_GT(std::abs(correction.z), 0.0);

        // The altitude update only observes the vertical channel.
        EXPECT_NEAR(correction.x, 0.0, 1e-9);
        EXPECT_NEAR(correction.y, 0.0, 1e-9);

        EXPECT_LT(after_std, before_std);
    }

    TEST(ErrorStateFilter, SecondUpdateSubtractsCarriedErrorState) {
        // Two altitude updates with no reset between them. The second update's residual
        // is (innovation - carried error state), so its result must reflect the error
        // already accumulated by the first update. This is the only path that exercises
        // the (h * error_state_) term with a non-zero error state; dropping or
        // sign-flipping it would change the answer.
        ErrorStateFilter filter;

        filter.UpdateAltitude(40.0, 9.0);
        const double error_after_first = -filter.PositionCorrection().z; // == error_state_(2, 0)
        ASSERT_GT(std::abs(error_after_first), 0.0);

        // The up channel is decoupled here (no Predict ran), so the scalar Kalman gain
        // is the closed-form P / (P + R) and the prediction is exact.
        const double variance_before_second = filter.AltitudeStdM() * filter.AltitudeStdM();
        const double measurement_variance = 9.0;
        const double gain = variance_before_second / (variance_before_second + measurement_variance);
        const double innovation = 10.0;
        const double expected_error_after = error_after_first + (gain * (innovation - error_after_first));

        filter.UpdateAltitude(innovation, measurement_variance);

        EXPECT_NEAR(-filter.PositionCorrection().z, expected_error_after, 1e-9);
    }

    TEST(ErrorStateFilter, ResetErrorStateZeroesCorrections) {
        ErrorStateFilter filter;

        filter.UpdateHorizontalPosition(30.0, -20.0, DiagonalCovariance(25.0, 25.0));
        filter.UpdateAltitude(40.0, 9.0);

        // Corrections are non-zero after folding in measurements.
        ASSERT_GT(std::abs(filter.PositionCorrection().x), 0.0);
        ASSERT_GT(std::abs(filter.PositionCorrection().z), 0.0);

        filter.ResetErrorState();

        const Vec3 position = filter.PositionCorrection();
        const Vec3 velocity = filter.VelocityCorrection();
        const Vec3 bias = filter.AccelerometerBiasCorrection();

        EXPECT_NEAR(position.x, 0.0, 1e-12);
        EXPECT_NEAR(position.y, 0.0, 1e-12);
        EXPECT_NEAR(position.z, 0.0, 1e-12);
        EXPECT_NEAR(velocity.x, 0.0, 1e-12);
        EXPECT_NEAR(velocity.y, 0.0, 1e-12);
        EXPECT_NEAR(velocity.z, 0.0, 1e-12);
        EXPECT_NEAR(bias.x, 0.0, 1e-12);
        EXPECT_NEAR(bias.y, 0.0, 1e-12);
        EXPECT_NEAR(bias.z, 0.0, 1e-12);
    }

    TEST(ErrorStateFilter, ResetErrorStateLeavesCovarianceIntact) {
        ErrorStateFilter filter;

        filter.UpdateHorizontalPosition(30.0, -20.0, DiagonalCovariance(25.0, 25.0));
        const double std_after_update = filter.HorizontalStdM();

        filter.ResetErrorState();

        // Resetting only zeroes the error state; the covariance is unchanged.
        EXPECT_NEAR(filter.HorizontalStdM(), std_after_update, 1e-12);
    }

    TEST(ErrorStateFilter, GatedUpdateAcceptsConsistentInnovation) {
        ErrorStateFilter filter;

        // A modest innovation against the large initial covariance has a small NIS
        // and so passes a generous chi-square gate.
        const bool accepted = filter.UpdateHorizontalPositionGated(30.0, -20.0, DiagonalCovariance(25.0, 25.0), 11.83);

        EXPECT_TRUE(accepted);

        // An accepted fix still applies a normal correction toward the measurement.
        EXPECT_LT(filter.PositionCorrection().x, 0.0);
        EXPECT_GT(filter.PositionCorrection().y, 0.0);
    }

    TEST(ErrorStateFilter, GatedUpdateRejectsInconsistentInnovation) {
        ErrorStateFilter filter;

        // A several-kilometre innovation with a metre-scale covariance produces a
        // huge NIS that fails any sane gate.
        const bool accepted = filter.UpdateHorizontalPositionGated(5000.0, -4000.0, DiagonalCovariance(1.0, 1.0), 11.83);

        EXPECT_FALSE(accepted);
    }

    TEST(ErrorStateFilter, GatedUpdateStillAppliesButDownWeightsRejectedFix) {
        ErrorStateFilter gated;
        ErrorStateFilter ungated;

        const double innovation_east = 5000.0;
        const double innovation_north = -4000.0;
        const Matrix<2, 2> covariance = DiagonalCovariance(1.0, 1.0);

        // The soft gate does not "accept" the inconsistent innovation, but unlike a
        // hard gate it still folds in a down-weighted update rather than discarding
        // it: the state moves, in the implied direction, but by strictly less than
        // the full-strength ungated update for the same innovation.
        const bool accepted = gated.UpdateHorizontalPositionGated(innovation_east, innovation_north, covariance, 11.83);
        ungated.UpdateHorizontalPosition(innovation_east, innovation_north, covariance);
        ASSERT_FALSE(accepted);

        const Vec3 g = gated.PositionCorrection();
        const Vec3 u = ungated.PositionCorrection();

        EXPECT_LT(g.x, 0.0);
        EXPECT_GT(g.y, 0.0);
        EXPECT_GT(std::abs(g.x), 0.0);
        EXPECT_GT(std::abs(g.y), 0.0);
        EXPECT_LT(std::abs(g.x), std::abs(u.x));
        EXPECT_LT(std::abs(g.y), std::abs(u.y));
    }

    TEST(ErrorStateFilter, GatedUpdateHeavilyDownWeightsHugeInnovationAfterCovarianceCollapse) {
        ErrorStateFilter gated;
        ErrorStateFilter ungated;

        // Drive the covariance down with a run of consistent fixes, reaching the
        // confident regime where a hard gate would starve. Resetting the error
        // state each time leaves the covariance to keep shrinking.
        for (int i = 0; i < 100; ++i) {
            gated.UpdateHorizontalPositionGated(0.0, 0.0, DiagonalCovariance(1.0, 1.0), 11.83);
            ungated.UpdateHorizontalPosition(0.0, 0.0, DiagonalCovariance(1.0, 1.0));
            gated.ResetErrorState();
            ungated.ResetErrorState();
        }
        ASSERT_LT(gated.HorizontalStdM(), 1.0);

        const double innovation_east = 5000.0;
        const double innovation_north = -4000.0;
        const Matrix<2, 2> covariance = DiagonalCovariance(1.0, 1.0);

        const bool accepted = gated.UpdateHorizontalPositionGated(innovation_east, innovation_north, covariance, 11.83);
        ungated.UpdateHorizontalPosition(innovation_east, innovation_north, covariance);
        ASSERT_FALSE(accepted);

        const Vec3 g = gated.PositionCorrection();
        const Vec3 u = ungated.PositionCorrection();

        // With the covariance collapsed, the inflation dominates the gain: the gated
        // correction is orders of magnitude smaller than the ungated one. This is
        // exactly the case where a hard gate would lock out recovery.
        EXPECT_LT(std::abs(g.x), 0.01 * std::abs(u.x));
        EXPECT_LT(std::abs(g.y), 0.01 * std::abs(u.y));
    }

    TEST(ErrorStateFilter, GatedUpdateMatchesUngatedForConsistentInnovation) {
        ErrorStateFilter gated;
        ErrorStateFilter ungated;

        const double innovation_east = 30.0;
        const double innovation_north = -20.0;
        const Matrix<2, 2> covariance = DiagonalCovariance(25.0, 25.0);

        const bool accepted = gated.UpdateHorizontalPositionGated(innovation_east, innovation_north, covariance, 11.83);
        ungated.UpdateHorizontalPosition(innovation_east, innovation_north, covariance);

        // A consistent fix passes the gate, so no inflation is applied (inflation
        // factor floors at one) and the gated correction matches the ungated one.
        ASSERT_TRUE(accepted);
        EXPECT_NEAR(gated.PositionCorrection().x, ungated.PositionCorrection().x, 1e-9);
        EXPECT_NEAR(gated.PositionCorrection().y, ungated.PositionCorrection().y, 1e-9);
    }

} // namespace
