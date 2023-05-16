#include <gtest/gtest.h>

#include "tercom/math/attitude.h"
#include "tercom/math/linear.h"
#include "tercom/nav/nav_state.h"
#include "tercom/nav/strapdown_ins.h"
#include "tercom/sensors/measurements.h"

#include <cmath>

namespace {

    using TERCOM::Math::Quaternion;
    using TERCOM::Math::Vec3;
    using TERCOM::Nav::kGravityEnu;
    using TERCOM::Nav::NavState;
    using TERCOM::Nav::StrapdownIns;
    using TERCOM::Sensors::ImuSample;

    // Specific force a perfect IMU reports for level, non-rotating flight: it must
    // cancel gravity, so it points up by exactly g (FLU body z aligns with ENU up
    // at identity attitude).
    constexpr double kG = 9.80665;
    constexpr Vec3 kLevelSpecificForce{0.0, 0.0, kG};
    constexpr double kTol = 1e-6;

    // An IMU sample for level, gravity-only flight at the given timestamp.
    ImuSample LevelSample(double time_s) {
        ImuSample sample;
        sample.angular_rate_radps = Vec3{0.0, 0.0, 0.0};
        sample.specific_force_mps2 = kLevelSpecificForce;
        sample.time_s = time_s;
        return sample;
    }

} // namespace

TEST(StrapdownIns, GravityConstantMatchesNavState) {
    EXPECT_NEAR(kGravityEnu.x, 0.0, kTol);
    EXPECT_NEAR(kGravityEnu.y, 0.0, kTol);
    EXPECT_NEAR(kGravityEnu.z, -kG, kTol);
}

TEST(StrapdownIns, ConstructorStoresInitialState) {
    NavState initial;
    initial.time_s = 1.5;
    initial.position_enu_m = Vec3{10.0, 20.0, 30.0};
    initial.velocity_enu_mps = Vec3{1.0, 2.0, 3.0};

    StrapdownIns ins(initial);
    const NavState& state = ins.State();

    EXPECT_NEAR(state.time_s, 1.5, kTol);
    EXPECT_NEAR(state.position_enu_m.x, 10.0, kTol);
    EXPECT_NEAR(state.position_enu_m.y, 20.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 30.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.x, 1.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.y, 2.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.z, 3.0, kTol);
}

TEST(StrapdownIns, LevelAtRestHoldsPositionAndVelocity) {
    NavState initial;
    initial.time_s = 0.0;
    initial.position_enu_m = Vec3{100.0, 200.0, 500.0};
    initial.velocity_enu_mps = Vec3{0.0, 0.0, 0.0};

    StrapdownIns ins(initial);

    const double dt = 0.01;
    double t = 0.0;
    for (int i = 0; i < 50; ++i) {
        t += dt;
        ins.Step(LevelSample(t));
    }

    const NavState& state = ins.State();
    EXPECT_NEAR(state.time_s, t, kTol);

    // Specific force cancels gravity, so the body is in free equilibrium:
    // position and velocity must stay essentially unchanged.
    EXPECT_NEAR(state.position_enu_m.x, 100.0, kTol);
    EXPECT_NEAR(state.position_enu_m.y, 200.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 500.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.x, 0.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.y, 0.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.z, 0.0, kTol);
}

TEST(StrapdownIns, ConstantEastVelocityAdvancesEast) {
    NavState initial;
    initial.time_s = 0.0;
    initial.position_enu_m = Vec3{0.0, 0.0, 1000.0};
    initial.velocity_enu_mps = Vec3{50.0, 0.0, 0.0};

    StrapdownIns ins(initial);

    const double dt = 0.01;
    double t = 0.0;
    const int steps = 100;
    for (int i = 0; i < steps; ++i) {
        t += dt;
        ins.Step(LevelSample(t));
    }

    const NavState& state = ins.State();
    EXPECT_NEAR(state.time_s, t, kTol);

    // Gravity is cancelled, so the east velocity stays constant and position
    // advances by v * t. North and up are untouched.
    EXPECT_NEAR(state.velocity_enu_mps.x, 50.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.y, 0.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.z, 0.0, kTol);
    EXPECT_NEAR(state.position_enu_m.x, 50.0 * t, kTol);
    EXPECT_NEAR(state.position_enu_m.y, 0.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 1000.0, kTol);
}

TEST(StrapdownIns, FreeFallWithZeroSpecificForceAccelerometererates) {
    NavState initial;
    initial.time_s = 0.0;
    initial.position_enu_m = Vec3{0.0, 0.0, 1000.0};
    initial.velocity_enu_mps = Vec3{0.0, 0.0, 0.0};

    StrapdownIns ins(initial);

    // A free-falling perfect IMU reads zero specific force: only gravity acts.
    ImuSample sample;
    sample.angular_rate_radps = Vec3{0.0, 0.0, 0.0};
    sample.specific_force_mps2 = Vec3{0.0, 0.0, 0.0};
    sample.time_s = 1.0;
    ins.Step(sample);

    const NavState& state = ins.State();
    // v = g * t with g downward; trapezoidal drop = 0.5 * g * t^2 over one step.
    EXPECT_NEAR(state.velocity_enu_mps.z, -kG * 1.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 1000.0 - (0.5 * kG * 1.0 * 1.0), kTol);
    EXPECT_NEAR(state.velocity_enu_mps.x, 0.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.y, 0.0, kTol);
}

TEST(StrapdownIns, OutOfOrderSampleOnlyAdvancesClock) {
    NavState initial;
    initial.time_s = 5.0;
    initial.position_enu_m = Vec3{1.0, 2.0, 3.0};
    initial.velocity_enu_mps = Vec3{4.0, 5.0, 6.0};

    StrapdownIns ins(initial);

    // Timestamp predates the current clock: state must not integrate.
    ins.Step(LevelSample(4.0));

    const NavState& state = ins.State();
    EXPECT_NEAR(state.time_s, 4.0, kTol);
    EXPECT_NEAR(state.position_enu_m.x, 1.0, kTol);
    EXPECT_NEAR(state.position_enu_m.y, 2.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 3.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.x, 4.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.y, 5.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.z, 6.0, kTol);
}

TEST(StrapdownIns, DuplicateTimestampOnlyAdvancesClock) {
    NavState initial;
    initial.time_s = 2.0;
    initial.position_enu_m = Vec3{7.0, 8.0, 9.0};
    initial.velocity_enu_mps = Vec3{0.0, 0.0, 0.0};

    StrapdownIns ins(initial);

    // dt == 0: same timestamp, no integration.
    ins.Step(LevelSample(2.0));

    const NavState& state = ins.State();
    EXPECT_NEAR(state.time_s, 2.0, kTol);
    EXPECT_NEAR(state.position_enu_m.x, 7.0, kTol);
    EXPECT_NEAR(state.position_enu_m.y, 8.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 9.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.x, 0.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.y, 0.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.z, 0.0, kTol);
}

TEST(StrapdownIns, AccelerometerBiasIsRemovedFromSpecificForce) {
    NavState initial;
    initial.time_s = 0.0;
    initial.position_enu_m = Vec3{0.0, 0.0, 0.0};
    initial.velocity_enu_mps = Vec3{0.0, 0.0, 0.0};
    // The state believes there is a +2 m/s^2 up bias on the accelerometer.
    initial.accelerometer_bias_mps2 = Vec3{0.0, 0.0, 2.0};

    StrapdownIns ins(initial);

    // The IMU reports gravity-cancelling force plus the bias it carries, so the
    // corrected specific force is exactly level and the state stays at rest.
    ImuSample sample;
    sample.angular_rate_radps = Vec3{0.0, 0.0, 0.0};
    sample.specific_force_mps2 = Vec3{0.0, 0.0, kG + 2.0};
    sample.time_s = 1.0;
    ins.Step(sample);

    const NavState& state = ins.State();
    EXPECT_NEAR(state.velocity_enu_mps.z, 0.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 0.0, kTol);
}

TEST(StrapdownIns, GyroscopeBiasIsRemovedFromAngularRate) {
    NavState initial;
    initial.time_s = 0.0;
    initial.attitude = Quaternion{1.0, 0.0, 0.0, 0.0};
    // The state believes there is a yaw-rate bias of 0.5 rad/s about up.
    initial.gyroscope_bias_radps = Vec3{0.0, 0.0, 0.5};

    StrapdownIns ins(initial);

    // The IMU reports exactly that bias, so the corrected rate is zero and the
    // attitude must remain the identity quaternion.
    ImuSample sample;
    sample.angular_rate_radps = Vec3{0.0, 0.0, 0.5};
    sample.specific_force_mps2 = kLevelSpecificForce;
    sample.time_s = 1.0;
    ins.Step(sample);

    const NavState& state = ins.State();
    EXPECT_NEAR(state.attitude.w, 1.0, kTol);
    EXPECT_NEAR(state.attitude.x, 0.0, kTol);
    EXPECT_NEAR(state.attitude.y, 0.0, kTol);
    EXPECT_NEAR(state.attitude.z, 0.0, kTol);
}

TEST(StrapdownIns, AngularRateRotatesAttitudeAboutUp) {
    NavState initial;
    initial.time_s = 0.0;
    initial.attitude = Quaternion{1.0, 0.0, 0.0, 0.0};

    StrapdownIns ins(initial);

    const double rate = 1.0; // rad/s about body up
    const double duration = 1.0;
    ImuSample sample;
    sample.angular_rate_radps = Vec3{0.0, 0.0, rate};
    sample.specific_force_mps2 = kLevelSpecificForce;
    sample.time_s = duration;
    ins.Step(sample);

    // A single step rotates by rate * duration about the up axis. The resulting
    // quaternion is (cos(theta/2), 0, 0, sin(theta/2)).
    const double half = (rate * duration) * 0.5;
    const NavState& state = ins.State();
    EXPECT_NEAR(state.attitude.w, std::cos(half), kTol);
    EXPECT_NEAR(state.attitude.x, 0.0, kTol);
    EXPECT_NEAR(state.attitude.y, 0.0, kTol);
    EXPECT_NEAR(state.attitude.z, std::sin(half), kTol);
}

TEST(StrapdownIns, ApplyCorrectionAddsToState) {
    NavState initial;
    initial.time_s = 3.0;
    initial.position_enu_m = Vec3{10.0, 20.0, 30.0};
    initial.velocity_enu_mps = Vec3{1.0, 2.0, 3.0};
    initial.accelerometer_bias_mps2 = Vec3{0.1, 0.2, 0.3};

    StrapdownIns ins(initial);
    ins.ApplyCorrection(Vec3{1.0, -2.0, 3.0}, Vec3{0.5, 0.5, -0.5}, Vec3{0.01, -0.01, 0.02});

    const NavState& state = ins.State();
    EXPECT_NEAR(state.position_enu_m.x, 11.0, kTol);
    EXPECT_NEAR(state.position_enu_m.y, 18.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 33.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.x, 1.5, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.y, 2.5, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.z, 2.5, kTol);
    EXPECT_NEAR(state.accelerometer_bias_mps2.x, 0.11, kTol);
    EXPECT_NEAR(state.accelerometer_bias_mps2.y, 0.19, kTol);
    EXPECT_NEAR(state.accelerometer_bias_mps2.z, 0.32, kTol);

    // The correction must not touch the clock.
    EXPECT_NEAR(state.time_s, 3.0, kTol);
}

TEST(StrapdownIns, ApplyZeroCorrectionLeavesStateUnchanged) {
    NavState initial;
    initial.position_enu_m = Vec3{5.0, 6.0, 7.0};
    initial.velocity_enu_mps = Vec3{8.0, 9.0, 10.0};
    initial.accelerometer_bias_mps2 = Vec3{1.0, 1.0, 1.0};

    StrapdownIns ins(initial);
    ins.ApplyCorrection(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0});

    const NavState& state = ins.State();
    EXPECT_NEAR(state.position_enu_m.x, 5.0, kTol);
    EXPECT_NEAR(state.position_enu_m.y, 6.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 7.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.x, 8.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.y, 9.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.z, 10.0, kTol);
    EXPECT_NEAR(state.accelerometer_bias_mps2.x, 1.0, kTol);
    EXPECT_NEAR(state.accelerometer_bias_mps2.y, 1.0, kTol);
    EXPECT_NEAR(state.accelerometer_bias_mps2.z, 1.0, kTol);
}

TEST(StrapdownIns, SetStateReplacesState) {
    NavState initial;
    initial.time_s = 0.0;
    initial.position_enu_m = Vec3{0.0, 0.0, 0.0};

    StrapdownIns ins(initial);

    NavState replacement;
    replacement.time_s = 42.0;
    replacement.position_enu_m = Vec3{100.0, 200.0, 300.0};
    replacement.velocity_enu_mps = Vec3{11.0, 12.0, 13.0};
    replacement.accelerometer_bias_mps2 = Vec3{0.5, 0.6, 0.7};
    replacement.gyroscope_bias_radps = Vec3{0.01, 0.02, 0.03};

    ins.SetState(replacement);

    const NavState& state = ins.State();
    EXPECT_NEAR(state.time_s, 42.0, kTol);
    EXPECT_NEAR(state.position_enu_m.x, 100.0, kTol);
    EXPECT_NEAR(state.position_enu_m.y, 200.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 300.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.x, 11.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.y, 12.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.z, 13.0, kTol);
    EXPECT_NEAR(state.accelerometer_bias_mps2.x, 0.5, kTol);
    EXPECT_NEAR(state.gyroscope_bias_radps.z, 0.03, kTol);
}

TEST(StrapdownIns, FirstSampleBeforeStateOnlySetsClock) {
    // The header notes the first sample after construction only sets the clock
    // if it predates the state. Construct at t = 10 and feed an earlier sample.
    NavState initial;
    initial.time_s = 10.0;
    initial.position_enu_m = Vec3{1.0, 1.0, 1.0};
    initial.velocity_enu_mps = Vec3{2.0, 2.0, 2.0};

    StrapdownIns ins(initial);
    ins.Step(LevelSample(9.0));

    const NavState& state = ins.State();
    EXPECT_NEAR(state.time_s, 9.0, kTol);
    EXPECT_NEAR(state.position_enu_m.x, 1.0, kTol);
    EXPECT_NEAR(state.velocity_enu_mps.x, 2.0, kTol);
}

TEST(StrapdownIns, CorrectionThenIntegrationUsesCorrectedVelocity) {
    NavState initial;
    initial.time_s = 0.0;
    initial.position_enu_m = Vec3{0.0, 0.0, 0.0};
    initial.velocity_enu_mps = Vec3{0.0, 0.0, 0.0};

    StrapdownIns ins(initial);

    // Inject a north velocity via a correction, then integrate gravity-only.
    ins.ApplyCorrection(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 25.0, 0.0}, Vec3{0.0, 0.0, 0.0});

    const double dt = 0.01;
    double t = 0.0;
    for (int i = 0; i < 40; ++i) {
        t += dt;
        ins.Step(LevelSample(t));
    }

    const NavState& state = ins.State();
    EXPECT_NEAR(state.velocity_enu_mps.y, 25.0, kTol);
    EXPECT_NEAR(state.position_enu_m.y, 25.0 * t, kTol);
    EXPECT_NEAR(state.position_enu_m.x, 0.0, kTol);
    EXPECT_NEAR(state.position_enu_m.z, 0.0, kTol);
}
