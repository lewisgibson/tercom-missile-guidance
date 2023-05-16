#include <gtest/gtest.h>

#include <cmath>

#include "tercom/geo/geo_point.h"
#include "tercom/math/linear.h"
#include "tercom/sensors/simulated_sensors.h"

namespace {

    using TERCOM::Geo::GeoPoint;
    using TERCOM::Math::Vec3;
    using namespace TERCOM::Sensors;

    TEST(SimulatedImu, AddsBiasAndIsSeededDeterministic) {
        SimulatedImuConfiguration configuration;
        configuration.accelerometer_noise_std_mps2 = 0.0; // isolate the bias
        configuration.gyroscope_noise_std_radps = 0.0;
        SimulatedImu imu(configuration);
        imu.SetTruth(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0}, 1.0);

        const ImuSample sample = imu.Read();
        EXPECT_NEAR(sample.specific_force_mps2.x, configuration.accelerometer_bias_mps2.x, 1e-12);
        EXPECT_NEAR(sample.specific_force_mps2.y, configuration.accelerometer_bias_mps2.y, 1e-12);
        EXPECT_NEAR(sample.angular_rate_radps.z, configuration.gyroscope_bias_radps.z, 1e-12);
        EXPECT_DOUBLE_EQ(sample.time_s, 1.0);
        EXPECT_EQ(imu.TrueAccelerometerBias().x, configuration.accelerometer_bias_mps2.x);
    }

    TEST(SimulatedImu, SameSeedSameNoiseSequence) {
        SimulatedImu a;
        SimulatedImu b;
        a.SetTruth(Vec3{0.1, 0.2, 0.3}, Vec3{1.0, 2.0, 3.0}, 0.5); // (angular rate, specific force)
        b.SetTruth(Vec3{0.1, 0.2, 0.3}, Vec3{1.0, 2.0, 3.0}, 0.5);
        const ImuSample sa = a.Read();
        const ImuSample sb = b.Read();
        EXPECT_DOUBLE_EQ(sa.specific_force_mps2.x, sb.specific_force_mps2.x);
        EXPECT_DOUBLE_EQ(sa.angular_rate_radps.y, sb.angular_rate_radps.y);
    }

    TEST(SimulatedImu, NoiseStdPerturbsReadingAwayFromTruthPlusBias) {
        // With a non-zero noise std the seeded draw must move the reading off the
        // exact truth-plus-bias value (the noise term is actually applied, not dropped).
        SimulatedImuConfiguration configuration;
        configuration.accelerometer_noise_std_mps2 = 0.5;
        configuration.gyroscope_noise_std_radps = 0.01;
        SimulatedImu imu(configuration);
        imu.SetTruth(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0}, 1.0);

        const ImuSample sample = imu.Read();
        EXPECT_NE(sample.specific_force_mps2.x, configuration.accelerometer_bias_mps2.x);
        EXPECT_NE(sample.angular_rate_radps.z, configuration.gyroscope_bias_radps.z);
    }

    TEST(SimulatedBarometer, InvalidUntilTruthSet) {
        SimulatedBarometer baro;
        EXPECT_FALSE(baro.Read().valid);
        baro.SetTruth(1000.0, 2.0);
        const BarometricSample sample = baro.Read();
        EXPECT_TRUE(sample.valid);
        EXPECT_DOUBLE_EQ(sample.time_s, 2.0);
    }

    TEST(SimulatedBarometer, AppliesConfiguredBias) {
        SimulatedBarometerConfiguration configuration;
        configuration.noise_std_m = 0.0;
        configuration.bias_m = 7.0;
        SimulatedBarometer baro(configuration);
        baro.SetTruth(500.0, 0.0);
        EXPECT_NEAR(baro.Read().altitude_m, 507.0, 1e-9);
    }

    TEST(SimulatedBarometer, NoiseStdPerturbsReadingAwayFromTruthPlusBias) {
        // truth + bias would be exactly 507.0; a non-zero noise std must move it.
        SimulatedBarometerConfiguration configuration;
        configuration.noise_std_m = 2.0;
        configuration.bias_m = 7.0;
        SimulatedBarometer baro(configuration);
        baro.SetTruth(500.0, 0.0);
        EXPECT_NE(baro.Read().altitude_m, 507.0);
    }

    TEST(SimulatedRadarAltimeter, NoiseFloorGovernsAtLowClearanceFractionGovernsAtHighClearance) {
        // The noise sigma is max(noise_floor_m, noise_fraction * clearance). Capture
        // the deterministic first Gaussian draw with sigma == 1 (floor 1, fraction 0),
        // then reuse the same seed to predict the exact perturbed clearance in each
        // regime, pinning which term wins the max.
        SimulatedRadarAltimeterConfiguration probe;
        probe.noise_fraction = 0.0;
        probe.noise_floor_m = 1.0;
        probe.seed = 99U;
        SimulatedRadarAltimeter probe_radar(probe);
        probe_radar.SetTruth(100.0, 0.0);
        const double draw = probe_radar.Read().clearance_m - 100.0; // == standard_normal_ * 1.0

        SimulatedRadarAltimeterConfiguration governed; // defaults: fraction 0.01, floor 0.5
        governed.seed = probe.seed;

        // Floor-dominated: at 10 m, fraction * clearance = 0.1 < floor 0.5.
        SimulatedRadarAltimeter low_radar(governed);
        low_radar.SetTruth(10.0, 0.0);
        EXPECT_NEAR(low_radar.Read().clearance_m, 10.0 + (draw * 0.5), 1e-9);

        // Fraction-dominated: at 1000 m, fraction * clearance = 10 > floor 0.5.
        SimulatedRadarAltimeter high_radar(governed);
        high_radar.SetTruth(1000.0, 0.0);
        EXPECT_NEAR(high_radar.Read().clearance_m, 1000.0 + (draw * 0.01 * 1000.0), 1e-9);
    }

    TEST(SimulatedRadarAltimeter, ValidWithinRange) {
        SimulatedRadarAltimeterConfiguration configuration;
        configuration.noise_fraction = 0.0;
        configuration.noise_floor_m = 0.0;
        SimulatedRadarAltimeter radar(configuration);
        radar.SetTruth(300.0, 1.0);
        const RadarAltitudeSample sample = radar.Read();
        EXPECT_TRUE(sample.valid);
        EXPECT_NEAR(sample.clearance_m, 300.0, 1e-9);
    }

    TEST(SimulatedRadarAltimeter, InvalidBeyondRangeOrNegativeOrUnset) {
        SimulatedRadarAltimeterConfiguration configuration;
        configuration.max_range_m = 1000.0;
        SimulatedRadarAltimeter radar(configuration);
        EXPECT_FALSE(radar.Read().valid); // no truth yet

        radar.SetTruth(5000.0, 0.0); // beyond range
        EXPECT_FALSE(radar.Read().valid);

        radar.SetTruth(-10.0, 0.0); // negative clearance
        EXPECT_FALSE(radar.Read().valid);
    }

    TEST(SimulatedGnss, OffByDefault) {
        SimulatedGnss gnss;
        gnss.SetTruth(GeoPoint{48.0, 11.0, 500.0}, 1.0);
        EXPECT_FALSE(gnss.Read().valid); // available is false by default
    }

    TEST(SimulatedGnss, ValidWhenEnabled) {
        SimulatedGnssConfiguration configuration;
        configuration.available = true;
        configuration.horizontal_std_m = 0.0;
        configuration.vertical_std_m = 0.0;
        configuration.satellites = 11;
        SimulatedGnss gnss(configuration);

        EXPECT_FALSE(gnss.Read().valid); // no truth yet

        const GeoPoint truth{48.1173, 11.5167, 545.0};
        gnss.SetTruth(truth, 3.0);
        const GnssFix fix = gnss.Read();
        ASSERT_TRUE(fix.valid);
        EXPECT_EQ(fix.satellites, 11);
        EXPECT_NEAR(fix.position.latitude_deg, truth.latitude_deg, 1e-6);
        EXPECT_NEAR(fix.position.longitude_deg, truth.longitude_deg, 1e-6);
        EXPECT_NEAR(fix.position.altitude_m, truth.altitude_m, 1e-9);
        EXPECT_DOUBLE_EQ(fix.time_s, 3.0);
    }

    TEST(SimulatedGnss, LongitudeNoiseScalesWithInverseCosineLatitude) {
        // Two receivers with the SAME seed draw the SAME east/north noise sequence.
        // The east-driven longitude offset is divided by cos(latitude), so at 60 deg
        // it must be exactly 1/cos(60 deg) = 2x the equator offset. This pins the
        // metres-to-degrees longitude scaling and its divide-by-cos(lat) behaviour.
        SimulatedGnssConfiguration configuration;
        configuration.available = true;
        configuration.horizontal_std_m = 1000.0; // large, so the perturbation dominates
        configuration.vertical_std_m = 0.0;
        configuration.seed = 12345U;

        SimulatedGnss at_equator(configuration);
        SimulatedGnss at_sixty(configuration);
        const GeoPoint equator{0.0, 10.0, 0.0};
        const GeoPoint sixty{60.0, 10.0, 0.0};
        at_equator.SetTruth(equator, 1.0);
        at_sixty.SetTruth(sixty, 1.0);

        const GnssFix fix0 = at_equator.Read();
        const GnssFix fix60 = at_sixty.Read();
        ASSERT_TRUE(fix0.valid);
        ASSERT_TRUE(fix60.valid);

        const double longitude_offset_equator = fix0.position.longitude_deg - equator.longitude_deg;
        const double longitude_offset_sixty = fix60.position.longitude_deg - sixty.longitude_deg;
        ASSERT_NE(longitude_offset_equator, 0.0);
        EXPECT_NEAR(longitude_offset_sixty / longitude_offset_equator, 1.0 / std::cos(TERCOM::Math::DegToRad(60.0)), 1e-9);

        // Latitude uses a fixed metres-per-degree scale everywhere, so the same north
        // draw yields an identical latitude offset at both sites.
        const double latitude_offset_equator = fix0.position.latitude_deg - equator.latitude_deg;
        const double latitude_offset_sixty = fix60.position.latitude_deg - sixty.latitude_deg;
        EXPECT_NEAR(latitude_offset_equator, latitude_offset_sixty, 1e-12);
    }

    TEST(SimulatedGnss, VerticalStdPerturbsAltitudeAwayFromTruth) {
        // With a non-zero vertical std the seeded draw must move the returned altitude
        // off the exact truth value, and it must use the vertical std (not the
        // horizontal one): zeroing the horizontal std leaves lat/lon exactly on truth,
        // so any altitude perturbation can only have come from vertical_std_m.
        SimulatedGnssConfiguration configuration;
        configuration.available = true;
        configuration.horizontal_std_m = 0.0;
        configuration.vertical_std_m = 5.0;
        configuration.seed = 4242U;
        SimulatedGnss gnss(configuration);

        const GeoPoint truth{48.0, 11.0, 500.0};
        gnss.SetTruth(truth, 1.0);
        const GnssFix fix = gnss.Read();
        ASSERT_TRUE(fix.valid);
        EXPECT_DOUBLE_EQ(fix.position.latitude_deg, truth.latitude_deg);
        EXPECT_DOUBLE_EQ(fix.position.longitude_deg, truth.longitude_deg);
        EXPECT_NE(fix.position.altitude_m, truth.altitude_m);
    }

    TEST(SimulatedGnss, NearPoleLongitudeGuardKeepsOffsetBounded) {
        // At the pole cos(latitude) is ~6e-17. Without the 1e-6 floor the longitude
        // conversion would divide by that and explode to ~1e14 degrees; the guard
        // clamps the divisor so the offset stays bounded (here well under 1e6 deg).
        SimulatedGnssConfiguration configuration;
        configuration.available = true;
        configuration.horizontal_std_m = 1000.0;
        configuration.vertical_std_m = 0.0;
        configuration.seed = 7U;
        SimulatedGnss gnss(configuration);

        gnss.SetTruth(GeoPoint{90.0, 10.0, 0.0}, 1.0);
        const GnssFix fix = gnss.Read();
        ASSERT_TRUE(fix.valid);
        EXPECT_TRUE(std::isfinite(fix.position.longitude_deg));
        EXPECT_LT(std::abs(fix.position.longitude_deg - 10.0), 1.0e6);
    }

} // namespace
