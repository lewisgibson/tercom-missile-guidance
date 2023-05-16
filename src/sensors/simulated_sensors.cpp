#include "tercom/sensors/simulated_sensors.h"

#include <algorithm>
#include <cmath>

#include "tercom/math/linear.h"

namespace TERCOM { namespace Sensors {

    SimulatedImu::SimulatedImu(const SimulatedImuConfiguration& configuration) : configuration_(configuration), generator_(configuration.seed) {
    }

    void SimulatedImu::SetTruth(const Math::Vec3& angular_rate_body_radps, const Math::Vec3& specific_force_body_mps2, double time_s) {
        truth_.angular_rate_radps = angular_rate_body_radps;
        truth_.specific_force_mps2 = specific_force_body_mps2;
        truth_.time_s = time_s;
    }

    ImuSample SimulatedImu::Read() {
        // Build a corrupted sample from the stored truth: true value plus the fixed
        // bias plus a fresh independent Gaussian draw on each axis. standard_normal_
        // yields a zero-mean unit-variance number, so multiplying by the standard
        // deviation scales it to the desired one-sigma noise level.
        const double an = configuration_.accelerometer_noise_std_mps2;
        const double gn = configuration_.gyroscope_noise_std_radps;

        ImuSample sample;
        sample.time_s = truth_.time_s;
        sample.specific_force_mps2 = truth_.specific_force_mps2 + configuration_.accelerometer_bias_mps2
                                     + Math::Vec3{standard_normal_(generator_) * an, standard_normal_(generator_) * an, standard_normal_(generator_) * an};
        sample.angular_rate_radps = truth_.angular_rate_radps + configuration_.gyroscope_bias_radps
                                    + Math::Vec3{standard_normal_(generator_) * gn, standard_normal_(generator_) * gn, standard_normal_(generator_) * gn};
        return sample;
    }

    SimulatedBarometer::SimulatedBarometer(const SimulatedBarometerConfiguration& configuration) : configuration_(configuration), generator_(configuration.seed) {
    }

    void SimulatedBarometer::SetTruth(double true_altitude_m, double time_s) {
        true_altitude_m_ = true_altitude_m;
        time_s_ = time_s;
        have_truth_ = true;
    }

    BarometricSample SimulatedBarometer::Read() {
        BarometricSample sample;
        sample.time_s = time_s_;
        sample.valid = have_truth_;
        sample.altitude_m = true_altitude_m_ + configuration_.bias_m + (standard_normal_(generator_) * configuration_.noise_std_m);
        return sample;
    }

    SimulatedRadarAltimeter::SimulatedRadarAltimeter(const SimulatedRadarAltimeterConfiguration& configuration) : configuration_(configuration), generator_(configuration.seed) {
    }

    void SimulatedRadarAltimeter::SetTruth(double true_clearance_m, double time_s) {
        true_clearance_m_ = true_clearance_m;
        time_s_ = time_s;
        have_truth_ = true;
    }

    RadarAltitudeSample SimulatedRadarAltimeter::Read() {
        RadarAltitudeSample sample;
        sample.time_s = time_s_;

        if (!have_truth_ || true_clearance_m_ < 0.0 || true_clearance_m_ > configuration_.max_range_m) {
            sample.valid = false;
            return sample;
        }

        // Noise scales with clearance but never falls below the floor, so the error
        // is the larger of "1 percent of range" and "the minimum metre value".
        const double sigma = std::max(configuration_.noise_floor_m, configuration_.noise_fraction * true_clearance_m_);
        sample.clearance_m = true_clearance_m_ + (standard_normal_(generator_) * sigma);
        sample.valid = true;
        return sample;
    }

    SimulatedGnss::SimulatedGnss(const SimulatedGnssConfiguration& configuration) : configuration_(configuration), generator_(configuration.seed) {
    }

    void SimulatedGnss::SetTruth(const Geo::GeoPoint& true_position, double time_s) {
        true_position_ = true_position;
        time_s_ = time_s;
        have_truth_ = true;
    }

    GnssFix SimulatedGnss::Read() {
        GnssFix fix;
        fix.time_s = time_s_;

        if (!configuration_.available || !have_truth_) {
            fix.valid = false;
            return fix;
        }

        // Approximate metres-to-degrees at this latitude for the horizontal noise.
        // One degree of latitude is about 111320 m everywhere (a mean great-circle
        // value). Lines of longitude crowd together toward the poles, so a degree of
        // longitude is shorter by a factor of cos(latitude); dividing by that factor
        // converts an east-west metre offset back into degrees.
        constexpr double kMetresPerDegree = 111320.0;
        const double cos_lat = std::cos(Math::DegToRad(true_position_.latitude_deg));
        const double east_noise = standard_normal_(generator_) * configuration_.horizontal_std_m;
        const double north_noise = standard_normal_(generator_) * configuration_.horizontal_std_m;

        fix.position.latitude_deg = true_position_.latitude_deg + (north_noise / kMetresPerDegree);
        // The 1e-6 floor avoids dividing by a near-zero cosine at the poles.
        fix.position.longitude_deg = true_position_.longitude_deg + (east_noise / (kMetresPerDegree * (std::abs(cos_lat) > 1e-6 ? cos_lat : 1e-6)));
        fix.position.altitude_m = true_position_.altitude_m + (standard_normal_(generator_) * configuration_.vertical_std_m);
        fix.satellites = configuration_.satellites;
        fix.valid = true;
        return fix;
    }

}} // namespace TERCOM::Sensors
