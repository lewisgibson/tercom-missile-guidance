#pragma once

// Simulated sensor backends for the software-in-the-loop harness. Each holds the
// latest ground-truth quantity (set by the simulator) and corrupts it with a
// deterministic, seeded noise and bias model on Read(), so the navigator sees
// realistic measurements without any real hardware.

#include <cstdint>
#include <random>

#include "tercom/geo/geo_point.h"
#include "tercom/math/linear.h"
#include "tercom/sensors/barometric_altimeter.h"
#include "tercom/sensors/gnss_receiver.h"
#include "tercom/sensors/imu.h"
#include "tercom/sensors/radar_altimeter.h"

namespace TERCOM { namespace Sensors {

    struct SimulatedImuConfiguration {
        // Constant accelerometer bias added to every sample, in metres per second
        // squared, one value per body axis. A bias is a fixed offset the sensor
        // adds on top of the true value; these few-centimetre-per-second-squared
        // numbers are representative of a real inertial unit and are the kind of
        // error the navigation filter is meant to estimate out. The exact values
        // are arbitrary nonzero magnitudes chosen to exercise that estimator.
        Math::Vec3 accelerometer_bias_mps2{0.03, -0.02, 0.04};
        // Navigation-grade gyroscope bias, order of 0.1 deg/hr, as found in the ring
        // laser or fibre-optic gyroscopes of a cruise-missile inertial unit. The
        // nine-state filter does not estimate attitude or gyroscope bias, so it relies
        // on tilt-coupled gravity error staying negligible; a navigation-grade gyroscope
        // makes that true. A tactical-grade gyroscope (order of 10 deg/hr) would need the
        // fifteen-state extension described in the documentation. Stored in radians
        // per second: 0.1 deg/hr is about 4.8e-7 rad/s (degrees times pi/180, then
        // divided by 3600 seconds per hour), so these per-axis values are roughly
        // 0.1, 0.06, and 0.12 deg/hr.
        Math::Vec3 gyroscope_bias_radps{4.8e-7, -3.0e-7, 6.0e-7};
        // White-noise standard deviation added independently to each sample. "std"
        // is the one-sigma spread of a zero-mean Gaussian: about two thirds of
        // samples land within plus or minus this much of the true value. These are
        // per-sample magnitudes, not noise densities, and are representative tactical
        // values rather than derived from a specific datasheet.
        double accelerometer_noise_std_mps2 = 0.02; // accelerometer noise, metres per second squared
        double gyroscope_noise_std_radps = 1.0e-4;  // gyroscope noise, radians per second
        // Seed for the pseudo-random generator. Fixing it makes every run produce
        // the identical noise stream, so simulations are reproducible. The value is
        // an arbitrary constant; only that it is fixed matters.
        std::uint64_t seed = 0x1A2B3C4D5E6Full;
    };

    class SimulatedImu : public IImu {
      public:
        explicit SimulatedImu(const SimulatedImuConfiguration& configuration = {});

        // Argument order matches the ImuSample fields: angular rate first, then
        // specific force.
        void SetTruth(const Math::Vec3& angular_rate_body_radps, const Math::Vec3& specific_force_body_mps2, double time_s);
        ImuSample Read() override;

        const Math::Vec3& TrueAccelerometerBias() const {
            return configuration_.accelerometer_bias_mps2;
        }

      private:
        SimulatedImuConfiguration configuration_;
        std::mt19937_64 generator_;
        std::normal_distribution<double> standard_normal_{0.0, 1.0};
        ImuSample truth_{};
    };

    struct SimulatedBarometerConfiguration {
        // Constant offset added to every reported altitude, in metres. A barometric
        // altimeter infers height from air pressure, so its zero drifts with weather
        // (a different sea-level pressure shifts every reading by the same amount);
        // six metres is a plausible standing offset for an uncalibrated unit.
        double bias_m = 6.0;
        // One-sigma white noise on each reading, in metres. Representative short-term
        // jitter for a pressure altimeter.
        double noise_std_m = 1.5;
        std::uint64_t seed = 0xBA12000Full; // fixed seed for reproducible noise
    };

    class SimulatedBarometer : public IBarometricAltimeter {
      public:
        explicit SimulatedBarometer(const SimulatedBarometerConfiguration& configuration = {});

        void SetTruth(double true_altitude_m, double time_s);
        BarometricSample Read() override;

      private:
        SimulatedBarometerConfiguration configuration_;
        std::mt19937_64 generator_;
        std::normal_distribution<double> standard_normal_{0.0, 1.0};
        double true_altitude_m_ = 0.0;
        double time_s_ = 0.0;
        bool have_truth_ = false;
    };

    struct SimulatedRadarAltimeterConfiguration {
        // Radar range error grows with distance, so noise is modelled as a fraction
        // of the measured clearance (1 percent, one sigma) rather than a fixed metre
        // value: a reading of 200 m carries about 2 m of one-sigma noise.
        double noise_fraction = 0.01; // one sigma, fraction of clearance
        // Lower bound on that noise, in metres, so very low clearances still carry a
        // realistic minimum error instead of collapsing toward zero.
        double noise_floor_m = 0.5;
        // Maximum sensing range, in metres. Beyond this the beam gets no usable
        // ground return and Read() reports an invalid sample. Typical of a
        // low-altitude terrain-following radar altimeter.
        double max_range_m = 5000.0;
        std::uint64_t seed = 0x4ADA4001ull; // fixed seed for reproducible noise
    };

    class SimulatedRadarAltimeter : public IRadarAltimeter {
      public:
        explicit SimulatedRadarAltimeter(const SimulatedRadarAltimeterConfiguration& configuration = {});

        void SetTruth(double true_clearance_m, double time_s);
        RadarAltitudeSample Read() override;

      private:
        SimulatedRadarAltimeterConfiguration configuration_;
        std::mt19937_64 generator_;
        std::normal_distribution<double> standard_normal_{0.0, 1.0};
        double true_clearance_m_ = 0.0;
        double time_s_ = 0.0;
        bool have_truth_ = false;
    };

    struct SimulatedGnssConfiguration {
        // One-sigma position noise, in metres. Vertical error is larger than
        // horizontal because satellite geometry constrains height less well, which
        // is typical of real GNSS fixes.
        double horizontal_std_m = 3.0;
        double vertical_std_m = 5.0;
        // Number of satellites reported in each fix. Only carried through for
        // realism and downstream quality checks; nine is a healthy open-sky count.
        int satellites = 9;
        bool available = false;         // off by default: TRN is the GPS-denied path
        std::uint64_t seed = 0x6055ull; // fixed seed for reproducible noise
    };

    class SimulatedGnss : public IGnssReceiver {
      public:
        explicit SimulatedGnss(const SimulatedGnssConfiguration& configuration = {});

        void SetTruth(const Geo::GeoPoint& true_position, double time_s);
        GnssFix Read() override;

      private:
        SimulatedGnssConfiguration configuration_;
        std::mt19937_64 generator_;
        std::normal_distribution<double> standard_normal_{0.0, 1.0};
        Geo::GeoPoint true_position_{};
        double time_s_ = 0.0;
        bool have_truth_ = false;
    };

}} // namespace TERCOM::Sensors
