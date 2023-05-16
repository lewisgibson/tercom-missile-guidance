#pragma once

// Error-state Kalman filter for terrain-referenced navigation.
//
// A Kalman filter is a recursive estimator that tracks a state plus its
// covariance (uncertainty): Predict propagates both forward in time using a
// motion model, and each Update fuses a new measurement, weighting it against the
// state by their relative uncertainties. An *error-state* filter does not track
// the navigation state itself (the strapdown INS already does that, very
// accurately over short spans); it tracks the small, slowly drifting *error* in
// that state. The error dynamics are nearly linear even when the vehicle is not,
// which is what lets a linear Kalman filter apply to a non-linear problem.
//
// This filter estimates the INS error in nine states: position (3), velocity (3),
// and accelerometer bias (3), all in the local ENU frame. It runs closed loop:
// Predict propagates the error covariance between measurements, the barometric
// and TERCOM updates fold in observations, and the resulting corrections are fed
// back into the strapdown INS, after which the error state is reset to zero (so
// the estimate is always "zero error" and the INS carries the absolute state).
//
// Why these nine and not also attitude and gyroscope bias: the sensors here can only
// observe position (terrain and baro). With no measurement that constrains
// heading or attitude error, adding those states would leave them unobservable,
// and an unobservable state in a Kalman filter drifts and can corrupt the
// observable ones. The dominant remaining error source that *is* coupled into
// position is accelerometer bias (it integrates twice into a growing position
// error), so it earns a place; attitude and gyroscope errors are instead lumped into
// an inflated velocity process noise (see accelerometer_noise_density_mps2 below).
//
// Estimating the accelerometer bias lets the filter reduce drift between terrain
// fixes rather than merely snapping back at each one. The vertical channel is
// held stable by an always-on barometric altitude update, as a nadir (straight
// down) terrain measurement alone cannot observe absolute altitude.

#include <cstddef>

#include "tercom/math/linear.h"
#include "tercom/math/matrix.h"

namespace TERCOM { namespace Nav {

    struct ErrorStateFilterConfiguration {
        // Initial one-sigma uncertainties seeded onto the covariance diagonal at
        // construction. They state how far the INS could be off before the first
        // fix; the filter shrinks them as measurements arrive, so exact values
        // matter only transiently. position 150 m reflects a coarse pre-launch /
        // last-GNSS fix, velocity 3 m/s a typical INS alignment error, and the
        // accelerometer bias 0.05 m/s^2 (~5 milli-g) a tactical-grade sensor.
        double initial_position_std_m = 150.0;
        double initial_velocity_std_mps = 3.0;
        double initial_accelerometer_bias_std_mps2 = 0.05;

        // Process noise: how much uncertainty the motion model itself injects each
        // step, given as continuous spectral densities (standard deviation per
        // root second), squared and multiplied by dt in Predict to form variances.
        //
        // White accelerometer noise that drives the velocity random walk. 0.04
        // m/s^2/sqrt(s) is larger than a real accelerometer's noise on purpose: it
        // also stands in for the heading and attitude drift this nine-state filter
        // does not estimate, keeping the velocity channel honestly uncertain.
        double accelerometer_noise_density_mps2 = 0.04;
        // Accelerometer bias random walk, m/s^2 per root second (i.e. m/s^3 in the
        // squared-and-times-dt sense). 2e-4 lets the estimated bias wander slowly
        // so the filter can track real bias drift without chasing measurement noise.
        double accelerometer_bias_random_walk_mps3 = 2e-4;
        // Small direct position process noise, 0.01 m/s. Keeps the position
        // covariance from collapsing to zero over long dead-reckoning stretches
        // between fixes, which would make the filter overconfident and reject good
        // fixes at the gate.
        double position_process_std_mps = 0.01;
    };

    class ErrorStateFilter {
      public:
        explicit ErrorStateFilter(const ErrorStateFilterConfiguration& configuration = {});

        // Propagate the covariance over dt using the current body-to-nav rotation.
        void Predict(double dt_s, const Math::Mat3& body_to_nav);

        // Fold in a measurement of absolute altitude (ENU up). innovation_up is
        // the nominal INS up-position minus the measured altitude.
        void UpdateAltitude(double innovation_up_m, double variance_m2);

        // Fold in a horizontal position fix. innovation_* is the nominal INS
        // position minus the fix; covariance is the 2x2 fix covariance (e, n).
        void UpdateHorizontalPosition(double innovation_east_m, double innovation_north_m, const Math::Matrix<2, 2>& covariance);

        // Soft-gated horizontal update. The fix is always folded in, but one whose
        // normalised innovation squared exceeds the chi-square gate (two degrees of
        // freedom) has its measurement covariance inflated in proportion to the
        // excess, so an inconsistent fix (a likely false terrain match) is heavily
        // down-weighted rather than rejected outright. A hard reject would starve
        // the filter once its covariance has collapsed; this never can. Returns true
        // when the fix was consistent (within the gate), false when down-weighted.
        bool UpdateHorizontalPositionGated(double innovation_east_m, double innovation_north_m, const Math::Matrix<2, 2>& covariance, double gate_chi2);

        // Corrections to add into the INS state to remove the estimated error.
        Math::Vec3 PositionCorrection() const;
        Math::Vec3 VelocityCorrection() const;
        Math::Vec3 AccelerometerBiasCorrection() const;

        // Zero the error state after its corrections have been applied (closed loop).
        void ResetErrorState();

        // One-sigma horizontal position uncertainty, for gating and diagnostics.
        double HorizontalStdM() const;
        double AltitudeStdM() const;

      private:
        template <std::size_t M>
        void Update(const Math::Matrix<M, 9>& h, const Math::Matrix<M, 1>& innovation, const Math::Matrix<M, M>& measurement_covariance);

        ErrorStateFilterConfiguration configuration_;
        Math::Matrix<9, 1> error_state_{};
        Math::Matrix<9, 9> covariance_{};
    };

}} // namespace TERCOM::Nav
