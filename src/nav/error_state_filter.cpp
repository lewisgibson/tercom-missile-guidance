#include "tercom/nav/error_state_filter.h"

#include <algorithm>
#include <cmath>

namespace TERCOM { namespace Nav {

    using Math::Matrix;

    ErrorStateFilter::ErrorStateFilter(const ErrorStateFilterConfiguration& configuration) : configuration_(configuration) {
        // Seed the initial covariance. Kalman covariance carries variances
        // (sigma^2) on the diagonal, so square the configured one-sigma values.
        // The error state itself starts at zero (best guess: the INS is correct).
        const double position_variance = configuration_.initial_position_std_m * configuration_.initial_position_std_m;
        const double velocity_variance = configuration_.initial_velocity_std_mps * configuration_.initial_velocity_std_mps;
        const double bias_variance = configuration_.initial_accelerometer_bias_std_mps2 * configuration_.initial_accelerometer_bias_std_mps2;

        // State layout: indices 0..2 position (e, n, u), 3..5 velocity, 6..8
        // accelerometer bias. Same variance on each of the three axes.
        for (std::size_t i = 0; i < 3; ++i) {
            covariance_(i, i) = position_variance;
            covariance_(3 + i, 3 + i) = velocity_variance;
            covariance_(6 + i, 6 + i) = bias_variance;
        }
    }

    void ErrorStateFilter::Predict(double dt_s, const Math::Mat3& body_to_nav) {
        if (dt_s <= 0.0) {
            return;
        }

        // Discrete state-transition matrix F, the linear map that takes the error
        // state from the start of the step to its end (error_next = F * error).
        // It comes from linearising the error dynamics:
        //   d(position) = velocity
        //   d(velocity) = -R_body_to_nav * accelerometer_bias_error
        //       (a bias on the accelerometer, once rotated into the nav frame,
        //        shows up as a velocity error of the opposite sign)
        //   d(accelerometer_bias) = 0 (a random walk: no deterministic drift, only noise)
        // Start from identity (state persists) and add the first-order coupling
        // terms; the dt^2/2 term is the second-order effect of bias feeding through
        // velocity into position over the step.
        Matrix<9, 9> transition = Matrix<9, 9>::Identity();
        for (std::size_t i = 0; i < 3; ++i) {
            transition(i, 3 + i) += dt_s; // position gains velocity * dt
        }
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) {
                const double rotation = body_to_nav.At(r, c);
                transition(3 + r, 6 + c) += -rotation * dt_s;          // velocity from bias
                transition(r, 6 + c) += -0.5 * rotation * dt_s * dt_s; // position from bias
            }
        }

        // Covariance propagation: P = F * P * F^T. This is the standard Kalman
        // prediction, transporting the uncertainty ellipsoid through the dynamics.
        covariance_ = transition * covariance_ * transition.Transposed();

        // Add process noise Q to the diagonal. Each is (spectral density)^2 * dt,
        // converting a per-root-second standard deviation into a variance accrued
        // over the step (variance of a random walk grows linearly with time).
        const double position_process = configuration_.position_process_std_mps * configuration_.position_process_std_mps * dt_s;
        const double velocity_process = configuration_.accelerometer_noise_density_mps2 * configuration_.accelerometer_noise_density_mps2 * dt_s;
        const double bias_process = configuration_.accelerometer_bias_random_walk_mps3 * configuration_.accelerometer_bias_random_walk_mps3 * dt_s;
        for (std::size_t i = 0; i < 3; ++i) {
            covariance_(i, i) += position_process;
            covariance_(3 + i, 3 + i) += velocity_process;
            covariance_(6 + i, 6 + i) += bias_process;
        }
    }

    // Generic Kalman measurement update for an M-dimensional measurement. h is the
    // observation matrix mapping the 9 states onto what the sensor sees;
    // measurement_covariance (R) is the sensor noise.
    template <std::size_t M>
    void ErrorStateFilter::Update(const Matrix<M, 9>& h, const Matrix<M, 1>& innovation, const Matrix<M, M>& measurement_covariance) {
        const Matrix<9, M> h_transposed = h.Transposed();
        // Innovation covariance S = H*P*H^T + R: the expected spread of the
        // measurement residual, combining state and sensor uncertainty.
        const Matrix<M, M> innovation_covariance = (h * covariance_ * h_transposed) + measurement_covariance;
        // Kalman gain K = P*H^T*S^-1: how strongly to trust the measurement. A
        // small S (confident measurement) drives the gain up.
        const Matrix<9, M> gain = covariance_ * h_transposed * Math::Inverse(innovation_covariance);

        // Residual of the measurement against what the current error state predicts
        // (in closed loop the error state is ~0, so this is essentially the raw
        // innovation), then nudge the state by gain * residual.
        const Matrix<M, 1> residual = innovation - (h * error_state_);
        error_state_ = error_state_ + (gain * residual);

        // Covariance update in Joseph (symmetric) form:
        //   P = (I - K*H) P (I - K*H)^T + K*R*K^T
        // The textbook short form P = (I - K*H) P is cheaper but, with finite
        // floating-point precision, can drift into a non-symmetric or negative
        // (impossible) covariance. The Joseph form is algebraically equal yet
        // stays symmetric and positive definite, which keeps the filter stable.
        const Matrix<9, 9> identity = Matrix<9, 9>::Identity();
        const Matrix<9, 9> factor = identity - (gain * h);
        covariance_ = (factor * covariance_ * factor.Transposed()) + (gain * measurement_covariance * gain.Transposed());
    }

    void ErrorStateFilter::UpdateAltitude(double innovation_up_m, double variance_m2) {
        // Scalar measurement of the Up position error. The observation matrix picks
        // out state index 2 (Up position) and nothing else.
        Matrix<1, 9> h{};
        h(0, 2) = 1.0;
        Matrix<1, 1> innovation{};
        innovation(0, 0) = innovation_up_m;
        Matrix<1, 1> measurement_covariance{};
        measurement_covariance(0, 0) = variance_m2;
        Update<1>(h, innovation, measurement_covariance);
    }

    void ErrorStateFilter::UpdateHorizontalPosition(double innovation_east_m, double innovation_north_m, const Matrix<2, 2>& covariance) {
        // Two-element measurement of the East/North position error. The observation
        // matrix picks out state indices 0 (East) and 1 (North).
        Matrix<2, 9> h{};
        h(0, 0) = 1.0;
        h(1, 1) = 1.0;
        Matrix<2, 1> innovation{};
        innovation(0, 0) = innovation_east_m;
        innovation(1, 0) = innovation_north_m;
        Update<2>(h, innovation, covariance);
    }

    bool ErrorStateFilter::UpdateHorizontalPositionGated(double innovation_east_m, double innovation_north_m, const Matrix<2, 2>& covariance, double gate_chi2) {
        // Innovation covariance S for the horizontal position measurement. Because
        // the observation matrix simply selects East/North, H*P*H^T is just the
        // top-left 2x2 block of the state covariance, so S = P[0:2,0:2] + R.
        Matrix<2, 2> innovation_covariance;
        innovation_covariance(0, 0) = covariance_(0, 0) + covariance(0, 0);
        innovation_covariance(0, 1) = covariance_(0, 1) + covariance(0, 1);
        innovation_covariance(1, 0) = covariance_(1, 0) + covariance(1, 0);
        innovation_covariance(1, 1) = covariance_(1, 1) + covariance(1, 1);
        const Matrix<2, 2> inverse = Math::Inverse(innovation_covariance);

        // Residual relative to the predicted error (near zero in closed loop).
        const double residual_east = innovation_east_m - error_state_(0, 0);
        const double residual_north = innovation_north_m - error_state_(1, 0);
        // Normalised innovation squared (NIS), residual^T * S^-1 * residual. This
        // measures the residual in units of its own expected spread: how many
        // sigmas off the fix is, accounting for the correlation between axes. If
        // the filter and the fix are consistent it follows a chi-square
        // distribution with two degrees of freedom (East and North).
        const double normalised_innovation_squared = (residual_east * ((inverse(0, 0) * residual_east) + (inverse(0, 1) * residual_north)))
                                                     + (residual_north * ((inverse(1, 0) * residual_east) + (inverse(1, 1) * residual_north)));

        // Soft gate (chi-square covariance scaling). A hard reject starves the
        // filter: once its covariance has collapsed after a run of good fixes, any
        // larger innovation is refused outright and the loop can never recover. So
        // instead of rejecting an outlier, inflate its measurement covariance in
        // proportion to how far the innovation exceeds the gate. A consistent fix
        // (NIS <= gate) is applied at face value; a suspect one is smoothly
        // down-weighted, and a wildly inconsistent one is inflated so heavily that
        // it barely moves the state -- robust to false terrain matches without ever
        // locking out a genuine correction.
        //
        // gate_chi2 is the chi-square threshold (caller passes 13.8, the 99.9th
        // percentile for two degrees of freedom): a consistent fix lands below it
        // 99.9% of the time. Inflation = max(1, NIS / gate): at or under the gate
        // the measurement covariance is unchanged; above it, R is scaled up by the
        // overshoot factor so the gain (and hence the state change) shrinks
        // proportionally.
        const double inflation = std::max(1.0, normalised_innovation_squared / gate_chi2);
        Matrix<2, 2> inflated = covariance;
        inflated(0, 0) *= inflation;
        inflated(0, 1) *= inflation;
        inflated(1, 0) *= inflation;
        inflated(1, 1) *= inflation;

        UpdateHorizontalPosition(innovation_east_m, innovation_north_m, inflated);
        // Report whether the fix was consistent (within the gate). The fix is
        // always applied; the return value only flags down-weighting for diagnostics.
        return normalised_innovation_squared <= gate_chi2;
    }

    // The estimated error is what the INS reads minus the truth, so the correction
    // that removes it is the negated error state. The INS adds these in, then the
    // error state is reset to zero (closed loop).
    Math::Vec3 ErrorStateFilter::PositionCorrection() const {
        return {-error_state_(0, 0), -error_state_(1, 0), -error_state_(2, 0)};
    }

    Math::Vec3 ErrorStateFilter::VelocityCorrection() const {
        return {-error_state_(3, 0), -error_state_(4, 0), -error_state_(5, 0)};
    }

    Math::Vec3 ErrorStateFilter::AccelerometerBiasCorrection() const {
        return {-error_state_(6, 0), -error_state_(7, 0), -error_state_(8, 0)};
    }

    // Closed-loop reset: once the corrections above have been folded into the INS,
    // the estimated error is zero by definition. Only the mean is zeroed; the
    // covariance (uncertainty) is kept, since correcting the state does not make us
    // certain of it.
    void ErrorStateFilter::ResetErrorState() {
        error_state_ = Matrix<9, 1>{};
    }

    // One-sigma horizontal position uncertainty, the RMS of the East and North
    // standard deviations (sqrt of the averaged variances). Used to size the
    // correlator search window and as a health diagnostic.
    double ErrorStateFilter::HorizontalStdM() const {
        return std::sqrt((covariance_(0, 0) + covariance_(1, 1)) / 2.0);
    }

    // One-sigma vertical (Up) position uncertainty.
    double ErrorStateFilter::AltitudeStdM() const {
        return std::sqrt(covariance_(2, 2));
    }

}} // namespace TERCOM::Nav
