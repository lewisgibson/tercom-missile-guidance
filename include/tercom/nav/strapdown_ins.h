#pragma once

// Strapdown inertial mechanisation in the local ENU frame. Each IMU sample
// advances attitude, velocity, and position. The same integrator drives the
// simulator's ground truth (fed an ideal IMU) and the onboard estimate (fed the
// noisy, biased IMU), so with a perfect IMU the estimate reproduces truth.

#include "tercom/nav/nav_state.h"
#include "tercom/sensors/measurements.h"

namespace TERCOM { namespace Nav {

    class StrapdownIns {
      public:
        explicit StrapdownIns(const NavState& initial_state);

        const NavState& State() const {
            return state_;
        }

        void SetState(const NavState& state) {
            state_ = state;
        }

        // Advance the mechanisation to the sample's timestamp. The first sample
        // after construction only sets the clock if it predates the state.
        void Step(const Sensors::ImuSample& sample);

        // Closed-loop feedback from the terrain filter: add the estimated INS
        // error corrections back into the state.
        void ApplyCorrection(const Math::Vec3& position_correction_m, const Math::Vec3& velocity_correction_mps, const Math::Vec3& accelerometer_bias_correction_mps2);

      private:
        NavState state_;
    };

}} // namespace TERCOM::Nav
