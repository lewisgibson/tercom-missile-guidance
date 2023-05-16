#pragma once

namespace TERCOM { namespace Trajectory {
    class Correction {
      private:
        const float pitch_;
        const float yaw_;
        const float roll_;
        const float power_;

      public:
        Correction(const float pitch, const float yaw, const float roll, const float power) : pitch_(pitch), yaw_(yaw), roll_(roll), power_(power) {
            //
        }

        const float Pitch();
        const float Yaw();
        const float Roll();
        const float Power();
    };
}} // namespace TERCOM::Trajectory
