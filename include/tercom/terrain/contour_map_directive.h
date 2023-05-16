#pragma once
#include <tuple>

namespace TERCOM { namespace Terrain {
    class ContourMapDirective {
      private:
        // Use float64_t when C++23 is compliant (GCC13)
        std::tuple<float, float, float> position_;
        float altitude_;

      public:
        explicit ContourMapDirective(float x, float y, float z, float altitude) : position_(std::tuple<float, float, float>(x, y, z)), altitude_(altitude) {
            //
        }
    };
}} // namespace TERCOM::Terrain
