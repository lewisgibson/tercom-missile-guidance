#pragma once
#include <tuple>

namespace TERCOM { namespace Radar {
    class Position {
      private:
        std::tuple<float, float, float> position_;

      public:
        explicit Position(float x, float y, float z) : position_(std::tuple<float, float, float>(x, y, z)) {
            //
        }
    };
}} // namespace TERCOM::Radar
