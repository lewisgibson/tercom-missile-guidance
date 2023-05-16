#pragma once
#include "../radar/position.h"

namespace TERCOM { namespace Trajectory {
    class Target : TERCOM::Radar::Position {
      public:
        Target(float x, float y, float z) : TERCOM::Radar::Position(x, y, z) {
            //
        }
    };
}} // namespace TERCOM::Trajectory
