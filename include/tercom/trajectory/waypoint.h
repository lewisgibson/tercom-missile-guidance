#pragma once

#include "tercom/math/linear.h"

namespace TERCOM { namespace Trajectory {

    // A point on the planned route, in the local ENU frame, with the target
    // ground speed to hold on the leg leading into it. ENU means the axes are
    // x = east, y = north, z = up, all in metres relative to a fixed local
    // origin.
    struct Waypoint {
        Math::Vec3 position_enu_m; // waypoint location (east, north, up) in metres
        double speed_mps = 0.0;    // desired ground speed on the inbound leg, in metres per second
    };

}} // namespace TERCOM::Trajectory
