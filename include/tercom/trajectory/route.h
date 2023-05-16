#pragma once

// An ordered list of waypoints with an active leg. A "leg" is the straight
// segment between two consecutive waypoints; the "active leg" is the one the
// vehicle is currently flying. The guidance law follows the segment from the
// previous waypoint to the active one, and the route advances the active
// waypoint as each is captured (reached) so the next leg becomes active.

#include <cstddef>
#include <vector>

#include "tercom/math/linear.h"
#include "tercom/trajectory/waypoint.h"

namespace TERCOM { namespace Trajectory {

    class Route {
      public:
        explicit Route(std::vector<Waypoint> waypoints);

        const std::vector<Waypoint>& Waypoints() const {
            return waypoints_;
        }

        // Index of the active waypoint (the end of the active leg) within
        // Waypoints().
        std::size_t ActiveIndex() const {
            return active_index_;
        }

        // The waypoint the vehicle is currently flying toward (end of the
        // active leg).
        const Waypoint& ActiveWaypoint() const;

        // The start of the active leg (the previous waypoint, or the first
        // waypoint when the active one is the first).
        Math::Vec3 SegmentStartEnuM() const;

        // True once the final waypoint has been captured; the route then stops
        // advancing.
        bool IsComplete() const {
            return complete_;
        }

        // Advance the active waypoint when the current one is captured or
        // passed. capture_radius_m is the horizontal distance, in metres,
        // within which the active waypoint counts as reached. Call once per
        // guidance step with the latest estimated position.
        void Update(const Math::Vec3& position_enu_m, double capture_radius_m);

      private:
        std::vector<Waypoint> waypoints_; // the planned route, in order
        std::size_t active_index_ = 0;    // index of the active (current target) waypoint
        bool complete_ = false;           // set when the last waypoint has been captured
    };

}} // namespace TERCOM::Trajectory
