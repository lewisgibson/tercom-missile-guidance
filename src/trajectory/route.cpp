#include "tercom/trajectory/route.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace TERCOM { namespace Trajectory {

    namespace {
        // Distance in the horizontal (east-north) plane only, ignoring altitude.
        // Waypoint capture and along-track progress are judged on the ground
        // track, so the up axis (z) is deliberately left out.
        double HorizontalDistance(const Math::Vec3& a, const Math::Vec3& b) {
            const double de = a.x - b.x; // east difference, metres
            const double dn = a.y - b.y; // north difference, metres
            return std::sqrt((de * de) + (dn * dn));
        }
    } // namespace

    Route::Route(std::vector<Waypoint> waypoints) : waypoints_(std::move(waypoints)) {
        if (waypoints_.empty()) {
            throw std::invalid_argument("Route requires at least one waypoint");
        }
        // The first waypoint is the start of the route, so the first leg runs
        // from waypoint 0 to waypoint 1. Start by flying toward index 1 when one
        // exists; with a single waypoint there is no leg, so target index 0.
        active_index_ = (waypoints_.size() > 1) ? 1 : 0;
    }

    const Waypoint& Route::ActiveWaypoint() const {
        return waypoints_[active_index_];
    }

    Math::Vec3 Route::SegmentStartEnuM() const {
        if (active_index_ == 0) {
            return waypoints_.front().position_enu_m;
        }
        return waypoints_[active_index_ - 1].position_enu_m;
    }

    void Route::Update(const Math::Vec3& position_enu_m, double capture_radius_m) {
        if (complete_) {
            return;
        }

        const Math::Vec3 start = SegmentStartEnuM();
        const Math::Vec3 target = ActiveWaypoint().position_enu_m;

        // A waypoint counts as reached if the vehicle is within the capture
        // radius of it horizontally.
        bool reached = HorizontalDistance(position_enu_m, target) <= capture_radius_m;

        // Also treat the waypoint as reached once the vehicle has passed the end
        // of the leg in the along-track direction. This handles overshoots: if
        // the vehicle flies past the waypoint without entering the capture
        // circle (for example on a tight corner), the route still advances
        // rather than turning back. "Along-track" is the projection of the
        // vehicle's offset from the leg start onto the leg direction.
        const double along_east = target.x - start.x;  // leg vector east component, metres
        const double along_north = target.y - start.y; // leg vector north component, metres
        const double segment_length = std::sqrt((along_east * along_east) + (along_north * along_north));
        if (segment_length > 0.0) {
            // Dot the offset-from-start with the leg vector, divided by the leg
            // length, giving signed distance travelled along the leg in metres.
            // (Dividing by length normalises the leg vector to unit length.)
            const double progress = (((position_enu_m.x - start.x) * along_east) + ((position_enu_m.y - start.y) * along_north)) / segment_length;
            if (progress >= segment_length) {
                reached = true;
            }
        }

        if (!reached) {
            return;
        }

        // Advance to the next leg, or mark the route complete once the final
        // waypoint has been captured.
        if (active_index_ + 1 < waypoints_.size()) {
            ++active_index_;
        } else {
            complete_ = true;
        }
    }

}} // namespace TERCOM::Trajectory
