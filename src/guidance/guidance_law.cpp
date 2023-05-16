#include "tercom/guidance/guidance_law.h"

#include <algorithm>
#include <cmath>

#include "tercom/math/linear.h"

namespace TERCOM { namespace Guidance {

    namespace {
        // Standard gravity, in metres per second squared (the shared
        // Math::kStandardGravity), used to convert the lateral acceleration
        // command into a bank angle below.
        constexpr double kGravity = Math::kStandardGravity;

        // Heading (radians, clockwise from north) of an ENU horizontal vector.
        // ENU has x = east and y = north, so atan2(east, north) measures the
        // angle from north toward east, the usual compass-bearing convention.
        double HeadingOf(double east, double north) {
            return std::atan2(east, north);
        }
    } // namespace

    GuidanceLaw::GuidanceLaw(const GuidanceConfiguration& configuration) : configuration_(configuration) {
    }

    Trajectory::Correction GuidanceLaw::Compute(const Nav::NavState& state, Trajectory::Route& route) const {
        // Advance the route first so we steer toward the correct leg if the
        // last step captured a waypoint.
        route.Update(state.position_enu_m, configuration_.capture_radius_m);

        const Math::Vec3 position = state.position_enu_m;
        const Math::Vec3 velocity = state.velocity_enu_mps;
        // Ground speed is the horizontal speed only (the up component is
        // ignored), used both as the L1 speed term and for the speed loop.
        const double ground_speed = std::sqrt((velocity.x * velocity.x) + (velocity.y * velocity.y));

        const Math::Vec3 leg_start = route.SegmentStartEnuM();
        const Math::Vec3 leg_end = route.ActiveWaypoint().position_enu_m;

        // Leg direction and length in the horizontal plane.
        const double along_east = leg_end.x - leg_start.x;
        const double along_north = leg_end.y - leg_start.y;
        const double segment_length = std::sqrt((along_east * along_east) + (along_north * along_north));

        // Reference point L1 ahead along the leg. Degenerate (zero-length) legs
        // aim straight at the active waypoint.
        Math::Vec3 reference_point = leg_end;
        // 1e-6 m guards against dividing by a zero-length leg; below this the
        // leg has no usable direction so we fall back to aiming at the waypoint.
        if (segment_length > 1e-6) {
            // Unit vector along the leg (leg direction normalised to length 1).
            const double unit_east = along_east / segment_length;
            const double unit_north = along_north / segment_length;
            // Decompose the vehicle's offset from the leg start into:
            // along_track  - distance projected onto the leg direction, and
            // cross_track  - perpendicular (sideways) distance off the leg.
            const double along_track = ((position.x - leg_start.x) * unit_east) + ((position.y - leg_start.y) * unit_north);
            const double cross_east = (position.x - leg_start.x) - (along_track * unit_east);
            const double cross_north = (position.y - leg_start.y) - (along_track * unit_north);
            const double cross_track = std::sqrt((cross_east * cross_east) + (cross_north * cross_north));

            // The reference point is where a circle of radius L1 centred on the
            // vehicle meets the leg ahead of it. By Pythagoras the forward
            // offset along the leg from the vehicle's foot is
            // sqrt(L1^2 - cross_track^2); max(0, ...) avoids a negative under
            // the root when the vehicle is further off track than L1.
            const double lookahead = std::sqrt(std::max(0.0, (configuration_.l1_distance_m * configuration_.l1_distance_m) - (cross_track * cross_track)));
            // Clamp the reference to lie within the leg [start, end] so it never
            // runs off either end of the segment.
            const double reference_along = std::clamp(along_track + lookahead, 0.0, segment_length);
            reference_point = Math::Vec3{leg_start.x + (reference_along * unit_east), leg_start.y + (reference_along * unit_north), leg_end.z};
        }

        const double to_reference_east = reference_point.x - position.x;
        const double to_reference_north = reference_point.y - position.y;

        // Use the velocity heading when moving (above 1 m/s, where the heading
        // is reliable), otherwise fall back to the bearing to the reference so
        // the command is still defined when nearly stationary.
        const double current_heading = (ground_speed > 1.0) ? HeadingOf(velocity.x, velocity.y) : HeadingOf(to_reference_east, to_reference_north);
        const double desired_heading = HeadingOf(to_reference_east, to_reference_north);
        // Smallest signed turn from current to desired heading, wrapped into
        // (-pi, pi] so a small error near the +/-180 boundary stays small.
        const double heading_error = Math::WrapToPi(desired_heading - current_heading);

        // L1 lateral acceleration command, then bank angle for a coordinated turn.
        // effective_speed floors the speed at 1 m/s so the command stays finite
        // and bounded at very low speed.
        const double effective_speed = std::max(ground_speed, 1.0);
        // L1 law: lateral acceleration = 2 * V^2 / L1 * sin(heading_error).
        // This is the centripetal acceleration of the circular arc through the
        // reference point; it drives the vehicle to turn toward the path.
        const double lateral_acceleration = (2.0 * effective_speed * effective_speed / configuration_.l1_distance_m) * std::sin(heading_error);
        // Coordinated-turn relation: in a balanced (no-sideslip) turn, banking
        // by angle phi produces horizontal acceleration g * tan(phi). Inverting
        // gives phi = atan2(lateral_acceleration, g). Clamp to the bank limit.
        const double bank = std::clamp(std::atan2(lateral_acceleration, kGravity), -configuration_.max_bank_rad, configuration_.max_bank_rad);

        // Altitude hold: proportional control on the height error toward the
        // leg-end altitude, clamped to the pitch limit. Positive error (target
        // above the vehicle) commands nose-up pitch to climb.
        const double altitude_error = leg_end.z - position.z;
        const double pitch = std::clamp(configuration_.altitude_gain_per_m * altitude_error, -configuration_.max_pitch_rad, configuration_.max_pitch_rad);

        // Speed hold: throttle = trim plus proportional correction on the speed
        // error toward the waypoint's target speed, clamped to [0, 1].
        const double target_speed = route.ActiveWaypoint().speed_mps;
        const double throttle = std::clamp(configuration_.trim_throttle + (configuration_.speed_gain_per_mps * (target_speed - ground_speed)), 0.0, 1.0);

        Trajectory::Correction correction;
        correction.roll_command_rad = bank;
        correction.pitch_command_rad = pitch;
        correction.yaw_command_rad = heading_error;
        correction.throttle = throttle;
        return correction;
    }

}} // namespace TERCOM::Guidance
