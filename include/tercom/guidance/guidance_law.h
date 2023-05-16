#pragma once

// L1 nonlinear path-following guidance (Park, Deyst, How, 2004). From the
// estimated state and the active route leg it produces a lateral acceleration
// command, converted to a bank angle, plus pitch and throttle commands to hold
// the leg's altitude and speed. This is route following between waypoints, not
// terminal intercept.
//
// The L1 idea in one paragraph: pick a reference point on the path a fixed
// distance ("L1 distance") ahead of the vehicle, then command the lateral
// (sideways) acceleration needed to fly a circular arc through that point. The
// vehicle naturally converges onto the path and tracks it; a larger L1 distance
// gives smoother, slower convergence, a smaller one gives tighter, more
// aggressive tracking.

#include "tercom/nav/nav_state.h"
#include "tercom/trajectory/correction.h"
#include "tercom/trajectory/route.h"

namespace TERCOM { namespace Guidance {

    // Tuning parameters for the guidance law. The defaults are tuned for a
    // cruise-speed (tens to low hundreds of metres per second) winged vehicle;
    // they are empirical starting points, not derived from first principles, and
    // are expected to be re-tuned per airframe.
    struct GuidanceConfiguration {
        // L1 lookahead distance, in metres: how far ahead along the path the
        // reference point is placed. This is the dominant lateral-tracking knob.
        // Larger means gentler, slower convergence and less overshoot; smaller
        // means tighter, more aggressive tracking but more risk of oscillation.
        // 800 m suits the cruise speeds and turn rates used here.
        double l1_distance_m = 800.0;

        // Maximum commanded bank angle, in radians (0.61 rad is about 35
        // degrees). Caps how hard the vehicle is allowed to turn; a steeper bank
        // turns faster but bleeds energy and stresses the airframe.
        double max_bank_rad = 0.61;

        // Maximum commanded pitch angle, in radians (0.35 rad is about 20
        // degrees). Caps climb and descent angles so altitude corrections stay
        // within a safe, flyable range.
        double max_pitch_rad = 0.35;

        // Altitude-hold proportional gain: pitch command (radians) per metre of
        // altitude error. At 0.002 rad/m a 100 m error commands 0.2 rad of
        // pitch (before clamping). Higher closes altitude error faster but risks
        // overshoot and pitch oscillation.
        double altitude_gain_per_m = 0.002;

        // Speed-hold proportional gain: throttle (normalised 0..1) per metre per
        // second of speed error. At 0.02 per m/s a 10 m/s shortfall adds 0.2 to
        // the throttle (before clamping). Higher chases the target speed harder.
        double speed_gain_per_mps = 0.02;

        // Trim throttle: the normalised throttle (0..1) that holds the commanded
        // speed in level flight, i.e. the operating point the speed loop adjusts
        // around. 0.6 leaves headroom in both directions to speed up or slow
        // down.
        double trim_throttle = 0.6;

        // Waypoint capture radius, in metres: how close (horizontally) the
        // vehicle must get before the active waypoint counts as reached and the
        // route advances. 300 m gives a comfortable margin at the turn rates the
        // bank limit allows; too small can be missed at speed, too large cuts
        // corners short.
        double capture_radius_m = 300.0;
    };

    class GuidanceLaw {
      public:
        explicit GuidanceLaw(const GuidanceConfiguration& configuration = {});

        // Compute corrections to follow the route from the current estimated
        // state. Advances the route's active waypoint as it is captured.
        Trajectory::Correction Compute(const Nav::NavState& state, Trajectory::Route& route) const;

      private:
        GuidanceConfiguration configuration_;
    };

}} // namespace TERCOM::Guidance
