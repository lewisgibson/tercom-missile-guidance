#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "tercom/guidance/guidance_law.h"
#include "tercom/math/linear.h"
#include "tercom/nav/nav_state.h"
#include "tercom/trajectory/correction.h"
#include "tercom/trajectory/route.h"
#include "tercom/trajectory/waypoint.h"

namespace {

    using TERCOM::Guidance::GuidanceConfiguration;
    using TERCOM::Guidance::GuidanceLaw;
    using TERCOM::Math::Vec3;
    using TERCOM::Nav::NavState;
    using TERCOM::Trajectory::Correction;
    using TERCOM::Trajectory::Route;
    using TERCOM::Trajectory::Waypoint;

    // A straight leg from the origin running 10 km along +east. The active
    // waypoint sits at the far end with a nominal target speed.
    Route MakeEastboundRoute(double target_speed_mps = 200.0, double leg_altitude_m = 1000.0) {
        std::vector<Waypoint> waypoints;
        Waypoint start;
        start.position_enu_m = Vec3{0.0, 0.0, leg_altitude_m};
        start.speed_mps = target_speed_mps;
        Waypoint finish;
        finish.position_enu_m = Vec3{10000.0, 0.0, leg_altitude_m};
        finish.speed_mps = target_speed_mps;
        waypoints.push_back(start);
        waypoints.push_back(finish);
        return Route{std::move(waypoints)};
    }

    // A vehicle moving east at the given speed and altitude.
    NavState MakeEastboundState(const Vec3& position, double speed_mps = 200.0) {
        NavState state;
        state.position_enu_m = position;
        state.velocity_enu_mps = Vec3{speed_mps, 0.0, 0.0};
        return state;
    }

    // On the path and tracking it: bank command is essentially zero.
    TEST(GuidanceLaw, OnPathCommandsZeroBank) {
        GuidanceLaw law{GuidanceConfiguration{}};
        Route route = MakeEastboundRoute();
        NavState state = MakeEastboundState(Vec3{2000.0, 0.0, 1000.0});

        const Correction correction = law.Compute(state, route);

        EXPECT_NEAR(correction.roll_command_rad, 0.0, 1e-3);
        EXPECT_NEAR(correction.yaw_command_rad, 0.0, 1e-3);
    }

    // Offset to the north of an eastbound leg: the bank must turn the vehicle
    // back toward the path. With the library's atan2(east, north) heading
    // convention the correcting bank for a north offset is positive.
    TEST(GuidanceLaw, NorthOffsetCommandsCorrectingBank) {
        GuidanceLaw law{GuidanceConfiguration{}};
        Route route = MakeEastboundRoute();
        NavState state = MakeEastboundState(Vec3{2000.0, 150.0, 1000.0});

        const Correction correction = law.Compute(state, route);

        EXPECT_GT(correction.roll_command_rad, 0.0);
        EXPECT_GT(correction.yaw_command_rad, 0.0);
    }

    // Offset to the south gives the opposite (also correcting) sign.
    TEST(GuidanceLaw, SouthOffsetCommandsOppositeBank) {
        GuidanceLaw law{GuidanceConfiguration{}};
        Route route = MakeEastboundRoute();
        NavState state = MakeEastboundState(Vec3{2000.0, -150.0, 1000.0});

        const Correction correction = law.Compute(state, route);

        EXPECT_LT(correction.roll_command_rad, 0.0);
        EXPECT_LT(correction.yaw_command_rad, 0.0);
    }

    // Symmetric offsets give opposite-signed banks of equal magnitude.
    TEST(GuidanceLaw, SymmetricOffsetsGiveOppositeSigns) {
        GuidanceLaw law{GuidanceConfiguration{}};
        Route north_route = MakeEastboundRoute();
        Route south_route = MakeEastboundRoute();
        NavState north = MakeEastboundState(Vec3{2000.0, 120.0, 1000.0});
        NavState south = MakeEastboundState(Vec3{2000.0, -120.0, 1000.0});

        const Correction north_correction = law.Compute(north, north_route);
        const Correction south_correction = law.Compute(south, south_route);

        EXPECT_GT(north_correction.roll_command_rad, 0.0);
        EXPECT_LT(south_correction.roll_command_rad, 0.0);
        EXPECT_NEAR(north_correction.roll_command_rad, -south_correction.roll_command_rad, 1e-9);
    }

    // A larger cross-track error produces a larger correcting bank (monotonic
    // response below saturation).
    TEST(GuidanceLaw, LargerOffsetCommandsLargerBank) {
        GuidanceLaw law{GuidanceConfiguration{}};
        Route small_route = MakeEastboundRoute();
        Route large_route = MakeEastboundRoute();
        NavState small = MakeEastboundState(Vec3{2000.0, 50.0, 1000.0});
        NavState large = MakeEastboundState(Vec3{2000.0, 200.0, 1000.0});

        const Correction small_correction = law.Compute(small, small_route);
        const Correction large_correction = law.Compute(large, large_route);

        EXPECT_GT(small_correction.roll_command_rad, 0.0);
        EXPECT_GT(large_correction.roll_command_rad, small_correction.roll_command_rad);
    }

    // A heading badly misaligned with the path saturates the bank to the limit.
    TEST(GuidanceLaw, LargeHeadingErrorClampsBankToMax) {
        GuidanceConfiguration configuration;
        configuration.max_bank_rad = 0.5;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute();

        // Flying north while the leg runs east: a 90 degree heading error.
        NavState state;
        state.position_enu_m = Vec3{2000.0, 0.0, 1000.0};
        state.velocity_enu_mps = Vec3{0.0, 200.0, 0.0};

        const Correction correction = law.Compute(state, route);

        EXPECT_NEAR(std::fabs(correction.roll_command_rad), configuration.max_bank_rad, 1e-9);
        EXPECT_LE(std::fabs(correction.roll_command_rad), configuration.max_bank_rad + 1e-12);
    }

    // Below the target speed throttle climbs above trim.
    TEST(GuidanceLaw, BelowTargetSpeedRaisesThrottle) {
        GuidanceConfiguration configuration;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute(200.0);
        // 15 m/s deficit keeps the throttle within [0, 1] (no clamp).
        NavState state = MakeEastboundState(Vec3{2000.0, 0.0, 1000.0}, 185.0);

        const Correction correction = law.Compute(state, route);

        EXPECT_GT(correction.throttle, configuration.trim_throttle);
        EXPECT_NEAR(correction.throttle, configuration.trim_throttle + (configuration.speed_gain_per_mps * (200.0 - 185.0)), 1e-9);
    }

    // Above the target speed throttle drops below trim.
    TEST(GuidanceLaw, AboveTargetSpeedLowersThrottle) {
        GuidanceConfiguration configuration;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute(200.0);
        // 15 m/s excess keeps the throttle within [0, 1] (no clamp).
        NavState state = MakeEastboundState(Vec3{2000.0, 0.0, 1000.0}, 215.0);

        const Correction correction = law.Compute(state, route);

        EXPECT_LT(correction.throttle, configuration.trim_throttle);
        EXPECT_NEAR(correction.throttle, configuration.trim_throttle + (configuration.speed_gain_per_mps * (200.0 - 215.0)), 1e-9);
    }

    // At the target speed throttle sits at trim.
    TEST(GuidanceLaw, AtTargetSpeedHoldsTrimThrottle) {
        GuidanceConfiguration configuration;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute(200.0);
        NavState state = MakeEastboundState(Vec3{2000.0, 0.0, 1000.0}, 200.0);

        const Correction correction = law.Compute(state, route);

        EXPECT_NEAR(correction.throttle, configuration.trim_throttle, 1e-9);
    }

    // Throttle is clamped into [0, 1] for an extreme speed deficit.
    TEST(GuidanceLaw, ThrottleClampedToUpperBound) {
        GuidanceConfiguration configuration;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute(200.0);
        // Nearly stopped against a high target speed drives throttle to its cap.
        NavState state = MakeEastboundState(Vec3{2000.0, 0.0, 1000.0}, 0.0);

        const Correction correction = law.Compute(state, route);

        EXPECT_LE(correction.throttle, 1.0);
        EXPECT_GE(correction.throttle, 0.0);
        EXPECT_NEAR(correction.throttle, 1.0, 1e-9);
    }

    // Throttle is clamped at zero for a large speed excess.
    TEST(GuidanceLaw, ThrottleClampedToLowerBound) {
        GuidanceConfiguration configuration;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute(50.0);
        NavState state = MakeEastboundState(Vec3{2000.0, 0.0, 1000.0}, 2000.0);

        const Correction correction = law.Compute(state, route);

        EXPECT_GE(correction.throttle, 0.0);
        EXPECT_NEAR(correction.throttle, 0.0, 1e-9);
    }

    // Below the leg altitude the pitch command climbs (positive).
    TEST(GuidanceLaw, BelowLegAltitudeCommandsClimb) {
        GuidanceConfiguration configuration;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute(200.0, 1000.0);
        // 100 m below keeps the pitch within +-max_pitch_rad (no clamp).
        NavState state = MakeEastboundState(Vec3{2000.0, 0.0, 900.0});

        const Correction correction = law.Compute(state, route);

        EXPECT_GT(correction.pitch_command_rad, 0.0);
        EXPECT_NEAR(correction.pitch_command_rad, configuration.altitude_gain_per_m * (1000.0 - 900.0), 1e-9);
    }

    // Above the leg altitude the pitch command descends (negative).
    TEST(GuidanceLaw, AboveLegAltitudeCommandsDescent) {
        GuidanceConfiguration configuration;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute(200.0, 1000.0);
        // 100 m above keeps the pitch within +-max_pitch_rad (no clamp).
        NavState state = MakeEastboundState(Vec3{2000.0, 0.0, 1100.0});

        const Correction correction = law.Compute(state, route);

        EXPECT_LT(correction.pitch_command_rad, 0.0);
        EXPECT_NEAR(correction.pitch_command_rad, configuration.altitude_gain_per_m * (1000.0 - 1100.0), 1e-9);
    }

    // At the leg altitude the pitch command is zero.
    TEST(GuidanceLaw, AtLegAltitudeCommandsZeroPitch) {
        GuidanceConfiguration configuration;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute(200.0, 1000.0);
        NavState state = MakeEastboundState(Vec3{2000.0, 0.0, 1000.0});

        const Correction correction = law.Compute(state, route);

        EXPECT_NEAR(correction.pitch_command_rad, 0.0, 1e-12);
    }

    // A large altitude error saturates the pitch command to the configured cap.
    TEST(GuidanceLaw, LargeAltitudeErrorClampsPitch) {
        GuidanceConfiguration configuration;
        configuration.max_pitch_rad = 0.3;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute(200.0, 5000.0);
        // Far below the leg: altitude_gain_per_m * error would exceed max_pitch.
        NavState state = MakeEastboundState(Vec3{2000.0, 0.0, 0.0});

        const Correction climb = law.Compute(state, route);
        EXPECT_NEAR(climb.pitch_command_rad, configuration.max_pitch_rad, 1e-9);

        Route descent_route = MakeEastboundRoute(200.0, 0.0);
        NavState high = MakeEastboundState(Vec3{2000.0, 0.0, 5000.0});
        const Correction descent = law.Compute(high, descent_route);
        EXPECT_NEAR(descent.pitch_command_rad, -configuration.max_pitch_rad, 1e-9);
    }

    // Compute advances the route: a vehicle sitting on the active waypoint
    // captures it and the route moves to the next leg (or completes).
    TEST(GuidanceLaw, ComputeAdvancesRouteOnCapture) {
        std::vector<Waypoint> waypoints;
        Waypoint a;
        a.position_enu_m = Vec3{0.0, 0.0, 1000.0};
        a.speed_mps = 200.0;
        Waypoint b;
        b.position_enu_m = Vec3{5000.0, 0.0, 1000.0};
        b.speed_mps = 200.0;
        Waypoint c;
        c.position_enu_m = Vec3{10000.0, 0.0, 1000.0};
        c.speed_mps = 200.0;
        waypoints.push_back(a);
        waypoints.push_back(b);
        waypoints.push_back(c);
        Route route{std::move(waypoints)};

        // Active leg starts targeting the second waypoint (index 1).
        EXPECT_EQ(route.ActiveIndex(), 1u);

        GuidanceLaw law{GuidanceConfiguration{}};
        // Place the vehicle right on waypoint b: it is captured and the route
        // advances to the final waypoint.
        NavState at_b = MakeEastboundState(Vec3{5000.0, 0.0, 1000.0});
        law.Compute(at_b, route);

        EXPECT_EQ(route.ActiveIndex(), 2u);
        EXPECT_FALSE(route.IsComplete());

        // Now reach the final waypoint: the route completes.
        NavState at_c = MakeEastboundState(Vec3{10000.0, 0.0, 1000.0});
        law.Compute(at_c, route);

        EXPECT_TRUE(route.IsComplete());
        EXPECT_EQ(route.ActiveIndex(), 2u);
    }

    // A vehicle far short of the waypoint does not advance the route.
    TEST(GuidanceLaw, ComputeDoesNotAdvanceWhenFarFromWaypoint) {
        GuidanceLaw law{GuidanceConfiguration{}};
        Route route = MakeEastboundRoute();
        EXPECT_EQ(route.ActiveIndex(), 1u);

        NavState state = MakeEastboundState(Vec3{1000.0, 0.0, 1000.0});
        law.Compute(state, route);

        EXPECT_EQ(route.ActiveIndex(), 1u);
        EXPECT_FALSE(route.IsComplete());
    }

    // A slow/stationary vehicle uses the bearing to the reference for its
    // heading, so an off-path stationary vehicle still produces a finite,
    // bounded bank command.
    TEST(GuidanceLaw, StationaryVehicleProducesBoundedBank) {
        GuidanceConfiguration configuration;
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute();
        NavState state;
        state.position_enu_m = Vec3{2000.0, 150.0, 1000.0};
        state.velocity_enu_mps = Vec3{0.0, 0.0, 0.0};

        const Correction correction = law.Compute(state, route);

        EXPECT_TRUE(std::isfinite(correction.roll_command_rad));
        EXPECT_LE(std::fabs(correction.roll_command_rad), configuration.max_bank_rad + 1e-12);
    }

    // A leg whose start and end coincide has zero length; the guidance law must take
    // its degenerate-leg fallback (aim straight at the active waypoint) and still
    // return finite, bounded commands rather than dividing by the zero length.
    TEST(GuidanceLaw, ZeroLengthLegProducesFiniteCommands) {
        GuidanceLaw law{GuidanceConfiguration{}};
        Waypoint point;
        point.position_enu_m = Vec3{1000.0, 1000.0, 1000.0};
        point.speed_mps = 200.0;
        std::vector<Waypoint> waypoints;
        waypoints.push_back(point);
        waypoints.push_back(point); // identical position: zero-length active leg
        Route route{std::move(waypoints)};

        // Far enough away that the waypoint is not immediately captured.
        const NavState state = MakeEastboundState(Vec3{500.0, 500.0, 1000.0});
        const Correction correction = law.Compute(state, route);

        EXPECT_TRUE(std::isfinite(correction.roll_command_rad));
        EXPECT_TRUE(std::isfinite(correction.pitch_command_rad));
        EXPECT_TRUE(std::isfinite(correction.yaw_command_rad));
        EXPECT_TRUE(std::isfinite(correction.throttle));
        EXPECT_LE(std::fabs(correction.roll_command_rad), GuidanceConfiguration{}.max_bank_rad + 1e-12);
    }

    // A vehicle further off-track than the L1 radius (here 2000 m north of the leg,
    // well beyond the 800 m default L1) exercises the degenerate-lookahead path: the
    // term sqrt(max(0, L1^2 - cross_track^2)) clamps to zero, so the reference point
    // collapses onto the foot of the perpendicular. The command must stay finite,
    // stay within the bank limit, and still steer back toward the path.
    TEST(GuidanceLaw, OffsetBeyondL1CollapsesLookaheadButStaysBounded) {
        GuidanceConfiguration configuration; // l1_distance_m defaults to 800 m
        GuidanceLaw law{configuration};
        Route route = MakeEastboundRoute();
        NavState state = MakeEastboundState(Vec3{2000.0, 2000.0, 1000.0});

        const Correction correction = law.Compute(state, route);

        EXPECT_TRUE(std::isfinite(correction.roll_command_rad));
        EXPECT_TRUE(std::isfinite(correction.yaw_command_rad));
        EXPECT_LE(std::fabs(correction.roll_command_rad), configuration.max_bank_rad + 1e-12);
        EXPECT_GT(correction.roll_command_rad, 0.0); // north offset -> positive correcting bank
    }

} // namespace
