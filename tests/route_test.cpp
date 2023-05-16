#include <gtest/gtest.h>

#include "tercom/math/linear.h"
#include "tercom/trajectory/route.h"
#include "tercom/trajectory/waypoint.h"

#include <stdexcept>
#include <vector>

namespace {

    using TERCOM::Math::Vec3;
    using TERCOM::Trajectory::Route;
    using TERCOM::Trajectory::Waypoint;

    Waypoint MakeWaypoint(double east, double north, double up, double speed = 0.0) {
        Waypoint wp;
        wp.position_enu_m = Vec3(east, north, up);
        wp.speed_mps = speed;
        return wp;
    }

} // namespace

TEST(TrajectoryRoute, EmptyWaypointsThrows) {
    const std::vector<Waypoint> empty;
    EXPECT_THROW(Route route(empty), std::invalid_argument);
}

TEST(TrajectoryRoute, SingleWaypointActiveIndexIsZero) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(10.0, 20.0, 5.0));

    const Route route(waypoints);

    EXPECT_EQ(route.ActiveIndex(), 0u);
    EXPECT_FALSE(route.IsComplete());
    EXPECT_EQ(route.Waypoints().size(), 1u);
}

TEST(TrajectoryRoute, MultipleWaypointsActiveIndexStartsAtOne) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(200.0, 0.0, 0.0));

    const Route route(waypoints);

    EXPECT_EQ(route.ActiveIndex(), 1u);
    EXPECT_FALSE(route.IsComplete());
    EXPECT_EQ(route.Waypoints().size(), 3u);
}

TEST(TrajectoryRoute, ActiveWaypointReturnsCorrectPoint) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 50.0, 10.0, 25.0));

    const Route route(waypoints);

    const Waypoint& active = route.ActiveWaypoint();
    EXPECT_NEAR(active.position_enu_m.x, 100.0, 1e-9);
    EXPECT_NEAR(active.position_enu_m.y, 50.0, 1e-9);
    EXPECT_NEAR(active.position_enu_m.z, 10.0, 1e-9);
    EXPECT_NEAR(active.speed_mps, 25.0, 1e-9);
}

TEST(TrajectoryRoute, ActiveWaypointSingleIsTheOnlyPoint) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(7.0, 8.0, 9.0));

    const Route route(waypoints);

    const Waypoint& active = route.ActiveWaypoint();
    EXPECT_NEAR(active.position_enu_m.x, 7.0, 1e-9);
    EXPECT_NEAR(active.position_enu_m.y, 8.0, 1e-9);
    EXPECT_NEAR(active.position_enu_m.z, 9.0, 1e-9);
}

TEST(TrajectoryRoute, SegmentStartIsPreviousWaypointForMultiLeg) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(200.0, 0.0, 0.0));

    const Route route(waypoints);

    // Active index is 1, so the segment start is waypoint 0.
    const Vec3 start = route.SegmentStartEnuM();
    EXPECT_NEAR(start.x, 0.0, 1e-9);
    EXPECT_NEAR(start.y, 0.0, 1e-9);
    EXPECT_NEAR(start.z, 0.0, 1e-9);
}

TEST(TrajectoryRoute, SegmentStartIsFirstWaypointWhenActiveIndexZero) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(3.0, 4.0, 5.0));

    const Route route(waypoints);

    // Single waypoint: active index 0, segment start is the front waypoint.
    const Vec3 start = route.SegmentStartEnuM();
    EXPECT_NEAR(start.x, 3.0, 1e-9);
    EXPECT_NEAR(start.y, 4.0, 1e-9);
    EXPECT_NEAR(start.z, 5.0, 1e-9);
}

TEST(TrajectoryRoute, UpdateAdvancesWhenWithinCaptureRadius) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(200.0, 0.0, 0.0));

    Route route(waypoints);
    EXPECT_EQ(route.ActiveIndex(), 1u);

    // Sit just short of waypoint 1, off to the side so along-track progress
    // does not pass the end of the leg, but within the capture radius.
    const Vec3 position(95.0, 3.0, 0.0);
    route.Update(position, 10.0);

    EXPECT_EQ(route.ActiveIndex(), 2u);
    EXPECT_FALSE(route.IsComplete());
}

TEST(TrajectoryRoute, UpdateDoesNotAdvanceWhenFarAndNotPassed) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(200.0, 0.0, 0.0));

    Route route(waypoints);
    EXPECT_EQ(route.ActiveIndex(), 1u);

    // Halfway down the first leg: far from the target and not past the end.
    const Vec3 position(50.0, 0.0, 0.0);
    route.Update(position, 10.0);

    EXPECT_EQ(route.ActiveIndex(), 1u);
    EXPECT_FALSE(route.IsComplete());
}

TEST(TrajectoryRoute, UpdateAdvancesWhenPassedEndOfLegOutsideCaptureRadius) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(200.0, 0.0, 0.0));

    Route route(waypoints);
    EXPECT_EQ(route.ActiveIndex(), 1u);

    // Past the end of the first leg in the along-track (east) direction but
    // far enough off-track that the horizontal distance exceeds the capture
    // radius.
    const Vec3 position(110.0, 80.0, 0.0);
    const double capture_radius = 5.0;
    EXPECT_GT(80.0, capture_radius);
    route.Update(position, capture_radius);

    EXPECT_EQ(route.ActiveIndex(), 2u);
    EXPECT_FALSE(route.IsComplete());
}

TEST(TrajectoryRoute, UpdateDoesNotAdvanceWhenJustBeforeEndOfLeg) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(200.0, 0.0, 0.0));

    Route route(waypoints);

    // Along-track progress of 99 m along a 100 m leg: not yet passed the end,
    // and 1 m of cross-track keeps it outside a tiny capture radius. The
    // horizontal distance to the target is sqrt(1 + 1) ~ 1.41 m.
    const Vec3 position(99.0, 1.0, 0.0);
    route.Update(position, 0.5);

    EXPECT_EQ(route.ActiveIndex(), 1u);
    EXPECT_FALSE(route.IsComplete());
}

TEST(TrajectoryRoute, CapturingLastWaypointCompletesRoute) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));

    Route route(waypoints);
    EXPECT_EQ(route.ActiveIndex(), 1u);
    EXPECT_FALSE(route.IsComplete());

    // Reach the final waypoint within the capture radius.
    route.Update(Vec3(99.0, 0.0, 0.0), 5.0);

    EXPECT_TRUE(route.IsComplete());
    // Index does not advance past the last waypoint.
    EXPECT_EQ(route.ActiveIndex(), 1u);
}

TEST(TrajectoryRoute, FurtherUpdatesAfterCompletionAreNoOps) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));

    Route route(waypoints);
    route.Update(Vec3(100.0, 0.0, 0.0), 5.0);
    ASSERT_TRUE(route.IsComplete());

    const std::size_t index_after_complete = route.ActiveIndex();

    // Any further Update calls must not change state.
    route.Update(Vec3(0.0, 0.0, 0.0), 5.0);
    route.Update(Vec3(500.0, 500.0, 0.0), 1000.0);

    EXPECT_TRUE(route.IsComplete());
    EXPECT_EQ(route.ActiveIndex(), index_after_complete);
}

TEST(TrajectoryRoute, MultiLegSequentialAdvanceThenComplete) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(200.0, 0.0, 0.0));

    Route route(waypoints);
    EXPECT_EQ(route.ActiveIndex(), 1u);

    // Capture the first intermediate waypoint.
    route.Update(Vec3(100.0, 0.0, 0.0), 5.0);
    EXPECT_EQ(route.ActiveIndex(), 2u);
    EXPECT_FALSE(route.IsComplete());

    // After advancing, the segment start should be the previous waypoint.
    const Vec3 start = route.SegmentStartEnuM();
    EXPECT_NEAR(start.x, 100.0, 1e-9);
    EXPECT_NEAR(start.y, 0.0, 1e-9);

    // Capture the final waypoint to complete the route.
    route.Update(Vec3(200.0, 0.0, 0.0), 5.0);
    EXPECT_TRUE(route.IsComplete());
    EXPECT_EQ(route.ActiveIndex(), 2u);
}

TEST(TrajectoryRoute, UpdateExactlyAtCaptureRadiusBoundaryAdvances) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(200.0, 0.0, 0.0));

    Route route(waypoints);

    // Exactly 10 m short of the target along a perpendicular offset so the
    // distance equals the capture radius (the comparison uses <=).
    const Vec3 position(100.0, 10.0, 0.0);
    route.Update(position, 10.0);

    EXPECT_EQ(route.ActiveIndex(), 2u);
}

TEST(TrajectoryRoute, UpdateAtSegmentEndExactlyAdvances) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(MakeWaypoint(0.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(100.0, 0.0, 0.0));
    waypoints.push_back(MakeWaypoint(200.0, 0.0, 0.0));

    Route route(waypoints);

    // Progress exactly equals the segment length (progress >= length is true),
    // while the cross-track offset keeps it outside a small capture radius.
    const Vec3 position(100.0, 20.0, 0.0);
    route.Update(position, 1.0);

    EXPECT_EQ(route.ActiveIndex(), 2u);
}
