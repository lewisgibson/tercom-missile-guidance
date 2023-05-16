#include <gtest/gtest.h>

#include "tercom/geo/geo_point.h"
#include "tercom/geo/geodesy.h"
#include "tercom/math/linear.h"

#include <cmath>

namespace {

    using TERCOM::Geo::EcefToGeodetic;
    using TERCOM::Geo::GeodeticToEcef;
    using TERCOM::Geo::GeoPoint;
    using TERCOM::Geo::HaversineDistanceM;
    using TERCOM::Geo::InitialBearingRad;
    using TERCOM::Geo::LocalFrame;
    using TERCOM::Math::kPi;
    using TERCOM::Math::Norm;
    using TERCOM::Math::Vec3;
    namespace Wgs84 = TERCOM::Geo::Wgs84;

} // namespace

TEST(Geodesy, GeodeticToEcefOriginIsSemiMajorAxisOnX) {
    const GeoPoint origin{0.0, 0.0, 0.0};
    const Vec3 ecef = GeodeticToEcef(origin);
    EXPECT_NEAR(ecef.x, Wgs84::kSemiMajorAxisM, 1e-3);
    EXPECT_NEAR(ecef.y, 0.0, 1e-3);
    EXPECT_NEAR(ecef.z, 0.0, 1e-3);
}

TEST(Geodesy, GeodeticToEcefAddsAltitudeAtEquator) {
    const GeoPoint origin{0.0, 0.0, 1000.0};
    const Vec3 ecef = GeodeticToEcef(origin);
    EXPECT_NEAR(ecef.x, Wgs84::kSemiMajorAxisM + 1000.0, 1e-3);
    EXPECT_NEAR(ecef.y, 0.0, 1e-3);
    EXPECT_NEAR(ecef.z, 0.0, 1e-3);
}

TEST(Geodesy, GeodeticToEcefNorthPoleOnPolarAxis) {
    const GeoPoint north_pole{90.0, 0.0, 0.0};
    const Vec3 ecef = GeodeticToEcef(north_pole);
    EXPECT_NEAR(ecef.x, 0.0, 1e-3);
    EXPECT_NEAR(ecef.y, 0.0, 1e-3);
    EXPECT_NEAR(ecef.z, Wgs84::kSemiMinorAxisM, 1e-3);
}

TEST(Geodesy, GeodeticToEcefLongitudeNinetyOnY) {
    const GeoPoint point{0.0, 90.0, 0.0};
    const Vec3 ecef = GeodeticToEcef(point);
    EXPECT_NEAR(ecef.x, 0.0, 1e-3);
    EXPECT_NEAR(ecef.y, Wgs84::kSemiMajorAxisM, 1e-3);
    EXPECT_NEAR(ecef.z, 0.0, 1e-3);
}

TEST(Geodesy, EcefToGeodeticOriginRoundTrip) {
    const GeoPoint origin{0.0, 0.0, 0.0};
    const GeoPoint back = EcefToGeodetic(GeodeticToEcef(origin));
    EXPECT_NEAR(back.latitude_deg, 0.0, 1e-6);
    EXPECT_NEAR(back.longitude_deg, 0.0, 1e-6);
    EXPECT_NEAR(back.altitude_m, 0.0, 1e-3);
}

TEST(Geodesy, GeodeticEcefRoundTripLondon) {
    const GeoPoint london{51.5, -0.13, 100.0};
    const GeoPoint back = EcefToGeodetic(GeodeticToEcef(london));
    EXPECT_NEAR(back.latitude_deg, london.latitude_deg, 1e-6);
    EXPECT_NEAR(back.longitude_deg, london.longitude_deg, 1e-6);
    EXPECT_NEAR(back.altitude_m, london.altitude_m, 1e-3);
}

TEST(Geodesy, GeodeticEcefRoundTripEquator) {
    const GeoPoint equator{0.0, 45.0, 0.0};
    const GeoPoint back = EcefToGeodetic(GeodeticToEcef(equator));
    EXPECT_NEAR(back.latitude_deg, equator.latitude_deg, 1e-6);
    EXPECT_NEAR(back.longitude_deg, equator.longitude_deg, 1e-6);
    EXPECT_NEAR(back.altitude_m, equator.altitude_m, 1e-3);
}

TEST(Geodesy, GeodeticEcefRoundTripSouthernWestern) {
    const GeoPoint point{-33.87, -70.65, 520.0};
    const GeoPoint back = EcefToGeodetic(GeodeticToEcef(point));
    EXPECT_NEAR(back.latitude_deg, point.latitude_deg, 1e-6);
    EXPECT_NEAR(back.longitude_deg, point.longitude_deg, 1e-6);
    EXPECT_NEAR(back.altitude_m, point.altitude_m, 1e-3);
}

TEST(Geodesy, GeodeticEcefRoundTripHighLatitude) {
    const GeoPoint point{78.22, 15.65, 2000.0};
    const GeoPoint back = EcefToGeodetic(GeodeticToEcef(point));
    EXPECT_NEAR(back.latitude_deg, point.latitude_deg, 1e-6);
    EXPECT_NEAR(back.longitude_deg, point.longitude_deg, 1e-6);
    EXPECT_NEAR(back.altitude_m, point.altitude_m, 1e-3);
}

TEST(Geodesy, EcefToGeodeticNorthPoleDegenerate) {
    const Vec3 polar{0.0, 0.0, Wgs84::kSemiMinorAxisM + 500.0};
    const GeoPoint result = EcefToGeodetic(polar);
    EXPECT_NEAR(result.latitude_deg, 90.0, 1e-9);
    EXPECT_NEAR(result.altitude_m, 500.0, 1e-3);
}

TEST(Geodesy, EcefToGeodeticSouthPoleDegenerate) {
    const Vec3 polar{0.0, 0.0, -(Wgs84::kSemiMinorAxisM + 250.0)};
    const GeoPoint result = EcefToGeodetic(polar);
    EXPECT_NEAR(result.latitude_deg, -90.0, 1e-9);
    EXPECT_NEAR(result.altitude_m, 250.0, 1e-3);
}

TEST(LocalFrame, OriginMapsToZeroVector) {
    const GeoPoint origin{51.5, -0.13, 100.0};
    const LocalFrame frame(origin);
    const Vec3 local = frame.ToLocal(origin);
    EXPECT_NEAR(local.x, 0.0, 1e-6);
    EXPECT_NEAR(local.y, 0.0, 1e-6);
    EXPECT_NEAR(local.z, 0.0, 1e-6);
}

TEST(LocalFrame, OriginAccessorReturnsConstructionPoint) {
    const GeoPoint origin{12.34, 56.78, 90.0};
    const LocalFrame frame(origin);
    EXPECT_DOUBLE_EQ(frame.Origin().latitude_deg, origin.latitude_deg);
    EXPECT_DOUBLE_EQ(frame.Origin().longitude_deg, origin.longitude_deg);
    EXPECT_DOUBLE_EQ(frame.Origin().altitude_m, origin.altitude_m);
}

TEST(LocalFrame, ToLocalToGeodeticRoundTripNearbyPoint) {
    const GeoPoint origin{40.0, -105.0, 1600.0};
    const LocalFrame frame(origin);
    const GeoPoint nearby{40.01, -104.99, 1650.0};
    const Vec3 enu = frame.ToLocal(nearby);
    const GeoPoint back = frame.ToGeodetic(enu);
    EXPECT_NEAR(back.latitude_deg, nearby.latitude_deg, 1e-9);
    EXPECT_NEAR(back.longitude_deg, nearby.longitude_deg, 1e-9);
    EXPECT_NEAR(back.altitude_m, nearby.altitude_m, 1e-6);
}

TEST(LocalFrame, ToGeodeticToLocalRoundTripVector) {
    const GeoPoint origin{-22.9, -43.2, 10.0};
    const LocalFrame frame(origin);
    const Vec3 enu{1234.0, -567.0, 89.0};
    const Vec3 back = frame.ToLocal(frame.ToGeodetic(enu));
    EXPECT_NEAR(back.x, enu.x, 1e-3);
    EXPECT_NEAR(back.y, enu.y, 1e-3);
    EXPECT_NEAR(back.z, enu.z, 1e-3);
}

TEST(LocalFrame, DueNorthPointHasPositiveNorthAndZeroEast) {
    const GeoPoint origin{45.0, 10.0, 0.0};
    const LocalFrame frame(origin);
    const GeoPoint north{45.01, 10.0, 0.0};
    const Vec3 enu = frame.ToLocal(north);
    EXPECT_GT(enu.y, 0.0);
    EXPECT_NEAR(enu.x, 0.0, 1e-3);
    // Roughly 0.01 degrees of latitude is just over 1100 m of northing.
    EXPECT_NEAR(enu.y, 1112.0, 5.0);
}

TEST(LocalFrame, DueEastPointHasPositiveEastAndZeroNorth) {
    const GeoPoint origin{0.0, 10.0, 0.0};
    const LocalFrame frame(origin);
    const GeoPoint east{0.0, 10.01, 0.0};
    const Vec3 enu = frame.ToLocal(east);
    EXPECT_GT(enu.x, 0.0);
    EXPECT_NEAR(enu.y, 0.0, 1e-3);
    // At the equator 0.01 degrees of longitude is about 1113 m of easting.
    EXPECT_NEAR(enu.x, 1113.0, 5.0);
}

TEST(LocalFrame, PointAboveOriginHasPositiveUp) {
    const GeoPoint origin{30.0, -90.0, 0.0};
    const LocalFrame frame(origin);
    const GeoPoint above{30.0, -90.0, 500.0};
    const Vec3 enu = frame.ToLocal(above);
    EXPECT_NEAR(enu.x, 0.0, 1e-6);
    EXPECT_NEAR(enu.y, 0.0, 1e-6);
    EXPECT_NEAR(enu.z, 500.0, 1e-3);
}

TEST(Geodesy, HaversineDistanceIdenticalPointsIsZero) {
    const GeoPoint point{48.85, 2.35, 35.0};
    EXPECT_NEAR(HaversineDistanceM(point, point), 0.0, 1e-6);
}

TEST(Geodesy, HaversineDistanceOneDegreeLatitudeIsAbout111Km) {
    const GeoPoint from{0.0, 0.0, 0.0};
    const GeoPoint to{1.0, 0.0, 0.0};
    EXPECT_NEAR(HaversineDistanceM(from, to), 111195.0, 300.0);
}

TEST(Geodesy, HaversineDistanceIsSymmetric) {
    const GeoPoint from{51.5, -0.13, 0.0};
    const GeoPoint to{48.85, 2.35, 0.0};
    EXPECT_NEAR(HaversineDistanceM(from, to), HaversineDistanceM(to, from), 1e-6);
}

TEST(Geodesy, HaversineDistanceOneDegreeLongitudeAtEquator) {
    const GeoPoint from{0.0, 0.0, 0.0};
    const GeoPoint to{0.0, 1.0, 0.0};
    EXPECT_NEAR(HaversineDistanceM(from, to), 111195.0, 300.0);
}

TEST(Geodesy, HaversineDistanceAntipodalIsFiniteHalfCircumference) {
    // Exact antipodes drive the haversine term to 1.0 (modulo floating-point), which
    // is precisely where the std::min(1.0, h) clamp guards against asin of a value
    // just above 1. The distance must be the half great-circle circumference.
    const GeoPoint from{0.0, 0.0, 0.0};
    const GeoPoint to{0.0, 180.0, 0.0};
    const double distance = HaversineDistanceM(from, to);
    EXPECT_TRUE(std::isfinite(distance));
    EXPECT_NEAR(distance, kPi * Wgs84::kMeanRadiusM, 1.0);
}

TEST(Geodesy, HaversineDistanceAntipodalOffEquatorIsFinite) {
    // A second antipodal pair away from the equator varies the latitude terms while
    // still summing to h ~ 1, exercising the same clamp from a different direction.
    const GeoPoint from{45.0, 45.0, 0.0};
    const GeoPoint to{-45.0, -135.0, 0.0};
    const double distance = HaversineDistanceM(from, to);
    EXPECT_TRUE(std::isfinite(distance));
    EXPECT_NEAR(distance, kPi * Wgs84::kMeanRadiusM, 1.0);
}

TEST(Geodesy, InitialBearingDueNorthIsZero) {
    const GeoPoint from{45.0, 10.0, 0.0};
    const GeoPoint to{46.0, 10.0, 0.0};
    EXPECT_NEAR(InitialBearingRad(from, to), 0.0, 1e-6);
}

TEST(Geodesy, InitialBearingDueEastIsHalfPi) {
    const GeoPoint from{0.0, 10.0, 0.0};
    const GeoPoint to{0.0, 11.0, 0.0};
    EXPECT_NEAR(InitialBearingRad(from, to), kPi / 2.0, 1e-6);
}

TEST(Geodesy, InitialBearingDueSouthIsPi) {
    const GeoPoint from{45.0, 10.0, 0.0};
    const GeoPoint to{44.0, 10.0, 0.0};
    EXPECT_NEAR(std::fabs(InitialBearingRad(from, to)), kPi, 1e-6);
}

TEST(Geodesy, InitialBearingDueWestIsNegativeHalfPi) {
    const GeoPoint from{0.0, 10.0, 0.0};
    const GeoPoint to{0.0, 9.0, 0.0};
    EXPECT_NEAR(InitialBearingRad(from, to), -kPi / 2.0, 1e-6);
}

TEST(Geodesy, LocalFrameEnuVectorMagnitudeMatchesEcefSeparation) {
    const GeoPoint origin{37.77, -122.42, 16.0};
    const LocalFrame frame(origin);
    const GeoPoint other{37.78, -122.41, 100.0};
    const Vec3 enu = frame.ToLocal(other);
    const Vec3 ecef_diff = GeodeticToEcef(other) - GeodeticToEcef(origin);
    // The ENU transform is a pure rotation, so it preserves length.
    EXPECT_NEAR(Norm(enu), Norm(ecef_diff), 1e-6);
}
