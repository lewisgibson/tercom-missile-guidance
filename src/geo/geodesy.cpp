#include "tercom/geo/geodesy.h"

#include <algorithm>
#include <cmath>

namespace TERCOM { namespace Geo {

    using Math::DegToRad;
    using Math::Mat3;
    using Math::Vec3;

    Vec3 GeodeticToEcef(const GeoPoint& point) {
        const double lat = DegToRad(point.latitude_deg);
        const double lon = DegToRad(point.longitude_deg);
        const double sin_lat = std::sin(lat);
        const double cos_lat = std::cos(lat);
        const double sin_lon = std::sin(lon);
        const double cos_lon = std::cos(lon);

        // Radius of curvature in the prime vertical (often written N): the
        // distance from the point on the surface, along the ellipsoid normal,
        // to where that normal crosses the spin axis. It depends on latitude
        // because the ellipsoid is not a sphere. This is the standard closed
        // form for the geodetic-to-ECEF conversion.
        const double prime_vertical = Wgs84::kSemiMajorAxisM / std::sqrt(1.0 - (Wgs84::kFirstEccentricitySquared * sin_lat * sin_lat));

        const double x = (prime_vertical + point.altitude_m) * cos_lat * cos_lon;
        const double y = (prime_vertical + point.altitude_m) * cos_lat * sin_lon;
        const double z = ((prime_vertical * (1.0 - Wgs84::kFirstEccentricitySquared)) + point.altitude_m) * sin_lat;
        return {x, y, z};
    }

    GeoPoint EcefToGeodetic(const Vec3& ecef) {
        const double a = Wgs84::kSemiMajorAxisM;
        const double b = Wgs84::kSemiMinorAxisM;
        const double e2 = Wgs84::kFirstEccentricitySquared;
        // Second eccentricity squared, e'^2 = (a^2 - b^2) / b^2, a companion to
        // the first eccentricity that Bowring's formula below needs.
        const double ep2 = ((a * a) - (b * b)) / (b * b);

        // p is the distance from the spin axis (the radius in the equatorial
        // plane). Longitude is exact from x and y; only latitude and altitude
        // require iteration or an approximation, because the surface is curved.
        const double p = std::sqrt((ecef.x * ecef.x) + (ecef.y * ecef.y));
        const double lon = std::atan2(ecef.y, ecef.x);

        GeoPoint result;
        result.longitude_deg = Math::RadToDeg(lon);

        // Degenerate case: the point is on the spin axis (a pole), where
        // longitude is undefined and the general formula divides by zero. The
        // 1e-9 m threshold (one nanometre) is just "p is effectively zero".
        // Latitude is then plus or minus 90 degrees and altitude is the
        // distance from the pole, |z| - b.
        if (p < 1e-9) {
            const double sign = (ecef.z >= 0.0) ? 1.0 : -1.0;
            result.latitude_deg = sign * 90.0;
            result.altitude_m = std::fabs(ecef.z) - b;
            return result;
        }

        // Bowring's formula: a one-shot closed-form approximation for latitude
        // that avoids iterating. theta is an auxiliary "parametric" angle used
        // to seed the latitude estimate; the next line refines it. Accurate to
        // well below a millimetre for any altitude near the Earth's surface.
        const double theta = std::atan2(ecef.z * a, p * b);
        const double sin_theta = std::sin(theta);
        const double cos_theta = std::cos(theta);
        const double lat = std::atan2(ecef.z + (ep2 * b * sin_theta * sin_theta * sin_theta), p - (e2 * a * cos_theta * cos_theta * cos_theta));
        const double sin_lat = std::sin(lat);
        const double prime_vertical = a / std::sqrt(1.0 - (e2 * sin_lat * sin_lat));

        result.latitude_deg = Math::RadToDeg(lat);
        // Altitude is how far the point sticks out past the ellipsoid surface
        // along the normal: project p back onto the surface and subtract the
        // prime-vertical radius.
        result.altitude_m = (p / std::cos(lat)) - prime_vertical;
        return result;
    }

    // Haversine: the standard great-circle distance on a sphere. h is the
    // "half-versed-sine" intermediate; the formula stays numerically stable for
    // very small distances where a naive cosine law loses precision. Treating
    // the Earth as a sphere of mean radius is good to a fraction of a percent,
    // which is plenty for waypoint spacing.
    double HaversineDistanceM(const GeoPoint& from, const GeoPoint& to) {
        const double lat1 = DegToRad(from.latitude_deg);
        const double lat2 = DegToRad(to.latitude_deg);
        const double d_lat = lat2 - lat1;
        const double d_lon = DegToRad(to.longitude_deg - from.longitude_deg);

        const double sin_half_lat = std::sin(d_lat / 2.0);
        const double sin_half_lon = std::sin(d_lon / 2.0);
        const double h = (sin_half_lat * sin_half_lat) + (std::cos(lat1) * std::cos(lat2) * sin_half_lon * sin_half_lon);
        // std::min(1.0, h) guards against tiny floating-point overshoot above 1
        // that would make asin produce NaN for near-antipodal points.
        return 2.0 * Wgs84::kMeanRadiusM * std::asin(std::sqrt(std::min(1.0, h)));
    }

    // Initial bearing (also called forward azimuth): the compass heading you
    // start out on to follow the great circle to the destination. It changes as
    // you travel, so it is only the heading at the start. Returned in radians
    // clockwise from north.
    double InitialBearingRad(const GeoPoint& from, const GeoPoint& to) {
        const double lat1 = DegToRad(from.latitude_deg);
        const double lat2 = DegToRad(to.latitude_deg);
        const double d_lon = DegToRad(to.longitude_deg - from.longitude_deg);

        const double y = std::sin(d_lon) * std::cos(lat2);
        const double x = (std::cos(lat1) * std::sin(lat2)) - (std::sin(lat1) * std::cos(lat2) * std::cos(d_lon));
        return std::atan2(y, x);
    }

    namespace {
        // Rotation that turns an ECEF vector into local East-North-Up at the
        // given latitude/longitude. The three rows are the east, north and up
        // unit vectors expressed in ECEF: row 0 (east) is horizontal and points
        // along increasing longitude, row 1 (north) points along the surface
        // toward the pole, row 2 (up) is the outward ellipsoid normal. This is
        // the textbook ECEF-to-ENU direction cosine matrix.
        Mat3 EcefToEnuRotation(double lat_rad, double lon_rad) {
            const double sin_lat = std::sin(lat_rad);
            const double cos_lat = std::cos(lat_rad);
            const double sin_lon = std::sin(lon_rad);
            const double cos_lon = std::cos(lon_rad);
            return Mat3({
                -sin_lon,
                cos_lon,
                0.0,
                -sin_lat * cos_lon,
                -sin_lat * sin_lon,
                cos_lat,
                cos_lat * cos_lon,
                cos_lat * sin_lon,
                sin_lat,
            });
        }
    } // namespace

    LocalFrame::LocalFrame(const GeoPoint& origin)
        : origin_(origin),
          origin_ecef_(GeodeticToEcef(origin)),
          ecef_to_enu_(EcefToEnuRotation(DegToRad(origin.latitude_deg), DegToRad(origin.longitude_deg))) {
    }

    // Map a geodetic point into the local frame: convert to ECEF, take the
    // offset from the origin, then rotate that offset into east/north/up.
    Vec3 LocalFrame::ToLocal(const GeoPoint& point) const {
        return ecef_to_enu_ * (GeodeticToEcef(point) - origin_ecef_);
    }

    // Inverse of ToLocal. Because the rotation matrix is orthonormal its inverse
    // is just its transpose, so rotate the ENU offset back into ECEF, add the
    // origin, and convert to geodetic.
    GeoPoint LocalFrame::ToGeodetic(const Vec3& enu) const {
        return EcefToGeodetic(origin_ecef_ + (ecef_to_enu_.Transposed() * enu));
    }

}} // namespace TERCOM::Geo
