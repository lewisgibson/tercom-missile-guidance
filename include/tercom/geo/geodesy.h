#pragma once

// WGS84 geodesy: conversions between geodetic coordinates, the Earth-Centred
// Earth-Fixed (ECEF) frame, and a local East-North-Up (ENU) tangent plane, plus
// great-circle helpers. The navigation pipeline runs in the local ENU frame;
// these conversions anchor it to the real ellipsoid and let real geographic
// digital elevation models be ingested.

#include "tercom/geo/geo_point.h"
#include "tercom/math/linear.h"

namespace TERCOM { namespace Geo {

    // WGS84 is the reference ellipsoid that GPS and most modern maps use. The
    // Earth is modelled as a sphere flattened slightly at the poles; these four
    // numbers fully describe that shape. The two defining constants (semi-major
    // axis and flattening) are exact by definition of the standard; the other
    // two are derived from them.
    namespace Wgs84 {
        // Equatorial radius in metres: the distance from the Earth's centre to
        // the equator (the long axis of the ellipse). Defined exactly by WGS84.
        inline constexpr double kSemiMajorAxisM = 6378137.0;
        // Flattening: how much shorter the polar axis is than the equatorial
        // axis, as a fraction. The denominator 298.257223563 is the WGS84
        // defining value, so flattening is about 1/298, i.e. the poles are
        // roughly 0.3 percent closer to the centre than the equator.
        inline constexpr double kFlattening = 1.0 / 298.257223563;
        // Polar radius in metres, derived: b = a * (1 - f). About 6356752 m.
        inline constexpr double kSemiMinorAxisM = kSemiMajorAxisM * (1.0 - kFlattening);
        // First eccentricity squared, derived: e^2 = f * (2 - f). This appears
        // throughout ellipsoid maths; it measures how far the ellipse departs
        // from a circle (zero would be a perfect sphere). About 0.00669.
        inline constexpr double kFirstEccentricitySquared = kFlattening * (2.0 - kFlattening);
        // Mean Earth radius in metres for treating the Earth as a sphere in the
        // great-circle helpers below. 6371008.8 m is the IUGG arithmetic-mean
        // radius R1 = (2a + b) / 3 for the WGS84 ellipsoid.
        inline constexpr double kMeanRadiusM = 6371008.8;
    } // namespace Wgs84

    // ECEF (Earth-Centred Earth-Fixed) is a Cartesian frame whose origin is the
    // Earth's centre, which rotates with the Earth: the x axis points at the
    // intersection of the equator and the prime meridian, z points to the north
    // pole, y completes the right-handed set. Converting a latitude/longitude/
    // altitude to and from x/y/z metres in this frame is the bridge between map
    // coordinates and ordinary vector maths.
    Math::Vec3 GeodeticToEcef(const GeoPoint& point);
    GeoPoint EcefToGeodetic(const Math::Vec3& ecef);

    // Great-circle distance and initial bearing (radians, clockwise from north).
    double HaversineDistanceM(const GeoPoint& from, const GeoPoint& to);
    double InitialBearingRad(const GeoPoint& from, const GeoPoint& to);

    // A local East-North-Up tangent plane anchored at a geodetic origin. ENU
    // vectors are in metres: x = east, y = north, z = up.
    class LocalFrame {
      public:
        explicit LocalFrame(const GeoPoint& origin);

        const GeoPoint& Origin() const {
            return origin_;
        }

        Math::Vec3 ToLocal(const GeoPoint& point) const;
        GeoPoint ToGeodetic(const Math::Vec3& enu) const;

      private:
        GeoPoint origin_;
        Math::Vec3 origin_ecef_;
        Math::Mat3 ecef_to_enu_;
    };

}} // namespace TERCOM::Geo
