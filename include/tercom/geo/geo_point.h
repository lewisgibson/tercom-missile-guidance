#pragma once

namespace TERCOM { namespace Geo {

    // A point in geodetic coordinates on the WGS84 ellipsoid.
    // Altitude is height above the ellipsoid in metres. Real digital elevation
    // models are usually referenced to a geoid (mean sea level); the difference
    // (the geoid undulation) is treated as a constant bias by this library and
    // is documented where it matters.
    struct GeoPoint {
        double latitude_deg = 0.0;
        double longitude_deg = 0.0;
        double altitude_m = 0.0;
    };

}} // namespace TERCOM::Geo
