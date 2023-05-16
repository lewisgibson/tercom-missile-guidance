#pragma once

// Abstract terrain elevation source. The TERCOM reference matrix and the
// correlator depend only on this interface, never on a concrete digital elevation
// model; the simulator also queries it to compute the ground-truth clearance it
// feeds the simulated radar altimeter. Positions are expressed in the local
// East-North-Up tangent plane in metres.

#include <optional>

namespace TERCOM { namespace Geo {

    // An axis-aligned bounding box in the local East-North-Up tangent plane, all
    // coordinates in metres. Describes the rectangular area an elevation model covers
    // and bounds the correlator's offset search.
    struct GridBounds {
        double min_east_m = 0.0;  // western edge, metres
        double min_north_m = 0.0; // southern edge, metres
        double max_east_m = 0.0;  // eastern edge, metres
        double max_north_m = 0.0; // northern edge, metres

        // True when (east_m, north_m) lies on or inside the box (edges inclusive).
        bool Contains(double east_m, double north_m) const {
            return east_m >= min_east_m && east_m <= max_east_m && north_m >= min_north_m && north_m <= max_north_m;
        }
    };

    class IElevationModel {
      public:
        virtual ~IElevationModel() = default;

        // Terrain elevation in metres at a local horizontal position, or nullopt
        // if the position lies outside the modelled area.
        virtual std::optional<double> ElevationAt(double east_m, double north_m) const = 0;

        // Rectangular extent of the modelled area in the local East-North-Up tangent
        // plane (metres). ElevationAt returns nullopt for any position outside these.
        virtual GridBounds Bounds() const = 0;
    };

}} // namespace TERCOM::Geo
