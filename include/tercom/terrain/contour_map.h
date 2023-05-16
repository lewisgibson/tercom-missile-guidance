#pragma once

// The onboard terrain reference map -- the "TERCOM matrix" the correlator matches
// measured terrain profiles against. It wraps a digital elevation model and adds
// the roughness query the correlator uses to judge whether an area carries enough
// relief to yield an observable fix.

#include "tercom/geo/elevation_model.h"
#include "tercom/geo/height_field.h"

namespace TERCOM { namespace Terrain {

    class ContourMap {
      public:
        explicit ContourMap(Geo::HeightField terrain);

        const Geo::IElevationModel& Elevation() const {
            return terrain_;
        }

        const Geo::HeightField& Field() const {
            return terrain_;
        }

        Geo::GridBounds Bounds() const {
            return terrain_.Bounds();
        }

        // Standard deviation of terrain elevation over the grid nodes inside the
        // region. Near-zero roughness means the terrain cannot constrain the
        // horizontal fix.
        double TerrainRoughnessM(const Geo::GridBounds& region) const;

      private:
        Geo::HeightField terrain_;
    };

}} // namespace TERCOM::Terrain
