#include "tercom/terrain/contour_map.h"

#include <cmath>
#include <utility>

namespace TERCOM { namespace Terrain {

    ContourMap::ContourMap(Geo::HeightField terrain) : terrain_(std::move(terrain)) {
    }

    // Roughness is the standard deviation of node elevations inside the region.
    // We accumulate the running sum and sum of squares in one pass and recover the
    // variance from them (variance = mean of squares minus square of the mean),
    // which avoids a second pass to compute deviations from the mean.
    double ContourMap::TerrainRoughnessM(const Geo::GridBounds& region) const {
        double sum = 0.0;
        double sum_of_squares = 0.0;
        std::size_t count = 0;

        // Walk the grid node by node. Nodes are laid out row-major from the
        // south-west corner, so the world position of a node is the origin plus its
        // index times the cell spacing; skip any node falling outside the region.
        for (std::size_t row = 0; row < terrain_.Rows(); ++row) {
            const double north = terrain_.OriginNorthM() + (static_cast<double>(row) * terrain_.CellSizeM());
            if (north < region.min_north_m || north > region.max_north_m) {
                continue;
            }
            for (std::size_t col = 0; col < terrain_.Cols(); ++col) {
                const double east = terrain_.OriginEastM() + (static_cast<double>(col) * terrain_.CellSizeM());
                if (east < region.min_east_m || east > region.max_east_m) {
                    continue;
                }
                const double elevation = static_cast<double>(terrain_.NodeElevation(col, row));
                sum += elevation;
                sum_of_squares += elevation * elevation;
                ++count;
            }
        }

        // A standard deviation needs at least two samples to be meaningful; with
        // fewer, report zero roughness so the caller treats the area as featureless.
        if (count < 2) {
            return 0.0;
        }
        const double mean = sum / static_cast<double>(count);
        const double variance = (sum_of_squares / static_cast<double>(count)) - (mean * mean);
        // Clamp at zero before the square root: floating-point cancellation in the
        // one-pass formula can leave a tiny negative variance for near-flat regions.
        return std::sqrt(variance > 0.0 ? variance : 0.0);
    }

}} // namespace TERCOM::Terrain
