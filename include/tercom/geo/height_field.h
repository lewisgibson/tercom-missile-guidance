#pragma once

// A regular-grid digital elevation model held in memory. Nodes are stored
// row-major from the south-west corner (north and east increasing). Sampling
// between nodes is bilinear.

#include <cstddef>
#include <optional>
#include <vector>

#include "tercom/geo/elevation_model.h"

namespace TERCOM { namespace Geo {

    // Marked final so that elevation lookups made through a HeightField pointer
    // devirtualise and inline; the TERCOM correlator relies on this for its hot loop.
    class HeightField final : public IElevationModel {
      public:
        // elevations holds cols * rows samples, row-major, row 0 at the southern
        // edge. cell_size_m is the node spacing in both axes. The origin is the
        // ENU position of node (0, 0).
        HeightField(std::size_t cols, std::size_t rows, double cell_size_m, double origin_east_m, double origin_north_m, std::vector<float> elevations);

        std::optional<double> ElevationAt(double east_m, double north_m) const override;
        GridBounds Bounds() const override;

        std::size_t Cols() const {
            return cols_;
        }

        std::size_t Rows() const {
            return rows_;
        }

        double CellSizeM() const {
            return cell_size_m_;
        }

        double OriginEastM() const {
            return origin_east_m_;
        }

        double OriginNorthM() const {
            return origin_north_m_;
        }

        // Elevation at a grid node. Indices must be in range.
        float NodeElevation(std::size_t col, std::size_t row) const;

        const std::vector<float>& Data() const {
            return elevations_;
        }

      private:
        std::size_t cols_;
        std::size_t rows_;
        double cell_size_m_;
        double origin_east_m_;
        double origin_north_m_;
        std::vector<float> elevations_;
    };

}} // namespace TERCOM::Geo
