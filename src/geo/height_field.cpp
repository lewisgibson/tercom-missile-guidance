#include "tercom/geo/height_field.h"

#include <cmath>
#include <stdexcept>

namespace TERCOM { namespace Geo {

    HeightField::HeightField(std::size_t cols, std::size_t rows, double cell_size_m, double origin_east_m, double origin_north_m, std::vector<float> elevations)
        : cols_(cols),
          rows_(rows),
          cell_size_m_(cell_size_m),
          origin_east_m_(origin_east_m),
          origin_north_m_(origin_north_m),
          elevations_(std::move(elevations)) {
        // Bilinear sampling needs a neighbour in each direction, so the grid
        // must be at least 2x2.
        if (cols_ < 2 || rows_ < 2) {
            throw std::invalid_argument("HeightField requires at least a 2x2 grid");
        }
        if (cell_size_m_ <= 0.0) {
            throw std::invalid_argument("HeightField cell size must be positive");
        }
        if (elevations_.size() != cols_ * rows_) {
            throw std::invalid_argument("HeightField elevation count does not match grid dimensions");
        }
    }

    float HeightField::NodeElevation(std::size_t col, std::size_t row) const {
        // Row-major indexing: each row holds cols_ samples, so node (col, row)
        // is at offset row * cols_ + col.
        return elevations_[(row * cols_) + col];
    }

    // The covered area runs from the origin node (0, 0) to the far node. With
    // N nodes spaced cell_size_m apart there are N - 1 gaps, hence the extent
    // is (N - 1) * cell_size_m, not N * cell_size_m.
    GridBounds HeightField::Bounds() const {
        return GridBounds{
            origin_east_m_,
            origin_north_m_,
            origin_east_m_ + (static_cast<double>(cols_ - 1) * cell_size_m_),
            origin_north_m_ + (static_cast<double>(rows_ - 1) * cell_size_m_),
        };
    }

    std::optional<double> HeightField::ElevationAt(double east_m, double north_m) const {
        // Convert the ENU query point into fractional grid coordinates, where an
        // integer value lands exactly on a node and the fractional part says how
        // far between nodes the point sits. Dividing by the cell size turns
        // metres into node units.
        const double local_east = (east_m - origin_east_m_) / cell_size_m_;
        const double local_north = (north_m - origin_north_m_) / cell_size_m_;

        // Outside the grid to the south or west: no data here.
        if (local_east < 0.0 || local_north < 0.0) {
            return std::nullopt;
        }

        // Outside to the north or east. The last node is at index cols_ - 1, so
        // that is the largest valid fractional coordinate.
        const double max_col = static_cast<double>(cols_ - 1);
        const double max_row = static_cast<double>(rows_ - 1);
        if (local_east > max_col || local_north > max_row) {
            return std::nullopt;
        }

        // col0/row0 are the south-west node of the cell containing the point;
        // col1/row1 (below) are its north-east neighbours.
        const auto col0 = static_cast<std::size_t>(std::floor(local_east));
        const auto row0 = static_cast<std::size_t>(std::floor(local_north));
        // Clamp the upper node so a sample exactly on the northern or eastern
        // edge still interpolates against an in-range neighbour.
        const std::size_t col1 = (col0 + 1 < cols_) ? col0 + 1 : col0;
        const std::size_t row1 = (row0 + 1 < rows_) ? row0 + 1 : row0;

        // Position within the cell, 0 at the south/west edge, 1 at north/east.
        const double frac_east = local_east - static_cast<double>(col0);
        const double frac_north = local_north - static_cast<double>(row0);

        // Bilinear interpolation: blend the four surrounding node elevations,
        // weighting each by how close the query point is to it. z00 is the
        // south-west corner, z11 the north-east, and so on.
        const double z00 = static_cast<double>(NodeElevation(col0, row0));
        const double z10 = static_cast<double>(NodeElevation(col1, row0));
        const double z01 = static_cast<double>(NodeElevation(col0, row1));
        const double z11 = static_cast<double>(NodeElevation(col1, row1));

        // Interpolate east-west along the southern and northern edges first,
        // then interpolate those two results north-south.
        const double south = z00 + ((z10 - z00) * frac_east);
        const double north = z01 + ((z11 - z01) * frac_east);
        return south + ((north - south) * frac_north);
    }

}} // namespace TERCOM::Geo
