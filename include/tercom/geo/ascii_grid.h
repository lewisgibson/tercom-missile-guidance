#pragma once

// Reader and writer for the Esri / USGS ASCII grid format (".asc"), a plain-text
// raster digital elevation model (DEM). The file begins with a small header of
// "key value" lines naming the grid width (ncols), height (nrows), the
// south-west reference point (xllcorner/yllcorner or xllcenter/yllcenter), the
// node spacing (cellsize) and an optional placeholder for missing data
// (NODATA_value). The elevation samples then follow as whitespace-separated
// numbers. Values in the file are ordered north row first (top of the file is
// the top of the map); HeightField stores south row first, so the loader flips
// the rows vertically. The grid is treated as a planar raster whose coordinates
// are local ENU metres; geographic (degree-spaced) grids should be projected
// before loading.

#include <istream>
#include <ostream>
#include <string>

#include "tercom/geo/height_field.h"

namespace TERCOM { namespace Geo {

    HeightField LoadEsriAsciiGrid(std::istream& input);
    HeightField LoadEsriAsciiGrid(const std::string& path);

    // nodata_value is the sentinel written for missing samples. -9999.0 is the
    // de facto default for this format (a value no real elevation would take).
    // HeightField has no concept of missing nodes, so the writer never actually
    // emits it; it is recorded in the header only for compatibility with other
    // tools that read the file.
    void WriteEsriAsciiGrid(std::ostream& output, const HeightField& field, double nodata_value = -9999.0);
    void WriteEsriAsciiGrid(const std::string& path, const HeightField& field, double nodata_value = -9999.0);

}} // namespace TERCOM::Geo
