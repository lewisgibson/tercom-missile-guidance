#include <gtest/gtest.h>

#include "tercom/geo/ascii_grid.h"
#include "tercom/geo/height_field.h"
#include "tercom/geo/synthetic_terrain.h"

#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

    // Build a small deterministic HeightField directly. Row 0 is the southern
    // edge, stored row-major.
    TERCOM::Geo::HeightField MakeSmallField() {
        const std::size_t cols = 4;
        const std::size_t rows = 3;
        std::vector<float> elevations(cols * rows);
        for (std::size_t row = 0; row < rows; ++row) {
            for (std::size_t col = 0; col < cols; ++col) {
                elevations[(row * cols) + col] = static_cast<float>((row * 10) + col) + 0.25F;
            }
        }
        return TERCOM::Geo::HeightField(cols, rows, 25.0, 100.0, 200.0, std::move(elevations));
    }

} // namespace

TEST(AsciiGrid, RoundTripPreservesGeometryAndElevations) {
    const TERCOM::Geo::HeightField field = MakeSmallField();

    std::ostringstream out;
    TERCOM::Geo::WriteEsriAsciiGrid(out, field);

    std::istringstream in(out.str());
    const TERCOM::Geo::HeightField reloaded = TERCOM::Geo::LoadEsriAsciiGrid(in);

    EXPECT_EQ(reloaded.Cols(), field.Cols());
    EXPECT_EQ(reloaded.Rows(), field.Rows());
    EXPECT_NEAR(reloaded.CellSizeM(), field.CellSizeM(), 1e-9);
    EXPECT_NEAR(reloaded.OriginEastM(), field.OriginEastM(), 1e-9);
    EXPECT_NEAR(reloaded.OriginNorthM(), field.OriginNorthM(), 1e-9);

    for (std::size_t row = 0; row < field.Rows(); ++row) {
        for (std::size_t col = 0; col < field.Cols(); ++col) {
            EXPECT_NEAR(reloaded.NodeElevation(col, row), field.NodeElevation(col, row), 1e-3F);
        }
    }
}

TEST(AsciiGrid, RoundTripFractalTerrain) {
    TERCOM::Geo::FractalTerrainConfiguration parameters;
    parameters.cols = 16;
    parameters.rows = 12;
    parameters.cell_size_m = 50.0;
    parameters.origin_east_m = -300.0;
    parameters.origin_north_m = 1234.5;
    parameters.seed = 0x1234ABCDULL;
    const TERCOM::Geo::HeightField field = TERCOM::Geo::GenerateFractalTerrain(parameters);

    std::ostringstream out;
    TERCOM::Geo::WriteEsriAsciiGrid(out, field);

    std::istringstream in(out.str());
    const TERCOM::Geo::HeightField reloaded = TERCOM::Geo::LoadEsriAsciiGrid(in);

    EXPECT_EQ(reloaded.Cols(), field.Cols());
    EXPECT_EQ(reloaded.Rows(), field.Rows());
    EXPECT_NEAR(reloaded.CellSizeM(), field.CellSizeM(), 1e-9);
    EXPECT_NEAR(reloaded.OriginEastM(), field.OriginEastM(), 1e-9);
    EXPECT_NEAR(reloaded.OriginNorthM(), field.OriginNorthM(), 1e-9);

    for (std::size_t row = 0; row < field.Rows(); ++row) {
        for (std::size_t col = 0; col < field.Cols(); ++col) {
            EXPECT_NEAR(reloaded.NodeElevation(col, row), field.NodeElevation(col, row), 1e-2F);
        }
    }
}

TEST(AsciiGrid, WriteUsesCellCentreHeaderKeys) {
    const TERCOM::Geo::HeightField field = MakeSmallField();

    std::ostringstream out;
    TERCOM::Geo::WriteEsriAsciiGrid(out, field);
    const std::string text = out.str();

    EXPECT_NE(text.find("ncols 4"), std::string::npos);
    EXPECT_NE(text.find("nrows 3"), std::string::npos);
    EXPECT_NE(text.find("xllcenter"), std::string::npos);
    EXPECT_NE(text.find("yllcenter"), std::string::npos);
    EXPECT_NE(text.find("cellsize"), std::string::npos);
    EXPECT_NE(text.find("NODATA_value"), std::string::npos);
}

TEST(AsciiGrid, WriteHonoursNodataValue) {
    const TERCOM::Geo::HeightField field = MakeSmallField();

    std::ostringstream out;
    TERCOM::Geo::WriteEsriAsciiGrid(out, field, -1.0);
    const std::string text = out.str();

    EXPECT_NE(text.find("NODATA_value -1"), std::string::npos);
}

TEST(AsciiGrid, ParseMinimalCornerGridShiftsOriginByHalfCell) {
    // North row is listed first in the file; row 0 of the HeightField is the
    // southern (last) file row. Corner origins are shifted by half a cell to a
    // cell centre.
    const std::string grid = "ncols 3\n"
                             "nrows 2\n"
                             "xllcorner 1000\n"
                             "yllcorner 2000\n"
                             "cellsize 30\n"
                             "NODATA_value -9999\n"
                             "10 11 12\n"  // northern row -> becomes HeightField row 1
                             "20 21 22\n"; // southern row -> becomes HeightField row 0

    std::istringstream in(grid);
    const TERCOM::Geo::HeightField field = TERCOM::Geo::LoadEsriAsciiGrid(in);

    EXPECT_EQ(field.Cols(), 3u);
    EXPECT_EQ(field.Rows(), 2u);
    EXPECT_NEAR(field.CellSizeM(), 30.0, 1e-9);

    // Corner origin shifted by half a cell (30 / 2 = 15).
    EXPECT_NEAR(field.OriginEastM(), 1015.0, 1e-9);
    EXPECT_NEAR(field.OriginNorthM(), 2015.0, 1e-9);

    // Vertical flip: southern file row becomes row 0.
    EXPECT_NEAR(field.NodeElevation(0, 0), 20.0F, 1e-4F);
    EXPECT_NEAR(field.NodeElevation(2, 0), 22.0F, 1e-4F);
    EXPECT_NEAR(field.NodeElevation(0, 1), 10.0F, 1e-4F);
    EXPECT_NEAR(field.NodeElevation(2, 1), 12.0F, 1e-4F);
}

TEST(AsciiGrid, ParseCentreGridKeepsOrigin) {
    const std::string grid = "ncols 2\n"
                             "nrows 2\n"
                             "xllcenter 500\n"
                             "yllcenter 600\n"
                             "cellsize 10\n"
                             "1 2\n"
                             "3 4\n";

    std::istringstream in(grid);
    const TERCOM::Geo::HeightField field = TERCOM::Geo::LoadEsriAsciiGrid(in);

    EXPECT_EQ(field.Cols(), 2u);
    EXPECT_EQ(field.Rows(), 2u);
    // Centre origins are kept unshifted.
    EXPECT_NEAR(field.OriginEastM(), 500.0, 1e-9);
    EXPECT_NEAR(field.OriginNorthM(), 600.0, 1e-9);

    EXPECT_NEAR(field.NodeElevation(0, 0), 3.0F, 1e-4F);
    EXPECT_NEAR(field.NodeElevation(1, 1), 2.0F, 1e-4F);
}

TEST(AsciiGrid, HeaderKeysAreCaseInsensitive) {
    const std::string grid = "NCOLS 2\n"
                             "NRows 2\n"
                             "XLLCorner 0\n"
                             "YLLCorner 0\n"
                             "CellSize 5\n"
                             "7 8\n"
                             "9 10\n";

    std::istringstream in(grid);
    const TERCOM::Geo::HeightField field = TERCOM::Geo::LoadEsriAsciiGrid(in);
    EXPECT_EQ(field.Cols(), 2u);
    EXPECT_EQ(field.Rows(), 2u);
    EXPECT_NEAR(field.CellSizeM(), 5.0, 1e-9);
}

TEST(AsciiGrid, UnknownHeaderKeyThrows) {
    const std::string grid = "ncols 2\n"
                             "nrows 2\n"
                             "xllcorner 0\n"
                             "yllcorner 0\n"
                             "cellsize 1\n"
                             "bogus 42\n"
                             "1 2\n"
                             "3 4\n";

    std::istringstream in(grid);
    EXPECT_THROW(TERCOM::Geo::LoadEsriAsciiGrid(in), std::runtime_error);
}

TEST(AsciiGrid, MissingRequiredFieldThrows) {
    // Missing cellsize.
    const std::string grid = "ncols 2\n"
                             "nrows 2\n"
                             "xllcorner 0\n"
                             "yllcorner 0\n"
                             "1 2\n"
                             "3 4\n";

    std::istringstream in(grid);
    EXPECT_THROW(TERCOM::Geo::LoadEsriAsciiGrid(in), std::runtime_error);
}

TEST(AsciiGrid, GridSmallerThanTwoByTwoThrows) {
    const std::string grid = "ncols 1\n"
                             "nrows 2\n"
                             "xllcorner 0\n"
                             "yllcorner 0\n"
                             "cellsize 1\n"
                             "5\n"
                             "6\n";

    std::istringstream in(grid);
    EXPECT_THROW(TERCOM::Geo::LoadEsriAsciiGrid(in), std::runtime_error);
}

TEST(AsciiGrid, EmptyInputThrows) {
    std::istringstream in("");
    EXPECT_THROW(TERCOM::Geo::LoadEsriAsciiGrid(in), std::runtime_error);
}

TEST(AsciiGrid, TooFewElevationValuesThrows) {
    const std::string grid = "ncols 3\n"
                             "nrows 2\n"
                             "xllcorner 0\n"
                             "yllcorner 0\n"
                             "cellsize 1\n"
                             "1 2 3\n"
                             "4 5\n"; // one short

    std::istringstream in(grid);
    EXPECT_THROW(TERCOM::Geo::LoadEsriAsciiGrid(in), std::runtime_error);
}

TEST(AsciiGrid, NonNumericHeaderValueThrows) {
    const std::string grid = "ncols abc\n"
                             "nrows 2\n";
    std::istringstream in(grid);
    EXPECT_THROW(TERCOM::Geo::LoadEsriAsciiGrid(in), std::runtime_error);
}

TEST(AsciiGrid, FilePathRoundTrip) {
    const std::vector<float> elevations = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F};
    const TERCOM::Geo::HeightField original(3, 2, 100.0, 10.0, 20.0, elevations);

    const std::string path = "tercom_ascii_grid_roundtrip.asc";
    TERCOM::Geo::WriteEsriAsciiGrid(path, original);
    const TERCOM::Geo::HeightField loaded = TERCOM::Geo::LoadEsriAsciiGrid(path);
    std::remove(path.c_str());

    EXPECT_EQ(loaded.Cols(), original.Cols());
    EXPECT_EQ(loaded.Rows(), original.Rows());
    EXPECT_NEAR(loaded.CellSizeM(), original.CellSizeM(), 1e-9);
    EXPECT_NEAR(loaded.OriginEastM(), original.OriginEastM(), 1e-9);
    EXPECT_NEAR(loaded.OriginNorthM(), original.OriginNorthM(), 1e-9);
    for (std::size_t row = 0; row < original.Rows(); ++row) {
        for (std::size_t col = 0; col < original.Cols(); ++col) {
            EXPECT_NEAR(loaded.NodeElevation(col, row), original.NodeElevation(col, row), 1e-3);
        }
    }
}

TEST(AsciiGrid, LoadMissingFileThrows) {
    EXPECT_THROW(TERCOM::Geo::LoadEsriAsciiGrid("/no/such/tercom/grid.asc"), std::runtime_error);
}

TEST(AsciiGrid, WriteToUnwritablePathThrows) {
    const std::vector<float> elevations = {1.0F, 2.0F, 3.0F, 4.0F};
    const TERCOM::Geo::HeightField field(2, 2, 1.0, 0.0, 0.0, elevations);
    EXPECT_THROW(TERCOM::Geo::WriteEsriAsciiGrid("/no/such/tercom/dir/grid.asc", field), std::runtime_error);
}

TEST(AsciiGrid, NodataCellsAreRejected) {
    // A grid that declares a NODATA sentinel and then uses it for a void cell must
    // be rejected: HeightField has no way to represent a hole, so silently keeping
    // the sentinel would inject bogus terrain.
    const std::string grid = "ncols 2\n"
                             "nrows 2\n"
                             "xllcorner 0\n"
                             "yllcorner 0\n"
                             "cellsize 1\n"
                             "NODATA_value -9999\n"
                             "1 2\n"
                             "3 -9999\n";
    std::istringstream in(grid);
    EXPECT_THROW(TERCOM::Geo::LoadEsriAsciiGrid(in), std::runtime_error);
}
