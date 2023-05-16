#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>
#include <vector>

#include "tercom/geo/elevation_model.h"
#include "tercom/geo/height_field.h"

namespace {

    // Builds a known 3x3 grid, cell 100, origin (0,0). Stored row-major with
    // row 0 at the southern edge (north increasing with row index):
    //   row 0 (north = 0):   10, 20, 30
    //   row 1 (north = 100): 40, 50, 60
    //   row 2 (north = 200): 70, 80, 90
    TERCOM::Geo::HeightField MakeGrid() {
        const std::vector<float> elevations = {
            10.0F,
            20.0F,
            30.0F,
            40.0F,
            50.0F,
            60.0F,
            70.0F,
            80.0F,
            90.0F,
        };
        return TERCOM::Geo::HeightField(3, 3, 100.0, 0.0, 0.0, elevations);
    }

} // namespace

TEST(HeightField, ConstructorAcceptsValidGrid) {
    EXPECT_NO_THROW({
        const std::vector<float> elevations(4, 0.0F);
        TERCOM::Geo::HeightField field(2, 2, 50.0, 0.0, 0.0, elevations);
    });
}

TEST(HeightField, ConstructorRejectsTooFewColumns) {
    const std::vector<float> elevations(2, 0.0F);
    EXPECT_THROW(TERCOM::Geo::HeightField(1, 2, 100.0, 0.0, 0.0, elevations), std::invalid_argument);
}

TEST(HeightField, ConstructorRejectsTooFewRows) {
    const std::vector<float> elevations(2, 0.0F);
    EXPECT_THROW(TERCOM::Geo::HeightField(2, 1, 100.0, 0.0, 0.0, elevations), std::invalid_argument);
}

TEST(HeightField, ConstructorRejectsZeroCellSize) {
    const std::vector<float> elevations(4, 0.0F);
    EXPECT_THROW(TERCOM::Geo::HeightField(2, 2, 0.0, 0.0, 0.0, elevations), std::invalid_argument);
}

TEST(HeightField, ConstructorRejectsNegativeCellSize) {
    const std::vector<float> elevations(4, 0.0F);
    EXPECT_THROW(TERCOM::Geo::HeightField(2, 2, -100.0, 0.0, 0.0, elevations), std::invalid_argument);
}

TEST(HeightField, ConstructorRejectsTooFewElevations) {
    const std::vector<float> elevations(8, 0.0F);
    EXPECT_THROW(TERCOM::Geo::HeightField(3, 3, 100.0, 0.0, 0.0, elevations), std::invalid_argument);
}

TEST(HeightField, ConstructorRejectsTooManyElevations) {
    const std::vector<float> elevations(10, 0.0F);
    EXPECT_THROW(TERCOM::Geo::HeightField(3, 3, 100.0, 0.0, 0.0, elevations), std::invalid_argument);
}

TEST(HeightField, AccessorsReportConstructionParameters) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    EXPECT_EQ(field.Cols(), 3u);
    EXPECT_EQ(field.Rows(), 3u);
    EXPECT_NEAR(field.CellSizeM(), 100.0, 1e-12);
    EXPECT_NEAR(field.OriginEastM(), 0.0, 1e-12);
    EXPECT_NEAR(field.OriginNorthM(), 0.0, 1e-12);
}

TEST(HeightField, AccessorsHonourNonZeroOrigin) {
    const std::vector<float> elevations(4, 0.0F);
    const TERCOM::Geo::HeightField field(2, 2, 25.0, -500.0, 1200.0, elevations);
    EXPECT_NEAR(field.CellSizeM(), 25.0, 1e-12);
    EXPECT_NEAR(field.OriginEastM(), -500.0, 1e-12);
    EXPECT_NEAR(field.OriginNorthM(), 1200.0, 1e-12);
}

TEST(HeightField, NodeElevationReturnsStoredNodes) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    EXPECT_FLOAT_EQ(field.NodeElevation(0, 0), 10.0F);
    EXPECT_FLOAT_EQ(field.NodeElevation(1, 0), 20.0F);
    EXPECT_FLOAT_EQ(field.NodeElevation(2, 0), 30.0F);
    EXPECT_FLOAT_EQ(field.NodeElevation(0, 1), 40.0F);
    EXPECT_FLOAT_EQ(field.NodeElevation(1, 1), 50.0F);
    EXPECT_FLOAT_EQ(field.NodeElevation(2, 1), 60.0F);
    EXPECT_FLOAT_EQ(field.NodeElevation(0, 2), 70.0F);
    EXPECT_FLOAT_EQ(field.NodeElevation(1, 2), 80.0F);
    EXPECT_FLOAT_EQ(field.NodeElevation(2, 2), 90.0F);
}

TEST(HeightField, DataMatchesRowMajorLayout) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    const std::vector<float>& data = field.Data();
    ASSERT_EQ(data.size(), 9u);
    EXPECT_FLOAT_EQ(data[0], 10.0F);
    EXPECT_FLOAT_EQ(data[4], 50.0F);
    EXPECT_FLOAT_EQ(data[8], 90.0F);
}

TEST(HeightField, ElevationOnNodeEqualsNode) {
    const TERCOM::Geo::HeightField field = MakeGrid();

    // South-west origin node.
    const std::optional<double> sw = field.ElevationAt(0.0, 0.0);
    ASSERT_TRUE(sw.has_value());
    EXPECT_NEAR(*sw, 10.0, 1e-9);

    // Centre node.
    const std::optional<double> centre = field.ElevationAt(100.0, 100.0);
    ASSERT_TRUE(centre.has_value());
    EXPECT_NEAR(*centre, 50.0, 1e-9);

    // North-east corner node.
    const std::optional<double> ne = field.ElevationAt(200.0, 200.0);
    ASSERT_TRUE(ne.has_value());
    EXPECT_NEAR(*ne, 90.0, 1e-9);
}

TEST(HeightField, ElevationAtCellCentreIsBilinearAverage) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    // Centre of south-west cell: corners 10, 20, 40, 50 -> average 30.
    const std::optional<double> value = field.ElevationAt(50.0, 50.0);
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, 30.0, 1e-9);
}

TEST(HeightField, ElevationAlongSouthEdgeInterpolatesEast) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    // Halfway between nodes 10 and 20 on the southern edge.
    const std::optional<double> value = field.ElevationAt(50.0, 0.0);
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, 15.0, 1e-9);
}

TEST(HeightField, ElevationOnNorthEdgeInterpolatesCorrectly) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    // North edge exactly at row 2; between nodes 70 and 80 -> 75.
    const std::optional<double> value = field.ElevationAt(50.0, 200.0);
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, 75.0, 1e-9);
}

TEST(HeightField, ElevationOnEastEdgeInterpolatesCorrectly) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    // East edge exactly at col 2; between nodes 30 and 60 -> 45.
    const std::optional<double> value = field.ElevationAt(200.0, 50.0);
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, 45.0, 1e-9);
}

TEST(HeightField, ElevationAtArbitraryInteriorPoint) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    // Inside south-west cell at frac_east = 0.25, frac_north = 0.75.
    // south = 10 + (20-10)*0.25 = 12.5
    // north = 40 + (50-40)*0.25 = 42.5
    // value = 12.5 + (42.5-12.5)*0.75 = 35.0
    const std::optional<double> value = field.ElevationAt(25.0, 75.0);
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, 35.0, 1e-9);
}

TEST(HeightField, ElevationOutsideNegativeEastReturnsNullopt) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    EXPECT_FALSE(field.ElevationAt(-1.0, 50.0).has_value());
}

TEST(HeightField, ElevationOutsideNegativeNorthReturnsNullopt) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    EXPECT_FALSE(field.ElevationAt(50.0, -1.0).has_value());
}

TEST(HeightField, ElevationBeyondMaxEastReturnsNullopt) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    EXPECT_FALSE(field.ElevationAt(200.0001, 50.0).has_value());
}

TEST(HeightField, ElevationBeyondMaxNorthReturnsNullopt) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    EXPECT_FALSE(field.ElevationAt(50.0, 200.0001).has_value());
}

TEST(HeightField, BoundsReportGridExtent) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    const TERCOM::Geo::GridBounds bounds = field.Bounds();
    EXPECT_NEAR(bounds.min_east_m, 0.0, 1e-9);
    EXPECT_NEAR(bounds.min_north_m, 0.0, 1e-9);
    EXPECT_NEAR(bounds.max_east_m, 200.0, 1e-9);
    EXPECT_NEAR(bounds.max_north_m, 200.0, 1e-9);
}

TEST(HeightField, BoundsHonourNonZeroOrigin) {
    const std::vector<float> elevations(6, 0.0F);
    const TERCOM::Geo::HeightField field(3, 2, 100.0, -50.0, 1000.0, elevations);
    const TERCOM::Geo::GridBounds bounds = field.Bounds();
    EXPECT_NEAR(bounds.min_east_m, -50.0, 1e-9);
    EXPECT_NEAR(bounds.min_north_m, 1000.0, 1e-9);
    EXPECT_NEAR(bounds.max_east_m, 150.0, 1e-9);   // -50 + (3-1)*100
    EXPECT_NEAR(bounds.max_north_m, 1100.0, 1e-9); // 1000 + (2-1)*100
}

TEST(HeightField, BoundsContainsInteriorAndCorners) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    const TERCOM::Geo::GridBounds bounds = field.Bounds();
    EXPECT_TRUE(bounds.Contains(0.0, 0.0));
    EXPECT_TRUE(bounds.Contains(200.0, 200.0));
    EXPECT_TRUE(bounds.Contains(100.0, 100.0));
}

TEST(HeightField, BoundsContainsRejectsOutsidePoints) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    const TERCOM::Geo::GridBounds bounds = field.Bounds();
    EXPECT_FALSE(bounds.Contains(-0.1, 100.0));
    EXPECT_FALSE(bounds.Contains(100.0, -0.1));
    EXPECT_FALSE(bounds.Contains(200.1, 100.0));
    EXPECT_FALSE(bounds.Contains(100.0, 200.1));
}

TEST(HeightField, ElevationViaInterfaceMatchesConcrete) {
    const TERCOM::Geo::HeightField field = MakeGrid();
    const TERCOM::Geo::IElevationModel& model = field;
    const std::optional<double> value = model.ElevationAt(50.0, 50.0);
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, 30.0, 1e-9);
    const TERCOM::Geo::GridBounds bounds = model.Bounds();
    EXPECT_NEAR(bounds.max_east_m, 200.0, 1e-9);
}
