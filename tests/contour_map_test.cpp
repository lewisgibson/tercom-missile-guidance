#include <gtest/gtest.h>

#include "tercom/geo/elevation_model.h"
#include "tercom/geo/height_field.h"
#include "tercom/terrain/contour_map.h"

#include <cmath>
#include <vector>

namespace {

    // A flat 3x3 field: every node sits at 100 m. cell_size 50 m, origin at
    // (10, 20) ENU. Nodes span east [10, 110], north [20, 120].
    TERCOM::Geo::HeightField MakeFlatField() {
        std::vector<float> elevations(9, 100.0F);
        return TERCOM::Geo::HeightField(3, 3, 50.0, 10.0, 20.0, std::move(elevations));
    }

    // A varied 3x3 field with a clear gradient so roughness is strictly
    // positive over any multi-node region.
    TERCOM::Geo::HeightField MakeVariedField() {
        // Row-major, row 0 at the south. cell_size 50 m, origin (0, 0).
        std::vector<float> elevations = {0.0F, 10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F, 80.0F};
        return TERCOM::Geo::HeightField(3, 3, 50.0, 0.0, 0.0, std::move(elevations));
    }

    // A region that comfortably encloses every node of the 3x3 fields above.
    TERCOM::Geo::GridBounds WideRegion() {
        return TERCOM::Geo::GridBounds{-1000.0, -1000.0, 1000.0, 1000.0};
    }

} // namespace

TEST(ContourMap, FieldExposesUnderlyingHeightField) {
    TERCOM::Terrain::ContourMap map(MakeVariedField());
    const TERCOM::Geo::HeightField& field = map.Field();

    EXPECT_EQ(field.Cols(), 3u);
    EXPECT_EQ(field.Rows(), 3u);
    EXPECT_NEAR(field.CellSizeM(), 50.0, 1e-9);
    EXPECT_NEAR(field.OriginEastM(), 0.0, 1e-9);
    EXPECT_NEAR(field.OriginNorthM(), 0.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(field.NodeElevation(0, 0)), 0.0, 1e-6);
    EXPECT_NEAR(static_cast<double>(field.NodeElevation(2, 2)), 80.0, 1e-6);
}

TEST(ContourMap, ElevationModelInterfaceSamplesTerrain) {
    TERCOM::Terrain::ContourMap map(MakeVariedField());
    const TERCOM::Geo::IElevationModel& model = map.Elevation();

    // Exactly on node (0, 0).
    const auto corner = model.ElevationAt(0.0, 0.0);
    ASSERT_TRUE(corner.has_value());
    EXPECT_NEAR(*corner, 0.0, 1e-6);

    // Exactly on node (col 2, row 2) at east 100, north 100.
    const auto far = model.ElevationAt(100.0, 100.0);
    ASSERT_TRUE(far.has_value());
    EXPECT_NEAR(*far, 80.0, 1e-6);

    // Midpoint of the south edge between node (0,0)=0 and (1,0)=10 -> 5.
    const auto mid = model.ElevationAt(25.0, 0.0);
    ASSERT_TRUE(mid.has_value());
    EXPECT_NEAR(*mid, 5.0, 1e-6);

    // Outside the modelled area returns no value.
    EXPECT_FALSE(model.ElevationAt(-50.0, 0.0).has_value());
    EXPECT_FALSE(model.ElevationAt(0.0, 5000.0).has_value());
}

TEST(ContourMap, BoundsMatchUnderlyingField) {
    TERCOM::Terrain::ContourMap map(MakeFlatField());
    const TERCOM::Geo::GridBounds bounds = map.Bounds();

    // Origin (10, 20), 3 nodes per axis, 50 m spacing -> spans 100 m.
    EXPECT_NEAR(bounds.min_east_m, 10.0, 1e-9);
    EXPECT_NEAR(bounds.min_north_m, 20.0, 1e-9);
    EXPECT_NEAR(bounds.max_east_m, 110.0, 1e-9);
    EXPECT_NEAR(bounds.max_north_m, 120.0, 1e-9);

    // ContourMap::Bounds must agree with the field it wraps.
    const TERCOM::Geo::GridBounds field_bounds = map.Field().Bounds();
    EXPECT_NEAR(bounds.min_east_m, field_bounds.min_east_m, 1e-9);
    EXPECT_NEAR(bounds.min_north_m, field_bounds.min_north_m, 1e-9);
    EXPECT_NEAR(bounds.max_east_m, field_bounds.max_east_m, 1e-9);
    EXPECT_NEAR(bounds.max_north_m, field_bounds.max_north_m, 1e-9);
}

TEST(ContourMap, FlatFieldHasZeroRoughness) {
    TERCOM::Terrain::ContourMap map(MakeFlatField());
    // All nine nodes are equal, so the standard deviation is exactly zero.
    EXPECT_NEAR(map.TerrainRoughnessM(WideRegion()), 0.0, 1e-9);
}

TEST(ContourMap, VariedFieldHasPositiveRoughness) {
    TERCOM::Terrain::ContourMap map(MakeVariedField());
    EXPECT_GT(map.TerrainRoughnessM(WideRegion()), 0.0);
}

TEST(ContourMap, RoughnessMatchesPopulationStandardDeviation) {
    // A tiny 2x2 field with known elevations so the expected population
    // standard deviation can be computed by hand.
    // Row-major: row 0 (south) = {0, 10}, row 1 (north) = {20, 30}.
    // Mean = 15, mean of squares = (0 + 100 + 400 + 900) / 4 = 350,
    // variance = 350 - 225 = 125, std = sqrt(125) ~= 11.1803398875.
    std::vector<float> elevations = {0.0F, 10.0F, 20.0F, 30.0F};
    TERCOM::Geo::HeightField field(2, 2, 100.0, 0.0, 0.0, std::move(elevations));
    TERCOM::Terrain::ContourMap map(std::move(field));

    const double expected = std::sqrt(125.0);
    EXPECT_NEAR(map.TerrainRoughnessM(WideRegion()), expected, 1e-6);
}

TEST(ContourMap, RegionWithSingleNodeReturnsZero) {
    TERCOM::Terrain::ContourMap map(MakeVariedField());
    // Tightly bound the region around just node (0, 0) at east/north 0.
    // The next nodes sit at 50 m, so only one node falls inside.
    const TERCOM::Geo::GridBounds single{-1.0, -1.0, 1.0, 1.0};
    EXPECT_NEAR(map.TerrainRoughnessM(single), 0.0, 1e-9);
}

TEST(ContourMap, EmptyRegionReturnsZero) {
    TERCOM::Terrain::ContourMap map(MakeVariedField());
    // A region entirely outside the field encloses zero nodes -> count < 2.
    const TERCOM::Geo::GridBounds outside{1000.0, 1000.0, 2000.0, 2000.0};
    EXPECT_NEAR(map.TerrainRoughnessM(outside), 0.0, 1e-9);
}

TEST(ContourMap, SubRegionSelectsExpectedNodes) {
    // Restricting the region to the southern row of the varied field selects
    // nodes {0, 10, 20}: mean = 10, mean of squares = (0 + 100 + 400)/3 = 500/3,
    // variance = 500/3 - 100 = 200/3, std = sqrt(200/3) ~= 8.16496580928.
    TERCOM::Terrain::ContourMap map(MakeVariedField());
    // South row sits at north 0; the next row is at north 50. Bound north in
    // [-1, 1] to keep only the southern row, east wide to keep all columns.
    const TERCOM::Geo::GridBounds south_row{-1.0, -1.0, 1000.0, 1.0};

    const double expected = std::sqrt(200.0 / 3.0);
    EXPECT_NEAR(map.TerrainRoughnessM(south_row), expected, 1e-6);
}

TEST(ContourMap, WesternClipExcludesWestColumn) {
    // min_east_m = 50 lies east of the east=0 column, exercising the western clip
    // (east < region.min_east_m). The varied field's columns by easting are
    // east=0:{0,30,60}, east=50:{10,40,70}, east=100:{20,50,80}. Dropping the
    // east=0 column leaves {10,20,40,50,70,80}: mean 45, population variance 625,
    // standard deviation exactly 25. GridBounds order is {min_east, min_north,
    // max_east, max_north}.
    TERCOM::Terrain::ContourMap map(MakeVariedField());
    const TERCOM::Geo::GridBounds clip_west{50.0, -1000.0, 1000.0, 1000.0};
    EXPECT_NEAR(map.TerrainRoughnessM(clip_west), 25.0, 1e-6);
}
