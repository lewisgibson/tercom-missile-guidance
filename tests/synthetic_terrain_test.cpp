#include <gtest/gtest.h>

#include "tercom/geo/height_field.h"
#include "tercom/geo/synthetic_terrain.h"

#include <cmath>
#include <cstddef>
#include <vector>

namespace {

    // A small, fast-to-generate field is enough to exercise the generator.
    TERCOM::Geo::FractalTerrainConfiguration MakeParameters() {
        TERCOM::Geo::FractalTerrainConfiguration parameters;
        parameters.cols = 16;
        parameters.rows = 12;
        parameters.cell_size_m = 50.0;
        parameters.origin_east_m = 1000.0;
        parameters.origin_north_m = -2000.0;
        parameters.base_elevation_m = 400.0;
        parameters.relief_m = 600.0;
        parameters.feature_wavelength_m = 4000.0;
        parameters.octaves = 5;
        parameters.lacunarity = 2.0;
        parameters.gain = 0.5;
        parameters.seed = 0x1234ABCDULL;
        return parameters;
    }

} // namespace

// Same parameters (same seed) must produce byte-identical elevation data.
TEST(SyntheticTerrain, DeterministicForSameSeed) {
    const TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();

    const TERCOM::Geo::HeightField first = TERCOM::Geo::GenerateFractalTerrain(parameters);
    const TERCOM::Geo::HeightField second = TERCOM::Geo::GenerateFractalTerrain(parameters);

    const std::vector<float>& a = first.Data();
    const std::vector<float>& b = second.Data();

    ASSERT_EQ(a.size(), b.size());
    ASSERT_EQ(a.size(), parameters.cols * parameters.rows);

    for (std::size_t i = 0; i < a.size(); ++i) {
        // Byte-identical: an exact bit-for-bit comparison, not a tolerance.
        EXPECT_EQ(a[i], b[i]) << "mismatch at index " << i;
    }
}

// A different seed must change at least one node.
TEST(SyntheticTerrain, DifferentSeedProducesDifferentField) {
    TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();
    const TERCOM::Geo::HeightField baseline = TERCOM::Geo::GenerateFractalTerrain(parameters);

    parameters.seed = 0xFEEDFACEULL;
    const TERCOM::Geo::HeightField altered = TERCOM::Geo::GenerateFractalTerrain(parameters);

    const std::vector<float>& a = baseline.Data();
    const std::vector<float>& b = altered.Data();

    ASSERT_EQ(a.size(), b.size());

    bool any_differ = false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            any_differ = true;
            break;
        }
    }
    EXPECT_TRUE(any_differ);
}

// The generated HeightField geometry must match the requested parameters.
TEST(SyntheticTerrain, GeometryMatchesParameters) {
    const TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();
    const TERCOM::Geo::HeightField field = TERCOM::Geo::GenerateFractalTerrain(parameters);

    EXPECT_EQ(field.Cols(), parameters.cols);
    EXPECT_EQ(field.Rows(), parameters.rows);
    EXPECT_NEAR(field.CellSizeM(), parameters.cell_size_m, 1e-9);
    EXPECT_NEAR(field.OriginEastM(), parameters.origin_east_m, 1e-9);
    EXPECT_NEAR(field.OriginNorthM(), parameters.origin_north_m, 1e-9);
    EXPECT_EQ(field.Data().size(), parameters.cols * parameters.rows);
}

// Elevations must be finite and stay roughly within base +- relief, with margin.
TEST(SyntheticTerrain, ElevationsFiniteAndWithinRelief) {
    const TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();
    const TERCOM::Geo::HeightField field = TERCOM::Geo::GenerateFractalTerrain(parameters);

    // The fBm output is normalised to approximately [-1, 1], then mapped to
    // base +- relief/2. Allow a generous margin around the full +- relief band
    // to absorb noise that can briefly exceed the nominal half-relief envelope.
    const double margin = parameters.relief_m;
    const double lower = (parameters.base_elevation_m - parameters.relief_m) - margin;
    const double upper = (parameters.base_elevation_m + parameters.relief_m) + margin;

    const std::vector<float>& data = field.Data();
    ASSERT_FALSE(data.empty());

    for (std::size_t i = 0; i < data.size(); ++i) {
        const double value = static_cast<double>(data[i]);
        EXPECT_TRUE(std::isfinite(value)) << "non-finite elevation at index " << i;
        EXPECT_GE(value, lower) << "elevation below band at index " << i;
        EXPECT_LE(value, upper) << "elevation above band at index " << i;
    }
}

// NodeElevation must agree with the row-major Data() backing store.
TEST(SyntheticTerrain, NodeElevationMatchesData) {
    const TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();
    const TERCOM::Geo::HeightField field = TERCOM::Geo::GenerateFractalTerrain(parameters);

    const std::vector<float>& data = field.Data();
    for (std::size_t row = 0; row < parameters.rows; ++row) {
        for (std::size_t col = 0; col < parameters.cols; ++col) {
            const float node = field.NodeElevation(col, row);
            const float stored = data[(row * parameters.cols) + col];
            EXPECT_EQ(node, stored) << "node mismatch at col " << col << " row " << row;
        }
    }
}

// octaves < 1 must be rejected.
TEST(SyntheticTerrain, ZeroOctavesThrows) {
    TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();
    parameters.octaves = 0;
    EXPECT_THROW(TERCOM::Geo::GenerateFractalTerrain(parameters), std::invalid_argument);
}

// Negative octaves must also be rejected.
TEST(SyntheticTerrain, NegativeOctavesThrows) {
    TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();
    parameters.octaves = -3;
    EXPECT_THROW(TERCOM::Geo::GenerateFractalTerrain(parameters), std::invalid_argument);
}

// A single octave is valid (boundary of the octave check).
TEST(SyntheticTerrain, SingleOctaveIsValid) {
    TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();
    parameters.octaves = 1;
    const TERCOM::Geo::HeightField field = TERCOM::Geo::GenerateFractalTerrain(parameters);
    EXPECT_EQ(field.Data().size(), parameters.cols * parameters.rows);
}

// feature_wavelength_m == 0 must be rejected.
TEST(SyntheticTerrain, ZeroWavelengthThrows) {
    TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();
    parameters.feature_wavelength_m = 0.0;
    EXPECT_THROW(TERCOM::Geo::GenerateFractalTerrain(parameters), std::invalid_argument);
}

// Negative feature_wavelength_m must be rejected.
TEST(SyntheticTerrain, NegativeWavelengthThrows) {
    TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();
    parameters.feature_wavelength_m = -100.0;
    EXPECT_THROW(TERCOM::Geo::GenerateFractalTerrain(parameters), std::invalid_argument);
}

// A positive wavelength at the validity boundary is accepted.
TEST(SyntheticTerrain, SmallPositiveWavelengthIsValid) {
    TERCOM::Geo::FractalTerrainConfiguration parameters = MakeParameters();
    parameters.feature_wavelength_m = 1.0;
    const TERCOM::Geo::HeightField field = TERCOM::Geo::GenerateFractalTerrain(parameters);
    EXPECT_EQ(field.Data().size(), parameters.cols * parameters.rows);

    const std::vector<float>& data = field.Data();
    for (std::size_t i = 0; i < data.size(); ++i) {
        EXPECT_TRUE(std::isfinite(static_cast<double>(data[i]))) << "non-finite at index " << i;
    }
}

// Default-constructed parameters (the documented defaults) must generate cleanly
// and honour their geometry. Uses a small grid to keep the test fast.
TEST(SyntheticTerrain, DefaultParamsGenerateValidField) {
    TERCOM::Geo::FractalTerrainConfiguration parameters; // documented defaults.
    parameters.cols = 8;
    parameters.rows = 8;
    const TERCOM::Geo::HeightField field = TERCOM::Geo::GenerateFractalTerrain(parameters);

    EXPECT_EQ(field.Cols(), 8u);
    EXPECT_EQ(field.Rows(), 8u);
    EXPECT_EQ(field.Data().size(), 64u);

    const std::vector<float>& data = field.Data();
    const double lower = (parameters.base_elevation_m - parameters.relief_m) - parameters.relief_m;
    const double upper = (parameters.base_elevation_m + parameters.relief_m) + parameters.relief_m;
    for (std::size_t i = 0; i < data.size(); ++i) {
        const double value = static_cast<double>(data[i]);
        EXPECT_TRUE(std::isfinite(value)) << "non-finite at index " << i;
        EXPECT_GE(value, lower);
        EXPECT_LE(value, upper);
    }
}
