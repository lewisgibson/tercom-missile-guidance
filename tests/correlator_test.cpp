#include <gtest/gtest.h>

#include "tercom/geo/height_field.h"
#include "tercom/geo/synthetic_terrain.h"
#include "tercom/math/linear.h"
#include "tercom/terrain/tercom_correlator.h"

#include <cmath>
#include <limits>
#include <vector>

namespace {

    using TERCOM::Geo::FractalTerrainConfiguration;
    using TERCOM::Geo::GenerateFractalTerrain;
    using TERCOM::Geo::HeightField;
    using TERCOM::Math::Vec3;
    using TERCOM::Terrain::TercomCorrelator;
    using TERCOM::Terrain::TercomCorrelatorConfiguration;
    using TERCOM::Terrain::TercomFix;
    using TERCOM::Terrain::TerrainProfileSample;

    // A rough reference map with plenty of relief so horizontal position is
    // observable. 400x400 nodes at 100 m spacing spans 0..39900 m on each axis.
    HeightField MakeReferenceMap() {
        FractalTerrainConfiguration parameters;
        parameters.cols = 400;
        parameters.rows = 400;
        parameters.cell_size_m = 100.0;
        parameters.origin_east_m = 0.0;
        parameters.origin_north_m = 0.0;
        parameters.base_elevation_m = 500.0;
        parameters.relief_m = 400.0;
        parameters.seed = 0xC0FFEEULL;
        return GenerateFractalTerrain(parameters);
    }

    // Build a measured profile of `count` samples spaced `spacing_m` apart along a
    // diagonal line starting at `start`. Each sample's stored position is the
    // INS-estimated (truth - true_offset) position; its measured elevation is the
    // reference elevation at the corresponding truth position
    // (ins_position + true_offset). Recovering true_offset is the correlator's job.
    std::vector<TerrainProfileSample> BuildProfile(const HeightField& map, const Vec3& start, const Vec3& true_offset, std::size_t count, double spacing_m) {
        std::vector<TerrainProfileSample> profile;
        profile.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const double along = static_cast<double>(i) * spacing_m;
            const Vec3 ins_position{start.x + along, start.y + along, 0.0};
            const Vec3 truth{ins_position.x + true_offset.x, ins_position.y + true_offset.y, 0.0};
            const auto reference = map.ElevationAt(truth.x, truth.y);
            EXPECT_TRUE(reference.has_value());

            TerrainProfileSample sample;
            sample.position_enu_m = ins_position;
            sample.measured_elevation_m = reference.value_or(0.0);
            profile.push_back(sample);
        }
        return profile;
    }

    constexpr double kKnownOffsetEast = 80.0;
    constexpr double kKnownOffsetNorth = -50.0;

} // namespace

TEST(TercomCorrelator, RecoversKnownOffsetWithZeroNoise) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    // Keep the path well inside the map so the shifted lookups stay in coverage.
    const Vec3 start{15000.0, 15000.0, 0.0};
    const std::vector<TerrainProfileSample> profile = BuildProfile(map, start, true_offset, 60u, 100.0);

    const Vec3 predicted{start.x + 30.0 * 100.0, start.y + 30.0 * 100.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    ASSERT_TRUE(fix.accepted) << fix.rejection_reason;
    EXPECT_TRUE(fix.rejection_reason.empty());

    // The grid resolves offsets to the search step (25 m); a couple of steps is a
    // generous tolerance for the recovered shift.
    EXPECT_NEAR(fix.offset_enu_m.x, kKnownOffsetEast, 50.0);
    EXPECT_NEAR(fix.offset_enu_m.y, kKnownOffsetNorth, 50.0);
    EXPECT_NEAR(fix.offset_enu_m.z, 0.0, 1e-9);

    // The corrected position is the prediction plus the recovered offset.
    EXPECT_NEAR(fix.position_enu_m.x, predicted.x + fix.offset_enu_m.x, 1e-9);
    EXPECT_NEAR(fix.position_enu_m.y, predicted.y + fix.offset_enu_m.y, 1e-9);

    EXPECT_GT(fix.confidence, 0.0);
    EXPECT_LE(fix.confidence, 1.0);
    EXPECT_GE(fix.mad_m, 0.0);
    EXPECT_GT(fix.terrain_roughness_m, 0.0);

    // The covariance diagonal is calibrated from the achieved residual:
    // sigma = clamp(min_fix_std + residual_sigma_gain * mad, min, max), written to
    // both axes (isotropic). Tie the assertion to that formula, not just to itself.
    EXPECT_GT(fix.covariance(0, 0), 0.0);
    EXPECT_GT(fix.covariance(1, 1), 0.0);
    const TercomCorrelatorConfiguration configuration;
    const double sigma = std::fmin(
        std::fmax(configuration.min_fix_std_m + (configuration.residual_sigma_gain * fix.mad_m), configuration.min_fix_std_m), configuration.max_fix_std_m);
    EXPECT_NEAR(fix.covariance(0, 0), sigma * sigma, 1e-6);
    EXPECT_NEAR(fix.covariance(1, 1), sigma * sigma, 1e-6);
}

TEST(TercomCorrelator, ZeroNoiseDrivesMadVeryLow) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    const Vec3 start{15000.0, 15000.0, 0.0};
    const std::vector<TerrainProfileSample> profile = BuildProfile(map, start, true_offset, 60u, 100.0);

    const Vec3 predicted{start.x + 30.0 * 100.0, start.y + 30.0 * 100.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    ASSERT_TRUE(fix.accepted) << fix.rejection_reason;
    // With no measurement noise the residual is just the 25 m grid quantization and
    // bilinear-interpolation error: sub-metre in practice, far below both the
    // acceptance threshold (60 m) and the terrain roughness (tens of metres).
    EXPECT_LT(fix.mad_m, 5.0);
}

TEST(TercomCorrelator, FlatProfileIsRejectedForLowRoughness) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    // All measured elevations identical: profile roughness is zero, below the
    // observability gate, so the fix must be rejected.
    std::vector<TerrainProfileSample> profile;
    profile.reserve(40u);
    for (std::size_t i = 0; i < 40u; ++i) {
        TerrainProfileSample sample;
        sample.position_enu_m = Vec3{15000.0 + static_cast<double>(i) * 100.0, 15000.0, 0.0};
        sample.measured_elevation_m = 500.0;
        profile.push_back(sample);
    }

    const Vec3 predicted{15000.0, 15000.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    EXPECT_FALSE(fix.accepted);
    EXPECT_FALSE(fix.rejection_reason.empty());
    EXPECT_EQ(fix.rejection_reason, "terrain too flat to be observable");
    EXPECT_NEAR(fix.terrain_roughness_m, 0.0, 1e-9);
}

TEST(TercomCorrelator, RoughnessJustBelowGateIsRejected) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    // Tiny alternating relief gives a roughness well under the 8 m default gate.
    std::vector<TerrainProfileSample> profile;
    profile.reserve(40u);
    for (std::size_t i = 0; i < 40u; ++i) {
        TerrainProfileSample sample;
        sample.position_enu_m = Vec3{15000.0 + static_cast<double>(i) * 100.0, 15000.0, 0.0};
        sample.measured_elevation_m = 500.0 + ((i % 2 == 0) ? 1.0 : -1.0);
        profile.push_back(sample);
    }

    const Vec3 predicted{15000.0, 15000.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    EXPECT_FALSE(fix.accepted);
    EXPECT_EQ(fix.rejection_reason, "terrain too flat to be observable");
    EXPECT_LT(fix.terrain_roughness_m, 8.0);
}

TEST(TercomCorrelator, TooFewSamplesIsRejected) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    const Vec3 start{15000.0, 15000.0, 0.0};
    // Default min_samples is 16; supply fewer.
    const std::vector<TerrainProfileSample> profile = BuildProfile(map, start, true_offset, 8u, 100.0);
    ASSERT_LT(profile.size(), 16u);

    const Vec3 predicted{start.x, start.y, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    EXPECT_FALSE(fix.accepted);
    EXPECT_EQ(fix.rejection_reason, "insufficient profile samples");
}

TEST(TercomCorrelator, EmptyProfileIsRejected) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    const std::vector<TerrainProfileSample> profile;
    const Vec3 predicted{15000.0, 15000.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    EXPECT_FALSE(fix.accepted);
    EXPECT_EQ(fix.rejection_reason, "insufficient profile samples");
}

TEST(TercomCorrelator, EmptyProfileIsRejectedEvenWithZeroMinSamples) {
    // With min_samples deliberately set to 0 the size gate cannot fire, so the
    // explicit empty-profile guard must still reject before the roughness
    // computation divides by the sample count (which would otherwise yield NaN).
    const HeightField map = MakeReferenceMap();
    TercomCorrelatorConfiguration configuration;
    configuration.min_samples = 0;
    const TercomCorrelator correlator(map, configuration);

    const std::vector<TerrainProfileSample> profile;
    const Vec3 predicted{15000.0, 15000.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    EXPECT_FALSE(fix.accepted);
    EXPECT_EQ(fix.rejection_reason, "insufficient profile samples");
}

TEST(TercomCorrelator, MinSamplesGateBoundary) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    const Vec3 start{15000.0, 15000.0, 0.0};
    const Vec3 predicted{start.x, start.y, 0.0};

    // One below the default minimum (16) is rejected specifically for sample count.
    const std::vector<TerrainProfileSample> below = BuildProfile(map, start, true_offset, 15u, 100.0);
    const TercomFix below_fix = correlator.Correlate(below, predicted);
    EXPECT_FALSE(below_fix.accepted);
    EXPECT_EQ(below_fix.rejection_reason, "insufficient profile samples");

    // At exactly the minimum the sample-count gate passes: the fix may still be
    // rejected downstream (roughness, residual) but no longer for too few samples.
    const std::vector<TerrainProfileSample> at = BuildProfile(map, start, true_offset, 16u, 100.0);
    const TercomFix at_fix = correlator.Correlate(at, predicted);
    EXPECT_NE(at_fix.rejection_reason, "insufficient profile samples");
}

TEST(TercomCorrelator, ProfileOutsideMapHasNoCoverage) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    // Place samples far outside the modelled area so no shifted lookup lands in
    // coverage. Give them relief so the roughness gate is cleared first.
    std::vector<TerrainProfileSample> profile;
    profile.reserve(40u);
    for (std::size_t i = 0; i < 40u; ++i) {
        TerrainProfileSample sample;
        sample.position_enu_m = Vec3{-100000.0 - static_cast<double>(i) * 100.0, -100000.0, 0.0};
        sample.measured_elevation_m = 500.0 + ((i % 2 == 0) ? 20.0 : -20.0);
        profile.push_back(sample);
    }

    const Vec3 predicted{-100000.0, -100000.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    EXPECT_FALSE(fix.accepted);
    EXPECT_EQ(fix.rejection_reason, "no reference-map coverage under the profile");
}

TEST(TercomCorrelator, SmallPriorConfinesSearchButStillRecoversOffset) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    const Vec3 start{15000.0, 15000.0, 0.0};
    const std::vector<TerrainProfileSample> profile = BuildProfile(map, start, true_offset, 60u, 100.0);

    const Vec3 predicted{start.x + 30.0 * 100.0, start.y + 30.0 * 100.0, 0.0};

    // A small prior (1 m) clamps the half-extent to the floor (min_search_radius
    // 150 m). The known offset magnitude is under 100 m, so it is still inside
    // the confined search and must be recovered.
    const double small_prior = 1.0;
    const TercomFix fix = correlator.Correlate(profile, predicted, small_prior);

    ASSERT_TRUE(fix.accepted) << fix.rejection_reason;
    EXPECT_NEAR(fix.offset_enu_m.x, kKnownOffsetEast, 50.0);
    EXPECT_NEAR(fix.offset_enu_m.y, kKnownOffsetNorth, 50.0);
    EXPECT_GT(fix.confidence, 0.0);
}

TEST(TercomCorrelator, DefaultInfinitePriorSearchesFullRadius) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    const Vec3 start{15000.0, 15000.0, 0.0};
    const std::vector<TerrainProfileSample> profile = BuildProfile(map, start, true_offset, 60u, 100.0);

    const Vec3 predicted{start.x + 30.0 * 100.0, start.y + 30.0 * 100.0, 0.0};

    // Default prior is infinite, which leaves the search at the full configured
    // radius. The recovered offset must match the explicit-infinity call.
    const TercomFix fix_default = correlator.Correlate(profile, predicted);
    const TercomFix fix_inf = correlator.Correlate(profile, predicted, std::numeric_limits<double>::infinity());

    ASSERT_TRUE(fix_default.accepted) << fix_default.rejection_reason;
    ASSERT_TRUE(fix_inf.accepted) << fix_inf.rejection_reason;
    EXPECT_NEAR(fix_default.offset_enu_m.x, fix_inf.offset_enu_m.x, 1e-9);
    EXPECT_NEAR(fix_default.offset_enu_m.y, fix_inf.offset_enu_m.y, 1e-9);
    EXPECT_NEAR(fix_default.offset_enu_m.x, kKnownOffsetEast, 50.0);
    EXPECT_NEAR(fix_default.offset_enu_m.y, kKnownOffsetNorth, 50.0);
}

TEST(TercomCorrelator, ZeroSearchUncertaintyScaleIgnoresPrior) {
    const HeightField map = MakeReferenceMap();
    TercomCorrelatorConfiguration configuration;
    // Disable the adaptive narrowing: even with a finite prior the full radius
    // is used.
    configuration.search_uncertainty_scale = 0.0;
    const TercomCorrelator correlator(map, configuration);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    const Vec3 start{15000.0, 15000.0, 0.0};
    const std::vector<TerrainProfileSample> profile = BuildProfile(map, start, true_offset, 60u, 100.0);

    const Vec3 predicted{start.x + 30.0 * 100.0, start.y + 30.0 * 100.0, 0.0};

    const TercomFix fix_tiny_prior = correlator.Correlate(profile, predicted, 0.5);
    const TercomFix fix_inf = correlator.Correlate(profile, predicted, std::numeric_limits<double>::infinity());

    ASSERT_TRUE(fix_tiny_prior.accepted) << fix_tiny_prior.rejection_reason;
    ASSERT_TRUE(fix_inf.accepted) << fix_inf.rejection_reason;
    // The prior is ignored, so both searches cover the same radius and agree.
    EXPECT_NEAR(fix_tiny_prior.offset_enu_m.x, fix_inf.offset_enu_m.x, 1e-9);
    EXPECT_NEAR(fix_tiny_prior.offset_enu_m.y, fix_inf.offset_enu_m.y, 1e-9);
}

TEST(TercomCorrelator, CovarianceRespectsConfiguredFloor) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelatorConfiguration configuration;
    const TercomCorrelator correlator(map, configuration);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    const Vec3 start{15000.0, 15000.0, 0.0};
    const std::vector<TerrainProfileSample> profile = BuildProfile(map, start, true_offset, 60u, 100.0);

    const Vec3 predicted{start.x + 30.0 * 100.0, start.y + 30.0 * 100.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    ASSERT_TRUE(fix.accepted) << fix.rejection_reason;
    // sigma = clamp(min_fix_std + residual_sigma_gain * mad, min_fix_std, max_fix_std).
    // So the one-sigma is at least the floor and at most the ceiling.
    const double sigma = std::sqrt(fix.covariance(0, 0));
    EXPECT_GE(sigma, configuration.min_fix_std_m - 1e-6);
    EXPECT_LE(sigma, configuration.max_fix_std_m + 1e-6);
}

TEST(TercomCorrelator, HighMadResidualIsRejected) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    // A rough profile whose elevations bear no relation to the terrain: even the
    // best offset leaves a large residual, exceeding max_acceptable_mad_m, so the
    // fix is rejected on residual (or ambiguity), never accepted.
    std::vector<TerrainProfileSample> profile;
    profile.reserve(60u);
    for (std::size_t i = 0; i < 60u; ++i) {
        const double phase = static_cast<double>(i);
        TerrainProfileSample sample;
        sample.position_enu_m = Vec3{15000.0 + phase * 100.0, 15000.0 + phase * 100.0, 0.0};
        // Large-amplitude swings far from the reference terrain band.
        sample.measured_elevation_m = 500.0 + 5000.0 * std::sin(phase * 1.3);
        profile.push_back(sample);
    }

    const Vec3 predicted{15000.0, 15000.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    EXPECT_FALSE(fix.accepted);
    EXPECT_FALSE(fix.rejection_reason.empty());
    // The roughness gate is cleared (large swings), so rejection is on the match
    // quality, not observability.
    EXPECT_NE(fix.rejection_reason, "terrain too flat to be observable");
    EXPECT_NE(fix.rejection_reason, "insufficient profile samples");
}

TEST(TercomCorrelator, HighResidualIsRejectedAboveThreshold) {
    const HeightField map = MakeReferenceMap();
    TercomCorrelatorConfiguration configuration;
    // Open the confidence gate wide so it never fires, isolating the residual gate;
    // the residual threshold stays at its low default so a poor match is rejected on
    // residual specifically, not on ambiguity.
    configuration.min_confidence = 0.0;
    const TercomCorrelator correlator(map, configuration);

    // A rough profile (clears the roughness gate) whose elevations bear no relation to
    // the reference band: even the best offset leaves a large mean-absolute-difference.
    std::vector<TerrainProfileSample> profile;
    profile.reserve(60u);
    for (std::size_t i = 0; i < 60u; ++i) {
        const double phase = static_cast<double>(i);
        TerrainProfileSample sample;
        sample.position_enu_m = Vec3{15000.0 + (phase * 100.0), 15000.0 + (phase * 100.0), 0.0};
        sample.measured_elevation_m = 500.0 + (5000.0 * std::sin(phase * 1.3));
        profile.push_back(sample);
    }

    const Vec3 predicted{15000.0, 15000.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    EXPECT_FALSE(fix.accepted);
    EXPECT_EQ(fix.rejection_reason, "correlation residual above threshold");
    // Prove it was the residual gate, not the confidence gate, that fired.
    EXPECT_GT(fix.mad_m, configuration.max_acceptable_mad_m);
}

TEST(TercomCorrelator, AcceptedFixReportsRoughnessNearProfileStd) {
    const HeightField map = MakeReferenceMap();
    const TercomCorrelator correlator(map);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    const Vec3 start{15000.0, 15000.0, 0.0};
    const std::vector<TerrainProfileSample> profile = BuildProfile(map, start, true_offset, 60u, 100.0);

    const Vec3 predicted{start.x + 30.0 * 100.0, start.y + 30.0 * 100.0, 0.0};
    const TercomFix fix = correlator.Correlate(profile, predicted);

    ASSERT_TRUE(fix.accepted) << fix.rejection_reason;

    // Independently compute the profile standard deviation and compare.
    double sum = 0.0;
    for (const auto& sample : profile) {
        sum += sample.measured_elevation_m;
    }
    const double mean = sum / static_cast<double>(profile.size());
    double variance = 0.0;
    for (const auto& sample : profile) {
        const double delta = sample.measured_elevation_m - mean;
        variance += delta * delta;
    }
    variance /= static_cast<double>(profile.size());
    const double expected_roughness = std::sqrt(variance);

    EXPECT_NEAR(fix.terrain_roughness_m, expected_roughness, 1e-6);
    EXPECT_GE(fix.terrain_roughness_m, 8.0);
}

TEST(TercomCorrelator, MissingCompetitorTreatsSeparationAsZero) {
    const HeightField map = MakeReferenceMap();
    TercomCorrelatorConfiguration configuration;
    // An exclusion neighbourhood larger than the whole search grid means no scored
    // cell ever lands outside it, so no competitor can be assessed and competitor_mad
    // stays infinite. The separation term is then forced to zero and the confidence
    // comes from contrast alone (0.5 * contrast).
    configuration.competitor_exclusion_m = 1.0e6;
    const TercomCorrelator correlator(map, configuration);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    const Vec3 start{15000.0, 15000.0, 0.0};
    const std::vector<TerrainProfileSample> profile = BuildProfile(map, start, true_offset, 60u, 100.0);
    const Vec3 predicted{start.x + (30.0 * 100.0), start.y + (30.0 * 100.0), 0.0};

    const TercomFix fix = correlator.Correlate(profile, predicted);

    ASSERT_TRUE(fix.accepted) << fix.rejection_reason;
    // With no assessable competitor the fix earns its confidence from contrast alone,
    // so it cannot exceed 0.5. A regression that defaulted the missing competitor to
    // separation = 1.0 (trusting an unverifiable fix) would push confidence above 0.5.
    EXPECT_GT(fix.confidence, 0.0);
    EXPECT_LE(fix.confidence, 0.5 + 1e-9);
}

TEST(TercomCorrelator, LowConfidenceIsRejectedAsAmbiguous) {
    const HeightField map = MakeReferenceMap();
    TercomCorrelatorConfiguration configuration;
    // Open the residual gate wide so any match passes it, isolating the confidence
    // gate; then demand near-perfect confidence so any ambiguity rejects the fix.
    configuration.max_acceptable_mad_m = 1.0e9;
    configuration.min_confidence = 0.99;
    const TercomCorrelator correlator(map, configuration);

    const Vec3 true_offset{kKnownOffsetEast, kKnownOffsetNorth, 0.0};
    const Vec3 start{15000.0, 15000.0, 0.0};
    const std::vector<TerrainProfileSample> profile = BuildProfile(map, start, true_offset, 60u, 100.0);
    const Vec3 predicted{start.x + (30.0 * 100.0), start.y + (30.0 * 100.0), 0.0};

    const TercomFix fix = correlator.Correlate(profile, predicted);

    EXPECT_FALSE(fix.accepted);
    EXPECT_EQ(fix.rejection_reason, "ambiguous correlation minimum");
    // Prove it was the confidence gate, not the residual gate, that fired.
    EXPECT_LT(fix.confidence, configuration.min_confidence);
    EXPECT_LE(fix.mad_m, configuration.max_acceptable_mad_m);
}
