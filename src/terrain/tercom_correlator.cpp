#include "tercom/terrain/tercom_correlator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace TERCOM { namespace Terrain {

    using Math::Vec3;

    TercomCorrelator::TercomCorrelator(const Geo::IElevationModel& reference_map, const TercomCorrelatorConfiguration& configuration)
        : reference_map_(reference_map),
          configuration_(configuration) {
    }

    namespace {
        // Standard deviation of the measured strip's heights, in metres. This is
        // the relief the strip actually carries: a profile that is nearly flat
        // (low std) cannot tell one place on the map from another, so the caller
        // uses this as an observability gate before attempting a fix.
        double ProfileRoughness(const std::vector<TerrainProfileSample>& profile) {
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
            // Guard the square root against a tiny negative variance from rounding.
            return std::sqrt(variance > 0.0 ? variance : 0.0);
        }
    } // namespace

    TercomFix TercomCorrelator::Correlate(const std::vector<TerrainProfileSample>& profile, const Vec3& predicted_position_enu_m, double prior_horizontal_std_m) const {
        TercomFix fix;
        fix.position_enu_m = predicted_position_enu_m;

        // Reject before ProfileRoughness, which divides by the sample count: an empty
        // profile must always fail here even if min_samples were configured to 0.
        if (profile.empty() || profile.size() < configuration_.min_samples) {
            fix.rejection_reason = "insufficient profile samples";
            return fix;
        }

        fix.terrain_roughness_m = ProfileRoughness(profile);
        if (fix.terrain_roughness_m < configuration_.min_terrain_roughness_m) {
            fix.rejection_reason = "terrain too flat to be observable";
            return fix;
        }

        // Confine the offset search to the INS uncertainty ellipse. A wide search
        // over the whole map admits false locks onto distant look-alike terrain;
        // bounding it to a few sigma of the predicted error keeps only plausible
        // offsets. The search widens automatically while the estimate is uncertain
        // (initial alignment) and tightens as the filter converges.
        double search_radius = configuration_.max_search_radius_m;
        if (std::isfinite(prior_horizontal_std_m) && configuration_.search_uncertainty_scale > 0.0) {
            search_radius = std::clamp(configuration_.search_uncertainty_scale * prior_horizontal_std_m, configuration_.min_search_radius_m, configuration_.max_search_radius_m);
        }
        // Number of grid cells from the centre to the edge of the search, so the
        // offset index runs over [-steps, steps] in each axis.
        const auto steps = static_cast<int>(std::lround(search_radius / configuration_.search_step_m));
        // An offset is only scored if at least half the strip lands on mapped
        // terrain; otherwise a near-edge offset could win on a handful of samples.
        const auto min_coverage = profile.size() / 2;

        // Dense (2*steps+1)-by-(2*steps+1) grid of MAD scores, one per candidate
        // offset, stored row-major and indexed by the shifted offset indices below.
        // Pre-filled with infinity so unscored cells never look like a good match.
        const std::size_t side = (2 * static_cast<std::size_t>(steps)) + 1;
        std::vector<double> mad_grid(side * side, std::numeric_limits<double>::infinity());
        double best_mad = std::numeric_limits<double>::infinity();
        double mad_sum = 0.0;
        std::size_t evaluated = 0;
        int best_east_index = 0;
        int best_north_index = 0;

        // Slide the measured strip over every offset and score it by mean absolute
        // difference (MAD) against the reference map. This is the correlator's hot
        // loop; link-time optimization inlines the elevation lookup into it.
        for (int east_index = -steps; east_index <= steps; ++east_index) {
            const double offset_east = static_cast<double>(east_index) * configuration_.search_step_m;
            for (int north_index = -steps; north_index <= steps; ++north_index) {
                const double offset_north = static_cast<double>(north_index) * configuration_.search_step_m;

                // Sum the absolute height error over the strip at this offset:
                // shift each sample's recorded position by the candidate offset,
                // look up the map there, and accumulate |measured - reference|.
                // Samples that fall off the map are skipped, not penalised.
                double absolute_difference = 0.0;
                std::size_t covered = 0;
                for (const auto& sample : profile) {
                    const std::optional<double> reference = reference_map_.ElevationAt(sample.position_enu_m.x + offset_east, sample.position_enu_m.y + offset_north);
                    if (!reference.has_value()) {
                        continue;
                    }
                    absolute_difference += std::fabs(sample.measured_elevation_m - *reference);
                    ++covered;
                }

                if (covered < min_coverage) {
                    continue;
                }

                // Mean absolute difference: average error per covered sample, so
                // offsets with different coverage are scored on the same footing.
                const double mad = absolute_difference / static_cast<double>(covered);
                // Map the signed offset indices to a non-negative row-major slot by
                // shifting them by steps (so -steps maps to 0, +steps to 2*steps).
                const std::size_t grid_index = (static_cast<std::size_t>(east_index + steps) * side) + static_cast<std::size_t>(north_index + steps);
                mad_grid[grid_index] = mad;
                mad_sum += mad;
                ++evaluated;

                if (mad < best_mad) {
                    best_mad = mad;
                    best_east_index = east_index;
                    best_north_index = north_index;
                }
            }
        }

        if (evaluated == 0) {
            fix.rejection_reason = "no reference-map coverage under the profile";
            return fix;
        }

        // Best competing minimum outside a neighbourhood of the best offset. A
        // distant rival of similar depth means the strip matches the terrain in
        // more than one place: the fix is ambiguous and must not be trusted.
        // Exclusion radius converted from metres to grid cells, at least one cell
        // so the winning cell itself is always excluded.
        const int exclusion = std::max(1, static_cast<int>(std::lround(configuration_.competitor_exclusion_m / configuration_.search_step_m)));
        double competitor_mad = std::numeric_limits<double>::infinity();
        for (int east_index = -steps; east_index <= steps; ++east_index) {
            for (int north_index = -steps; north_index <= steps; ++north_index) {
                if (std::abs(east_index - best_east_index) <= exclusion && std::abs(north_index - best_north_index) <= exclusion) {
                    continue;
                }
                const std::size_t grid_index = (static_cast<std::size_t>(east_index + steps) * side) + static_cast<std::size_t>(north_index + steps);
                competitor_mad = std::min(competitor_mad, mad_grid[grid_index]);
            }
        }

        const double mean_mad = mad_sum / static_cast<double>(evaluated);
        fix.mad_m = best_mad;

        // Confidence blends how far the best match sits below the average residual
        // (contrast) with how cleanly it beats its best distant rival (separation).
        // Contrast is the fractional drop of the best score below the mean score,
        // in 0..1: near 0 when the whole grid scores alike (a shallow, featureless
        // minimum), near 1 when the best offset stands out sharply.
        const double contrast = (mean_mad > 0.0) ? std::clamp((mean_mad - best_mad) / mean_mad, 0.0, 1.0) : 0.0;
        // Separation is the gap to the best distant rival, normalised so it is
        // near 0 when a rival nearly ties the winner (ambiguous) and approaches 1
        // when the winner is far better. The sensor_sigma_m term in the denominator
        // keeps a small gap from looking decisive when both scores are already low.
        // If no rival outside the exclusion neighbourhood could be assessed, the
        // ambiguity is unknown, not absent: treat separation as zero so the fix must
        // earn its confidence from contrast alone rather than being trusted blindly.
        const double separation = std::isfinite(competitor_mad) ? std::clamp((competitor_mad - best_mad) / (competitor_mad + configuration_.sensor_sigma_m), 0.0, 1.0) : 0.0;
        // Equal weighting: both a sharp minimum and a clear winner are required for
        // full confidence, so a fix that has one but not the other lands mid-scale.
        fix.confidence = std::clamp((0.5 * contrast) + (0.5 * separation), 0.0, 1.0);

        // Covariance calibrated to the achieved accuracy: the best-match residual
        // tracks the fix error closely, so the one-sigma scales with it. Confidence
        // governs acceptance below, not the covariance. The matrix is diagonal with
        // equal east and north variance (sigma^2): the search treats both axes the
        // same, so the reported uncertainty is isotropic.
        const double sigma = std::clamp(configuration_.min_fix_std_m + (configuration_.residual_sigma_gain * best_mad), configuration_.min_fix_std_m, configuration_.max_fix_std_m);
        fix.covariance(0, 0) = sigma * sigma;
        fix.covariance(1, 1) = sigma * sigma;

        // The winning offset in metres: the horizontal shift that best aligned the
        // strip with the map. Up component is zero; only the horizontal fix matters.
        const Vec3 best_offset{static_cast<double>(best_east_index) * configuration_.search_step_m, static_cast<double>(best_north_index) * configuration_.search_step_m, 0.0};

        if (best_mad > configuration_.max_acceptable_mad_m) {
            fix.rejection_reason = "correlation residual above threshold";
            return fix;
        }
        if (fix.confidence < configuration_.min_confidence) {
            fix.rejection_reason = "ambiguous correlation minimum";
            return fix;
        }

        // Accepted: the corrected position is the INS estimate plus the offset that
        // best matched the map. offset_enu_m is the negative of the INS horizontal
        // error, which the downstream filter folds back into its state.
        fix.offset_enu_m = best_offset;
        fix.position_enu_m = predicted_position_enu_m + best_offset;
        fix.accepted = true;
        return fix;
    }

}} // namespace TERCOM::Terrain
