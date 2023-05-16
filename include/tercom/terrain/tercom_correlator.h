#pragma once

// TERCOM batch correlator. Given a strip of measured terrain elevations tagged
// with the INS-estimated position at which each was taken, it slides the strip
// over the onboard reference map and finds the horizontal offset that minimises
// the mean absolute difference (MAD). MAD is the average of the absolute
// height differences (in metres) between the measured strip and the reference
// map at a candidate offset; it is preferred over mean-square because squaring
// lets a few large errors (a spurious altimeter return, an unmapped building)
// dominate the score, whereas the absolute value weights every sample linearly
// and so locks onto the true terrain shape rather than the loudest outlier.
//
// The result carries a confidence derived from the sharpness of the correlation
// minimum and a covariance for the downstream filter. Fixes over flat or
// repetitive terrain, where the minimum is shallow or ambiguous, are rejected:
// terrain must have relief for horizontal position to be observable.

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "tercom/geo/elevation_model.h"
#include "tercom/math/linear.h"
#include "tercom/math/matrix.h"

namespace TERCOM { namespace Terrain {

    // One element of the measured terrain strip: where the INS thought the vehicle
    // was (East-North-Up, metres) and the terrain height it sensed there (metres,
    // typically aircraft altitude minus radar-altimeter range to the ground).
    struct TerrainProfileSample {
        Math::Vec3 position_enu_m;         // INS-estimated position when measured
        double measured_elevation_m = 0.0; // terrain elevation under the vehicle
    };

    struct TercomFix {
        bool accepted = false;
        Math::Vec3 position_enu_m;        // corrected position estimate (~truth)
        Math::Vec3 offset_enu_m;          // applied horizontal shift (~ negative INS error)
        double mad_m = 0.0;               // mean absolute difference at the best offset, metres
        double confidence = 0.0;          // 0..1 sharpness of the correlation minimum
        double terrain_roughness_m = 0.0; // std of the measured profile, metres
        // Horizontal fix uncertainty as a 2x2 covariance in metres^2 over the
        // (east, north) plane. Handed to the navigation filter so it can weight
        // this fix against the dead-reckoned estimate.
        Math::Matrix<2, 2> covariance{};
        std::string rejection_reason; // empty when accepted
    };

    // Tuning for the correlator. The defaults are empirical: they were settled by
    // running the software-in-the-loop simulation and trading fix rate against
    // false-fix rate, not derived from a single formula. Each field documents what
    // it controls and which way to move it.
    struct TercomCorrelatorConfiguration {
        double min_search_radius_m = 150.0; // smallest the adaptive half-extent shrinks to, metres
        double max_search_radius_m = 400.0; // largest offset half-extent searched, metres
        // The offset search is confined to the INS uncertainty: the half-extent is
        // this many sigma of the filter's predicted horizontal error, clamped to
        // [min_search_radius_m, max_search_radius_m]. Larger admits more candidate
        // offsets (more chance of a false lock); smaller risks missing the true
        // offset when the INS has drifted far. Searching the whole map invites
        // false locks onto distant look-alike terrain; bounding the search to where
        // the vehicle can plausibly be is how real TERCOM/SITAN rejects them. The
        // 5-sigma default covers the true error with very high probability while
        // staying tight.
        double search_uncertainty_scale = 5.0;
        // Spacing of the candidate-offset grid, metres. This is the finest fix
        // resolution the search can resolve; smaller costs quadratically more work
        // (the search is a square grid), larger speeds it up but quantises the fix.
        double search_step_m = 25.0;
        // Fewest profile samples before a fix is even attempted. Too few samples
        // give an unreliable, easily-aliased correlation; this gates them out.
        std::size_t min_samples = 16;
        // Observability gate: the measured profile's height standard deviation
        // (metres) must exceed this or the terrain is too flat to pin down the
        // horizontal position and the fix is rejected. Raise to demand more relief.
        double min_terrain_roughness_m = 8.0;
        // Reject the fix if the best-match residual (MAD, metres) stays above this:
        // a high floor means the strip never really matched the map anywhere.
        double max_acceptable_mad_m = 60.0;
        // Reject when the computed confidence (0..1) is below this, i.e. the
        // correlation minimum is shallow or has a near-equal rival (ambiguous).
        double min_confidence = 0.25;
        // When judging ambiguity, ignore candidate offsets within this radius
        // (metres) of the best one: they are the same minimum, not a rival. The
        // best score outside this neighbourhood is the competitor used below.
        double competitor_exclusion_m = 80.0;
        // Expected one-sigma height-measurement noise, metres. Used as a softening
        // term in the separation score so a tiny competitor gap over quiet terrain
        // is not mistaken for genuine separation.
        double sensor_sigma_m = 10.0;
        double min_fix_std_m = 15.0;  // smallest reported horizontal one-sigma, metres (covariance floor)
        double max_fix_std_m = 200.0; // largest reported horizontal one-sigma, metres (covariance ceiling)
        // The fix one-sigma is calibrated to the best-match residual, which tracks
        // the achieved accuracy: sigma = min_fix_std + residual_sigma_gain * mad,
        // then clamped to [min_fix_std_m, max_fix_std_m]. This gain (metres of
        // reported sigma per metre of residual) sets how fast a worse match is
        // reported as a less certain fix; larger makes the filter trust poor
        // matches less.
        double residual_sigma_gain = 3.5;
    };

    class TercomCorrelator {
      public:
        explicit TercomCorrelator(const Geo::IElevationModel& reference_map, const TercomCorrelatorConfiguration& configuration = {});

        // Correlate the profile against the reference map. predicted_position_enu_m is
        // the INS estimate the offset is measured relative to;
        // prior_horizontal_std_m is the filter's one-sigma horizontal uncertainty,
        // which bounds the offset search (see search_uncertainty_scale). The
        // default leaves the search at its full configured radius.
        TercomFix Correlate(const std::vector<TerrainProfileSample>& profile, const Math::Vec3& predicted_position_enu_m,
            double prior_horizontal_std_m = std::numeric_limits<double>::infinity()) const;

      private:
        const Geo::IElevationModel& reference_map_;
        TercomCorrelatorConfiguration configuration_;
    };

}} // namespace TERCOM::Terrain
