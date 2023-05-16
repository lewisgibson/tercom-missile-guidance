#pragma once

// Terrain-referenced navigator: the closed loop that bounds inertial drift.
//
//   IMU      -> strapdown INS dead-reckoning + covariance prediction
//   baro     -> altitude update (keeps the vertical channel stable)
//   radar    -> accumulate measured terrain elevation (baro - clearance) into a
//               profile; once a strip is gathered, run the TERCOM correlator and
//               fold an accepted horizontal fix into the filter
//
// After every measurement the estimated INS error is fed back into the
// mechanisation and the error state is reset (closed loop). The estimate is read
// out through EstimatedState().

#include <cstddef>
#include <vector>

#include "tercom/nav/error_state_filter.h"
#include "tercom/nav/nav_state.h"
#include "tercom/nav/strapdown_ins.h"
#include "tercom/sensors/measurements.h"
#include "tercom/terrain/contour_map.h"
#include "tercom/terrain/tercom_correlator.h"

namespace TERCOM { namespace Nav {

    struct NavigatorConfiguration {
        // Radar samples gathered into one terrain strip before correlating. A
        // longer strip yields a sharper, less ambiguous correlation minimum and so
        // fewer false matches, at the cost of a longer interval between fixes.
        std::size_t correlation_batch_samples = 60; // radar samples per strip, count
        double barometer_variance_m2 = 4.0;         // baro altitude noise, m^2; (2 m one sigma)^2
        // Soft chi-square gate (two degrees of freedom: East and North): a terrain
        // fix whose normalised innovation squared exceeds this is folded in with an
        // inflated covariance rather than rejected. 13.8 is the 99.9th percentile
        // of the chi-square distribution with two degrees of freedom, so a
        // consistent fix passes the gate 99.9% of the time and only genuine
        // outliers (false terrain matches) get down-weighted.
        double fix_gate_chi2 = 13.8;
        ErrorStateFilterConfiguration filter;
        Terrain::TercomCorrelatorConfiguration correlator;
    };

    class TerrainReferencedNavigator {
      public:
        TerrainReferencedNavigator(Terrain::ContourMap reference_map, const NavState& initial_state, const NavigatorConfiguration& configuration = {});

        // Non-copyable and non-movable: the owned correlator holds a reference to
        // this object's own reference map, so copying or moving the navigator would
        // leave that reference dangling. It is meant to be constructed in place.
        TerrainReferencedNavigator(const TerrainReferencedNavigator&) = delete;
        TerrainReferencedNavigator& operator=(const TerrainReferencedNavigator&) = delete;
        TerrainReferencedNavigator(TerrainReferencedNavigator&&) = delete;
        TerrainReferencedNavigator& operator=(TerrainReferencedNavigator&&) = delete;
        ~TerrainReferencedNavigator() = default;

        void OnImu(const Sensors::ImuSample& sample);
        void OnBarometer(const Sensors::BarometricSample& sample);
        void OnRadarAltimeter(const Sensors::RadarAltitudeSample& sample);

        const NavState& EstimatedState() const {
            return ins_.State();
        }

        double HorizontalStdM() const {
            return filter_.HorizontalStdM();
        }

        double AltitudeStdM() const {
            return filter_.AltitudeStdM();
        }

        const Terrain::TercomFix& LastFix() const {
            return last_fix_;
        }

        std::size_t ProfileSize() const {
            return profile_.size();
        }

        std::size_t FixAttemptCount() const {
            return fix_attempts_;
        }

        std::size_t AcceptedFixCount() const {
            return accepted_fixes_;
        }

        // Of the accepted fixes, how many were inconsistent with the filter's
        // predicted uncertainty and thus folded in down-weighted by the soft gate.
        // A healthy run keeps this small; a rising count flags ambiguous terrain.
        std::size_t DownweightedFixCount() const {
            return downweighted_fixes_;
        }

      private:
        void ApplyFilterCorrections();

        Terrain::ContourMap reference_map_;
        Terrain::TercomCorrelator correlator_;
        StrapdownIns ins_;
        ErrorStateFilter filter_;
        NavigatorConfiguration configuration_;

        std::vector<Terrain::TerrainProfileSample> profile_;
        double latest_barometric_altitude_m_ = 0.0;
        bool have_barometer_ = false;
        Terrain::TercomFix last_fix_;
        std::size_t fix_attempts_ = 0;
        std::size_t accepted_fixes_ = 0;
        std::size_t downweighted_fixes_ = 0;
    };

}} // namespace TERCOM::Nav
