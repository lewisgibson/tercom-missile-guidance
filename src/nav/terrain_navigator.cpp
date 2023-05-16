#include "tercom/nav/terrain_navigator.h"

#include <utility>

namespace TERCOM { namespace Nav {

    TerrainReferencedNavigator::TerrainReferencedNavigator(Terrain::ContourMap reference_map, const NavState& initial_state, const NavigatorConfiguration& configuration)
        : reference_map_(std::move(reference_map)),
          correlator_(reference_map_.Elevation(), configuration.correlator),
          ins_(initial_state),
          filter_(configuration.filter),
          configuration_(configuration) {
        profile_.reserve(configuration_.correlation_batch_samples);
    }

    // IMU path: dead-reckon the INS forward, then propagate the filter covariance
    // over the same interval. The body-to-nav rotation is needed by Predict to map
    // accelerometer bias error into a velocity error in the nav frame.
    void TerrainReferencedNavigator::OnImu(const Sensors::ImuSample& sample) {
        const double previous_time = ins_.State().time_s;
        ins_.Step(sample);
        const double dt = sample.time_s - previous_time;
        if (dt > 0.0) {
            const Math::Mat3 body_to_nav = Math::RotationMatrixFromQuaternion(ins_.State().attitude);
            filter_.Predict(dt, body_to_nav);
        }
    }

    // Barometer path: cache the altitude (the radar path needs it to turn ground
    // clearance into terrain elevation) and feed an altitude update to the filter.
    // The innovation is INS up-position minus measured altitude, i.e. the error the
    // baro thinks the INS has. This always-on update is what keeps the vertical
    // channel from drifting, since the nadir terrain measurement cannot.
    void TerrainReferencedNavigator::OnBarometer(const Sensors::BarometricSample& sample) {
        if (!sample.valid) {
            return;
        }
        latest_barometric_altitude_m_ = sample.altitude_m;
        have_barometer_ = true;

        const double innovation = ins_.State().position_enu_m.z - sample.altitude_m;
        filter_.UpdateAltitude(innovation, configuration_.barometer_variance_m2);
        ApplyFilterCorrections();
    }

    void TerrainReferencedNavigator::OnRadarAltimeter(const Sensors::RadarAltitudeSample& sample) {
        if (!sample.valid || !have_barometer_) {
            return;
        }

        // Terrain elevation beneath the vehicle: absolute altitude minus clearance.
        // The radar measures height above the ground; subtracting it from the baro
        // altitude (height above the reference) gives the ground's own elevation,
        // which is what the reference map stores. Tag it with the current INS
        // position so the correlator knows where along track each sample was taken.
        const double terrain_elevation = latest_barometric_altitude_m_ - sample.clearance_m;
        profile_.push_back(Terrain::TerrainProfileSample{ins_.State().position_enu_m, terrain_elevation});

        // Correlate only once a full strip is gathered: a longer profile gives a
        // sharper, less ambiguous match (see correlation_batch_samples).
        if (profile_.size() < configuration_.correlation_batch_samples) {
            return;
        }

        // Match the measured strip against the reference map. The horizontal
        // uncertainty sizes the search window: a confident filter searches a tight
        // window, a drifting one searches wider.
        last_fix_ = correlator_.Correlate(profile_, ins_.State().position_enu_m, filter_.HorizontalStdM());
        ++fix_attempts_;
        if (last_fix_.accepted) {
            // Innovation is INS position minus the terrain fix: the horizontal
            // error the fix attributes to the INS.
            const double innovation_east = ins_.State().position_enu_m.x - last_fix_.position_enu_m.x;
            const double innovation_north = ins_.State().position_enu_m.y - last_fix_.position_enu_m.y;
            // The correlator passed its own checks; the filter's soft gate folds
            // the fix in robustly, inflating the covariance of any innovation that
            // is inconsistent with its predicted uncertainty rather than rejecting
            // it (a hard reject starves and diverges once the covariance collapses).
            const bool consistent = filter_.UpdateHorizontalPositionGated(innovation_east, innovation_north, last_fix_.covariance, configuration_.fix_gate_chi2);
            if (!consistent) {
                ++downweighted_fixes_;
            }
            ++accepted_fixes_;
            ApplyFilterCorrections();
        }

        // Start the next strip fresh whether or not a fix was accepted.
        profile_.clear();
    }

    // Closed-loop step shared by every measurement path: push the filter's
    // estimated error corrections into the INS, then zero the error state so the
    // next prediction starts from a clean "INS is correct" assumption.
    void TerrainReferencedNavigator::ApplyFilterCorrections() {
        ins_.ApplyCorrection(filter_.PositionCorrection(), filter_.VelocityCorrection(), filter_.AccelerometerBiasCorrection());
        filter_.ResetErrorState();
    }

}} // namespace TERCOM::Nav
