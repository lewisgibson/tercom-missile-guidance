#include "tercom/tercom.h"

#include <utility>

namespace TERCOM {

    GuidanceSystem::GuidanceSystem(Terrain::ContourMap reference_map, const Nav::NavState& initial_state, Trajectory::Route route, const GuidanceSystemConfiguration& configuration)
        : navigator_(std::move(reference_map), initial_state, configuration.navigator),
          guidance_(configuration.guidance),
          route_(std::move(route)) {
    }

    void GuidanceSystem::OnImu(const Sensors::ImuSample& sample) {
        navigator_.OnImu(sample);
    }

    void GuidanceSystem::OnBarometer(const Sensors::BarometricSample& sample) {
        navigator_.OnBarometer(sample);
    }

    void GuidanceSystem::OnRadarAltimeter(const Sensors::RadarAltitudeSample& sample) {
        navigator_.OnRadarAltimeter(sample);
    }

    // The one method that composes the two subsystems: feed the navigator's current
    // estimate and the route into the guidance law to get the control correction.
    // Every other method here just forwards to the navigator.
    Trajectory::Correction GuidanceSystem::CalculateCorrection() {
        return guidance_.Compute(navigator_.EstimatedState(), route_);
    }

    const Nav::NavState& GuidanceSystem::EstimatedState() const {
        return navigator_.EstimatedState();
    }

    double GuidanceSystem::HorizontalStdM() const {
        return navigator_.HorizontalStdM();
    }

    double GuidanceSystem::AltitudeStdM() const {
        return navigator_.AltitudeStdM();
    }

    const Nav::TerrainReferencedNavigator& GuidanceSystem::Navigator() const {
        return navigator_;
    }

    const Trajectory::Route& GuidanceSystem::Route() const {
        return route_;
    }

    bool GuidanceSystem::ReachedDestination() const {
        return route_.IsComplete();
    }

} // namespace TERCOM
