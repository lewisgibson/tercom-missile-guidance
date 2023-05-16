#pragma once

// Umbrella facade for the TERCOM terrain-referenced navigation and guidance
// system. A GuidanceSystem owns the terrain-referenced navigator (INS + filter +
// TERCOM correlator) and the guidance law. Feed it sensor samples; read back the
// estimated state and the control corrections to fly the planned route.

#include "tercom/guidance/guidance_law.h"
#include "tercom/nav/nav_state.h"
#include "tercom/nav/terrain_navigator.h"
#include "tercom/sensors/measurements.h"
#include "tercom/terrain/contour_map.h"
#include "tercom/trajectory/correction.h"
#include "tercom/trajectory/route.h"

namespace TERCOM {

    // Tuning for the two subsystems the facade owns: the terrain-referenced
    // navigator (INS, filter, and TERCOM correlator) and the guidance law.
    struct GuidanceSystemConfiguration {
        Nav::NavigatorConfiguration navigator;
        Guidance::GuidanceConfiguration guidance;
    };

    class GuidanceSystem {
      public:
        // reference_map is the onboard terrain map TERCOM matches against,
        // initial_state is where the navigator believes it starts, and route is the
        // plan to fly.
        GuidanceSystem(Terrain::ContourMap reference_map, const Nav::NavState& initial_state, Trajectory::Route route, const GuidanceSystemConfiguration& configuration = {});

        // Non-copyable and non-movable: it owns a TerrainReferencedNavigator, which
        // is itself pinned in place (see that class). Construct it where it lives.
        GuidanceSystem(const GuidanceSystem&) = delete;
        GuidanceSystem& operator=(const GuidanceSystem&) = delete;
        GuidanceSystem(GuidanceSystem&&) = delete;
        GuidanceSystem& operator=(GuidanceSystem&&) = delete;
        ~GuidanceSystem() = default;

        // Push one sensor sample into the navigator. IMU samples propagate the INS;
        // barometer and radar-altimeter samples are aiding measurements (the radar,
        // combined with the map, is what drives the TERCOM correction).
        void OnImu(const Sensors::ImuSample& sample);
        void OnBarometer(const Sensors::BarometricSample& sample);
        void OnRadarAltimeter(const Sensors::RadarAltitudeSample& sample);

        // Control corrections to follow the route from the current estimate.
        Trajectory::Correction CalculateCorrection();

        // Best current estimate of the full navigation state.
        const Nav::NavState& EstimatedState() const;
        // Filter's one-sigma horizontal position uncertainty, in metres.
        double HorizontalStdM() const;
        // Filter's one-sigma vertical (altitude) position uncertainty, in metres.
        double AltitudeStdM() const;
        // Direct access to the navigator for fix counts and diagnostics.
        const Nav::TerrainReferencedNavigator& Navigator() const;
        const Trajectory::Route& Route() const;
        // True once the last waypoint has been reached.
        bool ReachedDestination() const;

      private:
        Nav::TerrainReferencedNavigator navigator_;
        Guidance::GuidanceLaw guidance_;
        Trajectory::Route route_;
    };

} // namespace TERCOM
