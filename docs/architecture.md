# Architecture

This document describes how the library is laid out and how data flows through the
closed navigation loop. For the mathematics behind each block see
[algorithm.md](algorithm.md).

## Module layout

Everything lives under the top-level namespace `TERCOM`, split into PascalCase
sub-namespaces that mirror the directory layout under `include/tercom/` and `src/`:

| Namespace            | Directory     | Responsibility                                                       |
| -------------------- | ------------- | -------------------------------------------------------------------- |
| `TERCOM::Math`       | `math/`       | Vectors, 3x3 matrices, fixed-size matrices, quaternion attitude.     |
| `TERCOM::Geo`        | `geo/`        | WGS84 geodesy, the digital elevation model, terrain generation/IO.   |
| `TERCOM::Sensors`    | `sensors/`    | Sensor abstractions, simulated backends, NMEA parsing, serial GNSS.  |
| `TERCOM::Nav`        | `nav/`        | Strapdown INS, the error-state Kalman filter, the terrain navigator. |
| `TERCOM::Terrain`    | `terrain/`    | The onboard reference map and the TERCOM correlator.                 |
| `TERCOM::Trajectory` | `trajectory/` | Waypoints, routes, and the control-correction record.                |
| `TERCOM::Guidance`   | `guidance/`   | The L1 path-following guidance law.                                  |
| `TERCOM::Simulation` | `simulation/` | The software-in-the-loop simulator (truth model + sensor synthesis). |
| `TERCOM`             | `tercom.h`    | The `GuidanceSystem` facade that ties the navigator and guidance.    |

The dependency direction is strictly downward: `Math` depends on nothing, `Geo`
depends only on `Math`, `Sensors` depends on `Math` and `Geo` (a `GnssFix` carries
a `Geo::GeoPoint`), `Nav`/`Terrain` build on those, `Guidance` and the
`GuidanceSystem` facade sit on top, and `Simulation` depends on the facade. There
are no cycles.

## Frame conventions

One set of conventions is used everywhere so the inertial mechanisation and the
terrain matching compose cleanly:

- **Navigation frame:** local East-North-Up (ENU). `x` is east, `y` is north,
  `z` is up. Gravity is `kGravityEnu = (0, 0, -9.80665)`.
- **Body frame:** Forward-Left-Up (FLU), right-handed, pairs with ENU.
- **Attitude:** a unit quaternion that rotates a body vector into the navigation
  frame (`v_nav = q * v_body`), Hamilton convention, ZYX (yaw-pitch-roll) Euler
  order.
- **Geodetic anchoring:** `Geo::LocalFrame` ties the ENU plane to a WGS84 origin so
  real geographic elevation models can be ingested. Internally the whole loop runs
  in ENU metres.

## The closed loop

The heart of the system is `Nav::TerrainReferencedNavigator`, which owns a
`StrapdownIns`, an `ErrorStateFilter`, and a `TercomCorrelator`. The three sensor
callbacks drive it:

```
  IMU sample (100 Hz)
      -> StrapdownIns::Step           (dead-reckon attitude, velocity, position)
      -> ErrorStateFilter::Predict    (propagate the error covariance)

  barometer sample (10 Hz)
      -> ErrorStateFilter::UpdateAltitude   (observe absolute altitude)
      -> feed the correction back into the INS, reset the error state

  radar-altimeter sample (20 Hz)
      -> terrain elevation = barometric altitude - ground clearance
      -> append (INS position, elevation) to the current profile strip
      -> once the strip is full:
             TercomCorrelator::Correlate   (match the strip to the reference map)
             ErrorStateFilter::UpdateHorizontalPositionGated  (fold an accepted fix)
             feed the correction back into the INS, reset the error state
```

This is a feedback (closed-loop, complementary) error-state filter: the INS runs
the full nonlinear mechanisation, the filter estimates only the small error in that
mechanisation, and after every measurement the estimated error is added back into
the INS state and the error state is zeroed. Between terrain fixes the INS coasts;
each accepted fix bounds the drift.

```
        +-------------------- correction (closed loop) --------------------+
        |                                                                  |
        v                                                                  |
   StrapdownIns --state--> ErrorStateFilter <--innovation-- TercomCorrelator
        ^                        ^                                 ^
        | IMU                    | baro                            | radar + reference map
```

## The facade and the simulator

`GuidanceSystem` (in `tercom.h`) wraps the navigator together with a
`Guidance::GuidanceLaw` and a `Trajectory::Route`. You feed it sensor samples and
read back the estimated state and a `Trajectory::Correction` (bank, pitch,
throttle, plus a heading-error/yaw command left for downstream yaw control) to fly
the planned route. This is the surface a real vehicle's flight software would use.

`Simulation::Simulator` is the test harness around that facade. It flies an independent
ground-truth vehicle with a coordinated-turn flight model, derives the ideal IMU as
the exact inverse of the INS mechanisation, synthesises the barometer and radar
altimeter from the truth and the terrain, corrupts them with seeded bias and noise,
and runs a `GuidanceSystem` on the result. Because truth and estimate are kept
separate, the recorded error (estimate minus truth) isolates the navigation
performance. With the noise and bias switched off the estimate reproduces truth to
within floating-point error, which is the consistency check that proves the truth
model, the INS, and the filter all agree.

## Sensor abstraction

Every sensor is an interface in `Sensors` (`IImu`, `IBarometricAltimeter`,
`IRadarAltimeter`, `IGnssReceiver`). The simulator uses the simulated backends; a
real vehicle would supply hardware-backed implementations. The GNSS path ships with
both a simulated backend and a real one (`SerialNmeaGnss`) built on a pure,
unit-tested NMEA-0183 parser. See [sensors.md](sensors.md).

## Build and tooling

The CMake build (top-level `CMakeLists.txt` plus `src/`, `examples/`, `tools/`,
`tests/`, and the optional `benchmarks/` subdirectories) produces the
`tercom::tercom` library, the examples, the tools, the GoogleTest suite, and the
benchmark program. Warning flags, sanitizers, coverage, optimization (`-O3` in
release builds), clang-tidy, and cppcheck are wired through the helper modules in
`cmake/` and selected with the `CMakePresets.json` presets. See
[usage.md](usage.md) and the top-level README.
