# tercom-missile-guidance

![C++20](https://img.shields.io/badge/C%2B%2B-20-blue)
![Build: CMake](https://img.shields.io/badge/build-CMake-064F8C)
![Tests: 345 passing](https://img.shields.io/badge/tests-345%20passing-brightgreen)
![Coverage: 99% lines](https://img.shields.io/badge/coverage-99%25%20lines-brightgreen)
![License: MIT](https://img.shields.io/badge/license-MIT-green)

A C++20 library and software-in-the-loop (SITL) simulator for **terrain-referenced
navigation (TRN)**: the GPS-denied positioning technique that matches a strip of
terrain elevations measured under a vehicle against an onboard digital elevation
model to bound the drift of an inertial navigation system.

The classic name for the technique is **TERCOM** (Terrain Contour Matching). It is
a published, textbook localization algorithm; this project implements the
navigation and route-following mathematics end to end and exercises it in a
repeatable simulation.

## Scope

This is a **navigation and simulation** library. "Guidance" here means following a
planned route between waypoints, the same path-following problem a survey drone or
a UAV solves. There is deliberately **no** warhead, fuze, terminal-attack, or
target-destruction logic, and there is no intent to add any. The interesting
engineering is in the estimator, the terrain correlation, and the systems
integration, not in lethality.

## What it does

- **Strapdown inertial navigation** in a local East-North-Up frame (quaternion
  attitude, specific-force mechanisation).
- **Terrain correlation (TERCOM)** by mean-absolute-difference matching of a
  measured elevation profile against the reference map, with an adaptive search
  confined to the inertial uncertainty so it rejects distant look-alike terrain.
- **An error-state Kalman filter** (9-state ESKF: position, velocity, accelerometer
  bias) that fuses an always-on barometric altitude update with the terrain fixes
  and feeds the correction back into the INS (closed loop).
- **L1 path-following guidance** that turns the estimated state and the planned
  route into bank, pitch, throttle, and heading-error (yaw) commands.
- **A sensor hardware-abstraction layer** with simulated backends for the SITL
  harness and a real NMEA-0183 serial GNSS path (with a pure, tested parser).
- **A deterministic SITL simulator** that flies a coordinated-turn truth model,
  synthesises the sensors from truth, and runs the whole loop.

## Result

The bundled SITL scenario starts the estimator 150 m off truth and flies a 400 s,
three-leg cruise over procedurally generated terrain. With a navigation-grade IMU
the terrain fixes bound the inertial drift and the estimate converges:

| Metric                    | Value                            |
| ------------------------- | -------------------------------- |
| Terrain fixes             | 95 accepted / 133 attempts (71%) |
| Final horizontal error    | 16.5 m                           |
| RMS after first fix       | 60.5 m                           |
| Steady-state RMS (> 60 s) | 54.0 m                           |
| Steady-state peak         | 137.6 m                          |

Unaided, the same inertial unit drifts to several hundred metres over the run. The
result is fully deterministic: the same build prints the same numbers every time.
See [docs/algorithm.md](docs/algorithm.md) for why, and the
[sitl-simulation example](examples/sitl-simulation) to reproduce it (it also writes
a CSV of the convergence history).

## Performance

The whole closed loop is cheap. Run `make benchmark` for the full output; the key
figures are below:

| Operation                          | Average  | Throughput        |
| ---------------------------------- | -------- | ----------------- |
| Strapdown INS step                 | ~29 ns   | ~35 million/s     |
| ESKF covariance prediction         | ~165 ns  | ~6 million/s      |
| TERCOM correlate (bounded search)  | ~0.06 ms | --                |
| Full SITL run (400 s, 40000 steps) | ~27 ms   | ~14500x real time |

<sub>Measured single-threaded on an AMD Ryzen 9 9950X3D, GCC 13, RelWithDebInfo
(`-O3`); your numbers will differ but the ratios hold.</sub>

So 400 seconds of flight, navigation filter and terrain correlation included,
computes in well under a tenth of a second. The optimized builds enable `-O3`
(see `cmake/Optimization.cmake`); the hot paths stay plain, readable code and let
the compiler vectorise them (the covariance prediction alone is about three times
faster than at `-O2`).

## Building

Requirements: CMake >= 3.24, a C++20 compiler (GCC >= 11 or Clang >= 14), and
Ninja. GoogleTest is fetched automatically at configure time.

```bash
cmake --preset dev
cmake --build build/dev
ctest --preset dev
```

Or use the thin `Makefile` wrapper:

```bash
make build      # configure + build (Debug)
make test       # run the unit tests
make simulate   # build and run the SITL simulation example
make benchmark  # build optimized and run the benchmarks
make lint       # format check + clang-tidy + cppcheck + prettier + actionlint
make coverage   # build instrumented, run tests, emit an HTML report
```

The CMake package installs and exports a `tercom::tercom` target, so a downstream
project can `find_package(tercom)` and link against it.

## Documentation

- [docs/architecture.md](docs/architecture.md) - module layout and data flow
- [docs/algorithm.md](docs/algorithm.md) - the INS, ESKF, TERCOM correlator, L1
  guidance, and the cited literature
- [docs/sensors.md](docs/sensors.md) - the sensor abstraction layer and NMEA parsing
- [docs/usage.md](docs/usage.md) - using the library in your own program

## License

MIT. See [LICENSE](LICENSE).
