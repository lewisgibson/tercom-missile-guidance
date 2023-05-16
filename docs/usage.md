# Usage

This guide covers building the library, running the examples and tools, and using
the library from your own program.

## Building and testing

The project uses CMake presets (see `CMakePresets.json`). The common ones:

| Preset         | What it configures                                   |
| -------------- | ---------------------------------------------------- |
| `dev`          | Debug build with examples, tools, and tests.         |
| `release`      | Optimised (RelWithDebInfo) build.                    |
| `asan`         | Debug build with Address and UB sanitizers.          |
| `coverage`     | Debug build with coverage instrumentation.           |
| `tidy`         | Debug build that runs clang-tidy during compilation. |
| `cppcheck`     | Debug build that runs cppcheck during compilation.   |
| `strict-gcc`   | RelWithDebInfo, GCC, warnings as errors.             |
| `strict-clang` | RelWithDebInfo, Clang, warnings as errors.           |

```bash
cmake --preset dev
cmake --build build/dev
ctest --preset dev
```

The thin `Makefile` wraps the common tasks:

```bash
make build      # configure + build (dev)
make test       # run the unit tests
make simulate   # build and run the SITL simulation example
make benchmark  # build optimized and run the performance benchmarks
make format     # reformat all sources with clang-format
make lint       # format check + clang-tidy + cppcheck + prettier + actionlint
make coverage   # build instrumented, run tests, emit an HTML report
make clean      # remove the build tree
```

GoogleTest is fetched automatically at configure time; nothing needs to be
installed. clang-format defaults to the versioned binary (`clang-format-18`) this
project was developed against; override it if your distribution names it
differently, for example `make format-check CLANG_FORMAT=clang-format`. clang-tidy
is located by CMake's `find_program`, preferring `clang-tidy-18` and falling back to
an unversioned `clang-tidy`.

## Running the examples and tools

After `cmake --build build/dev` the binaries are under `build/dev/bin/`:

- `tracking` - a real-time-style demo that flies a straight cruise leg, feeds
  simulated sensors to a `GuidanceSystem`, and prints the estimate, the error
  against truth, and the control correction once a second. It runs for a fixed
  simulated duration (180 s, roughly 5 s of wall time) or until the route completes,
  then exits on its own; press Ctrl-C to stop it early.
- `sitl-simulation` - runs the full `Simulation::Simulator` over the tuned scenario, prints
  the convergence summary, and writes a CSV time history:

  ```bash
  ./build/dev/bin/sitl-simulation sitl-convergence.csv
  ```

  The CSV columns are time, truth east/north, estimate east/north, horizontal
  error, altitude error, estimated horizontal one-sigma, a terrain-fix flag, and the
  fix diagnostics (TERCOM mean-absolute-difference and confidence), so the
  convergence can be plotted directly.

- `generate-terrain` - synthesises a fractal digital elevation model and writes an
  Esri ASCII grid:

  ```bash
  ./build/dev/bin/generate-terrain --cols 700 --rows 700 --cell 100 \
      --base 600 --relief 520 --wavelength 2500 --octaves 7 --seed 2024 \
      --out terrain.asc
  ```

  Pass `-` as the output path to write to standard output.

## Using the library

Link against the exported target and feed a `GuidanceSystem` your sensor samples.

```cmake
find_package(tercom REQUIRED)
target_link_libraries(my_app PRIVATE tercom::tercom)
```

```cpp
#include "tercom/geo/synthetic_terrain.h"
#include "tercom/nav/nav_state.h"
#include "tercom/tercom.h"
#include "tercom/trajectory/route.h"

using namespace TERCOM;

// Onboard reference map (load a real DEM with Geo::LoadEsriAsciiGrid, or generate one).
Geo::HeightField map = Geo::GenerateFractalTerrain({});

// Planned route in the local ENU frame, with a target speed per leg.
Trajectory::Route route({
    {{5000.0, 5000.0, 1100.0}, 200.0},
    {{25000.0, 15000.0, 1100.0}, 200.0},
});

// Initial estimate (position, velocity, attitude); biases start unknown.
Nav::NavState initial;
initial.position_enu_m = {5000.0, 5000.0, 1100.0};
initial.velocity_enu_mps = {179.0, 89.0, 0.0};

GuidanceSystem guidance(Terrain::ContourMap(map), initial, route);

// Drive it from your sensor loop:
guidance.OnImu(imu_sample);              // at the IMU rate (e.g. 100 Hz)
guidance.OnBarometer(barometer_sample);  // at the barometer rate
guidance.OnRadarAltimeter(radar_sample); // at the radar rate

// Read back the estimate and the control correction to fly the route:
const Nav::NavState& estimate = guidance.EstimatedState();
const Trajectory::Correction correction = guidance.CalculateCorrection();
// correction.roll_command_rad, .pitch_command_rad, .yaw_command_rad, .throttle
```

The sensor samples are plain records (`Sensors::ImuSample`, `BarometricSample`,
`RadarAltitudeSample`). In simulation they come from the simulated backends; on real
hardware they come from your drivers. For a real GNSS receiver,
`Sensors::SerialNmeaGnss` reads NMEA-0183 from a serial device; see
[sensors.md](sensors.md).

## Configuration

Behaviour is tuned through plain configuration structs with sensible defaults:

- `Nav::NavigatorConfiguration` - the correlation batch size, the soft-gate threshold, and
  nested `ErrorStateFilterConfiguration` and `Terrain::TercomCorrelatorConfiguration`.
- `Terrain::TercomCorrelatorConfiguration` - the search radius bounds and uncertainty
  scaling, the observability and confidence gates, and the covariance calibration.
- `Guidance::GuidanceConfiguration` - the L1 lookahead distance, the bank/pitch limits, and
  the altitude/speed gains.

The shipped defaults are the tuned, known-good operating point; see
[algorithm.md](algorithm.md) for what they mean and why they were chosen.

## Tooling notes

- C++20 is required. The build sets `CMAKE_CXX_EXTENSIONS OFF` and exports
  `compile_commands.json`.
- The strict warning set (`-Wconversion`, `-Wsign-conversion`, `-Wold-style-cast`,
  `-Wdouble-promotion`, and more) is on for first-party targets; the strict presets
  promote warnings to errors. Fetched dependencies are never passed through it.
- `clang-tidy` and `cppcheck` run on the library, examples, tools, and benchmarks
  (not on the test translation units). The rulesets are curated in `.clang-tidy` and
  `cmake/StaticAnalysis.cmake`.
