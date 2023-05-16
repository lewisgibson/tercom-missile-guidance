# Sensors

The navigation core never talks to hardware directly. It consumes plain
measurement records, and every sensor is an abstract interface with both a
simulated backend (for the software-in-the-loop harness) and, where it matters, a
real hardware backend. This keeps the estimator testable and lets the same code run
against a simulation or a real vehicle.

## Measurement records

`sensors/measurements.h` defines the records the pipeline consumes. Timestamps are
seconds on the vehicle clock; frames follow the library convention (body axes
Forward-Left-Up, navigation frame East-North-Up).

| Record                | Fields                                               |
| --------------------- | ---------------------------------------------------- |
| `ImuSample`           | body angular rate, body specific force, time         |
| `BarometricSample`    | altitude above the local ENU datum, time, valid flag |
| `RadarAltitudeSample` | ground clearance, time, valid flag                   |
| `GnssFix`             | geodetic position, satellite count, time, valid flag |

The `valid` flag models real dropouts: a barometer with no reading, a radar
altimeter beyond range (on real hardware also excessive bank or a specular
surface), or a GNSS receiver without a fix.

## Interfaces

Each sensor is a one-method interface in `TERCOM::Sensors`:

- `IImu::Read() -> ImuSample`
- `IBarometricAltimeter::Read() -> BarometricSample`
- `IRadarAltimeter::Read() -> RadarAltitudeSample`
- `IGnssReceiver::Read() -> GnssFix`

The navigator consumes only the IMU, barometer, and radar-altimeter interfaces (it
exposes only `OnImu`/`OnBarometer`/`OnRadarAltimeter`, see `nav/terrain_navigator.h`).
GNSS is intentionally outside the navigation loop: terrain-referenced navigation is
precisely the fallback for when no satellite fix is available. The `IGnssReceiver`
interface and its simulated and NMEA-0183 serial backends are provided and
unit-tested as a hardware-abstraction component, but no library path currently
consumes a `GnssFix`; wiring it in for initial alignment or an independent
cross-check is left to the integrator (or future work).

## Simulated backends

`sensors/simulated_sensors.h` provides `SimulatedImu`, `SimulatedBarometer`,
`SimulatedRadarAltimeter`, and `SimulatedGnss`. Each holds the latest ground-truth
quantity (set by the simulator through `SetTruth`) and corrupts it on `Read()` with
a deterministic, seeded bias and noise model. Because the randomness is seeded and
carries no global state, a run is fully reproducible.

Notable model details:

- The IMU adds a constant bias plus white noise to the specific force and angular
  rate. The default gyroscope bias is navigation grade (order of 0.1 deg/hr); see the
  assumptions in [algorithm.md](algorithm.md).
- The radar altimeter noise is a fraction of the measured clearance with a floor,
  and the sample is marked invalid beyond the configured maximum range, so noise
  grows with altitude the way a real altimeter's does.
- The barometer carries a constant bias on top of white noise.
- GNSS is off by default (the GPS-denied path is the point); when enabled it
  perturbs the truth in metres mapped to degrees at the local latitude.

## Real GNSS over NMEA-0183

For a real receiver the library ships a complete path from serial bytes to a fix.

### The parser (`sensors/nmea.h`)

`ParseGgaSentence` and `NmeaChecksumValid` are pure functions, so they can be unit
tested without any hardware. The parser:

- validates the NMEA checksum (the XOR of the characters between `$` and `*` against
  the two hex digits after `*`), treating a sentence with no `*` as valid since some
  emitters omit it;
- accepts only the GGA sentence type from any talker (`$GPGGA`, `$GNGGA`, `$GLGGA`,
  and so on) and rejects other sentence types such as RMC or GSV;
- converts the `ddmm.mmmm` / `dddmm.mmmm` latitude and longitude fields to signed
  decimal degrees, honouring the N/S and E/W hemisphere fields;
- rejects a sentence whose fix-quality field is zero (no fix) and any malformed or
  empty position.

For example, the canonical sentence
`$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47`
parses to latitude 48.1173 deg, longitude 11.5167 deg, 8 satellites; the same
sentence with a corrupted checksum, or with a fix quality of 0, is rejected.

### The serial backend (`sensors/serial_gnss.h`)

`SerialNmeaGnss` drives a real device (for example a u-blox module on
`/dev/ttyUSB0`) over POSIX termios. It is non-blocking: each `Read()` drains the
bytes available, parses any complete GGA sentences, and returns the latest fix. The
class is non-copyable and non-movable (it owns a file descriptor) and its
constructor throws if the device cannot be opened or configured. The serial code is
guarded so the rest of the library still builds on non-POSIX platforms; only this
one backend is unavailable there.

Because `SerialNmeaGnss` implements `IGnssReceiver`, swapping the simulated receiver
for a real one is a one-line change at the construction site; nothing downstream
changes.
