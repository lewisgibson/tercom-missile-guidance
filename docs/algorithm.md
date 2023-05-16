# Algorithms

This document explains the estimation and guidance mathematics the library
implements: the strapdown inertial navigation, the error-state Kalman filter, the
TERCOM terrain correlator, and the L1 path-following law. It closes with the
modelling assumptions and the literature the design follows.

## The problem

An inertial navigation system (INS) integrates accelerometer and gyroscope
measurements to propagate position, velocity, and attitude. With nothing to correct
it, its error grows without bound: accelerometer bias integrates twice into a
position error that grows as t-squared, and gyroscope bias tilts the platform, which
leaks gravity into the horizontal channels and drives a position error that grows as
t-cubed. Over a few minutes even a good inertial unit drifts hundreds of metres.

Terrain-referenced navigation (TRN) bounds that drift without satellites. A
downward radar altimeter measures ground clearance; subtracting it from a
barometric altitude gives the terrain elevation directly beneath the vehicle. A
strip of those elevations, matched against an onboard digital elevation model,
reveals where the vehicle actually is. TERCOM (Terrain Contour Matching) is the
classic batch form of this idea.

## Strapdown inertial mechanisation

`Nav::StrapdownIns` integrates one IMU sample at a time in the ENU frame
(`math/attitude.h`, `nav/strapdown_ins.cpp`):

1. Subtract the current bias estimates from the raw gyroscope and accelerometer signals.
2. Rotate the bias-corrected specific force from body to navigation frame with the
   current attitude quaternion, then add gravity to get inertial acceleration:
   `a_nav = q * f_body + g_enu`.
3. Integrate position with the constant-acceleration rule (x += v*dt + 0.5*a*dt^2)
   and velocity with the rectangular rule (v += a*dt) over the interval.
4. Propagate attitude by the body-frame rotation vector `omega * dt` using the exact
   quaternion exponential, and renormalise.

The same integrator drives the simulator's truth model fed an ideal IMU, so a
noise-free, bias-free run reproduces truth to floating-point precision. That mutual
consistency is what makes the simulated error meaningful.

## Error-state Kalman filter

Rather than estimating the full nonlinear state, the filter estimates the small
_error_ in the INS state and feeds the correction back (a feedback, or closed-loop,
complementary filter). `Nav::ErrorStateFilter` carries nine states in ENU:

- position error (3),
- velocity error (3),
- accelerometer bias error (3).

### Prediction

Between measurements the error covariance is propagated with the linearised
error dynamics (`error_state_filter.cpp`):

```
  d(position)   = velocity
  d(velocity)   = -R_body_to_nav * accelerometer_bias_error
  d(accelerometer_bias) = 0                 (modelled as a random walk)
```

The discrete transition matrix carries velocity into position and the accelerometer
bias into both velocity (first order) and position (second order) over the step.
Process noise is added for the velocity random walk (white accelerometer noise), the
accelerometer-bias random walk, and a small direct position term that keeps the
filter from becoming overconfident on long dead-reckoning stretches. The velocity
process term is deliberately generous: it also absorbs the unmodelled heading drift
this nine-state filter does not estimate.

### Updates

Two measurement models fold in observations:

- **Barometric altitude** (always on): a scalar update on the up position. A nadir
  terrain measurement cannot observe absolute altitude, so this keeps the vertical
  channel stable.
- **Horizontal position** from an accepted TERCOM fix: a 2-D update on east and
  north using the fix covariance.

Every update uses the Joseph form, which keeps the covariance symmetric and positive
definite even under finite-precision arithmetic. After each update the position,
velocity, and bias corrections are added back into the INS and the error state is
reset to zero.

### The soft gate

A false terrain fix that slipped through the correlator would jolt the filter. The
obvious defence, a hard chi-square gate that rejects any fix whose normalised
innovation squared (NIS) exceeds a threshold, backfires here: once a run of good
fixes has collapsed the covariance, the gate refuses every subsequent correction
larger than a few sigma and the loop diverges with no way to recover.

So `UpdateHorizontalPositionGated` uses a **soft gate** (chi-square covariance
scaling). Instead of rejecting an outlier it inflates the measurement covariance in
proportion to how far the innovation exceeds the gate:

```
  inflation = max(1, NIS / gate)
  R' = inflation * R
```

A consistent fix (NIS within the gate) is applied at full weight. A suspect one is
smoothly down-weighted. A wildly inconsistent one is inflated so heavily it barely
moves the state. This is robust to false matches without ever locking out a genuine
correction.

## TERCOM correlator

`Terrain::TercomCorrelator` matches a measured elevation strip against the reference
map (`terrain/tercom_correlator.cpp`).

### Matching metric

For each candidate horizontal offset on a search grid, the strip is shifted by that
offset and the **mean absolute difference (MAD)** between the measured elevations and
the reference map is computed. MAD is preferred over mean-square difference in the
TRN literature for its robustness to outliers and to a constant vertical bias. The
offset with the smallest MAD is the candidate fix.

### Bounding the search to the INS uncertainty

Searching the whole map invites a false lock onto distant look-alike terrain, which
is the dominant failure mode on repetitive ground. The correlator instead confines
the offset search to the inertial uncertainty ellipse: the search half-extent is a
few sigma of the filter's predicted horizontal error, clamped between a floor and a
ceiling. The search widens automatically while the estimate is uncertain (initial
alignment) and tightens as the filter converges. This is how real TERCOM/SITAN
systems reject distant ambiguities, and in this implementation it is what removes the
periodic error spikes that an unconstrained search produces.

### Observability and confidence

Flat or repetitive terrain cannot constrain horizontal position, so a fix is
rejected unless:

- the profile carries enough relief (its elevation standard deviation exceeds a
  threshold),
- the best-match residual (MAD) is below a ceiling, and
- the correlation minimum is unambiguous.

Ambiguity is judged with a confidence in [0, 1] that blends _contrast_ (how far the
best match sits below the average residual) with _separation_ (how cleanly it beats
its best rival outside an exclusion neighbourhood). A distant competitor of similar
depth means the strip matches in more than one place and the fix is not trusted.

The fix covariance is calibrated to the achieved accuracy: the one-sigma scales with
the best-match residual, which tracks the fix error closely, clamped between a floor
and a ceiling. The downstream filter therefore weights each fix by how good it
actually was.

## L1 path-following guidance

`Guidance::GuidanceLaw` follows the planned route with the L1 nonlinear guidance law
(Park, Deyst, and How). It picks a reference point a lookahead distance L1 ahead
along the active leg, forms the lateral acceleration needed to drive the vehicle
onto the line through that point, and converts it to a coordinated-turn bank angle.
Pitch holds the leg altitude and throttle holds the leg speed, both clamped to limit
values. The wrapped heading error to the desired course is also emitted as a yaw
command for any downstream yaw control to use. This is route following between
waypoints, the same problem a survey drone solves, not terminal intercept.

## Modelling assumptions

The nine-state filter estimates position, velocity, and accelerometer bias, but not
attitude or gyroscope bias. It therefore cannot correct tilt-coupled gravity error. With
a **navigation-grade** gyroscope (order of 0.1 deg/hr, as in a cruise-missile ring-laser
or fibre-optic inertial unit) that error is negligible over the flight and the
nine-state filter is complete and correct, which is the configuration the library
ships and the simulator demonstrates. A tactical-grade gyroscope (order of 10 deg/hr)
would make tilt the dominant error and call for a fifteen-state Pinson filter that
adds three attitude-error and three gyroscope-bias states. That is a natural, scoped
extension; the current code documents the boundary rather than pretending past it.

## Result

The bundled scenario starts the estimate 150 m off truth and flies a 400 s,
three-leg cruise over procedurally generated terrain. The terrain fixes bound the
drift and the estimate converges to a final error around 17 m with a steady-state
RMS around 55 m and no divergence, fully deterministically. See the
[sitl-simulation example](../examples/sitl-simulation) and the `sitl_test`.

## References

The technique and the design choices follow the published terrain-aided navigation
literature:

- L. D. Hostetler and R. D. Andreas, "Nonlinear Kalman Filtering Techniques for
  Terrain-Aided Navigation," IEEE Transactions on Automatic Control, vol. 28,
  no. 3, 1983. The SITAN recursive (single-profile, Kalman) approach, in contrast
  to the batch TERCOM matching used here.
- J. P. Golden, "Terrain Contour Matching (TERCOM): A Cruise Missile Guidance Aid,"
  Proc. SPIE, 1980. The classic batch terrain-correlation description.
- U.S. Patent 4,829,304, "Map-Aided Navigation System Employing TERCOM-SITAN Signal
  Processing," 1989. Combining batch matching with a recursive terrain filter.
- N. Bergman, "Recursive Bayesian Estimation: Navigation and Tracking
  Applications," Ph.D. thesis, Linkoping University, 1999; and F. Gustafsson et al.,
  "Particle Filters for Positioning, Navigation, and Tracking," IEEE Transactions on
  Signal Processing, vol. 50, no. 2, 2002. Particle-filter TRN, the modern
  large-uncertainty alternative to the Kalman approaches.
- AFTI/F-16 SITAN flight tests reported terrain-aided accuracies on the order of
  75 m CEP, the kind of performance this class of estimator targets.
- Reference terrain in operational systems is typically DTED Level 2 (about 30 m,
  one arc-second, post spacing); the synthetic terrain here uses 100 m posts.
