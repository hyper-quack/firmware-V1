# Navigation EKF — position & velocity fusion

This is the estimator that answers **"where am I and how fast am I moving?"** by
fusing GPS, barometer, lidar, and optical flow with the accelerometer. Read
[sensor-fusion.md](sensor-fusion.md) first for the whole-stack overview; this file
is the deep dive on the nav filter.

- [ekf.rs](../src/ekf.rs) — the filter
- [main.rs](../src/main.rs) — `ekf_task` wiring

---

## 1. Architecture — why *loosely-coupled*

PX4's `ekf2` is one big **tightly-coupled** 24-state error-state EKF: attitude,
velocity, position, and every bias estimated together in a single covariance.
It's excellent but large, and a bug anywhere couples into everything.

scky-fc uses the **loosely-coupled** design instead:

```text
   ┌─────────────────────────── attitude path ──────────────────────────┐
   IMU ─► filters ─► estimator ─► Mahony AHRS ─► attitude (roll/pitch/yaw)
                                       │
                                       ├─► world-frame accel (gravity removed)
                                       ▼
   GPS ───────┐                  ┌──────────┐
   baro ──────┼─── measurements ►│ nav EKF  │─► position + velocity
   lidar ─────┤                  │ (ekf.rs) │
   flow ──────┘                  └──────────┘
```

The attitude filter and the nav filter are **separate**. The AHRS hands the EKF
two things: the orientation (to rotate the accelerometer into the world) and the
gravity-removed world acceleration. The EKF never touches orientation. This is
smaller, debuggable, and each half can be reasoned about alone.

| | PX4 ekf2 | scky-fc |
|---|---|---|
| Structure | one 24-state EKF | AHRS + nav EKF, separate |
| Attitude | in the filter | Mahony complementary ([ahrs.rs](../src/ahrs.rs)) |
| Pos/vel | in the filter | this nav EKF |
| Coupling | tight | loose |
| States | 24 | 3 axes × 3 = 9 |

---

## 2. The trick: three independent 3-state filters

The motion model is *constant acceleration per axis* and every measurement is a
direct position or velocity — **nothing couples North to East to Up.** So the
filter is block-diagonal and factorises exactly into **three identical filters**,
one per axis ([`Axis1D`](../src/ekf.rs)). Each axis carries **3 states**:

```text
   state per axis:   x = [ position , velocity , accel_bias ]
   covariance:       P = symmetric 3×3 (stored as its upper triangle)
```

The third state, `accel_bias`, absorbs a slowly-varying world-frame acceleration
error — gravity-removal residual, accelerometer scale/offset, small attitude
error. Without it, any constant acceleration error integrates straight into a
runaway velocity between aiding updates; with it, the filter learns and subtracts
that error (true acceleration used for integration is `a − bias`).

This is mathematically identical to the block-diagonal 9-state filter, but each
covariance is a hand-writable 3×3 — no matrix library, no allocations. Every
covariance update uses the symmetric form `P −= (P Hᵀ)(H P)/S`, which keeps `P`
symmetric exactly.

---

## 3. Predict (strapdown)

Each axis runs a constant-acceleration Kalman prediction at 100 Hz, driven by the
world-frame acceleration `a` from the AHRS. The bias state subtracts itself from
the integrated acceleration (`a − bias`), which is what the `−½dt²`/`−dt` entries
in `F` encode:

```text
   accel    = a − bias
   position += velocity·dt + ½·accel·dt²
   velocity += accel·dt
   bias      = bias                      (random walk)

   F = [[1, dt, −½dt²],     P = F · P · Fᵀ + Q
        [0,  1,   −dt ],
        [0,  0,    1  ]]
```

`Q` adds accelerometer white noise (PSD `Q_ACCEL`) to the pos/vel block plus a
small **bias random walk** (PSD `Q_ACCEL_BIAS`) on the bias state — that random
walk is what lets the bias estimate adapt over time instead of freezing. Larger
`Q_ACCEL` ⇒ trusts aiding sensors more; larger `Q_ACCEL_BIAS` ⇒ bias adapts faster
(noisier).

**World frame & gravity.** The estimator rotates the body accelerometer into the
world frame with the AHRS quaternion (`rotate_body_to_world`) and subtracts
gravity, so a level, still craft predicts zero acceleration:

```text
   a_world = R(q) · (accel_body · g) − [0, 0, g]      (world Z up)
```

---

## 4. Update (per sensor)

Each measurement is a scalar Kalman update on one axis with `H = [1,0]` (position)
or `H = [0,1]` (velocity):

```text
   innovation  y = z − H·x
   gain        K = P·Hᵀ / (H·P·Hᵀ + R)
   x += K·y ;  P = (I − K·H)·P
```

| Sensor | Fuses | Axes | Noise R (1σ) | Rate |
|---|---|---|---|---|
| GPS position | position | N, E | `R_GPS_POS · HDOP` (2.5 m) | on fix (~5 Hz) |
| GPS velocity | velocity | N, E | `R_GPS_VEL` (0.5 m/s) | on fix |
| Barometer | position | Up | `R_BARO` (1.5 m) | 10 Hz |
| Lidar (MTF-01) | position | Up | `R_LIDAR` (0.05 m) | 20 Hz |
| Optical flow | velocity | N, E | `R_FLOW / quality` (0.3 m/s) | 20 Hz |

Each measurement is applied **once per new sample** (GPS gated on a fresh NMEA
sentence; the rest at sub-rates matched to their sensor output) so the covariance
doesn't artificially collapse from re-using stale data.

### Vertical datum
Up uses a **launch reference** shared by baro (`rel_altitude_m`) and lidar (AGL,
flat-ground assumption near the ground). GPS *altitude* is intentionally **not**
fused — NMEA vertical accuracy is poor and the baro is far better. GPS only anchors
the **horizontal** origin (first 3D fix → local tangent plane).

---

## 5. Frames & the north-alignment caveat

- **World frame:** X = north, Y = east, Z = up (the AHRS frame).
- **GPS:** lat/lon projected to local north/east metres via an equirectangular
  projection about the origin (cm-accurate at UAV ranges); inverse used to
  reconstruct lat/lon for `GLOBAL_POSITION_INT`.
- **Output:** `LOCAL_POSITION_NED` (#32) and `GLOBAL_POSITION_INT` (#33) are NED
  (Z down), so Up and vUp are negated on the way out.

> **Horizontal fusion needs a calibrated compass.** GPS north/east are *true*
> geographic; the accelerometer's world X/Y come from the AHRS heading. They only
> agree if the AHRS yaw is referenced to true north (compass orientation +
> declination calibrated). Until then, **vertical fusion is fully valid** and
> horizontal fusion is correct up to a constant heading rotation. Same calibration
> dependency as the compass note in [sensor-fusion.md](sensor-fusion.md).

---

## 6. Convergence & output

The solution is flagged **converged** once an origin is set and horizontal 1σ
(`pos_std`) drops below 5 m. `GLOBAL_POSITION_INT` switches from raw-GPS passthrough
to the fused solution at that point; `LOCAL_POSITION_NED` is only emitted when
converged. The platform's **NAV · EKF** card shows FUSED vs CONVERGING.

---

## 7. Tuning quick-reference (constants in [ekf.rs](../src/ekf.rs))

| Constant | Default | Raise it to… |
|---|---|---|
| `Q_ACCEL` | 0.5 | trust aiding sensors more, IMU less (smoother, laggier) |
| `Q_ACCEL_BIAS` | 0.02 | let the accel-bias estimate adapt faster (noisier) |
| `R_GPS_POS` | 2.5 m | distrust GPS horizontal (less jumpy, slower) |
| `R_GPS_VEL` | 0.5 m/s | distrust GPS velocity |
| `R_BARO` | 1.5 m | distrust baro height (more IMU/lidar in Up) |
| `R_LIDAR` | 0.05 m | distrust lidar height |
| `R_FLOW` | 0.3 m/s | distrust optical-flow velocity |

---

## 8. Known limitations / next steps

- **Accel-bias is world-frame, not body-frame.** Each axis estimates a slowly-
  varying *world*-frame acceleration bias (§2). True accelerometer bias is fixed in
  the *body* frame and rotates with attitude, so this is an approximation — most
  accurate when yaw is stable, and physically strongest on the Up axis (gravity-
  removal residual). A body-frame bias would need the tightly-coupled formulation.
- **Flat-ground lidar.** Lidar AGL is treated as Up position; over a step/slope it
  injects error. Gate it tighter or model terrain to improve.
- **Compass calibration required for horizontal accuracy** — see §5 and
  [compass-cal.md](compass-cal.md).
- **Side TF-Luna lidars are not fused** here — they're obstacle sensors
  (`DISTANCE_SENSOR`), not nav aiding. Using them as map/avoidance constraints is a
  separate controller concern.
- **Loosely-coupled ceiling.** If you later need GPS-denied robustness or tight
  vision/IMU coupling, this is the point you'd migrate toward an ekf2-style
  tightly-coupled filter. The module boundary (`ekf.rs` consumes attitude + world
  accel + measurements) makes that swap localized.
