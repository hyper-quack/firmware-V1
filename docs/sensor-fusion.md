# Sensor Fusion & Filtering — scky-fc

How the firmware turns a box of noisy sensors into clean attitude, heading,
position, and velocity. Everything is RTIC, no heap, no blocking in the real-time
path. **This file is the map** — start here, then follow the links into each
subsystem.

---

## 0. The whole picture (read this first)

The system is now big enough to deserve one diagram. Every sensor flows into one
of **two estimators** — the **AHRS** (orientation) and the **nav EKF** (position/
velocity) — and out as MAVLink:

```text
  SENSORS                    FILTERS / DRIVERS            ESTIMATORS                 OUTPUT (MAVLink)
  ───────                    ─────────────────            ──────────                 ────────────────
  2× IMU (SPI) ─────► filters.rs ─► estimator.rs ─┬─► AHRS (ahrs.rs) ──► attitude ──► ATTITUDE, HIGHRES_IMU
  compass (I2C) ─────────────────────────────────┘     Mahony 9-DOF        │
                                                        (roll/pitch/yaw)    │ world accel
                                                                            ▼
  GPS (UART) ─────► gps.rs ──────────────────────┐                   ┌────────────┐
  baro (I2C) ─────► baro.rs ──────────────────────┼── measurements ─►│ nav EKF    │─► LOCAL_POSITION_NED
  MTF-01 flow+lidar (UART) ─► mtf01.rs ─► nav.rs ─┤                   │ (ekf.rs)   │   GLOBAL_POSITION_INT
                                                  │                   └────────────┘
  (TF-Luna ×2 side lidar ─► tfluna.rs ─► obstacle/avoidance, DISTANCE_SENSOR — not fused into nav)
  (ExpressLRS RX ─► crsf.rs ─► RC_CHANNELS — control input, not an estimator)
```

Two estimators, deliberately **separate** (loosely-coupled):

| Estimator | Module | Estimates | Inputs | Doc |
|---|---|---|---|---|
| **AHRS** | [ahrs.rs](../src/ahrs.rs) | orientation (roll/pitch/yaw) | gyro, accel, mag | §1–§7 below |
| **nav EKF** | [ekf.rs](../src/ekf.rs) | position, velocity | AHRS + GPS, baro, lidar, flow | [ekf.md](ekf.md) |

### Where each thing is documented

| Subsystem | Code | Doc |
|---|---|---|
| Digital filters (low-pass / notch) | [filters.rs](../src/filters.rs) | §3–§4 below |
| Attitude (Mahony, incl. mag heading) | [ahrs.rs](../src/ahrs.rs), [estimator.rs](../src/estimator.rs) | §1–§7 below |
| Position/velocity EKF | [ekf.rs](../src/ekf.rs) | [ekf.md](ekf.md) |
| GPS + compass | [gps.rs](../src/gps.rs), [compass.rs](../src/compass.rs) | [gps-compass.md](gps-compass.md) |
| Compass calibration (rotation / iron / declination) | [compass.rs](../src/compass.rs), [estimator.rs](../src/estimator.rs) | [compass-cal.md](compass-cal.md) |
| Barometer | [baro.rs](../src/baro.rs) | [baro-spl06.md](baro-spl06.md) |
| Optical flow + down lidar | [mtf01.rs](../src/mtf01.rs), [nav.rs](../src/nav.rs) | [mtf01-elrs.md](mtf01-elrs.md) |
| ExpressLRS RC | [crsf.rs](../src/crsf.rs) | [mtf01-elrs.md](mtf01-elrs.md) |
| Side obstacle lidars | [tfluna.rs](../src/tfluna.rs) | [proximity-tfluna.md](proximity-tfluna.md) |

> **One calibration caveat spans both estimators:** heading accuracy needs the
> compass calibrated (mount rotation, hard/soft-iron, declination). The mechanism
> exists — constants in [main.rs](../src/main.rs) plus an in-field RC-triggered
> hard-iron collector — but the *values* must be set/collected on hardware. Until
> then, **heading** (and therefore EKF **horizontal** position/velocity, which is
> rotated by heading) is correct only up to a constant offset; **vertical** fusion
> and roll/pitch are unaffected. See [compass-cal.md](compass-cal.md) and
> [ekf.md](ekf.md) §5.

The rest of this file details the attitude half (the EKF half is in
[ekf.md](ekf.md)).

---

## 1. Pipeline overview

```
                 1 kHz                         1 kHz                   1 kHz
  ┌─────────┐  per-axis LPF   ┌──────────┐   rotate +    ┌──────────────────┐
  │ IMU1    │──gyro/accel────►│  ImuLpf  │──►ImuOut1 ──┐ combine           │
  │ (SPI1)  │                 └──────────┘             ├─►│ Estimator        │
  └─────────┘                                          │  │  ├ body-frame    │
  ┌─────────┐                 ┌──────────┐             │  │  ├ vote/average  │──►Attitude
  │ IMU2    │──gyro/accel────►│  ImuLpf  │──►ImuOut2 ──┘  │  └ Mahony AHRS    │   (roll,
  │ (SPI4)  │                 └──────────┘                └──────────────────┘    pitch,
  └─────────┘                                                                     yaw, q,
                                                                                  rates, bias)
                                                                                     │
                                                                                     ▼
                                                                          usb_task  (~10 Hz print)
```

Each stage and the RTIC task it runs in:

| Stage | Where | Rate | Task / priority |
|---|---|---|---|
| Raw SPI read | `Imu::read` | 1 kHz | `imu1_task` / `imu2_task` (**prio 3**) |
| Per-axis low-pass | `ImuLpf::apply` | 1 kHz | same as above |
| Mount rotation + combine | `Estimator::update` | 1 kHz | `estimator_task` (**prio 2**) |
| Attitude filter | `Mahony::update` | 1 kHz | `estimator_task` |
| Telemetry | `usb_task` | ~10 Hz | `usb_task` (**prio 1**) |

Priorities guarantee the sampling can preempt fusion, and both preempt USB —
so logging can never perturb the estimate.

---

## 2. Mapping to PX4

This is intentionally the PX4 *structure*, scaled down to what this board can
currently sense (IMUs only — no GPS/mag fused yet).

| PX4 module | scky-fc equivalent | Notes |
|---|---|---|
| `sensors` (per-sensor rotation, voting, combining) | `estimator.rs` | rotation + average of healthy IMUs |
| `LowPassFilter2p`, `NotchFilter` in the gyro/accel pipe | `filters.rs` | same coefficient math |
| `attitude_estimator_q` (quaternion complementary filter) | `ahrs.rs` (`Mahony`) | same algorithm class |
| `ekf2` (24-state EKF: pos/vel/att/bias/wind) | *roadmap* — see §8 | needs GPS/baro/mag |

PX4 runs `ekf2` as its production estimator, but its lighter
`attitude_estimator_q` is exactly a Mahony-style quaternion complementary
filter — which is what we implement here. It is robust, cheap, deterministic,
and the correct first estimator for an IMU-only bring-up.

---

## 3. Low-pass filter (`LowPassFilter2p`)

A 2nd-order Butterworth, identical to PX4's `LowPassFilter2p`. It removes
high-frequency sensor/vibration noise before fusion (and, conceptually,
anti-aliases the signal we integrate).

Coefficients for sample rate `fs` and cutoff `fc`:

```
fr  = fs / fc
ohm = tan(pi / fr)
c   = 1 + 2·cos(pi/4)·ohm + ohm²
b0  = ohm²/c      b1 = 2·b0      b2 = b0
a1  = 2·(ohm²−1)/c
a2  = (1 − 2·cos(pi/4)·ohm + ohm²)/c
```

Direct-form-I update with two delay elements:

```
d0     = x − a1·d1 − a2·d2
y      = b0·d0 + b1·d1 + b2·d2
d2 = d1 ;  d1 = d0
```

Defaults (in `main.rs`): `fs = 1000 Hz`, **gyro cutoff 80 Hz**, **accel cutoff
20 Hz**. Accel is filtered harder because the only thing we use it for is the
gravity direction, which is essentially DC; the gyro carries the fast motion so
it keeps a higher corner. `fc ≤ 0` or `fc ≥ fs/2` disables a filter
(pass-through).

---

## 4. Notch filter (`NotchFilter`)

A 2nd-order biquad notch matching PX4's `NotchFilter`, for rejecting a narrow
vibration band (prop wash / motor harmonics):

```
alpha  = tan(pi·BW / fs)
beta   = −cos(2·pi·f_notch / fs)
a0i    = 1/(alpha+1)
b0 = a0i     b1 = 2·beta·a0i     b2 = a0i
a1 = b1      a2 = (1−alpha)·a0i
```

It is **implemented but not yet in the live path**: a useful notch needs a target
frequency, which normally tracks motor RPM (ESC telemetry) or a dynamic-notch
estimator — neither exists until the motor-output stage lands. The code +
coefficients are ready to drop in. See [filters.rs](../src/filters.rs).

---

## 5. Mount rotation + dual-IMU combine (`Estimator`)

The hwdef mounts the two IMUs differently:

- IMU1 → `ROTATION_ROLL_180` → `(x, −y, −z)`
- IMU2 → `ROTATION_PITCH_180` → `(−x, y, −z)`

Both gyro and accel vectors are rotated into the **common body frame** first —
otherwise averaging them would be nonsense. Then:

- **both healthy** → element-wise average (simple equal-weight vote),
- **one healthy** → use it,
- **neither** → hold `gyro = 0`, `accel = +1 g z` (level), so the filter coasts.

This is the small-scale version of PX4's sensor voting/combining. A richer
version (innovation-based weighting, disagreement detection, dropping a diverging
sensor) is a natural next step.

---

## 6. Attitude filter (`Mahony`)

A quaternion complementary filter on SO(3). Per step (gyro `ω` in rad/s, accel
normalized to unit gravity `â`, step `dt`):

1. **Estimated gravity** from the current quaternion `q = [q0,q1,q2,q3]`:
   ```
   v = ( 2(q1q3 − q0q2),  2(q0q1 + q2q3),  q0² − q1² − q2² + q3² )
   ```
2. **Error** = measured × estimated gravity:  `e = â × v`
3. **Feedback** into the gyro:
   ```
   bias  += 2·Ki · e · dt      (integral  → gyro-bias estimate)
   ω     += bias + 2·Kp · e    (proportional + bias correction)
   ```
4. **Integrate** the quaternion and renormalize:
   ```
   q̇ = ½ · q ⊗ (0, ωx, ωy, ωz)
   q  = normalize(q + q̇·dt)
   ```

Gains (in `main.rs`): `Kp = 1.0`, `Ki = 0.05`. Raise `Kp` for snappier accel
tracking (more noise sensitivity); raise `Ki` for faster bias learning (risk of
windup under sustained acceleration).

On the first valid accel sample the quaternion is **seeded** from the measured
gravity (roll/pitch), so the estimate is correct immediately instead of
converging from "level" over a second.

Euler output (ZYX):
```
roll  = atan2(2(q0q1 + q2q3), 1 − 2(q1² + q2²))
pitch = asin (2(q0q2 − q3q1))            // clamped at ±90° (gimbal lock)
yaw   = atan2(2(q0q3 + q1q2), 1 − 2(q2² + q3²))
```

---

## 7. Heading: mag-aided yaw (Phase 2)

Gravity gives an **absolute** reference for roll and pitch — tilt is directly
observable from the accelerometer — so those two are corrected every step and do
**not** drift. Yaw (heading) is rotation *about* the gravity vector, so gravity
says nothing about it.

With the external compass now wired ([gps-compass.md](gps-compass.md)), the
`Mahony::update` step takes an optional magnetometer vector and adds the standard
**9-DOF heading-correction term**: the cross-product between the measured field
and the field direction predicted from the current quaternion, folded so it
constrains only heading (not tilt). The earth-frame field is split into a
horizontal reference `bx` and vertical `bz`, so a tilted compass still yields the
correct heading. The result feeds the same PI loop as the accel term.

Effect: when a healthy compass is present, **yaw is absolute and does not drift**.
When the compass is absent or unhealthy, `update` is called with `mag = None` and
the filter degrades gracefully to the 6-DOF accel-only form (yaw = gyro
integration, drifts) — no code path change, just a missing correction.

> **Not yet tuned on hardware:** the mag is fused in its raw sensor frame assuming
> the compass is mounted aligned with the board. Real units need (a) a mount
> rotation/sign map (ArduPilot's `COMPASS_ORIENT`), (b) hard/soft-iron
> calibration, and (c) magnetic declination to convert magnetic → true north.
> These are bench-calibration steps for when the module is powered and spinning.

The fused attitude is emitted as MAVLink `ATTITUDE` (#30) and the heading also
rides in `GLOBAL_POSITION_INT` (#33), so ground stations show a real fused
heading rather than integrating one themselves.

---

## 8. Roadmap to a full EKF (PX4 `ekf2`-style)

The complementary filter estimates attitude only. A full navigation EKF adds
position/velocity and fuses more sensors. Target state vector (PX4 ekf2):

| States | Meaning | Driving sensor |
|---|---|---|
| q (4) | orientation | gyro (predict), accel/mag (update) |
| v (3) | NED velocity | accel (predict), GPS/flow (update) |
| p (3) | NED position | velocity (predict), GPS/baro (update) |
| b_g (3) | gyro bias | estimated online |
| b_a (3) | accel bias | estimated online |
| (wind, mag field…) | extended states | airspeed, mag |

Prerequisites before that's worth doing here: **baro** (SPL06 on I2C2, driver not
yet written), **magnetometer** (external), and ideally **GPS**. Until then the
Mahony filter is the right estimator. The module boundaries (`filters` →
`estimator` → consumer) are arranged so the estimator can be swapped without
touching the sampling or telemetry tasks.

---

## 9. Tuning quick-reference

All constants live at the top of the `app` module in [main.rs](../src/main.rs):

| Constant | Default | Effect |
|---|---|---|
| `GYRO_CUTOFF_HZ` | 80 | lower = smoother gyro, more phase lag |
| `ACCEL_CUTOFF_HZ` | 20 | lower = steadier roll/pitch, slower to react |
| `AHRS_KP` | 1.0 | higher = trusts accel more (faster, noisier) |
| `AHRS_KI` | 0.05 | higher = faster gyro-bias learning |
| `SAMPLE_HZ` / `DT` | 1000 / 1 ms | task tick; keep in sync with the `Mono::delay` periods |
