# Compass calibration

A magnetometer gives a *heading* only after three corrections. Skipping them is
the usual reason a compass "points the wrong way" or the EKF's horizontal position
drifts in the wrong direction. All three live in [`MagCal`](../src/compass.rs) plus
a declination term in [`estimator.rs`](../src/estimator.rs).

```text
   raw mag ──► [ hard-iron offset ] ──► [ soft-iron scale ] ──► [ mount rotation ] ──► body field
                                                                                          │
   (fused for heading in the AHRS)  ◄── [ + declination → true north ] ◄──────────────────┘
```

---

## 1. Mount rotation — orientation

The compass usually sits on the GPS mast, rotated relative to the flight
controller. [`MagRotation`](../src/compass.rs) maps the sensor axes into the body
frame: `None`, `Yaw90`, `Yaw180`, `Yaw270`, or `Roll180` (upside-down). Set it with
`MAG_ROTATION` in [main.rs](../src/main.rs).

**How to find it:** with the firmware running, point the nose to magnetic north and
watch the heading; rotate the craft 90° clockwise and confirm the heading
increases by ~90°. If it jumps the wrong way or by the wrong amount, change the
rotation until nose-right gives heading-right.

## 2. Hard-iron & soft-iron

Nearby steel and magnets bias the field (**hard-iron** = a constant offset) and
distort its shape (**soft-iron** = axis-dependent scale). Uncorrected, the heading
swings sinusoidally as the craft yaws.

`MagCal` removes them as `corrected = scale ⊙ (raw − offset)`, applied **in the
sensor frame before the mount rotation** (the iron is fixed to the sensor). Set
`MAG_OFFSET` / `MAG_SCALE` from a bench calibration, or use the in-field collector:

### In-field collector (RC-triggered)

`MagCal` has an online min/max collector wired to an RC AUX switch
(`CAL_RC_CHANNEL`, default channel 6) in `estimator_task`:

1. Flip the AUX switch **high** → `start_collection()`.
2. Rotate the craft slowly through **all** orientations (figure-8s, every axis).
3. Flip the switch **low** → `finish_collection()` sets
   `offset = (max+min)/2` per axis and `scale` to equalise the axis ranges.

Centre stick (~1500 µs) does nothing, so a lost RC link can't start a calibration.
The result applies immediately; persisting it across reboots (flash storage) is a
later addition — for now copy the converged values into `MAG_OFFSET`/`MAG_SCALE`.

## 3. Declination — magnetic vs true north

The compass measures heading relative to **magnetic** north; GPS works in **true**
north. The angle between them (declination) varies by location — tens of degrees in
some places. Until it's applied, the AHRS heading and the EKF's GPS-referenced
horizontal axes disagree by exactly that angle.

Set `MAG_DECLINATION_DEG` (east-positive) for your location (look it up by lat/lon
on any declination calculator). The estimator adds it to the reported yaw **and**
rotates the world-frame acceleration it hands the EKF, so heading, flow, and EKF
all share true north.

---

## 4. Why this matters for the EKF

This is the calibration the EKF's *horizontal* fusion depends on (see
[ekf.md](ekf.md) §5): GPS north/east only line up with the accelerometer-derived
world X/Y once heading is true-north-referenced. **Vertical** fusion (baro / lidar /
accel-Z) and roll/pitch never depend on any of this, so they're correct even with
an uncalibrated compass.

## 5. Sign-check on the bench (no flight needed)

| Symptom | Fix |
|---|---|
| Heading rotates the wrong way as you yaw | wrong `MAG_ROTATION` |
| Heading swings ± as you yaw level | hard-iron — run the collector |
| Heading off by a constant ~5–20° | set `MAG_DECLINATION_DEG` |
| EKF position drifts at an angle to motion | heading not true-north (rotation or declination) |
