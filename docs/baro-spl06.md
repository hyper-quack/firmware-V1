# Barometer — Goertek SPL06-001

The board's onboard barometer, on **I2C2 at 0x76** (ArduPilot hwdef
`BARO SPL06 I2C:0:0x76`). It gives absolute air pressure → a pressure altitude,
the missing vertical reference the IMU/flow stack can't observe on its own.

- [baro.rs](../src/baro.rs) — driver
- [mavlink.rs](../src/mavlink.rs) — `SCALED_PRESSURE` (#29)

---

## Shared I2C2 bus

The SPL06 sits on the **same bus as the compass**. There is only one I2C
controller, so it can't be owned by two drivers. Both [compass.rs](../src/compass.rs)
and [baro.rs](../src/baro.rs) are therefore **bus-agnostic** — every method takes
`&mut I2C` instead of owning it — and a single `i2c_task` owns the `I2c<I2C2>`
peripheral and polls both:

- compass every loop (~100 Hz, for the attitude filter),
- baro every 5th loop (~20 Hz, comfortably above its 8 Hz conversion rate).

This is the standard embedded pattern for a shared bus and keeps all I2C access
serialized in one place (no locking, no bus contention).

---

## Driver

The SPL06 reports *raw* pressure/temperature that must be linearised with nine
per-chip calibration coefficients (`c0,c1,c00,c10,c01,c11,c20,c21,c30`) read from
registers `0x10..0x21`. `init()`:

1. checks the product id (`0x10`),
2. soft-resets and waits for `MEAS_CFG.COEF_RDY`/`SENSOR_RDY`,
3. reads + sign-extends the nine coefficients (they are 12/16/20-bit fields),
4. configures ×8 oversampling at 8 Hz for both channels and starts continuous
   measurement.

Each `read()` burst-reads the 6 raw bytes and applies the datasheet compensation
polynomial to get pressure (Pa) and temperature (°C), then:

- **Absolute (ISA) altitude** via the international barometric formula against a
  101325 Pa sea-level reference, and
- **Relative altitude** against a ground-pressure reference captured on the first
  valid sample — this is the launch-referenced height that starts at zero and is
  the genuinely useful number for a hover/altitude-hold.

> Baro altitude is **absolute/relative-to-launch**, complementary to the MTF-01
> **lidar AGL** (height above whatever is directly below, ≤8 m). The EKF step will
> blend baro (long-range, drift-free vertical reference) with the lidar (precise
> near-ground) and accel.

---

## Output + UI

Emitted as `SCALED_PRESSURE` (#29) at 10 Hz: `press_abs` (hPa), `press_diff = 0`,
`temperature` (centi-°C). To keep MAVLink semantics clean the **altitude is
computed on the consumer side** from `press_abs` (the platform captures its own
ground reference on connect), rather than abusing the `press_diff` field.

In csky_platform the **BARO · SPL06** card shows launch-relative altitude (large),
pressure, ISA altitude, and temperature, with an OK/NO-DATA badge.

Verify with QGC/pymavlink on `SCALED_PRESSURE`: `press_abs` ≈ local QNH (~1013 hPa
near sea level, lower at altitude) and `temperature` ≈ ambient; cover/uncover or
raise the board a metre and watch the altitude move.
