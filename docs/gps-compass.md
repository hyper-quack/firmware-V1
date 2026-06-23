# GPS + Compass bring-up (Phase 1)

This is the first half of adding absolute heading/position to scky-fc: get the
**uBlox NEO-M8N GPS** and its **HMC5883/QMC5883 compass** reading reliably and
streamed over MAVLink so the data can be verified *before* any fusion is built on
top of it. Fusion (EKF2 yaw/position) is Phase 2 and is intentionally not wired
yet — see [sensor-fusion.md](sensor-fusion.md).

- [gps.rs](../src/gps.rs) — USART1 NMEA parser
- [compass.rs](../src/compass.rs) — I2C2 magnetometer (auto-detects QMC/HMC)
- [mavlink.rs](../src/mavlink.rs) — `GPS_RAW_INT` + mag in `HIGHRES_IMU`
- [main.rs](../src/main.rs) — RTIC wiring

---

## 1. Wiring

The module is the common "Ublox NEO-M8N + HMC5883" Pixhawk-style unit: a 6-pin
GPS connector (UART + power) and a separate compass on I2C.

| Module wire | Connects to | MCU pin | Notes |
|---|---|---|---|
| GPS TX | FC RX | **PA10** (USART1_RX) | GPS → FC |
| GPS RX | FC TX | **PA9** (USART1_TX) | FC → GPS (only used by Phase-2 UBX config) |
| Compass SCL | I2C2 SCL | **PB10** | shared bus with SPL06 baro |
| Compass SDA | I2C2 SDA | **PB11** | shared bus with SPL06 baro |
| VCC | 5 V | — | GPS+compass both off 5 V |
| GND | GND | — | |

> **Pull-ups:** I2C2 is configured open-drain with **no internal pull-ups**. The
> GPS module carries its own SCL/SDA pull-ups, so this is fine with the module
> attached. If you ever drive I2C2 with a bare sensor, add ~4.7 kΩ pull-ups.

The pin assignment comes straight from the ArduPilot hwdef
(`SERIAL1 = GPS` on USART1; `I2C_ORDER I2C2`).

---

## 2. GPS driver — NMEA over USART1

uBlox modules power up emitting **NMEA-0183** ASCII at **9600 baud** with no
configuration, so Phase 1 parses NMEA rather than UBX binary — "plug it in and
watch the fix appear." Two sentences are decoded and merged:

| Sentence | Fields used |
|---|---|
| `GGA` | fix quality, satellites, latitude, longitude, MSL altitude, HDOP |
| `RMC` | ground speed (knots → cm/s), course over ground |

The parser ([`NmeaParser`](../src/gps.rs)) is a byte-fed state machine: it buffers
one `$…*CC` sentence, verifies the XOR checksum, then fills a [`GpsData`] whose
units map 1:1 onto MAVLink `GPS_RAW_INT` (lat/lon in 1e7-deg, alt in mm, etc.).

**RX is interrupt-driven.** The STM32H7 USART has only a one-byte receive buffer,
so a 1 kHz polled reader would drop bytes at GPS baud. Instead the `USART1`
interrupt (RTIC hardware task, **priority 4** — above IMU sampling) drains every
byte the instant it arrives. The ISR is tiny (feed one byte into the parser), so
running it above the IMU loop costs only a few microseconds of jitter, and a UART
overrun — which *is* unrecoverable — can never happen.

> **Changing baud:** if your module was pre-set to another rate (38400 is common
> on some Pixhawk units), edit `GPS_BAUD` in [main.rs](../src/main.rs) and rebuild.
> A `$…GGA` line with a valid checksum but wrong baud will just never appear.

### UBX / velocity (Phase 2)

NMEA gives horizontal speed and course but not the 3D NED velocity an EKF wants.
Phase 2 will send a UBX `CFG` burst over `PA9` to enable `NAV-PVT` (lat/lon/alt +
velN/E/D + accuracy in one binary message) and parse that instead. The TX pin and
shared-data plumbing are already in place for it.

---

## 3. Compass driver — magnetometer over I2C2

Modules labelled "HMC5883" overwhelmingly ship a **QMC5883L** clone (I2C `0x0D`),
not a genuine Honeywell **HMC5883L** (`0x1E`). [`Compass::init`](../src/compass.rs)
probes both:

- **QMC5883L** — identified by chip-id reg `0x0D == 0xFF`; configured continuous,
  8 G range, 200 Hz; data little-endian, axis order X,Y,Z; ~3000 LSB/Gauss.
- **HMC5883L** — identified by id regs `0x0A..0x0C == "H43"`; configured 8-avg,
  15 Hz, continuous; data big-endian, axis order **X,Z,Y**; 1090 LSB/Gauss.

Whichever answers is read at **~75 Hz** by `compass_task` (priority 1) and
published as [`MagData`] in **Gauss**, board frame. If neither answers, `kind` is
`None` and the task publishes an unhealthy reading without touching the bus (so a
missing compass never stalls I2C).

Absolute scale only has to be roughly right for bring-up — the test is that all
three axes swing sensibly as you rotate the board. Hard/soft-iron calibration and
turning the vector into a heading happen in the fusion phase.

---

## 4. What goes out on MAVLink

Over the USB CDC link (same MAVLink 2 stream as the IMUs):

| Message | ID | Rate | Carries |
|---|---|---|---|
| `GPS_RAW_INT` | 24 | 5 Hz | fix type, sats, lat, lon, MSL alt, HDOP, ground speed, course |
| `HIGHRES_IMU` (id 0) | 105 | 20 Hz | IMU0 accel/gyro **+ magnetometer** (`xmag/ymag/zmag`, Gauss) |

The magnetometer rides in IMU0's `HIGHRES_IMU` with the `XMAG/YMAG/ZMAG`
fields_updated bits set, exactly as PX4 reports an integrated mag.

---

## 5. How to verify (no fusion needed)

The new sensors are connected, so you can confirm them directly:

**A. csky_platform** — the **GPS · COMPASS** card (`GpsCompassCard`) shows a
heading rose, octant, fix status, satellite count, and lat/lon/alt/HDOP. Heading
comes from the FC's fused `ATTITUDE` yaw (mag-aided); GPS fields from
`GPS_RAW_INT` / `GLOBAL_POSITION_INT`. The NavigationHUD compass tape is driven by
the same fused heading.

**B. QGroundControl** — MAVLink Inspector → `GPS_RAW_INT` shows `satellites_visible`
climbing and `fix_type` reaching 3; `HIGHRES_IMU` shows non-zero `xmag/ymag/zmag`
that change as you rotate the board.

**C. Raw / pymavlink** — point a connection at `/dev/ttyACM0` and watch:

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
while True:
    msg = m.recv_match(type=['GPS_RAW_INT', 'HIGHRES_IMU'], blocking=True)
    print(msg)
```

Note: the USB stream is **binary MAVLink**, so a plain `picocom`/`cat` shows
garbage — use a MAVLink reader. (If you specifically want human-readable ASCII for
`picocom`, that's a separate debug build; ask and I'll add a text mode.)

Indoors a GPS will report `fix_type = 0` and 0 sats forever — verify near a window
or outside. The compass works indoors.

---

## 6. Where each component sits — board map

The full peripheral placement for the project (current + planned), so wiring is
mapped in one place:

| Component | Bus | MCU pins | Addr / CS | Status |
|---|---|---|---|---|
| IMU1 (ICM-42688-P) | SPI1 | PA5/PA6/PA7 | CS PA4 | ✅ working |
| IMU2 (ICM-42688-P) | SPI4 | PE12/PE13/PE14 | CS PB1 | ✅ working |
| USB CDC (MAVLink) | OTG2_FS | PA11/PA12 | — | ✅ working |
| **GPS (NEO-M8N)** | **USART1** | **PA9 TX / PA10 RX** | 9600 baud | ✅ **Phase 1** |
| **Compass (QMC/HMC5883)** | **I2C2** | **PB10 SCL / PB11 SDA** | 0x0D / 0x1E | ✅ **Phase 1** |
| Barometer (SPL06) | I2C2 | PB10/PB11 (shared) | 0x76 | ⏳ planned (EKF alt) |
| MTF-01 (flow + lidar) | USART2 | PD5 TX / PD6 RX | 115200 | ⏳ Phase 3 |
| OSD | SPI2 | PB13/14/15 | CS PB12 | — not planned |
| Dataflash | SPI3 | PC10/11/12 | CS PA15 | — not planned |

Spare UARTs still free for future use: USART6 (PC6/7), UART7 (PE7/8),
UART8 (PE0/1), plus the ESC-telem/MSP/RC ports.

---

## 7. Logging / debug

Everything is observable on the live MAVLink stream (§5) — there is no silent
state. `GPS_RAW_INT.satellites_visible`/`fix_type` and the `HIGHRES_IMU` mag
fields are the bring-up health indicators. Sentence-decode liveness is also
tracked internally (`GpsData.sentences` increments per valid NMEA sentence) and
can be surfaced in a custom status message if needed during debugging.
