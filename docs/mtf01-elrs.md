# MTF-01 flow/lidar + ExpressLRS RC (Phase 3)

Adds the last two external links: the **MicoAir MTF-01** (downward optical flow +
lidar) for height and horizontal motion, and the **HappyModel ExpressLRS 900 MHz**
receiver for RC control. Both are UART devices; both RX paths are interrupt-driven
like the GPS.

- [mtf01.rs](../src/mtf01.rs) — MTF-01 MSP v2 parser (USART2)
- [crsf.rs](../src/crsf.rs) — CRSF parser for ExpressLRS (UART5)
- [nav.rs](../src/nav.rs) — flow + lidar dead-reckoning
- [mavlink.rs](../src/mavlink.rs) — `DISTANCE_SENSOR`, `OPTICAL_FLOW`, `RC_CHANNELS`

---

## 1. Wiring

| Device | Wire | MCU pin | Bus | Baud |
|---|---|---|---|---|
| MTF-01 | TX → FC RX | **PD6** (USART2_RX) | USART2 | 115200 |
| MTF-01 | RX → FC TX | **PD5** (USART2_TX) | USART2 | (unused for now) |
| ELRS RX | TX → FC RX | **PB5** (UART5_RX) | UART5 | 420000 |
| ELRS RX | RX → FC TX | **PB6** (UART5_TX) | UART5 | (telemetry, future) |

Power both from 5 V / GND. The ELRS UART5 pins come from the hwdef
`SERIAL5 = RCIN`; USART2 was the spare UART chosen for the MTF-01.

---

## 2. MTF-01 — MSP v2 (optical flow + lidar)

The MTF-01 must be set to **MSP output mode** in its configurator (it also offers
MAVLink; MSP is simpler to parse and is what this driver expects). In MSP mode it
*pushes* two INAV-style sensor messages at ~100 Hz without being polled:

| MSP function | Hex | Payload |
|---|---|---|
| `MSP2_SENSOR_RANGEFINDER` | 0x1F01 | quality (u8), distance_mm (i32) |
| `MSP2_SENSOR_OPTIC_FLOW` | 0x1F02 | quality (u8), motionX (i32), motionY (i32) |

[`MspParser`](../src/mtf01.rs) is a byte-fed MSP v2 state machine with CRC-8/DVB-S2
validation (shared with CRSF). A negative rangefinder distance means out-of-range.

### Height + flow fusion ([nav.rs](../src/nav.rs))

- **Height (AGL):** the lidar slant range is projected onto the vertical axis with
  the attitude tilt — `h = range · cos(roll)·cos(pitch)` — and gated to the
  0.05–8 m working range.
- **Horizontal velocity:** optical flow is an *angular* ground-motion rate, so
  `v = ω · h`. The body rotation rate is removed first (gyro de-rotation using the
  bias-corrected body rates), then the result is rotated into the earth frame by
  heading and integrated to a relative position.

> **Two hardware-calibration constants** in `nav.rs` need bench values once the
> sensor is powered: `FLOW_SCALE` (raw MSP units → rad/s) and the flow axis/sign
> pairing (mount-dependent). Until then position is qualitatively right but not
> metrically calibrated. This is dead-reckoning — it drifts without GPS/EKF
> correction (the EKF fusion is the planned next step).

Emitted as `DISTANCE_SENSOR` (#132, 10 Hz, facing-down), `OPTICAL_FLOW` (#100,
20 Hz), and the lidar height also fills `GLOBAL_POSITION_INT.relative_alt` so the
ground station's AGL reads from the lidar.

---

## 3. ExpressLRS — CRSF RC

The HappyModel ELRS 900 RX speaks **CRSF at 420 kbaud**. [`CrsfParser`](../src/crsf.rs)
decodes:

| Frame | Type | Contents |
|---|---|---|
| RC_CHANNELS_PACKED | 0x16 | 16 channels × 11 bits (172…1811, 992 = centre) |
| LINK_STATISTICS | 0x14 | uplink RSSI (-dBm) + **link quality (0…100 %)** |

Channels are converted to standard 988–2012 µs and sent as `RC_CHANNELS` (#65,
10 Hz). The CRSF **link quality** is packed into the message's RSSI byte — it's the
primary ELRS health metric. Link loss is detected by frame-staleness (the ground
station shows "failsafe" when frames stop for >1 s).

> The TX side (UART5 PB6) is wired but unused for now — it's reserved for CRSF
> telemetry back to the handset. The transmitter is a HappyModel ELRS 900 TX bound
> to a FlySky handset; binding/model setup happens on the TX, not the FC.

---

## 4. Verify (with QGC / pymavlink)

| What | Message | Check |
|---|---|---|
| Lidar height | `DISTANCE_SENSOR` | `current_distance` (cm) tracks hand distance to a surface |
| Flow | `OPTICAL_FLOW` | `quality` > 0 over texture; `flow_comp_m_x/y` respond to sliding the board |
| RC | `RC_CHANNELS` | sticks move `chan1..4`; `rssi` = link quality; stops on TX off |

In csky_platform: the **ALTITUDE · FLOW** card shows the AGL tape + flow velocity/
quality; the **RC · ELRS 900** card shows channel bars + link quality + failsafe.

---

## 5. Updated board placement map

| Component | Bus | MCU pins | Addr / detail | Status |
|---|---|---|---|---|
| IMU1 (ICM-42688-P) | SPI1 | PA5/PA6/PA7 | CS PA4 | ✅ |
| IMU2 (ICM-42688-P) | SPI4 | PE12/PE13/PE14 | CS PB1 | ✅ |
| USB CDC (MAVLink) | OTG2_FS | PA11/PA12 | — | ✅ |
| GPS (NEO-M8N) | USART1 | PA9 / PA10 | 9600 NMEA | ✅ |
| Compass (QMC/HMC5883) | I2C2 | PB10 / PB11 | 0x0D / 0x1E | ✅ |
| **MTF-01 (flow + lidar)** | **USART2** | **PD5 / PD6** | 115200 MSP | ✅ **Phase 3** |
| **ExpressLRS 900 RX** | **UART5** | **PB5 / PB6** | 420000 CRSF | ✅ **Phase 3** |
| Barometer (SPL06) | I2C2 | PB10/PB11 (shared) | 0x76 | ✅ (see [baro-spl06.md](baro-spl06.md)) |
| OSD | SPI2 | PB13/14/15 | CS PB12 | — not planned |
| Dataflash | SPI3 | PC10/11/12 | CS PA15 | — not planned |

Spare UARTs remaining: USART6 (PC6/7), UART7 (PE7/8), UART8 (PE0/1).
