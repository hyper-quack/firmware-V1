# Side obstacle lidars — TF-Luna ×2

Two Benewake **TF-Luna** single-point lidars mounted on the **left and right** of
the airframe for horizontal obstacle / collision avoidance (range 0.2–8 m).

- [tfluna.rs](../src/tfluna.rs) — UART frame parser
- [mavlink.rs](../src/mavlink.rs) — `DISTANCE_SENSOR` (#132), per-orientation

---

## 1. Why UART, one per sensor

The TF-Luna supports UART **or** I2C. We use **UART (default, 115200)**:

- it's the factory default — the sensor auto-streams frames with zero setup,
- there's no address-collision problem (both TF-Lunas ship as I2C 0x10; I2C would
  require reconfiguring one sensor's address and would also pile onto the single
  shared I2C2 bus already carrying the compass + baro),
- the H7 has spare UARTs, so each sensor gets a dedicated, isolated link.

| Sensor | UART | MCU pins | Baud |
|---|---|---|---|
| **Left** | USART6 | PC6 TX / **PC7 RX** | 115200 |
| **Right** | UART7 | **PE7 RX** / PE8 TX | 115200 |

Only the FC **RX** pin is needed (the sensor streams unprompted); TX is wired but
unused. Power from 5 V / GND.

> To use I2C instead (e.g. if you run short on UARTs), the TF-Luna's I2C address is
> reconfigurable — but you'd set the two to different addresses and add them to the
> `i2c_task` poll loop on I2C2. UART was chosen to keep them independent.

---

## 2. Frame format

The TF-Luna streams a fixed **9-byte** frame at 100 Hz:

```text
0x59 0x59 Dist_L Dist_H Amp_L Amp_H Temp_L Temp_H Checksum
```

`Checksum = sum(byte[0..8]) & 0xFF`. [`TfLunaParser`](../src/tfluna.rs) is a byte-fed
state machine (header-sync → body → checksum) feeding off the UART RX interrupt
(priority 4, same pattern as every other UART here).

A reading is flagged **valid** only when the amplitude is trustworthy
(`100 ≤ amp ≠ 0xFFFF`, rejecting weak returns and saturation) and the distance is
in the 20–800 cm working range.

---

## 3. Output + UI

Each sensor is emitted as a `DISTANCE_SENSOR` (#132) at 15 Hz, distinguished by the
standard `MAV_SENSOR_ORIENTATION` + `id` fields:

| Sensor | orientation | id |
|---|---|---|
| Down (MTF-01 lidar) | 25 (down) | 0 |
| Left (TF-Luna) | 6 (yaw 270 / left) | 1 |
| Right (TF-Luna) | 2 (yaw 90 / right) | 2 |

Side lidars are sent **even when out of range** (reported at max range) so the
ground station can distinguish "clear" from "stale link".

In csky_platform the **PROXIMITY · TF-LUNA** card shows a top-down drone glyph with
left/right clearance bars that shorten and change colour as obstacles approach:
green (clear) → amber (< 1.5 m) → red (< 0.5 m), with a TOO CLOSE / NEAR / CLEAR
badge. The platform routes `DISTANCE_SENSOR` by `orientation` — down → AGL altitude,
left/right → proximity.

Verify with QGC/pymavlink on `DISTANCE_SENSOR`: watch `current_distance` track a
hand approaching each sensor, and confirm `orientation`/`id` differ per sensor.

---

## 4. Board UART map after this phase

| UART | Use | Pins |
|---|---|---|
| USART1 | GPS | PA9 / PA10 |
| USART2 | MTF-01 flow+lidar | PD5 / PD6 |
| UART5 | ExpressLRS (CRSF) | PB5 / PB6 |
| USART6 | TF-Luna left | PC6 / PC7 |
| UART7 | TF-Luna right | PE7 / PE8 |
| **UART8** | **free** | PE0 / PE1 |

USART3 (ESC telem) and UART4 (MSP DisplayPort) remain per their hwdef roles; UART8
is the only fully free spare left.
