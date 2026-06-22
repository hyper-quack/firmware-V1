# MAVLink 2 telemetry contract

The flight controller emits an unsigned MAVLink 2 byte stream over its USB
CDC-ACM port. The link is binary: do not decode it as lines or UTF-8. USB CDC
baud-rate settings are ignored.

## Link identity

| Property | Value |
|---|---:|
| MAVLink version | 2 (`0xFD` frame marker) |
| System ID | 1 |
| Component ID | 1 (`MAV_COMP_ID_AUTOPILOT1`) |
| Signing | Disabled for this local USB link |

## Messages sent

| Message | ID | Rate | Purpose |
|---|---:|---:|---|
| `HEARTBEAT` | 0 | 1 Hz | Link/liveness detection; quadrotor, active, not armed |
| `SYS_STATUS` | 1 | 1 Hz | Aggregate accel/gyro present, enabled, and health bits |
| `HIGHRES_IMU` | 105 | 20 Hz per connected IMU | Filtered acceleration and angular velocity in SI units |
| `SCKY_IMU_STATUS` | 42000 | 1 Hz per IMU | Per-device connection, health, and `WHO_AM_I` |

`HIGHRES_IMU.id` and `SCKY_IMU_STATUS.imu_id` use the same zero-based IDs:

| ID | Physical device | Bus | Mount correction already applied |
|---:|---|---|---|
| 0 | IMU1 | SPI1 | Roll 180 degrees |
| 1 | IMU2 | SPI4 | Pitch 180 degrees |

`HIGHRES_IMU` values are already expressed in the shared aircraft body frame:

- `xacc`, `yacc`, `zacc`: m/s²
- `xgyro`, `ygyro`, `zgyro`: rad/s
- `fields_updated = 0x003F`: only accel and gyro are valid
- magnetometer, pressure, altitude, and temperature fields: `NaN`
- `time_usec`: microseconds since firmware boot

Only healthy IMUs emit `HIGHRES_IMU`. Always use `SCKY_IMU_STATUS` for explicit
connection state; also consider an IMU stale if no status arrives for 3 seconds.
In this first firmware slice, `connected` means a supported InvenSense device
answered `WHO_AM_I` at boot, and `healthy` means it is accepted by the estimator.

## Custom status schema

The canonical dialect is
[`message_definitions/scky.xml`](../message_definitions/scky.xml). Its custom
message has CRC extra 38 and this decoded shape:

```ts
type SckyImuStatus = {
  time_boot_ms: number; // uint32, ms since firmware boot
  imu_id: number;      // uint8: 0 or 1
  connected: number;   // uint8 MAV_BOOL: 0 or 1
  healthy: number;     // uint8 MAV_BOOL: 0 or 1
  whoami: number;      // uint8, show as hex (for example 0x47)
};
```

Known `whoami` values are maintained in `src/imu.rs`; `0x47` is the common
ICM-42688-P. A failed bus commonly reports `0x00` or `0xFF`.

## Platform-side generation

Copy `scky.xml` beside the MAVLink repository's `common.xml`, then generate the
TypeScript classes with the official generator:

```bash
mavgen.py \
  --lang=TypeScript \
  --wire-protocol=2.0 \
  --output=generated/mavlink \
  message_definitions/v1.0/scky.xml
```

Feed arbitrary serial chunks into a streaming MAVLink parser; one USB read is
not guaranteed to contain exactly one packet. Dispatch decoded messages by
message ID/name, and store IMU samples keyed by `id`.

For a Rust receiver, use a MAVLink 2 parser generated/extended from the same
dialect. If the selected Rust library cannot generate custom dialects, decode
the three standard messages with it and add message 42000 using the exact
8-byte little-endian payload above.
