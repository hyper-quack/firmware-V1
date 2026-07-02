# ESC control, telemetry & configuration

This firmware drives the four motor ESCs with **bit-banged DShot**, reads
**BLHeli32 / KISS telemetry** over a UART, and exposes motor-test + configuration
to the ground station over the USB MAVLink link. There is no closed-loop flight
control yet, so the **master enable switch is the arm**: nothing spins until the
ground station turns it on, and a motor test always auto-stops at its timeout.

Pin assignments below are taken from the **DAKEFPVH743 ArduPilot hwdef** (the
authoritative source for this board).

## Wiring

| Function | Pin(s) | Notes |
|----------|--------|-------|
| Motor M1..M4 (DShot) | **PA0, PA1, PA2, PA3** | hwdef `M1..M4 = PA0..PA3` (TIM2). All on GPIOA → one atomic `BSRR` write drives them together. Driven as plain GPIO (bit-bang), push-pull, idle low. |
| ESC telemetry **T pad** | **USART3 RX = PD9** (TX PD8) | hwdef `DEFAULT_SERIAL3_PROTOCOL = ESCTelemetry`, SERIAL3 = USART3. 115200 8N1, one-way. BLHeli32/KISS 10-byte frames. |
| ESC current **C pad** | (ADC, not yet wired) | `cur_scale`/`cur_offset` calibration fields exist; analog read is a follow-up. Current is read from the T-wire telemetry for now. |
| **G / V** | GND / battery sense | Common ground + pack voltage (voltage also arrives in the T-wire telemetry). |

See [`src/dshot.rs`](../src/dshot.rs) `MOTOR_BITS` / `port()` and the USART3 block in
[`src/main.rs`](../src/main.rs) `init`.

When a command arrives over USB the FC emits a `STATUSTEXT` ack (e.g.
`ESC: set master=1 proto=0 hz=1000`), visible in the ground-station telemetry
feed — use it to confirm the uplink is landing.

## Why bit-banged DShot

`stm32h7xx-hal` 0.16 implements DMA `TargetAddress` for SPI/UART/ADC/DAC/SAI but
**not for timers**, so the usual timer-CCR-over-DMA technique is unavailable
through the HAL. Bit-banging on plain GPIO keeps the output pin-flexible and is
appropriate for bench motor testing. DShot150 is the default for timing margin at
the board's 64 MHz HSI core clock; frames are sent with interrupts enabled, and
DShot's 4-bit CRC lets the ESC silently drop any frame a higher-priority ISR
corrupted.

## MAVLink contract

| Message | ID | crc_extra | Dir | Purpose |
|---------|----|-----------|-----|---------|
| `COMMAND_LONG` + `MAV_CMD_DO_MOTOR_TEST` | 76 / cmd 209 | 152 | GS→FC | Timed single-motor spin |
| `SCKY_ESC_TELEM` | 42010 | 91 | FC→GS | rpm/voltage/current/temp/errors ×4 + mAh + current |
| `SCKY_ESC_CONFIG` | 42011 | 55 | FC→GS | Echo of live config |
| `SCKY_ESC_SET` | 42012 | 8 | GS→FC | Write config |
| `SCKY_ESC_CMD` | 42013 | 106 | GS→FC | DShot special command (beacon, direction, 3D, save) |

Definitions live in [`message_definitions/scky.xml`](../message_definitions/scky.xml).

## Configurable (DShot command + FC side)

- DShot protocol (150/300/600) and output refresh rate.
- Bidirectional-DShot flag (reflected to the GS; eRPM decode is a follow-up — telemetry comes from the T wire).
- Motor spin direction (DShot 20/21 + save 12), 3D mode (9/10), beacon (1..5).
- Motor pole count (for eRPM→RPM).
- Analog current-sense scale/offset (C-pad calibration).

**Out of scope:** PWM frequency, motor timing, demag, startup/ramp power — these
live in the ESC EEPROM and require BLHeli32 4-way passthrough (not implemented).

## Per-motor output model

Each of the four motors is **independent**: it has its own throttle target, its
own motor-test watchdog, and its own queue of DShot special commands. The ground
station can spin any subset of motors at once. (Earlier the FC held a single
"one motor at a time" test slot; when the GS re-sent several non-zero sliders the
slot was overwritten back and forth, so concurrently-driven motors alternated on
the wire and read as *linked / jittering / accumulating*. That is fixed.)

> **Per-motor RPM is approximate.** The shared telemetry T-wire carries no motor
> id, so decoded records are attributed round-robin (see `EscTelemetry::ingest`).
> Treat the per-motor RPM bars as aggregate health, not a true per-motor reading.

## How special commands are sent (AM32 / BLHeli_32)

Direction (20/21), 3D (9/10), save (12) and beacon (1..5) are not throttle, so
they are emitted only when these ESC-firmware preconditions are met:

1. **At zero throttle.** The FC forces the target motor to stop, then emits the
   command once it has spun down — AM32/BLHeli ignore config commands while the
   motor is driven.
2. **With the telemetry-request bit set.** Without it the ESC silently drops the
   command. (This bit-banged path reads telemetry from the T-wire UART, so the
   bit is *only* set for these special-command frames, never for throttle.)
3. **Repeated** several frames in a row, and **queued in order** — e.g. a spin
   direction change is `21` (or `20`) immediately followed by `12` (save); both
   are held in a small per-motor FIFO so the first is not overwritten by the
   second. Reversing a motor in the GS sends exactly this pair.

## Safety model

1. `master_enabled` defaults to **false**; `EscConfig::frames` returns
   `MOTOR_STOP` for every motor while it is off and clears all per-motor state.
2. A motor test carries a timeout (default 3 s); the GS must re-issue
   `DO_MOTOR_TEST` to sustain a spin.
3. **Propellers removed** for bench tests. A propless motor has almost no inertia
   and AM32/BLHeli stall protection will cut it well below full throttle (often
   ~30–50 %); it then needs throttle back to zero to re-arm. That ceiling is the
   ESC protecting the motor, not a throttle-scaling bug — throttle maps linearly
   `0..100 % → DShot 48..2047`. With props loaded the usable range is much higher.
