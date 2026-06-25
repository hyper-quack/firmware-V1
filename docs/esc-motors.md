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

## Safety model

1. `master_enabled` defaults to **false**; `EscConfig::frames` returns
   `MOTOR_STOP` for every motor while it is off and cancels any running test.
2. A motor test carries a timeout (default 3 s); the GS must re-issue
   `DO_MOTOR_TEST` to sustain a spin.
3. Special commands (direction/3D/save) are ignored while a test is running.
4. First bench tests: **propellers removed.**
