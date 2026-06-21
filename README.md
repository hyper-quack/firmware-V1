# scky-fc — RTIC Flight-Controller Firmware (STM32H743 / DAKEFPVH743)

A from-scratch, [RTIC](https://rtic.rs)-based flight-controller firmware in Rust
for the **DAKEFPV H743** flight controller, intended to eventually replace
ArduPilot on this hardware. This first drop brings up the board, both IMUs, and
USB telemetry — the foundation everything else builds on.

> **Status: Milestone 1 + RTIC scaffold.** It probes both IMUs, samples them at
> 1 kHz in real-time tasks, and streams data over USB CDC. It is **not** yet a
> closed-loop flight controller (no estimation, mixing, or motor output).

---

## 1. How the hardware was identified

No `hwdef.dat` was provided — only the compiled ArduPilot artifacts
(`arducopter.apj`, `arducopter.elf`, `features.txt`). The hardware definition was
recovered from those and then cross-checked against the authoritative ArduPilot
source:

| Source | What it gave us |
|---|---|
| `arducopter.apj` (JSON header) | `board_id 1193`, `summary DAKEFPVH743`, `STM32H743xx`, USB VID/PID `0x1209/0x5741` |
| `arducopter.elf` (symbols/strings) | IMU backend = `AP_InertialSensor_Invensensev3`; baro = `AP_Baro_SPL06` |
| ArduPilot `DAKEFPVH743Pro/hwdef.dat` | exact SPI buses, CS pins, SPI mode/speed, IMU rotations, I2C/baro |

The board's own `hwdef.dat` simply `include`s `../DAKEFPVH743Pro/hwdef.dat`, which
is where the sensor wiring lives.

---

## 2. Hardware map (ArduPilot hwdef → this firmware)

**MCU:** STM32H743xx, Cortex-M7F currently running from the internal 64 MHz HSI.
The board definition specifies an 8 MHz HSE crystal, but the tested non-Pro
DAKEFPVH743 board did not complete the 480 MHz HSE/PLL startup path. The internal
clock is used until that board-specific clock issue is characterized.

### IMUs — dual InvenSense v3, each on its own SPI bus

ArduPilot declares both IMUs as `IMU Invensensev3` and auto-detects the exact part
via `WHO_AM_I`. On this board they are almost certainly **ICM-42688-P**
(`WHO_AM_I = 0x47`); the firmware accepts the whole v3 family and prints whatever
ID it actually reads so you can confirm.

| ArduPilot line | Bus | SCK | MISO | MOSI | CS | Mode | Speed | Rotation |
|---|---|---|---|---|---|---|---|---|
| `SPIDEV imu1 SPI1 … GYRO1_CS` | SPI1 | PA5 | PA6 | PA7 | **PA4** | MODE3 | 1–16 MHz | `ROLL_180` |
| `SPIDEV imu2 SPI4 … GYRO2_CS` | SPI4 | PE12 | PE13 | PE14 | **PB1** | MODE3 | 1–16 MHz | `PITCH_180` |

> **Interrupts:** the hwdef defines **no DRDY/EXTI pins** for the IMUs. ArduPilot
> polls the sensor FIFO on a timer; this firmware does the same with a 1 kHz
> Systick-driven RTIC task (see §4). The mounting rotations above are documented
> for when the attitude estimator is added — raw output is currently unrotated.

### Other devices on the board (present, not yet driven)

| Function | ArduPilot line | Bus / pins |
|---|---|---|
| OSD | `SPIDEV osd SPI2 … OSD1_CS MODE0` | SPI2 (PB13/14/15), CS PB12 |
| Dataflash | `SPIDEV dataflash SPI3 … FLASH1_CS MODE3` | SPI3 (PC10/11/12), CS PA15 |
| Barometer | `BARO SPL06 I2C:0:0x76` | I2C2 (PB10 SCL / PB11 SDA), addr 0x76 |

### USB

USB-C is on **PA11/PA12 = OTG2_FS** (the HAL's `USB2` peripheral), used in
full-speed device mode for the CDC-ACM debug console. VID/PID reused from
ArduPilot: `0x1209/0x5741`.

---

## 3. SPI configuration translation, in detail

`MODE3` in the hwdef means SPI clock polarity high + phase on second edge
(CPOL=1, CPHA=1) → `embedded_hal::spi::MODE_3`. ArduPilot uses a 1 MHz "low" probe
speed and 16 MHz "high" data speed; this firmware runs a single conservative
1 MHz clock for both probe and data. This matches ArduPilot's safe probe speed
and is sufficient for the current 1 kHz register sampling. Chip-select is
**software-driven GPIO** (active-low), exactly as
ArduPilot does it — the CS pins are ordinary outputs, not the SPI peripheral's
hardware NSS.

Register access follows the InvenSense convention: bit 7 of the address byte set
= read, cleared = write; the contiguous block `TEMP→ACCEL→GYRO` (regs `0x1D…0x2A`)
is burst-read big-endian in one transaction.

---

## 4. RTIC architecture

Higher number = higher priority. The key invariant from the brief — *USB must
never delay IMU sampling* — is enforced by priority: the 1 kHz sampling tasks
outrank the USB interrupt, and telemetry writes are non-blocking (dropped if the
host isn't draining).

| Task | Kind | Prio | Rate | Job |
|---|---|---|---|---|
| `imu1_task` | async software | **3** | 1 kHz | read SPI1 IMU, publish sample |
| `imu2_task` | async software | **3** | 1 kHz | read SPI4 IMU, publish sample |
| `usb_task` | async software | 1 | 1 kHz poll | poll USB stack + stream telemetry (~20 Hz) and heartbeat (1 Hz) |

- **One owner for USB.** `usb_task` exclusively owns the USB device + serial port
  and both *polls* the stack (every ~1 ms, keeping enumeration alive and flushing
  the IN endpoint) and *writes* telemetry. This removes all cross-task locking on
  USB and — crucially — guarantees the stack is polled even when no host-driven
  interrupt happens to fire, which is what makes data actually come out. Writes go
  through `pump_write`, which polls between 64-byte packets so full lines flush.
- **Monotonic:** Systick @ 1 kHz (`rtic-monotonics`), clocked from the 64 MHz core.
- **No dynamic allocation** anywhere in the control path; samples are passed
  through RTIC shared resources, log lines built in stack `heapless::String`s.
- **Dispatchers:** `LPTIM1`, `LPTIM2` (unused peripherals borrowed for the two
  software-task priority levels).

---

## 5. Project layout

```
scky_firmware/
├── Cargo.toml            # deps + release profile (LTO, opt-level=s)
├── rust-toolchain.toml   # stable + thumbv7em target + llvm-tools
├── memory.x              # H743 flash/RAM linker layout (flash @ 0x08000000)
├── build.rs              # feeds memory.x to the linker
├── .cargo/config.toml    # target triple + probe-rs runner
├── src/
│   ├── main.rs           # RTIC app: clocks, SPI, USB, tasks
│   └── imu.rs            # InvenSense v3 SPI driver (probe/config/read)
└── README.md
```

---

## 6. Toolchain prerequisites

```bash
# Rust target + helpers (rust-toolchain.toml pins these, rustup will auto-install)
rustup target add thumbv7em-none-eabihf
rustup component add llvm-tools

# To flash over SWD (recommended):
cargo install probe-rs-tools

# To produce a raw .bin and/or flash over USB DFU:
cargo install cargo-binutils      # gives `cargo objcopy`
sudo apt install dfu-util         # already present on this machine
```

---

## 7. Build

```bash
cargo build --release
```

Produces the ELF at `target/thumbv7em-none-eabihf/release/scky-fc`
(~30 KB text — trivially fits the 2 MB flash).

Create a raw binary for DFU:

```bash
cargo objcopy --release -- -O binary scky-fc.bin
# (equivalent without cargo-binutils:)
# llvm-objcopy -O binary target/thumbv7em-none-eabihf/release/scky-fc scky-fc.bin
```

---

## 8. Flashing

This firmware is laid out to **own the whole chip** (vectors at `0x0800_0000`).
That means it replaces ArduPilot, including ArduPilot's own bootloader. Pick one
of the two paths below.

> ⚠️ **Back up first.** If you ever want ArduPilot back, dump the existing flash
> before overwriting:
> `probe-rs read b32 0x08000000 0x200000 --chip STM32H743VITx > ardu_backup.bin`

### Path A — SWD debug probe (recommended)

Wire an ST-Link / J-Link / CMSIS-DAP probe to the board's **SWDIO / SWCLK / GND
(+ 3V3)** pads. Then a single command builds, flashes and opens an RTT/log view:

```bash
cargo run --release
```

(`.cargo/config.toml` sets the runner to `probe-rs run --chip STM32H743VITx`.)
Flash-only, without running:

```bash
probe-rs download --chip STM32H743VITx \
    target/thumbv7em-none-eabihf/release/scky-fc
```

If `probe-rs` reports the wrong part, list candidates with
`probe-rs chip list | grep H743` and adjust the `--chip` value.

### Path B — USB DFU (no probe needed)

Uses the **STM32 system (ROM) bootloader**, not the ArduPilot one.

1. Enter system DFU: hold **BOOT0 high (3V3)** and tap reset (or use the board's
   BOOT/DFU button/pad if present), then plug in USB-C.
2. Confirm the device shows up (`0483:df11`):
   ```bash
   dfu-util -l
   ```
3. Flash and start:
   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D scky-fc.bin
   ```

> The bare `dfu-util -a 0 -D scky-fc.bin` form only works if your `dfu-util`
> already knows the start address from the DfuSe descriptor; the explicit
> `-s 0x08000000:leave` is the reliable form for the STM32 ROM bootloader.
>
> **Note:** the *ArduPilot* bootloader that ships on the board speaks ArduPilot's
> own upload protocol (via `Tools/scripts/uploader.py` / Mission Planner), **not**
> `dfu-util`. It cannot load this raw image. Use Path A or the system DFU above.

---

## 9. Viewing the USB CDC console

After flashing, the board enumerates as a USB CDC serial port. The firmware
**streams continuously on its own** — you do not have to type anything.

```bash
# Confirm which node is ours right after plugging in:
dmesg | tail -n 5            # look for "cdc_acm ... ttyACMx: USB ACM device"
ls /dev/ttyACM*

# Open it (CDC ignores the baud rate, any value works):
screen /dev/ttyACM0          # quit with Ctrl-A then K
# or:  picocom /dev/ttyACM0
# or just dump it raw:  cat /dev/ttyACM0
```

If you get a permission error, add yourself to `dialout`
(`sudo usermod -aG dialout $USER`, then re-login) or `sudo cat /dev/ttyACM0`.
If the node is `ttyACM1` instead of `ttyACM0`, just use that — `dmesg` tells you
which one.

### Expected output (Milestone 1)

It prints live data ~20× per second plus a 1 Hz heartbeat. Tilt the board and
`roll`/`pitch` should track gravity; rotate it and the gyro rates spike — that's
your "is it reading correctly?" check.

```
IMU1 OK WHO_AM_I=0x47 | roll=  +0.4 pitch=  -1.2 deg | gyro r/p/y=   +0.3/   -0.1/   +0.0 dps | acc=+0.01/-0.02/+0.99 g
IMU2 OK WHO_AM_I=0x47 | roll=  -0.2 pitch=  +0.5 deg | gyro r/p/y=   -0.2/   +0.2/   -0.1 dps | acc=-0.00/+0.01/+1.00 g
[HB up=3s] IMU1=ICM-42688-P(OK) IMU2=ICM-42688-P(OK)
```

- `roll`/`pitch` are derived from the gravity vector (accel). With the board
  flat both read ~0° and `acc z` reads ~+1.00 g. Yaw is intentionally omitted —
  it can't be recovered from accel alone (needs the magnetometer or integration),
  and is left for the estimator.
- A failed IMU prints `FAIL WHO_AM_I=0x00` (MISO stuck low) or `0xFF` (stuck
  high) — see troubleshooting.

> **Note on the raw sensor frame:** roll/pitch and the gyro axes are *unrotated*
> — the board's `ROLL_180` / `PITCH_180` mount rotations aren't applied yet, so
> signs/axes may not match the airframe. That's fine for a read-back sanity check.

---

## 10. Milestone 1 acceptance — the bring-up gate

The brief's critical first milestone is: *init SPI, toggle CS, read WHO_AM_I from
both IMUs, output over USB; if it fails, stop and debug hardware.* That is exactly
what the `OK / FAIL` + `WHO_AM_I=0x..` lines above tell you. Until **both** report
`OK`, do not move on to estimation/control.

| Reading | Meaning |
|---|---|
| `WHO_AM_I=0x47` (or other known v3 id) | bus + sensor good ✅ |
| `WHO_AM_I=0x00` | MISO stuck low — no power, wrong MISO pin, or CS never asserted |
| `WHO_AM_I=0xFF` | MISO stuck high — MISO floating / not connected |
| known id but `FAIL` | unexpected part — extend `KNOWN_WHOAMI` in `src/imu.rs` |

---

## 11. Troubleshooting

- **Port enumerates (`/dev/ttyACMx` exists) but nothing prints.** This is what
  the current design fixes: `usb_task` polls the USB stack every ~1 ms and
  `pump_write` polls between packets, so multi-packet lines actually flush even
  with no host-driven interrupt. If you still see silence, confirm you opened the
  right node (`dmesg | tail`), try `cat /dev/ttyACMx` directly, and check it isn't
  a permissions issue (`dialout`).
- **No USB serial device appears at all.** The 480 MHz tree needs core voltage
  VOS0 (`pwr.vos0()` — already set). USB runs off HSI48 (enabled automatically by
  the HAL's `freeze()`). If enumeration still fails, some boards gate USB on the
  internal 3.3 V regulator — uncomment the `usbregen` block referenced in the
  HAL's `usb_rtic` example and re-test.
- **One IMU OK, the other FAIL.** Each IMU is on a *separate* bus (SPI1 vs SPI4),
  so a single failure points at that bus/CS specifically — check the CS pin
  (PA4 vs PB1) and the SPIx pin alternate-function wiring in `init`.
- **Both `0x00`.** Suspect power/reset to the IMUs or a swapped MOSI/MISO. Drop
  the SPI clock (change `8.MHz()` in `main.rs`) to rule out signal integrity.
- **Won't build / wrong chip on flash.** `probe-rs chip list | grep H743` and fix
  `--chip` in `.cargo/config.toml` (e.g. `STM32H743VITx` vs `STM32H743ZITx`).
- **`static_mut_refs` warning.** Expected; it mirrors the HAL's own USB example
  and is sound here (the EP buffer is touched once, in `init`).

---

## 12. Assumptions & caveats (stated, not guessed)

1. **Exact IMU P/N** is auto-detected. ArduPilot only commits to the *family*
   (`Invensensev3`); this firmware reports the real `WHO_AM_I`. If yours isn't in
   `KNOWN_WHOAMI`, add it.
2. **Polled, not interrupt-driven, sampling** — the hwdef defines no IMU DRDY
   pins, so a 1 kHz Systick task is used (matching ArduPilot's FIFO polling).
3. **No status-LED GPIO** is defined in the hwdef for a simple LED (only a
   TIM1 LED-strip output on PE9), so the heartbeat is emitted over USB only.
4. **Flash base = `0x0800_0000`** (full-chip replacement). Coexisting with the
   ArduPilot bootloader at an offset is intentionally not attempted.
5. **USB clock = HSI48.** Robust and independent of the PLL; if you re-tune the
   clock tree, leave HSI48 enabled or re-point `kernel_usb_clk_mux`.
6. **Raw sensor frames are unrotated.** The `ROLL_180` / `PITCH_180` mounting
   rotations are documented for the future estimator, not yet applied.

---

## 13. Roadmap

- [ ] Apply mount rotations; add gyro/accel calibration & temperature comp.
- [ ] FIFO burst reads + consistency/health voting across the two IMUs.
- [ ] SPL06 barometer (I2C2) and the OSD/dataflash on SPI2/SPI3.
- [ ] Attitude estimator → rate controller → motor mixer → DShot/PWM output.
- [ ] Swap busy-wait CDC logging for a defmt-over-RTT or framed binary link.
```
# firmware-V1
# firmware-V1
# firmware-V1
