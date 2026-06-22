# scky-fc — RTIC Flight-Controller Firmware (STM32H743 / DAKEFPVH743)

A from-scratch, [RTIC](https://rtic.rs)-based flight-controller firmware in Rust
for the **DAKEFPV H743** flight controller, intended to eventually replace
ArduPilot on this hardware. This first drop brings up the board, both IMUs, and
USB telemetry — the foundation everything else builds on.

> **Status: Milestone 1 + attitude estimation.** It probes both IMUs, samples
> and low-pass-filters them at 1 kHz, fuses them into a stable roll/pitch/yaw
> estimate (PX4-style complementary filter), and streams it over USB CDC. It is
> **not** yet a closed-loop flight controller (no control or motor output).
>
> The full fusion pipeline is documented in
> **[docs/sensor-fusion.md](docs/sensor-fusion.md)**.

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
| `imu1_task` | async software | **3** | 1 kHz | read SPI1 IMU, low-pass filter, publish `out1` |
| `imu2_task` | async software | **3** | 1 kHz | read SPI4 IMU, low-pass filter, publish `out2` |
| `estimator_task` | async software | **2** | 1 kHz | rotate + combine both IMUs, run attitude filter, publish `att` |
| `usb_task` | async software | 1 | 1 kHz poll | poll USB stack + stream attitude (~10 Hz) and heartbeat (1 Hz) |

- **Estimator sits between sampling and USB.** Sampling (prio 3) can preempt
  fusion (prio 2), and both preempt USB (prio 1) — so neither fusion nor logging
  can ever perturb the 1 kHz sampling. The fusion math is in
  [docs/sensor-fusion.md](docs/sensor-fusion.md).
- **One owner for USB.** `usb_task` exclusively owns the USB device + serial port
  and both *polls* the stack (every ~1 ms, keeping enumeration alive and flushing
  the IN endpoint) and *writes* telemetry. This removes all cross-task locking on
  USB and — crucially — guarantees the stack is polled even when no host-driven
  interrupt happens to fire, which is what makes data actually come out. Writes go
  through `pump_write`, which polls between 64-byte packets so full lines flush.
- **Monotonic:** Systick @ 1 kHz (`rtic-monotonics`), clocked from the 64 MHz core.
- **No dynamic allocation** anywhere in the control path; samples are passed
  through RTIC shared resources, log lines built in stack `heapless::String`s.
- **Dispatchers:** `LPTIM1`, `LPTIM2`, `LPTIM3` (unused peripherals borrowed for
  the three software-task priority levels).

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
│   ├── main.rs           # RTIC app: clocks, SPI, USB, tasks, wiring
│   ├── imu.rs            # InvenSense v3 SPI driver (probe/config/read)
│   ├── filters.rs        # PX4-style 2nd-order low-pass + notch filters
│   ├── ahrs.rs           # Mahony quaternion complementary attitude filter
│   └── estimator.rs      # mount rotation + dual-IMU combine + fusion driver
├── docs/
│   └── sensor-fusion.md  # the filtering + fusion math, and PX4 mapping
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
sudo apt install gcc-arm-none-eabi   # gives arm-none-eabi-objcopy (already present here)
sudo apt install dfu-util            # already present on this machine
# (cargo-binutils / `cargo objcopy` is OPTIONAL and NOT installed here — see §7)
```

---

## 7. Build & convert to a flashable binary

### Step 1 — build the ELF

```bash
cargo build --release
```

Produces the ELF at `target/thumbv7em-none-eabihf/release/scky-fc`
(~47 KB — trivially fits the 2 MB flash). The ELF is what `probe-rs`/SWD flashes
directly (Path A in §8), so if you flash over SWD you can stop here.

### Step 2 — convert ELF → raw `.bin` (only needed for USB DFU)

> **Use `arm-none-eabi-objcopy` — this is the method that works on this machine.**
> `cargo objcopy` fails here because `cargo-binutils` is **not installed**; don't
> use it unless you run `cargo install cargo-binutils` first.

```bash
# ✅ WORKS — uses the GNU ARM objcopy already in /usr/bin (sudo apt install
#    gcc-arm-none-eabi if you ever need it on another machine):
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/scky-fc \
    scky-fc.bin
```

Verify the result is a real Cortex-M image (sanity check):

```bash
file scky-fc.bin
# -> ARM Cortex-M firmware, initial SP at 0x20020000, reset at 0x08000298, ...
```

A correct `.bin` reports an initial stack pointer in RAM (`0x2002_0000`, top of
DTCM) and a reset vector in flash (`0x0800_0xxx`). If `file` says "data" or the
first bytes are all zero, the conversion targeted the wrong file.

**Alternatives (if you prefer not to use `arm-none-eabi-objcopy`):**

```bash
# (a) plain GNU objcopy (also already in /usr/bin):
objcopy -O binary target/thumbv7em-none-eabihf/release/scky-fc scky-fc.bin

# (b) LLVM objcopy shipped with the rustup `llvm-tools` component
#     (rustup component add llvm-tools), invoked by full path:
"$(find ~/.rustup/toolchains -name llvm-objcopy | head -1)" \
    -O binary target/thumbv7em-none-eabihf/release/scky-fc scky-fc.bin

# (c) the `cargo objcopy` convenience wrapper — ONLY after installing the tool:
cargo install cargo-binutils
cargo objcopy --release -- -O binary scky-fc.bin
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
picocom /dev/ttyACM0         # RECOMMENDED — quit with Ctrl-A then Ctrl-X
# or:  tio /dev/ttyACM0      # quit with Ctrl-T then Q
# or just dump it raw:       cat /dev/ttyACM0   (quit with Ctrl-C)
# or:  screen /dev/ttyACM0   # quit with Ctrl-A then K, then y
```

If you get a permission error, add yourself to `dialout`
(`sudo usermod -aG dialout $USER`, then re-login) or `sudo cat /dev/ttyACM0`.
If the node is `ttyACM1` instead of `ttyACM0`, just use that — `dmesg` tells you
which one.

**Closing `screen`:** `screen` does *not* quit with `Ctrl-C` (that goes to the
device) and there is no exit-on-flood — the keystrokes are: **`Ctrl-A`** then
**`K`** then **`y`** (kill window), or **`Ctrl-A`** then **`\`** then **`y`**
(quit). If a screen got "stuck" in the background, kill it from another terminal
with `screen -X -S $(screen -ls | grep -o '[0-9]*\.ttys*[^ ]*' | head -1) quit`
or simply `pkill screen`. The telemetry now refreshes at ~10 Hz (down from 20),
which also keeps the terminal responsive; `picocom`/`tio` are easier to exit and
recommended over `screen`.

### Expected output

The estimator runs at 1 kHz; the console prints the fused attitude at ~10 Hz plus
a 1 Hz heartbeat. **Tilt the board** and `roll`/`pitch` track gravity and hold
steady; **rotate it** and the `rate` values spike. That's your "is it reading
correctly?" check.

```
ATT roll=   +0.4 pitch=   -1.2 yaw=   +0.0 deg | rate r/p/y=   +0.3/   -0.1/   +0.0 dps | bias=+0.00/+0.00/+0.00
ATT roll=  +18.7 pitch=   -1.1 yaw=   +3.9 deg | rate r/p/y=  +42.5/   -0.4/   +1.1 dps | bias=-0.01/+0.00/+0.02
[HB up=3s] IMU1=ICM-42688-P WHO_AM_I=0x47(OK) IMU2=ICM-42688-P WHO_AM_I=0x47(OK)
```

- `roll`/`pitch` come from the **fused** gravity + gyro estimate (not raw accel),
  so they're smooth and drift-free. Flat board ⇒ ~0°/0°.
- `yaw` is now produced (gyro-integrated). With **no magnetometer** on this board
  it has no absolute reference and slowly drifts — expected, and explained in
  [docs/sensor-fusion.md §7](docs/sensor-fusion.md). Roll/pitch do **not** drift.
- `bias` is the estimator's live gyro-bias estimate (deg/s) — it should settle to
  small values within a few seconds of sitting still.
- A failed IMU shows in the heartbeat as `WHO_AM_I=0x00(FAIL)` (MISO stuck low) or
  `0xFF` — see troubleshooting.

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
- **No USB serial device appears at all.** USB runs off HSI48 (enabled
  automatically by the HAL's `freeze()`); the core runs on the internal 64 MHz
  HSI (the tested board hangs on the 480 MHz HSE/PLL startup path — see §2). If
  enumeration still fails, some boards gate USB on the internal 3.3 V regulator —
  uncomment the `usbregen` block referenced in the HAL's `usb_rtic` example and
  re-test.
- **One IMU OK, the other FAIL.** Each IMU is on a *separate* bus (SPI1 vs SPI4),
  so a single failure points at that bus/CS specifically — check the CS pin
  (PA4 vs PB1) and the SPIx pin alternate-function wiring in `init`.
- **Both `0x00`.** Suspect power/reset to the IMUs or a swapped MOSI/MISO. The SPI
  clock is already a conservative `1.MHz()` in `init`; raise it only after
  WHO_AM_I is solid.
- **`roll`/`pitch` jump or settle slowly.** Tune the filter — see
  [docs/sensor-fusion.md §9](docs/sensor-fusion.md). Lower `AHRS_KP` for less
  noise, raise it for faster response.
- **Won't build / wrong chip on flash.** `probe-rs chip list | grep H743` and fix
  `--chip` in `.cargo/config.toml` (e.g. `STM32H743VITx` vs `STM32H743ZITx`).

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
5. **Core clock = internal HSI (64 MHz); USB clock = HSI48.** The tested board
   hangs on the 480 MHz HSE/PLL path (§2), so the firmware boots on HSI for
   reliability. Both are independent of the external crystal.
6. **Yaw is gyro-integrated and drifts.** No magnetometer is fused (the board has
   no onboard compass), so heading has no absolute reference. Roll/pitch are
   gravity-referenced and drift-free. See
   [docs/sensor-fusion.md §7](docs/sensor-fusion.md).
7. **Mount rotations ARE applied** in the estimator (`ROLL_180` for IMU1,
   `PITCH_180` for IMU2) so both sensors share the body frame before fusion.

---

## 13. Roadmap

- [x] Per-IMU low-pass filtering + dual-IMU combine + Mahony attitude estimator.
- [ ] Magnetometer fusion for absolute yaw; gyro/accel calibration & temp-comp.
- [ ] FIFO burst reads + innovation-weighted IMU voting.
- [ ] SPL06 barometer (I2C2) and the OSD/dataflash on SPI2/SPI3.
- [ ] Full EKF2-style nav filter once baro/mag/GPS exist (see docs §8).
- [ ] Rate controller → motor mixer → DShot/PWM output.
- [ ] Swap busy-wait CDC logging for a defmt-over-RTT or framed binary link.
