//! ESC manager: configuration, per-motor test state, DShot command queue and
//! the master safety interlock that sits between commands and the [`crate::dshot`]
//! output.
//!
//! There is no closed-loop flight control yet, so the **master enable** switch is
//! the arm for persistent output. A standard MAVLink motor test may also
//! temporarily enable the output path so bench testing does not depend on the
//! custom ESC-config message landing first.
//!
//! Each of the four motors is independent: it carries its own throttle target,
//! its own watchdog deadline and its own queue of DShot special commands. The
//! host can therefore spin any subset of motors at once (four sliders in the
//! ground station) without the motors interfering — earlier this state was a
//! single shared "one motor at a time" slot, which made concurrently-driven
//! motors alternate and read as "linked".
//!
//! # AM32 / BLHeli_32 alignment
//!
//! The output path mirrors what those ESC firmwares actually require:
//!   * a *continuous* DShot stream — zero-throttle frames are emitted even while
//!     disarmed so the ESC stays armed-idle instead of re-detecting the signal;
//!   * **throttle is ramped, never stepped** (a propless motor desyncs on an
//!     instant 0 → N jump and the ESC's stall protection cuts it);
//!   * **special commands (direction / 3D / save / beacon) are only honoured at
//!     zero throttle and must carry the telemetry-request bit** — without that
//!     bit AM32/BLHeli ignore the command — and each is repeated several frames.

use crate::dshot::{make_frame, Protocol};
use crate::esc_telem::TelemFrame;

/// Number of motors / ESCs.
pub const N_MOTORS: usize = 4;

/// Latest decoded ESC telemetry, indexed by motor. Published by the telemetry
/// UART interrupt and read by the USB task for `SCKY_ESC_TELEM`.
#[derive(Clone, Copy)]
pub struct EscTelemetry {
    pub rpm: [i32; N_MOTORS],
    pub centivolt: [u16; N_MOTORS],
    pub centiamp: [u16; N_MOTORS],
    pub temp: [u8; N_MOTORS],
    pub err: [u8; N_MOTORS],
    /// Consumption (mAh) of the most recently reported ESC.
    pub mah: u16,
    /// Round-robin slot the next decoded record is attributed to (a single shared
    /// telemetry wire carries no motor id — see [`crate::esc_telem`]).
    rr: usize,
}

impl EscTelemetry {
    pub const fn new() -> Self {
        Self {
            rpm: [0; N_MOTORS],
            centivolt: [0; N_MOTORS],
            centiamp: [0; N_MOTORS],
            temp: [0; N_MOTORS],
            err: [0; N_MOTORS],
            mah: 0,
            rr: 0,
        }
    }

    /// Store a decoded telemetry record into the next motor slot.
    pub fn ingest(&mut self, f: TelemFrame, pole_count: u8) {
        let i = self.rr % N_MOTORS;
        self.rpm[i] = f.rpm(pole_count);
        self.centivolt[i] = f.centivolt;
        self.centiamp[i] = f.centiamp;
        self.temp[i] = f.temp_c;
        self.mah = f.mah;
        self.rr = (self.rr + 1) % N_MOTORS;
    }

    /// Sum of per-motor current in amps (aggregate pack current).
    pub fn total_current_a(&self) -> f32 {
        self.centiamp.iter().map(|&c| c as f32).sum::<f32>() / 100.0
    }
}

/// Lowest DShot throttle value (0..47 are reserved as special commands).
const DSHOT_MIN_THROTTLE: u16 = 48;
const DSHOT_MAX_THROTTLE: u16 = 2047;

/// How long (ms) a motor test runs if the host does not refresh it. Also the
/// watchdog horizon: if the host stops talking, motors stop within this window.
pub const DEFAULT_TEST_TIMEOUT_MS: u32 = 3000;

/// Times each queued DShot special command is repeated on the wire. AM32 /
/// BLHeli_32 require a command to be seen several frames in a row before acting.
const CMD_REPEATS: u8 = 10;

/// Per-motor special-command FIFO depth. The common case is a 2-command
/// sequence — e.g. spin-direction (20/21) immediately followed by save (12) —
/// which must transmit in order; a single slot would drop the first.
const CMD_DEPTH: usize = 6;

/// Max throttle increase per output tick (DShot units). The throttle target is
/// applied gradually, not as a step: a free-spinning (propless) motor desyncs on
/// an instantaneous 0 → N jump — the motor kicks, AM32/BLHeli_32 stall protection
/// trips and the motor stops ("starts then stops"). At the default 1 kHz refresh
/// this ramps 0 → full over ~1 s, 0 → a 30 % test over ~300 ms. Throttle *down*
/// (including stop) is applied instantly — reducing throttle never desyncs.
const RAMP_STEP_PER_TICK: u16 = 2;

/// Live, host-tunable ESC configuration. Mirrors `SCKY_ESC_CONFIG` /
/// `SCKY_ESC_SET` on the wire.
#[derive(Clone, Copy)]
pub struct EscConfig {
    /// Master output enable. **Defaults to `false`** — nothing spins until the
    /// ground station explicitly turns it on.
    pub master_enabled: bool,
    pub protocol: Protocol,
    /// Output refresh rate (Hz). Capped to the 1 kHz monotonic tick in `main`.
    pub refresh_hz: u16,
    /// Bidirectional-DShot request flag (telemetry is read from the UART here, so
    /// this is reflected to the GS but does not change the bit-bang output).
    pub bidir: bool,
    /// Bit per motor: 1 = last commanded spin direction was "reversed".
    /// Informational reflection of the last direction command (direction is
    /// stored on the ESC itself, not applied here).
    pub dir_mask: u8,
    /// Bit per motor: 1 = 3D mode last commanded on.
    pub mode3d_mask: u8,
    /// Motor magnetic pole count, for eRPM → RPM in [`crate::esc_telem`].
    pub pole_count: u8,
    /// Analog current-sense calibration (C pad): scale (A per volt-equivalent)
    /// and offset (mV). See [`crate::esc_telem::analog_current_a`].
    pub cur_scale: f32,
    pub cur_offset: f32,
}

impl Default for EscConfig {
    fn default() -> Self {
        Self {
            master_enabled: false,
            protocol: Protocol::Dshot150,
            refresh_hz: 1000,
            bidir: false,
            dir_mask: 0,
            mode3d_mask: 0,
            pole_count: 14,
            cur_scale: 490.0, // SpeedyBee BL32 50A default
            cur_offset: 0.0,
        }
    }
}

/// A small in-order FIFO of pending DShot special commands for one motor. Each
/// entry is `(dshot_code, repeats_left)`; the front entry is emitted every output
/// tick and popped once its repeats are exhausted.
#[derive(Clone, Copy)]
struct CmdFifo {
    items: [(u16, u8); CMD_DEPTH],
    len: u8,
}

impl CmdFifo {
    const fn new() -> Self {
        Self { items: [(0, 0); CMD_DEPTH], len: 0 }
    }

    /// Enqueue a command to be repeated [`CMD_REPEATS`] times. Dropped silently if
    /// the queue is full (only ever a runaway host would overflow `CMD_DEPTH`).
    fn push(&mut self, code: u16) {
        if (self.len as usize) < CMD_DEPTH {
            self.items[self.len as usize] = (code, CMD_REPEATS);
            self.len += 1;
        }
    }

    /// Emit the front command's code for this tick, counting one repeat and
    /// popping the entry when its repeats run out. `None` when the queue is empty.
    fn next_code(&mut self) -> Option<u16> {
        if self.len == 0 {
            return None;
        }
        let (code, reps) = self.items[0];
        if reps <= 1 {
            for i in 1..self.len as usize {
                self.items[i - 1] = self.items[i];
            }
            self.len -= 1;
        } else {
            self.items[0].1 = reps - 1;
        }
        Some(code)
    }

    fn clear(&mut self) {
        self.len = 0;
    }
}

/// Independent output state for one motor.
#[derive(Clone, Copy)]
struct MotorOut {
    /// Target throttle (`48..2047`) or `0` = stop. Set by a motor test.
    target: u16,
    /// Monotonic deadline (ms) after which a non-zero target expires to stop —
    /// the host must refresh the motor test to sustain a spin.
    until_ms: u32,
    /// Throttle actually on the wire, slewed toward `target` each tick.
    applied: u16,
    /// Pending special commands (direction / 3D / save / beacon).
    cmds: CmdFifo,
}

impl MotorOut {
    const fn new() -> Self {
        Self { target: 0, until_ms: 0, applied: 0, cmds: CmdFifo::new() }
    }

    fn reset(&mut self) {
        self.target = 0;
        self.until_ms = 0;
        self.applied = 0;
        self.cmds.clear();
    }
}

const INIT_MOTOR: MotorOut = MotorOut::new();

/// ESC controller state owned by the firmware and mutated by inbound commands.
pub struct Esc {
    pub config: EscConfig,
    motors: [MotorOut; N_MOTORS],
}

/// Slew the currently applied throttle toward `target`. Increases are capped at
/// [`RAMP_STEP_PER_TICK`] and start at the lowest real throttle so a ramping
/// throttle never emits a reserved 1..47 special command; decreases (incl. stop)
/// are instant — reducing throttle never desyncs.
fn slew(applied: u16, target: u16) -> u16 {
    if target <= applied {
        return target; // throttle down / stop: instant
    }
    let from = applied.max(DSHOT_MIN_THROTTLE);
    (from + RAMP_STEP_PER_TICK).min(target)
}

impl Esc {
    pub const fn new() -> Self {
        Self {
            // `EscConfig::default()` is not const; fill the fields explicitly.
            config: EscConfig {
                master_enabled: false,
                protocol: Protocol::Dshot150,
                refresh_hz: 1000,
                bidir: false,
                dir_mask: 0,
                mode3d_mask: 0,
                pole_count: 14,
                cur_scale: 490.0,
                cur_offset: 0.0,
            },
            motors: [INIT_MOTOR; N_MOTORS],
        }
    }

    /// Apply a `SCKY_ESC_SET` config write. Disabling the master immediately
    /// cancels all motor activity (the frame computed next tick is MOTOR_STOP).
    #[allow(clippy::too_many_arguments)]
    pub fn apply_set(
        &mut self,
        master_enabled: bool,
        protocol: u8,
        refresh_hz: u16,
        bidir: bool,
        dir_mask: u8,
        mode3d_mask: u8,
        pole_count: u8,
        cur_scale: f32,
        cur_offset: f32,
    ) {
        self.config.master_enabled = master_enabled;
        self.config.protocol = Protocol::from_u8(protocol);
        self.config.refresh_hz = refresh_hz.clamp(50, 1000);
        self.config.bidir = bidir;
        self.config.dir_mask = dir_mask;
        self.config.mode3d_mask = mode3d_mask;
        self.config.pole_count = pole_count.clamp(2, 64);
        self.config.cur_scale = cur_scale;
        self.config.cur_offset = cur_offset;
        if !master_enabled {
            for m in self.motors.iter_mut() {
                m.reset();
            }
        }
    }

    /// Start a timed motor test from a `MAV_CMD_DO_MOTOR_TEST`. `motor` is 1-based
    /// (1..4); `throttle_pct` is 0..100; `timeout_ms` 0 uses the default. A valid
    /// spin temporarily enables the master path so standard GCS motor-test tools
    /// work even if the custom `SCKY_ESC_SET` master toggle was not sent. A zero
    /// throttle is a per-motor stop and never arms the master.
    pub fn start_test(&mut self, motor: u8, throttle_pct: f32, timeout_ms: u32, now_ms: u32) -> bool {
        // Be tolerant of host conventions: some tools send motor 0 for the
        // first output even though MAV_CMD_DO_MOTOR_TEST is nominally 1-based.
        let motor = if motor == 0 { 1 } else { motor };
        if motor as usize > N_MOTORS {
            return false;
        }
        let idx = motor as usize - 1;
        // Some frontends encode 10% as 0.10 instead of 10.0. Accept both.
        let throttle_pct = if throttle_pct > 0.0 && throttle_pct <= 1.0 {
            throttle_pct * 100.0
        } else {
            throttle_pct
        };
        let pct = throttle_pct.clamp(0.0, 100.0) / 100.0;
        let span = (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) as f32;
        let value = if pct <= 0.0 {
            0
        } else {
            DSHOT_MIN_THROTTLE + (pct * span) as u16
        };
        // A zero or tiny timeout is nearly indistinguishable from "snaps back to
        // zero" in the UI, so give bench motor tests a useful minimum window.
        let timeout = if timeout_ms < 500 {
            DEFAULT_TEST_TIMEOUT_MS
        } else {
            timeout_ms
        };
        if value != 0 {
            self.config.master_enabled = true;
        }
        let m = &mut self.motors[idx];
        m.target = value;
        if value != 0 {
            m.until_ms = now_ms.wrapping_add(timeout);
        }
        true
    }

    /// Stop all motors immediately (clears every per-motor throttle target).
    pub fn stop_all(&mut self) {
        for m in self.motors.iter_mut() {
            m.target = 0;
        }
    }

    /// Snapshot of the first actively-spinning motor for telemetry diagnostics:
    /// `(motor_1_based, dshot_value, remaining_ms)`.
    pub fn active_test(&self, now_ms: u32) -> Option<(u8, u16, u32)> {
        for (i, m) in self.motors.iter().enumerate() {
            if m.target != 0 {
                return Some((i as u8 + 1, m.target, m.until_ms.wrapping_sub(now_ms)));
            }
        }
        None
    }

    /// Queue a DShot special command (`dshot_cmd`, e.g. 20/21 spin direction,
    /// 9/10 3D off/on, 12 save, 1..5 beacon). `target` 0 = all motors, 1..4 = a
    /// single motor. The motor's throttle is forced to zero first because
    /// AM32/BLHeli only honour these commands at rest, and the command is queued
    /// (not overwritten) so a direction-then-save sequence both transmit. Tracks
    /// direction/3D in the config so the GS reflects intent.
    pub fn queue_command(&mut self, target: u8, dshot_cmd: u16) {
        if target == 0 {
            self.config.master_enabled = true;
            for m in self.motors.iter_mut() {
                m.target = 0;
                m.cmds.push(dshot_cmd);
            }
            self.track_command(0xFF, dshot_cmd);
        } else if (target as usize) <= N_MOTORS {
            self.config.master_enabled = true;
            let idx = target as usize - 1;
            self.motors[idx].target = 0;
            self.motors[idx].cmds.push(dshot_cmd);
            self.track_command(idx as u8, dshot_cmd);
        }
    }

    /// Reflect direction/3D commands into the config masks (informational only).
    fn track_command(&mut self, motor_idx: u8, dshot_cmd: u16) {
        let set = |mask: &mut u8, on: bool| {
            if motor_idx == 0xFF {
                *mask = if on { 0x0F } else { 0 };
            } else {
                let bit = 1 << motor_idx;
                if on {
                    *mask |= bit;
                } else {
                    *mask &= !bit;
                }
            }
        };
        match dshot_cmd {
            20 => set(&mut self.config.dir_mask, false), // spin normal
            21 => set(&mut self.config.dir_mask, true),  // spin reversed
            9 => set(&mut self.config.mode3d_mask, false), // 3D off
            10 => set(&mut self.config.mode3d_mask, true), // 3D on
            _ => {}
        }
    }

    /// Compute the four DShot frames to transmit this tick.
    ///
    /// Safety interlock: with the master disabled, every motor gets MOTOR_STOP and
    /// all per-motor state is cleared. Otherwise, per motor: an expired test
    /// target reverts to stop; a queued special command is drained (one repeat per
    /// tick, **with the telemetry bit set**) but only while that motor is at rest;
    /// then the throttle is slewed toward its target.
    pub fn frames(&mut self, now_ms: u32) -> [u16; N_MOTORS] {
        if !self.config.master_enabled {
            for m in self.motors.iter_mut() {
                m.reset();
            }
            return [make_frame(0, false); N_MOTORS];
        }

        let mut out = [make_frame(0, false); N_MOTORS];
        for i in 0..N_MOTORS {
            let m = &mut self.motors[i];

            // Watchdog: a spin the host stopped refreshing expires to stop.
            if m.target != 0 && now_ms.wrapping_sub(m.until_ms) < u32::MAX / 2 {
                m.target = 0;
            }

            // Special command (direction/3D/save/beacon): only honoured by the ESC
            // at zero throttle, and only with the telemetry-request bit set.
            if m.target == 0 && m.applied == 0 {
                if let Some(code) = m.cmds.next_code() {
                    out[i] = make_frame(code, true);
                    continue;
                }
            }

            // Slew toward the target so the ESC never sees a throttle step.
            m.applied = slew(m.applied, m.target);
            out[i] = make_frame(m.applied, false);
        }
        out
    }
}
