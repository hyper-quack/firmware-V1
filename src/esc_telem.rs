//! BLHeli32 / KISS ESC telemetry decoder for the ESC↔FC link's **T pad**.
//!
//! Each telemetry record is 10 bytes, big-endian, with a trailing CRC8
//! (polynomial 0x07, MSB-first, init 0 — the BLHeli/KISS variant used by
//! Betaflight's `esc_sensor`):
//!
//! | bytes | field          | units            |
//! |-------|----------------|------------------|
//! | 0     | temperature    | °C               |
//! | 1..2  | voltage        | centivolts (0.01 V) |
//! | 3..4  | current        | centiamps  (0.01 A) |
//! | 5..6  | consumption    | mAh              |
//! | 7..8  | eRPM / 100     | (×100 = eRPM)    |
//! | 9     | CRC8           |                  |
//!
//! The stream carries no frame-sync byte, so the parser keeps a rolling 10-byte
//! window and accepts it whenever the CRC checks out, sliding by one byte on a
//! mismatch to resynchronise.
//!
//! Per-motor attribution on a single shared telemetry wire requires per-motor
//! DShot telemetry requests (a bidirectional-DShot feature not implemented here);
//! `main` distributes decoded records round-robin across the four motor slots and
//! keeps the latest for the aggregate power panel.

/// One decoded ESC telemetry record.
#[derive(Clone, Copy, Default)]
pub struct TelemFrame {
    pub temp_c: u8,
    pub centivolt: u16,
    pub centiamp: u16,
    pub mah: u16,
    pub erpm: u32,
}

impl TelemFrame {
    /// Mechanical RPM from electrical RPM and the motor pole count.
    pub fn rpm(&self, pole_count: u8) -> i32 {
        let poles = pole_count.max(2) as u32;
        (self.erpm * 2 / poles) as i32
    }
}

fn update_crc8(crc: u8, byte: u8) -> u8 {
    let mut c = crc ^ byte;
    for _ in 0..8 {
        c = if c & 0x80 != 0 { (c << 1) ^ 0x07 } else { c << 1 };
    }
    c
}

fn crc8(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |c, &b| update_crc8(c, b))
}

const FRAME_LEN: usize = 10;

/// Rolling-window decoder. `push` one received byte at a time; returns
/// `Some(frame)` when a CRC-valid 10-byte record completes.
pub struct EscTelemParser {
    buf: [u8; FRAME_LEN],
    len: usize,
    pub rx_bytes: u32,
    pub frames: u32,
    pub crc_errors: u32,
}

impl EscTelemParser {
    pub const fn new() -> Self {
        Self { buf: [0; FRAME_LEN], len: 0, rx_bytes: 0, frames: 0, crc_errors: 0 }
    }

    pub fn push(&mut self, byte: u8) -> Option<TelemFrame> {
        self.rx_bytes = self.rx_bytes.wrapping_add(1);
        if self.len < FRAME_LEN {
            self.buf[self.len] = byte;
            self.len += 1;
        } else {
            // Slide window left by one and append.
            self.buf.rotate_left(1);
            self.buf[FRAME_LEN - 1] = byte;
        }

        if self.len < FRAME_LEN {
            return None;
        }

        if crc8(&self.buf[..FRAME_LEN - 1]) == self.buf[FRAME_LEN - 1] {
            self.frames = self.frames.wrapping_add(1);
            self.len = 0; // consume the accepted frame; don't reuse its bytes
            Some(TelemFrame {
                temp_c: self.buf[0],
                centivolt: u16::from_be_bytes([self.buf[1], self.buf[2]]),
                centiamp: u16::from_be_bytes([self.buf[3], self.buf[4]]),
                mah: u16::from_be_bytes([self.buf[5], self.buf[6]]),
                erpm: u16::from_be_bytes([self.buf[7], self.buf[8]]) as u32 * 100,
            })
        } else {
            // Will slide on the next byte. Count a mismatch only when the window
            // was full (avoids over-counting during initial fill/resync).
            self.crc_errors = self.crc_errors.wrapping_add(1);
            None
        }
    }
}

/// Convert a raw analog current-sense ADC reading (millivolts at the FC pin) into
/// amperes using the host-set scale/offset calibration. Matches the common
/// `A = (mV - offset) * scale / 1000` convention (SpeedyBee BL32 50A: scale≈490,
/// offset 0).
///
/// Reserved for the C-pad ADC path (not yet wired — see `main`/README); kept so
/// the `cur_scale`/`cur_offset` calibration has a defined meaning end-to-end.
#[allow(dead_code)]
pub fn analog_current_a(adc_mv: f32, scale: f32, offset: f32) -> f32 {
    ((adc_mv - offset) * scale / 1000.0).max(0.0)
}
