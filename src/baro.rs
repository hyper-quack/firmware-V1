//! Goertek SPL06-001 barometric pressure sensor driver (I2C).
//!
//! Wired to **I2C2** at address **0x76** (per the ArduPilot hwdef
//! `BARO SPL06 I2C:0:0x76`), sharing the bus with the compass. Like
//! [`crate::compass`], the bus is passed in per call rather than owned, so one
//! task drives both sensors.
//!
//! The SPL06 reports raw pressure/temperature that must be linearised with nine
//! per-chip calibration coefficients read from the device, then turned into an
//! altitude with the international barometric formula. We also capture a
//! ground-reference pressure on the first valid sample so the UI can show a
//! relative (launch-referenced) altitude that starts at zero.

use embedded_hal::blocking::i2c::{Write, WriteRead};

const ADDR: u8 = 0x76;

const REG_PSR: u8 = 0x00; // pressure [B2,B1,B0] then temperature [B2,B1,B0] —
                          // one 6-byte burst from here covers both (0x00..0x05).
const REG_PRS_CFG: u8 = 0x06;
const REG_TMP_CFG: u8 = 0x07;
const REG_MEAS_CFG: u8 = 0x08;
const REG_CFG: u8 = 0x09;
const REG_RESET: u8 = 0x0C;
const REG_ID: u8 = 0x0D; // product/revision id, 0x10
const REG_COEF: u8 = 0x10; // 18 calibration bytes

const PRODUCT_ID: u8 = 0x10;
// 8 samples/s, ×8 oversampling for both pressure and temperature. ≤8× needs no
// result-shift in CFG. Scale factor kP/kT for ×8 oversampling = 7864320.
const PRS_CFG: u8 = 0x33;
const TMP_CFG: u8 = 0xB3; // bit7=1: use the external (MEMS) sensor the coeffs match
const MEAS_CTRL_CONT: u8 = 0x07; // continuous pressure + temperature
const K_OVERSAMPLE_8X: f32 = 7_864_320.0;

const SEA_LEVEL_PA: f32 = 101_325.0;

/// Latest barometer reading.
#[derive(Clone, Copy, Default)]
pub struct BaroData {
    pub pressure_pa: f32,
    pub temperature_c: f32,
    /// Absolute pressure altitude (ISA), metres.
    pub altitude_m: f32,
    /// Altitude relative to the ground reference captured at startup, metres.
    pub rel_altitude_m: f32,
    pub healthy: bool,
}

/// SPL06 state: calibration coefficients + ground reference. Bus passed per call.
pub struct Baro {
    present: bool,
    // Calibration coefficients (already sign-extended).
    c0: f32,
    c1: f32,
    c00: f32,
    c10: f32,
    c01: f32,
    c11: f32,
    c20: f32,
    c21: f32,
    c30: f32,
    ground_pa: f32,
    ground_set: bool,
}

impl Baro {
    pub const fn new() -> Self {
        Self {
            present: false,
            c0: 0.0,
            c1: 0.0,
            c00: 0.0,
            c10: 0.0,
            c01: 0.0,
            c11: 0.0,
            c20: 0.0,
            c21: 0.0,
            c30: 0.0,
            ground_pa: 0.0,
            ground_set: false,
        }
    }

    pub fn present(&self) -> bool {
        self.present
    }

    /// Probe, reset, load calibration, and start continuous conversion.
    /// `delay_us` blocks (init only) while the device boots and coefficients
    /// become ready. Returns whether an SPL06 was found.
    pub fn init<I2C, E>(&mut self, i2c: &mut I2C, delay_us: &dyn Fn(u32)) -> bool
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        let mut id = [0u8; 1];
        if i2c.write_read(ADDR, &[REG_ID], &mut id).is_err() || id[0] != PRODUCT_ID {
            self.present = false;
            return false;
        }

        // Soft reset, then wait for the sensor + coefficients to come ready.
        let _ = i2c.write(ADDR, &[REG_RESET, 0x09]);
        delay_us(40_000);

        // Poll MEAS_CFG: bit7 COEF_RDY, bit6 SENSOR_RDY.
        let mut ready = false;
        for _ in 0..50 {
            let mut m = [0u8; 1];
            if i2c.write_read(ADDR, &[REG_MEAS_CFG], &mut m).is_ok()
                && (m[0] & 0xC0) == 0xC0
            {
                ready = true;
                break;
            }
            delay_us(2_000);
        }
        if !ready {
            self.present = false;
            return false;
        }

        // Read the 18 calibration bytes and unpack the nine coefficients.
        let mut c = [0u8; 18];
        if i2c.write_read(ADDR, &[REG_COEF], &mut c).is_err() {
            self.present = false;
            return false;
        }
        self.unpack_coefficients(&c);

        // Configure rates/oversampling and start continuous measurement.
        let _ = i2c.write(ADDR, &[REG_PRS_CFG, PRS_CFG]);
        let _ = i2c.write(ADDR, &[REG_TMP_CFG, TMP_CFG]);
        let _ = i2c.write(ADDR, &[REG_CFG, 0x00]); // no shift (≤8×), no FIFO/int
        let _ = i2c.write(ADDR, &[REG_MEAS_CFG, MEAS_CTRL_CONT]);

        self.present = true;
        true
    }

    fn unpack_coefficients(&mut self, c: &[u8; 18]) {
        let u = |v: u32, bits: u32| -> f32 {
            // Sign-extend a `bits`-wide two's-complement value.
            let shift = 32 - bits;
            ((v << shift) as i32 >> shift) as f32
        };
        let c0 = ((c[0] as u32) << 4) | ((c[1] as u32) >> 4);
        let c1 = (((c[1] as u32) & 0x0F) << 8) | c[2] as u32;
        let c00 = ((c[3] as u32) << 12) | ((c[4] as u32) << 4) | ((c[5] as u32) >> 4);
        let c10 = (((c[5] as u32) & 0x0F) << 16) | ((c[6] as u32) << 8) | c[7] as u32;
        let c01 = ((c[8] as u32) << 8) | c[9] as u32;
        let c11 = ((c[10] as u32) << 8) | c[11] as u32;
        let c20 = ((c[12] as u32) << 8) | c[13] as u32;
        let c21 = ((c[14] as u32) << 8) | c[15] as u32;
        let c30 = ((c[16] as u32) << 8) | c[17] as u32;

        self.c0 = u(c0, 12);
        self.c1 = u(c1, 12);
        self.c00 = u(c00, 20);
        self.c10 = u(c10, 20);
        self.c01 = u(c01, 16);
        self.c11 = u(c11, 16);
        self.c20 = u(c20, 16);
        self.c21 = u(c21, 16);
        self.c30 = u(c30, 16);
    }

    /// Read + compensate one sample.
    pub fn read<I2C, E>(&mut self, i2c: &mut I2C) -> BaroData
    where
        I2C: WriteRead<Error = E>,
    {
        if !self.present {
            return BaroData::default();
        }
        // Burst-read pressure (3) + temperature (3) from 0x00.
        let mut b = [0u8; 6];
        if i2c.write_read(ADDR, &[REG_PSR], &mut b).is_err() {
            return BaroData {
                healthy: false,
                ..Default::default()
            };
        }
        let praw = read24(b[0], b[1], b[2]);
        let traw = read24(b[3], b[4], b[5]);

        let praw_sc = praw / K_OVERSAMPLE_8X;
        let traw_sc = traw / K_OVERSAMPLE_8X;

        // SPL06 compensation polynomial (datasheet §8.11).
        let pressure = self.c00
            + praw_sc * (self.c10 + praw_sc * (self.c20 + praw_sc * self.c30))
            + traw_sc * self.c01
            + traw_sc * praw_sc * (self.c11 + praw_sc * self.c21);
        let temperature = 0.5 * self.c0 + self.c1 * traw_sc;

        // International barometric formula, ISA sea-level reference.
        let altitude = 44_330.0 * (1.0 - libm::powf(pressure / SEA_LEVEL_PA, 0.190_295));

        // Capture the launch reference on the first valid reading.
        if !self.ground_set && pressure > 30_000.0 {
            self.ground_pa = pressure;
            self.ground_set = true;
        }
        let rel_altitude = if self.ground_set {
            44_330.0 * (1.0 - libm::powf(pressure / self.ground_pa, 0.190_295))
        } else {
            0.0
        };

        BaroData {
            pressure_pa: pressure,
            temperature_c: temperature,
            altitude_m: altitude,
            rel_altitude_m: rel_altitude,
            healthy: true,
        }
    }
}

/// Sign-extend a 24-bit big-endian two's-complement value to f32.
fn read24(b2: u8, b1: u8, b0: u8) -> f32 {
    let raw = ((b2 as u32) << 16) | ((b1 as u32) << 8) | b0 as u32;
    ((raw << 8) as i32 >> 8) as f32
}
