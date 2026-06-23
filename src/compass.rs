//! 3-axis magnetometer driver for the external compass on the GPS module.
//!
//! Wired to **I2C2** (PB10 SCL / PB11 SDA) — the board's only I2C bus, shared
//! with the SPL06 barometer (0x76). ArduPilot probes external I2C buses for the
//! compass, so the magnetometer lives here alongside the baro.
//!
//! Modules sold as "NEO-M8N + HMC5883" almost always carry a **QMC5883L** clone
//! (I2C 0x0D), not a genuine Honeywell **HMC5883L** (0x1E). This driver probes
//! for both and adapts its register map + axis order accordingly, so either part
//! works without a config change.
//!
//! The bus is **not owned** by this driver — every method takes `&mut I2C`, so a
//! single task can drive both the compass and the baro on the shared I2C2 bus
//! (see [`crate::baro`]).
//!
//! Output is in **Gauss** (the unit MAVLink `HIGHRES_IMU` mag fields expect).

use embedded_hal::blocking::i2c::{Write, WriteRead};

const QMC_ADDR: u8 = 0x0D;
const HMC_ADDR: u8 = 0x1E;

// QMC5883L registers.
const QMC_REG_DATA: u8 = 0x00; // X_LSB,X_MSB,Y_LSB,Y_MSB,Z_LSB,Z_MSB
const QMC_REG_CTRL1: u8 = 0x09;
const QMC_REG_SETRESET: u8 = 0x0B;
const QMC_REG_CHIPID: u8 = 0x0D; // reads 0xFF on QMC5883L
// CTRL1 = OSR 512 | RNG 8G | ODR 200 Hz | MODE continuous.
const QMC_CTRL1_CONT: u8 = 0b0001_1101;
const QMC_LSB_PER_GAUSS: f32 = 3000.0; // 8 G range

// HMC5883L registers.
const HMC_REG_CFG_A: u8 = 0x00;
const HMC_REG_CFG_B: u8 = 0x01;
const HMC_REG_MODE: u8 = 0x02;
const HMC_REG_DATA: u8 = 0x03; // X_MSB,X_LSB,Z_MSB,Z_LSB,Y_MSB,Y_LSB
const HMC_REG_IDA: u8 = 0x0A; // 'H','4','3'
const HMC_LSB_PER_GAUSS: f32 = 1090.0; // gain 0xA0 (±1.3 Ga)

/// Which silicon was detected.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum MagKind {
    #[default]
    None,
    Qmc5883l,
    Hmc5883l,
}

impl MagKind {
    pub fn name(&self) -> &'static str {
        match self {
            MagKind::None => "none",
            MagKind::Qmc5883l => "QMC5883L",
            MagKind::Hmc5883l => "HMC5883L",
        }
    }
}

/// Latest magnetometer reading in Gauss, board (sensor) frame.
#[derive(Clone, Copy, Default)]
pub struct MagData {
    pub field: [f32; 3],
    pub kind: MagKind,
    pub healthy: bool,
}

/// Magnetometer state. The I2C bus is passed in per call (shared bus).
#[derive(Default)]
pub struct Compass {
    kind: MagKind,
}

impl Compass {
    pub const fn new() -> Self {
        Self {
            kind: MagKind::None,
        }
    }

    pub fn kind(&self) -> MagKind {
        self.kind
    }

    /// Probe both addresses and configure whichever part answers. Returns the
    /// detected kind (`None` if nothing responded).
    pub fn init<I2C, E>(&mut self, i2c: &mut I2C) -> MagKind
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        // QMC5883L: chip-id register 0x0D reads 0xFF.
        let mut id = [0u8; 1];
        if i2c.write_read(QMC_ADDR, &[QMC_REG_CHIPID], &mut id).is_ok() && id[0] == 0xFF {
            // Recommended set/reset period, then continuous mode.
            let _ = i2c.write(QMC_ADDR, &[QMC_REG_SETRESET, 0x01]);
            let _ = i2c.write(QMC_ADDR, &[QMC_REG_CTRL1, QMC_CTRL1_CONT]);
            self.kind = MagKind::Qmc5883l;
            return self.kind;
        }

        // HMC5883L: identity registers 0x0A..0x0C read 'H','4','3'.
        let mut hid = [0u8; 3];
        if i2c.write_read(HMC_ADDR, &[HMC_REG_IDA], &mut hid).is_ok() && &hid == b"H43" {
            let _ = i2c.write(HMC_ADDR, &[HMC_REG_CFG_A, 0x70]); // 8-avg, 15 Hz
            let _ = i2c.write(HMC_ADDR, &[HMC_REG_CFG_B, 0xA0]); // gain ±1.3 Ga
            let _ = i2c.write(HMC_ADDR, &[HMC_REG_MODE, 0x00]); // continuous
            self.kind = MagKind::Hmc5883l;
            return self.kind;
        }

        self.kind = MagKind::None;
        self.kind
    }

    /// Read one sample. On bus error returns an unhealthy [`MagData`].
    pub fn read<I2C, E>(&mut self, i2c: &mut I2C) -> MagData
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        match self.kind {
            MagKind::Qmc5883l => self.read_qmc(i2c),
            MagKind::Hmc5883l => self.read_hmc(i2c),
            MagKind::None => MagData::default(),
        }
    }

    fn read_qmc<I2C, E>(&mut self, i2c: &mut I2C) -> MagData
    where
        I2C: WriteRead<Error = E>,
    {
        let mut b = [0u8; 6];
        if i2c.write_read(QMC_ADDR, &[QMC_REG_DATA], &mut b).is_err() {
            return MagData {
                kind: self.kind,
                healthy: false,
                ..Default::default()
            };
        }
        // Little-endian, axis order X, Y, Z.
        let x = le16(b[0], b[1]);
        let y = le16(b[2], b[3]);
        let z = le16(b[4], b[5]);
        MagData {
            field: [
                x as f32 / QMC_LSB_PER_GAUSS,
                y as f32 / QMC_LSB_PER_GAUSS,
                z as f32 / QMC_LSB_PER_GAUSS,
            ],
            kind: self.kind,
            healthy: true,
        }
    }

    fn read_hmc<I2C, E>(&mut self, i2c: &mut I2C) -> MagData
    where
        I2C: WriteRead<Error = E>,
    {
        let mut b = [0u8; 6];
        if i2c.write_read(HMC_ADDR, &[HMC_REG_DATA], &mut b).is_err() {
            return MagData {
                kind: self.kind,
                healthy: false,
                ..Default::default()
            };
        }
        // Big-endian, axis order X, Z, Y (note the swap).
        let x = be16(b[0], b[1]);
        let z = be16(b[2], b[3]);
        let y = be16(b[4], b[5]);
        MagData {
            field: [
                x as f32 / HMC_LSB_PER_GAUSS,
                y as f32 / HMC_LSB_PER_GAUSS,
                z as f32 / HMC_LSB_PER_GAUSS,
            ],
            kind: self.kind,
            healthy: true,
        }
    }
}

fn le16(lo: u8, hi: u8) -> i16 {
    (((hi as u16) << 8) | lo as u16) as i16
}

fn be16(hi: u8, lo: u8) -> i16 {
    (((hi as u16) << 8) | lo as u16) as i16
}
