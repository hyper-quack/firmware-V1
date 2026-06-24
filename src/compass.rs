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

/// Compass mounting orientation, board-relative. A compass on a GPS mast is
/// frequently rotated in yaw (or flipped) relative to the flight controller.
#[derive(Clone, Copy)]
pub enum MagRotation {
    None,
    Yaw90,
    Yaw180,
    Yaw270,
    /// Mounted upside-down.
    Roll180,
}

impl MagRotation {
    /// Rotate a field vector into the body frame.
    fn apply(self, v: [f32; 3]) -> [f32; 3] {
        match self {
            MagRotation::None => v,
            MagRotation::Yaw90 => [-v[1], v[0], v[2]],
            MagRotation::Yaw180 => [-v[0], -v[1], v[2]],
            MagRotation::Yaw270 => [v[1], -v[0], v[2]],
            MagRotation::Roll180 => [v[0], -v[1], -v[2]],
        }
    }
}

/// Compass calibration: mounting rotation + hard-iron offset + soft-iron scale,
/// with an optional online min/max collector that estimates the hard-iron offset.
///
/// Correction order (the physical one): the iron distortion is fixed in the
/// **sensor** frame, so offset/scale are applied to the raw reading *first*, then
/// the mounting rotation maps it into the body frame.
///
/// ```text
///   corrected_body = rotation · ( scale ⊙ (raw − offset) )
/// ```
#[derive(Clone, Copy)]
pub struct MagCal {
    pub rotation: MagRotation,
    /// Hard-iron offset (Gauss), subtracted from the raw field.
    pub offset: [f32; 3],
    /// Soft-iron diagonal scale, multiplied after offset removal.
    pub scale: [f32; 3],
    // --- online hard-iron collection ---
    collecting: bool,
    min: [f32; 3],
    max: [f32; 3],
}

impl MagCal {
    /// Identity calibration (no rotation, no offset, unit scale). Replace the
    /// fields with bench-measured values, or run the online collector.
    pub const fn identity() -> Self {
        Self {
            rotation: MagRotation::None,
            offset: [0.0; 3],
            scale: [1.0; 3],
            collecting: false,
            min: [0.0; 3],
            max: [0.0; 3],
        }
    }

    pub const fn new(rotation: MagRotation, offset: [f32; 3], scale: [f32; 3]) -> Self {
        Self {
            rotation,
            offset,
            scale,
            collecting: false,
            min: [0.0; 3],
            max: [0.0; 3],
        }
    }

    /// Apply the full correction to a raw field (Gauss), returning a body-frame
    /// vector. Also feeds the online collector when active.
    pub fn apply(&mut self, raw: [f32; 3]) -> [f32; 3] {
        if self.collecting {
            for i in 0..3 {
                if raw[i] < self.min[i] {
                    self.min[i] = raw[i];
                }
                if raw[i] > self.max[i] {
                    self.max[i] = raw[i];
                }
            }
        }
        let c = [
            (raw[0] - self.offset[0]) * self.scale[0],
            (raw[1] - self.offset[1]) * self.scale[1],
            (raw[2] - self.offset[2]) * self.scale[2],
        ];
        self.rotation.apply(c)
    }

    /// Begin online hard-iron collection. Rotate the vehicle through all
    /// orientations (figure-8s) while this runs, then call [`finish_collection`].
    pub fn start_collection(&mut self) {
        self.collecting = true;
        self.min = [f32::MAX; 3];
        self.max = [f32::MIN; 3];
    }

    /// Finish collection: hard-iron offset = midpoint of each axis range,
    /// soft-iron scale = equalise each axis's range to the average range. No-op if
    /// too little data was gathered.
    pub fn finish_collection(&mut self) {
        self.collecting = false;
        let mut range = [0.0f32; 3];
        for i in 0..3 {
            range[i] = (self.max[i] - self.min[i]) * 0.5;
        }
        if range[0] <= 0.0 || range[1] <= 0.0 || range[2] <= 0.0 {
            return; // not enough motion
        }
        let avg = (range[0] + range[1] + range[2]) / 3.0;
        for i in 0..3 {
            self.offset[i] = (self.max[i] + self.min[i]) * 0.5;
            self.scale[i] = avg / range[i];
        }
    }
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
