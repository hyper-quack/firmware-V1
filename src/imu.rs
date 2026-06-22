//! TDK InvenSense v3 IMU driver (ICM-426xx / ICM-456xy family) over SPI.
//!
//! The DAKEFPVH743 hwdef declares BOTH IMUs as `IMU Invensensev3` — ArduPilot
//! auto-detects the exact part from the WHO_AM_I register, so we do the same.
//! On the production board the parts are almost always ICM-42688-P
//! (WHO_AM_I = 0x47), but this driver accepts the whole family and reports the
//! ID it actually finds over USB so you can confirm the real silicon.
//!
//! Bus parameters come straight from the ArduPilot hwdef:
//!   SPIDEV imu1 SPI1 DEVID1 GYRO1_CS MODE3 1*MHZ 16*MHZ   (rotation ROLL_180)
//!   SPIDEV imu2 SPI4 DEVID1 GYRO2_CS MODE3 1*MHZ 16*MHZ   (rotation PITCH_180)
//!
//! SPI MODE3 (CPOL=1, CPHA=1), MSB first, software chip-select. Register reads
//! set bit 7 of the address; writes clear it.

use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;

// --- Bank-0 register map (common across the v3 family) ---------------------
const REG_DEVICE_CONFIG: u8 = 0x11; // bit0 = soft reset
const REG_TEMP_DATA1: u8 = 0x1D; // start of the contiguous data block
const REG_PWR_MGMT0: u8 = 0x4E;
const REG_GYRO_CONFIG0: u8 = 0x4F;
const REG_ACCEL_CONFIG0: u8 = 0x50;
const REG_WHO_AM_I: u8 = 0x75;

const READ: u8 = 0x80; // OR into the address byte to request a read

/// WHO_AM_I values for the parts ArduPilot's Invensensev3 backend supports.
/// We treat any of these as "this is a valid v3 IMU".
const KNOWN_WHOAMI: &[(u8, &str)] = &[
    (0x47, "ICM-42688-P"),
    (0x42, "ICM-42605"),
    (0x6F, "IIM-42652"),
    (0x49, "ICM-42686-P"),
    (0x67, "ICM-42670-P"),
    (0x3B, "ICM-40609-D"),
    (0xE9, "ICM-45686"),
    (0xDB, "ICM-40605"),
];

/// One synchronous accel + gyro + temperature reading, raw counts.
#[derive(Clone, Copy, Default)]
pub struct Sample {
    pub acc: [i16; 3],
    pub gyr: [i16; 3],
    pub temp: i16,
}

impl Sample {
    /// Gyro in deg/s, assuming the ±2000 dps full-scale configured in `init`.
    pub fn gyro_dps(&self) -> [f32; 3] {
        const LSB_PER_DPS: f32 = 16.4; // 2000 dps FS
        [
            self.gyr[0] as f32 / LSB_PER_DPS,
            self.gyr[1] as f32 / LSB_PER_DPS,
            self.gyr[2] as f32 / LSB_PER_DPS,
        ]
    }

    /// Accel in g, assuming the ±16 g full-scale configured in `init`.
    pub fn accel_g(&self) -> [f32; 3] {
        const LSB_PER_G: f32 = 2048.0; // 16 g FS
        [
            self.acc[0] as f32 / LSB_PER_G,
            self.acc[1] as f32 / LSB_PER_G,
            self.acc[2] as f32 / LSB_PER_G,
        ]
    }

    /// Roll angle (deg) derived from the gravity vector: rotation about X.
    /// NOTE: raw sensor frame — the board mount rotation is not yet applied,
    /// so the sign/axis may differ from the airframe until the estimator lands.
    pub fn roll_deg(&self) -> f32 {
        let a = self.accel_g();
        libm::atan2f(a[1], a[2]) * 57.295_78
    }

    /// Pitch angle (deg) derived from the gravity vector: rotation about Y.
    pub fn pitch_deg(&self) -> f32 {
        let a = self.accel_g();
        libm::atan2f(-a[0], libm::sqrtf(a[1] * a[1] + a[2] * a[2])) * 57.295_78
    }
}

/// Detection / health result for an IMU.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum Health {
    /// Not probed yet.
    #[default]
    Unknown,
    /// WHO_AM_I matched a known v3 part. Field is the raw ID byte.
    Ok(u8),
    /// SPI returned an all-low / all-high bus (wiring / power / CS fault), or
    /// WHO_AM_I did not match any known part. Field is the byte we read.
    Bad(u8),
}

impl Health {
    pub fn whoami(&self) -> u8 {
        match self {
            Health::Ok(id) | Health::Bad(id) => *id,
            Health::Unknown => 0,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Health::Ok(id) => part_name(*id),
            Health::Bad(_) => "INVALID",
            Health::Unknown => "?",
        }
    }
}

fn part_name(id: u8) -> &'static str {
    let mut i = 0;
    while i < KNOWN_WHOAMI.len() {
        if KNOWN_WHOAMI[i].0 == id {
            return KNOWN_WHOAMI[i].1;
        }
        i += 1;
    }
    "UNKNOWN-v3"
}

fn is_known(id: u8) -> bool {
    let mut i = 0;
    while i < KNOWN_WHOAMI.len() {
        if KNOWN_WHOAMI[i].0 == id {
            return true;
        }
        i += 1;
    }
    false
}

/// Filtered, physical-unit output of one IMU channel, published by the sampling
/// task for the estimator to consume. Gyro in deg/s, accel in g, both already
/// low-pass filtered and in the *raw sensor frame* (board rotation applied later
/// by the estimator).
#[derive(Clone, Copy, Default)]
pub struct ImuOut {
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
    pub health: Health,
}

/// An InvenSense v3 IMU bound to a specific SPI bus + software CS pin.
///
/// Generic over the concrete HAL types so the two IMUs (SPI1/PA4 and
/// SPI4/PB1) can each be their own monomorphised instance.
pub struct Imu<SPI, CS> {
    spi: SPI,
    cs: CS,
    pub health: Health,
}

impl<SPI, CS> Imu<SPI, CS>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    /// Wrap an already-configured SPI bus and CS pin. CS is driven idle-high.
    pub fn new(spi: SPI, mut cs: CS) -> Self {
        let _ = cs.set_high();
        Self {
            spi,
            cs,
            health: Health::Unknown,
        }
    }

    #[inline]
    fn read_reg(&mut self, reg: u8) -> u8 {
        let mut buf = [reg | READ, 0x00];
        let _ = self.cs.set_low();
        let _ = self.spi.transfer(&mut buf);
        let _ = self.cs.set_high();
        buf[1]
    }

    #[inline]
    fn write_reg(&mut self, reg: u8, val: u8) {
        let mut buf = [reg & !READ, val];
        let _ = self.cs.set_low();
        let _ = self.spi.transfer(&mut buf);
        let _ = self.cs.set_high();
    }

    /// Probe + configure the IMU. `delay_us` blocks for the given microseconds
    /// (init only runs once, so a busy-wait is fine here).
    ///
    /// Configuration: ±2000 dps gyro, ±16 g accel, 1 kHz ODR, both sensors in
    /// low-noise mode. Returns the resulting [`Health`].
    pub fn init(&mut self, delay_us: &dyn Fn(u32)) -> Health {
        // Soft reset, then wait for the device to come back (datasheet: 1 ms).
        self.write_reg(REG_DEVICE_CONFIG, 0x01);
        delay_us(2_000);

        let id = self.read_reg(REG_WHO_AM_I);
        if id == 0x00 || id == 0xFF || !is_known(id) {
            // 0x00 / 0xFF strongly indicate a dead bus (no MISO, no power, or
            // wrong CS). Anything else unknown is still a failure.
            self.health = Health::Bad(id);
            return self.health;
        }

        // GYRO_CONFIG0:  bits[7:5]=FS_SEL (0 => 2000 dps), bits[3:0]=ODR (6 => 1 kHz)
        self.write_reg(REG_GYRO_CONFIG0, 0x06);
        // ACCEL_CONFIG0: bits[7:5]=FS_SEL (0 => 16 g),    bits[3:0]=ODR (6 => 1 kHz)
        self.write_reg(REG_ACCEL_CONFIG0, 0x06);

        // PWR_MGMT0: gyro + accel in Low-Noise mode (0b1111). Datasheet requires
        // no register writes for 200 us after this; give it margin.
        self.write_reg(REG_PWR_MGMT0, 0x0F);
        delay_us(1_000);

        self.health = Health::Ok(id);
        self.health
    }

    /// Burst-read temperature + accel + gyro in one transaction.
    ///
    /// The data registers are contiguous from TEMP_DATA1 (0x1D):
    ///   0x1D..0x1F temp, 0x1F..0x25 accel XYZ, 0x25..0x2B gyro XYZ
    /// all big-endian (high byte first).
    pub fn read(&mut self) -> Sample {
        let mut buf = [0u8; 1 + 14];
        buf[0] = REG_TEMP_DATA1 | READ;
        let _ = self.cs.set_low();
        let _ = self.spi.transfer(&mut buf);
        let _ = self.cs.set_high();

        let be = |hi: usize| -> i16 { (((buf[hi] as u16) << 8) | buf[hi + 1] as u16) as i16 };

        Sample {
            temp: be(1),
            acc: [be(3), be(5), be(7)],
            gyr: [be(9), be(11), be(13)],
        }
    }
}
