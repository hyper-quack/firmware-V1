//! Minimal allocation-free MAVLink 2 encoder for the first telemetry slice.
//!
//! Only messages emitted by this firmware live here. Keeping the encoder small
//! avoids pulling a complete generated dialect into the flight-control binary.

use heapless::Vec;

pub const MAV_SYS_STATUS_SENSOR_3D_ACCEL: u32 = 1 << 1;
pub const MAV_SYS_STATUS_SENSOR_3D_GYRO: u32 = 1 << 2;

const STX_V2: u8 = 0xFD;
const SYSTEM_ID: u8 = 1;
const COMPONENT_ID: u8 = 1; // MAV_COMP_ID_AUTOPILOT1
const MAX_FRAME_LEN: usize = 280;

const MSG_HEARTBEAT: u32 = 0;
const CRC_HEARTBEAT: u8 = 50;
const MSG_SYS_STATUS: u32 = 1;
const CRC_SYS_STATUS: u8 = 124;
const MSG_HIGHRES_IMU: u32 = 105;
const CRC_HIGHRES_IMU: u8 = 93;
const MSG_SCKY_IMU_STATUS: u32 = 42_000;
const CRC_SCKY_IMU_STATUS: u8 = 38;

pub type Frame = Vec<u8, MAX_FRAME_LEN>;

/// Stateful MAVLink encoder. Sequence numbers are shared by all messages on
/// one link, as required by MAVLink packet-loss detection.
pub struct Encoder {
    sequence: u8,
}

impl Encoder {
    pub const fn new() -> Self {
        Self { sequence: 0 }
    }

    pub fn heartbeat(&mut self) -> Frame {
        let mut p = Payload::new();
        p.u32(0); // custom_mode
        p.u8(2); // MAV_TYPE_QUADROTOR
        p.u8(0); // MAV_AUTOPILOT_GENERIC
        p.u8(0); // base_mode: no armed/control flags yet
        p.u8(4); // MAV_STATE_ACTIVE
        p.u8(3); // mavlink_version (always 3 for MAVLink 2)
        self.frame(MSG_HEARTBEAT, CRC_HEARTBEAT, p.as_slice())
    }

    pub fn sys_status(&mut self, sensors_present: u32, sensors_healthy: u32) -> Frame {
        let mut p = Payload::new();
        p.u32(sensors_present);
        p.u32(sensors_present); // all detected sensors are enabled
        p.u32(sensors_healthy);
        p.u16(0); // load unavailable
        p.u16(u16::MAX); // battery voltage unavailable
        p.i16(-1); // battery current unavailable
        p.u16(0); // drop_rate_comm
        p.u16(0); // errors_comm
        p.u16(0); // errors_count1
        p.u16(0); // errors_count2
        p.u16(0); // errors_count3
        p.u16(0); // errors_count4
        p.i8(-1); // battery remaining unavailable
        self.frame(MSG_SYS_STATUS, CRC_SYS_STATUS, p.as_slice())
    }

    /// Standard HIGHRES_IMU (message 105). Acceleration is m/s^2, angular
    /// velocity is rad/s, and `id` identifies physical IMU 0 or 1.
    pub fn highres_imu(
        &mut self,
        time_usec: u64,
        id: u8,
        accel_g: [f32; 3],
        gyro_dps: [f32; 3],
    ) -> Frame {
        const G_TO_M_S2: f32 = 9.806_65;
        const DEG_TO_RAD: f32 = 0.017_453_293;

        let mut p = Payload::new();
        p.u64(time_usec);
        for value in accel_g {
            p.f32(value * G_TO_M_S2);
        }
        for value in gyro_dps {
            p.f32(value * DEG_TO_RAD);
        }
        // Magnetometer, pressure, altitude, and temperature are not measured
        // in this slice. NaN plus fields_updated=0 for those fields is explicit.
        for _ in 0..7 {
            p.f32(f32::NAN);
        }
        p.u16(0x003F); // x/y/z accel + x/y/z gyro updated
        p.u8(id); // MAVLink 2 extension field
        self.frame(MSG_HIGHRES_IMU, CRC_HIGHRES_IMU, p.as_slice())
    }

    /// Per-device status from `message_definitions/scky.xml`.
    pub fn imu_status(
        &mut self,
        time_boot_ms: u32,
        imu_id: u8,
        connected: bool,
        healthy: bool,
        whoami: u8,
    ) -> Frame {
        let mut p = Payload::new();
        p.u32(time_boot_ms);
        p.u8(imu_id);
        p.u8(connected as u8);
        p.u8(healthy as u8);
        p.u8(whoami);
        self.frame(MSG_SCKY_IMU_STATUS, CRC_SCKY_IMU_STATUS, p.as_slice())
    }

    fn frame(&mut self, message_id: u32, crc_extra: u8, payload: &[u8]) -> Frame {
        let mut out = Frame::new();
        let header = [
            payload.len() as u8,
            0, // incompat_flags: unsigned packet
            0, // compat_flags
            self.sequence,
            SYSTEM_ID,
            COMPONENT_ID,
            message_id as u8,
            (message_id >> 8) as u8,
            (message_id >> 16) as u8,
        ];
        self.sequence = self.sequence.wrapping_add(1);

        let _ = out.push(STX_V2);
        let _ = out.extend_from_slice(&header);
        let _ = out.extend_from_slice(payload);

        let mut crc = 0xFFFF;
        for &byte in header.iter().chain(payload.iter()) {
            crc = crc_accumulate(byte, crc);
        }
        crc = crc_accumulate(crc_extra, crc);
        let _ = out.extend_from_slice(&crc.to_le_bytes());
        out
    }
}

fn crc_accumulate(byte: u8, crc: u16) -> u16 {
    let mut tmp = byte ^ crc as u8;
    tmp ^= tmp << 4;
    (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4)
}

struct Payload {
    bytes: Vec<u8, 255>,
}

impl Payload {
    fn new() -> Self {
        Self { bytes: Vec::new() }
    }

    fn as_slice(&self) -> &[u8] {
        self.bytes.as_slice()
    }

    fn u8(&mut self, value: u8) {
        let _ = self.bytes.push(value);
    }

    fn i8(&mut self, value: i8) {
        self.u8(value as u8);
    }

    fn u16(&mut self, value: u16) {
        let _ = self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn i16(&mut self, value: i16) {
        let _ = self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn u32(&mut self, value: u32) {
        let _ = self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn u64(&mut self, value: u64) {
        let _ = self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn f32(&mut self, value: f32) {
        let _ = self.bytes.extend_from_slice(&value.to_le_bytes());
    }
}
