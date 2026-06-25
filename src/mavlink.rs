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
const MSG_GPS_RAW_INT: u32 = 24;
const CRC_GPS_RAW_INT: u8 = 24;
const MSG_SCALED_PRESSURE: u32 = 29;
const CRC_SCALED_PRESSURE: u8 = 115;
const MSG_ATTITUDE: u32 = 30;
const CRC_ATTITUDE: u8 = 39;
const MSG_LOCAL_POSITION_NED: u32 = 32;
const CRC_LOCAL_POSITION_NED: u8 = 185;
const MSG_GLOBAL_POSITION_INT: u32 = 33;
const CRC_GLOBAL_POSITION_INT: u8 = 104;
const MSG_RC_CHANNELS: u32 = 65;
const CRC_RC_CHANNELS: u8 = 118;
const MSG_OPTICAL_FLOW: u32 = 100;
const CRC_OPTICAL_FLOW: u8 = 175;
const MSG_DISTANCE_SENSOR: u32 = 132;
const CRC_DISTANCE_SENSOR: u8 = 85;
const MSG_HIGHRES_IMU: u32 = 105;
const CRC_HIGHRES_IMU: u8 = 93;
const MSG_STATUSTEXT: u32 = 253;
const CRC_STATUSTEXT: u8 = 83;
const MSG_SCKY_IMU_STATUS: u32 = 42_000;
const CRC_SCKY_IMU_STATUS: u8 = 38;
const MSG_SCKY_ESC_TELEM: u32 = 42_010;
const CRC_SCKY_ESC_TELEM: u8 = 91;
const MSG_SCKY_ESC_CONFIG: u32 = 42_011;
const CRC_SCKY_ESC_CONFIG: u8 = 55;

// Inbound (ground-station → FC) message ids + crc_extra, used by `Decoder`.
const MSG_COMMAND_LONG: u32 = 76;
const CRC_COMMAND_LONG: u8 = 152;
const MSG_SCKY_ESC_SET: u32 = 42_012;
const CRC_SCKY_ESC_SET: u8 = 8;
const MSG_SCKY_ESC_CMD: u32 = 42_013;
const CRC_SCKY_ESC_CMD: u8 = 106;

/// `MAV_CMD_DO_MOTOR_TEST` command id used inside `COMMAND_LONG`.
const MAV_CMD_DO_MOTOR_TEST: u16 = 209;

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
    /// velocity is rad/s, magnetometer (when present) is Gauss, and `id`
    /// identifies physical IMU 0 or 1. Pass `mag_ga = None` when this stream has
    /// no magnetometer attached.
    pub fn highres_imu(
        &mut self,
        time_usec: u64,
        id: u8,
        accel_g: [f32; 3],
        gyro_dps: [f32; 3],
        mag_ga: Option<[f32; 3]>,
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
        // Magnetometer (Gauss). Pressure/altitude/temperature are not measured
        // in this slice — NaN with their fields_updated bits clear is explicit.
        match mag_ga {
            Some(m) => {
                for value in m {
                    p.f32(value);
                }
            }
            None => {
                for _ in 0..3 {
                    p.f32(f32::NAN);
                }
            }
        }
        for _ in 0..4 {
            p.f32(f32::NAN); // abs_pressure, diff_pressure, pressure_alt, temperature
        }
        // bits 0..5 = x/y/z accel + x/y/z gyro; bits 6..8 = x/y/z mag.
        let updated = if mag_ga.is_some() { 0x01FF } else { 0x003F };
        p.u16(updated);
        p.u8(id); // MAVLink 2 extension field
        self.frame(MSG_HIGHRES_IMU, CRC_HIGHRES_IMU, p.as_slice())
    }

    /// SCALED_PRESSURE (message 29): barometer. `press_abs`/`press_diff` in hPa,
    /// `temperature` in centidegrees Celsius.
    pub fn scaled_pressure(
        &mut self,
        time_boot_ms: u32,
        press_abs_hpa: f32,
        press_diff_hpa: f32,
        temperature_cdeg: i16,
    ) -> Frame {
        let mut p = Payload::new();
        p.u32(time_boot_ms);
        p.f32(press_abs_hpa);
        p.f32(press_diff_hpa);
        p.i16(temperature_cdeg);
        self.frame(MSG_SCALED_PRESSURE, CRC_SCALED_PRESSURE, p.as_slice())
    }

    /// Standard ATTITUDE (message 30). Angles in radians, rates in rad/s.
    pub fn attitude(
        &mut self,
        time_boot_ms: u32,
        roll_rad: f32,
        pitch_rad: f32,
        yaw_rad: f32,
        rollspeed: f32,
        pitchspeed: f32,
        yawspeed: f32,
    ) -> Frame {
        let mut p = Payload::new();
        p.u32(time_boot_ms);
        p.f32(roll_rad);
        p.f32(pitch_rad);
        p.f32(yaw_rad);
        p.f32(rollspeed);
        p.f32(pitchspeed);
        p.f32(yawspeed);
        self.frame(MSG_ATTITUDE, CRC_ATTITUDE, p.as_slice())
    }

    /// LOCAL_POSITION_NED (message 32): fused position/velocity in the local
    /// tangent frame, metres and m/s, **NED** (z down, vz down-positive).
    #[allow(clippy::too_many_arguments)]
    pub fn local_position_ned(
        &mut self,
        time_boot_ms: u32,
        x: f32,
        y: f32,
        z: f32,
        vx: f32,
        vy: f32,
        vz: f32,
    ) -> Frame {
        let mut p = Payload::new();
        p.u32(time_boot_ms);
        p.f32(x);
        p.f32(y);
        p.f32(z);
        p.f32(vx);
        p.f32(vy);
        p.f32(vz);
        self.frame(MSG_LOCAL_POSITION_NED, CRC_LOCAL_POSITION_NED, p.as_slice())
    }

    /// Standard GLOBAL_POSITION_INT (message 33): fused/global position. lat/lon
    /// in 1e7-deg, altitudes in mm, velocities in cm/s, heading in centidegrees
    /// (65535 = unknown).
    #[allow(clippy::too_many_arguments)]
    pub fn global_position_int(
        &mut self,
        time_boot_ms: u32,
        lat_e7: i32,
        lon_e7: i32,
        alt_mm: i32,
        relative_alt_mm: i32,
        vx_cms: i16,
        vy_cms: i16,
        vz_cms: i16,
        hdg_cdeg: u16,
    ) -> Frame {
        let mut p = Payload::new();
        p.u32(time_boot_ms);
        p.i32(lat_e7);
        p.i32(lon_e7);
        p.i32(alt_mm);
        p.i32(relative_alt_mm);
        p.i16(vx_cms);
        p.i16(vy_cms);
        p.i16(vz_cms);
        p.u16(hdg_cdeg);
        self.frame(MSG_GLOBAL_POSITION_INT, CRC_GLOBAL_POSITION_INT, p.as_slice())
    }

    /// Standard GPS_RAW_INT (message 24). Units match the wire format directly:
    /// lat/lon in degrees * 1e7, alt in mm (MSL), eph = HDOP * 100, vel in cm/s,
    /// cog in centidegrees.
    #[allow(clippy::too_many_arguments)]
    pub fn gps_raw_int(
        &mut self,
        time_usec: u64,
        fix_type: u8,
        lat_e7: i32,
        lon_e7: i32,
        alt_mm: i32,
        eph: u16,
        vel_cms: u16,
        cog_cdeg: u16,
        sats: u8,
    ) -> Frame {
        let mut p = Payload::new();
        p.u64(time_usec);
        p.i32(lat_e7);
        p.i32(lon_e7);
        p.i32(alt_mm);
        p.u16(eph);
        p.u16(u16::MAX); // epv (VDOP) unknown
        p.u16(vel_cms);
        p.u16(cog_cdeg);
        p.u8(fix_type);
        p.u8(sats);
        self.frame(MSG_GPS_RAW_INT, CRC_GPS_RAW_INT, p.as_slice())
    }

    /// STATUSTEXT (message 253): human-readable diagnostic string.  `severity` is
    /// a `MAV_SEVERITY` value (6 = DEBUG).  `text` is truncated / zero-padded to
    /// exactly 50 bytes as required by the wire format.
    pub fn statustext(&mut self, severity: u8, text: &str) -> Frame {
        let mut p = Payload::new();
        p.u8(severity);
        let bytes = text.as_bytes();
        for i in 0..50_usize {
            p.u8(if i < bytes.len() { bytes[i] } else { 0 });
        }
        self.frame(MSG_STATUSTEXT, CRC_STATUSTEXT, p.as_slice())
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

    /// DISTANCE_SENSOR (message 132). Distances in cm. `orientation` is a
    /// MAV_SENSOR_ORIENTATION (25 = down, 0 = forward, 2 = right, 6 = left);
    /// `id` distinguishes multiple rangefinders.
    #[allow(clippy::too_many_arguments)]
    pub fn distance_sensor(
        &mut self,
        time_boot_ms: u32,
        min_cm: u16,
        max_cm: u16,
        current_cm: u16,
        orientation: u8,
        id: u8,
    ) -> Frame {
        let mut p = Payload::new();
        p.u32(time_boot_ms);
        p.u16(min_cm);
        p.u16(max_cm);
        p.u16(current_cm);
        p.u8(0); // type: 0 = laser rangefinder
        p.u8(id);
        p.u8(orientation);
        p.u8(0); // covariance unknown
        self.frame(MSG_DISTANCE_SENSOR, CRC_DISTANCE_SENSOR, p.as_slice())
    }

    /// OPTICAL_FLOW (message 100). `flow_comp_m` are de-rotated ground velocities
    /// (m/s), `ground_distance` is height (m), `quality` 0..255.
    #[allow(clippy::too_many_arguments)]
    pub fn optical_flow(
        &mut self,
        time_usec: u64,
        flow_x: i16,
        flow_y: i16,
        flow_comp_m_x: f32,
        flow_comp_m_y: f32,
        ground_distance: f32,
        quality: u8,
    ) -> Frame {
        let mut p = Payload::new();
        p.u64(time_usec);
        p.f32(flow_comp_m_x);
        p.f32(flow_comp_m_y);
        p.f32(ground_distance);
        p.i16(flow_x);
        p.i16(flow_y);
        p.u8(0); // sensor_id
        p.u8(quality);
        self.frame(MSG_OPTICAL_FLOW, CRC_OPTICAL_FLOW, p.as_slice())
    }

    /// RC_CHANNELS (message 65): up to 18 channels in µs. `rssi` 0..254
    /// (255 = unknown). Unused channels should be 65535 (UINT16_MAX).
    pub fn rc_channels(
        &mut self,
        time_boot_ms: u32,
        chancount: u8,
        ch: &[u16; 18],
        rssi: u8,
    ) -> Frame {
        let mut p = Payload::new();
        p.u32(time_boot_ms);
        for &c in ch.iter() {
            p.u16(c);
        }
        p.u8(chancount);
        p.u8(rssi);
        self.frame(MSG_RC_CHANNELS, CRC_RC_CHANNELS, p.as_slice())
    }

    /// SCKY_ESC_TELEM (42010): per-motor ESC telemetry plus aggregate consumption
    /// and analog current. `centivolt`/`centiamp` are hundredths of a volt/amp.
    #[allow(clippy::too_many_arguments)]
    pub fn esc_telem(
        &mut self,
        mah_consumed: f32,
        analog_current: f32,
        rpm: &[i32; 4],
        centivolt: &[u16; 4],
        centiamp: &[u16; 4],
        temperature: &[u8; 4],
        error_count: &[u8; 4],
    ) -> Frame {
        let mut p = Payload::new();
        p.f32(mah_consumed);
        p.f32(analog_current);
        for &v in rpm.iter() {
            p.i32(v);
        }
        for &v in centivolt.iter() {
            p.u16(v);
        }
        for &v in centiamp.iter() {
            p.u16(v);
        }
        for &v in temperature.iter() {
            p.u8(v);
        }
        for &v in error_count.iter() {
            p.u8(v);
        }
        self.frame(MSG_SCKY_ESC_TELEM, CRC_SCKY_ESC_TELEM, p.as_slice())
    }

    /// SCKY_ESC_CONFIG (42011): echo of the live ESC configuration so the ground
    /// station reflects the FC's actual state.
    #[allow(clippy::too_many_arguments)]
    pub fn esc_config(
        &mut self,
        cur_scale: f32,
        cur_offset: f32,
        refresh_hz: u16,
        protocol: u8,
        master_enabled: bool,
        bidir: bool,
        dir_mask: u8,
        pole_count: u8,
        mode3d_mask: u8,
    ) -> Frame {
        let mut p = Payload::new();
        p.f32(cur_scale);
        p.f32(cur_offset);
        p.u16(refresh_hz);
        p.u8(protocol);
        p.u8(master_enabled as u8);
        p.u8(bidir as u8);
        p.u8(dir_mask);
        p.u8(pole_count);
        p.u8(mode3d_mask);
        self.frame(MSG_SCKY_ESC_CONFIG, CRC_SCKY_ESC_CONFIG, p.as_slice())
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

    fn i32(&mut self, value: i32) {
        let _ = self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn u64(&mut self, value: u64) {
        let _ = self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn f32(&mut self, value: f32) {
        let _ = self.bytes.extend_from_slice(&value.to_le_bytes());
    }
}

// ===========================================================================
// Inbound (ground-station → FC) decoding.
//
// The firmware's first RX-side MAVLink. A small byte-at-a-time state machine
// reassembles MAVLink 2 frames, validates the CRC (with the per-message
// crc_extra), and yields the few commands this firmware acts on. Signed frames
// (incompat flag 0x01) are parsed and their 13-byte signature skipped; the
// signature itself is not verified. Payloads are zero-extended to the message's
// full length before field extraction so MAVLink-2 trailing-zero truncation is
// handled transparently.
// ===========================================================================

/// A command the firmware understands.
pub enum Inbound {
    /// MAV_CMD_DO_MOTOR_TEST. `motor` is 1-based.
    MotorTest {
        motor: u8,
        throttle_type: u8,
        throttle: f32,
        timeout_s: f32,
        count: u8,
    },
    /// SCKY_ESC_SET: write the live ESC configuration.
    EscSet {
        master_enabled: bool,
        protocol: u8,
        refresh_hz: u16,
        bidir: bool,
        dir_mask: u8,
        mode3d_mask: u8,
        pole_count: u8,
        cur_scale: f32,
        cur_offset: f32,
    },
    /// SCKY_ESC_CMD: one-shot DShot special command. `target` 0 = all, 1..4 = one.
    EscCmd { target: u8, command: u16 },
}

#[derive(Clone, Copy, PartialEq)]
enum St {
    Stx,
    Len,
    Incompat,
    Compat,
    Seq,
    SysId,
    CompId,
    MsgId0,
    MsgId1,
    MsgId2,
    Payload,
    Crc0,
    Crc1,
    Sig,
}

const MAX_PAYLOAD: usize = 64;

/// Streaming MAVLink 2 frame decoder.
pub struct Decoder {
    st: St,
    len: usize,
    incompat: u8,
    msgid: u32,
    payload: [u8; MAX_PAYLOAD],
    idx: usize,
    crc_rx: u16,
    crc_calc: u16,
    sig_left: u8,
}

impl Decoder {
    pub const fn new() -> Self {
        Self {
            st: St::Stx,
            len: 0,
            incompat: 0,
            msgid: 0,
            payload: [0; MAX_PAYLOAD],
            idx: 0,
            crc_rx: 0,
            crc_calc: 0,
            sig_left: 0,
        }
    }

    /// crc_extra for the inbound messages we validate; `None` = not one we accept.
    fn crc_extra(msgid: u32) -> Option<u8> {
        match msgid {
            MSG_COMMAND_LONG => Some(CRC_COMMAND_LONG),
            MSG_SCKY_ESC_SET => Some(CRC_SCKY_ESC_SET),
            MSG_SCKY_ESC_CMD => Some(CRC_SCKY_ESC_CMD),
            _ => None,
        }
    }

    /// Feed one received byte. Returns a decoded command when a CRC-valid frame
    /// for a recognised message completes.
    pub fn push(&mut self, byte: u8) -> Option<Inbound> {
        match self.st {
            St::Stx => {
                if byte == STX_V2 {
                    self.crc_calc = 0xFFFF;
                    self.idx = 0;
                    self.msgid = 0;
                    self.st = St::Len;
                }
            }
            St::Len => {
                self.len = byte as usize;
                self.accumulate(byte);
                self.st = St::Incompat;
            }
            St::Incompat => {
                self.incompat = byte;
                self.accumulate(byte);
                self.st = St::Compat;
            }
            St::Compat => {
                self.accumulate(byte);
                self.st = St::Seq;
            }
            St::Seq => {
                self.accumulate(byte);
                self.st = St::SysId;
            }
            St::SysId => {
                self.accumulate(byte);
                self.st = St::CompId;
            }
            St::CompId => {
                self.accumulate(byte);
                self.st = St::MsgId0;
            }
            St::MsgId0 => {
                self.msgid = byte as u32;
                self.accumulate(byte);
                self.st = St::MsgId1;
            }
            St::MsgId1 => {
                self.msgid |= (byte as u32) << 8;
                self.accumulate(byte);
                self.st = St::MsgId2;
            }
            St::MsgId2 => {
                self.msgid |= (byte as u32) << 16;
                self.accumulate(byte);
                self.st = if self.len == 0 { St::Crc0 } else { St::Payload };
            }
            St::Payload => {
                if self.idx < MAX_PAYLOAD {
                    self.payload[self.idx] = byte;
                }
                self.accumulate(byte);
                self.idx += 1;
                if self.idx >= self.len {
                    self.st = St::Crc0;
                }
            }
            St::Crc0 => {
                self.crc_rx = byte as u16;
                self.st = St::Crc1;
            }
            St::Crc1 => {
                self.crc_rx |= (byte as u16) << 8;
                let result = self.finish();
                // After the CRC, a signed frame carries 13 signature bytes.
                if self.incompat & 0x01 != 0 {
                    self.sig_left = 13;
                    self.st = St::Sig;
                } else {
                    self.st = St::Stx;
                }
                return result;
            }
            St::Sig => {
                self.sig_left -= 1;
                if self.sig_left == 0 {
                    self.st = St::Stx;
                }
            }
        }
        None
    }

    fn accumulate(&mut self, byte: u8) {
        self.crc_calc = crc_accumulate(byte, self.crc_calc);
    }

    /// Validate CRC and, if the message is recognised, decode it. Always leaves
    /// the machine ready (caller resets `st`).
    fn finish(&mut self) -> Option<Inbound> {
        let extra = Self::crc_extra(self.msgid)?;
        let crc = crc_accumulate(extra, self.crc_calc);
        if crc != self.crc_rx {
            return None;
        }
        // Zero-extend the (possibly truncated) payload for field extraction.
        let mut p = [0u8; MAX_PAYLOAD];
        let n = self.len.min(MAX_PAYLOAD);
        p[..n].copy_from_slice(&self.payload[..n]);

        match self.msgid {
            MSG_COMMAND_LONG => {
                let command = u16::from_le_bytes([p[28], p[29]]);
                if command != MAV_CMD_DO_MOTOR_TEST {
                    return None;
                }
                Some(Inbound::MotorTest {
                    motor: f32_at(&p, 0) as u8,           // param1: motor instance
                    throttle_type: f32_at(&p, 4) as u8,   // param2: throttle type
                    throttle: f32_at(&p, 8),              // param3: throttle value
                    timeout_s: f32_at(&p, 12),            // param4: timeout (s)
                    count: f32_at(&p, 16) as u8,          // param5: motor count
                })
            }
            MSG_SCKY_ESC_SET => Some(Inbound::EscSet {
                cur_scale: f32_at(&p, 0),
                cur_offset: f32_at(&p, 4),
                refresh_hz: u16::from_le_bytes([p[8], p[9]]),
                protocol: p[10],
                master_enabled: p[11] != 0,
                bidir: p[12] != 0,
                dir_mask: p[13],
                pole_count: p[14],
                mode3d_mask: p[15],
            }),
            MSG_SCKY_ESC_CMD => Some(Inbound::EscCmd {
                target: p[2],
                command: p[3] as u16,
            }),
            _ => None,
        }
    }
}

#[inline]
fn f32_at(p: &[u8], off: usize) -> f32 {
    f32::from_le_bytes([p[off], p[off + 1], p[off + 2], p[off + 3]])
}
