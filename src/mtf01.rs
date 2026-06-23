//! MicoAir MTF-01 optical-flow + lidar driver — MSP v2 sensor protocol.
//!
//! Wired to **USART2** (PD5 TX / PD6 RX) at 115200. The MTF-01 must be set to
//! **MSP output mode** (its configurator offers MSP / MAVLink); in MSP mode it
//! *pushes* two INAV-style MSP v2 sensor messages without being polled:
//!   * `MSP2_SENSOR_RANGEFINDER` (0x1F01) — downward lidar distance
//!   * `MSP2_SENSOR_OPTIC_FLOW`  (0x1F02) — optical-flow motion
//!
//! MSP v2 frame:
//! ```text
//!   '$' 'X' <dir> <flag> <func_lo> <func_hi> <size_lo> <size_hi> <payload…> <crc8>
//! ```
//! CRC is CRC-8/DVB-S2 over `flag .. payload` (everything after `'$' 'X' <dir>`).
//!
//! RX is interrupt-driven like the other UARTs.

use crate::crsf::crc8_dvb_s2;

const MSP2_SENSOR_RANGEFINDER: u16 = 0x1F01;
const MSP2_SENSOR_OPTIC_FLOW: u16 = 0x1F02;
const MAX_PAYLOAD: usize = 32;

/// Latest MTF-01 readings.
#[derive(Clone, Copy, Default)]
pub struct Mtf01Data {
    /// Downward distance, mm. Valid only when `dist_valid`.
    pub dist_mm: i32,
    /// Lidar return quality (0..255).
    pub dist_quality: u8,
    pub dist_valid: bool,
    /// Raw optical-flow motion (sensor units; scaled to rad/s in `nav.rs`).
    pub flow_x: i32,
    pub flow_y: i32,
    /// Optical-flow quality (0..255). Low quality = untrustworthy surface.
    pub flow_quality: u8,
    pub flow_valid: bool,
}

#[derive(Clone, Copy, PartialEq)]
enum State {
    Dollar,
    X,
    Dir,
    Flag,
    FuncLo,
    FuncHi,
    SizeLo,
    SizeHi,
    Payload,
    Crc,
}

/// Byte-fed MSP v2 frame assembler + decoder.
pub struct MspParser {
    state: State,
    func: u16,
    size: usize,
    idx: usize,
    crc: u8,
    flag: u8,
    buf: [u8; MAX_PAYLOAD],
    data: Mtf01Data,
}

impl MspParser {
    pub const fn new() -> Self {
        Self {
            state: State::Dollar,
            func: 0,
            size: 0,
            idx: 0,
            crc: 0,
            flag: 0,
            buf: [0; MAX_PAYLOAD],
            data: Mtf01Data {
                dist_mm: 0,
                dist_quality: 0,
                dist_valid: false,
                flow_x: 0,
                flow_y: 0,
                flow_quality: 0,
                flow_valid: false,
            },
        }
    }

    pub fn data(&self) -> Mtf01Data {
        self.data
    }

    /// Feed one received byte. Returns `true` when a recognised sensor message
    /// was just decoded.
    pub fn push(&mut self, b: u8) -> bool {
        match self.state {
            State::Dollar => {
                if b == b'$' {
                    self.state = State::X;
                }
                false
            }
            State::X => {
                self.state = if b == b'X' { State::Dir } else { State::Dollar };
                false
            }
            State::Dir => {
                // direction char ('<' / '>' / '!'); not CRC'd.
                self.state = State::Flag;
                false
            }
            State::Flag => {
                self.flag = b;
                self.crc = crc8_dvb_s2(0, b);
                self.state = State::FuncLo;
                false
            }
            State::FuncLo => {
                self.func = b as u16;
                self.crc = crc8_dvb_s2(self.crc, b);
                self.state = State::FuncHi;
                false
            }
            State::FuncHi => {
                self.func |= (b as u16) << 8;
                self.crc = crc8_dvb_s2(self.crc, b);
                self.state = State::SizeLo;
                false
            }
            State::SizeLo => {
                self.size = b as usize;
                self.crc = crc8_dvb_s2(self.crc, b);
                self.state = State::SizeHi;
                false
            }
            State::SizeHi => {
                self.size |= (b as usize) << 8;
                self.crc = crc8_dvb_s2(self.crc, b);
                self.idx = 0;
                self.state = if self.size == 0 {
                    State::Crc
                } else if self.size > MAX_PAYLOAD {
                    State::Dollar // oversized — drop
                } else {
                    State::Payload
                };
                false
            }
            State::Payload => {
                self.buf[self.idx] = b;
                self.idx += 1;
                self.crc = crc8_dvb_s2(self.crc, b);
                if self.idx >= self.size {
                    self.state = State::Crc;
                }
                false
            }
            State::Crc => {
                let ok = b == self.crc;
                self.state = State::Dollar;
                if ok {
                    self.decode()
                } else {
                    false
                }
            }
        }
    }

    fn decode(&mut self) -> bool {
        let p = &self.buf[..self.size];
        match self.func {
            MSP2_SENSOR_RANGEFINDER if p.len() >= 5 => {
                self.data.dist_quality = p[0];
                let dist = i32::from_le_bytes([p[1], p[2], p[3], p[4]]);
                self.data.dist_mm = dist;
                // Negative distance = out of range / no return.
                self.data.dist_valid = dist >= 0;
                true
            }
            MSP2_SENSOR_OPTIC_FLOW if p.len() >= 9 => {
                self.data.flow_quality = p[0];
                self.data.flow_x = i32::from_le_bytes([p[1], p[2], p[3], p[4]]);
                self.data.flow_y = i32::from_le_bytes([p[5], p[6], p[7], p[8]]);
                self.data.flow_valid = p[0] > 0;
                true
            }
            _ => false,
        }
    }
}
