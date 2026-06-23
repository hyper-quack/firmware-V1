//! CRSF (Crossfire) protocol parser — for the ExpressLRS receiver.
//!
//! The HappyModel ES900/EP-series ExpressLRS RX (900 MHz) outputs **CRSF** over
//! UART, wired to **UART5** (PB5 RX / PB6 TX), which the ArduPilot hwdef assigns
//! as `SERIAL5 = RCIN`. ELRS runs CRSF at **420000 baud**.
//!
//! CRSF frame layout:
//! ```text
//!   [addr] [len] [type] [payload …] [crc8]
//!          \_______ len bytes ________/
//! ```
//! `len` counts `type + payload + crc`. The CRC is CRC-8/DVB-S2 (poly 0xD5) over
//! `type + payload`. We decode two frame types:
//!   * `0x16` RC_CHANNELS_PACKED — 16 channels × 11 bits in 22 bytes
//!   * `0x14` LINK_STATISTICS    — uplink RSSI / link quality (for the UI)
//!
//! Like the GPS UART, RX is interrupt-driven: at 420 kbaud a polled reader would
//! overrun the H7's one-byte buffer instantly.

const ADDR_FLIGHT_CONTROLLER: u8 = 0xC8;
const ADDR_BROADCAST: u8 = 0xEE;
const TYPE_RC_CHANNELS: u8 = 0x16;
const TYPE_LINK_STATS: u8 = 0x14;
const MAX_PAYLOAD: usize = 64;

/// Decoded RC + link state.
#[derive(Clone, Copy)]
pub struct RcChannels {
    /// 16 channels, raw CRSF units (≈172 = 988 µs … 1811 = 2012 µs, 992 = centre).
    pub ch: [u16; 16],
    /// Uplink link quality, 0..100 % (from LINK_STATISTICS).
    pub link_quality: u8,
    /// Uplink RSSI in -dBm (e.g. 70 means -70 dBm).
    pub rssi_dbm: u8,
    /// Count of valid RC frames decoded — increments on each RC update; a stalled
    /// counter means link loss (the consumer applies the failsafe timeout).
    pub frames: u32,
}

impl Default for RcChannels {
    fn default() -> Self {
        Self {
            ch: [992; 16], // centre sticks until the link is up
            link_quality: 0,
            rssi_dbm: 0,
            frames: 0,
        }
    }
}

impl RcChannels {
    /// Channel `i` (0-based) converted to a standard 988–2012 µs PWM value.
    pub fn ch_us(&self, i: usize) -> u16 {
        // Betaflight mapping: us = (raw * 1024 / 1639) + 881.
        let raw = self.ch.get(i).copied().unwrap_or(992) as u32;
        ((raw * 1024) / 1639 + 881) as u16
    }
}

#[derive(Clone, Copy, PartialEq)]
enum State {
    Sync,
    Len,
    Data,
}

/// Byte-fed CRSF frame assembler + decoder.
pub struct CrsfParser {
    state: State,
    len: usize,
    idx: usize,
    buf: [u8; MAX_PAYLOAD + 2],
    data: RcChannels,
}

impl CrsfParser {
    pub const fn new() -> Self {
        Self {
            state: State::Sync,
            len: 0,
            idx: 0,
            buf: [0; MAX_PAYLOAD + 2],
            data: RcChannels {
                ch: [992; 16],
                link_quality: 0,
                rssi_dbm: 0,
                frames: 0,
            },
        }
    }

    pub fn data(&self) -> RcChannels {
        self.data
    }

    /// Feed one received byte. Returns `true` when an RC_CHANNELS frame was just
    /// decoded (i.e. fresh stick data is available).
    pub fn push(&mut self, byte: u8) -> bool {
        match self.state {
            State::Sync => {
                if byte == ADDR_FLIGHT_CONTROLLER || byte == ADDR_BROADCAST {
                    self.state = State::Len;
                }
                false
            }
            State::Len => {
                // `len` = type + payload + crc. Reject implausible lengths.
                if (2..=MAX_PAYLOAD + 1).contains(&(byte as usize)) {
                    self.len = byte as usize;
                    self.idx = 0;
                    self.state = State::Data;
                } else {
                    self.state = State::Sync;
                }
                false
            }
            State::Data => {
                self.buf[self.idx] = byte;
                self.idx += 1;
                if self.idx >= self.len {
                    self.state = State::Sync;
                    return self.process();
                }
                false
            }
        }
    }

    fn process(&mut self) -> bool {
        // buf = [type, payload…, crc]; CRC covers type + payload.
        let crc_pos = self.len - 1;
        let mut crc = 0u8;
        for &b in &self.buf[..crc_pos] {
            crc = crc8_dvb_s2(crc, b);
        }
        if crc != self.buf[crc_pos] {
            return false;
        }

        let frame_type = self.buf[0];
        let payload = &self.buf[1..crc_pos];
        match frame_type {
            TYPE_RC_CHANNELS if payload.len() >= 22 => {
                unpack_channels(payload, &mut self.data.ch);
                self.data.frames = self.data.frames.wrapping_add(1);
                true
            }
            TYPE_LINK_STATS if payload.len() >= 10 => {
                // [0]=up_rssi_ant1, [1]=up_rssi_ant2, [2]=up_link_quality, …
                self.data.rssi_dbm = payload[0];
                self.data.link_quality = payload[2];
                false
            }
            _ => false,
        }
    }
}

/// Unpack 16 little-endian 11-bit channels from 22 bytes.
fn unpack_channels(p: &[u8], ch: &mut [u16; 16]) {
    let mut bit = 0usize;
    for c in ch.iter_mut() {
        let byte = bit / 8;
        let shift = bit % 8;
        // Read 11 bits little-endian across up to three bytes.
        let mut val = (p[byte] as u32) >> shift;
        val |= (p[byte + 1] as u32) << (8 - shift);
        if shift > 5 {
            val |= (p[byte + 2] as u32) << (16 - shift);
        }
        *c = (val & 0x07FF) as u16;
        bit += 11;
    }
}

/// CRC-8/DVB-S2, polynomial 0xD5 — used by both CRSF and MSP v2.
pub fn crc8_dvb_s2(crc: u8, byte: u8) -> u8 {
    let mut c = crc ^ byte;
    for _ in 0..8 {
        if c & 0x80 != 0 {
            c = (c << 1) ^ 0xD5;
        } else {
            c <<= 1;
        }
    }
    c
}
