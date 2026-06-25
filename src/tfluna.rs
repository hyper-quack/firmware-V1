//! Benewake TF-Luna single-point lidar — UART parser.
//!
//! Two of these are mounted on the left and right of the airframe for horizontal
//! obstacle / collision avoidance. The TF-Luna's default interface is **UART at
//! 115200**, streaming a fixed 9-byte frame at 100 Hz with no configuration, so
//! one sensor per UART is the simplest wiring:
//!   * left  → USART6 (PC6 TX / PC7 RX)
//!   * right → UART7  (PE7 RX / PE8 TX)
//!
//! Frame: `0x59 0x59 Dist_L Dist_H Amp_L Amp_H Temp_L Temp_H Checksum`
//! where `Checksum = sum(byte[0..8]) & 0xFF`. Distance is in cm; amplitude is the
//! return signal strength used to gate reliability.
//!
//! RX is interrupt-driven like every other UART on this board.

/// Working range, cm (TF-Luna spec 0.2–8 m).
pub const MIN_CM: u16 = 20;
pub const MAX_CM: u16 = 800;
/// Reject returns weaker than this amplitude (Benewake's reliability floor).
const AMP_MIN: u16 = 100;
/// Amplitude saturation marker (target too close / too reflective).
const AMP_SATURATED: u16 = 0xFFFF;

/// Latest TF-Luna reading.
#[derive(Clone, Copy, Default)]
pub struct TfLunaData {
    /// Distance, cm.
    pub distance_cm: u16,
    /// Return signal strength.
    pub amplitude: u16,
    /// True when the reading is in-range and the amplitude is trustworthy.
    pub valid: bool,
    /// Count of valid frames decoded (liveness).
    pub frames: u32,
    /// Total bytes received (including sync/body bytes).  Wraps at 2^32.
    pub rx_bytes: u32,
    /// Frames where the checksum did not match — non-zero means electrical noise
    /// or a baud-rate mismatch.
    pub checksum_errors: u32,
}

#[derive(Clone, Copy, PartialEq)]
enum State {
    Sync1,
    Sync2,
    Body,
}

/// Byte-fed TF-Luna frame assembler + decoder.
pub struct TfLunaParser {
    state: State,
    buf: [u8; 9],
    idx: usize,
    data: TfLunaData,
}

impl TfLunaParser {
    pub const fn new() -> Self {
        Self {
            state: State::Sync1,
            buf: [0; 9],
            idx: 0,
            data: TfLunaData {
                distance_cm: 0,
                amplitude: 0,
                valid: false,
                frames: 0,
                rx_bytes: 0,
                checksum_errors: 0,
            },
        }
    }

    pub fn data(&self) -> TfLunaData {
        self.data
    }

    /// Feed one received byte. Returns `true` when a checksum-valid frame was
    /// just decoded.
    pub fn push(&mut self, byte: u8) -> bool {
        self.data.rx_bytes = self.data.rx_bytes.wrapping_add(1);
        match self.state {
            State::Sync1 => {
                if byte == 0x59 {
                    self.buf[0] = byte;
                    self.state = State::Sync2;
                }
                false
            }
            State::Sync2 => {
                if byte == 0x59 {
                    self.buf[1] = byte;
                    self.idx = 2;
                    self.state = State::Body;
                } else {
                    // Not a header pair; resync (this byte might be a new 0x59).
                    self.state = if byte == 0x59 { State::Sync2 } else { State::Sync1 };
                    if byte == 0x59 {
                        self.buf[0] = byte;
                    }
                }
                false
            }
            State::Body => {
                self.buf[self.idx] = byte;
                self.idx += 1;
                if self.idx == 9 {
                    self.state = State::Sync1;
                    return self.process();
                }
                false
            }
        }
    }

    fn process(&mut self) -> bool {
        let sum: u32 = self.buf[..8].iter().map(|&b| b as u32).sum();
        if (sum & 0xFF) as u8 != self.buf[8] {
            self.data.checksum_errors = self.data.checksum_errors.wrapping_add(1);
            return false;
        }
        let distance = u16::from_le_bytes([self.buf[2], self.buf[3]]);
        let amplitude = u16::from_le_bytes([self.buf[4], self.buf[5]]);

        self.data.distance_cm = distance;
        self.data.amplitude = amplitude;
        self.data.valid = amplitude >= AMP_MIN
            && amplitude != AMP_SATURATED
            && (MIN_CM..=MAX_CM).contains(&distance);
        self.data.frames = self.data.frames.wrapping_add(1);
        true
    }
}
