//! Bit-banged DShot output for the four motor ESCs.
//!
//! # Why bit-bang and not timer/DMA?
//!
//! `stm32h7xx-hal` 0.16 implements DMA `TargetAddress` for SPI/UART/ADC/DAC/SAI
//! but **not for timers**, so the usual "stream CCR values on the timer update
//! DMA request" technique is not available through the HAL. Rather than program
//! the DMA + timer at the raw-PAC level (hard to get right without hardware, and
//! tied to a specific timer's alternate-function pins), this driver bit-bangs the
//! DShot waveform on plain GPIO. That makes the output **pin-flexible** (any four
//! GPIOs on one port) and keeps the firmware honest about its current scope:
//! bench motor testing / spin-up, not yet closed-loop flight.
//!
//! All four motor pins must live on **one GPIO port** so a single atomic `BSRR`
//! write drives them together with minimal skew. The mapping `M1..M4 = PA0..PA3`
//! (all on GPIOA) is taken from the DAKEFPVH743 hwdef (TIM2 outputs) — see
//! [`MOTOR_BITS`] / [`port`].
//!
//! Frames are emitted with interrupts briefly masked and edges are scheduled
//! against DWT CYCCNT, so a UART/IMU interrupt and `asm::delay` loop overhead
//! cannot stretch one DShot bit and make the ESC reject the packet. DShot150 is
//! the default because its 6.67 µs bit has the most timing margin at the board's
//! 64 MHz HSI core clock.

use stm32h7xx_hal::pac;

/// Core clock the firmware actually runs on (internal HSI — see README §2/§5).
const CORE_HZ: u32 = 64_000_000;

/// GPIO port carrying all four motor signals (GPIOA on this board). Change
/// together with [`MOTOR_BITS`] if your motor pads move to a different port.
///
/// Returns the raw `GPIOA` block. The driver only ever touches `BSRR` (atomic,
/// write-only), so it can run concurrently with the HAL owning other pins on the
/// same port without a data race.
#[inline(always)]
fn port() -> *const pac::gpioa::RegisterBlock {
    pac::GPIOA::ptr()
}

/// Bit position (0..15) of each motor signal within [`port`]'s `BSRR`.
/// Index 0 = M1 … index 3 = M4. `PA0..PA3` per the DAKEFPVH743 hwdef.
pub const MOTOR_BITS: [u8; 4] = [0, 1, 2, 3];

/// DShot bitrate. Higher rates need tighter timing; on a 64 MHz bit-bang,
/// DShot150 has the most margin and is the safe default for bench testing.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Protocol {
    Dshot150,
    Dshot300,
    Dshot600,
}

impl Protocol {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Protocol::Dshot300,
            2 => Protocol::Dshot600,
            _ => Protocol::Dshot150,
        }
    }

    pub fn as_u8(self) -> u8 {
        match self {
            Protocol::Dshot150 => 0,
            Protocol::Dshot300 => 1,
            Protocol::Dshot600 => 2,
        }
    }

    /// Bit period in core clock cycles.
    fn bit_cycles(self) -> u32 {
        let hz = match self {
            Protocol::Dshot150 => 150_000,
            Protocol::Dshot300 => 300_000,
            Protocol::Dshot600 => 600_000,
        };
        CORE_HZ / hz
    }
}

/// Build the 16-bit DShot frame for an 11-bit value (`0` = stop / `1..47` =
/// special command / `48..2047` = throttle). `telem` requests ESC telemetry on
/// the signal wire (bidirectional DShot); leave `false` when telemetry arrives on
/// a dedicated UART. Standard (non-inverted) CRC.
pub fn make_frame(value: u16, telem: bool) -> u16 {
    let v = value & 0x07FF;
    let packet = (v << 1) | telem as u16; // 12 bits: value + telemetry request
    let crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    (packet << 4) | crc
}

/// Configure the four motor pins as push-pull outputs, driven low (idle).
///
/// Done with raw register writes so the pins are never moved into the HAL's typed
/// GPIO ownership — the bit-bang path needs direct `BSRR` access anyway. Safe to
/// call once during init after the GPIO clock is enabled.
pub fn init_pins() {
    // SAFETY: single-threaded init; we only touch this port's MODER/OTYPER/OSPEEDR
    // for the four motor bits, which are not owned by any HAL driver.
    unsafe {
        let p = &*port();
        for &b in MOTOR_BITS.iter() {
            let b = b as u32;
            // MODER: 0b01 = general-purpose output.
            p.moder.modify(|r, w| w.bits((r.bits() & !(0b11 << (b * 2))) | (0b01 << (b * 2))));
            // OTYPER: 0 = push-pull (default, set explicitly).
            p.otyper.modify(|r, w| w.bits(r.bits() & !(1 << b)));
            // OSPEEDR: 0b11 = very-high speed for clean edges.
            p.ospeedr.modify(|r, w| w.bits(r.bits() | (0b11 << (b * 2))));
        }
        // Drive all motor lines low (idle).
        let reset_mask: u32 = MOTOR_BITS.iter().fold(0, |m, &b| m | (1 << (b as u32 + 16)));
        p.bsrr.write(|w| w.bits(reset_mask));
    }
}

/// Emit one DShot frame per motor on all four lines in parallel.
///
/// Each bit: drive all lines high, hold `T0` (0.375·bit) for the common low part,
/// pull low any line whose bit is 0, hold another 0.375·bit so '1' lines stay high
/// to 0.75·bit, then pull all low for the remaining 0.25·bit. Runs with interrupts
/// enabled (see module docs).
pub fn send_frames(frames: &[u16; 4], proto: Protocol) {
    let bit = proto.bit_cycles();
    let t_low = (bit * 3) / 8; // 0.375 · bit  -> high portion common to 0 and 1
    let t_mid = (bit * 3) / 8; // extra 0.375 · bit -> '1' lines stay high to 0.75
    let t_rest = bit - t_low - t_mid; // remaining ~0.25 · bit, all low

    // Precompute, for each of the 16 bit-times (MSB first), the BSRR reset mask of
    // motor lines whose bit is 0 (pulled low early to encode a '0').
    let set_all: u32 = MOTOR_BITS.iter().fold(0, |m, &b| m | (1 << b as u32));
    let reset_all: u32 = set_all << 16;
    let mut zero_resets = [0u32; 16];
    for (bit_idx, shift) in (0..16u16).rev().enumerate() {
        let mut zero_reset: u32 = 0;
        for (i, &b) in MOTOR_BITS.iter().enumerate() {
            if (frames[i] >> shift) & 1 == 0 {
                zero_reset |= 1 << (b as u32 + 16);
            }
        }
        zero_resets[bit_idx] = zero_reset;
    }

    // SAFETY: BSRR is atomic and write-only; concurrent HAL use of other pins on
    // this port is unaffected.
    let p = unsafe { &*port() };

    cortex_m::interrupt::free(|_| {
        let mut start = cortex_m::peripheral::DWT::cycle_count();
        for &zero_reset in zero_resets.iter() {
            unsafe { p.bsrr.write(|w| w.bits(set_all)) }; // all high
            wait_until(start.wrapping_add(t_low));
            unsafe { p.bsrr.write(|w| w.bits(zero_reset)) }; // '0' lines low
            wait_until(start.wrapping_add(t_low + t_mid));
            unsafe { p.bsrr.write(|w| w.bits(reset_all)) }; // all low
            start = start.wrapping_add(t_low + t_mid + t_rest);
            wait_until(start);
        }
    });
}

#[inline(always)]
fn wait_until(deadline: u32) {
    while (cortex_m::peripheral::DWT::cycle_count().wrapping_sub(deadline) as i32) < 0 {}
}
