//! Minimal motor-output diagnostic for DAKEFPVH743.
//!
//! This bypasses MAVLink, sensors, and RTIC. It brings up USB status text and
//! emits DShot on PA0..PA3 with a sequence designed to actually spin BLHeli_32 /
//! AM32 ESCs and to isolate a misbehaving channel:
//!
//!   1. ARM   — 3 s of zero throttle to all four lines so the ESCs arm. AM32 /
//!              BLHeli_32 ignore throttle until they have seen a stream of zeros,
//!              and slamming throttle without arming is what makes a motor kick
//!              and desync ("spins for an instant in a weird way").
//!   2. SWEEP — each motor M1..M4 in turn: ramp 48 -> ~300 over 2 s, hold 2 s,
//!              stop 1 s. Only one line is driven non-zero at a time, so you can
//!              see per-channel which motor responds (swap an ESC onto the M1 pad
//!              to tell whether a fault follows the ESC/wire or the FC pad).
//!
//! The whole sequence repeats forever. DShot150 is used for the most timing
//! margin on the 64 MHz HSI bit-bang (see `dshot.rs`).

#![no_std]
#![no_main]

#[path = "../dshot.rs"]
mod dshot;

use core::fmt::Write as _;
use cortex_m_rt::entry;
use heapless::String;
use panic_halt as _;
use stm32h7xx_hal::pac;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::rcc::rec::{Spi123ClkSel, UsbClkSel};
use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
use usb_device::prelude::*;

type Bus = UsbBus<USB2>;

const CORE_HZ: u32 = 64_000_000;
const FRAME_HZ: u32 = 500;
const FRAME_PERIOD_CYCLES: u32 = CORE_HZ / FRAME_HZ;

/// DShot throttle band used for the bench sweep. Low values keep a propless motor
/// from desyncing; ramping in avoids the startup kick.
const RAMP_MIN: u16 = 48; // lowest non-special DShot throttle
const RAMP_MAX: u16 = 300; // ~15% of 2047 — enough to spin, gentle on no load

// Phase durations, in frames at FRAME_HZ.
const ARM_FRAMES: u32 = 3 * FRAME_HZ; // 3 s arming (all zero)
const RAMP_FRAMES: u32 = 2 * FRAME_HZ; // 2 s ramp 48 -> RAMP_MAX
const HOLD_FRAMES: u32 = 2 * FRAME_HZ; // 2 s hold at RAMP_MAX
const GAP_FRAMES: u32 = 1 * FRAME_HZ; // 1 s stop between motors
const PER_MOTOR_FRAMES: u32 = RAMP_FRAMES + HOLD_FRAMES + GAP_FRAMES;
const SWEEP_FRAMES: u32 = PER_MOTOR_FRAMES * 4;
const CYCLE_FRAMES: u32 = ARM_FRAMES + SWEEP_FRAMES;

/// Compute the four DShot throttle values for a position within the repeating
/// cycle, plus a 1-based "active motor" id for reporting (0 = arming phase).
fn cycle_values(cycle_pos: u32) -> ([u16; 4], u8) {
    if cycle_pos < ARM_FRAMES {
        return ([0; 4], 0);
    }
    let sweep_pos = cycle_pos - ARM_FRAMES;
    let motor = (sweep_pos / PER_MOTOR_FRAMES) as usize; // 0..3
    let phase_pos = sweep_pos % PER_MOTOR_FRAMES;

    let value = if phase_pos < RAMP_FRAMES {
        // Linear ramp RAMP_MIN -> RAMP_MAX.
        let span = (RAMP_MAX - RAMP_MIN) as u32;
        RAMP_MIN + (span * phase_pos / RAMP_FRAMES) as u16
    } else if phase_pos < RAMP_FRAMES + HOLD_FRAMES {
        RAMP_MAX
    } else {
        0 // gap
    };

    let mut values = [0u16; 4];
    values[motor] = value;
    (values, motor as u8 + 1)
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.freeze(pwrcfg, &dp.SYSCFG);

    let _ = ccdr.clocks.hsi48_ck().unwrap();
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);
    ccdr.peripheral.kernel_spi123_clk_mux(Spi123ClkSel::Per);

    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    dshot::init_pins();

    let usb = USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        gpioa.pa11.into_alternate::<10>(),
        gpioa.pa12.into_alternate::<10>(),
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );

    let bus: &'static usb_device::bus::UsbBusAllocator<Bus> = cortex_m::singleton!(
        : usb_device::bus::UsbBusAllocator<Bus> =
            UsbBus::new(usb, cortex_m::singleton!(: [u32; 1024] = [0; 1024]).unwrap())
    )
    .unwrap();
    let mut serial = usbd_serial::SerialPort::new(bus);
    let mut device = UsbDeviceBuilder::new(bus, UsbVidPid(0x1209, 0x5744))
        .strings(&[StringDescriptors::default()
            .manufacturer("scky")
            .product("DAKEFPVH743 motor diagnostic")
            .serial_number("MOTOR-1")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let mut next_frame = cortex_m::peripheral::DWT::cycle_count();
    let mut frame_count: u32 = 0;
    let mut last_report: u32 = 0;

    loop {
        device.poll(&mut [&mut serial]);

        let cycle_pos = frame_count % CYCLE_FRAMES;
        let (values, active_motor) = cycle_values(cycle_pos);
        let frames = [
            dshot::make_frame(values[0], false),
            dshot::make_frame(values[1], false),
            dshot::make_frame(values[2], false),
            dshot::make_frame(values[3], false),
        ];
        dshot::send_frames(&frames, dshot::Protocol::Dshot150);

        if frame_count.wrapping_sub(last_report) >= FRAME_HZ / 2 {
            last_report = frame_count;
            let mut line: String<120> = String::new();
            let _ = write!(
                line,
                "SCKY MOTOR DIAG DShot150 active_motor={} values=[{},{},{},{}] t_s={} frames={}\r\n",
                active_motor,
                values[0],
                values[1],
                values[2],
                values[3],
                cycle_pos / FRAME_HZ,
                frame_count
            );
            write_all(&mut device, &mut serial, line.as_bytes());
        }

        frame_count = frame_count.wrapping_add(1);
        next_frame = next_frame.wrapping_add(FRAME_PERIOD_CYCLES);
        wait_until(next_frame);
    }
}

#[inline(always)]
fn wait_until(deadline: u32) {
    while (cortex_m::peripheral::DWT::cycle_count().wrapping_sub(deadline) as i32) < 0 {}
}


fn write_all(device: &mut UsbDevice<'static, Bus>, serial: &mut usbd_serial::SerialPort<'static, Bus>, mut data: &[u8]) {
    while !data.is_empty() {
        match serial.write(data) {
            Ok(n) if n > 0 => data = &data[n..],
            _ => {
                device.poll(&mut [serial]);
                break;
            }
        }
    }
}
