//! All-motor ESC bring-up + run test for DAKEFPVH743.
//!
//! A single self-contained binary (no MAVLink, no RTIC, no sensors) that:
//!
//!   1. ARM    — 3 s of zero throttle on all four lines so AM32 / BLHeli_32 arm.
//!   2. CONFIG — reconfigure every ESC to a clean baseline *while stopped*, the
//!               only state in which they accept config commands, and with the
//!               DShot telemetry bit set (without it the ESC ignores the command):
//!               spin-direction NORMAL (20) → 3D OFF (9) → SAVE (12).
//!   3. REARM  — 3 s of zeros: SAVE makes AM32 reboot, so let it come back and
//!               re-arm before driving throttle.
//!   4. RUN    — ramp all four together 48 → ~12 % over 2 s, then hold 12 %
//!               forever. All four lines carry the *same* value every frame, so a
//!               motor that spins differently (or not at all) is a hardware/wiring
//!               difference, not a command difference.
//!
//! DShot150 is used for the most timing margin on the 64 MHz HSI bit-bang
//! (see `dshot.rs`). Progress is printed over USB CDC serial.
//!
//! ⚠️ Bench only, **propellers removed**. A propless motor has little inertia and
//! the ESC's stall protection may still cut it — 12 % is chosen to stay gentle.

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

/// Lowest non-special DShot throttle, and the run target (~12 % of 0..2047).
const DSHOT_MIN: u16 = 48;
const RUN_VALUE: u16 = DSHOT_MIN + (1999u16 * 12) / 100; // 12 % throttle ≈ 287

/// DShot special commands used by the CONFIG phase. Sent with the telemetry bit.
const CMD_SPIN_NORMAL: u16 = 20;
const CMD_3D_OFF: u16 = 9;
const CMD_SAVE: u16 = 12;

// Phase boundaries, in frames at FRAME_HZ.
const ARM_FRAMES: u32 = 3 * FRAME_HZ; // 3 s arming (all zero)
const CMD_FRAMES: u32 = FRAME_HZ / 5; // 100 ms (~50 frames) per config command
const ARM_END: u32 = ARM_FRAMES;
const CFG_DIR_END: u32 = ARM_END + CMD_FRAMES;
const CFG_3D_END: u32 = CFG_DIR_END + CMD_FRAMES;
const CFG_SAVE_END: u32 = CFG_3D_END + CMD_FRAMES;
const REARM_END: u32 = CFG_SAVE_END + 3 * FRAME_HZ; // 3 s for the post-save reboot
const RAMP_FRAMES: u32 = 2 * FRAME_HZ; // 2 s ramp 48 -> RUN_VALUE
const RAMP_END: u32 = REARM_END + RAMP_FRAMES; // after this: hold RUN_VALUE forever

/// One DShot frame value for all four motors plus a short phase label, for the
/// position `frame` within the (non-repeating) startup sequence. `telem` is the
/// telemetry/command bit — true only for special commands.
fn frame_plan(frame: u32) -> (u16, bool, &'static str) {
    if frame < ARM_END {
        (0, false, "ARM")
    } else if frame < CFG_DIR_END {
        (CMD_SPIN_NORMAL, true, "CFG dir-normal")
    } else if frame < CFG_3D_END {
        (CMD_3D_OFF, true, "CFG 3D-off")
    } else if frame < CFG_SAVE_END {
        (CMD_SAVE, true, "CFG save")
    } else if frame < REARM_END {
        (0, false, "REARM")
    } else if frame < RAMP_END {
        // Linear ramp DSHOT_MIN -> RUN_VALUE over RAMP_FRAMES.
        let span = (RUN_VALUE - DSHOT_MIN) as u32;
        let pos = frame - REARM_END;
        (DSHOT_MIN + (span * pos / RAMP_FRAMES) as u16, false, "RUN ramp")
    } else {
        (RUN_VALUE, false, "RUN hold 12%")
    }
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
    let mut device = UsbDeviceBuilder::new(bus, UsbVidPid(0x1209, 0x5745))
        .strings(&[StringDescriptors::default()
            .manufacturer("scky")
            .product("DAKEFPVH743 ESC all-motor 12% test")
            .serial_number("ALL-12")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let mut next_frame = cortex_m::peripheral::DWT::cycle_count();
    let mut frame_count: u32 = 0;
    let mut last_report: u32 = 0;

    loop {
        device.poll(&mut [&mut serial]);

        // The startup sequence (ARM/CONFIG/REARM/ramp) runs once; once past
        // RAMP_END the plan holds 12 % forever, so clamp the counter there to keep
        // the steady-state and avoid the wrap re-triggering ARM/SAVE.
        let plan_frame = frame_count.min(RAMP_END);
        let (value, telem, phase) = frame_plan(plan_frame);
        let f = dshot::make_frame(value, telem);
        let frames = [f; 4]; // all four motors identical
        dshot::send_frames(&frames, dshot::Protocol::Dshot150);

        if frame_count.wrapping_sub(last_report) >= FRAME_HZ / 2 {
            last_report = frame_count;
            let mut line: String<120> = String::new();
            let _ = write!(
                line,
                "SCKY ESC ALLTEST DShot150 phase=\"{}\" value={} telem={} t_s={} frames={}\r\n",
                phase,
                value,
                telem as u8,
                plan_frame / FRAME_HZ,
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
