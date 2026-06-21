//! scky-fc — an RTIC flight-controller firmware skeleton for the
//! DAKEFPVH743 (STM32H743xx) board.
//!
//! What this firmware does today (Milestone 1 + RTIC scaffold):
//!   * brings the H7 up on its reliable internal 64 MHz clock,
//!   * configures SPI1 (IMU1) and SPI4 (IMU2) exactly as ArduPilot's hwdef,
//!   * probes both InvenSense v3 IMUs (WHO_AM_I) and configures them,
//!   * samples both IMUs at 1 kHz in independent high-priority RTIC tasks,
//!   * streams status + live accel/gyro over a USB CDC-ACM serial port.
//!
//! See README.md for the full hardware map, build and flashing instructions.
//!
//! Task / priority layout (higher number = higher priority):
//!   prio 3  imu1_task / imu2_task   1 kHz sampling, never blocked by USB
//!   prio 2  otg_fs (hardware IRQ)   USB enumeration / transfers
//!   prio 1  telemetry / heartbeat   ~50 Hz + 1 Hz status over CDC

#![no_std]
#![no_main]

mod imu;

use panic_halt as _;

use rtic_monotonics::systick::prelude::*;

// 1 kHz Systick monotonic drives every periodic task.
systick_monotonic!(Mono, 1000);

#[rtic::app(
    device = stm32h7xx_hal::pac,
    peripherals = true,
    dispatchers = [LPTIM1, LPTIM2]
)]
mod app {
    use super::*;

    use core::fmt::Write as _;
    use embedded_hal::spi::MODE_3;
    use heapless::String;
    use stm32h7xx_hal::gpio::{Output, Pin};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc::rec::{Spi123ClkSel, UsbClkSel};
    use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
    use stm32h7xx_hal::{pac, spi};
    use usb_device::prelude::*;

    use crate::imu::{Health, Imu, Sample};

    // ---- Concrete types for the two IMU instances -------------------------
    type Imu1 = Imu<spi::Spi<pac::SPI1, spi::Enabled>, Pin<'A', 4, Output>>;
    type Imu2 = Imu<spi::Spi<pac::SPI4, spi::Enabled>, Pin<'B', 1, Output>>;

    // The USB-C port wires to PA11/PA12 = OTG2_FS, i.e. the HAL's USB2.
    type MyUsbBus = UsbBus<USB2>;

    #[shared]
    struct Shared {
        /// Latest sample + health for each IMU, published by the sampling tasks.
        s1: Sample,
        s2: Sample,
        h1: Health,
        h2: Health,
    }

    #[local]
    struct Local {
        imu1: Imu1,
        imu2: Imu2,
        // USB is owned exclusively by `usb_task`, which both polls the stack and
        // writes telemetry — so there is no cross-task locking on USB at all.
        usb_dev: UsbDevice<'static, MyUsbBus>,
        serial: usbd_serial::SerialPort<'static, MyUsbBus>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp: pac::Peripherals = cx.device;
        let cp = cx.core;

        // --- Power and clocks ------------------------------------------------
        // Use the internal 64 MHz HSI clock. The DAKEFPVH743 under test stops
        // during the previous HSE/480 MHz VOS0 startup path, before USB can
        // enumerate. HSI keeps boot independent of the external oscillator and
        // is ample for the current 1 kHz polling workload.
        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();
        let rcc = dp.RCC.constrain();
        let mut ccdr = rcc.freeze(pwrcfg, &dp.SYSCFG);

        // USB kernel clock: HSI48 (always enabled by `freeze`). This avoids
        // tying USB to the PLL and works regardless of the 480 MHz tree.
        let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must be running");
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

        // SPI1/2/3 reset to PLL1-Q as their kernel clock source. PLL1-Q is not
        // enabled by this clock configuration, so leaving the reset selection
        // makes `SPI1.spi(...)` panic and halt before USB can enumerate.
        // `per_ck` is sourced from the running HSI clock and is always present.
        ccdr.peripheral
            .kernel_spi123_clk_mux(Spi123ClkSel::Per);

        // --- Systick monotonic @ 1 kHz -------------------------------------
        Mono::start(cp.SYST, 64_000_000);

        // --- GPIO ----------------------------------------------------------
        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

        // --- SPI1 -> IMU1  (PA5 SCK / PA6 MISO / PA7 MOSI, CS PA4) ----------
        let spi1 = dp.SPI1.spi(
            (
                gpioa.pa5.into_alternate::<5>(),
                gpioa.pa6.into_alternate::<5>(),
                gpioa.pa7.into_alternate::<5>(),
            ),
            MODE_3,
            1.MHz(),
            ccdr.peripheral.SPI1,
            &ccdr.clocks,
        );
        let cs1 = gpioa.pa4.into_push_pull_output();

        // --- SPI4 -> IMU2  (PE12 SCK / PE13 MISO / PE14 MOSI, CS PB1) -------
        let spi4 = dp.SPI4.spi(
            (
                gpioe.pe12.into_alternate::<5>(),
                gpioe.pe13.into_alternate::<5>(),
                gpioe.pe14.into_alternate::<5>(),
            ),
            MODE_3,
            1.MHz(),
            ccdr.peripheral.SPI4,
            &ccdr.clocks,
        );
        let cs2 = gpiob.pb1.into_push_pull_output();

        let mut imu1 = Imu::new(spi1, cs1);
        let mut imu2 = Imu::new(spi4, cs2);

        // Blocking busy-wait in us (init only). 64 cycles ≈ 1 us at 64 MHz.
        let delay_us = |us: u32| cortex_m::asm::delay(us.saturating_mul(64));
        let h1 = imu1.init(&delay_us);
        let h2 = imu2.init(&delay_us);

        // --- USB CDC-ACM  (OTG2_FS internal full-speed PHY, PA11/PA12) ------
        let usb = USB2::new(
            dp.OTG2_HS_GLOBAL,
            dp.OTG2_HS_DEVICE,
            dp.OTG2_HS_PWRCLK,
            gpioa.pa11.into_alternate::<10>(),
            gpioa.pa12.into_alternate::<10>(),
            ccdr.peripheral.USB2OTG,
            &ccdr.clocks,
        );

        // Endpoint memory + bus allocator, both as one-shot 'static singletons
        // (no `static mut`, so no `static_mut_refs` warning). The EP-memory
        // singleton is nested inline so its 'static reference is consumed
        // directly, without a binding that would reborrow away the lifetime.
        let bus_ref: &'static usb_device::bus::UsbBusAllocator<MyUsbBus> = cortex_m::singleton!(
            : usb_device::bus::UsbBusAllocator<MyUsbBus> =
                UsbBus::new(usb, cortex_m::singleton!(: [u32; 1024] = [0u32; 1024]).unwrap())
        )
        .unwrap();

        let serial = usbd_serial::SerialPort::new(bus_ref);
        let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x1209, 0x5741))
            .strings(&[StringDescriptors::default()
                .manufacturer("scky")
                .product("scky-fc H743")
                .serial_number("0001")])
            .unwrap()
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        // --- Kick off the periodic tasks -----------------------------------
        imu1_task::spawn().ok();
        imu2_task::spawn().ok();
        usb_task::spawn().ok();

        (
            Shared {
                s1: Sample::default(),
                s2: Sample::default(),
                h1,
                h2,
            },
            Local {
                imu1,
                imu2,
                usb_dev,
                serial,
            },
        )
    }

    /// IMU1 sampling — 1 kHz, highest priority. Never blocked by USB.
    #[task(priority = 3, local = [imu1], shared = [s1, h1])]
    async fn imu1_task(mut cx: imu1_task::Context) {
        loop {
            if let Health::Ok(_) = cx.local.imu1.health {
                let sample = cx.local.imu1.read();
                cx.shared.s1.lock(|s| *s = sample);
            }
            cx.shared.h1.lock(|h| *h = cx.local.imu1.health);
            Mono::delay(1.millis()).await;
        }
    }

    /// IMU2 sampling — 1 kHz, highest priority.
    #[task(priority = 3, local = [imu2], shared = [s2, h2])]
    async fn imu2_task(mut cx: imu2_task::Context) {
        loop {
            if let Health::Ok(_) = cx.local.imu2.health {
                let sample = cx.local.imu2.read();
                cx.shared.s2.lock(|s| *s = sample);
            }
            cx.shared.h2.lock(|h| *h = cx.local.imu2.health);
            Mono::delay(1.millis()).await;
        }
    }

    /// Owns the whole USB stack: polls it at ~1 kHz (keeps enumeration alive and
    /// flushes the IN endpoint) and streams human-readable telemetry. Lowest
    /// priority, so it can never delay the IMU sampling tasks.
    #[task(priority = 1, local = [usb_dev, serial], shared = [s1, s2, h1, h2])]
    async fn usb_task(cx: usb_task::Context) {
        let usb_dev = cx.local.usb_dev;
        let serial = cx.local.serial;
        let usb_task::SharedResources {
            mut s1,
            mut s2,
            mut h1,
            mut h2,
            ..
        } = cx.shared;

        let mut tick: u32 = 0;
        loop {
            // Service the USB stack every tick (~1 ms). Discard any host->device
            // bytes so the OUT endpoint never stalls.
            if usb_dev.poll(&mut [serial]) {
                let mut scratch = [0u8; 64];
                let _ = serial.read(&mut scratch);
            }

            // Live IMU data ~20 Hz.
            if tick % 50 == 0 {
                let (a1, h1v) = (s1.lock(|s| *s), h1.lock(|h| *h));
                let (a2, h2v) = (s2.lock(|s| *s), h2.lock(|h| *h));

                let mut line: String<320> = String::new();
                let _ = write!(line, "{}", FmtImu("IMU1", &h1v, &a1));
                let _ = write!(line, "{}", FmtImu("IMU2", &h2v, &a2));
                pump_write(usb_dev, serial, line.as_bytes());
            }

            // Status banner / heartbeat ~1 Hz.
            if tick % 1000 == 0 {
                let h1v = h1.lock(|h| *h);
                let h2v = h2.lock(|h| *h);
                let mut line: String<160> = String::new();
                let _ = write!(
                    line,
                    "[HB up={}s] IMU1={}({}) IMU2={}({})\r\n",
                    tick / 1000,
                    h1v.name(),
                    health_word(&h1v),
                    h2v.name(),
                    health_word(&h2v),
                );
                pump_write(usb_dev, serial, line.as_bytes());
            }

            tick = tick.wrapping_add(1);
            Mono::delay(1.millis()).await;
        }
    }

    fn health_word(h: &Health) -> &'static str {
        match h {
            Health::Ok(_) => "OK",
            Health::Bad(_) => "FAIL",
            Health::Unknown => "----",
        }
    }

    /// One formatted telemetry line for an IMU: detection status + derived
    /// roll/pitch (from gravity), gyro rates, and raw accel — easy to eyeball
    /// while tilting the board to confirm it reads correctly.
    struct FmtImu<'a>(&'a str, &'a Health, &'a Sample);
    impl core::fmt::Display for FmtImu<'_> {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            let (tag, h, s) = (self.0, self.1, self.2);
            let g = s.gyro_dps();
            let a = s.accel_g();
            write!(
                f,
                "{} {} WHO_AM_I=0x{:02X} | roll={:+6.1} pitch={:+6.1} deg | \
                 gyro r/p/y={:+7.1}/{:+7.1}/{:+7.1} dps | acc={:+5.2}/{:+5.2}/{:+5.2} g\r\n",
                tag,
                health_word(h),
                h.whoami(),
                s.roll_deg(),
                s.pitch_deg(),
                g[0],
                g[1],
                g[2],
                a[0],
                a[1],
                a[2],
            )
        }
    }

    /// Write a whole buffer to the CDC IN endpoint, polling the stack between
    /// packets so multi-packet lines (>64 B) actually get flushed. Bounded spin
    /// so it gives up (drops the rest) if the host isn't reading the port.
    fn pump_write(
        usb_dev: &mut UsbDevice<'static, MyUsbBus>,
        serial: &mut usbd_serial::SerialPort<'static, MyUsbBus>,
        data: &[u8],
    ) {
        let mut off = 0;
        let mut spins = 0u32;
        while off < data.len() {
            match serial.write(&data[off..]) {
                Ok(n) if n > 0 => {
                    off += n;
                    spins = 0;
                }
                _ => {
                    // Endpoint full (or not yet open): let the stack flush it.
                    let _ = usb_dev.poll(&mut [serial]);
                    spins += 1;
                    if spins > 2000 {
                        break; // host not draining — drop the remainder
                    }
                }
            }
        }
    }
}
