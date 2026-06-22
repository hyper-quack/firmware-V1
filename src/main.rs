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
//!   prio 3  imu1_task / imu2_task   1 kHz sampling + per-IMU low-pass filtering
//!   prio 2  estimator              1 kHz sensor fusion (Mahony attitude filter)
//!   prio 1  usb_task               USB poll + 20 Hz/IMU MAVLink / 1 Hz status
//!
//! The sensor-fusion pipeline (filters -> dual-IMU combine -> attitude filter)
//! is documented in `docs/sensor-fusion.md`.

#![no_std]
#![no_main]

mod ahrs;
mod estimator;
mod filters;
mod imu;
mod mavlink;

use panic_halt as _;

use rtic_monotonics::systick::prelude::*;

// 1 kHz Systick monotonic drives every periodic task.
systick_monotonic!(Mono, 1000);

#[rtic::app(
    device = stm32h7xx_hal::pac,
    peripherals = true,
    dispatchers = [LPTIM1, LPTIM2, LPTIM3]
)]
mod app {
    use super::*;

    use embedded_hal::spi::MODE_3;
    use stm32h7xx_hal::gpio::{Output, Pin};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc::rec::{Spi123ClkSel, UsbClkSel};
    use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
    use stm32h7xx_hal::{pac, spi};
    use usb_device::prelude::*;

    use crate::ahrs::Attitude;
    use crate::estimator::{Estimator, Rotation};
    use crate::filters::ImuLpf;
    use crate::imu::{Health, Imu, ImuOut};
    use crate::mavlink::{Encoder, MAV_SYS_STATUS_SENSOR_3D_ACCEL, MAV_SYS_STATUS_SENSOR_3D_GYRO};

    // ---- Fusion tuning ----------------------------------------------------
    /// Tasks tick at 1 kHz (Systick monotonic), so the filter sample rate and
    /// fusion step are both 1 ms.
    const SAMPLE_HZ: f32 = 1000.0;
    const DT: f32 = 1.0 / SAMPLE_HZ;
    const GYRO_CUTOFF_HZ: f32 = 80.0; // gyro low-pass corner
    const ACCEL_CUTOFF_HZ: f32 = 20.0; // accel low-pass corner (gravity is ~DC)
    const AHRS_KP: f32 = 1.0; // accel->attitude correction gain
    const AHRS_KI: f32 = 0.05; // gyro-bias learning gain

    // ---- Concrete types for the two IMU instances -------------------------
    type Imu1 = Imu<spi::Spi<pac::SPI1, spi::Enabled>, Pin<'A', 4, Output>>;
    type Imu2 = Imu<spi::Spi<pac::SPI4, spi::Enabled>, Pin<'B', 1, Output>>;

    // The USB-C port wires to PA11/PA12 = OTG2_FS, i.e. the HAL's USB2.
    type MyUsbBus = UsbBus<USB2>;

    #[shared]
    struct Shared {
        /// Latest filtered output of each IMU, published by the sampling tasks.
        out1: ImuOut,
        out2: ImuOut,
        /// Fused attitude, published by the estimator task.
        att: Attitude,
    }

    #[local]
    struct Local {
        imu1: Imu1,
        lpf1: ImuLpf,
        imu2: Imu2,
        lpf2: ImuLpf,
        est: Estimator,
        // USB is owned exclusively by `usb_task`, which both polls the stack and
        // writes telemetry — so there is no cross-task locking on USB at all.
        usb_dev: UsbDevice<'static, MyUsbBus>,
        serial: usbd_serial::SerialPort<'static, MyUsbBus>,
        mavlink: Encoder,
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
        ccdr.peripheral.kernel_spi123_clk_mux(Spi123ClkSel::Per);

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

        // --- Build the fusion pipeline -------------------------------------
        let lpf1 = ImuLpf::new(SAMPLE_HZ, GYRO_CUTOFF_HZ, ACCEL_CUTOFF_HZ);
        let lpf2 = ImuLpf::new(SAMPLE_HZ, GYRO_CUTOFF_HZ, ACCEL_CUTOFF_HZ);
        let est = Estimator::new(AHRS_KP, AHRS_KI, Rotation::Roll180, Rotation::Pitch180);

        // --- Kick off the periodic tasks -----------------------------------
        imu1_task::spawn().ok();
        imu2_task::spawn().ok();
        estimator_task::spawn().ok();
        usb_task::spawn().ok();

        (
            Shared {
                out1: ImuOut {
                    health: h1,
                    ..Default::default()
                },
                out2: ImuOut {
                    health: h2,
                    ..Default::default()
                },
                att: Attitude::default(),
            },
            Local {
                imu1,
                lpf1,
                imu2,
                lpf2,
                est,
                usb_dev,
                serial,
                mavlink: Encoder::new(),
            },
        )
    }

    /// IMU1 sampling + low-pass filtering — 1 kHz, highest priority.
    #[task(priority = 3, local = [imu1, lpf1], shared = [out1])]
    async fn imu1_task(mut cx: imu1_task::Context) {
        loop {
            let health = cx.local.imu1.health;
            let out = if let Health::Ok(_) = health {
                let s = cx.local.imu1.read();
                let (gyro, accel) = cx.local.lpf1.apply(s.gyro_dps(), s.accel_g());
                ImuOut {
                    gyro,
                    accel,
                    health,
                }
            } else {
                ImuOut {
                    health,
                    ..Default::default()
                }
            };
            cx.shared.out1.lock(|o| *o = out);
            Mono::delay(1.millis()).await;
        }
    }

    /// IMU2 sampling + low-pass filtering — 1 kHz, highest priority.
    #[task(priority = 3, local = [imu2, lpf2], shared = [out2])]
    async fn imu2_task(mut cx: imu2_task::Context) {
        loop {
            let health = cx.local.imu2.health;
            let out = if let Health::Ok(_) = health {
                let s = cx.local.imu2.read();
                let (gyro, accel) = cx.local.lpf2.apply(s.gyro_dps(), s.accel_g());
                ImuOut {
                    gyro,
                    accel,
                    health,
                }
            } else {
                ImuOut {
                    health,
                    ..Default::default()
                }
            };
            cx.shared.out2.lock(|o| *o = out);
            Mono::delay(1.millis()).await;
        }
    }

    /// Sensor fusion — 1 kHz. Combines both filtered IMUs and runs the Mahony
    /// attitude filter. Priority 2: above USB, below raw sampling.
    #[task(priority = 2, local = [est], shared = [out1, out2, att])]
    async fn estimator_task(cx: estimator_task::Context) {
        let est = cx.local.est;
        let estimator_task::SharedResources {
            mut out1,
            mut out2,
            mut att,
            ..
        } = cx.shared;

        loop {
            let o1 = out1.lock(|o| *o);
            let o2 = out2.lock(|o| *o);
            let a = est.update(&o1, &o2, DT);
            att.lock(|x| *x = a);
            Mono::delay(1.millis()).await;
        }
    }

    /// Owns the whole USB stack: polls it at ~1 kHz (keeps enumeration alive and
    /// flushes the IN endpoint) and streams MAVLink 2 telemetry. Lowest
    /// priority, so it can never delay the IMU sampling tasks.
    #[task(priority = 1, local = [usb_dev, serial, mavlink], shared = [out1, out2])]
    async fn usb_task(cx: usb_task::Context) {
        let usb_dev = cx.local.usb_dev;
        let serial = cx.local.serial;
        let mavlink = cx.local.mavlink;
        let usb_task::SharedResources {
            mut out1, mut out2, ..
        } = cx.shared;

        let mut tick: u32 = 0;
        loop {
            // Service the USB stack every tick (~1 ms). Discard any host->device
            // bytes so the OUT endpoint never stalls.
            usb_dev.poll(&mut [serial]);
            {
                let mut scratch = [0u8; 64];
                let _ = serial.read(&mut scratch);
            }

            // Only write when fully configured; timekeeping continues while the
            // host is absent so MAVLink timestamps remain time-since-boot.
            if usb_dev.state() != usb_device::device::UsbDeviceState::Configured {
                tick = tick.wrapping_add(1);
                Mono::delay(1.millis()).await;
                continue;
            }

            // Two independent 20 Hz HIGHRES_IMU streams, staggered to avoid a
            // burst of back-to-back USB writes. IDs are zero-based.
            if tick % 50 == 0 {
                let o = out1.lock(|o| *o);
                if matches!(o.health, Health::Ok(_)) {
                    let frame = mavlink.highres_imu(
                        tick as u64 * 1_000,
                        0,
                        Rotation::Roll180.apply(o.accel),
                        Rotation::Roll180.apply(o.gyro),
                    );
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            }
            if tick % 50 == 25 {
                let o = out2.lock(|o| *o);
                if matches!(o.health, Health::Ok(_)) {
                    let frame = mavlink.highres_imu(
                        tick as u64 * 1_000,
                        1,
                        Rotation::Pitch180.apply(o.accel),
                        Rotation::Pitch180.apply(o.gyro),
                    );
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            }

            // Spread 1 Hz status packets across the USB frame.
            if tick % 1000 == 5 {
                let frame = mavlink.heartbeat();
                pump_write(usb_dev, serial, frame.as_slice());
            }
            if tick % 1000 == 10 {
                let h1 = out1.lock(|o| o.health);
                let h2 = out2.lock(|o| o.health);
                let any_ok = matches!(h1, Health::Ok(_)) || matches!(h2, Health::Ok(_));
                let sensors = MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_GYRO;
                let frame = mavlink.sys_status(sensors, if any_ok { sensors } else { 0 });
                pump_write(usb_dev, serial, frame.as_slice());
            }
            if tick % 1000 == 15 {
                let h = out1.lock(|o| o.health);
                let ok = matches!(h, Health::Ok(_));
                let frame = mavlink.imu_status(tick, 0, ok, ok, h.whoami());
                pump_write(usb_dev, serial, frame.as_slice());
            }
            if tick % 1000 == 20 {
                let h = out2.lock(|o| o.health);
                let ok = matches!(h, Health::Ok(_));
                let frame = mavlink.imu_status(tick, 1, ok, ok, h.whoami());
                pump_write(usb_dev, serial, frame.as_slice());
            }

            tick = tick.wrapping_add(1);
            Mono::delay(1.millis()).await;
        }
    }

    /// Write a buffer to the CDC IN endpoint in one non-blocking call. Frames
    /// longer than one USB packet (64 B) are written in a tight loop with a
    /// single poll() between packets; if the endpoint is still busy after
    /// draining, the remainder is dropped (a later frame will resynchronize).
    fn pump_write(
        usb_dev: &mut UsbDevice<'static, MyUsbBus>,
        serial: &mut usbd_serial::SerialPort<'static, MyUsbBus>,
        data: &[u8],
    ) {
        let mut off = 0;
        while off < data.len() {
            match serial.write(&data[off..]) {
                Ok(n) if n > 0 => off += n,
                _ => {
                    // Flush the endpoint and give it one more chance.
                    usb_dev.poll(&mut [serial]);
                    match serial.write(&data[off..]) {
                        Ok(n) if n > 0 => off += n,
                        _ => break, // host not draining — drop remainder
                    }
                }
            }
        }
    }
}
