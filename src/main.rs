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
mod baro;
mod compass;
mod crsf;
mod estimator;
mod filters;
mod gps;
mod imu;
mod mavlink;
mod mtf01;
mod nav;

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
    use stm32h7xx_hal::serial::Rx;
    use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
    use stm32h7xx_hal::{i2c, pac, spi};
    use usb_device::prelude::*;

    use crate::ahrs::Attitude;
    use crate::baro::{Baro, BaroData};
    use crate::compass::{Compass, MagData};
    use crate::crsf::{CrsfParser, RcChannels};
    use crate::estimator::{Estimator, Rotation};
    use crate::filters::ImuLpf;
    use crate::gps::{GpsData, NmeaParser};
    use crate::imu::{Health, Imu, ImuOut};
    use crate::mavlink::{Encoder, MAV_SYS_STATUS_SENSOR_3D_ACCEL, MAV_SYS_STATUS_SENSOR_3D_GYRO};
    use crate::mtf01::{Mtf01Data, MspParser};
    use crate::nav::{Nav, NavState};

    // ---- Fusion tuning ----------------------------------------------------
    /// Tasks tick at 1 kHz (Systick monotonic), so the filter sample rate and
    /// fusion step are both 1 ms.
    const SAMPLE_HZ: f32 = 1000.0;
    const DT: f32 = 1.0 / SAMPLE_HZ;
    const DEG2RAD: f32 = core::f32::consts::PI / 180.0;
    const GYRO_CUTOFF_HZ: f32 = 80.0; // gyro low-pass corner
    const ACCEL_CUTOFF_HZ: f32 = 20.0; // accel low-pass corner (gravity is ~DC)
    const AHRS_KP: f32 = 1.0; // accel->attitude correction gain
    const AHRS_KI: f32 = 0.05; // gyro-bias learning gain

    /// GPS NMEA baud. uBlox NEO-M8N powers up emitting NMEA at 9600 (factory
    /// default); if a module has been pre-set to another rate (38400 is common
    /// on Pixhawk-branded units), change this and rebuild.
    const GPS_BAUD: u32 = 9600;
    /// MTF-01 MSP output baud (USART2).
    const MTF01_BAUD: u32 = 115_200;
    /// ExpressLRS / CRSF baud (UART5). ELRS uses 420 kbaud.
    const CRSF_BAUD: u32 = 420_000;

    // ---- Concrete types for the two IMU instances -------------------------
    type Imu1 = Imu<spi::Spi<pac::SPI1, spi::Enabled>, Pin<'A', 4, Output>>;
    type Imu2 = Imu<spi::Spi<pac::SPI4, spi::Enabled>, Pin<'B', 1, Output>>;

    // I2C2 carries both the external compass and the SPL06 baro; one task owns
    // the bus and polls both.
    type I2c2 = i2c::I2c<pac::I2C2>;

    // The USB-C port wires to PA11/PA12 = OTG2_FS, i.e. the HAL's USB2.
    type MyUsbBus = UsbBus<USB2>;

    #[shared]
    struct Shared {
        /// Latest filtered output of each IMU, published by the sampling tasks.
        out1: ImuOut,
        out2: ImuOut,
        /// Fused attitude, published by the estimator task.
        att: Attitude,
        /// Latest GPS solution, published by the USART1 RX interrupt.
        gps: GpsData,
        /// Latest magnetometer reading, published by the I2C task.
        mag: MagData,
        /// Latest barometer reading, published by the I2C task.
        baro: BaroData,
        /// Latest MTF-01 flow + lidar, published by the USART2 RX interrupt.
        flow: Mtf01Data,
        /// Latest RC channels + link, published by the UART5 RX interrupt.
        rc: RcChannels,
        /// Flow/lidar navigation estimate, published by the nav task.
        navs: NavState,
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
        // GPS UART receiver + NMEA decoder, owned by the USART1 interrupt.
        gps_rx: Rx<pac::USART1>,
        gps_parser: NmeaParser,
        // Shared I2C2 bus + the two sensors that hang off it.
        i2c2: I2c2,
        compass: Compass,
        baro: Baro,
        // MTF-01 receiver + MSP decoder, owned by the USART2 interrupt.
        mtf_rx: Rx<pac::USART2>,
        mtf_parser: MspParser,
        // ExpressLRS receiver + CRSF decoder, owned by the UART5 interrupt.
        crsf_rx: Rx<pac::UART5>,
        crsf_parser: CrsfParser,
        // Flow/lidar dead-reckoning integrator.
        nav: Nav,
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
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
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

        // --- USART1 -> GPS  (PA9 TX / PA10 RX) -----------------------------
        // ArduPilot's SERIAL1 = GPS. RX is interrupt-driven: the H7 USART has a
        // 1-byte buffer, so a polled reader would overrun at GPS baud.
        let serial1 = dp
            .USART1
            .serial(
                (
                    gpioa.pa9.into_alternate::<7>(),
                    gpioa.pa10.into_alternate::<7>(),
                ),
                GPS_BAUD.bps(),
                ccdr.peripheral.USART1,
                &ccdr.clocks,
            )
            .unwrap();
        let (_gps_tx, mut gps_rx) = serial1.split();
        gps_rx.listen(); // enable RXNE interrupt

        // --- I2C2 -> external compass  (PB10 SCL / PB11 SDA) ---------------
        // Shared bus with the SPL06 baro (0x76); the magnetometer probes the
        // QMC5883L (0x0D) / HMC5883L (0x1E) addresses.
        let mut i2c2 = dp.I2C2.i2c(
            (
                gpiob.pb10.into_alternate_open_drain(),
                gpiob.pb11.into_alternate_open_drain(),
            ),
            400.kHz(),
            ccdr.peripheral.I2C2,
            &ccdr.clocks,
        );
        // Both the compass and the SPL06 baro live on this bus; probe both now.
        let mut compass = Compass::new();
        compass.init(&mut i2c2);
        let mut baro = Baro::new();
        baro.init(&mut i2c2, &delay_us);

        // --- USART2 -> MTF-01 optical flow + lidar  (PD5 TX / PD6 RX) -------
        let serial2 = dp
            .USART2
            .serial(
                (
                    gpiod.pd5.into_alternate::<7>(),
                    gpiod.pd6.into_alternate::<7>(),
                ),
                MTF01_BAUD.bps(),
                ccdr.peripheral.USART2,
                &ccdr.clocks,
            )
            .unwrap();
        let (_mtf_tx, mut mtf_rx) = serial2.split();
        mtf_rx.listen();

        // --- UART5 -> ExpressLRS receiver (CRSF)  (PB5 RX / PB6 TX) ---------
        let serial5 = dp
            .UART5
            .serial(
                (
                    gpiob.pb6.into_alternate::<14>(),
                    gpiob.pb5.into_alternate::<14>(),
                ),
                CRSF_BAUD.bps(),
                ccdr.peripheral.UART5,
                &ccdr.clocks,
            )
            .unwrap();
        let (_crsf_tx, mut crsf_rx) = serial5.split();
        crsf_rx.listen();

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
        i2c_task::spawn().ok();
        nav_task::spawn().ok();
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
                gps: GpsData::default(),
                mag: MagData::default(),
                baro: BaroData::default(),
                flow: Mtf01Data::default(),
                rc: RcChannels::default(),
                navs: NavState::default(),
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
                gps_rx,
                gps_parser: NmeaParser::new(),
                i2c2,
                compass,
                baro,
                mtf_rx,
                mtf_parser: MspParser::new(),
                crsf_rx,
                crsf_parser: CrsfParser::new(),
                nav: Nav::new(),
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
    #[task(priority = 2, local = [est], shared = [out1, out2, att, mag])]
    async fn estimator_task(cx: estimator_task::Context) {
        let est = cx.local.est;
        let estimator_task::SharedResources {
            mut out1,
            mut out2,
            mut att,
            mut mag,
            ..
        } = cx.shared;

        loop {
            let o1 = out1.lock(|o| *o);
            let o2 = out2.lock(|o| *o);
            let m = mag.lock(|m| *m);
            let mag_field = m.healthy.then_some(m.field);
            let a = est.update(&o1, &o2, mag_field, DT);
            att.lock(|x| *x = a);
            Mono::delay(1.millis()).await;
        }
    }

    /// USART1 RX interrupt — drains every received byte into the NMEA parser and
    /// publishes a fresh [`GpsData`] when a sentence completes. Priority 4 (above
    /// IMU sampling): a UART overrun loses bytes irrecoverably, while IMU polling
    /// tolerates the few-microsecond jitter of this short ISR.
    #[task(binds = USART1, priority = 4, local = [gps_rx, gps_parser], shared = [gps])]
    fn usart1_rx(mut cx: usart1_rx::Context) {
        let parser = cx.local.gps_parser;
        let mut updated = false;
        // Drain the receiver. `read()` self-clears error flags (incl. overrun),
        // so any error just ends this drain; the next RXNE re-enters the ISR.
        while let Ok(byte) = cx.local.gps_rx.read() {
            if parser.push(byte) {
                updated = true;
            }
        }
        if updated {
            let data = parser.data();
            cx.shared.gps.lock(|g| *g = data);
        }
    }

    /// USART2 RX interrupt — MTF-01 MSP frames (optical flow + lidar). Same
    /// interrupt-driven, priority-4 pattern as the GPS UART.
    #[task(binds = USART2, priority = 4, local = [mtf_rx, mtf_parser], shared = [flow])]
    fn usart2_rx(mut cx: usart2_rx::Context) {
        let parser = cx.local.mtf_parser;
        let mut updated = false;
        while let Ok(byte) = cx.local.mtf_rx.read() {
            if parser.push(byte) {
                updated = true;
            }
        }
        if updated {
            let data = parser.data();
            cx.shared.flow.lock(|f| *f = data);
        }
    }

    /// UART5 RX interrupt — CRSF frames from the ExpressLRS receiver. Priority 4:
    /// at 420 kbaud a missed byte corrupts a whole RC frame.
    #[task(binds = UART5, priority = 4, local = [crsf_rx, crsf_parser], shared = [rc])]
    fn uart5_rx(mut cx: uart5_rx::Context) {
        let parser = cx.local.crsf_parser;
        let mut updated = false;
        while let Ok(byte) = cx.local.crsf_rx.read() {
            if parser.push(byte) {
                updated = true;
            }
        }
        if updated {
            let data = parser.data();
            cx.shared.rc.lock(|r| *r = data);
        }
    }

    /// I2C2 sensor poll — magnetometer + SPL06 barometer share the bus, so one
    /// task owns it. Compass every loop (~100 Hz for the AHRS); baro every 5th
    /// loop (~20 Hz, well above its 8 Hz conversion rate). Priority 1; each
    /// blocking I2C transfer is sub-millisecond.
    #[task(priority = 1, local = [i2c2, compass, baro], shared = [mag, baro])]
    async fn i2c_task(cx: i2c_task::Context) {
        let i2c2 = cx.local.i2c2;
        let compass = cx.local.compass;
        let baro = cx.local.baro;
        let i2c_task::SharedResources {
            mut mag,
            baro: mut baro_shared,
            ..
        } = cx.shared;

        let mut n: u32 = 0;
        loop {
            let m = compass.read(i2c2);
            mag.lock(|x| *x = m);
            if n % 5 == 0 {
                let b = baro.read(i2c2);
                baro_shared.lock(|x| *x = b);
            }
            n = n.wrapping_add(1);
            Mono::delay(10.millis()).await;
        }
    }

    /// Flow/lidar navigation — 50 Hz dead-reckoning from MTF-01 + attitude.
    #[task(priority = 1, local = [nav], shared = [flow, att, navs])]
    async fn nav_task(cx: nav_task::Context) {
        let nav = cx.local.nav;
        let nav_task::SharedResources {
            mut flow,
            mut att,
            mut navs,
            ..
        } = cx.shared;
        const NAV_DT: f32 = 0.02; // 50 Hz
        loop {
            let f = flow.lock(|f| *f);
            let a = att.lock(|a| *a);
            nav.update(&f, &a, NAV_DT);
            let s = nav.state();
            navs.lock(|n| *n = s);
            Mono::delay(20.millis()).await;
        }
    }

    /// Owns the whole USB stack: polls it at ~1 kHz (keeps enumeration alive and
    /// flushes the IN endpoint) and streams MAVLink 2 telemetry. Lowest
    /// priority, so it can never delay the IMU sampling tasks.
    #[task(priority = 1, local = [usb_dev, serial, mavlink], shared = [out1, out2, gps, mag, att, flow, rc, navs, baro])]
    async fn usb_task(cx: usb_task::Context) {
        let usb_dev = cx.local.usb_dev;
        let serial = cx.local.serial;
        let mavlink = cx.local.mavlink;
        let usb_task::SharedResources {
            mut out1,
            mut out2,
            mut gps,
            mut mag,
            mut att,
            mut flow,
            mut rc,
            mut navs,
            mut baro,
            ..
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
            // burst of back-to-back USB writes. IDs are zero-based. IMU0 carries
            // the magnetometer (one board compass) when it is healthy.
            if tick % 50 == 0 {
                let o = out1.lock(|o| *o);
                if matches!(o.health, Health::Ok(_)) {
                    let m = mag.lock(|m| *m);
                    let mag_field = m.healthy.then_some(m.field);
                    let frame = mavlink.highres_imu(
                        tick as u64 * 1_000,
                        0,
                        Rotation::Roll180.apply(o.accel),
                        Rotation::Roll180.apply(o.gyro),
                        mag_field,
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
                        None,
                    );
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            }

            // Fused attitude (incl. mag-aided absolute yaw) at 25 Hz.
            if tick % 40 == 30 {
                let a = att.lock(|x| *x);
                let frame = mavlink.attitude(
                    tick,
                    a.roll * DEG2RAD,
                    a.pitch * DEG2RAD,
                    a.yaw * DEG2RAD,
                    a.rates[0] * DEG2RAD,
                    a.rates[1] * DEG2RAD,
                    a.rates[2] * DEG2RAD,
                );
                pump_write(usb_dev, serial, frame.as_slice());
            }

            // GPS raw + fused global position at 5 Hz.
            if tick % 200 == 40 {
                let g = gps.lock(|g| *g);
                let frame = mavlink.gps_raw_int(
                    tick as u64 * 1_000,
                    g.fix_type,
                    g.lat_e7,
                    g.lon_e7,
                    g.alt_mm,
                    g.eph,
                    g.vel_cms,
                    g.cog_cdeg,
                    g.sats,
                );
                pump_write(usb_dev, serial, frame.as_slice());
            }
            if tick % 200 == 60 {
                let g = gps.lock(|g| *g);
                let yaw_deg = att.lock(|x| x.yaw);
                // Heading 0..360 deg -> centidegrees.
                let hdg = {
                    let mut h = yaw_deg;
                    while h < 0.0 {
                        h += 360.0;
                    }
                    while h >= 360.0 {
                        h -= 360.0;
                    }
                    (h * 100.0) as u16
                };
                // Ground velocity NED from GPS speed + course.
                let (vx, vy) = if g.cog_cdeg != u16::MAX {
                    let cog = (g.cog_cdeg as f32 / 100.0) * DEG2RAD;
                    let v = g.vel_cms as f32;
                    ((v * libm::cosf(cog)) as i16, (v * libm::sinf(cog)) as i16)
                } else {
                    (0, 0)
                };
                // Above-ground height from the lidar (mm) when valid, else 0.
                let n = navs.lock(|n| *n);
                let rel_alt = if n.height_valid {
                    (n.height_m * 1000.0) as i32
                } else {
                    0
                };
                let frame = mavlink.global_position_int(
                    tick,
                    g.lat_e7,
                    g.lon_e7,
                    g.alt_mm,
                    rel_alt,
                    vx,
                    vy,
                    0,
                    hdg,
                );
                pump_write(usb_dev, serial, frame.as_slice());
            }

            // Lidar height as DISTANCE_SENSOR at 10 Hz.
            if tick % 100 == 70 {
                let f = flow.lock(|f| *f);
                if f.dist_valid {
                    let cm = (f.dist_mm / 10).clamp(0, u16::MAX as i32) as u16;
                    let frame = mavlink.distance_sensor(tick, 2, 800, cm);
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            }

            // Optical flow at 20 Hz.
            if tick % 50 == 35 {
                let f = flow.lock(|f| *f);
                let n = navs.lock(|n| *n);
                if f.flow_valid {
                    let frame = mavlink.optical_flow(
                        tick as u64 * 1_000,
                        (f.flow_x.clamp(i16::MIN as i32, i16::MAX as i32)) as i16,
                        (f.flow_y.clamp(i16::MIN as i32, i16::MAX as i32)) as i16,
                        n.vx,
                        n.vy,
                        n.height_m,
                        f.flow_quality,
                    );
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            }

            // RC channels + link at 10 Hz.
            if tick % 100 == 80 {
                let r = rc.lock(|r| *r);
                let mut ch = [u16::MAX; 18];
                for i in 0..16 {
                    ch[i] = r.ch_us(i);
                }
                // CRSF uplink link-quality (0..100) in the RSSI byte — the
                // primary ELRS link-health metric for the ground station.
                let frame = mavlink.rc_channels(tick, 16, &ch, r.link_quality);
                pump_write(usb_dev, serial, frame.as_slice());
            }

            // Barometer at 10 Hz.
            if tick % 100 == 90 {
                let b = baro.lock(|b| *b);
                if b.healthy {
                    let frame = mavlink.scaled_pressure(
                        tick,
                        b.pressure_pa / 100.0,            // press_abs: Pa -> hPa
                        0.0,                              // press_diff: none
                        (b.temperature_c * 100.0) as i16, // centidegrees C
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
