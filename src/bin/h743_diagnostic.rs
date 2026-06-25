//! Minimal DAKEFPVH743 boot/USB diagnostic.
//!
//! This intentionally avoids the HSE, PLL, RTIC, and sensor initialization so
//! it can distinguish a basic MCU/USB problem from the main firmware startup.

#![no_std]
#![no_main]

use core::fmt::Write as _;
use cortex_m_rt::entry;
use heapless::String;
use panic_halt as _;
use stm32h7xx_hal::pac;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::rcc::rec::UsbClkSel;
use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
use usb_device::prelude::*;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Stay on the internal HSI system clock. This removes the external crystal
    // and high-speed PLL from the startup path under test.
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.freeze(pwrcfg, &dp.SYSCFG);

    let _ = ccdr.clocks.hsi48_ck().unwrap();
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

    // DAKEFPVH743 LED0. It toggles while the main loop is alive.
    let mut led = gpiod.pd10.into_push_pull_output();
    let _ = led.set_high();

    let usb = USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        gpioa.pa11.into_alternate::<10>(),
        gpioa.pa12.into_alternate::<10>(),
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );

    let bus: &'static usb_device::bus::UsbBusAllocator<UsbBus<USB2>> =
        cortex_m::singleton!(
            : usb_device::bus::UsbBusAllocator<UsbBus<USB2>> =
                UsbBus::new(usb, cortex_m::singleton!(: [u32; 1024] = [0; 1024]).unwrap())
        )
        .unwrap();

    let mut serial = usbd_serial::SerialPort::new(bus);
    let mut device = UsbDeviceBuilder::new(bus, UsbVidPid(0x1209, 0x5742))
        .strings(&[StringDescriptors::default()
            .manufacturer("scky")
            .product("DAKEFPVH743 diagnostic")
            .serial_number("DIAG-1")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let mut ms: u32 = 0;
    let mut rx_total: u32 = 0;
    let mut last_rx: usize = 0;
    let mut last0: u8 = 0;
    let mut last1: u8 = 0;

    loop {
        device.poll(&mut [&mut serial]);

        let mut input = [0u8; 64];
        if let Ok(n) = serial.read(&mut input) {
            if n > 0 {
                rx_total = rx_total.wrapping_add(n as u32);
                last_rx = n;
                last0 = input[0];
                last1 = if n > 1 { input[1] } else { 0 };
                write_all(&mut device, &mut serial, b"RX ");
                write_all(&mut device, &mut serial, &input[..n]);
                write_all(&mut device, &mut serial, b"\r\n");
            }
        }

        if ms % 500 == 0 {
            let _ = led.toggle();
            let mut line: String<96> = String::new();
            let _ = write!(
                line,
                "SCKY USB RAW DIAG t={}ms state={:?} rx_total={} last_n={} b0={:02x} b1={:02x}\r\n",
                ms,
                device.state(),
                rx_total,
                last_rx,
                last0,
                last1
            );
            write_all(&mut device, &mut serial, line.as_bytes());
        }

        ms = ms.wrapping_add(1);
        cortex_m::asm::delay(64_000);
    }
}

fn write_all(
    device: &mut UsbDevice<'static, UsbBus<USB2>>,
    serial: &mut usbd_serial::SerialPort<'static, UsbBus<USB2>>,
    mut data: &[u8],
) {
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
