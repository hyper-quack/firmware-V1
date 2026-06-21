//! DAKEFPVH743 RTIC/USB diagnostic: scheduler and USB, deliberately no SPI.

#![no_std]
#![no_main]

use panic_halt as _;
use rtic_monotonics::systick::prelude::*;

systick_monotonic!(Mono, 1000);

#[rtic::app(
    device = stm32h7xx_hal::pac,
    peripherals = true,
    dispatchers = [LPTIM1]
)]
mod app {
    use super::*;
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc::rec::UsbClkSel;
    use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
    use usb_device::prelude::*;

    type Bus = UsbBus<USB2>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        device: UsbDevice<'static, Bus>,
        serial: usbd_serial::SerialPort<'static, Bus>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;
        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();
        let rcc = dp.RCC.constrain();
        let mut ccdr = rcc.freeze(pwrcfg, &dp.SYSCFG);

        let _ = ccdr.clocks.hsi48_ck().unwrap();
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);
        Mono::start(cx.core.SYST, 64_000_000);

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
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
        let serial = usbd_serial::SerialPort::new(bus);
        let device = UsbDeviceBuilder::new(bus, UsbVidPid(0x1209, 0x5743))
            .strings(&[StringDescriptors::default()
                .manufacturer("scky")
                .product("DAKEFPVH743 RTIC diagnostic")
                .serial_number("RTIC-1")])
            .unwrap()
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        usb_task::spawn().ok();
        (Shared {}, Local { device, serial })
    }

    #[task(priority = 1, local = [device, serial])]
    async fn usb_task(cx: usb_task::Context) {
        loop {
            if cx.local.device.poll(&mut [cx.local.serial]) {
                let mut input = [0u8; 64];
                let _ = cx.local.serial.read(&mut input);
                let _ = cx.local.serial.write(b"DAKEFPVH743 RTIC OK\r\n");
            }
            Mono::delay(1.millis()).await;
        }
    }
}
