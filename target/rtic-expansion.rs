#[doc = r" The RTIC application module"] pub mod app
{
    #[doc =
    r" Always include the device crate which contains the vector table"] use
    stm32h7xx_hal :: pac as
    you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml;
    #[doc =
    r" Holds the maximum priority level for use by async HAL drivers."]
    #[no_mangle] static RTIC_ASYNC_MAX_LOGICAL_PRIO : u8 = 1 << stm32h7xx_hal
    :: pac :: NVIC_PRIO_BITS; use super :: * ; use core :: fmt :: Write as _;
    use embedded_hal :: spi :: MODE_3; use heapless :: String; use
    stm32h7xx_hal :: gpio :: { Output, Pin }; use stm32h7xx_hal :: prelude ::
    * ; use stm32h7xx_hal :: rcc :: rec :: { Spi123ClkSel, UsbClkSel }; use
    stm32h7xx_hal :: usb_hs :: { UsbBus, USB2 }; use stm32h7xx_hal ::
    { pac, spi }; use usb_device :: prelude :: * ; use crate :: imu ::
    { Health, Imu, Sample }; type Imu1 = Imu < spi :: Spi < pac :: SPI1, spi
    :: Enabled > , Pin < 'A', 4, Output > > ; type Imu2 = Imu < spi :: Spi <
    pac :: SPI4, spi :: Enabled > , Pin < 'B', 1, Output > > ; type MyUsbBus =
    UsbBus < USB2 > ; fn health_word(h : & Health) -> & 'static str
    {
        match h
        {
            Health :: Ok(_) => "OK", Health :: Bad(_) => "FAIL", Health ::
            Unknown => "----",
        }
    }
    #[doc =
    " One formatted telemetry line for an IMU: detection status + derived"]
    #[doc =
    " roll/pitch (from gravity), gyro rates, and raw accel — easy to eyeball"]
    #[doc = " while tilting the board to confirm it reads correctly."] struct
    FmtImu < 'a > (& 'a str, & 'a Health, & 'a Sample); impl core :: fmt ::
    Display for FmtImu < '_ >
    {
        fn fmt(& self, f : & mut core :: fmt :: Formatter < '_ >) -> core ::
        fmt :: Result
        {
            let (tag, h, s) = (self.0, self.1, self.2); let g = s.gyro_dps();
            let a = s.accel_g(); write!
            (f,
            "{} {} WHO_AM_I=0x{:02X} | roll={:+6.1} pitch={:+6.1} deg | \
                 gyro r/p/y={:+7.1}/{:+7.1}/{:+7.1} dps | acc={:+5.2}/{:+5.2}/{:+5.2} g\r\n",
            tag, health_word(h), h.whoami(), s.roll_deg(), s.pitch_deg(),
            g[0], g[1], g[2], a[0], a[1], a[2],)
        }
    }
    #[doc =
    " Write a whole buffer to the CDC IN endpoint, polling the stack between"]
    #[doc =
    " packets so multi-packet lines (>64 B) actually get flushed. Bounded spin"]
    #[doc =
    " so it gives up (drops the rest) if the host isn\'t reading the port."]
    fn
    pump_write(usb_dev : & mut UsbDevice < 'static, MyUsbBus > , serial : &
    mut usbd_serial :: SerialPort < 'static, MyUsbBus > , data : & [u8],)
    {
        let mut off = 0; let mut spins = 0u32; while off < data.len()
        {
            match serial.write(& data [off ..])
            {
                Ok(n) if n > 0 => { off += n; spins = 0; } _ =>
                {
                    let _ = usb_dev.poll(& mut [serial]); spins += 1; if spins >
                    2000 { break; }
                }
            }
        }
    } #[doc = r" User code end"] #[doc = r"Shared resources"] struct Shared
    {
        #[doc =
        " Latest sample + health for each IMU, published by the sampling tasks."]
        s1 : Sample, s2 : Sample, h1 : Health, h2 : Health,
    } #[doc = r"Local resources"] struct Local
    {
        imu1 : Imu1, imu2 : Imu2, usb_dev : UsbDevice < 'static, MyUsbBus > ,
        serial : usbd_serial :: SerialPort < 'static, MyUsbBus > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct __rtic_internal_init_Context <
    'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > ,
        #[doc = r" The space used to allocate async executors in bytes."] pub
        executors_size : usize, #[doc = r" Core peripherals"] pub core : rtic
        :: export :: Peripherals, #[doc = r" Device peripherals (PAC)"] pub
        device : stm32h7xx_hal :: pac :: Peripherals,
        #[doc = r" Critical section token for init"] pub cs : rtic :: export
        :: CriticalSection < 'a > ,
    } impl < 'a > __rtic_internal_init_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn
        new(core : rtic :: export :: Peripherals, executors_size : usize) ->
        Self
        {
            __rtic_internal_init_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, core :
                core, device : stm32h7xx_hal :: pac :: Peripherals :: steal(),
                cs : rtic :: export :: CriticalSection :: new(),
                executors_size,
            }
        }
    } #[allow(non_snake_case)] #[doc = "Initialization function"] pub mod init
    {
        #[doc(inline)] pub use super :: __rtic_internal_init_Context as
        Context;
    } #[inline(always)] #[allow(non_snake_case)] fn init(cx : init :: Context)
    -> (Shared, Local)
    {
        let dp : pac :: Peripherals = cx.device; let cp = cx.core; let pwr =
        dp.PWR.constrain(); let pwrcfg = pwr.freeze(); let rcc =
        dp.RCC.constrain(); let mut ccdr = rcc.freeze(pwrcfg, & dp.SYSCFG);
        let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must be running");
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel :: Hsi48);
        ccdr.peripheral.kernel_spi123_clk_mux(Spi123ClkSel :: Per); Mono ::
        start(cp.SYST, 64_000_000); let gpioa =
        dp.GPIOA.split(ccdr.peripheral.GPIOA); let gpiob =
        dp.GPIOB.split(ccdr.peripheral.GPIOB); let gpioe =
        dp.GPIOE.split(ccdr.peripheral.GPIOE); let spi1 =
        dp.SPI1.spi((gpioa.pa5.into_alternate :: < 5 > (),
        gpioa.pa6.into_alternate :: < 5 > (), gpioa.pa7.into_alternate :: < 5
        > (),), MODE_3, 1.MHz(), ccdr.peripheral.SPI1, & ccdr.clocks,); let
        cs1 = gpioa.pa4.into_push_pull_output(); let spi4 =
        dp.SPI4.spi((gpioe.pe12.into_alternate :: < 5 > (),
        gpioe.pe13.into_alternate :: < 5 > (), gpioe.pe14.into_alternate :: <
        5 > (),), MODE_3, 1.MHz(), ccdr.peripheral.SPI4, & ccdr.clocks,); let
        cs2 = gpiob.pb1.into_push_pull_output(); let mut imu1 = Imu ::
        new(spi1, cs1); let mut imu2 = Imu :: new(spi4, cs2); let delay_us = |
        us : u32 | cortex_m :: asm :: delay(us.saturating_mul(64)); let h1 =
        imu1.init(& delay_us); let h2 = imu2.init(& delay_us); let usb = USB2
        ::
        new(dp.OTG2_HS_GLOBAL, dp.OTG2_HS_DEVICE, dp.OTG2_HS_PWRCLK,
        gpioa.pa11.into_alternate :: < 10 > (), gpioa.pa12.into_alternate :: <
        10 > (), ccdr.peripheral.USB2OTG, & ccdr.clocks,); let bus_ref : &
        'static usb_device :: bus :: UsbBusAllocator < MyUsbBus > = cortex_m
        :: singleton!
        (: usb_device::bus::UsbBusAllocator<MyUsbBus> =
        UsbBus::new(usb,
        cortex_m::singleton!(: [u32; 1024] =
        [0u32; 1024]).unwrap())).unwrap(); let serial = usbd_serial ::
        SerialPort :: new(bus_ref); let usb_dev = UsbDeviceBuilder ::
        new(bus_ref,
        UsbVidPid(0x1209,
        0x5741)).strings(&
        [StringDescriptors ::
        default().manufacturer("scky").product("scky-fc H743").serial_number("0001")]).unwrap().device_class(usbd_serial
        :: USB_CLASS_CDC).build(); imu1_task :: spawn().ok(); imu2_task ::
        spawn().ok(); usb_task :: spawn().ok();
        (Shared
        { s1 : Sample :: default(), s2 : Sample :: default(), h1, h2, }, Local
        { imu1, imu2, usb_dev, serial, },)
    } impl < 'a > __rtic_internal_imu1_taskLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_imu1_taskLocalResources
            {
                imu1 : & mut *
                (& mut *
                __rtic_internal_local_resource_imu1.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_imu1_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_imu1_taskSharedResources
            {
                s1 : shared_resources :: s1_that_needs_to_be_locked :: new(),
                h1 : shared_resources :: h1_that_needs_to_be_locked :: new(),
                __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_imu2_taskLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_imu2_taskLocalResources
            {
                imu2 : & mut *
                (& mut *
                __rtic_internal_local_resource_imu2.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_imu2_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_imu2_taskSharedResources
            {
                s2 : shared_resources :: s2_that_needs_to_be_locked :: new(),
                h2 : shared_resources :: h2_that_needs_to_be_locked :: new(),
                __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_usb_taskLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usb_taskLocalResources
            {
                usb_dev : & mut *
                (& mut *
                __rtic_internal_local_resource_usb_dev.get_mut()).as_mut_ptr(),
                serial : & mut *
                (& mut *
                __rtic_internal_local_resource_serial.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_usb_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usb_taskSharedResources
            {
                s1 : shared_resources :: s1_that_needs_to_be_locked :: new(),
                s2 : shared_resources :: s2_that_needs_to_be_locked :: new(),
                h1 : shared_resources :: h1_that_needs_to_be_locked :: new(),
                h2 : shared_resources :: h2_that_needs_to_be_locked :: new(),
                __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `imu1_task` has access to"] pub struct
    __rtic_internal_imu1_taskLocalResources < 'a >
    {
        #[allow(missing_docs)] pub imu1 : & 'a mut Imu1, #[doc(hidden)] pub
        __rtic_internal_marker : :: core :: marker :: PhantomData < & 'a () >
        ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `imu1_task` has access to"] pub struct
    __rtic_internal_imu1_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub s1 : shared_resources ::
        s1_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub h1 :
        shared_resources :: h1_that_needs_to_be_locked < 'a > , #[doc(hidden)]
        pub __rtic_internal_marker : core :: marker :: PhantomData < & 'a () >
        ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct
    __rtic_internal_imu1_task_Context < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : imu1_task :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        imu1_task :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_imu1_task_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_imu1_task_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                imu1_task :: LocalResources :: new(), shared : imu1_task ::
                SharedResources :: new(),
            }
        }
    } #[doc = r" Spawns the task directly"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_imu1_task_spawn() -> :: core ::
    result :: Result < (), () >
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(imu1_task, & __rtic_internal_imu1_task_EXEC); if
            exec.try_allocate()
            {
                exec.spawn(imu1_task(unsafe
                { imu1_task :: Context :: new() })); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM2); Ok(())
            } else { Err(()) }
        }
    } #[doc = r" Gives waker to the task"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_imu1_task_waker() -> :: core :: task
    :: Waker
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(imu1_task, & __rtic_internal_imu1_task_EXEC);
            exec.waker(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(imu1_task, & __rtic_internal_imu1_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM2);
            })
        }
    } #[allow(non_snake_case)] #[doc = "Software task"] pub mod imu1_task
    {
        #[doc(inline)] pub use super ::
        __rtic_internal_imu1_taskLocalResources as LocalResources;
        #[doc(inline)] pub use super ::
        __rtic_internal_imu1_taskSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_imu1_task_Context as
        Context; #[doc(inline)] pub use super ::
        __rtic_internal_imu1_task_spawn as spawn; #[doc(inline)] pub use super
        :: __rtic_internal_imu1_task_waker as waker;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `imu2_task` has access to"] pub struct
    __rtic_internal_imu2_taskLocalResources < 'a >
    {
        #[allow(missing_docs)] pub imu2 : & 'a mut Imu2, #[doc(hidden)] pub
        __rtic_internal_marker : :: core :: marker :: PhantomData < & 'a () >
        ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `imu2_task` has access to"] pub struct
    __rtic_internal_imu2_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub s2 : shared_resources ::
        s2_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub h2 :
        shared_resources :: h2_that_needs_to_be_locked < 'a > , #[doc(hidden)]
        pub __rtic_internal_marker : core :: marker :: PhantomData < & 'a () >
        ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct
    __rtic_internal_imu2_task_Context < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : imu2_task :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        imu2_task :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_imu2_task_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_imu2_task_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                imu2_task :: LocalResources :: new(), shared : imu2_task ::
                SharedResources :: new(),
            }
        }
    } #[doc = r" Spawns the task directly"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_imu2_task_spawn() -> :: core ::
    result :: Result < (), () >
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(imu2_task, & __rtic_internal_imu2_task_EXEC); if
            exec.try_allocate()
            {
                exec.spawn(imu2_task(unsafe
                { imu2_task :: Context :: new() })); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM2); Ok(())
            } else { Err(()) }
        }
    } #[doc = r" Gives waker to the task"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_imu2_task_waker() -> :: core :: task
    :: Waker
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(imu2_task, & __rtic_internal_imu2_task_EXEC);
            exec.waker(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(imu2_task, & __rtic_internal_imu2_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM2);
            })
        }
    } #[allow(non_snake_case)] #[doc = "Software task"] pub mod imu2_task
    {
        #[doc(inline)] pub use super ::
        __rtic_internal_imu2_taskLocalResources as LocalResources;
        #[doc(inline)] pub use super ::
        __rtic_internal_imu2_taskSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_imu2_task_Context as
        Context; #[doc(inline)] pub use super ::
        __rtic_internal_imu2_task_spawn as spawn; #[doc(inline)] pub use super
        :: __rtic_internal_imu2_task_waker as waker;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `usb_task` has access to"] pub struct
    __rtic_internal_usb_taskLocalResources < 'a >
    {
        #[allow(missing_docs)] pub usb_dev : & 'a mut UsbDevice < 'static,
        MyUsbBus > , #[allow(missing_docs)] pub serial : & 'a mut usbd_serial
        :: SerialPort < 'static, MyUsbBus > , #[doc(hidden)] pub
        __rtic_internal_marker : :: core :: marker :: PhantomData < & 'a () >
        ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `usb_task` has access to"] pub struct
    __rtic_internal_usb_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub s1 : shared_resources ::
        s1_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub s2 :
        shared_resources :: s2_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub h1 : shared_resources ::
        h1_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub h2 :
        shared_resources :: h2_that_needs_to_be_locked < 'a > , #[doc(hidden)]
        pub __rtic_internal_marker : core :: marker :: PhantomData < & 'a () >
        ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct __rtic_internal_usb_task_Context
    < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : usb_task :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        usb_task :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_usb_task_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usb_task_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                usb_task :: LocalResources :: new(), shared : usb_task ::
                SharedResources :: new(),
            }
        }
    } #[doc = r" Spawns the task directly"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_usb_task_spawn() -> :: core ::
    result :: Result < (), () >
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(usb_task, & __rtic_internal_usb_task_EXEC); if
            exec.try_allocate()
            {
                exec.spawn(usb_task(unsafe { usb_task :: Context :: new() }));
                rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1); Ok(())
            } else { Err(()) }
        }
    } #[doc = r" Gives waker to the task"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_usb_task_waker() -> :: core :: task
    :: Waker
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(usb_task, & __rtic_internal_usb_task_EXEC);
            exec.waker(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(usb_task, & __rtic_internal_usb_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1);
            })
        }
    } #[allow(non_snake_case)] #[doc = "Software task"] pub mod usb_task
    {
        #[doc(inline)] pub use super :: __rtic_internal_usb_taskLocalResources
        as LocalResources; #[doc(inline)] pub use super ::
        __rtic_internal_usb_taskSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_usb_task_Context as
        Context; #[doc(inline)] pub use super ::
        __rtic_internal_usb_task_spawn as spawn; #[doc(inline)] pub use super
        :: __rtic_internal_usb_task_waker as waker;
    } #[allow(non_snake_case)] async fn imu1_task < 'a >
    (mut cx : imu1_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; loop
        {
            if let Health :: Ok(_) = cx.local.imu1.health
            {
                let sample = cx.local.imu1.read();
                cx.shared.s1.lock(| s | * s = sample);
            } cx.shared.h1.lock(| h | * h = cx.local.imu1.health); Mono ::
            delay(1.millis()).await;
        }
    } #[allow(non_snake_case)] async fn imu2_task < 'a >
    (mut cx : imu2_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; loop
        {
            if let Health :: Ok(_) = cx.local.imu2.health
            {
                let sample = cx.local.imu2.read();
                cx.shared.s2.lock(| s | * s = sample);
            } cx.shared.h2.lock(| h | * h = cx.local.imu2.health); Mono ::
            delay(1.millis()).await;
        }
    } #[allow(non_snake_case)] async fn usb_task < 'a >
    (cx : usb_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let
        usb_dev = cx.local.usb_dev; let serial = cx.local.serial; let usb_task
        :: SharedResources { mut s1, mut s2, mut h1, mut h2, .. } = cx.shared;
        let mut tick : u32 = 0; loop
        {
            if usb_dev.poll(& mut [serial])
            {
                let mut scratch = [0u8; 64]; let _ =
                serial.read(& mut scratch);
            } if tick % 50 == 0
            {
                let (a1, h1v) = (s1.lock(| s | * s), h1.lock(| h | * h)); let
                (a2, h2v) = (s2.lock(| s | * s), h2.lock(| h | * h)); let mut
                line : String < 320 > = String :: new(); let _ = write!
                (line, "{}", FmtImu("IMU1", &h1v, &a1)); let _ = write!
                (line, "{}", FmtImu("IMU2", &h2v, &a2));
                pump_write(usb_dev, serial, line.as_bytes());
            } if tick % 1000 == 0
            {
                let h1v = h1.lock(| h | * h); let h2v = h2.lock(| h | * h);
                let mut line : String < 160 > = String :: new(); let _ =
                write!
                (line, "[HB up={}s] IMU1={}({}) IMU2={}({})\r\n", tick / 1000,
                h1v.name(), health_word(&h1v), h2v.name(),
                health_word(&h2v),);
                pump_write(usb_dev, serial, line.as_bytes());
            } tick = tick.wrapping_add(1); Mono :: delay(1.millis()).await;
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic0"] static
    __rtic_internal_shared_resource_s1 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Sample >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: s1_that_needs_to_be_locked < 'a >
    {
        type T = Sample; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut Sample) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 3u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_s1.get_mut() as * mut _,
                CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic1"] static
    __rtic_internal_shared_resource_s2 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Sample >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: s2_that_needs_to_be_locked < 'a >
    {
        type T = Sample; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut Sample) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 3u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_s2.get_mut() as * mut _,
                CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic2"] static
    __rtic_internal_shared_resource_h1 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Health >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: h1_that_needs_to_be_locked < 'a >
    {
        type T = Health; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut Health) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 3u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_h1.get_mut() as * mut _,
                CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic3"] static
    __rtic_internal_shared_resource_h2 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Health >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: h2_that_needs_to_be_locked < 'a >
    {
        type T = Health; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut Health) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 3u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_h2.get_mut() as * mut _,
                CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } mod shared_resources
    {
        #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        s1_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > s1_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                s1_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        s2_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > s2_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                s2_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        h1_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > h1_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                h1_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        h2_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > h2_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                h2_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic4"] static
    __rtic_internal_local_resource_imu1 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Imu1 >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic5"] static
    __rtic_internal_local_resource_imu2 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Imu2 >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic6"] static
    __rtic_internal_local_resource_usb_dev : rtic :: RacyCell < core :: mem ::
    MaybeUninit < UsbDevice < 'static, MyUsbBus > >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic7"] static
    __rtic_internal_local_resource_serial : rtic :: RacyCell < core :: mem ::
    MaybeUninit < usbd_serial :: SerialPort < 'static, MyUsbBus > >> = rtic ::
    RacyCell :: new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_upper_case_globals)] static __rtic_internal_imu1_task_EXEC :
    rtic :: export :: executor :: AsyncTaskExecutorPtr = rtic :: export ::
    executor :: AsyncTaskExecutorPtr :: new();
    #[allow(non_upper_case_globals)] static __rtic_internal_imu2_task_EXEC :
    rtic :: export :: executor :: AsyncTaskExecutorPtr = rtic :: export ::
    executor :: AsyncTaskExecutorPtr :: new();
    #[allow(non_upper_case_globals)] static __rtic_internal_usb_task_EXEC :
    rtic :: export :: executor :: AsyncTaskExecutorPtr = rtic :: export ::
    executor :: AsyncTaskExecutorPtr :: new(); #[allow(non_snake_case)]
    #[doc = "Interrupt handler to dispatch async tasks at priority 1"]
    #[no_mangle] unsafe fn LPTIM1()
    {
        #[doc = r" The priority of this interrupt handler"] const PRIORITY :
        u8 = 1u8; rtic :: export ::
        run(PRIORITY, ||
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(usb_task, & __rtic_internal_usb_task_EXEC);
            exec.poll(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(usb_task, & __rtic_internal_usb_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1);
            });
        });
    } #[allow(non_snake_case)]
    #[doc = "Interrupt handler to dispatch async tasks at priority 3"]
    #[no_mangle] unsafe fn LPTIM2()
    {
        #[doc = r" The priority of this interrupt handler"] const PRIORITY :
        u8 = 3u8; rtic :: export ::
        run(PRIORITY, ||
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(imu1_task, & __rtic_internal_imu1_task_EXEC);
            exec.poll(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(imu1_task, & __rtic_internal_imu1_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM2);
            }); let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(imu2_task, & __rtic_internal_imu2_task_EXEC);
            exec.poll(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(imu2_task, & __rtic_internal_imu2_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM2);
            });
        });
    } #[doc(hidden)] #[no_mangle] unsafe extern "C" fn main() -> !
    {
        rtic :: export :: assert_send :: < Sample > (); rtic :: export ::
        assert_send :: < Health > (); rtic :: export :: interrupt ::
        disable(); let mut core : rtic :: export :: Peripherals = rtic ::
        export :: Peripherals :: steal().into(); let _ =
        you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml ::
        interrupt :: LPTIM1; let _ =
        you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml ::
        interrupt :: LPTIM2; const _ : () = if
        (1 << stm32h7xx_hal :: pac :: NVIC_PRIO_BITS) < 1u8 as usize
        {
            :: core :: panic!
            ("Maximum priority used by interrupt vector 'LPTIM1' is more than supported by hardware");
        };
        core.NVIC.set_priority(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: LPTIM1, rtic :: export ::
        cortex_logical2hw(1u8, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS),); rtic
        :: export :: NVIC ::
        unmask(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: LPTIM1); const _ : () = if
        (1 << stm32h7xx_hal :: pac :: NVIC_PRIO_BITS) < 3u8 as usize
        {
            :: core :: panic!
            ("Maximum priority used by interrupt vector 'LPTIM2' is more than supported by hardware");
        };
        core.NVIC.set_priority(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: LPTIM2, rtic :: export ::
        cortex_logical2hw(3u8, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS),); rtic
        :: export :: NVIC ::
        unmask(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: LPTIM2); #[inline(never)] fn __rtic_init_resources < F
        > (f : F) where F : FnOnce() { f(); } let mut executors_size = 0; let
        executor = :: core :: mem :: ManuallyDrop ::
        new(rtic :: export :: executor :: AsyncTaskExecutor ::
        new_1_args(imu1_task)); executors_size += :: core :: mem ::
        size_of_val(& executor);
        __rtic_internal_imu1_task_EXEC.set_in_main(& executor); let executor =
        :: core :: mem :: ManuallyDrop ::
        new(rtic :: export :: executor :: AsyncTaskExecutor ::
        new_1_args(imu2_task)); executors_size += :: core :: mem ::
        size_of_val(& executor);
        __rtic_internal_imu2_task_EXEC.set_in_main(& executor); let executor =
        :: core :: mem :: ManuallyDrop ::
        new(rtic :: export :: executor :: AsyncTaskExecutor ::
        new_1_args(usb_task)); executors_size += :: core :: mem ::
        size_of_val(& executor);
        __rtic_internal_usb_task_EXEC.set_in_main(& executor); extern "C"
        { pub static _stack_start : u32; pub static __ebss : u32; } let
        stack_start = & _stack_start as * const _ as u32; let ebss = & __ebss
        as * const _ as u32; if stack_start > ebss
        {
            if rtic :: export :: msp :: read() <= ebss
            { panic! ("Stack overflow after allocating executors"); }
        }
        __rtic_init_resources(||
        {
            let (shared_resources, local_resources) =
            init(init :: Context :: new(core.into(), executors_size));
            __rtic_internal_shared_resource_s1.get_mut().write(core :: mem ::
            MaybeUninit :: new(shared_resources.s1));
            __rtic_internal_shared_resource_s2.get_mut().write(core :: mem ::
            MaybeUninit :: new(shared_resources.s2));
            __rtic_internal_shared_resource_h1.get_mut().write(core :: mem ::
            MaybeUninit :: new(shared_resources.h1));
            __rtic_internal_shared_resource_h2.get_mut().write(core :: mem ::
            MaybeUninit :: new(shared_resources.h2));
            __rtic_internal_local_resource_imu1.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.imu1));
            __rtic_internal_local_resource_imu2.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.imu2));
            __rtic_internal_local_resource_usb_dev.get_mut().write(core :: mem
            :: MaybeUninit :: new(local_resources.usb_dev));
            __rtic_internal_local_resource_serial.get_mut().write(core :: mem
            :: MaybeUninit :: new(local_resources.serial)); rtic :: export ::
            interrupt :: enable();
        }); loop {}
    }
}