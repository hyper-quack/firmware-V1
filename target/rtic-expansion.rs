#[doc = r" The RTIC application module"] pub mod app
{
    #[doc =
    r" Always include the device crate which contains the vector table"] use
    stm32h7xx_hal :: pac as
    you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml;
    #[doc =
    r" Holds the maximum priority level for use by async HAL drivers."]
    #[no_mangle] static RTIC_ASYNC_MAX_LOGICAL_PRIO : u8 = 1 << stm32h7xx_hal
    :: pac :: NVIC_PRIO_BITS; use super :: * ; use embedded_hal :: spi ::
    MODE_3; use stm32h7xx_hal :: gpio :: { Output, Pin }; use stm32h7xx_hal ::
    prelude :: * ; use stm32h7xx_hal :: rcc :: rec ::
    { Spi123ClkSel, UsbClkSel }; use stm32h7xx_hal :: usb_hs ::
    { UsbBus, USB2 }; use stm32h7xx_hal :: { pac, spi }; use usb_device ::
    prelude :: * ; use crate :: ahrs :: Attitude; use crate :: estimator ::
    { Estimator, Rotation }; use crate :: filters :: ImuLpf; use crate :: imu
    :: { Health, Imu, ImuOut }; use crate :: mavlink ::
    {
        Encoder, MAV_SYS_STATUS_SENSOR_3D_ACCEL, MAV_SYS_STATUS_SENSOR_3D_GYRO
    };
    #[doc =
    " Tasks tick at 1 kHz (Systick monotonic), so the filter sample rate and"]
    #[doc = " fusion step are both 1 ms."] const SAMPLE_HZ : f32 = 1000.0;
    const DT : f32 = 1.0 / SAMPLE_HZ; const GYRO_CUTOFF_HZ : f32 = 80.0; const
    ACCEL_CUTOFF_HZ : f32 = 20.0; const AHRS_KP : f32 = 1.0; const AHRS_KI :
    f32 = 0.05; type Imu1 = Imu < spi :: Spi < pac :: SPI1, spi :: Enabled > ,
    Pin < 'A', 4, Output > > ; type Imu2 = Imu < spi :: Spi < pac :: SPI4, spi
    :: Enabled > , Pin < 'B', 1, Output > > ; type MyUsbBus = UsbBus < USB2 >
    ;
    #[doc =
    " Write a buffer to the CDC IN endpoint in one non-blocking call. Frames"]
    #[doc =
    " longer than one USB packet (64 B) are written in a tight loop with a"]
    #[doc =
    " single poll() between packets; if the endpoint is still busy after"]
    #[doc =
    " draining, the remainder is dropped (a later frame will resynchronize)."]
    fn
    pump_write(usb_dev : & mut UsbDevice < 'static, MyUsbBus > , serial : &
    mut usbd_serial :: SerialPort < 'static, MyUsbBus > , data : & [u8],)
    {
        let mut off = 0; while off < data.len()
        {
            match serial.write(& data [off ..])
            {
                Ok(n) if n > 0 => off += n, _ =>
                {
                    usb_dev.poll(& mut [serial]); match
                    serial.write(& data [off ..])
                    { Ok(n) if n > 0 => off += n, _ => break, }
                }
            }
        }
    } #[doc = r" User code end"] #[doc = r"Shared resources"] struct Shared
    {
        #[doc =
        " Latest filtered output of each IMU, published by the sampling tasks."]
        out1 : ImuOut, out2 : ImuOut,
        #[doc = " Fused attitude, published by the estimator task."] att :
        Attitude,
    } #[doc = r"Local resources"] struct Local
    {
        imu1 : Imu1, lpf1 : ImuLpf, imu2 : Imu2, lpf2 : ImuLpf, est :
        Estimator, usb_dev : UsbDevice < 'static, MyUsbBus > , serial :
        usbd_serial :: SerialPort < 'static, MyUsbBus > , mavlink : Encoder,
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
        :: USB_CLASS_CDC).build(); let lpf1 = ImuLpf ::
        new(SAMPLE_HZ, GYRO_CUTOFF_HZ, ACCEL_CUTOFF_HZ); let lpf2 = ImuLpf ::
        new(SAMPLE_HZ, GYRO_CUTOFF_HZ, ACCEL_CUTOFF_HZ); let est = Estimator
        :: new(AHRS_KP, AHRS_KI, Rotation :: Roll180, Rotation :: Pitch180);
        imu1_task :: spawn().ok(); imu2_task :: spawn().ok(); estimator_task
        :: spawn().ok(); usb_task :: spawn().ok();
        (Shared
        {
            out1 : ImuOut { health : h1, .. Default :: default() }, out2 :
            ImuOut { health : h2, .. Default :: default() }, att : Attitude ::
            default(),
        }, Local
        {
            imu1, lpf1, imu2, lpf2, est, usb_dev, serial, mavlink : Encoder ::
            new(),
        },)
    } impl < 'a > __rtic_internal_imu1_taskLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_imu1_taskLocalResources
            {
                imu1 : & mut *
                (& mut *
                __rtic_internal_local_resource_imu1.get_mut()).as_mut_ptr(),
                lpf1 : & mut *
                (& mut *
                __rtic_internal_local_resource_lpf1.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_imu1_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_imu1_taskSharedResources
            {
                out1 : shared_resources :: out1_that_needs_to_be_locked ::
                new(), __rtic_internal_marker : core :: marker :: PhantomData,
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
                lpf2 : & mut *
                (& mut *
                __rtic_internal_local_resource_lpf2.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_imu2_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_imu2_taskSharedResources
            {
                out2 : shared_resources :: out2_that_needs_to_be_locked ::
                new(), __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_estimator_taskLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_estimator_taskLocalResources
            {
                est : & mut *
                (& mut *
                __rtic_internal_local_resource_est.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_estimator_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_estimator_taskSharedResources
            {
                out1 : shared_resources :: out1_that_needs_to_be_locked ::
                new(), out2 : shared_resources :: out2_that_needs_to_be_locked
                :: new(), att : shared_resources ::
                att_that_needs_to_be_locked :: new(), __rtic_internal_marker :
                core :: marker :: PhantomData,
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
                mavlink : & mut *
                (& mut *
                __rtic_internal_local_resource_mavlink.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_usb_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usb_taskSharedResources
            {
                out1 : shared_resources :: out1_that_needs_to_be_locked ::
                new(), out2 : shared_resources :: out2_that_needs_to_be_locked
                :: new(), __rtic_internal_marker : core :: marker ::
                PhantomData,
            }
        }
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `imu1_task` has access to"] pub struct
    __rtic_internal_imu1_taskLocalResources < 'a >
    {
        #[allow(missing_docs)] pub imu1 : & 'a mut Imu1,
        #[allow(missing_docs)] pub lpf1 : & 'a mut ImuLpf, #[doc(hidden)] pub
        __rtic_internal_marker : :: core :: marker :: PhantomData < & 'a () >
        ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `imu1_task` has access to"] pub struct
    __rtic_internal_imu1_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub out1 : shared_resources ::
        out1_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
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
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM3); Ok(())
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
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM3);
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
        #[allow(missing_docs)] pub imu2 : & 'a mut Imu2,
        #[allow(missing_docs)] pub lpf2 : & 'a mut ImuLpf, #[doc(hidden)] pub
        __rtic_internal_marker : :: core :: marker :: PhantomData < & 'a () >
        ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `imu2_task` has access to"] pub struct
    __rtic_internal_imu2_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub out2 : shared_resources ::
        out2_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
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
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM3); Ok(())
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
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM3);
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
    #[doc = "Local resources `estimator_task` has access to"] pub struct
    __rtic_internal_estimator_taskLocalResources < 'a >
    {
        #[allow(missing_docs)] pub est : & 'a mut Estimator, #[doc(hidden)]
        pub __rtic_internal_marker : :: core :: marker :: PhantomData < & 'a
        () > ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `estimator_task` has access to"] pub struct
    __rtic_internal_estimator_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub out1 : shared_resources ::
        out1_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub out2
        : shared_resources :: out2_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub att : shared_resources ::
        att_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct
    __rtic_internal_estimator_task_Context < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : estimator_task :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        estimator_task :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_estimator_task_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_estimator_task_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                estimator_task :: LocalResources :: new(), shared :
                estimator_task :: SharedResources :: new(),
            }
        }
    } #[doc = r" Spawns the task directly"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_estimator_task_spawn() -> :: core ::
    result :: Result < (), () >
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(estimator_task, &
            __rtic_internal_estimator_task_EXEC); if exec.try_allocate()
            {
                exec.spawn(estimator_task(unsafe
                { estimator_task :: Context :: new() })); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM2); Ok(())
            } else { Err(()) }
        }
    } #[doc = r" Gives waker to the task"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_estimator_task_waker() -> :: core ::
    task :: Waker
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(estimator_task, &
            __rtic_internal_estimator_task_EXEC);
            exec.waker(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(estimator_task, &
                __rtic_internal_estimator_task_EXEC); exec.set_pending(); rtic
                :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM2);
            })
        }
    } #[allow(non_snake_case)] #[doc = "Software task"] pub mod estimator_task
    {
        #[doc(inline)] pub use super ::
        __rtic_internal_estimator_taskLocalResources as LocalResources;
        #[doc(inline)] pub use super ::
        __rtic_internal_estimator_taskSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_estimator_task_Context
        as Context; #[doc(inline)] pub use super ::
        __rtic_internal_estimator_task_spawn as spawn; #[doc(inline)] pub use
        super :: __rtic_internal_estimator_task_waker as waker;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `usb_task` has access to"] pub struct
    __rtic_internal_usb_taskLocalResources < 'a >
    {
        #[allow(missing_docs)] pub usb_dev : & 'a mut UsbDevice < 'static,
        MyUsbBus > , #[allow(missing_docs)] pub serial : & 'a mut usbd_serial
        :: SerialPort < 'static, MyUsbBus > , #[allow(missing_docs)] pub
        mavlink : & 'a mut Encoder, #[doc(hidden)] pub __rtic_internal_marker
        : :: core :: marker :: PhantomData < & 'a () > ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `usb_task` has access to"] pub struct
    __rtic_internal_usb_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub out1 : shared_resources ::
        out1_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub out2
        : shared_resources :: out2_that_needs_to_be_locked < 'a > ,
        #[doc(hidden)] pub __rtic_internal_marker : core :: marker ::
        PhantomData < & 'a () > ,
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
            let health = cx.local.imu1.health; let out = if let Health ::
            Ok(_) = health
            {
                let s = cx.local.imu1.read(); let (gyro, accel) =
                cx.local.lpf1.apply(s.gyro_dps(), s.accel_g()); ImuOut
                { gyro, accel, health, }
            } else { ImuOut { health, .. Default :: default() } };
            cx.shared.out1.lock(| o | * o = out); Mono ::
            delay(1.millis()).await;
        }
    } #[allow(non_snake_case)] async fn imu2_task < 'a >
    (mut cx : imu2_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; loop
        {
            let health = cx.local.imu2.health; let out = if let Health ::
            Ok(_) = health
            {
                let s = cx.local.imu2.read(); let (gyro, accel) =
                cx.local.lpf2.apply(s.gyro_dps(), s.accel_g()); ImuOut
                { gyro, accel, health, }
            } else { ImuOut { health, .. Default :: default() } };
            cx.shared.out2.lock(| o | * o = out); Mono ::
            delay(1.millis()).await;
        }
    } #[allow(non_snake_case)] async fn estimator_task < 'a >
    (cx : estimator_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let est =
        cx.local.est; let estimator_task :: SharedResources
        { mut out1, mut out2, mut att, .. } = cx.shared; loop
        {
            let o1 = out1.lock(| o | * o); let o2 = out2.lock(| o | * o); let
            a = est.update(& o1, & o2, DT); att.lock(| x | * x = a); Mono ::
            delay(1.millis()).await;
        }
    } #[allow(non_snake_case)] async fn usb_task < 'a >
    (cx : usb_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let
        usb_dev = cx.local.usb_dev; let serial = cx.local.serial; let mavlink
        = cx.local.mavlink; let usb_task :: SharedResources
        { mut out1, mut out2, .. } = cx.shared; let mut tick : u32 = 0; loop
        {
            usb_dev.poll(& mut [serial]);
            {
                let mut scratch = [0u8; 64]; let _ =
                serial.read(& mut scratch);
            } if usb_dev.state() != usb_device :: device :: UsbDeviceState ::
            Configured
            {
                tick = tick.wrapping_add(1); Mono :: delay(1.millis()).await;
                continue;
            } if tick % 50 == 0
            {
                let o = out1.lock(| o | * o); if matches!
                (o.health, Health::Ok(_))
                {
                    let frame =
                    mavlink.highres_imu(tick as u64 * 1_000, 0, Rotation ::
                    Roll180.apply(o.accel), Rotation :: Roll180.apply(o.gyro),);
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            } if tick % 50 == 25
            {
                let o = out2.lock(| o | * o); if matches!
                (o.health, Health::Ok(_))
                {
                    let frame =
                    mavlink.highres_imu(tick as u64 * 1_000, 1, Rotation ::
                    Pitch180.apply(o.accel), Rotation ::
                    Pitch180.apply(o.gyro),);
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            } if tick % 1000 == 5
            {
                let frame = mavlink.heartbeat();
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 1000 == 10
            {
                let h1 = out1.lock(| o | o.health); let h2 =
                out2.lock(| o | o.health); let any_ok = matches!
                (h1, Health::Ok(_)) || matches! (h2, Health::Ok(_)); let
                sensors = MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                MAV_SYS_STATUS_SENSOR_3D_GYRO; let frame =
                mavlink.sys_status(sensors, if any_ok { sensors } else { 0 });
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 1000 == 15
            {
                let h = out1.lock(| o | o.health); let ok = matches!
                (h, Health::Ok(_)); let frame =
                mavlink.imu_status(tick, 0, ok, ok, h.whoami());
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 1000 == 20
            {
                let h = out2.lock(| o | o.health); let ok = matches!
                (h, Health::Ok(_)); let frame =
                mavlink.imu_status(tick, 1, ok, ok, h.whoami());
                pump_write(usb_dev, serial, frame.as_slice());
            } tick = tick.wrapping_add(1); Mono :: delay(1.millis()).await;
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic0"] static
    __rtic_internal_shared_resource_out1 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < ImuOut >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: out1_that_needs_to_be_locked < 'a >
    {
        type T = ImuOut; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut ImuOut) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 3u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_out1.get_mut() as * mut
                _, CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic1"] static
    __rtic_internal_shared_resource_out2 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < ImuOut >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: out2_that_needs_to_be_locked < 'a >
    {
        type T = ImuOut; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut ImuOut) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 3u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_out2.get_mut() as * mut
                _, CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic2"] static
    __rtic_internal_shared_resource_att : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Attitude >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: att_that_needs_to_be_locked < 'a >
    {
        type T = Attitude; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut Attitude) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 2u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_att.get_mut() as * mut _,
                CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } mod shared_resources
    {
        #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        out1_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > out1_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                out1_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        out2_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > out2_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                out2_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        att_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > att_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                att_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic3"] static
    __rtic_internal_local_resource_imu1 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Imu1 >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic4"] static
    __rtic_internal_local_resource_lpf1 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < ImuLpf >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic5"] static
    __rtic_internal_local_resource_imu2 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Imu2 >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic6"] static
    __rtic_internal_local_resource_lpf2 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < ImuLpf >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic7"] static
    __rtic_internal_local_resource_est : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Estimator >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic8"] static
    __rtic_internal_local_resource_usb_dev : rtic :: RacyCell < core :: mem ::
    MaybeUninit < UsbDevice < 'static, MyUsbBus > >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic9"] static
    __rtic_internal_local_resource_serial : rtic :: RacyCell < core :: mem ::
    MaybeUninit < usbd_serial :: SerialPort < 'static, MyUsbBus > >> = rtic ::
    RacyCell :: new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic10"] static
    __rtic_internal_local_resource_mavlink : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Encoder >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_upper_case_globals)] static __rtic_internal_imu1_task_EXEC :
    rtic :: export :: executor :: AsyncTaskExecutorPtr = rtic :: export ::
    executor :: AsyncTaskExecutorPtr :: new();
    #[allow(non_upper_case_globals)] static __rtic_internal_imu2_task_EXEC :
    rtic :: export :: executor :: AsyncTaskExecutorPtr = rtic :: export ::
    executor :: AsyncTaskExecutorPtr :: new();
    #[allow(non_upper_case_globals)] static
    __rtic_internal_estimator_task_EXEC : rtic :: export :: executor ::
    AsyncTaskExecutorPtr = rtic :: export :: executor :: AsyncTaskExecutorPtr
    :: new(); #[allow(non_upper_case_globals)] static
    __rtic_internal_usb_task_EXEC : rtic :: export :: executor ::
    AsyncTaskExecutorPtr = rtic :: export :: executor :: AsyncTaskExecutorPtr
    :: new(); #[allow(non_snake_case)]
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
    #[doc = "Interrupt handler to dispatch async tasks at priority 2"]
    #[no_mangle] unsafe fn LPTIM2()
    {
        #[doc = r" The priority of this interrupt handler"] const PRIORITY :
        u8 = 2u8; rtic :: export ::
        run(PRIORITY, ||
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(estimator_task, &
            __rtic_internal_estimator_task_EXEC);
            exec.poll(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(estimator_task, &
                __rtic_internal_estimator_task_EXEC); exec.set_pending(); rtic
                :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM2);
            });
        });
    } #[allow(non_snake_case)]
    #[doc = "Interrupt handler to dispatch async tasks at priority 3"]
    #[no_mangle] unsafe fn LPTIM3()
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
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM3);
            }); let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(imu2_task, & __rtic_internal_imu2_task_EXEC);
            exec.poll(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(imu2_task, & __rtic_internal_imu2_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM3);
            });
        });
    } #[doc(hidden)] #[no_mangle] unsafe extern "C" fn main() -> !
    {
        rtic :: export :: assert_send :: < ImuOut > (); rtic :: export ::
        assert_send :: < Attitude > (); rtic :: export :: interrupt ::
        disable(); let mut core : rtic :: export :: Peripherals = rtic ::
        export :: Peripherals :: steal().into(); let _ =
        you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml ::
        interrupt :: LPTIM1; let _ =
        you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml ::
        interrupt :: LPTIM2; let _ =
        you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml ::
        interrupt :: LPTIM3; const _ : () = if
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
        (1 << stm32h7xx_hal :: pac :: NVIC_PRIO_BITS) < 2u8 as usize
        {
            :: core :: panic!
            ("Maximum priority used by interrupt vector 'LPTIM2' is more than supported by hardware");
        };
        core.NVIC.set_priority(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: LPTIM2, rtic :: export ::
        cortex_logical2hw(2u8, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS),); rtic
        :: export :: NVIC ::
        unmask(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: LPTIM2); const _ : () = if
        (1 << stm32h7xx_hal :: pac :: NVIC_PRIO_BITS) < 3u8 as usize
        {
            :: core :: panic!
            ("Maximum priority used by interrupt vector 'LPTIM3' is more than supported by hardware");
        };
        core.NVIC.set_priority(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: LPTIM3, rtic :: export ::
        cortex_logical2hw(3u8, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS),); rtic
        :: export :: NVIC ::
        unmask(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: LPTIM3); #[inline(never)] fn __rtic_init_resources < F
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
        new_1_args(estimator_task)); executors_size += :: core :: mem ::
        size_of_val(& executor);
        __rtic_internal_estimator_task_EXEC.set_in_main(& executor); let
        executor = :: core :: mem :: ManuallyDrop ::
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
            __rtic_internal_shared_resource_out1.get_mut().write(core :: mem
            :: MaybeUninit :: new(shared_resources.out1));
            __rtic_internal_shared_resource_out2.get_mut().write(core :: mem
            :: MaybeUninit :: new(shared_resources.out2));
            __rtic_internal_shared_resource_att.get_mut().write(core :: mem ::
            MaybeUninit :: new(shared_resources.att));
            __rtic_internal_local_resource_imu1.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.imu1));
            __rtic_internal_local_resource_lpf1.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.lpf1));
            __rtic_internal_local_resource_imu2.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.imu2));
            __rtic_internal_local_resource_lpf2.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.lpf2));
            __rtic_internal_local_resource_est.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.est));
            __rtic_internal_local_resource_usb_dev.get_mut().write(core :: mem
            :: MaybeUninit :: new(local_resources.usb_dev));
            __rtic_internal_local_resource_serial.get_mut().write(core :: mem
            :: MaybeUninit :: new(local_resources.serial));
            __rtic_internal_local_resource_mavlink.get_mut().write(core :: mem
            :: MaybeUninit :: new(local_resources.mavlink)); rtic :: export ::
            interrupt :: enable();
        }); loop {}
    }
}