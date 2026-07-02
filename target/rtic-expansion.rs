#[doc = r" The RTIC application module"] pub mod app
{
    #[doc =
    r" Always include the device crate which contains the vector table"] use
    stm32h7xx_hal :: pac as
    you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml;
    #[doc =
    r" Holds the maximum priority level for use by async HAL drivers."]
    #[no_mangle] static RTIC_ASYNC_MAX_LOGICAL_PRIO : u8 = 3u8; use super :: *
    ; use core :: fmt :: Write as FmtWrite; use embedded_hal :: spi :: MODE_3;
    use stm32h7xx_hal :: gpio :: { Output, Pin }; use stm32h7xx_hal :: prelude
    :: * ; use stm32h7xx_hal :: rcc :: rec :: { Spi123ClkSel, UsbClkSel }; use
    stm32h7xx_hal :: serial :: { self, Rx }; use stm32h7xx_hal :: usb_hs ::
    { UsbBus, USB2 }; use stm32h7xx_hal :: { i2c, pac, spi }; use usb_device
    :: prelude :: * ; use crate :: ahrs :: Attitude; use crate :: baro ::
    { Baro, BaroData }; use crate :: compass ::
    { Compass, MagCal, MagData, MagRotation }; use crate :: crsf ::
    { CrsfParser, RcChannels }; use crate :: ekf :: { Ekf, NavSolution }; use
    crate :: dshot; use crate :: esc :: { Esc, EscTelemetry }; use crate ::
    esc_telem :: EscTelemParser; use crate :: estimator ::
    { Estimator, Rotation }; use crate :: filters :: ImuLpf; use crate :: gps
    :: { GpsData, NmeaParser }; use crate :: imu :: { Health, Imu, ImuOut };
    use crate :: mavlink ::
    {
        DecodeDiag, Decoder, Encoder, Inbound, MAV_SYS_STATUS_SENSOR_3D_ACCEL,
        MAV_SYS_STATUS_SENSOR_3D_GYRO, MAV_CMD_DO_MOTOR_TEST,
    }; use crate :: mtf01 :: { Mtf01Data, MspParser }; use crate :: nav ::
    { Nav, NavState }; use crate :: tfluna :: { TfLunaData, TfLunaParser };
    #[doc =
    " Tasks tick at 1 kHz (Systick monotonic), so the filter sample rate and"]
    #[doc = " fusion step are both 1 ms."] const SAMPLE_HZ : f32 = 1000.0;
    const DT : f32 = 1.0 / SAMPLE_HZ; const DEG2RAD : f32 = core :: f32 ::
    consts :: PI / 180.0;
    #[doc =
    " Magnetic declination at your location, degrees east-positive. 0 leaves the"]
    #[doc =
    " heading magnetic (EKF horizontal will be rotated by the local declination)."]
    const MAG_DECLINATION_DEG : f32 = 0.0;
    #[doc = " Compass mount orientation relative to the FC."] const
    MAG_ROTATION : MagRotation = MagRotation :: None;
    #[doc =
    " Hard-iron offset (Gauss) and soft-iron scale — from a bench calibration or"]
    #[doc = " the in-field RC-triggered collector. Identity until measured."]
    const MAG_OFFSET : [f32; 3] = [0.0; 3]; const MAG_SCALE : [f32; 3] =
    [1.0; 3];
    #[doc =
    " RC channel (0-based) whose high position triggers in-field compass"]
    #[doc =
    " calibration: flip high, fly figure-8s, flip low to store. AUX by default."]
    const CAL_RC_CHANNEL : usize = 6; const GYRO_CUTOFF_HZ : f32 = 80.0; const
    ACCEL_CUTOFF_HZ : f32 = 20.0; const AHRS_KP : f32 = 1.0; const AHRS_KI :
    f32 = 0.05;
    #[doc =
    " GPS NMEA baud. uBlox NEO-M8N powers up emitting NMEA at 9600 (factory"]
    #[doc =
    " default); if a module has been pre-set to another rate (38400 is common"]
    #[doc = " on Pixhawk-branded units), change this and rebuild."] const
    GPS_BAUD : u32 = 9600; #[doc = " MTF-01 MSP output baud (USART2)."] const
    MTF01_BAUD : u32 = 115_200;
    #[doc = " ExpressLRS / CRSF baud (UART5). ELRS uses 420 kbaud."] const
    CRSF_BAUD : u32 = 420_000;
    #[doc = " TF-Luna side-lidar baud (USART6 / UART7). Default is 115200."]
    const TFLUNA_BAUD : u32 = 115_200;
    #[doc =
    " ESC telemetry (BLHeli32 / KISS T pad → USART3 RX / PD9). Standard is 115200."]
    const ESC_TELEM_BAUD : u32 = 115_200; const FIRMWARE_TAG : & str =
    "scky-fc esc-permotor 2026-06-27"; type Imu1 = Imu < spi :: Spi < pac ::
    SPI1, spi :: Enabled > , Pin < 'A', 4, Output > > ; type Imu2 = Imu < spi
    :: Spi < pac :: SPI4, spi :: Enabled > , Pin < 'B', 1, Output > > ; type
    I2c2 = i2c :: I2c < pac :: I2C2 > ; type MyUsbBus = UsbBus < USB2 > ;
    #[doc =
    " Write a buffer to the CDC IN endpoint in one non-blocking call. Frames"]
    #[doc =
    " longer than one USB packet (64 B) are written in a tight loop with a"]
    #[doc =
    " single poll() between packets; if the endpoint is still busy after"]
    #[doc =
    " draining, the remainder is dropped (a later frame will resynchronize)."]
    #[doc =
    " Apply one decoded inbound MAVLink command to the ESC controller."] fn
    apply_inbound(e : & mut Esc, cmd : Inbound, now_ms : u32) -> bool
    {
        match cmd
        {
            Inbound :: MotorTest { motor, throttle, timeout_s, .. } =>
            {
                let timeout_ms = if timeout_s.is_finite() && timeout_s > 0.0
                { (timeout_s * 1000.0) as u32 } else { 0 };
                e.start_test(motor, throttle, timeout_ms, now_ms)
            } Inbound :: EscSet
            {
                master_enabled, protocol, refresh_hz, bidir, dir_mask,
                mode3d_mask, pole_count, cur_scale, cur_offset,
            } =>
            {
                e.apply_set(master_enabled, protocol, refresh_hz, bidir,
                dir_mask, mode3d_mask, pole_count, cur_scale, cur_offset,);
                true
            } Inbound :: EscCmd { target, command } =>
            { e.queue_command(target, command); true }
        }
    } fn
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
        #[doc = " Latest GPS solution, published by the USART1 RX interrupt."]
        gps : GpsData,
        #[doc = " Latest magnetometer reading, published by the I2C task."]
        mag : MagData,
        #[doc = " Latest barometer reading, published by the I2C task."] baro
        : BaroData,
        #[doc =
        " Latest MTF-01 flow + lidar, published by the USART2 RX interrupt."]
        flow : Mtf01Data,
        #[doc =
        " Latest RC channels + link, published by the UART5 RX interrupt."] rc
        : RcChannels,
        #[doc = " Flow/lidar navigation estimate, published by the nav task."]
        navs : NavState,
        #[doc =
        " Side obstacle lidars, published by the USART6 / UART7 interrupts."]
        prox_left : TfLunaData, prox_right : TfLunaData,
        #[doc =
        " World-frame (N/E/Up) gravity-removed acceleration, published by the"]
        #[doc = " estimator for the EKF prediction step."] accel_w : [f32; 3],
        #[doc = " Fused navigation solution, published by the EKF task."]
        navsol : NavSolution,
        #[doc =
        " ESC controller: config, motor-test state, DShot command queue. Mutated"]
        #[doc = " by the USB command path, read by `dshot_task`."] esc : Esc,
        #[doc =
        " Latest ESC telemetry, published by the USART3 RX interrupt."]
        esc_tlm : EscTelemetry,
    } #[doc = r"Local resources"] struct Local
    {
        imu1 : Imu1, lpf1 : ImuLpf, imu2 : Imu2, lpf2 : ImuLpf, est :
        Estimator, usb_dev : UsbDevice < 'static, MyUsbBus > , serial :
        usbd_serial :: SerialPort < 'static, MyUsbBus > , mavlink : Encoder,
        gps_rx : Rx < pac :: USART1 > , gps_parser : NmeaParser, i2c2 : I2c2,
        compass : Compass, baro : Baro, mtf_rx : Rx < pac :: USART2 > ,
        mtf_parser : MspParser, crsf_rx : Rx < pac :: UART5 > , crsf_parser :
        CrsfParser, nav : Nav, tfl_left_rx : Rx < pac :: USART6 > ,
        tfl_left_parser : TfLunaParser, tfl_right_rx : Rx < pac :: UART7 > ,
        tfl_right_parser : TfLunaParser, ekf : Ekf, esc_tx_rx : Rx < pac ::
        USART3 > , esc_tx_parser : EscTelemParser, decoder : Decoder,
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
        let dp : pac :: Peripherals = cx.device; let mut cp = cx.core; let pwr
        = dp.PWR.constrain(); let pwrcfg = pwr.freeze(); let rcc =
        dp.RCC.constrain(); let mut ccdr = rcc.freeze(pwrcfg, & dp.SYSCFG);
        let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must be running");
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel :: Hsi48);
        ccdr.peripheral.kernel_spi123_clk_mux(Spi123ClkSel :: Per);
        cp.DCB.enable_trace(); cp.DWT.enable_cycle_counter(); Mono ::
        start(cp.SYST, 64_000_000); let gpioa =
        dp.GPIOA.split(ccdr.peripheral.GPIOA); let gpiob =
        dp.GPIOB.split(ccdr.peripheral.GPIOB); let gpioc =
        dp.GPIOC.split(ccdr.peripheral.GPIOC); let gpiod =
        dp.GPIOD.split(ccdr.peripheral.GPIOD); let gpioe =
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
        imu1.init(& delay_us); let h2 = imu2.init(& delay_us); let serial1 =
        dp.USART1.serial((gpioa.pa9.into_alternate :: < 7 > (),
        gpioa.pa10.into_alternate :: < 7 > ().internal_pull_up(true),),
        GPS_BAUD.bps(), ccdr.peripheral.USART1, & ccdr.clocks,).unwrap(); let
        (_gps_tx, mut gps_rx) = serial1.split(); gps_rx.listen(); let mut i2c2
        =
        dp.I2C2.i2c((gpiob.pb10.into_alternate_open_drain(),
        gpiob.pb11.into_alternate_open_drain(),), 400.kHz(),
        ccdr.peripheral.I2C2, & ccdr.clocks,); let mut compass = Compass ::
        new(); compass.init(& mut i2c2); let mut baro = Baro :: new();
        baro.init(& mut i2c2, & delay_us); let serial2 =
        dp.USART2.serial((gpiod.pd5.into_alternate :: < 7 > (),
        gpiod.pd6.into_alternate :: < 7 > ().internal_pull_up(true),),
        MTF01_BAUD.bps(), ccdr.peripheral.USART2, & ccdr.clocks,).unwrap();
        let (_mtf_tx, mut mtf_rx) = serial2.split(); mtf_rx.listen(); let
        serial5 =
        dp.UART5.serial((gpiob.pb6.into_alternate :: < 14 > (),
        gpiob.pb5.into_alternate :: < 14 > ().internal_pull_up(true),),
        CRSF_BAUD.bps(), ccdr.peripheral.UART5, & ccdr.clocks,).unwrap(); let
        (_crsf_tx, mut crsf_rx) = serial5.split(); crsf_rx.listen(); let
        serial6 =
        dp.USART6.serial((gpioc.pc6.into_alternate :: < 7 >
        ().internal_pull_up(true), gpioc.pc7.into_alternate :: < 7 > (),),
        serial :: config :: Config :: new(TFLUNA_BAUD.bps()).swaptxrx(true),
        ccdr.peripheral.USART6, & ccdr.clocks,).unwrap(); let
        (_tfl_l_tx, mut tfl_left_rx) = serial6.split(); tfl_left_rx.listen();
        let serial7 =
        dp.UART7.serial((gpioe.pe8.into_alternate :: < 7 > (),
        gpioe.pe7.into_alternate :: < 7 > ().internal_pull_up(true),),
        TFLUNA_BAUD.bps(), ccdr.peripheral.UART7, & ccdr.clocks,).unwrap();
        let (_tfl_r_tx, mut tfl_right_rx) = serial7.split();
        tfl_right_rx.listen(); let serial3 =
        dp.USART3.serial((gpiod.pd8.into_alternate :: < 7 > (),
        gpiod.pd9.into_alternate :: < 7 > ().internal_pull_up(true),),
        ESC_TELEM_BAUD.bps(), ccdr.peripheral.USART3, &
        ccdr.clocks,).unwrap(); let (_esc_tx_tx, mut esc_tx_rx) =
        serial3.split(); esc_tx_rx.listen(); dshot :: init_pins(); let usb =
        USB2 ::
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
        new(SAMPLE_HZ, GYRO_CUTOFF_HZ, ACCEL_CUTOFF_HZ); let mut est =
        Estimator ::
        new(AHRS_KP, AHRS_KI, Rotation :: Roll180, Rotation :: Pitch180);
        est.set_declination(MAG_DECLINATION_DEG);
        est.set_mag_cal(MagCal :: new(MAG_ROTATION, MAG_OFFSET, MAG_SCALE));
        imu1_task :: spawn().ok(); imu2_task :: spawn().ok(); estimator_task
        :: spawn().ok(); i2c_task :: spawn().ok(); nav_task :: spawn().ok();
        ekf_task :: spawn().ok(); usb_task :: spawn().ok(); dshot_task ::
        spawn().ok();
        (Shared
        {
            out1 : ImuOut { health : h1, .. Default :: default() }, out2 :
            ImuOut { health : h2, .. Default :: default() }, att : Attitude ::
            default(), gps : GpsData :: default(), mag : MagData :: default(),
            baro : BaroData :: default(), flow : Mtf01Data :: default(), rc :
            RcChannels :: default(), navs : NavState :: default(), prox_left :
            TfLunaData :: default(), prox_right : TfLunaData :: default(),
            accel_w : [0.0; 3], navsol : NavSolution :: default(), esc : Esc
            :: new(), esc_tlm : EscTelemetry :: new(),
        }, Local
        {
            imu1, lpf1, imu2, lpf2, est, usb_dev, serial, mavlink : Encoder ::
            new(), gps_rx, gps_parser : NmeaParser :: new(), i2c2, compass,
            baro, mtf_rx, mtf_parser : MspParser :: new(), crsf_rx,
            crsf_parser : CrsfParser :: new(), nav : Nav :: new(),
            tfl_left_rx, tfl_left_parser : TfLunaParser :: new(),
            tfl_right_rx, tfl_right_parser : TfLunaParser :: new(), ekf : Ekf
            :: new(), esc_tx_rx, esc_tx_parser : EscTelemParser :: new(),
            decoder : Decoder :: new(),
        },)
    } #[allow(non_snake_case)] #[no_mangle] unsafe fn USART1()
    {
        const PRIORITY : u8 = 4u8; rtic :: export ::
        run(PRIORITY, || { usart1_rx(usart1_rx :: Context :: new()) });
    } impl < 'a > __rtic_internal_usart1_rxLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart1_rxLocalResources
            {
                gps_rx : & mut *
                (& mut *
                __rtic_internal_local_resource_gps_rx.get_mut()).as_mut_ptr(),
                gps_parser : & mut *
                (& mut *
                __rtic_internal_local_resource_gps_parser.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_usart1_rxSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart1_rxSharedResources
            {
                gps : shared_resources :: gps_that_needs_to_be_locked ::
                new(), __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } #[allow(non_snake_case)] #[no_mangle] unsafe fn USART2()
    {
        const PRIORITY : u8 = 4u8; rtic :: export ::
        run(PRIORITY, || { usart2_rx(usart2_rx :: Context :: new()) });
    } impl < 'a > __rtic_internal_usart2_rxLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart2_rxLocalResources
            {
                mtf_rx : & mut *
                (& mut *
                __rtic_internal_local_resource_mtf_rx.get_mut()).as_mut_ptr(),
                mtf_parser : & mut *
                (& mut *
                __rtic_internal_local_resource_mtf_parser.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_usart2_rxSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart2_rxSharedResources
            {
                flow : shared_resources :: flow_that_needs_to_be_locked ::
                new(), __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } #[allow(non_snake_case)] #[no_mangle] unsafe fn UART5()
    {
        const PRIORITY : u8 = 4u8; rtic :: export ::
        run(PRIORITY, || { uart5_rx(uart5_rx :: Context :: new()) });
    } impl < 'a > __rtic_internal_uart5_rxLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_uart5_rxLocalResources
            {
                crsf_rx : & mut *
                (& mut *
                __rtic_internal_local_resource_crsf_rx.get_mut()).as_mut_ptr(),
                crsf_parser : & mut *
                (& mut *
                __rtic_internal_local_resource_crsf_parser.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_uart5_rxSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_uart5_rxSharedResources
            {
                rc : shared_resources :: rc_that_needs_to_be_locked :: new(),
                __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } #[allow(non_snake_case)] #[no_mangle] unsafe fn USART6()
    {
        const PRIORITY : u8 = 4u8; rtic :: export ::
        run(PRIORITY, || { usart6_rx(usart6_rx :: Context :: new()) });
    } impl < 'a > __rtic_internal_usart6_rxLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart6_rxLocalResources
            {
                tfl_left_rx : & mut *
                (& mut *
                __rtic_internal_local_resource_tfl_left_rx.get_mut()).as_mut_ptr(),
                tfl_left_parser : & mut *
                (& mut *
                __rtic_internal_local_resource_tfl_left_parser.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_usart6_rxSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart6_rxSharedResources
            {
                prox_left : shared_resources ::
                prox_left_that_needs_to_be_locked :: new(),
                __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } #[allow(non_snake_case)] #[no_mangle] unsafe fn UART7()
    {
        const PRIORITY : u8 = 4u8; rtic :: export ::
        run(PRIORITY, || { uart7_rx(uart7_rx :: Context :: new()) });
    } impl < 'a > __rtic_internal_uart7_rxLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_uart7_rxLocalResources
            {
                tfl_right_rx : & mut *
                (& mut *
                __rtic_internal_local_resource_tfl_right_rx.get_mut()).as_mut_ptr(),
                tfl_right_parser : & mut *
                (& mut *
                __rtic_internal_local_resource_tfl_right_parser.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_uart7_rxSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_uart7_rxSharedResources
            {
                prox_right : shared_resources ::
                prox_right_that_needs_to_be_locked :: new(),
                __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } #[allow(non_snake_case)] #[no_mangle] unsafe fn USART3()
    {
        const PRIORITY : u8 = 4u8; rtic :: export ::
        run(PRIORITY, || { usart3_rx(usart3_rx :: Context :: new()) });
    } impl < 'a > __rtic_internal_usart3_rxLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart3_rxLocalResources
            {
                esc_tx_rx : & mut *
                (& mut *
                __rtic_internal_local_resource_esc_tx_rx.get_mut()).as_mut_ptr(),
                esc_tx_parser : & mut *
                (& mut *
                __rtic_internal_local_resource_esc_tx_parser.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_usart3_rxSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart3_rxSharedResources
            {
                esc : shared_resources :: esc_that_needs_to_be_locked ::
                new(), esc_tlm : shared_resources ::
                esc_tlm_that_needs_to_be_locked :: new(),
                __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `usart1_rx` has access to"] pub struct
    __rtic_internal_usart1_rxLocalResources < 'a >
    {
        #[allow(missing_docs)] pub gps_rx : & 'a mut Rx < pac :: USART1 > ,
        #[allow(missing_docs)] pub gps_parser : & 'a mut NmeaParser,
        #[doc(hidden)] pub __rtic_internal_marker : :: core :: marker ::
        PhantomData < & 'a () > ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `usart1_rx` has access to"] pub struct
    __rtic_internal_usart1_rxSharedResources < 'a >
    {
        #[allow(missing_docs)] pub gps : shared_resources ::
        gps_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct
    __rtic_internal_usart1_rx_Context < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : usart1_rx :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        usart1_rx :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_usart1_rx_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart1_rx_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                usart1_rx :: LocalResources :: new(), shared : usart1_rx ::
                SharedResources :: new(),
            }
        }
    } #[allow(non_snake_case)] #[doc = "Hardware task"] pub mod usart1_rx
    {
        #[doc(inline)] pub use super ::
        __rtic_internal_usart1_rxLocalResources as LocalResources;
        #[doc(inline)] pub use super ::
        __rtic_internal_usart1_rxSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_usart1_rx_Context as
        Context;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `usart2_rx` has access to"] pub struct
    __rtic_internal_usart2_rxLocalResources < 'a >
    {
        #[allow(missing_docs)] pub mtf_rx : & 'a mut Rx < pac :: USART2 > ,
        #[allow(missing_docs)] pub mtf_parser : & 'a mut MspParser,
        #[doc(hidden)] pub __rtic_internal_marker : :: core :: marker ::
        PhantomData < & 'a () > ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `usart2_rx` has access to"] pub struct
    __rtic_internal_usart2_rxSharedResources < 'a >
    {
        #[allow(missing_docs)] pub flow : shared_resources ::
        flow_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct
    __rtic_internal_usart2_rx_Context < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : usart2_rx :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        usart2_rx :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_usart2_rx_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart2_rx_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                usart2_rx :: LocalResources :: new(), shared : usart2_rx ::
                SharedResources :: new(),
            }
        }
    } #[allow(non_snake_case)] #[doc = "Hardware task"] pub mod usart2_rx
    {
        #[doc(inline)] pub use super ::
        __rtic_internal_usart2_rxLocalResources as LocalResources;
        #[doc(inline)] pub use super ::
        __rtic_internal_usart2_rxSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_usart2_rx_Context as
        Context;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `uart5_rx` has access to"] pub struct
    __rtic_internal_uart5_rxLocalResources < 'a >
    {
        #[allow(missing_docs)] pub crsf_rx : & 'a mut Rx < pac :: UART5 > ,
        #[allow(missing_docs)] pub crsf_parser : & 'a mut CrsfParser,
        #[doc(hidden)] pub __rtic_internal_marker : :: core :: marker ::
        PhantomData < & 'a () > ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `uart5_rx` has access to"] pub struct
    __rtic_internal_uart5_rxSharedResources < 'a >
    {
        #[allow(missing_docs)] pub rc : shared_resources ::
        rc_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct __rtic_internal_uart5_rx_Context
    < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : uart5_rx :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        uart5_rx :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_uart5_rx_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_uart5_rx_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                uart5_rx :: LocalResources :: new(), shared : uart5_rx ::
                SharedResources :: new(),
            }
        }
    } #[allow(non_snake_case)] #[doc = "Hardware task"] pub mod uart5_rx
    {
        #[doc(inline)] pub use super :: __rtic_internal_uart5_rxLocalResources
        as LocalResources; #[doc(inline)] pub use super ::
        __rtic_internal_uart5_rxSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_uart5_rx_Context as
        Context;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `usart6_rx` has access to"] pub struct
    __rtic_internal_usart6_rxLocalResources < 'a >
    {
        #[allow(missing_docs)] pub tfl_left_rx : & 'a mut Rx < pac :: USART6 >
        , #[allow(missing_docs)] pub tfl_left_parser : & 'a mut TfLunaParser,
        #[doc(hidden)] pub __rtic_internal_marker : :: core :: marker ::
        PhantomData < & 'a () > ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `usart6_rx` has access to"] pub struct
    __rtic_internal_usart6_rxSharedResources < 'a >
    {
        #[allow(missing_docs)] pub prox_left : shared_resources ::
        prox_left_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct
    __rtic_internal_usart6_rx_Context < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : usart6_rx :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        usart6_rx :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_usart6_rx_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart6_rx_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                usart6_rx :: LocalResources :: new(), shared : usart6_rx ::
                SharedResources :: new(),
            }
        }
    } #[allow(non_snake_case)] #[doc = "Hardware task"] pub mod usart6_rx
    {
        #[doc(inline)] pub use super ::
        __rtic_internal_usart6_rxLocalResources as LocalResources;
        #[doc(inline)] pub use super ::
        __rtic_internal_usart6_rxSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_usart6_rx_Context as
        Context;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `uart7_rx` has access to"] pub struct
    __rtic_internal_uart7_rxLocalResources < 'a >
    {
        #[allow(missing_docs)] pub tfl_right_rx : & 'a mut Rx < pac :: UART7 >
        , #[allow(missing_docs)] pub tfl_right_parser : & 'a mut TfLunaParser,
        #[doc(hidden)] pub __rtic_internal_marker : :: core :: marker ::
        PhantomData < & 'a () > ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `uart7_rx` has access to"] pub struct
    __rtic_internal_uart7_rxSharedResources < 'a >
    {
        #[allow(missing_docs)] pub prox_right : shared_resources ::
        prox_right_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct __rtic_internal_uart7_rx_Context
    < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : uart7_rx :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        uart7_rx :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_uart7_rx_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_uart7_rx_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                uart7_rx :: LocalResources :: new(), shared : uart7_rx ::
                SharedResources :: new(),
            }
        }
    } #[allow(non_snake_case)] #[doc = "Hardware task"] pub mod uart7_rx
    {
        #[doc(inline)] pub use super :: __rtic_internal_uart7_rxLocalResources
        as LocalResources; #[doc(inline)] pub use super ::
        __rtic_internal_uart7_rxSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_uart7_rx_Context as
        Context;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `usart3_rx` has access to"] pub struct
    __rtic_internal_usart3_rxLocalResources < 'a >
    {
        #[allow(missing_docs)] pub esc_tx_rx : & 'a mut Rx < pac :: USART3 > ,
        #[allow(missing_docs)] pub esc_tx_parser : & 'a mut EscTelemParser,
        #[doc(hidden)] pub __rtic_internal_marker : :: core :: marker ::
        PhantomData < & 'a () > ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `usart3_rx` has access to"] pub struct
    __rtic_internal_usart3_rxSharedResources < 'a >
    {
        #[allow(missing_docs)] pub esc : shared_resources ::
        esc_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub
        esc_tlm : shared_resources :: esc_tlm_that_needs_to_be_locked < 'a > ,
        #[doc(hidden)] pub __rtic_internal_marker : core :: marker ::
        PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct
    __rtic_internal_usart3_rx_Context < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : usart3_rx :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        usart3_rx :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_usart3_rx_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_usart3_rx_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                usart3_rx :: LocalResources :: new(), shared : usart3_rx ::
                SharedResources :: new(),
            }
        }
    } #[allow(non_snake_case)] #[doc = "Hardware task"] pub mod usart3_rx
    {
        #[doc(inline)] pub use super ::
        __rtic_internal_usart3_rxLocalResources as LocalResources;
        #[doc(inline)] pub use super ::
        __rtic_internal_usart3_rxSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_usart3_rx_Context as
        Context;
    } #[allow(non_snake_case)] fn usart1_rx(mut cx : usart1_rx :: Context)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let parser
        = cx.local.gps_parser; let mut updated = false; while let Ok(byte) =
        cx.local.gps_rx.read() { if parser.push(byte) { updated = true; } } if
        updated
        { let data = parser.data(); cx.shared.gps.lock(| g | * g = data); }
    } #[allow(non_snake_case)] fn usart2_rx(mut cx : usart2_rx :: Context)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let parser
        = cx.local.mtf_parser; let mut updated = false; while let Ok(byte) =
        cx.local.mtf_rx.read() { if parser.push(byte) { updated = true; } } if
        updated
        { let data = parser.data(); cx.shared.flow.lock(| f | * f = data); }
    } #[allow(non_snake_case)] fn uart5_rx(mut cx : uart5_rx :: Context)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let parser
        = cx.local.crsf_parser; let mut updated = false; while let Ok(byte) =
        cx.local.crsf_rx.read() { if parser.push(byte) { updated = true; } }
        if updated
        { let data = parser.data(); cx.shared.rc.lock(| r | * r = data); }
    } #[allow(non_snake_case)] fn usart6_rx(mut cx : usart6_rx :: Context)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let parser
        = cx.local.tfl_left_parser; let mut updated = false; while let
        Ok(byte) = cx.local.tfl_left_rx.read()
        { if parser.push(byte) { updated = true; } } if updated
        {
            let data = parser.data();
            cx.shared.prox_left.lock(| p | * p = data);
        }
    } #[allow(non_snake_case)] fn uart7_rx(mut cx : uart7_rx :: Context)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let parser
        = cx.local.tfl_right_parser; let mut updated = false; while let
        Ok(byte) = cx.local.tfl_right_rx.read()
        { if parser.push(byte) { updated = true; } } if updated
        {
            let data = parser.data();
            cx.shared.prox_right.lock(| p | * p = data);
        }
    } #[allow(non_snake_case)] fn usart3_rx(mut cx : usart3_rx :: Context)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let parser
        = cx.local.esc_tx_parser; let poles =
        cx.shared.esc.lock(| e | e.config.pole_count); while let Ok(byte) =
        cx.local.esc_tx_rx.read()
        {
            if let Some(frame) = parser.push(byte)
            { cx.shared.esc_tlm.lock(| t | t.ingest(frame, poles)); }
        }
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
                att_that_needs_to_be_locked :: new(), mag : shared_resources
                :: mag_that_needs_to_be_locked :: new(), accel_w :
                shared_resources :: accel_w_that_needs_to_be_locked :: new(),
                rc : shared_resources :: rc_that_needs_to_be_locked :: new(),
                __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_dshot_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_dshot_taskSharedResources
            {
                esc : shared_resources :: esc_that_needs_to_be_locked ::
                new(), __rtic_internal_marker : core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_i2c_taskLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_i2c_taskLocalResources
            {
                i2c2 : & mut *
                (& mut *
                __rtic_internal_local_resource_i2c2.get_mut()).as_mut_ptr(),
                compass : & mut *
                (& mut *
                __rtic_internal_local_resource_compass.get_mut()).as_mut_ptr(),
                baro : & mut *
                (& mut *
                __rtic_internal_local_resource_baro.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_i2c_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_i2c_taskSharedResources
            {
                mag : shared_resources :: mag_that_needs_to_be_locked ::
                new(), baro : shared_resources :: baro_that_needs_to_be_locked
                :: new(), __rtic_internal_marker : core :: marker ::
                PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_nav_taskLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_nav_taskLocalResources
            {
                nav : & mut *
                (& mut *
                __rtic_internal_local_resource_nav.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_nav_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_nav_taskSharedResources
            {
                flow : shared_resources :: flow_that_needs_to_be_locked ::
                new(), att : shared_resources :: att_that_needs_to_be_locked
                :: new(), navs : shared_resources ::
                navs_that_needs_to_be_locked :: new(), __rtic_internal_marker
                : core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_ekf_taskLocalResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_ekf_taskLocalResources
            {
                ekf : & mut *
                (& mut *
                __rtic_internal_local_resource_ekf.get_mut()).as_mut_ptr(),
                __rtic_internal_marker : :: core :: marker :: PhantomData,
            }
        }
    } impl < 'a > __rtic_internal_ekf_taskSharedResources < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_ekf_taskSharedResources
            {
                accel_w : shared_resources :: accel_w_that_needs_to_be_locked
                :: new(), gps : shared_resources ::
                gps_that_needs_to_be_locked :: new(), baro : shared_resources
                :: baro_that_needs_to_be_locked :: new(), navs :
                shared_resources :: navs_that_needs_to_be_locked :: new(),
                navsol : shared_resources :: navsol_that_needs_to_be_locked ::
                new(), __rtic_internal_marker : core :: marker :: PhantomData,
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
                decoder : & mut *
                (& mut *
                __rtic_internal_local_resource_decoder.get_mut()).as_mut_ptr(),
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
                :: new(), gps : shared_resources ::
                gps_that_needs_to_be_locked :: new(), mag : shared_resources
                :: mag_that_needs_to_be_locked :: new(), att :
                shared_resources :: att_that_needs_to_be_locked :: new(), flow
                : shared_resources :: flow_that_needs_to_be_locked :: new(),
                rc : shared_resources :: rc_that_needs_to_be_locked :: new(),
                navs : shared_resources :: navs_that_needs_to_be_locked ::
                new(), baro : shared_resources :: baro_that_needs_to_be_locked
                :: new(), prox_left : shared_resources ::
                prox_left_that_needs_to_be_locked :: new(), prox_right :
                shared_resources :: prox_right_that_needs_to_be_locked ::
                new(), navsol : shared_resources ::
                navsol_that_needs_to_be_locked :: new(), esc :
                shared_resources :: esc_that_needs_to_be_locked :: new(),
                esc_tlm : shared_resources :: esc_tlm_that_needs_to_be_locked
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
        att_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub mag :
        shared_resources :: mag_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub accel_w : shared_resources ::
        accel_w_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub rc
        : shared_resources :: rc_that_needs_to_be_locked < 'a > ,
        #[doc(hidden)] pub __rtic_internal_marker : core :: marker ::
        PhantomData < & 'a () > ,
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
    #[doc = "Shared resources `dshot_task` has access to"] pub struct
    __rtic_internal_dshot_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub esc : shared_resources ::
        esc_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct
    __rtic_internal_dshot_task_Context < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Shared Resources this task has access to"] pub
        shared : dshot_task :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_dshot_task_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_dshot_task_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, shared :
                dshot_task :: SharedResources :: new(),
            }
        }
    } #[doc = r" Spawns the task directly"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_dshot_task_spawn() -> :: core ::
    result :: Result < (), () >
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(dshot_task, & __rtic_internal_dshot_task_EXEC); if
            exec.try_allocate()
            {
                exec.spawn(dshot_task(unsafe
                { dshot_task :: Context :: new() })); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1); Ok(())
            } else { Err(()) }
        }
    } #[doc = r" Gives waker to the task"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_dshot_task_waker() -> :: core ::
    task :: Waker
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(dshot_task, & __rtic_internal_dshot_task_EXEC);
            exec.waker(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(dshot_task, &
                __rtic_internal_dshot_task_EXEC); exec.set_pending(); rtic ::
                export :: pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1);
            })
        }
    } #[allow(non_snake_case)] #[doc = "Software task"] pub mod dshot_task
    {
        #[doc(inline)] pub use super ::
        __rtic_internal_dshot_taskSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_dshot_task_Context as
        Context; #[doc(inline)] pub use super ::
        __rtic_internal_dshot_task_spawn as spawn; #[doc(inline)] pub use
        super :: __rtic_internal_dshot_task_waker as waker;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `i2c_task` has access to"] pub struct
    __rtic_internal_i2c_taskLocalResources < 'a >
    {
        #[allow(missing_docs)] pub i2c2 : & 'a mut I2c2,
        #[allow(missing_docs)] pub compass : & 'a mut Compass,
        #[allow(missing_docs)] pub baro : & 'a mut Baro, #[doc(hidden)] pub
        __rtic_internal_marker : :: core :: marker :: PhantomData < & 'a () >
        ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `i2c_task` has access to"] pub struct
    __rtic_internal_i2c_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub mag : shared_resources ::
        mag_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub baro :
        shared_resources :: baro_that_needs_to_be_locked < 'a > ,
        #[doc(hidden)] pub __rtic_internal_marker : core :: marker ::
        PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct __rtic_internal_i2c_task_Context
    < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : i2c_task :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        i2c_task :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_i2c_task_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_i2c_task_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                i2c_task :: LocalResources :: new(), shared : i2c_task ::
                SharedResources :: new(),
            }
        }
    } #[doc = r" Spawns the task directly"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_i2c_task_spawn() -> :: core ::
    result :: Result < (), () >
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(i2c_task, & __rtic_internal_i2c_task_EXEC); if
            exec.try_allocate()
            {
                exec.spawn(i2c_task(unsafe { i2c_task :: Context :: new() }));
                rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1); Ok(())
            } else { Err(()) }
        }
    } #[doc = r" Gives waker to the task"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_i2c_task_waker() -> :: core :: task
    :: Waker
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(i2c_task, & __rtic_internal_i2c_task_EXEC);
            exec.waker(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(i2c_task, & __rtic_internal_i2c_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1);
            })
        }
    } #[allow(non_snake_case)] #[doc = "Software task"] pub mod i2c_task
    {
        #[doc(inline)] pub use super :: __rtic_internal_i2c_taskLocalResources
        as LocalResources; #[doc(inline)] pub use super ::
        __rtic_internal_i2c_taskSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_i2c_task_Context as
        Context; #[doc(inline)] pub use super ::
        __rtic_internal_i2c_task_spawn as spawn; #[doc(inline)] pub use super
        :: __rtic_internal_i2c_task_waker as waker;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `nav_task` has access to"] pub struct
    __rtic_internal_nav_taskLocalResources < 'a >
    {
        #[allow(missing_docs)] pub nav : & 'a mut Nav, #[doc(hidden)] pub
        __rtic_internal_marker : :: core :: marker :: PhantomData < & 'a () >
        ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `nav_task` has access to"] pub struct
    __rtic_internal_nav_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub flow : shared_resources ::
        flow_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub att :
        shared_resources :: att_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub navs : shared_resources ::
        navs_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct __rtic_internal_nav_task_Context
    < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : nav_task :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        nav_task :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_nav_task_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_nav_task_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                nav_task :: LocalResources :: new(), shared : nav_task ::
                SharedResources :: new(),
            }
        }
    } #[doc = r" Spawns the task directly"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_nav_task_spawn() -> :: core ::
    result :: Result < (), () >
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(nav_task, & __rtic_internal_nav_task_EXEC); if
            exec.try_allocate()
            {
                exec.spawn(nav_task(unsafe { nav_task :: Context :: new() }));
                rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1); Ok(())
            } else { Err(()) }
        }
    } #[doc = r" Gives waker to the task"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_nav_task_waker() -> :: core :: task
    :: Waker
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(nav_task, & __rtic_internal_nav_task_EXEC);
            exec.waker(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(nav_task, & __rtic_internal_nav_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1);
            })
        }
    } #[allow(non_snake_case)] #[doc = "Software task"] pub mod nav_task
    {
        #[doc(inline)] pub use super :: __rtic_internal_nav_taskLocalResources
        as LocalResources; #[doc(inline)] pub use super ::
        __rtic_internal_nav_taskSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_nav_task_Context as
        Context; #[doc(inline)] pub use super ::
        __rtic_internal_nav_task_spawn as spawn; #[doc(inline)] pub use super
        :: __rtic_internal_nav_task_waker as waker;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `ekf_task` has access to"] pub struct
    __rtic_internal_ekf_taskLocalResources < 'a >
    {
        #[allow(missing_docs)] pub ekf : & 'a mut Ekf, #[doc(hidden)] pub
        __rtic_internal_marker : :: core :: marker :: PhantomData < & 'a () >
        ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `ekf_task` has access to"] pub struct
    __rtic_internal_ekf_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub accel_w : shared_resources ::
        accel_w_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub
        gps : shared_resources :: gps_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub baro : shared_resources ::
        baro_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub navs
        : shared_resources :: navs_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub navsol : shared_resources ::
        navsol_that_needs_to_be_locked < 'a > , #[doc(hidden)] pub
        __rtic_internal_marker : core :: marker :: PhantomData < & 'a () > ,
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct __rtic_internal_ekf_task_Context
    < 'a >
    {
        #[doc(hidden)] __rtic_internal_p : :: core :: marker :: PhantomData <
        & 'a () > , #[doc = r" Local Resources this task has access to"] pub
        local : ekf_task :: LocalResources < 'a > ,
        #[doc = r" Shared Resources this task has access to"] pub shared :
        ekf_task :: SharedResources < 'a > ,
    } impl < 'a > __rtic_internal_ekf_task_Context < 'a >
    {
        #[inline(always)] #[allow(missing_docs)] pub unsafe fn new() -> Self
        {
            __rtic_internal_ekf_task_Context
            {
                __rtic_internal_p : :: core :: marker :: PhantomData, local :
                ekf_task :: LocalResources :: new(), shared : ekf_task ::
                SharedResources :: new(),
            }
        }
    } #[doc = r" Spawns the task directly"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_ekf_task_spawn() -> :: core ::
    result :: Result < (), () >
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(ekf_task, & __rtic_internal_ekf_task_EXEC); if
            exec.try_allocate()
            {
                exec.spawn(ekf_task(unsafe { ekf_task :: Context :: new() }));
                rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1); Ok(())
            } else { Err(()) }
        }
    } #[doc = r" Gives waker to the task"] #[allow(non_snake_case)]
    #[doc(hidden)] pub fn __rtic_internal_ekf_task_waker() -> :: core :: task
    :: Waker
    {
        unsafe
        {
            let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(ekf_task, & __rtic_internal_ekf_task_EXEC);
            exec.waker(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(ekf_task, & __rtic_internal_ekf_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1);
            })
        }
    } #[allow(non_snake_case)] #[doc = "Software task"] pub mod ekf_task
    {
        #[doc(inline)] pub use super :: __rtic_internal_ekf_taskLocalResources
        as LocalResources; #[doc(inline)] pub use super ::
        __rtic_internal_ekf_taskSharedResources as SharedResources;
        #[doc(inline)] pub use super :: __rtic_internal_ekf_task_Context as
        Context; #[doc(inline)] pub use super ::
        __rtic_internal_ekf_task_spawn as spawn; #[doc(inline)] pub use super
        :: __rtic_internal_ekf_task_waker as waker;
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Local resources `usb_task` has access to"] pub struct
    __rtic_internal_usb_taskLocalResources < 'a >
    {
        #[allow(missing_docs)] pub usb_dev : & 'a mut UsbDevice < 'static,
        MyUsbBus > , #[allow(missing_docs)] pub serial : & 'a mut usbd_serial
        :: SerialPort < 'static, MyUsbBus > , #[allow(missing_docs)] pub
        mavlink : & 'a mut Encoder, #[allow(missing_docs)] pub decoder : & 'a
        mut Decoder, #[doc(hidden)] pub __rtic_internal_marker : :: core ::
        marker :: PhantomData < & 'a () > ,
    } #[allow(non_snake_case)] #[allow(non_camel_case_types)]
    #[doc = "Shared resources `usb_task` has access to"] pub struct
    __rtic_internal_usb_taskSharedResources < 'a >
    {
        #[allow(missing_docs)] pub out1 : shared_resources ::
        out1_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub out2
        : shared_resources :: out2_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub gps : shared_resources ::
        gps_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub mag :
        shared_resources :: mag_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub att : shared_resources ::
        att_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub flow :
        shared_resources :: flow_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub rc : shared_resources ::
        rc_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub navs :
        shared_resources :: navs_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub baro : shared_resources ::
        baro_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub
        prox_left : shared_resources :: prox_left_that_needs_to_be_locked < 'a
        > , #[allow(missing_docs)] pub prox_right : shared_resources ::
        prox_right_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub
        navsol : shared_resources :: navsol_that_needs_to_be_locked < 'a > ,
        #[allow(missing_docs)] pub esc : shared_resources ::
        esc_that_needs_to_be_locked < 'a > , #[allow(missing_docs)] pub
        esc_tlm : shared_resources :: esc_tlm_that_needs_to_be_locked < 'a > ,
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
        { mut out1, mut out2, mut att, mut mag, mut accel_w, mut rc, .. } =
        cx.shared; let mut cal_active = false; loop
        {
            let ch = rc.lock(| r | r.ch_us(CAL_RC_CHANNEL)); if ch > 1700 && !
            cal_active
            { est.mag_cal_mut().start_collection(); cal_active = true; } else
            if ch < 1300 && cal_active
            { est.mag_cal_mut().finish_collection(); cal_active = false; } let
            o1 = out1.lock(| o | * o); let o2 = out2.lock(| o | * o); let m =
            mag.lock(| m | * m); let mag_field = m.healthy.then_some(m.field);
            let a = est.update(& o1, & o2, mag_field, DT);
            att.lock(| x | * x = a);
            accel_w.lock(| x | * x = est.accel_world()); Mono ::
            delay(1.millis()).await;
        }
    } #[allow(non_snake_case)] async fn dshot_task < 'a >
    (mut cx : dshot_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; loop
        {
            let now = Mono :: now().ticks() as u32; let
            (frames, proto, refresh) =
            cx.shared.esc.lock(| e |
            (e.frames(now), e.config.protocol, e.config.refresh_hz)); dshot ::
            send_frames(& frames, proto); let period_ms =
            (1000 / u32 :: from(refresh).max(1)).max(1); Mono ::
            delay(period_ms.millis()).await;
        }
    } #[allow(non_snake_case)] async fn i2c_task < 'a >
    (cx : i2c_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let i2c2 =
        cx.local.i2c2; let compass = cx.local.compass; let baro =
        cx.local.baro; let i2c_task :: SharedResources
        { mut mag, baro : mut baro_shared, .. } = cx.shared; let mut n : u32 =
        0; loop
        {
            let m = compass.read(i2c2); mag.lock(| x | * x = m); if n % 5 == 0
            { let b = baro.read(i2c2); baro_shared.lock(| x | * x = b); } n =
            n.wrapping_add(1); Mono :: delay(10.millis()).await;
        }
    } #[allow(non_snake_case)] async fn nav_task < 'a >
    (cx : nav_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let nav =
        cx.local.nav; let nav_task :: SharedResources
        { mut flow, mut att, mut navs, .. } = cx.shared; const NAV_DT : f32 =
        0.02; loop
        {
            let f = flow.lock(| f | * f); let a = att.lock(| a | * a);
            nav.update(& f, & a, NAV_DT); let s = nav.state();
            navs.lock(| n | * n = s); Mono :: delay(20.millis()).await;
        }
    } #[allow(non_snake_case)] async fn ekf_task < 'a >
    (cx : ekf_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let ekf =
        cx.local.ekf; let ekf_task :: SharedResources
        { mut accel_w, mut gps, mut baro, mut navs, mut navsol, .. } =
        cx.shared; const EKF_DT : f32 = 0.01; let mut last_gps_seq : u32 = 0;
        let mut tick : u32 = 0; loop
        {
            let aw = accel_w.lock(| x | * x); ekf.predict(aw, EKF_DT); let g =
            gps.lock(| g | * g); if g.fix_type >= 3 && g.sentences !=
            last_gps_seq
            {
                last_gps_seq = g.sentences; let lat = g.lat_e7 as f32 *
                1.0e-7; let lon = g.lon_e7 as f32 * 1.0e-7; let alt = g.alt_mm
                as f32 * 1.0e-3; if ! ekf.origin_set()
                { ekf.set_origin(lat, lon, alt); } let (n, e) =
                ekf.gps_to_local(lat, lon);
                ekf.fuse_gps_pos(n, e, g.eph as f32 / 100.0); if g.cog_cdeg !=
                u16 :: MAX && g.vel_cms > 0
                {
                    let cog = (g.cog_cdeg as f32 / 100.0) * DEG2RAD; let v =
                    g.vel_cms as f32 / 100.0;
                    ekf.fuse_gps_vel(v * libm :: cosf(cog), v * libm ::
                    sinf(cog));
                }
            } if tick % 10 == 0
            {
                let b = baro.lock(| b | * b); if b.healthy
                { ekf.fuse_baro(b.rel_altitude_m); }
            } if tick % 5 == 0
            {
                let nv = navs.lock(| n | * n); if nv.height_valid
                {
                    ekf.fuse_lidar(nv.height_m); if nv.flow_quality >= 30
                    {
                        ekf.fuse_flow_vel(nv.vx, nv.vy, nv.flow_quality as f32 /
                        255.0);
                    }
                }
            } navsol.lock(| x | * x = ekf.solution()); tick =
            tick.wrapping_add(1); Mono :: delay(10.millis()).await;
        }
    } #[allow(non_snake_case)] async fn usb_task < 'a >
    (cx : usb_task :: Context < 'a >)
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; let
        usb_dev = cx.local.usb_dev; let serial = cx.local.serial; let mavlink
        = cx.local.mavlink; let decoder = cx.local.decoder; let usb_task ::
        SharedResources
        {
            mut out1, mut out2, mut gps, mut mag, mut att, mut flow, mut rc,
            mut navs, mut baro, mut prox_left, mut prox_right, mut navsol, mut
            esc, mut esc_tlm, ..
        } = cx.shared; let mut tick : u32 = 0; let mut boot_announces_left :
        u8 = 12; let mut last_rx_diag_ms : u32 = 0; loop
        {
            usb_dev.poll(& mut [serial]);
            {
                let mut scratch = [0u8; 64]; if let Ok(n) =
                serial.read(& mut scratch)
                {
                    let now = Mono :: now().ticks() as u32; for & b in & scratch
                    [.. n]
                    {
                        if let Some(cmd) = decoder.push(b)
                        {
                            let is_set = matches! (cmd, Inbound::EscSet { .. }); let
                            is_motor_test = matches! (cmd, Inbound::MotorTest { .. });
                            let mut ack : heapless :: String < 50 > = heapless :: String
                            :: new(); match & cmd
                            {
                                Inbound :: MotorTest { motor, throttle, .. } =>
                                {
                                    let _ = write!
                                    (ack, "ESC: motor {} test {}%", motor, *throttle as i32);
                                } Inbound :: EscSet
                                { master_enabled, protocol, refresh_hz, .. } =>
                                {
                                    let _ = write!
                                    (ack, "ESC: set master={} proto={} hz={}", *master_enabled
                                    as u8, protocol, refresh_hz);
                                } Inbound :: EscCmd { target, command } =>
                                {
                                    let _ = write!
                                    (ack, "ESC: cmd {} -> tgt {}", command, target);
                                }
                            } let accepted = esc.lock(| e | apply_inbound(e, cmd, now));
                            let frame = mavlink.statustext(6, & ack);
                            pump_write(usb_dev, serial, frame.as_slice()); if ! accepted
                            {
                                let mut reject : heapless :: String < 50 > = heapless ::
                                String :: new(); let _ = write!
                                (reject, "ESC: reject {}", ack.as_str()); let frame =
                                mavlink.statustext(3, & reject);
                                pump_write(usb_dev, serial, frame.as_slice());
                            } if is_motor_test
                            {
                                let frame =
                                mavlink.command_ack(MAV_CMD_DO_MOTOR_TEST, if accepted { 0 }
                                else { 4 },); pump_write(usb_dev, serial, frame.as_slice());
                            } if is_set || is_motor_test
                            {
                                let c = esc.lock(| e | e.config); let cf =
                                mavlink.esc_config(c.cur_scale, c.cur_offset, c.refresh_hz,
                                c.protocol.as_u8(), c.master_enabled, c.bidir, c.dir_mask,
                                c.pole_count, c.mode3d_mask,);
                                pump_write(usb_dev, serial, cf.as_slice());
                            }
                        }
                    } if let Some(diag) = decoder.take_diag()
                    {
                        let elapsed = now.wrapping_sub(last_rx_diag_ms); if elapsed
                        >= 250
                        {
                            last_rx_diag_ms = now; let mut s : heapless :: String < 50 >
                            = heapless :: String :: new(); match diag
                            {
                                DecodeDiag :: Mavlink1 =>
                                { let _ = write! (s, "RX MAVLink1 unsupported"); }
                                DecodeDiag :: CommandLong { command } =>
                                { let _ = write! (s, "RX cmdlong {} ignored", command); }
                                DecodeDiag :: Unsupported { msgid } =>
                                { let _ = write! (s, "RX ignored msg {}", msgid); }
                                DecodeDiag :: CrcFail { msgid, got, expected } =>
                                {
                                    let _ = write!
                                    (s, "RX crc msg {} got {:04x} exp {:04x}", msgid, got,
                                    expected);
                                } DecodeDiag :: Oversize { msgid, len } =>
                                {
                                    let _ = write! (s, "RX oversize msg {} len {}", msgid, len);
                                }
                            } let frame = mavlink.statustext(4, & s);
                            pump_write(usb_dev, serial, frame.as_slice());
                        }
                    }
                }
            } if usb_dev.state() != usb_device :: device :: UsbDeviceState ::
            Configured
            {
                tick = tick.wrapping_add(1); Mono :: delay(1.millis()).await;
                continue;
            } if boot_announces_left > 0 && tick % 250 == 1
            {
                boot_announces_left -= 1; let frame =
                mavlink.statustext(6, FIRMWARE_TAG);
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 50 == 0
            {
                let o = out1.lock(| o | * o); if matches!
                (o.health, Health::Ok(_))
                {
                    let m = mag.lock(| m | * m); let mag_field =
                    m.healthy.then_some(m.field); let frame =
                    mavlink.highres_imu(tick as u64 * 1_000, 0, Rotation ::
                    Roll180.apply(o.accel), Rotation :: Roll180.apply(o.gyro),
                    mag_field,); pump_write(usb_dev, serial, frame.as_slice());
                }
            } if tick % 50 == 25
            {
                let o = out2.lock(| o | * o); if matches!
                (o.health, Health::Ok(_))
                {
                    let frame =
                    mavlink.highres_imu(tick as u64 * 1_000, 1, Rotation ::
                    Pitch180.apply(o.accel), Rotation :: Pitch180.apply(o.gyro),
                    None,); pump_write(usb_dev, serial, frame.as_slice());
                }
            } if tick % 40 == 30
            {
                let a = att.lock(| x | * x); let frame =
                mavlink.attitude(tick, a.roll * DEG2RAD, a.pitch * DEG2RAD,
                a.yaw * DEG2RAD, a.rates [0] * DEG2RAD, a.rates [1] * DEG2RAD,
                a.rates [2] * DEG2RAD,);
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 200 == 40
            {
                let g = gps.lock(| g | * g); let frame =
                mavlink.gps_raw_int(tick as u64 * 1_000, g.fix_type, g.lat_e7,
                g.lon_e7, g.alt_mm, g.eph, g.vel_cms, g.cog_cdeg, g.sats,);
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 200 == 60
            {
                let g = gps.lock(| g | * g); let sol = navsol.lock(| s | * s);
                let yaw_deg = att.lock(| x | x.yaw); let hdg =
                {
                    let mut h = yaw_deg; while h < 0.0 { h += 360.0; } while h
                    >= 360.0 { h -= 360.0; } (h * 100.0) as u16
                }; let (lat_e7, lon_e7, alt_mm, rel_alt_mm, vx, vy, vz) = if
                sol.converged
                {
                    (sol.lat_e7, sol.lon_e7, sol.alt_mm, sol.rel_alt_mm,
                    (sol.vel [0] * 100.0) as i16, (sol.vel [1] * 100.0) as i16,
                    (- sol.vel [2] * 100.0) as i16,)
                } else
                {
                    let (vx, vy) = if g.cog_cdeg != u16 :: MAX
                    {
                        let cog = (g.cog_cdeg as f32 / 100.0) * DEG2RAD; let v =
                        g.vel_cms as f32;
                        ((v * libm :: cosf(cog)) as i16, (v * libm :: sinf(cog)) as
                        i16)
                    } else { (0, 0) };
                    (g.lat_e7, g.lon_e7, g.alt_mm, 0, vx, vy, 0)
                }; let frame =
                mavlink.global_position_int(tick, lat_e7, lon_e7, alt_mm,
                rel_alt_mm, vx, vy, vz, hdg,);
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 100 == 50
            {
                let sol = navsol.lock(| s | * s); if sol.converged
                {
                    let frame =
                    mavlink.local_position_ned(tick, sol.pos [0], sol.pos [1], -
                    sol.pos [2], sol.vel [0], sol.vel [1], - sol.vel [2],);
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            } if tick % 100 == 70
            {
                let f = flow.lock(| f | * f); if f.dist_valid
                {
                    let cm = (f.dist_mm / 10).clamp(0, u16 :: MAX as i32) as
                    u16; let frame =
                    mavlink.distance_sensor(tick, 2, 800, cm, 25, 0);
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            } if tick % 66 == 33
            {
                let l = prox_left.lock(| p | * p); let r =
                prox_right.lock(| p | * p); let lcm = if l.valid
                { l.distance_cm } else { tfluna :: MAX_CM }; let rcm = if
                r.valid { r.distance_cm } else { tfluna :: MAX_CM }; let fl =
                mavlink.distance_sensor(tick, tfluna :: MIN_CM, tfluna ::
                MAX_CM, lcm, 6, 1);
                pump_write(usb_dev, serial, fl.as_slice()); let fr =
                mavlink.distance_sensor(tick, tfluna :: MIN_CM, tfluna ::
                MAX_CM, rcm, 2, 2);
                pump_write(usb_dev, serial, fr.as_slice());
            } if tick % 50 == 35
            {
                let f = flow.lock(| f | * f); let n = navs.lock(| n | * n); if
                f.flow_valid
                {
                    let frame =
                    mavlink.optical_flow(tick as u64 * 1_000,
                    (f.flow_x.clamp(i16 :: MIN as i32, i16 :: MAX as i32)) as
                    i16, (f.flow_y.clamp(i16 :: MIN as i32, i16 :: MAX as i32))
                    as i16, n.vx, n.vy, n.height_m, f.flow_quality,);
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            } if tick % 100 == 80
            {
                let r = rc.lock(| r | * r); let mut ch = [u16 :: MAX; 18]; for
                i in 0 .. 16 { ch [i] = r.ch_us(i); } let frame =
                mavlink.rc_channels(tick, 16, & ch, r.link_quality);
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 100 == 90
            {
                let b = baro.lock(| b | * b); if b.healthy
                {
                    let frame =
                    mavlink.scaled_pressure(tick, b.pressure_pa / 100.0, 0.0,
                    (b.temperature_c * 100.0) as i16,);
                    pump_write(usb_dev, serial, frame.as_slice());
                }
            } if tick % 100 == 45
            {
                let t = esc_tlm.lock(| t | * t); let frame =
                mavlink.esc_telem(t.mah as f32, t.total_current_a(), & t.rpm,
                & t.centivolt, & t.centiamp, & t.temp, & t.err,);
                pump_write(usb_dev, serial, frame.as_slice()); let frame =
                mavlink.esc_status(tick as u64 * 1_000, & t.rpm, &
                t.centivolt, & t.centiamp,);
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 1000 == 25
            {
                let c = esc.lock(| e | e.config); let frame =
                mavlink.esc_config(c.cur_scale, c.cur_offset, c.refresh_hz,
                c.protocol.as_u8(), c.master_enabled, c.bidir, c.dir_mask,
                c.pole_count, c.mode3d_mask,);
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 1000 == 30
            {
                let t = esc_tlm.lock(| t | * t); let frame =
                mavlink.esc_info(tick as u64 * 1_000, & t.temp, & t.err);
                pump_write(usb_dev, serial, frame.as_slice());
            } if tick % 500 == 125
            {
                let now = Mono :: now().ticks() as u32; let (master, active) =
                esc.lock(| e | (e.config.master_enabled, e.active_test(now)));
                let mut s : heapless :: String < 50 > = heapless :: String ::
                new(); if let Some((motor, value, remaining)) = active
                {
                    let _ = write!
                    (s, "ESC out master={} m{} dshot={} rem={}", master as u8,
                    motor, value, remaining);
                } else
                {
                    let _ = write! (s, "ESC out master={} idle", master as u8);
                } let frame = mavlink.statustext(6, & s);
                pump_write(usb_dev, serial, frame.as_slice());
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
            } if tick % 500 == 450
            {
                let l = prox_left.lock(| p | * p); let mut s : heapless ::
                String < 50 > = heapless :: String :: new(); let _ = write!
                (s, "L rx={} fr={} ck={} d={} a={}", l.rx_bytes, l.frames,
                l.checksum_errors, l.distance_cm, l.amplitude); let frame =
                mavlink.statustext(6, & s);
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
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic3"] static
    __rtic_internal_shared_resource_gps : rtic :: RacyCell < core :: mem ::
    MaybeUninit < GpsData >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: gps_that_needs_to_be_locked < 'a >
    {
        type T = GpsData; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut GpsData) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 4u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_gps.get_mut() as * mut _,
                CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic4"] static
    __rtic_internal_shared_resource_mag : rtic :: RacyCell < core :: mem ::
    MaybeUninit < MagData >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: mag_that_needs_to_be_locked < 'a >
    {
        type T = MagData; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut MagData) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 2u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_mag.get_mut() as * mut _,
                CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic5"] static
    __rtic_internal_shared_resource_baro : rtic :: RacyCell < core :: mem ::
    MaybeUninit < BaroData >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: baro_that_needs_to_be_locked < 'a >
    {
        type T = BaroData; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut BaroData) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 1u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_baro.get_mut() as * mut
                _, CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic6"] static
    __rtic_internal_shared_resource_flow : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Mtf01Data >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: flow_that_needs_to_be_locked < 'a >
    {
        type T = Mtf01Data; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut Mtf01Data) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 4u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_flow.get_mut() as * mut
                _, CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic7"] static
    __rtic_internal_shared_resource_rc : rtic :: RacyCell < core :: mem ::
    MaybeUninit < RcChannels >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: rc_that_needs_to_be_locked < 'a >
    {
        type T = RcChannels; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut RcChannels) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 4u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_rc.get_mut() as * mut _,
                CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic8"] static
    __rtic_internal_shared_resource_navs : rtic :: RacyCell < core :: mem ::
    MaybeUninit < NavState >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: navs_that_needs_to_be_locked < 'a >
    {
        type T = NavState; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut NavState) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 1u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_navs.get_mut() as * mut
                _, CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic9"] static
    __rtic_internal_shared_resource_prox_left : rtic :: RacyCell < core :: mem
    :: MaybeUninit < TfLunaData >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: prox_left_that_needs_to_be_locked < 'a >
    {
        type T = TfLunaData; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut TfLunaData) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 4u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_prox_left.get_mut() as *
                mut _, CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic10"] static
    __rtic_internal_shared_resource_prox_right : rtic :: RacyCell < core ::
    mem :: MaybeUninit < TfLunaData >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: prox_right_that_needs_to_be_locked < 'a >
    {
        type T = TfLunaData; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut TfLunaData) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 4u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_prox_right.get_mut() as *
                mut _, CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic11"] static
    __rtic_internal_shared_resource_accel_w : rtic :: RacyCell < core :: mem
    :: MaybeUninit < [f32; 3] >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: accel_w_that_needs_to_be_locked < 'a >
    {
        type T = [f32; 3]; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut [f32; 3]) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 2u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_accel_w.get_mut() as *
                mut _, CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic12"] static
    __rtic_internal_shared_resource_navsol : rtic :: RacyCell < core :: mem ::
    MaybeUninit < NavSolution >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: navsol_that_needs_to_be_locked < 'a >
    {
        type T = NavSolution; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut NavSolution) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 1u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_navsol.get_mut() as * mut
                _, CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic13"] static
    __rtic_internal_shared_resource_esc : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Esc >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: esc_that_needs_to_be_locked < 'a >
    {
        type T = Esc; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut Esc) -> RTIC_INTERNAL_R) ->
        RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 4u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_esc.get_mut() as * mut _,
                CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic14"] static
    __rtic_internal_shared_resource_esc_tlm : rtic :: RacyCell < core :: mem
    :: MaybeUninit < EscTelemetry >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit()); impl < 'a > rtic :: Mutex for
    shared_resources :: esc_tlm_that_needs_to_be_locked < 'a >
    {
        type T = EscTelemetry; #[inline(always)] fn lock < RTIC_INTERNAL_R >
        (& mut self, f : impl FnOnce(& mut EscTelemetry) -> RTIC_INTERNAL_R)
        -> RTIC_INTERNAL_R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 4u8; unsafe
            {
                rtic :: export ::
                lock(__rtic_internal_shared_resource_esc_tlm.get_mut() as *
                mut _, CEILING, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS, f,)
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
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        gps_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > gps_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                gps_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        mag_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > mag_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                mag_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        baro_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > baro_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                baro_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        flow_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > flow_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                flow_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        rc_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > rc_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                rc_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        navs_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > navs_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                navs_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        prox_left_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > prox_left_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                prox_left_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        prox_right_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > prox_right_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                prox_right_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        accel_w_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > accel_w_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                accel_w_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        navsol_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > navsol_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                navsol_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        esc_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > esc_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                esc_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        } #[doc(hidden)] #[allow(non_camel_case_types)] pub struct
        esc_tlm_that_needs_to_be_locked < 'a >
        { __rtic_internal_p : :: core :: marker :: PhantomData < & 'a () > , }
        impl < 'a > esc_tlm_that_needs_to_be_locked < 'a >
        {
            #[inline(always)] pub unsafe fn new() -> Self
            {
                esc_tlm_that_needs_to_be_locked
                { __rtic_internal_p : :: core :: marker :: PhantomData }
            }
        }
    } #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic15"] static
    __rtic_internal_local_resource_imu1 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Imu1 >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic16"] static
    __rtic_internal_local_resource_lpf1 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < ImuLpf >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic17"] static
    __rtic_internal_local_resource_imu2 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Imu2 >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic18"] static
    __rtic_internal_local_resource_lpf2 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < ImuLpf >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic19"] static
    __rtic_internal_local_resource_est : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Estimator >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic20"] static
    __rtic_internal_local_resource_usb_dev : rtic :: RacyCell < core :: mem ::
    MaybeUninit < UsbDevice < 'static, MyUsbBus > >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic21"] static
    __rtic_internal_local_resource_serial : rtic :: RacyCell < core :: mem ::
    MaybeUninit < usbd_serial :: SerialPort < 'static, MyUsbBus > >> = rtic ::
    RacyCell :: new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic22"] static
    __rtic_internal_local_resource_mavlink : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Encoder >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic23"] static
    __rtic_internal_local_resource_gps_rx : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Rx < pac :: USART1 > >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic24"] static
    __rtic_internal_local_resource_gps_parser : rtic :: RacyCell < core :: mem
    :: MaybeUninit < NmeaParser >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic25"] static
    __rtic_internal_local_resource_i2c2 : rtic :: RacyCell < core :: mem ::
    MaybeUninit < I2c2 >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic26"] static
    __rtic_internal_local_resource_compass : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Compass >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic27"] static
    __rtic_internal_local_resource_baro : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Baro >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic28"] static
    __rtic_internal_local_resource_mtf_rx : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Rx < pac :: USART2 > >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic29"] static
    __rtic_internal_local_resource_mtf_parser : rtic :: RacyCell < core :: mem
    :: MaybeUninit < MspParser >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic30"] static
    __rtic_internal_local_resource_crsf_rx : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Rx < pac :: UART5 > >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic31"] static
    __rtic_internal_local_resource_crsf_parser : rtic :: RacyCell < core ::
    mem :: MaybeUninit < CrsfParser >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic32"] static
    __rtic_internal_local_resource_nav : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Nav >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic33"] static
    __rtic_internal_local_resource_tfl_left_rx : rtic :: RacyCell < core ::
    mem :: MaybeUninit < Rx < pac :: USART6 > >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic34"] static
    __rtic_internal_local_resource_tfl_left_parser : rtic :: RacyCell < core
    :: mem :: MaybeUninit < TfLunaParser >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic35"] static
    __rtic_internal_local_resource_tfl_right_rx : rtic :: RacyCell < core ::
    mem :: MaybeUninit < Rx < pac :: UART7 > >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic36"] static
    __rtic_internal_local_resource_tfl_right_parser : rtic :: RacyCell < core
    :: mem :: MaybeUninit < TfLunaParser >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic37"] static
    __rtic_internal_local_resource_ekf : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Ekf >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic38"] static
    __rtic_internal_local_resource_esc_tx_rx : rtic :: RacyCell < core :: mem
    :: MaybeUninit < Rx < pac :: USART3 > >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic39"] static
    __rtic_internal_local_resource_esc_tx_parser : rtic :: RacyCell < core ::
    mem :: MaybeUninit < EscTelemParser >> = rtic :: RacyCell ::
    new(core :: mem :: MaybeUninit :: uninit());
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)]
    #[doc(hidden)] #[link_section = ".uninit.rtic40"] static
    __rtic_internal_local_resource_decoder : rtic :: RacyCell < core :: mem ::
    MaybeUninit < Decoder >> = rtic :: RacyCell ::
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
    __rtic_internal_dshot_task_EXEC : rtic :: export :: executor ::
    AsyncTaskExecutorPtr = rtic :: export :: executor :: AsyncTaskExecutorPtr
    :: new(); #[allow(non_upper_case_globals)] static
    __rtic_internal_i2c_task_EXEC : rtic :: export :: executor ::
    AsyncTaskExecutorPtr = rtic :: export :: executor :: AsyncTaskExecutorPtr
    :: new(); #[allow(non_upper_case_globals)] static
    __rtic_internal_nav_task_EXEC : rtic :: export :: executor ::
    AsyncTaskExecutorPtr = rtic :: export :: executor :: AsyncTaskExecutorPtr
    :: new(); #[allow(non_upper_case_globals)] static
    __rtic_internal_ekf_task_EXEC : rtic :: export :: executor ::
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
            from_ptr_1_args(dshot_task, & __rtic_internal_dshot_task_EXEC);
            exec.poll(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(dshot_task, &
                __rtic_internal_dshot_task_EXEC); exec.set_pending(); rtic ::
                export :: pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1);
            }); let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(ekf_task, & __rtic_internal_ekf_task_EXEC);
            exec.poll(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(ekf_task, & __rtic_internal_ekf_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1);
            }); let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(i2c_task, & __rtic_internal_i2c_task_EXEC);
            exec.poll(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(i2c_task, & __rtic_internal_i2c_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1);
            }); let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
            from_ptr_1_args(nav_task, & __rtic_internal_nav_task_EXEC);
            exec.poll(||
            {
                let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
                from_ptr_1_args(nav_task, & __rtic_internal_nav_task_EXEC);
                exec.set_pending(); rtic :: export ::
                pend(stm32h7xx_hal :: pac :: interrupt :: LPTIM1);
            }); let exec = rtic :: export :: executor :: AsyncTaskExecutor ::
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
        assert_send :: < Attitude > (); rtic :: export :: assert_send :: <
        GpsData > (); rtic :: export :: assert_send :: < MagData > (); rtic ::
        export :: assert_send :: < BaroData > (); rtic :: export ::
        assert_send :: < Mtf01Data > (); rtic :: export :: assert_send :: <
        RcChannels > (); rtic :: export :: assert_send :: < NavState > ();
        rtic :: export :: assert_send :: < TfLunaData > (); rtic :: export ::
        assert_send :: < [f32; 3] > (); rtic :: export :: assert_send :: <
        NavSolution > (); rtic :: export :: assert_send :: < Esc > (); rtic ::
        export :: assert_send :: < EscTelemetry > (); rtic :: export ::
        assert_send :: < Baro > (); rtic :: export :: interrupt :: disable();
        let mut core : rtic :: export :: Peripherals = rtic :: export ::
        Peripherals :: steal().into(); let _ =
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
        :: interrupt :: LPTIM3); const _ : () = if
        (1 << stm32h7xx_hal :: pac :: NVIC_PRIO_BITS) < 4u8 as usize
        {
            :: core :: panic!
            ("Maximum priority used by interrupt vector 'USART1' is more than supported by hardware");
        };
        core.NVIC.set_priority(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: USART1, rtic :: export ::
        cortex_logical2hw(4u8, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS),); rtic
        :: export :: NVIC ::
        unmask(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: USART1); const _ : () = if
        (1 << stm32h7xx_hal :: pac :: NVIC_PRIO_BITS) < 4u8 as usize
        {
            :: core :: panic!
            ("Maximum priority used by interrupt vector 'USART2' is more than supported by hardware");
        };
        core.NVIC.set_priority(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: USART2, rtic :: export ::
        cortex_logical2hw(4u8, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS),); rtic
        :: export :: NVIC ::
        unmask(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: USART2); const _ : () = if
        (1 << stm32h7xx_hal :: pac :: NVIC_PRIO_BITS) < 4u8 as usize
        {
            :: core :: panic!
            ("Maximum priority used by interrupt vector 'UART5' is more than supported by hardware");
        };
        core.NVIC.set_priority(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: UART5, rtic :: export ::
        cortex_logical2hw(4u8, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS),); rtic
        :: export :: NVIC ::
        unmask(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: UART5); const _ : () = if
        (1 << stm32h7xx_hal :: pac :: NVIC_PRIO_BITS) < 4u8 as usize
        {
            :: core :: panic!
            ("Maximum priority used by interrupt vector 'USART6' is more than supported by hardware");
        };
        core.NVIC.set_priority(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: USART6, rtic :: export ::
        cortex_logical2hw(4u8, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS),); rtic
        :: export :: NVIC ::
        unmask(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: USART6); const _ : () = if
        (1 << stm32h7xx_hal :: pac :: NVIC_PRIO_BITS) < 4u8 as usize
        {
            :: core :: panic!
            ("Maximum priority used by interrupt vector 'UART7' is more than supported by hardware");
        };
        core.NVIC.set_priority(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: UART7, rtic :: export ::
        cortex_logical2hw(4u8, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS),); rtic
        :: export :: NVIC ::
        unmask(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: UART7); const _ : () = if
        (1 << stm32h7xx_hal :: pac :: NVIC_PRIO_BITS) < 4u8 as usize
        {
            :: core :: panic!
            ("Maximum priority used by interrupt vector 'USART3' is more than supported by hardware");
        };
        core.NVIC.set_priority(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: USART3, rtic :: export ::
        cortex_logical2hw(4u8, stm32h7xx_hal :: pac :: NVIC_PRIO_BITS),); rtic
        :: export :: NVIC ::
        unmask(you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml
        :: interrupt :: USART3); #[inline(never)] fn __rtic_init_resources < F
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
        new_1_args(dshot_task)); executors_size += :: core :: mem ::
        size_of_val(& executor);
        __rtic_internal_dshot_task_EXEC.set_in_main(& executor); let executor
        = :: core :: mem :: ManuallyDrop ::
        new(rtic :: export :: executor :: AsyncTaskExecutor ::
        new_1_args(i2c_task)); executors_size += :: core :: mem ::
        size_of_val(& executor);
        __rtic_internal_i2c_task_EXEC.set_in_main(& executor); let executor =
        :: core :: mem :: ManuallyDrop ::
        new(rtic :: export :: executor :: AsyncTaskExecutor ::
        new_1_args(nav_task)); executors_size += :: core :: mem ::
        size_of_val(& executor);
        __rtic_internal_nav_task_EXEC.set_in_main(& executor); let executor =
        :: core :: mem :: ManuallyDrop ::
        new(rtic :: export :: executor :: AsyncTaskExecutor ::
        new_1_args(ekf_task)); executors_size += :: core :: mem ::
        size_of_val(& executor);
        __rtic_internal_ekf_task_EXEC.set_in_main(& executor); let executor =
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
            __rtic_internal_shared_resource_out1.get_mut().write(core :: mem
            :: MaybeUninit :: new(shared_resources.out1));
            __rtic_internal_shared_resource_out2.get_mut().write(core :: mem
            :: MaybeUninit :: new(shared_resources.out2));
            __rtic_internal_shared_resource_att.get_mut().write(core :: mem ::
            MaybeUninit :: new(shared_resources.att));
            __rtic_internal_shared_resource_gps.get_mut().write(core :: mem ::
            MaybeUninit :: new(shared_resources.gps));
            __rtic_internal_shared_resource_mag.get_mut().write(core :: mem ::
            MaybeUninit :: new(shared_resources.mag));
            __rtic_internal_shared_resource_baro.get_mut().write(core :: mem
            :: MaybeUninit :: new(shared_resources.baro));
            __rtic_internal_shared_resource_flow.get_mut().write(core :: mem
            :: MaybeUninit :: new(shared_resources.flow));
            __rtic_internal_shared_resource_rc.get_mut().write(core :: mem ::
            MaybeUninit :: new(shared_resources.rc));
            __rtic_internal_shared_resource_navs.get_mut().write(core :: mem
            :: MaybeUninit :: new(shared_resources.navs));
            __rtic_internal_shared_resource_prox_left.get_mut().write(core ::
            mem :: MaybeUninit :: new(shared_resources.prox_left));
            __rtic_internal_shared_resource_prox_right.get_mut().write(core ::
            mem :: MaybeUninit :: new(shared_resources.prox_right));
            __rtic_internal_shared_resource_accel_w.get_mut().write(core ::
            mem :: MaybeUninit :: new(shared_resources.accel_w));
            __rtic_internal_shared_resource_navsol.get_mut().write(core :: mem
            :: MaybeUninit :: new(shared_resources.navsol));
            __rtic_internal_shared_resource_esc.get_mut().write(core :: mem ::
            MaybeUninit :: new(shared_resources.esc));
            __rtic_internal_shared_resource_esc_tlm.get_mut().write(core ::
            mem :: MaybeUninit :: new(shared_resources.esc_tlm));
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
            :: MaybeUninit :: new(local_resources.mavlink));
            __rtic_internal_local_resource_gps_rx.get_mut().write(core :: mem
            :: MaybeUninit :: new(local_resources.gps_rx));
            __rtic_internal_local_resource_gps_parser.get_mut().write(core ::
            mem :: MaybeUninit :: new(local_resources.gps_parser));
            __rtic_internal_local_resource_i2c2.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.i2c2));
            __rtic_internal_local_resource_compass.get_mut().write(core :: mem
            :: MaybeUninit :: new(local_resources.compass));
            __rtic_internal_local_resource_baro.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.baro));
            __rtic_internal_local_resource_mtf_rx.get_mut().write(core :: mem
            :: MaybeUninit :: new(local_resources.mtf_rx));
            __rtic_internal_local_resource_mtf_parser.get_mut().write(core ::
            mem :: MaybeUninit :: new(local_resources.mtf_parser));
            __rtic_internal_local_resource_crsf_rx.get_mut().write(core :: mem
            :: MaybeUninit :: new(local_resources.crsf_rx));
            __rtic_internal_local_resource_crsf_parser.get_mut().write(core ::
            mem :: MaybeUninit :: new(local_resources.crsf_parser));
            __rtic_internal_local_resource_nav.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.nav));
            __rtic_internal_local_resource_tfl_left_rx.get_mut().write(core ::
            mem :: MaybeUninit :: new(local_resources.tfl_left_rx));
            __rtic_internal_local_resource_tfl_left_parser.get_mut().write(core
            :: mem :: MaybeUninit :: new(local_resources.tfl_left_parser));
            __rtic_internal_local_resource_tfl_right_rx.get_mut().write(core
            :: mem :: MaybeUninit :: new(local_resources.tfl_right_rx));
            __rtic_internal_local_resource_tfl_right_parser.get_mut().write(core
            :: mem :: MaybeUninit :: new(local_resources.tfl_right_parser));
            __rtic_internal_local_resource_ekf.get_mut().write(core :: mem ::
            MaybeUninit :: new(local_resources.ekf));
            __rtic_internal_local_resource_esc_tx_rx.get_mut().write(core ::
            mem :: MaybeUninit :: new(local_resources.esc_tx_rx));
            __rtic_internal_local_resource_esc_tx_parser.get_mut().write(core
            :: mem :: MaybeUninit :: new(local_resources.esc_tx_parser));
            __rtic_internal_local_resource_decoder.get_mut().write(core :: mem
            :: MaybeUninit :: new(local_resources.decoder)); rtic :: export ::
            interrupt :: enable();
        }); loop {}
    }
}