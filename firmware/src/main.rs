//! Alakol

#![no_main]
#![no_std]
#![feature(integer_atomics)]
#![feature(lang_items)]
#![allow(dead_code)]
#![allow(unused_macros)]

extern crate cortex_m_rt as rt;
use core::mem;
use core::sync::atomic::{AtomicI32, AtomicU16, Ordering};
use rt::{entry, exception};

extern crate alloc;
extern crate alloc_cortex_m;
extern crate cortex_m;
extern crate panic_itm;
extern crate stm32h7_ethernet as ethernet;

#[macro_use(block)]
extern crate nb;

#[macro_use]
extern crate log;

use alloc_cortex_m::CortexMHeap;
use stm32h7xx_hal::hal::digital::v1_compat::OldOutputPin;
use stm32h7xx_hal::hal::digital::v2::OutputPin;
use stm32h7xx_hal::rcc::CoreClocks;
use stm32h7xx_hal::{adc, prelude::*, spi, stm32, stm32::interrupt};

use ssd1306::prelude::*;
use ssd1306::Builder;

use ms5611_spi::Ms5611;

use cortex_m_log::log::{trick_init, Logger};
use cortex_m_log::{
    destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
};
use cortex_m_log::{print, println};

/// Pull in build information (from `built` crate)
mod build_info {
    #![allow(dead_code)]
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

// modules
mod config;
mod delay;
mod display;
mod flash;
mod http;
mod io;
mod omron_m2;
mod sensors;
mod server;
mod time;

fn rcc_reset(rcc: &stm32::RCC) {
    // Reset all peripherals
    rcc.ahb1rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.ahb1rstr.write(|w| unsafe { w.bits(0) });
    rcc.apb1lrstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb1lrstr.write(|w| unsafe { w.bits(0) });
    rcc.apb1hrstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb1hrstr.write(|w| unsafe { w.bits(0) });

    rcc.ahb2rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.ahb2rstr.write(|w| unsafe { w.bits(0) });
    rcc.apb2rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb2rstr.write(|w| unsafe { w.bits(0) });

    // do not reset the cpu
    rcc.ahb3rstr.write(|w| unsafe { w.bits(0x7FFF_FFFF) });
    rcc.ahb3rstr.write(|w| unsafe { w.bits(0) });
    rcc.apb3rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb3rstr.write(|w| unsafe { w.bits(0) });

    rcc.ahb4rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.ahb4rstr.write(|w| unsafe { w.bits(0) });
    rcc.apb4rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb4rstr.write(|w| unsafe { w.bits(0) });
}

/// Configure SYSTICK for 1ms timebase
fn systick_init(syst: &mut stm32::SYST, clocks: CoreClocks) {
    let c_ck_mhz = clocks.c_ck().0 / 1_000_000;

    let syst_calib = 0x3E8;

    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((syst_calib * c_ck_mhz) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
}

/// Set Boot address option bytes to 0x0800_0000 for any value of BOOT
fn flash_boot_init(flash: &stm32::FLASH) {
    // Option byte modification sequence - see RM Section 3.4.3
    flash
        .optkeyr
        .write(|w| unsafe { w.optkeyr().bits(0x0819_2A3B) });
    flash
        .optkeyr
        .write(|w| unsafe { w.optkeyr().bits(0x4C5D_6E7F) });

    flash.optcr.modify(|_, w| w.optlock().clear_bit());

    // Write new option byte values
    flash.boot_prgr.write(|w| unsafe {
        w.boot_add1().bits(0x0800).boot_add0().bits(0x0800)
    });

    // Set OPTSTART to 1
    flash.optcr.modify(|_, w| w.optstart().set_bit());

    // Wait until OPT_BUSY is cleared
    while flash.optsr_cur.read().opt_busy().bit_is_set() {}

    flash.optcr.modify(|_, w| w.optlock().set_bit());
}

/// ======================================================================
/// Entry point
/// ======================================================================

const ORDER: Ordering = Ordering::Relaxed;

static mut TIME: i64 = 0;
static PROC: AtomicI32 = AtomicI32::new(0);

static VALVE_STATE: AtomicU16 = AtomicU16::new(0);
static MOTOR_SPEED: AtomicU16 = AtomicU16::new(0);
static GAUGE_CALIBRATION_PEND: AtomicU16 = AtomicU16::new(0);

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

/// Ethernet descriptor rings are a global singleton
#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing = ethernet::DesRing::new();

#[link_section = ".axisram.server"]
static mut DS_STORE: server::DataServerStorageStatic =
    server::DataServerStorageStatic::new();

#[link_section = ".axisram.heap"]
static mut HEAP: [u32; 0x10000] = [0; 0x10000];

// the program entry point
#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = stm32::CorePeripherals::take().unwrap();
    let mut log = InterruptSyncItm::new(Itm::new(cp.ITM));

    // ARMv7-M DEMCR: Set TRCENA. Enabled DWT and ITM units
    unsafe { *(0xE000_EDFC as *mut u32) |= 1 << 24 };

    rcc_reset(&dp.RCC);
    dp.RCC.ahb2enr.modify(|_, w| {
        w.sram1en()
            .set_bit()
            .sram2en()
            .set_bit()
            .sram3en()
            .set_bit()
    });

    // Ensure debug blocks are clocked before interacting with them
    dp.DBGMCU.cr.write(|w| {
        w.dbgsleep_d1()
            .set_bit()
            .dbgstby_d3()
            .set_bit()
            .d1dbgcken()
            .set_bit()
            .d3dbgcken()
            .set_bit()
            .traceclken()
            .set_bit()
    });

    // SWO: Unlock
    unsafe { *(0x5c00_3fb0 as *mut u32) = 0xC5AC_CE55 };
    // SWO CODR Register: Increase SWO speed
    unsafe { *(0x5c00_3010 as *mut _) = 500 - 1 };

    // ITM Trace Enable Register: Enable lower 8 stimulus ports
    unsafe { *(0xE000_0E00 as *mut _) = 0xFF };
    // ITM Trace Control Register: Enable ITM
    unsafe { *(0xE000_0E80 as *mut _) = 1 };

    // Global Allocator - After SRAM Enabled
    unsafe {
        ALLOCATOR.init(
            &HEAP as *const _ as usize,
            HEAP.len() * mem::size_of::<u32>(),
        )
    }

    println!(log, "");
    println!(log, "+++++++++++++++++++++ ALAKOL +++++++++++++++++++++");
    println!(
        log,
        " Version {} {}",
        build_info::PKG_VERSION,
        build_info::GIT_VERSION.unwrap()
    );
    println!(log, " Platform {}", build_info::TARGET);
    println!(log, " Built on {}", build_info::BUILT_TIME_UTC);
    println!(log, " {}", build_info::RUSTC_VERSION);
    println!(log, "+++++++++++++++++++++++++++++++++++++++++++++++++\n");

    print!(log, " Initialising power...               ");
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    println!(log, "OK");

    print!(log, " Initialising clocks...               ");
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc
        .use_hse(38_400.khz())
        .sys_ck(200.mhz())
        .hclk(200.mhz())
        .per_ck(4.mhz()) // CSI for ADC[123]
        .pll1_q_ck(100.mhz()) // for SPI[123]
        .pll1_r_ck(100.mhz()) // for TRACECK
        .freeze(vos, &dp.SYSCFG);

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    let timer2 = dp.TIM2.timer(1.hz(), &mut ccdr);
    let mut delay2 = delay::DelayFromTimer::new(timer2);

    println!(log, "OK");

    // ----------------------------------------------------------
    print!(log, " Initialising system...               ");
    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    // TODO: ETH DMA coherence issues
    // cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter();
    println!(log, "OK");

    // Flash storage
    let mut flash_store = flash::FlashStorage::new(flash::Flash::new(dp.FLASH));

    // ----------------------------------------------------------
    print!(log, " Initialising IO...                   ");

    // Acquire GPIO peripherals. This also enables the clock for the
    // GPIO in the RCC register.
    let gpioa = dp.GPIOA.split(&mut ccdr.ahb4);
    let gpiob = dp.GPIOB.split(&mut ccdr.ahb4);
    let gpioc = dp.GPIOC.split(&mut ccdr.ahb4);
    let gpiod = dp.GPIOD.split(&mut ccdr.ahb4);
    let gpioe = dp.GPIOE.split(&mut ccdr.ahb4);

    let (
        hw,
        mut ethernet_led,
        _stat_led,
        mut act_led,
        oled_spi_pins,
        oled_dc,
        mut oled_rst,
        mut oled_cs,
        pressure_spi_pins,
        pressure_cs,
        v_psense,
        v_tsense,
        pwr_mon,
        valve_io,
        motor_io,
    ) = io::gpio_setup(gpioa, gpiob, gpioc, gpiod, gpioe);

    println!(log, "Hardware version {}...OK", hw);
    #[cfg(feature = "hardware0")]
    assert_eq!(hw, 0, "This FW was compiled for hardware version 0!");

    // ----------------------------------------------------------
    print!(log, " Initialising Motor...                ");

    // Configure PWM at 10kHz
    let motor_pwm = dp.TIM3.pwm(motor_io, 10.khz(), &mut ccdr);

    let mut omron = omron_m2::OmronM2::new(motor_pwm, valve_io);

    println!(log, "OK");

    // ----------------------------------------------------------
    print!(log, " Initialising Ethernet...             ");
    assert_eq!(ccdr.clocks.hclk().0, 200_000_000); // HCLK 200MHz
    assert_eq!(ccdr.clocks.pclk1().0, 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk2().0, 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk4().0, 100_000_000); // PCLK 100MHz

    let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(
        &config::DEFAULT_CONFIG.mac_address,
    );
    let (eth_dma, mut eth_mac) = unsafe {
        ethernet::ethernet_init(
            dp.ETHERNET_MAC,
            dp.ETHERNET_MTL,
            dp.ETHERNET_DMA,
            &mut DES_RING,
            mac_addr,
        )
    };
    unsafe {
        ethernet::enable_interrupt();
        cp.NVIC.set_priority(stm32::Interrupt::ETH, 196); // Mid prio
        cortex_m::peripheral::NVIC::unmask(stm32::Interrupt::ETH);
    }

    // HTTP Server
    let http = http::HttpServer::new(
        // Valve
        |state| VALVE_STATE.store(state as u16, ORDER),
        // Motor Speed
        |motor| MOTOR_SPEED.store(motor, ORDER),
    );

    let mut dstore = server::DataServerStorage::new();
    let mut data_server = unsafe {
        server::DataServer::new(
            &mut DS_STORE,
            &mut dstore,
            eth_dma,
            mac_addr,
            http,
            0,
        )
    };

    println!(log, "OK");

    // ----------------------------------------------------------
    print!(log, " Initialising SPI...                  ");

    // Initialise the SPI peripheral.
    let oled_spi = dp.SPI3.spi(oled_spi_pins, spi::MODE_0, 8.mhz(), &ccdr);
    oled_cs.set_low().unwrap();

    // Initialise the SPI peripheral.
    let pressure_spi =
        dp.SPI1.spi(pressure_spi_pins, spi::MODE_3, 10.mhz(), &ccdr);

    println!(log, "OK");

    // ----------------------------------------------------------
    print!(log, " Initialising ADCs...                 ");

    // Setup ADC1
    let mut adc1 = dp.ADC1.adc(&mut delay, &mut ccdr);
    adc1.set_resolution(adc::Resolution::SIXTEENBIT);

    // Setup ADC3
    let mut adc3 = dp.ADC3.adc(&mut delay, &mut ccdr);
    adc3.set_resolution(adc::Resolution::SIXTEENBIT);

    // Setup Temperature Sensor
    let mut internal_temperature = adc::Temperature::new();
    internal_temperature.enable(&adc3);
    delay.delay_us(25_u16);

    println!(log, "OK");

    // ----------------------------------------------------------
    print!(log, " Initialising Sensors...              ");

    let pressure_cs_old: OldOutputPin<_> = pressure_cs.into();
    let ms5611 =
        Ms5611::new(pressure_spi, pressure_cs_old, &mut delay2).unwrap();

    let mut sensors = sensors::Sensors::new(
        adc1.enable(),
        adc3.enable(),
        v_psense,
        v_tsense,
        pwr_mon,
        internal_temperature,
        ms5611,
    );
    sensors
        .set_gauge_pressure_calibration(flash_store.data.gauge_calibration_mv);

    println!(log, "OK");

    // ----------------------------------------------------------
    print!(log, " Initialising Display...              ");

    let mut disp: GraphicsMode<_> =
        Builder::new().connect_spi(oled_spi, oled_dc).into();

    disp.reset(&mut oled_rst, &mut delay).unwrap();
    disp.init().unwrap();
    disp.flush().unwrap();

    let (x, y) = disp.get_dimensions();
    print!(log, "{} x {}      ", x, y);

    // Display controls
    fn speed_adjust(incr: i16) {
        let speed = MOTOR_SPEED.load(ORDER);
        let speed = match speed as i16 + incr {
            x if x < 0 => 0,
            x @ 0..=100 => x,
            _ => 100,
        } as u16;
        MOTOR_SPEED.store(speed, ORDER);
    }
    let gauge_calibration = || GAUGE_CALIBRATION_PEND.store(1, ORDER);
    let motor_slow = || speed_adjust(-10);
    let motor_fast = || speed_adjust(10);
    let valve_close =
        || VALVE_STATE.store(omron_m2::ValveState::Closed as u16, ORDER);
    let valve_open =
        || VALVE_STATE.store(omron_m2::ValveState::Open as u16, ORDER);

    // Create Display
    let mut display = display::Oled::new(
        disp,
        gauge_calibration,
        motor_slow,
        motor_fast,
        valve_close,
        valve_open,
    );
    display.version(build_info::PKG_VERSION);
    display.build(build_info::GIT_VERSION.unwrap());

    println!(log, "OK");

    // ----------------------------------------------------------
    // Begin periodic tasks

    systick_init(&mut delay.free(), ccdr.clocks);
    unsafe {
        cp.SCB.shpr[15 - 4].write(128);
    } // systick exception priority

    // ----------------------------------------------------------
    // Main application loop
    println!(log, "Done!");
    println!(log, "");

    let logger = Logger {
        inner: log,
        level: log::LevelFilter::Trace,
    };

    unsafe {
        let _ = trick_init(&logger);
    }

    let mut a = 0i32;
    let mut eth_up = false;
    let mut last_ui_time: i64 = 0;
    loop {
        let time = cortex_m::interrupt::free(|_| unsafe { TIME });

        act_led.set_high().unwrap();

        // Buttons
        if io::get_button(io::Buttons::Left) != 0 {
            // LEFT
            display.click_screen(io::Buttons::Left);
            PROC.store(0, ORDER); // Force update
            last_ui_time = time;
        }
        if io::get_button(io::Buttons::Right) != 0 {
            // RIGHT
            display.click_screen(io::Buttons::Right);
            PROC.store(0, ORDER); // Force update
            last_ui_time = time;
        }

        // Quadrature encoder
        a = match io::get_quadrature_encoder() {
            io::QuadratureEncoderEvent::Increment => {
                last_ui_time = time;
                a + 1
            }
            io::QuadratureEncoderEvent::Decrement => {
                last_ui_time = time;
                a - 1
            }
            _ => a,
        };
        a = display.set_cursor_line(a);

        // Ethernet
        let eth_last = eth_up;
        eth_up = eth_mac.phy_poll_link();
        io::ethernet_led(
            &mut ethernet_led,
            if eth_up {
                io::EthernetLED::Green
            } else {
                io::EthernetLED::Off
            },
        )
        .unwrap();
        if eth_up != eth_last {
            // Interface state change
            data_server.interface_updown(time as i64)
        }

        // Calibrate?
        if GAUGE_CALIBRATION_PEND.load(ORDER) > 0 {
            // Do calibration
            let cal_value = sensors.calibrate_gauge_pressure();

            // Write to flash
            flash_store.data.gauge_calibration_mv = cal_value;
            flash_store.store();

            GAUGE_CALIBRATION_PEND.store(0, ORDER); // Next service 100ms
        }

        // Read Gauge Pressure
        let _ = sensors.read_gauge_pressure_mv();

        // Update outputs
        omron.set_valve(VALVE_STATE.load(ORDER).into());
        omron.set_motor_speed(MOTOR_SPEED.load(ORDER));

        // Slow data routine
        let sensor_data = if PROC.load(ORDER) <= 0 {
            while PROC.load(ORDER) <= 0 {
                PROC.store(100, ORDER); // Next service 100ms
            }

            // Read MS5611
            sensors.read_ms5611(&mut delay2);
            let data = sensors.collect_data();

            // Update measurement screen
            display.cpu_temperature(data.internal_temperature);
            display.k_temperature(data.thermocouple_temperature);
            display.sensor_temperature(data.ms5611_temperature_c);
            display.pressure(data.ms5611_pressure_kpa);
            display.gauge(data.gauge_pressure);

            // Update ethernet screen
            display.ip(data_server.ipv4_address());
            display.gateway(data_server.router());

            // Update motors screen
            display.motor_pwm(omron.motor_speed());
            display.valve(omron.valve_state());
            display.supply_voltage(data.supply_voltage);

            // Update time screen
            display.time(time::SimpleTime(time));

            // 2 hour timeout
            let display_saver = time > (last_ui_time + 3_600_000);

            // Draw
            display.draw_screen(display_saver);

            Some(data)
        } else {
            None
        };

        act_led.set_low().unwrap();

        if eth_up {
            data_server.work(time as i64, sensor_data);
        }

        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn ETH() {
    unsafe { ethernet::interrupt_handler() }
}

#[exception]
fn SysTick() {
    let time = unsafe { &mut TIME };
    *time += 1;

    PROC.fetch_sub(1, Ordering::Relaxed);
    io::debounce_tick(false);
}

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}

#[lang = "oom"]
#[no_mangle]
// The out of memory handler
pub fn rust_oom(_: core::alloc::Layout) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
