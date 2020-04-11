//! Input and output routines

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};
use stm32h7xx_hal::gpio::{
    Alternate, Analog, Input, Output, PullUp, PushPull, Speed, AF2, AF5, AF6,
};
use stm32h7xx_hal::hal::digital::v2::{InputPin, OutputPin};
use stm32h7xx_hal::Never;
use stm32h7xx_hal::{gpio, spi, stm32};

#[cfg(feature = "hardware0")]
pub type PressureSPIPinsT = (
    gpio::gpiob::PB3<Alternate<AF5>>,
    gpio::gpiob::PB4<Alternate<AF5>>,
    gpio::gpiob::PB5<Alternate<AF5>>,
);
#[cfg(not(feature = "hardware0"))]
pub type PressureSPIPinsT = (
    gpio::gpioa::PA5<Alternate<AF5>>,
    gpio::gpiob::PB4<Alternate<AF5>>,
    gpio::gpiob::PB5<Alternate<AF5>>,
);

pub fn gpio_setup(
    gpioa: gpio::gpioa::Parts,
    gpiob: gpio::gpiob::Parts,
    gpioc: gpio::gpioc::Parts,
    gpiod: gpio::gpiod::Parts,
    gpioe: gpio::gpioe::Parts,
) -> (
    u8,
    (
        // ethernet led
        gpio::gpioe::PE2<Output<PushPull>>,
        gpio::gpioe::PE3<Output<PushPull>>,
    ),
    gpio::gpioe::PE0<Output<PushPull>>, // stat_led
    gpio::gpioe::PE1<Output<PushPull>>, // act_led
    (
        // oled
        gpio::gpioc::PC10<Alternate<AF6>>,
        spi::NoMiso,
        gpio::gpioc::PC12<Alternate<AF6>>,
    ),
    gpio::gpiod::PD0<Output<PushPull>>,
    gpio::gpioc::PC11<Output<PushPull>>,
    gpio::gpioa::PA15<Output<PushPull>>,
    PressureSPIPinsT,
    gpio::gpioa::PA4<Output<PushPull>>, // pressure_cs
    gpio::gpioa::PA0<Analog>,
    gpio::gpioa::PA3<Analog>,
    gpio::gpioa::PA6<Analog>,
    gpio::gpiob::PB0<Output<PushPull>>, // valve_io
    gpio::gpiob::PB1<Alternate<AF2>>,   // motor_io
) {
    // Hardware version
    let h0 = gpioe.pe12.into_pull_up_input();
    let h1 = gpioe.pe13.into_pull_up_input();
    let h2 = gpioe.pe14.into_pull_up_input();
    let h3 = gpioe.pe15.into_pull_up_input();
    let hw = hardware_version(h0, h1, h2, h3).expect("Unknown version");

    // Stat LED
    let stat_led = gpioe.pe0.into_push_pull_output();
    // Act LED
    let act_led = gpioe.pe1.into_push_pull_output();

    // Ethernet LED
    let ethernet_led = (
        gpioe.pe2.into_push_pull_output(),
        gpioe.pe3.into_push_pull_output(),
    );

    // SW
    gpiob.pb6.into_pull_up_input();
    gpiob.pb7.into_pull_up_input();
    gpiob.pb8.into_pull_up_input();
    gpiob.pb9.into_pull_up_input();
    debounce_tick(true);

    // PB3 TRACESWO
    #[cfg(not(feature = "hardware0"))]
    let _traceswo = gpiob.pb3.into_alternate_af0().set_speed(Speed::VeryHigh);

    // Ethernet
    let rmii_ref_clk = gpioa.pa1.into_alternate_af11();
    rmii_ref_clk.set_speed(Speed::VeryHigh);
    let rmii_mdio = gpioa.pa2.into_alternate_af11();
    rmii_mdio.set_speed(Speed::VeryHigh);
    let rmii_mdc = gpioc.pc1.into_alternate_af11();
    rmii_mdc.set_speed(Speed::VeryHigh);
    let rmii_crs_dv = gpioa.pa7.into_alternate_af11();
    rmii_crs_dv.set_speed(Speed::VeryHigh);
    let rmii_rxd0 = gpioc.pc4.into_alternate_af11();
    rmii_rxd0.set_speed(Speed::VeryHigh);
    let rmii_rxd1 = gpioc.pc5.into_alternate_af11();
    rmii_rxd1.set_speed(Speed::VeryHigh);
    let rmii_tx_en = gpiob.pb11.into_alternate_af11();
    rmii_tx_en.set_speed(Speed::VeryHigh);
    let rxii_txd0 = gpiob.pb12.into_alternate_af11();
    rxii_txd0.set_speed(Speed::VeryHigh);
    let rmii_txd1 = gpiob.pb13.into_alternate_af11();
    rmii_txd1.set_speed(Speed::VeryHigh);

    // OLED
    let oled_sck = gpioc.pc10.into_alternate_af6();
    let oled_miso = spi::NoMiso;
    let oled_mosi = gpioc.pc12.into_alternate_af6();
    let oled_spi = (oled_sck, oled_miso, oled_mosi);
    let oled_dc = gpiod.pd0.into_push_pull_output();
    let oled_rst = gpioc.pc11.into_push_pull_output();
    let oled_cs = gpioa.pa15.into_push_pull_output();

    // Pressure sensor
    #[cfg(feature = "hardware0")]
    let pressure_sck = gpiob.pb3.into_alternate_af5();
    #[cfg(not(feature = "hardware0"))]
    let pressure_sck = gpioa.pa5.into_alternate_af5();
    let pressure_miso = gpiob.pb4.into_alternate_af5();
    let pressure_mosi = gpiob.pb5.into_alternate_af5();
    let pressure_spi = (pressure_sck, pressure_miso, pressure_mosi);
    let pressure_cs = gpioa.pa4.into_push_pull_output();

    // ADCs
    let v_psense = gpioa.pa0.into_analog();
    let v_tsense = gpioa.pa3.into_analog();
    let pwr_mon = gpioa.pa6.into_analog();

    // Motors
    let valve_io = gpiob.pb0.into_push_pull_output();
    let motor_io = gpiob.pb1.into_alternate_af2();

    // Hardware
    (
        hw,
        ethernet_led,
        stat_led,
        act_led,
        oled_spi,
        oled_dc,
        oled_rst,
        oled_cs,
        pressure_spi,
        pressure_cs,
        v_psense,
        v_tsense,
        pwr_mon,
        valve_io,
        motor_io,
    )
}

pub fn hardware_version(
    h0: gpio::gpioe::PE12<Input<PullUp>>,
    h1: gpio::gpioe::PE13<Input<PullUp>>,
    h2: gpio::gpioe::PE14<Input<PullUp>>,
    h3: gpio::gpioe::PE15<Input<PullUp>>,
) -> Result<u8, &'static str> {
    match (
        h0.is_low().unwrap(),
        h1.is_low().unwrap(),
        h2.is_low().unwrap(),
        h3.is_low().unwrap(),
    ) {
        (false, false, false, false) => Ok(0),
        (true, false, false, false) => Ok(1),
        _ => Err("Unknown"),
    }
}

pub enum EthernetLED {
    Off,
    Orange,
    Green,
}
/// Set state of ethernet LED
pub fn ethernet_led(
    ethernet_led: &mut (
        gpio::gpioe::PE2<Output<PushPull>>,
        gpio::gpioe::PE3<Output<PushPull>>,
    ),
    state: EthernetLED,
) -> Result<(), Never> {
    match state {
        EthernetLED::Off => {
            ethernet_led.0.set_high()?;
            ethernet_led.1.set_high()?;
        }
        EthernetLED::Orange => {
            ethernet_led.0.set_high()?;
            ethernet_led.1.set_low()?;
        }
        EthernetLED::Green => {
            ethernet_led.0.set_low()?;
            ethernet_led.1.set_high()?;
        }
    }

    Ok(())
}

/// Button Debounce
struct Debounce {
    button_state: u32,
    ct0: u32,
    ct1: u32,
    rpt: u32,
}
static GPIOB_BUTTON_PRESS: AtomicU32 = AtomicU32::new(0);
static GPIOB_BUTTON_REPEAT: AtomicU32 = AtomicU32::new(0);

const REPEAT_START: u32 = 500; // 500ms
const REPEAT_NEXT: u32 = 400; // 400ms
                              // const DEB_RUNTIME: u32 = 5000; // 5s

const QUADRATURE_LUT: [i32; 16] =
    [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];

static QUADRATURE_COUNT: AtomicI32 = AtomicI32::new(0);

/// Button Debounce
pub fn debounce_tick(reset: bool) {
    // Mutable state - function is not reentrant
    static mut DEB_GPIOB: Debounce = Debounce {
        button_state: 0,
        ct0: 0xFFFF_FFFF,
        ct1: 0xFFFF_FFFF,
        rpt: 0,
    };
    let deb = unsafe { &mut DEB_GPIOB };

    // GPIO access - read only
    let gpiob = unsafe { &*stm32::GPIOB::ptr() };
    let _current_button_state = !gpiob.idr.read().bits() & 0x3C0;

    if reset {
        deb.button_state = _current_button_state;
        deb.ct0 = 0xFFFF_FFFF;
        deb.ct1 = 0xFFFF_FFFF;
        deb.rpt = 0;
        return;
    }

    let _last_button_state = deb.button_state;
    let i = deb.button_state ^ _current_button_state;

    // Debounce Logic
    deb.ct0 = !(deb.ct0 & i);
    deb.ct1 = deb.ct0 ^ (deb.ct1 & i);
    let j = i & deb.ct0 & deb.ct1;
    deb.button_state ^= j;
    GPIOB_BUTTON_PRESS.fetch_or(deb.button_state & j, Ordering::Relaxed);

    // Repeat presses
    if deb.button_state == 0 {
        deb.rpt = REPEAT_START;
    }
    if deb.rpt == 0 {
        deb.rpt = REPEAT_NEXT;
        GPIOB_BUTTON_REPEAT.fetch_or(deb.button_state, Ordering::Relaxed);
    } else {
        deb.rpt -= 1;
    }

    // Quadrature encoder
    let encoder_state = QUADRATURE_LUT[(((_last_button_state >> 6) & 0xC)
        | ((deb.button_state >> 8) & 0x3))
        as usize];
    QUADRATURE_COUNT.fetch_add(encoder_state, Ordering::Relaxed);
}

pub enum Buttons {
    None = 0,
    Left = 0x0000_0080,
    Right = 0x0000_0040,
}
/// Read and clear button press events in `mask`
pub fn get_button_press(buttons: Buttons) -> u32 {
    let mask = buttons as u32;
    let result = mask & GPIOB_BUTTON_PRESS.load(Ordering::Relaxed);
    GPIOB_BUTTON_PRESS.fetch_xor(result, Ordering::Relaxed);
    result
}
/// Read and clear button repeat events in `mask`
pub fn get_button_repeat(buttons: Buttons) -> u32 {
    let mask = buttons as u32;

    let result = mask & GPIOB_BUTTON_REPEAT.load(Ordering::Relaxed);
    GPIOB_BUTTON_REPEAT.fetch_xor(result, Ordering::Relaxed);
    result
}
/// Read and clear button events
pub fn get_button(buttons: Buttons) -> u32 {
    let mask = buttons as u32;
    let pb = mask & GPIOB_BUTTON_PRESS.load(Ordering::Relaxed);
    GPIOB_BUTTON_PRESS.fetch_xor(pb, Ordering::Relaxed);
    let pr = mask & GPIOB_BUTTON_REPEAT.load(Ordering::Relaxed);
    GPIOB_BUTTON_REPEAT.fetch_xor(pr, Ordering::Relaxed);
    pb as u32 | pr as u32
}

pub enum QuadratureEncoderEvent {
    None,
    Increment,
    Decrement,
}
/// Return quadrature encoder events
pub fn get_quadrature_encoder() -> QuadratureEncoderEvent {
    // println!("Quad {}", QUADRATURE_COUNT.load(Ordering::Relaxed));
    match QUADRATURE_COUNT.load(Ordering::Relaxed) {
        x if x > 3 => {
            QUADRATURE_COUNT.fetch_sub(4, Ordering::Relaxed);
            QuadratureEncoderEvent::Increment
        }
        x if x < 0 => {
            QUADRATURE_COUNT.fetch_add(4, Ordering::Relaxed);
            QuadratureEncoderEvent::Decrement
        }
        _ => QuadratureEncoderEvent::None,
    }
}
