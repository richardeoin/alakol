//! A display with multiple screens and a cursor

use core::fmt;
use core::fmt::Write;
use stm32h7xx_hal::{gpio, pac, spi};

use crate::io::Buttons;
use crate::omron_m2::ValveState;
use crate::sensors::ThermocoupleReading as TReading;
use crate::time::SimpleTime;
use smoltcp::wire::{Ipv4Address, Ipv4Cidr};

use embedded_graphics::fonts::{Font12x16, Font6x12, Font6x8};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, Rect};
use ssd1306::prelude::*;

pub type OmronOled = GraphicsMode<
    SpiInterface<
        spi::Spi<
            pac::SPI3,
            (
                gpio::gpioc::PC10<gpio::Alternate<gpio::AF6>>,
                spi::NoMiso,
                gpio::gpioc::PC12<gpio::Alternate<gpio::AF6>>,
            ),
        >,
        gpio::gpiod::PD0<gpio::Output<gpio::PushPull>>,
    >,
>;

enum Screen {
    EthernetStatus,
    Measurement,
    MotorControl,
    Time,
}

// Define a screen with properties
macro_rules! screen {
    ($SCREEN:ident, $screen:ident, $draw:ident, $title:expr =>
     [ $($ELEMENT:ident, $element:ident:
         ($type:ty, $init:expr, $format:expr, $row:expr)$( => $func_left:ident, $func_right:ident)*,)* ]) => {

        pub struct $SCREEN {
            $(
                $element: $type
            ),*
        }
        impl $SCREEN {
            pub fn new() -> Self {
                $SCREEN {
                    $(
                        $element: $init
                    ),*
                }
            }

            // Click on a row in this screen
            pub fn click(oled: &mut Oled, button: Buttons, row: i32) {
                match (row, button) {
                    (0, Buttons::Right) => oled.next_screen(),
                    (0, Buttons::Left) => oled.previous_screen(),
                    $(
                        $(
                            ($row, Buttons::Right) => (oled.$func_right)(),
                            ($row, Buttons::Left) => (oled.$func_left)(),
                        )*
                    )*
                    _ => {},
                }
            }
            // Can a row in this screen be clicked on?
            #[allow(unused_variables)]
            pub fn can_click(row:i32) -> bool {
                match row {
                    0 => true,
                    $(
                        $(
                            $row => { let $func_left = 0; true },
                        )*
                    )*
                    _ => false
                }
            }
        }
        impl Oled {
            $(
                /// Set display element
                pub fn $element(&mut self, value: $type) {
                    self.$screen.$element = value;
                }
            )*
        }
        impl Oled {
            pub fn $draw(&mut self) {
                self.do_title($title);
                $(
                    let value = self.$screen.$element;
                    $ELEMENT { oled: self, x: 0 }
                    .write_fmt(format_args!($format, value)).ok();
                )*
                self.do_cursor(self.cursor_line,
                               $SCREEN::can_click(self.cursor_line));
                self.disp.flush().unwrap();
            }
        }
        $(
            struct $ELEMENT<'p> {
                oled: &'p mut Oled,
                x: i32,
            }
            impl<'p> fmt::Write for $ELEMENT<'p> {
                fn write_str(&mut self, s: &str) -> fmt::Result {
                    self.x = self.oled.do_small_text(s, self.x, row!($row));
                    Ok(())
                }
            }
        )*
    }
}

macro_rules! row {
    ($r:expr) => {
        match $r {
            0 => 0,
            x => 7 + (9 * x),
        }
    };
}

screen! {
    MeasurementScreen, measurement, draw_measurement_screen, "Measurements" => [
        CPUTemperature, cpu_temperature: (f32, 0.0, "CPU T:  {:.0} °C", 1),
        KTemperature, k_temperature: (TReading, TReading::NONE, "Thermo: {} °C", 2),
        SensorTemperature, sensor_temperature: (f32, 0.0, "Sensor: {:.1} °C", 3),
        Pressure, pressure: (f32, 0.0, "Pressure: {:.2} kPa", 4),
        Gauge, gauge: (f32, 0.0, "Gauge P: {:3.3} kPa", 5) => none, gauge_calibrate,
    ]
}
screen! {
    MotorControlScreen, motor_control, draw_motor_control_screen, "Motor" => [
        MotorPWM, motor_pwm: (u16, 0, "Motor PWM: {:?} %", 1) => motor_slow, motor_fast,
        Valve, valve: (ValveState, ValveState::Closed, "Valve: {:?}", 2) => valve_close, valve_open,
        SupplyVoltage, supply_voltage: (f32, 0.0, "Supply: {:.2} V", 3),
    ]
}
screen! {
    EthernetStatusScreen, ethernet_status, draw_ethernet_status_screen, "Ethernet Status" => [
        IP, ip: (Ipv4Cidr, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0), "IP: {}", 1), // CIDR!
        Gateway, gateway: (Ipv4Address, Ipv4Address::UNSPECIFIED, "Gateway: {}", 2),
    ]
}
screen! {
    TimeScreen, time, draw_time_screen, "Time" => [
        Time, time: (SimpleTime, SimpleTime(0), "Uptime: {}", 1),
    ]
}

pub struct Oled {
    disp: OmronOled,
    current_screen: Screen,
    pub measurement: MeasurementScreen,
    motor_control: MotorControlScreen,
    ethernet_status: EthernetStatusScreen,
    time: TimeScreen,
    cursor_line: i32,
    none: fn(),
    gauge_calibrate: fn(),
    motor_slow: fn(),
    motor_fast: fn(),
    valve_close: fn(),
    valve_open: fn(),
}

impl Oled {
    pub fn next_screen(&mut self) {
        use Screen::*;
        self.current_screen = match self.current_screen {
            Measurement => MotorControl,
            MotorControl => EthernetStatus,
            EthernetStatus => Time,
            Time => Measurement,
        };
    }
    pub fn previous_screen(&mut self) {
        use Screen::*;
        self.current_screen = match self.current_screen {
            MotorControl => Measurement,
            EthernetStatus => MotorControl,
            Time => EthernetStatus,
            Measurement => Time,
        };
    }
    pub fn click_screen(&mut self, button: Buttons) {
        match self.current_screen {
            Screen::EthernetStatus => {
                EthernetStatusScreen::click(self, button, self.cursor_line)
            }
            Screen::Measurement => {
                MeasurementScreen::click(self, button, self.cursor_line)
            }
            Screen::MotorControl => {
                MotorControlScreen::click(self, button, self.cursor_line)
            }
            Screen::Time => TimeScreen::click(self, button, self.cursor_line),
        };
    }
    pub fn draw_screen(&mut self) {
        self.disp.clear();
        match self.current_screen {
            Screen::EthernetStatus => self.draw_ethernet_status_screen(),
            Screen::Measurement => self.draw_measurement_screen(),
            Screen::MotorControl => self.draw_motor_control_screen(),
            Screen::Time => self.draw_time_screen(),
        };
    }

    /// Init
    pub fn new(
        disp: OmronOled,
        gauge_calibrate: fn(),
        motor_slow: fn(),
        motor_fast: fn(),
        valve_close: fn(),
        valve_open: fn(),
    ) -> Self {
        Oled {
            disp,
            current_screen: Screen::Measurement,
            measurement: MeasurementScreen::new(),
            motor_control: MotorControlScreen::new(),
            ethernet_status: EthernetStatusScreen::new(),
            time: TimeScreen::new(),
            cursor_line: 0,
            none: || {},
            gauge_calibrate,
            motor_slow,
            motor_fast,
            valve_close,
            valve_open,
        }
    }

    /// Sets the current line for the cursor
    pub fn set_cursor_line(&mut self, line: i32) -> i32 {
        if (line >= 0) && (line < 6) {
            self.cursor_line = line;
        }

        self.cursor_line
    }

    pub fn draw_test_screen(&mut self) {
        self.disp.draw(
            Line::new(Coord::new(8, 16 + 16), Coord::new(8 + 16, 16 + 16))
                .with_stroke(Some(1u8.into()))
                .into_iter(),
        );
        self.disp.draw(
            Line::new(Coord::new(8, 16 + 16), Coord::new(8 + 8, 16))
                .with_stroke(Some(1u8.into()))
                .into_iter(),
        );
        self.disp.draw(
            Line::new(Coord::new(8 + 16, 16 + 16), Coord::new(8 + 8, 16))
                .with_stroke(Some(1u8.into()))
                .into_iter(),
        );

        self.disp.draw(
            Rect::new(Coord::new(48, 16), Coord::new(48 + 16, 16 + 16))
                .with_stroke(Some(1u8.into()))
                .into_iter(),
        );

        self.disp.draw(
            Circle::new(Coord::new(96, 16 + 8), 8)
                .with_stroke(Some(1u8.into()))
                .into_iter(),
        );

        self.disp.draw(
            Font12x16::render_str("Hello World!")
                .with_stroke(Some(0u8.into()))
                .with_fill(Some(1u8.into()))
                .translate(Coord::new(5, 50))
                .into_iter(),
        );

        self.disp.flush().unwrap();
    }

    /// Draw title with underline
    fn do_title(&mut self, string: &str) {
        let hwidth = 3 * string.len() as i32;
        let (start, stop) = (64 - hwidth, 64 + hwidth);

        self.disp.draw(
            Font6x12::render_str(string)
                .with_stroke(Some(1u8.into()))
                .translate(Coord::new(start, 0))
                .into_iter(),
        );
        let under = 12;

        self.disp.draw(
            Line::new(Coord::new(start - 1, under), Coord::new(stop, under))
                .with_stroke(Some(1u8.into()))
                .into_iter(),
        );
    }

    /// Draw small text at x, y. Returns new x position.
    fn do_small_text(&mut self, s: &str, x: i32, y: i32) -> i32 {
        self.disp.draw(
            Font6x8::render_str(s)
                .with_stroke(Some(1u8.into()))
                .translate(Coord::new(x, y))
                .into_iter(),
        );
        x + (6 * s.len()) as i32
    }

    /// Draws a cursor
    fn do_cursor(&mut self, line: i32, filled: bool) {
        let rect = Rect::new(
            Coord::new(123, row!(line) + 1),
            Coord::new(127, row!(line) + 6),
        );
        let cursor = match filled {
            true => rect.with_fill(Some(1u8.into())),
            false => rect.with_stroke(Some(1u8.into())),
        };
        self.disp.draw(cursor.into_iter());
    }
}
