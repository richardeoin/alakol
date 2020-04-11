//! Omron M2 Blood Pressure Monitor Control
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::hal::digital::v2::OutputPin;
use stm32h7xx_hal::{gpio, prelude::*, pwm, stm32};

#[derive(Copy, Clone, Debug)]
pub enum ValveState {
    Open = 0,
    Closed,
}
impl From<u16> for ValveState {
    fn from(a: u16) -> Self {
        match a {
            1 => ValveState::Closed,
            _ => ValveState::Open,
        }
    }
}

/// Represents a state of the Omron M2 hardware
pub struct OmronM2 {
    motor_pwm: pwm::Pwm<stm32::TIM3, pwm::C4>,
    valve_io: gpio::gpiob::PB0<Output<PushPull>>,
    motor_speed: u16,
    valve_state: ValveState,
}

impl OmronM2 {
    pub fn new(
        motor_pwm: pwm::Pwm<stm32::TIM3, pwm::C4>,
        valve_io: gpio::gpiob::PB0<Output<PushPull>>,
    ) -> Self {
        OmronM2 {
            motor_pwm,
            valve_io,
            motor_speed: 0,
            valve_state: ValveState::Open,
        }
    }

    /// Change the motor speed by a percentage
    pub fn change_motor_speed(&mut self, delta: i16) {
        let speed = self.motor_speed as i16 + delta;

        self.motor_speed = match speed {
            x if x > 100 => 100,
            x if x < 0 => 0,
            x => x as u16,
        };

        self.set_motor_speed(self.motor_speed);
    }
    /// Set the motor speed
    ///
    /// motor_speed is a percentage 0 to 100 inclusive
    pub fn set_motor_speed(&mut self, speed: u16) {
        self.motor_speed = speed;

        self.motor_pwm.enable();

        let max = self.motor_pwm.get_max_duty() as u32;

        // motor_speed is a percentage 0 to 100 inclusive
        assert!(self.motor_speed <= 100);
        let duty = max * self.motor_speed as u32 / 100_u32;

        assert!(duty < 65536);
        self.motor_pwm.set_duty(duty as u16);
    }

    /// Set the state of the Omron M2 valve
    pub fn set_valve(&mut self, state: ValveState) {
        self.valve_state = state;

        use ValveState::*;
        match self.valve_state {
            Open => self.valve_io.set_low(),
            Closed => self.valve_io.set_high(),
        }
        .ok();
    }

    /// Current motor speed
    pub fn motor_speed(&self) -> u16 {
        self.motor_speed
    }

    /// Current valve state
    pub fn valve_state(&self) -> ValveState {
        self.valve_state
    }
}
