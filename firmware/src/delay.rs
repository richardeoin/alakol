//! Timer as delay provider

use hal::blocking::delay::{DelayMs, DelayUs};
use hal::timer::CountDown;
use stm32h7xx_hal::hal;

use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::time::Hertz;

/// Timer as a delay provider
pub struct DelayFromTimer<T>(T);

impl<T> DelayFromTimer<T> {
    /// Creates delay provider from timer
    pub fn new(timer: T) -> Self {
        Self(timer)
    }

    /// Releases the Timer
    pub fn free(self) -> T {
        self.0
    }
}

macro_rules! impl_delay_from_timer  {
    ($(($Delay:ident, $delay:ident, $num:expr)),+) => {
        $(
            impl<T> $Delay<u32> for DelayFromTimer<T>
            where
                T: CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u32) {
                    self.0.start(($num / (t*2)).hz());
                    block!(self.0.wait()).ok();
                }
            }

            impl<T> $Delay<u16> for DelayFromTimer<T>
            where
                T: CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u16) {
                    self.$delay(t as u32);
                }
            }

            impl<T> $Delay<u8> for DelayFromTimer<T>
            where
                T: CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u8) {
                    self.$delay(t as u32);
                }
            }
        )+
    }
}

impl_delay_from_timer! {
    (DelayMs, delay_ms, 1_000),
    (DelayUs, delay_us, 1_000_000)
}
