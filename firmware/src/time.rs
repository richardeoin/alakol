//! Time
use core::fmt;

#[derive(Copy, Clone, Debug)]
pub struct SimpleTime(pub i64);

impl fmt::Display for SimpleTime {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let epoch = self.0 / 1000;
        let hours = epoch / 3600;
        let minutes = (epoch / 60) - (hours * 60);
        let seconds = epoch % 60;
        write!(f, "{:02}:{:02}:{:02}", hours, minutes, seconds)
    }
}
