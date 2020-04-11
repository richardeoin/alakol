//! Sensor Data
#![allow(clippy::let_and_return)]

use ms5611_spi::Oversampling;
use stm32h7xx_hal::{
    adc, gpio,
    hal::blocking::delay::DelayMs,
    prelude::*,
    signature::{TS_CAL_110, TS_CAL_30},
    spi, stm32,
};
use thermocouple::{prelude::*, KType};

use core::fmt;
use generic_array::typenum::U32;
use stm32h7xx_hal::gpio::Analog;
use stm32h7xx_hal::hal::digital::v1_compat::OldOutputPin;

use serde::Serialize;
use serde_cbor::error::Error as CborError;
use serde_cbor::ser::SliceWrite;
use serde_cbor::Serializer;

/// Represent a reading from the external thermocouple
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ThermocoupleReading(Option<f32>);
impl fmt::Display for ThermocoupleReading {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(x) => write!(f, "{:.1}", x),
            None => write!(f, "--.-"),
        }
    }
}
impl ThermocoupleReading {
    pub const NONE: ThermocoupleReading = ThermocoupleReading(None);
}

/// Collection of all sensor data
#[derive(Debug, PartialEq, Serialize)]
pub struct SensorData {
    pub ms5611_temperature: f32,
    pub ms5611_pressure: f32,
    pub internal_temperature: f32,
    pub gauge_pressure: f32,
    pub thermocouple_temperature: ThermocoupleReading,
    pub supply_voltage: f32,
}
impl SensorData {
    /// Serialise to a buffer, consuming self
    /// Returns the number of bytes written
    pub fn serialise_to_buffer(
        self,
        buf: &mut [u8],
    ) -> Result<usize, CborError> {
        // Serialise
        let writer = SliceWrite::new(&mut buf[..]);
        let mut ser = Serializer::new(writer);
        ser.self_describe()?;
        self.serialize(&mut ser)?;

        // Bytes written
        let writer = ser.into_inner();
        Ok(writer.bytes_written())
    }
}

pub type MS5611 = ms5611_spi::Ms5611<
    spi::Spi<stm32::SPI1, crate::io::PressureSPIPinsT>,
    OldOutputPin<gpio::gpioa::PA4<gpio::Output<gpio::PushPull>>>,
>;

/// Grouping of all available sensors
pub struct Sensors {
    adc1: adc::Adc<stm32::ADC1, adc::Enabled>,
    adc3: adc::Adc<stm32::ADC3, adc::Enabled>,
    v_psense: gpio::gpioa::PA0<Analog>,
    v_tsense: gpio::gpioa::PA3<Analog>,
    pwr_mon: gpio::gpioa::PA6<Analog>,
    internal_temperature: adc::Temperature,
    ms5611: MS5611,
    latest_ms5611_reading: Option<ms5611_spi::Sample>,
    thermocouple: KType,
    gauge_pressure_offset_mv: f32,
    gauge_pressure_filter: median::Filter<f32, U32>,
}

macro_rules! read_channel {
    ($($channel:ident: $read_channel_fn:ident),*) => {
        $(
            /// Reads voltage from ADC channel
            fn $read_channel_fn(&mut self) -> f32 {
                let vdda = 2.500;

                let word: u32 = self
                    .adc1
                    .read(&mut self.$channel)
                    .expect("ADC channel read failed.");

                // Return pin voltage
                vdda * word as f32 / self.adc1.max_sample() as f32
            }
        )*
    }
}

impl Sensors {
    pub fn new(
        adc1: adc::Adc<stm32::ADC1, adc::Enabled>,
        adc3: adc::Adc<stm32::ADC3, adc::Enabled>,
        v_psense: gpio::gpioa::PA0<Analog>,
        v_tsense: gpio::gpioa::PA3<Analog>,
        pwr_mon: gpio::gpioa::PA6<Analog>,
        internal_temperature: adc::Temperature,
        ms5611: MS5611,
    ) -> Self {
        Sensors {
            adc1,
            adc3,
            v_psense,
            v_tsense,
            pwr_mon,
            internal_temperature,
            ms5611,
            latest_ms5611_reading: None,
            thermocouple: KType::new(),
            gauge_pressure_offset_mv: 0.,
            gauge_pressure_filter: median::Filter::new(),
        }
    }

    // 1st level read: Raw Data =================================

    read_channel! {
        v_psense: read_v_psense,
        v_tsense: read_v_tsense,
        pwr_mon: read_pwr_mon
    }

    /// Reads Instantaneous, Uncalibrated Gauge Pressure -> mV
    /// This pressure is stored in a moving average buffer
    pub fn read_gauge_pressure_mv(&mut self) -> f32 {
        let instrumentation_input = (self.read_v_psense() - 1.25) / 20.0;
        self.gauge_pressure_filter
            .consume(1000. * instrumentation_input)
    }

    /// Reads the MS5611 sensor
    pub fn read_ms5611(&mut self, delay: &mut impl DelayMs<u8>) {
        let sample = self
            .ms5611
            .get_second_order_sample(Oversampling::OS_2048, delay)
            .unwrap();

        self.latest_ms5611_reading = Some(sample);
        // Sensor appears to read about 8ºC above ambient
        self.thermocouple = self.thermocouple.with_reference_temperature(
            ((sample.temperature - 8) as f32 / 100.0).celsius(),
        );
    }

    // Data processing ===========================================

    /// Collect data from the various sensors
    pub fn collect_data(&mut self) -> SensorData {
        SensorData {
            ms5611_temperature: self.ms5611_temperature(),
            ms5611_pressure: self.ms5611_pressure(),
            internal_temperature: self.internal_temperature(),
            gauge_pressure: self.gauge_pressure(),
            thermocouple_temperature: self.thermocouple(),
            supply_voltage: self.supply_voltage(),
        }
    }

    /// Returns last temperature reading from MS5611, in °C
    fn ms5611_temperature(&self) -> f32 {
        self.latest_ms5611_reading
            .expect("MS5611 not read yet!")
            .temperature as f32
            / 100.0
    }

    /// Returns last temperature reading from MS5611, in hPa
    fn ms5611_pressure(&self) -> f32 {
        self.latest_ms5611_reading
            .expect("MS5611 not read yet!")
            .pressure as f32
            / 1000.0
    }

    /// Reads the STM32H7 internal temperature
    fn internal_temperature(&mut self) -> f32 {
        let vdda = 2.500;

        let word: u32 = self
            .adc3
            .read(&mut self.internal_temperature)
            .expect("Temperature read failed.");

        // Average slope
        let cal = (110.0 - 30.0)
            / (TS_CAL_110::get().read() - TS_CAL_30::get().read()) as f32;
        // Calibration values are measured at VDDA = 3.3 V ± 10 mV
        let word_3v3 = word as f32 * vdda / 3.3;
        // Linear interpolation
        let temperature =
            cal * (word_3v3 - TS_CAL_30::get().read() as f32) + 30.0;

        temperature
    }

    /// Returns moving median of gauge pressure
    fn gauge_pressure(&mut self) -> f32 {
        let instrumentation_input_mv = self.gauge_pressure_filter.median();
        // Span voltage
        let k = 31.0 / 37.0; // Constant mV/kPa

        // Zeroing
        let offset = self.gauge_pressure_offset_mv; // mV

        (instrumentation_input_mv - offset) / k
    }

    /// Reads Thermocouple Temperature -> °C
    fn thermocouple(&mut self) -> ThermocoupleReading {
        let meas_v = self.read_v_tsense();

        match meas_v {
            x if x >= 2.49 => ThermocoupleReading(None), // Not connected
            x => {
                // Thermoelectric potential
                let t_potential = (1000. * (x - 1.25) / 50.4).millivolts();
                let t: Celsius =
                    self.thermocouple.sense_temperature(t_potential);

                ThermocoupleReading(Some(t.0))
            }
        }
    }

    /// Reads Supply Voltage -> V
    fn supply_voltage(&mut self) -> f32 {
        self.read_pwr_mon() * 11.0
    }

    // Calibration ================================================

    /// Writes Gauge Pressure calibration value directly
    pub fn set_gauge_pressure_calibration(&mut self, gauge: f32) {
        self.gauge_pressure_offset_mv = gauge;
    }

    /// Assuming that Gauge Pressure is currently zero, calibrate the
    /// gauge pressure reading
    pub fn calibrate_gauge_pressure(&mut self) -> f32 {
        self.gauge_pressure_offset_mv = self.read_gauge_pressure_mv();
        self.gauge_pressure_offset_mv
    }
}
