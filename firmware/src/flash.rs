//! STM32H7 Flash Memory

use core::ptr;
use core::slice;
use stm32h7xx_hal::stm32;

use serde::{Deserialize, Serialize};
use serde_cbor::error::Error as CborError;
use serde_cbor::ser::SliceWrite;
use serde_cbor::{Deserializer, Serializer};

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct FlashStorageData {
    pub gauge_calibration_mv: f32,
}
impl FlashStorageData {
    /// Initial value when storage is not initialised
    const BLANK: FlashStorageData = FlashStorageData {
        gauge_calibration_mv: 0.,
    };
}

/// Storage of program data in flash
pub struct FlashStorage {
    flash: Flash,
    pub data: FlashStorageData,
}
impl FlashStorage {
    const ADDRESS: usize = 0x081F_0000;
    const BUFFER_SIZE: usize = 64;

    /// Creates a new flash storage and populates data with the values
    /// currently in memory
    pub fn new(flash: Flash) -> Self {
        // Try to load from flash, but otherwise leave as the default value
        let data = match Self::try_load() {
            Ok(data) => data,
            Err(e) => {
                info!("Failed to load from flash storage: {:?}", e);
                FlashStorageData::BLANK
            }
        };

        FlashStorage { flash, data }
    }

    /// Write the current contents of data to flash
    pub fn store(&mut self) {
        if let Err(e) = self.try_store() {
            warn!("Failed to write to flash storage: {:?}", e);
        }
    }

    fn try_load() -> Result<FlashStorageData, CborError> {
        let mut buf = [0u8; Self::BUFFER_SIZE];
        let storage = unsafe {
            let ptr = Self::ADDRESS as *const u8;
            slice::from_raw_parts(ptr, Self::BUFFER_SIZE)
        };
        buf.clone_from_slice(&storage);

        // Deserialise
        let mut deser = Deserializer::from_mut_slice(&mut buf);

        serde::de::Deserialize::deserialize(&mut deser)
    }
    fn try_store(&mut self) -> Result<(), CborError> {
        // Serialise
        let mut buf = [0u8; Self::BUFFER_SIZE];
        let writer = SliceWrite::new(&mut buf[..]);
        let mut ser = Serializer::new(writer);
        ser.self_describe()?;
        self.data.serialize(&mut ser)?;

        // Write to flash
        self.flash.unlock(FlashBank::Bank2).unwrap();

        // Erase
        self.flash
            .erase(FlashBank::Bank2, FlashSector::Sector7)
            .unwrap();

        // Write
        self.flash.write(Self::ADDRESS, &buf).unwrap();

        self.flash.lock(FlashBank::Bank2).unwrap();

        Ok(())
    }
}

/// Flash Wrapper
pub struct Flash {
    rb: stm32::FLASH,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FlashError {
    WriteError,
    EraseError,
    LockFailed,
    UnlockFailed,
}

/// One of two flash banks
#[derive(Clone, Copy, Debug)]
pub enum FlashBank {
    Bank1,
    Bank2,
}

/// Sector within each bank
#[derive(Clone, Copy, Debug)]
pub enum FlashSector {
    Sector0 = 0,
    Sector1,
    Sector2,
    Sector3,
    Sector4,
    Sector5,
    Sector6,
    Sector7,
}

macro_rules! flash_bank_impl {
    ($($Bank:ident, $erase:ident, $write:ident, $unlock:ident, $lock:ident =>
       ($cr:ident, $psize:ident, $ser:ident, $snb:ident, $start:ident,
        $pg:ident, $bsy:ident, $sr:ident, $operr:ident, $pgserr:ident,
        $wrperr:ident, $keyr:ident, $qw:ident, $fw:ident) =>
        [$addr_start:expr, $addr_end:expr]),*) => {

        /// Erases a flash sector
        pub fn erase(&mut self, bank: FlashBank, sector: FlashSector)
                     -> Result<(), FlashError> {
            match bank {
                $(FlashBank::$Bank => self.$erase(sector)),*
            }
        }

        /// Writes to an address in flash
        pub fn write(
            &mut self,
            address: usize,
            data: &[u8],
        ) -> Result<(), FlashError> {
            match address {
                $(addr @ $addr_start..=$addr_end =>
                  self.$write(addr, data),)*
                _ => panic!("Bad address")
            }
        }

        /// Locks a flash sector
        pub fn lock(&mut self, bank: FlashBank)
                     -> Result<(), FlashError> {
            match bank {
                $(FlashBank::$Bank => self.$lock()),*
            }
        }

        /// Unlocks a flash sector
        pub fn unlock(&mut self, bank: FlashBank)
                      -> Result<(), FlashError> {
            match bank {
                $(FlashBank::$Bank => self.$unlock()),*
            }
        }


        $(
            fn $erase(&mut self, sector: FlashSector) -> Result<(), FlashError> {
                // Configure
                self.rb.$cr.modify(|_, w| unsafe {
                    w.$psize()
                        .bits(2) // Program Size (PSIZE) = 32 bits
                        .$ser()
                        .set_bit() // Sector Erase
                        .$snb()
                        .bits(sector as u8)
                });
                // Start sector erase
                self.rb.$cr.modify(|_, w| w.$start().set_bit());

                // Wait
                while self.rb.$sr.read().$qw().bit_is_set() {}

                // Check for errors
                let sr = self.rb.$sr.read();
                if sr.$operr().bit_is_set() || sr.$pgserr().bit_is_set() {
                    Err(FlashError::EraseError)
                } else {
                    Ok(())
                }
            }

            // Writes to an address in flash
            fn $write(
                &mut self,
                address: usize,
                data: &[u8],
            ) -> Result<(), FlashError> {
                // Set Program Size (PSIZE) = 32 bits
                self.rb.$cr.modify(|_, w| unsafe { w.$psize().bits(2) });
                // Enable buffer for write operations
                self.rb.$cr.modify(|_, w| w.$pg().set_bit());

                // Copy data
                unsafe {
                    let src = data.as_ptr();
                    let dst = address as *mut u8;
                    ptr::copy_nonoverlapping(src, dst, data.len());
                }

                // Trigger Write
                self.rb.$cr.modify(|_, w| w.$fw().set_bit());

                // Wait
                while self.rb.$sr.read().$bsy().bit_is_set() {}

                // Check for errors
                let sr = self.rb.$sr.read();
                if sr.$operr().bit_is_set()
                    || sr.$pgserr().bit_is_set()
                    || sr.$wrperr().bit_is_set()
                {
                    return Err(FlashError::WriteError);
                }

                // Disable buffer
                self.rb.$cr.modify(|_, w| w.$pg().clear_bit());

                Ok(())
            }

            // Unlocks flash bank
            fn $unlock(&mut self) -> Result<(), FlashError> {
                // Wait
                while self.rb.$sr.read().$bsy().bit_is_set() {}

                if self.rb.$cr.read().$lock().bit_is_set() {
                    // Flash unlock sequence
                    self.rb
                        .$keyr
                        .write(|w| unsafe { w.$keyr().bits(0x4567_0123) });
                    self.rb
                        .$keyr
                        .write(|w| unsafe { w.$keyr().bits(0xCDEF_89AB) });
                }

                // Check lock status
                if self.rb.$cr.read().$lock().bit_is_clear() {
                    Ok(())
                } else {
                    Err(FlashError::UnlockFailed)
                }
            }

            // Locks flash bac
            fn $lock(&mut self) -> Result<(), FlashError> {
                // Wait
                while self.rb.$sr.read().$bsy().bit_is_set() {}

                // Lock
                self.rb.$cr.modify(|_, w| w.$lock().set_bit());
                // Check lock status
                if self.rb.$cr.read().$lock().bit_is_set() {
                    Ok(())
                } else {
                    Err(FlashError::LockFailed)
                }
            }
        )+
    }
}

impl Flash {
    flash_bank_impl!(
        Bank1, erase1, write1, unlock1, lock1 =>
            (cr1, psize1, ser1, snb1, start1,
             pg1, bsy1, sr1, operr1, pgserr1,
             wrperr1, keyr1, qw1, fw1) =>
            [0x0800_0000, 0x080F_FFFF],
        Bank2, erase2, write2, unlock2, lock2 =>
            (cr2, psize2, ser2, snb2, start2,
             pg2, bsy2, sr2, operr2, pgserr2,
             wrperr2, keyr2, qw2, fw2) =>
            [0x0810_0000, 0x081F_FFFF]
    );

    /// Create new wrapper around STM32 Flash
    pub fn new(flash: stm32::FLASH) -> Self {
        Flash { rb: flash }
    }
}
