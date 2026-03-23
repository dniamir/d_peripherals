/// Driver for IS31FL3193 LED driver
use phf::Map;  // Efficient map for register maps
use phf_macros::phf_map;

use crate::d_peripherals::chip::{Chip, CommProvider, ChipError};
use crate::d_peripherals::chip_implementations::{Addressable, ShadowComm, CommError};
use crate::d_peripherals::chip_map::{Field, FieldMapProvider};

use crate::{d_log::dlogger_common::DLogger, d_info};  // Logging

#[derive(Copy, Clone, Debug)]
pub enum LedColor { RED, GREEN, BLUE, YELLOW, PURPLE, TEAL, ALL, WHITE }

pub const IS31_NUM_REGS: usize = 24;

#[derive(Debug)]
pub enum IS31FL3193Error {
    NotFound,
    BusError(ChipError),
    CommError(CommError),
}

// Error conversions
impl From<ChipError> for IS31FL3193Error {
    fn from(err: ChipError) -> Self {IS31FL3193Error::BusError(err)}
}
impl From<CommError> for IS31FL3193Error {
    fn from(err: CommError) -> Self {IS31FL3193Error::CommError(err)}
}

type IS3Chip<COMM> = Chip<ShadowComm<COMM>, IS31FL3193FieldMap>;

pub struct IS31FL3193<COMM> 
{
    pub chip: IS3Chip<COMM>,
    pub status: i8,
}

impl <COMM> IS31FL3193<COMM> {
    pub const DEFAULT_I2C_ADDRESS: u8 = 0x68;
    pub const WHO_AM_I_REG: u8 = 0x00;
    pub const WHO_AM_I_VAL: u8 = 0x01;
}

impl <COMM> IS31FL3193<COMM>
where
    COMM: CommProvider + Addressable,
{
    /// Create a new IS31FL3193 instance
    /// This is tricky, because the chip has no ability to read register
    /// So a shadow map is used to store the current register values
    pub async fn new_i2c<T: Into<Option<u8>>>(i2c: COMM, i2c_addr: T) -> Result<Self, IS31FL3193Error> {

        // Default i2c address
        let i2c_addr = i2c_addr.into();
        let i2c_addr = i2c_addr.unwrap_or(Self::DEFAULT_I2C_ADDRESS);

        // Define shadow registers
        let i2c_shadow = ShadowComm::<COMM>::new(i2c);
        let chip= IS3Chip::new_i2c(i2c_shadow, i2c_addr);

        let mut this = Self {
            chip,
            status: -1,
        };

        // Perform soft reset and reset shadow register map
        this.soft_reset().await?;

        Ok(this)
    }
}

impl <COMM> IS31FL3193<COMM>
where
    COMM: CommProvider,
{
    /// Perform a soft reset
    /// Unlike other functions, this will perform immediately
    pub async fn soft_reset(&mut self) -> Result<(), IS31FL3193Error>  {
        d_info!("Performing Soft Reset");
        DLogger::hold();
        let hold_count = DLogger::get_hold_count();

        // Ignore a returned error, since ACK will likely not be returned
        let _ = self.chip.comm.true_raw_write(0x2F, 0).await;
        self.chip.comm.reset_shadow();
        
        // Return hold count
        DLogger::set_hold(hold_count);
        
        Ok(())
    }

    /// Update LEDs with new settings
    /// This needs to be called ANYTIME the settings are updated
    pub async fn update_leds(&mut self) -> Result<(), IS31FL3193Error>  {

        d_info!("Updating LEDs");

        // Ignore reset bit
        DLogger::hold();
        self.chip.comm.dirty_bits.borrow_mut()[0x2F] = 0;
        
        self.chip.comm.sync_all().await?;
        self.chip.comm.true_raw_write(0x1C, 0b000).await?;  // Update time registers
        self.chip.comm.true_raw_write(0x1D, 0b111).await?;   // Enable LED controls
        self.chip.comm.true_raw_write(0x07, 0x0).await?;   // PWM update registers
        DLogger::release();
        Ok(())
    }

    /// Set the color and the brightness of the driver
    /// each path enables one of three paths, which should be connected to
    /// the blue LED, red LED, or green LED
    pub async fn set_color(&mut self, color: LedColor, brightness_per: u8) -> Result<(), IS31FL3193Error>  {
        d_info!("Setting LED color");
        let brightness = ((brightness_per as u16 * 255) / 100) as u8;

        DLogger::hold();
        
        match color {
            LedColor::RED => {
                self.chip.write_reg_str("PWM1", brightness).await?;
            }

            LedColor::GREEN => {
                self.chip.write_reg_str("PWM2", brightness).await?;
            }

            LedColor::BLUE => {
                self.chip.write_reg_str("PWM3", brightness).await?;
            }

            LedColor::YELLOW => {
                self.chip.write_reg_str("PWM1", brightness).await?;
                self.chip.write_reg_str("PWM2", brightness).await?;
            }

            LedColor::PURPLE => {
                self.chip.write_reg_str("PWM1", brightness).await?;
                self.chip.write_reg_str("PWM3", brightness).await?;
            }

            LedColor::TEAL => {
                self.chip.write_reg_str("PWM2", brightness).await?;
                self.chip.write_reg_str("PWM3", brightness).await?;
            }

            LedColor::ALL | LedColor::WHITE => {
                self.chip.write_reg_str("PWM1", brightness).await?;
                self.chip.write_reg_str("PWM2", brightness).await?;
                self.chip.write_reg_str("PWM3", brightness).await?;
            }
        }

        DLogger::release();

        Ok(())
    }

    /// Set timing for pulses
    pub async fn set_timing(&mut self, t0: u8, t1: u8, t2: u8, t3: u8, t4: u8) -> Result<(), IS31FL3193Error>  {
        
        d_info!("Setting LED pulse timing");

        DLogger::hold();

        self.chip.write_field_str("T01", t0).await?;  // Set T0 in shot mode
        self.chip.write_field_str("T02", t0).await?;  // Set T0 in shot mode
        self.chip.write_field_str("T03", t0).await?;  // Set T0 in shot mode

        self.chip.write_field_str("T11", t1).await?;  // Set T1 in shot mode
        self.chip.write_field_str("T12", t1).await?;  // Set T1 in shot mode
        self.chip.write_field_str("T13", t1).await?;  // Set T1 in shot mode

        self.chip.write_field_str("T21", t2).await?;  // Set T2 in shot mode
        self.chip.write_field_str("T22", t2).await?;  // Set T2 in shot mode
        self.chip.write_field_str("T23", t2).await?;  // Set T2 in shot mode

        self.chip.write_field_str("T31", t3).await?;  // Set T3 in shot mode
        self.chip.write_field_str("T32", t3).await?;  // Set T3 in shot mode
        self.chip.write_field_str("T33", t3).await?;  // Set T3 in shot mode

        self.chip.write_field_str("T41", t4).await?;  // Set T4 in shot mode
        self.chip.write_field_str("T42", t4).await?;  // Set T4 in shot mode
        self.chip.write_field_str("T43", t4).await?;  // Set T4 in shot mode

        DLogger::release();

        Ok(())
    }
}

#[derive(Copy, Clone)]
pub struct IS31FL3193FieldMap;

impl FieldMapProvider for IS31FL3193FieldMap {
    fn get_read_field(name: &str) -> Option<Field> {
        Some(*FIELD_MAP.get(name)?)
    }
    fn get_write_field(name: &str) -> Option<Field> {
        IS31FL3193FieldMap::get_read_field(name)
    }
}

// I2C Map - (SPI map is different)
pub static FIELD_MAP: Map<&'static str, Field> = phf_map! {
    "Shutdown" => Field { reg: 0x00, offset: 0, bits: 8, writable: true, signed: false, },
    "EN" => Field { reg: 0x00, offset: 5, bits: 1, writable: true, signed: false, },
    "SSD" => Field { reg: 0x00, offset: 0, bits: 1, writable: true, signed: false, },

    "Breathing Control" => Field { reg: 0x01, offset: 0, bits: 8, writable: true, signed: false, },
    "RM" => Field { reg: 0x01, offset: 5, bits: 1, writable: true, signed: false, },
    "HT" => Field { reg: 0x01, offset: 4, bits: 1, writable: true, signed: false, },
    "BME" => Field { reg: 0x01, offset: 2, bits: 1, writable: true, signed: false, },
    "CSS" => Field { reg: 0x01, offset: 0, bits: 2, writable: true, signed: false, },

    "LED Mode" => Field { reg: 0x02, offset: 0, bits: 8, writable: true, signed: false, },
    "RGB" => Field { reg: 0x02, offset: 5, bits: 1, writable: true, signed: false, },

    "Current Setting" => Field { reg: 0x03, offset: 0, bits: 8, writable: true, signed: false, },
    "CS" => Field { reg: 0x03, offset: 2, bits: 3, writable: true, signed: false, },

    "PWM1" => Field { reg: 0x04, offset: 0, bits: 8, writable: true, signed: false, },
    "PWM2" => Field { reg: 0x05, offset: 0, bits: 8, writable: true, signed: false, },
    "PWM3" => Field { reg: 0x06, offset: 0, bits: 8, writable: true, signed: false, },

    "PWM Update" => Field { reg: 0x07, offset: 0, bits: 8, writable: true, signed: false, },

    "T01" => Field { reg: 0x0A, offset: 4, bits: 4, writable: true, signed: false, },
    "T02" => Field { reg: 0x0B, offset: 4, bits: 4, writable: true, signed: false, },
    "T03" => Field { reg: 0x0C, offset: 4, bits: 4, writable: true, signed: false, },

    "T11" => Field { reg: 0x10, offset: 5, bits: 3, writable: true, signed: false, },
    "T12" => Field { reg: 0x11, offset: 5, bits: 3, writable: true, signed: false, },
    "T13" => Field { reg: 0x12, offset: 5, bits: 3, writable: true, signed: false, },

    "T21" => Field { reg: 0x10, offset: 1, bits: 4, writable: true, signed: false, },
    "T22" => Field { reg: 0x11, offset: 1, bits: 4, writable: true, signed: false, },
    "T23" => Field { reg: 0x12, offset: 1, bits: 4, writable: true, signed: false, },

    "T31" => Field { reg: 0x16, offset: 5, bits: 3, writable: true, signed: false, },
    "T32" => Field { reg: 0x17, offset: 5, bits: 3, writable: true, signed: false, },
    "T33" => Field { reg: 0x18, offset: 5, bits: 3, writable: true, signed: false, },

    "T41" => Field { reg: 0x16, offset: 1, bits: 4, writable: true, signed: false, },
    "T42" => Field { reg: 0x17, offset: 1, bits: 4, writable: true, signed: false, },
    "T43" => Field { reg: 0x18, offset: 1, bits: 4, writable: true, signed: false, },

    "Time Update" => Field { reg: 0x1C, offset: 0, bits: 8, writable: true, signed: false, },
    "LED Control" => Field { reg: 0x1D, offset: 0, bits: 3, writable: true, signed: false, },
    "Reset" => Field { reg: 0x2F, offset: 0, bits: 3, writable: true, signed: false, },
};