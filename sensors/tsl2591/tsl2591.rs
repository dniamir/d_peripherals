use core::marker::PhantomData;

use phf::Map;  // Efficient map for register maps
use phf_macros::phf_map;

use embassy_time::Timer;

use crate::d_peripherals::chip::{Chip, CommProvider, ChipError};
use crate::d_peripherals::chip_implementations::Addressable;
use crate::d_peripherals::chip_map::{Field, FieldMapProvider};
use crate::{d_log::dlogger::DLogger, d_info};  // Logging

#[derive(Debug)]
pub enum TSL2591Error {
    NotFound,
    BusError(ChipError),
}

// Error conversion 
impl From<ChipError> for TSL2591Error {
    fn from(err: ChipError) -> Self {TSL2591Error::BusError(err)}
}

pub struct TSL2591<I2C, ChipGeneric=Chip<I2C, TSL2591FieldMap>> {
    pub chip: ChipGeneric,
    enabled: bool,
    pub _i2c: PhantomData<I2C>,
    pub interrupt_pin: Option<u16>,
}

impl <I2C, ChipGeneric> TSL2591<I2C, ChipGeneric> {
    pub const DEFAULT_I2C_ADDRESS: u8 = 0x29;
    pub const WHO_AM_I_REG: u8 = 0x12;
    pub const WHO_AM_I_VAL: u8 = 0x50;
}

impl <I2C> TSL2591<I2C, Chip<I2C, TSL2591FieldMap>> 
where
    I2C: CommProvider + Addressable,
{
    // Constructor for when a Chip is not given
    pub fn new_i2c<T: Into<Option<u8>>>(i2c: I2C, i2c_addr: T) -> Result<Self, TSL2591Error> {

        // Default i2c address
        let i2c_addr = i2c_addr.into();
        let i2c_addr = i2c_addr.unwrap_or(Self::DEFAULT_I2C_ADDRESS);

        let chip: Chip<I2C, TSL2591FieldMap> = Chip::new_i2c(i2c, i2c_addr);

        d_info!("Constructing new TSL2591 sensor");
        let this = Self {
                                       chip,
                                       enabled: false,
                                       _i2c: PhantomData,
                                       interrupt_pin: Some(0),
                                      };
        Ok(this)
    }
}

impl <I2C> TSL2591<I2C, Chip<I2C, TSL2591FieldMap>> 
where
    I2C: CommProvider,
{
    // Initialize the system by checking the WHOAMI register
    pub async fn initialize(&mut self) -> Result<bool, TSL2591Error> {

        d_info!("Initializing TSL2591");
        DLogger::hold();

        self.reset().await?;
        // Timer::after_millis(200).await;  // WHOAMI will not read if there's a delay - but this is already built in

        let id = self.chip.read_field_str("ID").await?; // Assuming 0x12 is ID register address

        DLogger::release();

        if id == Self::WHO_AM_I_VAL {
            d_info!("TSL connection suffessful");
            Ok(true)
        } else {
            d_info!("TSL connection failed");
            Ok(false)
        }
    }

    // Reset the sensor
    pub async fn reset(&self) -> Result<(), TSL2591Error> {
        d_info!("Resetting TSL2591");
        DLogger::hold();
        self.chip.write_field_str("SRESET", 1).await?;
        DLogger::release();
        Ok(())
    }

    // Enable the sensor
    pub async fn enable(&mut self) -> Result<(), TSL2591Error> {
        d_info!("Enabling TSL2591");
        DLogger::hold();
        self.chip.write_field_str("PON", 1).await?;
        self.chip.write_field_str("AEN", 1).await?;
        self.enabled = true;
        DLogger::release();
        Ok(())
    }

    // Disable the sensor
    pub async fn disable(&mut self) -> Result<(), TSL2591Error> {
        d_info!("Disabling TSL2591");
        DLogger::hold();
        self.chip.write_field_str("PON", 0).await?;
        self.chip.write_field_str("AEN", 0).await?;
        self.enabled = false;
        DLogger::release();
        Ok(())
    }

    // Clear interrupts
    pub async fn clear_interrupt(&self) -> Result<(), TSL2591Error> {
        d_info!("Clearing TSL2591 interrupt");
        DLogger::hold();
        self.chip.read_reg(0b11100111).await?;
        DLogger::release();
        Ok(())
    }

    // Enable interrupts
    pub async fn enable_interrupt(&self) -> Result<(), TSL2591Error> {
        d_info!("Enabling TSL2591 interrupt");
        DLogger::hold();
        self.chip.read_reg(0b11100100).await?;
        DLogger::release();
        Ok(())
    }

    // Basic configuration for the sensor
    pub async fn basic_config(&self) -> Result<(), TSL2591Error> {
        d_info!("Initializing TSL2591 basic configuration");
        DLogger::hold();
        self.chip.write_field_str("AGAIN", 0b01).await?;
        self.chip.write_field_str("ATIME", 0b010).await?;
        DLogger::release();
        Ok(())
    }

    // Read sensor output
    // Gives 3 outputs, one for each type of output
    // Full spectrum, IR spectrum, and visibible spectrum
    pub async fn read_full_luminosity(&mut self) -> Result<(u16, u16, u16), TSL2591Error> {
        d_info!("Reading full luminosity from TSL2591");
        DLogger::hold();
        let mut disable_after = false;

        if  !self.enabled {
            self.enable().await?;
            disable_after = true;
            Timer::after_millis(600).await;  // Maximum integration time
        }

        // CHAN0 must be read before CHAN1
        // See: https://forums.adafruit.com/viewtopic.php?f=19&t=124176
        let mut register_out = [0u8; 4];
        self.chip.read_regs_str("C0DATAL", &mut register_out).await?;

        let light_fs = u16::from_be_bytes([register_out[1], register_out[0]]);
        let light_ir = u16::from_be_bytes([register_out[3], register_out[2]]);
        let light_vs = light_fs - light_ir;
        
        // Disable if originally disabled
        if disable_after { self.disable().await?; }

        DLogger::release();

        d_info!("Light FS: {}, Light IR: {}, Light VS: {}", light_fs, light_ir, light_vs);

        Ok((light_fs, light_ir, light_vs))

    }

    // fn write_tsl_field(&mut self, reg_str: &str, reg_val: u8) -> Result<(), E> {
    //     let reg_dets = TSL2591FieldMap::get_field(reg_str)?;
    //     self.chip.write_field(self::COMMAND_BIT | reg_dets.reg, val, reg_dets.offset, reg_dets.bits);
    // }

    // fn read_tsl_field(&mut self, reg: u8) -> Result<u8, E> {
    //     let mut buffer = [0u8; 1];
    //     self.chip.write_read(self.address, &[Self::COMMAND_BIT | reg], &mut buffer)?;
    //     Ok(buffer[0])
    // }
}

pub struct TSL2591FieldMap;

impl TSL2591FieldMap {
    const CMD_BIT: u8 = 0b1 << 7;
    const TRANSACTION: u8 = 0b01 << 5;
}

impl FieldMapProvider for TSL2591FieldMap {

    fn get_read_field(name: &str) -> Option<Field> {
        let field = FIELD_MAP.get(name)?;

        // The *field tells rust to copy the remaining fields
        let new_field = Field {reg: field.reg | Self::CMD_BIT | Self::TRANSACTION, ..*field};

        Some(new_field)
    }
    
    fn get_write_field(name: &str) -> Option<Field> {
        TSL2591FieldMap::get_read_field(name)
    }
}

static FIELD_MAP: Map<&'static str, Field> = phf_map! {
    // Enable Register
    "ENABLE" => Field { reg: 0x00, offset: 0, bits: 8, writable: true },
    "NPIEN" => Field { reg: 0x00, offset: 7, bits: 1, writable: true },
    "SAI" => Field { reg: 0x00, offset: 6, bits: 1, writable: true },
    "AIEN" => Field { reg: 0x00, offset: 4, bits: 1, writable: true },
    "AEN" => Field { reg: 0x00, offset: 1, bits: 1, writable: true },
    "PON" => Field { reg: 0x00, offset: 0, bits: 1, writable: true },

    // Control Register
    "CONTROL" => Field { reg: 0x01, offset: 0, bits: 8, writable: true },
    "ATIME" => Field { reg: 0x01, offset: 0, bits: 3, writable: true },
    "AGAIN" => Field { reg: 0x01, offset: 4, bits: 2, writable: true },
    "SRESET" => Field { reg: 0x01, offset: 7, bits: 1, writable: true },

    // ALS Data Register
    "C0DATAL" => Field { reg: 0x14, offset: 0, bits: 8, writable: false },
    "C0DATAH" => Field { reg: 0x15, offset: 0, bits: 8, writable: false },
    "C1DATAL" => Field { reg: 0x16, offset: 0, bits: 8, writable: false },
    "C1DATAH" => Field { reg: 0x17, offset: 0, bits: 8, writable: false },

    // Interrupts and Persists
    "PERSIST" => Field { reg: 0x0C, offset: 0, bits: 4, writable: true },
    "AILTL" => Field { reg: 0x04, offset: 0, bits: 8, writable: true },
    "AILTH" => Field { reg: 0x05, offset: 0, bits: 8, writable: true },
    "AIHTL" => Field { reg: 0x06, offset: 0, bits: 8, writable: true },
    "AIHTH" => Field { reg: 0x07, offset: 0, bits: 8, writable: true },
    "NPAILTL" => Field { reg: 0x08, offset: 0, bits: 8, writable: true },
    "NPAILTH" => Field { reg: 0x09, offset: 0, bits: 8, writable: true },
    "NPAIHTL" => Field { reg: 0x0A, offset: 0, bits: 8, writable: true },
    "NPAIHTH" => Field { reg: 0x0B, offset: 0, bits: 8, writable: true },

    // Status Register
    "STATUS" => Field { reg: 0x13, offset: 0, bits: 8, writable: true },
    "NPINTR" => Field { reg: 0x13, offset: 5, bits: 1, writable: true },
    "AINT" => Field { reg: 0x13, offset: 4, bits: 1, writable: true },
    "AVALID" => Field { reg: 0x13, offset: 0, bits: 1, writable: true },

    // ID Register
    "ID" => Field { reg: 0x12, offset: 0, bits: 8, writable: false },
};  
