use core::marker::PhantomData;

use crate::embassy_hal::twim::Error as TwimError;
use crate::d_peripherals::chip_map;
use crate::{d_log::dlogger::DLogger, d_info};  // Logging


/// Define some error types
#[derive(Debug)]
pub enum I2CError {
    NotFound,
    I2CError(TwimError),
}

// Error conversion 
impl From<TwimError> for I2CError {
    fn from(err: TwimError) -> Self {I2CError::I2CError(err)}
}

// Generic I2C trait definitions
#[allow(async_fn_in_trait)]  // Have to surpress warning, or else have to explicitely define output as a future, which is cumbersome
pub trait I2CProvider {
    async fn write_read(&self, i2c_address: u8, reg: u8, reg_vals: &mut [u8]) -> Result<(), I2CError>;  
    async fn write(&self, i2c_address: u8, reg: u8, reg_val: u8) -> Result<(), I2CError>;
}

// Struct definition
pub struct Chip<I2C, MAP=chip_map::NoFieldMap> {
    pub i2c: I2C,  // Can be a mutex (supported) or an I2C bus (not supported)
    pub i2c_addr: u8,
    pub _map: PhantomData<MAP>,
}

impl <I2C> Chip<I2C, chip_map::NoFieldMap> {
    pub fn new_generic(i2c: I2C, i2c_addr: u8) -> Self {
        Self { i2c, i2c_addr: i2c_addr, _map: PhantomData }
    }
}

// MUTEX implementations for I2C generic - Any MAP
impl<I2C, MAP,> Chip<I2C, MAP> 
where
    I2C: I2CProvider,
{

    pub async fn read_regs(&self, reg: u8, reg_values: &mut [u8]) -> Result<(), I2CError> {
        // Basic function to read multiple registers
        self.i2c.write_read(self.i2c_addr, reg, reg_values).await?;

        let mut reg_idx = 0;
        for reg_value in reg_values.iter() {
            d_info!("Read Register: 0x{=u8:X}, {=u8:b}, 0x{=u8:X}, {}", reg + reg_idx, reg_value, reg_value, reg_value);
            reg_idx += 1;
        }
        Ok(())
    }

    pub async fn write_reg(&self, reg: u8, reg_val: u8) -> Result<(), I2CError> {
        // Basic function to write a single register
        self.i2c.write(self.i2c_addr, reg, reg_val).await?;
        d_info!("Write Register: 0x{=u8:X}, {=u8:b}, 0x{=u8:X}, {}", reg, reg_val, reg_val, reg_val);
        Ok(())
    }

    pub async fn read_reg(&self, reg: u8) -> Result<u8, I2CError> {
        // Basic function to read a single register
        let mut reg_vals = [0];
    
        // Read reg
        DLogger::hold();
        self.read_regs(reg, &mut reg_vals).await?;
        DLogger::release();

        let reg_value = reg_vals[0];
        d_info!("Read Register: 0x{=u8:X}, {=u8:b}, 0x{=u8:X}, {}", reg, reg_value, reg_value, reg_value);
        Ok(reg_value)
    }
}

// MUTEX implementations for I2C generic - Defined Map using chip_map
impl<I2C, MAP,> Chip<I2C, MAP> 
where
    I2C: I2CProvider,
    MAP: chip_map::FieldMapProvider,
{

    pub async fn read_regs_str(&self, reg_str: &str, reg_values: &mut [u8]) -> Result<(), I2CError> {
        // Basic function to read multiple registers using a string name
        
        // Get field details
        let reg_dets = MAP::get_field(reg_str).ok_or(I2CError::NotFound)?;
        
        // Read the registers
        self.read_regs(reg_dets.reg, reg_values).await?;
        Ok(())
    }

    pub async fn read_reg_str(&self, reg_str: &str) -> Result<u8, I2CError> {
        // Function to read a single register using a string name
        
        // Get field details
        let reg_dets = MAP::get_field(reg_str).ok_or(I2CError::NotFound)?;
        
        // Just read the raw register value
        DLogger::hold();
        let reg_value = self.read_reg(reg_dets.reg).await?;
        DLogger::release();
        d_info!("Read Register: {}, {=u8:b}, 0x{=u8:X}, {}", reg_str, reg_value, reg_value, reg_value);
        Ok(reg_value)
    }

    pub async fn write_reg_str(&self, reg_str: &str, reg_val: u8) -> Result<(), I2CError> {
        // Function to write a single register using a string name
        
        // Get register details
        let reg_dets = MAP::get_field(reg_str).ok_or(I2CError::NotFound)?;
        
        // Write the register
        DLogger::hold();
        self.write_reg(reg_dets.reg, reg_val).await?;
        DLogger::release();
        d_info!("Write Register: {}, {=u8:b}, 0x{=u8:X}, {}", reg_str, reg_val, reg_val, reg_val);
        Ok(())
    }

    pub async fn read_field(&self, field: &str) -> Result<u8, I2CError> {
        // Function to read a single field using a string name

        // Get field details
        let field_dets = MAP::get_field(field).ok_or(I2CError::NotFound)?;
        let field_reg: u8 = field_dets.reg as u8;
        let field_offset: u8 = field_dets.offset as u8;
        let field_bits: u8 = field_dets.bits as u8;

        // Read the field
        DLogger::hold();
        let reg_val = self.read_reg(field_reg as u8).await?;
        DLogger::release();

        // Get field value from masking
        let mask = (((1u32 << field_bits) - 1) << field_offset) as u8;
        let field_val = (reg_val & mask) >> field_offset;

        d_info!("Read Field: {}, {=u8:b}, 0x{=u8:X}, {}", field, field_val, field_val, field_val);

        Ok(field_val)
    }

    pub async fn write_field(&self, field: &str, field_val: u8) -> Result<(), I2CError> {
        // Function to write a single field using a string name
       
        // Get field details
        let field_dets = MAP::get_field(field).ok_or(I2CError::NotFound)?;
        let field_reg: u8 = field_dets.reg as u8;
        let field_offset: u8 = field_dets.offset as u8;
        let field_bits: u8 = field_dets.bits as u8;

        // Read the register
        DLogger::hold();
        let curr_field_val = self.read_reg(field_reg).await?;

        // Clear the field
        let mask = ((1u32 << field_bits) - 1) << field_offset;
        let cleared = (curr_field_val as u32) & !mask;
        let inserted = ((field_val as u32) << field_offset) & mask;
        let field_val = (cleared | inserted) as u8;
    
        // Write the register
        self.write_reg(field_reg, field_val).await?;
        DLogger::release();

        d_info!("Write Field: {}, {=u8:b}, 0x{=u8:X}, {}", field, field_val, field_val, field_val);

        Ok(())
    }
}
