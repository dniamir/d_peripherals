use core::marker::PhantomData;

use crate::d_peripherals::chip_implementations::{Addressable, CommError};
use crate::d_peripherals::chip_map;
use crate::{d_log::dlogger::DLogger, d_info};  // Logging


/// Define some error types
#[derive(Debug)]
pub enum ChipError {
    FieldNotFound,
    BusError(CommError),
}

// Error conversion 
impl From<CommError> for ChipError {
    fn from(err: CommError) -> Self {ChipError::BusError(err)}
}

// Generic I2C trait definitions
#[allow(async_fn_in_trait)]  // Have to surpress warning, or else have to explicitely define output as a future, which is cumbersome
pub trait CommProvider {
    async fn write_read(&self, regs: &[u8], reg_vals: &mut [u8]) -> Result<(), CommError>;  
    async fn write(&self, regs: &[u8], reg_vals: &[u8]) -> Result<(), CommError>;
}

// Struct definition
pub struct Chip<COMM, MAP=chip_map::NoFieldMap> {
    pub comm: COMM,  // Can be a mutex (supported) or an I2C bus (not supported yet)
    pub _map: PhantomData<MAP>,
}

impl <I2C, MAP> Chip<I2C, MAP> 
where
    I2C: CommProvider + Addressable,
{
    pub fn new_i2c(mut i2c: I2C, i2c_addr: u8) -> Self {
        i2c.set_address(i2c_addr);
        Self {comm: i2c, _map: PhantomData }
    }
}

// MUTEX implementations for I2C generic - Any MAP
impl<COMM, MAP,> Chip<COMM, MAP> 
where
    COMM: CommProvider,
{

    // Basic function to write multiple registers
    pub async fn write_regs(&self, reg: u8, reg_vals: &[u8]) -> Result<(), ChipError> {

        // Log writes
        reg_vals.iter().enumerate().for_each(|(i, &val)| {
            d_info!("Write Registers: 0x{:X}, {:b}, 0x{:X}, {}", reg + i as u8, val, val, val);
        });

        // Perform write
        self.comm.write(&[reg], reg_vals).await?;
        Ok(())
    }

    // Basic function to read multiple registers
    pub async fn read_regs(&self, reg: u8, reg_vals: &mut [u8]) -> Result<(), ChipError> {
        
        // Perform read
        self.comm.write_read(&[reg], reg_vals).await?;

        // Log reads
        reg_vals.iter().enumerate().for_each(|(i, &val)| {
            d_info!("Read Registers: 0x{:X}, {:b}, 0x{:X}, {}", reg + i as u8, val, val, val);
        });
        Ok(())
    }

    // Basic function to write a single 8 bit register
    pub async fn write_reg(&self, reg: u8, reg_val: u8) -> Result<(), ChipError> {
            
        // Log write
        d_info!("Write Register: 0x{:X}, {:b}, 0x{:X}, {}", reg, reg_val, reg_val, reg_val);

        // Read reg
        DLogger::hold();
        self.comm.write(&[reg], &[reg_val]).await?;
        DLogger::release();

        Ok(())
    }

    // Basic function to read a 8 bit single register
    pub async fn read_reg(&self, reg: u8) -> Result<u8, ChipError> {

        let mut reg_vals = [0];
    
        // Read reg
        DLogger::hold();
        self.read_regs(reg, &mut reg_vals).await?;
        DLogger::release();

        let reg_val = reg_vals[0];
        d_info!("Read Register: 0x{:X}, {:b}, 0x{:X}, {}", reg, reg_val, reg_val, reg_val);
        Ok(reg_val)
    }

    // Basic function to write a single 8 bit register
    pub async fn write_reg16(&self, reg: u8, reg_val: u16) -> Result<(), ChipError> {
            
        // Log write
        d_info!("Write Register: 0x{:X}, {:b}, 0x{:X}, {}", reg, reg_val, reg_val, reg_val);

        // Read reg
        DLogger::hold();
        let write_buff: [u8; 2] = reg_val.to_be_bytes();
        self.write_regs(reg, &write_buff).await?;
        DLogger::release();

        Ok(())
    }

    // Basic function to read a 8 bit single register
    pub async fn read_reg16(&self, reg: u8) -> Result<u16, ChipError> {
    
        // Read reg
        DLogger::hold();
        let mut read_buff = [0u8; 2];
        self.read_regs(reg, &mut read_buff).await?;
        let reg_val = u16::from_be_bytes(read_buff);
        DLogger::release();

        // Log value
        d_info!("Read Register: 0x{:X}, {:b}, 0x{:X}, {}", reg, reg_val, reg_val, reg_val);
        Ok(reg_val)
    }

    // Function to write a single 8 bit field using the field details
    pub async fn write_field(&self, field_reg: u8, field_offset: u8, field_bits: u8, field_val: u8) -> Result<(), ChipError> {

        // Read the register
        DLogger::hold();
        let curr_field_val = self.read_reg(field_reg).await?;
        DLogger::release();

        // Clear the field
        let mask = ((1u32 << field_bits) - 1) << field_offset;
        let cleared = (curr_field_val as u32) & !mask;
        let inserted = ((field_val as u32) << field_offset) & mask;
        let field_val = (cleared | inserted) as u8;
    
        // Write the register
        d_info!("Write Field: 0x{:X}, {:b}, 0x{:X}, {}", field_reg, field_val, field_val, field_val);
        DLogger::hold();
        self.write_reg(field_reg, field_val).await?;
        DLogger::release();

        Ok(())
    }

    // Function to read a single 8 bit field using the field details
    pub async fn read_field(&self, field_reg: u8, field_offset: u8, field_bits: u8) -> Result<u8, ChipError> {
        
        // Read the field
        DLogger::hold();
        let reg_val = self.read_reg(field_reg).await?;
        DLogger::release();

        // Get field value from masking
        let mask = (((1u32 << field_bits) - 1) << field_offset) as u8;
        let field_val = (reg_val & mask) >> field_offset;

        d_info!("Read Field: 0x{:X}, {:b}, 0x{:X}, {}", field_reg, field_val, field_val, field_val);

        Ok(field_val)
    }

    // Function to write a single fiel that holds a 16 bit value using the field details
    pub async fn write_field16(&self, field_reg: u8, field_offset: u8, field_bits: u8, field_val: u16) -> Result<(), ChipError> {

        // Read the register
        DLogger::hold();
        let reg_val = self.read_reg16(field_reg).await?;
        DLogger::release();

        // Clear the field
        let mask = ((1u32 << field_bits) - 1) << field_offset;
        let cleared = (reg_val as u32) & !mask;
        let inserted = ((field_val as u32) << field_offset) & mask;
        let field_val = (cleared | inserted) as u16;
    
        // Write the register
        d_info!("Write Field: 0x{:X}, {:b}, 0x{:X}, {}", field_reg, field_val, field_val, field_val);
        DLogger::hold();
        self.write_reg16(field_reg, field_val).await?;
        DLogger::release();

        Ok(())
    }

    // Function to read a 16 bit field using the field details
    pub async fn read_field16(&self, field_reg: u8, field_offset: u8, field_bits: u8) -> Result<u16, ChipError> {
        
        // Read the field
        DLogger::hold();
        let reg_val = self.read_reg16(field_reg).await?;
        DLogger::release();

        // Get field value from masking
        let mask = (((1u32 << field_bits) - 1) << field_offset) as u16;
        let field_val = (reg_val & mask) >> field_offset;

        d_info!("Read Field: 0x{:X}, {:b}, 0x{:X}, {}", field_reg, field_val, field_val, field_val);

        Ok(field_val)
    }
}

// MUTEX implementations for I2C generic - Defined Map using chip_map
impl<I2C, MAP,> Chip<I2C, MAP> 
where
    I2C: CommProvider,
    MAP: chip_map::FieldMapProvider,
{

    // Basic function to read multiple 8 bit registers using a string name
    pub async fn read_regs_str(&self, reg_str: &str, reg_vals: &mut [u8]) -> Result<(), ChipError> {
        
        // Get field details
        let reg_dets = MAP::get_read_field(reg_str).ok_or(ChipError::FieldNotFound)?;
        
        // Read the registers
        self.read_regs(reg_dets.reg, reg_vals).await?;
        Ok(())
    }

    // Function to read a single 8 bit egister using a string name
    pub async fn read_reg_str(&self, reg_str: &str) -> Result<u8, ChipError> {
        
        // Get field details
        let reg_dets = MAP::get_read_field(reg_str).ok_or(ChipError::FieldNotFound)?;
        
        // Just read the raw register value
        DLogger::hold();
        let reg_val = self.read_reg(reg_dets.reg).await?;
        DLogger::release();
        d_info!("Read Register: {}, {:b}, 0x{:X}, {}", reg_str, reg_val, reg_val, reg_val);
        Ok(reg_val)
    }

    // Function to write a single 8 bit register using a string name
    pub async fn write_reg_str(&self, reg_str: &str, reg_val: u8) -> Result<(), ChipError> {
        
        // Get register details
        let reg_dets = MAP::get_write_field(reg_str).ok_or(ChipError::FieldNotFound)?;
        
        // Write the register
        d_info!("Write Register: {}, {:b}, 0x{:X}, {}", reg_str, reg_val, reg_val, reg_val);
        DLogger::hold();
        self.write_reg(reg_dets.reg, reg_val).await?;
        DLogger::release();
        Ok(())
    }

    // Function to read a single 8 bit field using a string name
    pub async fn read_field_str(&self, field: &str) -> Result<u8, ChipError> {

        // Get field details
        let field_dets = MAP::get_read_field(field).ok_or(ChipError::FieldNotFound)?;
        let field_reg: u8 = field_dets.reg as u8;
        let field_offset: u8 = field_dets.offset as u8;
        let field_bits: u8 = field_dets.bits as u8;

        // Read the field
        DLogger::hold();
        let field_val = self.read_field(field_reg, field_offset, field_bits).await.unwrap();
        DLogger::release();

        d_info!("Read Field: {}, {:b}, 0x{:X}, {}", field, field_val, field_val, field_val);

        Ok(field_val)
    }

    // Function to write a single 8 bit field using a string name
    pub async fn write_field_str(&self, field: &str, field_val: u8) -> Result<(), ChipError> {
       
        // Get field details
        let field_dets = MAP::get_write_field(field).ok_or(ChipError::FieldNotFound)?;
        let field_reg: u8 = field_dets.reg as u8;
        let field_offset: u8 = field_dets.offset as u8;
        let field_bits: u8 = field_dets.bits as u8;

        // Write the field
        d_info!("Write Field: {}, {:b}, 0x{:X}, {}", field, field_val, field_val, field_val);
        DLogger::hold();
        self.write_field(field_reg, field_offset, field_bits, field_val).await?;
        DLogger::release();

        Ok(())
    }

    // Function to read a single 16 bit field using a string name
    pub async fn read_field_st16(&self, field: &str) -> Result<u16, ChipError> {

        // Get field details
        let field_dets = MAP::get_read_field(field).ok_or(ChipError::FieldNotFound)?;
        let field_reg: u8 = field_dets.reg as u8;
        let field_offset: u8 = field_dets.offset as u8;
        let field_bits: u8 = field_dets.bits as u8;

        // Read the field
        DLogger::hold();
        let field_val = self.read_field16(field_reg, field_offset, field_bits).await.unwrap();
        DLogger::release();

        d_info!("Read Field: {}, {:b}, 0x{:X}, {}", field, field_val, field_val, field_val);

        Ok(field_val)
    }

    // Function to write a single 16 bit field using a string name
    pub async fn write_field_str16(&self, field: &str, field_val: u16) -> Result<(), ChipError> {
       
        // Get field details
        let field_dets = MAP::get_write_field(field).ok_or(ChipError::FieldNotFound)?;
        let field_reg: u8 = field_dets.reg as u8;
        let field_offset: u8 = field_dets.offset as u8;
        let field_bits: u8 = field_dets.bits as u8;

        // Write the field
        d_info!("Write Field: {}, {:b}, 0x{:X}, {}", field, field_val, field_val, field_val);
        DLogger::hold();
        self.write_field16(field_reg, field_offset, field_bits, field_val).await?;
        DLogger::release();

        Ok(())
    }

    // Currently has issues because we need to define a u8 slice from a u16 slice, which is not easily possible.
    // // Basic function to read multiple 16bit registers using a string name
    // pub async fn read_regs_str16(&self, reg_str: &str, reg_vals: &mut [u16]) -> Result<(), ChipError> {
        
    //     // Get field details
    //     let reg_dets = MAP::get_read_field(reg_str).ok_or(ChipError::FieldNotFound)?;
        
    //     // Read the registers
    //     let mut read_buff = [0u8; 2 * reg_vals.len() as usize];
    //     self.read_regs(reg, &mut read_buff).await?;

    //     // Convert [u8] into [u16]
    //     for (i, chunk) in read_buff.chunks_exact(2).enumerate() {
    //         reg_vals[i] = u16::from_be_bytes(chunk.try_into().unwrap());
    //     }
    //     Ok(())
    // }

    // Function to read a single 16bit register using a string name
    pub async fn read_reg_str16(&self, reg_str: &str) -> Result<u16, ChipError> {
        
        // Get field details
        let reg_dets = MAP::get_read_field(reg_str).ok_or(ChipError::FieldNotFound)?;
        
        // Just read the raw register value
        DLogger::hold();
        let reg_val = self.read_reg16(reg_dets.reg).await?;
        DLogger::release();
        d_info!("Read Register: {}, {:b}, 0x{:X}, {}", reg_str, reg_val, reg_val, reg_val);
        Ok(reg_val)
    }

    // Function to write a single 16bit register using a string name
    pub async fn write_reg_str16(&self, reg_str: &str, reg_val: u16) -> Result<(), ChipError> {
        
        // Get register details
        let reg_dets = MAP::get_write_field(reg_str).ok_or(ChipError::FieldNotFound)?;
        
        // Write the register
        d_info!("Write Register: {}, {:b}, 0x{:X}, {}", reg_str, reg_val, reg_val, reg_val);
        DLogger::hold();
        self.write_reg16(reg_dets.reg, reg_val).await?;
        DLogger::release();
        Ok(())
    }
}
