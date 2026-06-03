use phf::Map;  // Efficient map for register maps
use phf_macros::phf_map;

use embassy_time::Timer;

use crate::d_peripherals::chip::{Chip, CommProvider, ChipError};
use crate::d_peripherals::chip_implementations::Addressable;
use crate::d_peripherals::chip_map::{Field, FieldMapProvider};
use crate::{d_log::dlogger_common::DLogger, d_info};  // Logging

#[derive(Debug)]
pub enum TSL2591Error {
    NotFound,
    BusError(ChipError),
}

// Error conversion 
impl From<ChipError> for TSL2591Error {
    fn from(err: ChipError) -> Self {TSL2591Error::BusError(err)}
}

type TSLChip<COMM> = Chip<COMM, TSL2591FieldMap>;

pub struct TSL2591<COMM> {
    pub chip: TSLChip<COMM>,
    enabled: bool,
    pub interrupt_pin: Option<u16>,
}

impl <COMM> TSL2591<COMM> {
    pub const DEFAULT_I2C_ADDRESS: u8 = 0x29;
    pub const WHO_AM_I_REG: u8 = 0x12;
    pub const WHO_AM_I_VAL: u8 = 0x50;
}

impl <COMM> TSL2591<COMM> 
where
    COMM: CommProvider + Addressable,
{
    // Constructor for when a Chip is not given
    pub fn new_i2c<T: Into<Option<u8>>>(i2c: COMM, i2c_addr: T) -> Result<Self, TSL2591Error> {

        // Default i2c address
        let i2c_addr = i2c_addr.into();
        let i2c_addr = i2c_addr.unwrap_or(Self::DEFAULT_I2C_ADDRESS);

        let chip = TSLChip::new_i2c(i2c, i2c_addr);

        d_info!("Constructing new TSL2591 sensor");
        let this = Self {
                                       chip,
                                       enabled: false,
                                       interrupt_pin: Some(0),
                                      };
        Ok(this)
    }
}

impl <COMM> TSL2591<COMM> 
where
    COMM: CommProvider,
{
    // Initialize the system by checking the WHOAMI register
    pub async fn initialize(&mut self) -> Result<bool, TSL2591Error> {

        d_info!("Initializing TSL2591");
        DLogger::hold();

        self.reset().await?;
        Timer::after_millis(1000).await;
        DLogger::reset_hold();

        DLogger::hold();
        let id = self.chip.read_field_str("ID").await?; // ID is 0x12 (0xB2 with command bit) and should return 0x50
        DLogger::reset_hold();

        if id == Self::WHO_AM_I_VAL {
            d_info!("TSL connection successful");
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
        let hold_count = DLogger::get_hold_count();

        // Ignore a returned error, since ACK will likely not be returned
        let _ = self.chip.write_field_str("SRESET", 1).await;

        // Return hold count
        DLogger::set_hold(hold_count);
        
        
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

    // Clear interrupts from both types of interrupts
    pub async fn clear_interrupt(&self) -> Result<(), TSL2591Error> {
        d_info!("Clearing TSL2591 interrupt");
        DLogger::hold();
        self.chip.read_reg(0b11100111).await?;
        DLogger::release();
        Ok(())
    }

    // Enable persist interrupts
    pub async fn enable_p_interrupt(&self) -> Result<(), TSL2591Error> {
        d_info!("Enabling TSL2591 persist interrupt");
        DLogger::hold();
        self.chip.write_field_str("NPIEN", 0).await?;
        self.chip.write_field_str("AIEN", 1).await?;
        DLogger::release();
        Ok(())
    }

    // Enable no-persist interrupts
    pub async fn enable_np_interrupt(&self) -> Result<(), TSL2591Error> {
        d_info!("Enabling TSL2591 no-persist interrupt");
        DLogger::hold();
        self.chip.write_field_str("AIEN", 0).await?;
        self.chip.write_field_str("NPIEN", 1).await?;
        DLogger::release();
        Ok(())
    }

    // Disable interrupts
    pub async fn disable_interrupt(&self) -> Result<(), TSL2591Error> {
        d_info!("Disabling TSL2591 interrupts");
        DLogger::hold();
        self.chip.write_field_str("AIEN", 0).await?;
        self.chip.write_field_str("NPIEN", 0).await?;
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

    // Set persist interrupts
    // Persist - Multiple samples are needed to trigger the interrupt
    // HIGH and LOW interrupts are NOT hysteresis based
    // Pin will go LOW when the signal is below the LOW threshold or above the HIGH threshold
    pub async fn set_p_interrupt(&self, low_thresh: u16, high_thresh: u16, persist: u8) -> Result<(), TSL2591Error> {

        DLogger::hold();
        
        // Split into LSB and MSB
        let high_thresh_bytes = high_thresh.to_le_bytes();
        let high_thresh_lsb = high_thresh_bytes[0];
        let high_thresh_msb = high_thresh_bytes[1];

        let low_thresh_bytes = low_thresh.to_le_bytes();
        let low_thresh_lsb = low_thresh_bytes[0];
        let low_thresh_msb = low_thresh_bytes[1];

        // Write interrupt values
        self.chip.write_field_str("AIHTL", high_thresh_lsb).await?;      // Persist high threshold low byte
        self.chip.write_field_str("AIHTH", high_thresh_msb).await?;      // Persist high threshold high byte

        self.chip.write_field_str("AILTL", low_thresh_lsb).await?;     // Persist low threshold low byte
        self.chip.write_field_str("AILTH", low_thresh_msb).await?;     // Persist low threshold high byte

        // Write persist value - how many consecutive readings are needed
        self.chip.write_field_str("PERSIST", persist).await?;

        // Enable persist interrupts
        self.enable_p_interrupt().await?;

        // Clear interrupt
        self.clear_interrupt().await?;

        DLogger::release();
        
        Ok(())
    }

    // Read persist interrupt thresholds from registers
    // Returns (low_thresh, high_thresh) as u16
    pub async fn read_p_interrupt_thresholds(&self) -> Result<(u16, u16), TSL2591Error> {
        
        DLogger::hold();
        
        let mut buf = [0u8; 4];
        self.chip.read_regs_str("AILTL", &mut buf).await?;
        let low_thresh  = u16::from_le_bytes([buf[0], buf[1]]);
        let high_thresh = u16::from_le_bytes([buf[2], buf[3]]);

        DLogger::release();

        Ok((low_thresh, high_thresh))
    }

    // Read no-persist interrupt thresholds from registers
    // Returns (low_thresh, high_thresh) as u16
    pub async fn read_np_interrupt_thresholds(&self) -> Result<(u16, u16), TSL2591Error> {
        
        DLogger::hold();
        
        let mut buf = [0u8; 4];
        self.chip.read_regs_str("NPAILTL", &mut buf).await?;
        let low_thresh  = u16::from_le_bytes([buf[0], buf[1]]);
        let high_thresh = u16::from_le_bytes([buf[2], buf[3]]);

        DLogger::release();

        Ok((low_thresh, high_thresh))
    }

    // Set no-persist interrupts
    // No-persist - Only a single sample is needed to trigger the interrupt
    // HIGH and LOW interrupts are NOT hysteresis based
    // Pin will go LOW when the signal is below the LOW threshold or above the HIGH threshold
    pub async fn set_np_interrupt(&self, low_thresh: u16, high_thresh: u16) -> Result<(), TSL2591Error> {

        DLogger::hold();
        
        // Split into LSB and MSB
        let high_thresh_bytes = high_thresh.to_le_bytes();
        let high_thresh_lsb = high_thresh_bytes[0];
        let high_thresh_msb = high_thresh_bytes[1];

        let low_thresh_bytes = low_thresh.to_le_bytes();
        let low_thresh_lsb = low_thresh_bytes[0];
        let low_thresh_msb = low_thresh_bytes[1];

        // Write interrupt values
        self.chip.write_field_str("NPAIHTL", high_thresh_lsb).await?;      // No persist high threshold low byte
        self.chip.write_field_str("NPAIHTH", high_thresh_msb).await?;      // No persist high threshold high byte

        self.chip.write_field_str("NPAILTL", low_thresh_lsb).await?;     // No persist low threshold low byte
        self.chip.write_field_str("NPAILTH", low_thresh_msb).await?;     // No persist low threshold high byte

        // Enable no-persist interrupts
        self.enable_np_interrupt().await?;

        // Clear interrupt
        self.clear_interrupt().await?;

        DLogger::release();
        
        Ok(())
    }

    // Read sensor output
    // Gives 3 outputs, one for each type of output
    // Full spectrum, IR spectrum, and visibible spectrum
    pub async fn read_full_luminosity(&mut self) -> Result<(u16, u16, u16), TSL2591Error> {
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

        d_info!("Light FS: {}", light_fs);
        d_info!("Light IR: {}", light_ir);
        d_info!("Light VS: {}", light_vs);

        Ok((light_fs, light_ir, light_vs))

    }
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
        let reg = field.reg | Self::CMD_BIT | Self::TRANSACTION;
        let new_field = Field {reg: reg, ..*field};

        Some(new_field)
    }
    
    fn get_write_field(name: &str) -> Option<Field> {
        TSL2591FieldMap::get_read_field(name)
    }
}

static FIELD_MAP: Map<&'static str, Field> = phf_map! {
    // Enable Register
    "ENABLE" => Field { reg: 0x00, offset: 0, bits: 8, writable: true, signed: false, },
    "NPIEN" => Field { reg: 0x00, offset: 7, bits: 1, writable: true, signed: false, },
    "SAI" => Field { reg: 0x00, offset: 6, bits: 1, writable: true, signed: false, },
    "AIEN" => Field { reg: 0x00, offset: 4, bits: 1, writable: true, signed: false, },
    "AEN" => Field { reg: 0x00, offset: 1, bits: 1, writable: true, signed: false, },
    "PON" => Field { reg: 0x00, offset: 0, bits: 1, writable: true, signed: false, },

    // Control Register
    "CONTROL" => Field { reg: 0x01, offset: 0, bits: 8, writable: true, signed: false, },
    "ATIME" => Field { reg: 0x01, offset: 0, bits: 3, writable: true, signed: false, },
    "AGAIN" => Field { reg: 0x01, offset: 4, bits: 2, writable: true, signed: false, },
    "SRESET" => Field { reg: 0x01, offset: 7, bits: 1, writable: true, signed: false, },

    // ALS Data Register
    "C0DATAL" => Field { reg: 0x14, offset: 0, bits: 8, writable: true, signed: false, },
    "C0DATAH" => Field { reg: 0x15, offset: 0, bits: 8, writable: true, signed: false, },
    "C1DATAL" => Field { reg: 0x16, offset: 0, bits: 8, writable: true, signed: false, },
    "C1DATAH" => Field { reg: 0x17, offset: 0, bits: 8, writable: true, signed: false, },

    // Interrupts and Persists
    "PERSIST" => Field { reg: 0x0C, offset: 0, bits: 4, writable: true, signed: false, },
    "AILTL" => Field { reg: 0x04, offset: 0, bits: 8, writable: true, signed: false, },
    "AILTH" => Field { reg: 0x05, offset: 0, bits: 8, writable: true, signed: false, },
    "AIHTL" => Field { reg: 0x06, offset: 0, bits: 8, writable: true, signed: false, },
    "AIHTH" => Field { reg: 0x07, offset: 0, bits: 8, writable: true, signed: false, },
    "NPAILTL" => Field { reg: 0x08, offset: 0, bits: 8, writable: true, signed: false, },
    "NPAILTH" => Field { reg: 0x09, offset: 0, bits: 8, writable: true, signed: false, },
    "NPAIHTL" => Field { reg: 0x0A, offset: 0, bits: 8, writable: true, signed: false, },
    "NPAIHTH" => Field { reg: 0x0B, offset: 0, bits: 8, writable: true, signed: false, },

    // Status Register
    "STATUS" => Field { reg: 0x13, offset: 0, bits: 8, writable: true, signed: false, },
    "NPINTR" => Field { reg: 0x13, offset: 5, bits: 1, writable: true, signed: false, },
    "AINT" => Field { reg: 0x13, offset: 4, bits: 1, writable: true, signed: false, },
    "AVALID" => Field { reg: 0x13, offset: 0, bits: 1, writable: true, signed: false, },

    // ID Register
    "ID" => Field { reg: 0x12, offset: 0, bits: 8, writable: true, signed: false, },
};  
