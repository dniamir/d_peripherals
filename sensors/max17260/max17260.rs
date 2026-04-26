use phf::Map;  // Efficient map for register maps
use phf_macros::phf_map;

use crate::d_peripherals::chip::{Chip, CommProvider, ChipError};
use crate::d_peripherals::chip_implementations::Addressable;
use crate::d_peripherals::chip_map::{Field, FieldMapProvider};
use crate::{d_log::dlogger_common::DLogger, d_info};

#[derive(Debug)]
pub enum MAX17260Error {
    NotFound,
    BusError(ChipError),
}

// Error conversion 
impl From<ChipError> for MAX17260Error {
    fn from(err: ChipError) -> Self {MAX17260Error::BusError(err)}
}

type MAX17260Chip<COMM> = Chip<COMM, MAX17260FieldMap>;

pub struct MAX17260<COMM> {
    pub chip: MAX17260Chip<COMM>,
    pub interrupt_pin: Option<u16>,
}

impl <COMM> MAX17260<COMM> {
    pub const DEFAULT_I2C_ADDRESS: u8 = 0x36;
    pub const WHO_AM_I_REG: u8 = 0x21;
    pub const WHO_AM_I_VAL: u16 = 0x4031;

    // uint16_t DesignCap = 0x1450;
    // uint16_t IchgTerm = 0x333;
    // uint16_t VEmpty = 0xa561;

    // Conversions
    pub const SEC_PER_LSB: f32 = 5.625;
    pub const PER_PER_LSB: f32 = 1.0 / 256.0;
    pub const MAH_PER_LSB: f32 = 0.5;
    pub const UA_PER_LSB: f32 = 156.25;
    pub const V_PER_LSB: f32 =  78.125 * 1e-6;
    pub const V_EMPTY_PER_LSB: f32 = 10.0 * 1e-3;           // V / LSB
    pub const V_RECOVERY_PER_LSB: f32 = 40.0 * 1e-3;        // V / LSB
    pub const MA_TERM_PER_LSB: f32 = 1.0 / 6.4;             // mA / LSB
    pub const DEGC_PER_LSB: f32 = 1.0 / 256.0;
}

impl <COMM> MAX17260<COMM> 
where
    COMM: CommProvider + Addressable,
{
    // Constructor for when a Chip is not given
    pub fn new_i2c<T: Into<Option<u8>>>(i2c: COMM, i2c_addr: T) -> Result<Self, MAX17260Error> {

        // Default i2c address
        let i2c_addr = i2c_addr.into();
        let i2c_addr = i2c_addr.unwrap_or(Self::DEFAULT_I2C_ADDRESS);

        let chip = MAX17260Chip::new_i2c(i2c, i2c_addr);

        d_info!("Constructing new MAX17260 sensor");
        let this = Self {
                            chip,
                            interrupt_pin: Some(0),
                            };
        Ok(this)
    }
}

impl <COMM> MAX17260<COMM> 
where
    COMM: CommProvider,
{
    // Read battery voltage
    pub async fn read_level_voltage(&mut self) -> Result<f32, MAX17260Error> {
        
        DLogger::hold();
        let vcell_lsb = self.chip.read_field_str16("VCell").await? as f32;
        DLogger::release();
        
        let vcell_v = vcell_lsb * Self::V_PER_LSB;
        d_info!("Battery Voltage: {}V", vcell_v, 2);
        Ok(vcell_v) 
    }

    // Read battery level percentage
    pub async fn read_level_percent(&mut self) -> Result<f32, MAX17260Error> {
        
        DLogger::hold();
        let batt_lsb = self.chip.read_field_str16("RepSOC").await? as f32;
        DLogger::release();
        
        let batt_per = batt_lsb * Self::PER_PER_LSB;
        d_info!("Battery Percent: {}%", batt_per, 2);
        Ok(batt_per) 
    }

    // Read battery level in mAh
    pub async fn read_level_mahrs(&mut self) -> Result<f32, MAX17260Error> {
        
        DLogger::hold();
        let batt_lsb = self.chip.read_field_str16("RepCap").await? as f32;
        DLogger::release();
        
        let batt_mah = batt_lsb * Self::MAH_PER_LSB;
        d_info!("Battery Charge: {}mAh", batt_mah, 1);
        Ok(batt_mah) 
    }

    // Read time to empty in seconds
    pub async fn read_tte(&mut self) -> Result<f32, MAX17260Error> {
        
        DLogger::hold();
        let tte_lsb = self.chip.read_field_str16("TTE").await? as f32;
        DLogger::release();
        
        let tte_s = tte_lsb * Self::SEC_PER_LSB;
        Ok(tte_s) 
    }

    // Read time to full in seconds
    pub async fn read_ttf(&mut self) -> Result<f32, MAX17260Error> {
        
        DLogger::hold();
        let ttf_lsb = self.chip.read_field_str16("TTF").await? as f32;
        DLogger::release();
        
        let ttf_s = ttf_lsb * Self::SEC_PER_LSB;
        Ok(ttf_s) 
    }

    // Read current in uA
    pub async fn read_current(&mut self, avg: bool) -> Result<f32, MAX17260Error> {
        
        let field = if avg { "AvgCurrent" } else { "Current" };
        
        DLogger::hold();
        let current_lsb = self.chip.read_field_str16(field).await? as f32;
        DLogger::release();
        
        let current_ua = current_lsb * Self::UA_PER_LSB;
        Ok(current_ua)
    }

    // Read temperature in degC
    pub async fn read_temperature(&mut self, avg: bool) -> Result<f32, MAX17260Error> {
        let field = if avg { "AvgDieTemp" } else { "DieTemp" };
        
        DLogger::hold();
        let temp_lsb = self.chip.read_field_str16(field).await? as f32;
        DLogger::release();
        
        let temp_degc = temp_lsb * Self::DEGC_PER_LSB;
        Ok(temp_degc)
    }
}

pub struct MAX17260FieldMap;

impl MAX17260FieldMap {}

impl FieldMapProvider for MAX17260FieldMap {

    fn get_read_field(name: &str) -> Option<Field> {
        Some(*FIELD_MAP.get(name)?)
    }
    
    fn get_write_field(name: &str) -> Option<Field> {
        MAX17260FieldMap::get_read_field(name)
    }
}

static FIELD_MAP: Map<&'static str, Field> = phf_map! {
    // Status Register
    "Status" => Field { reg: 0x00, offset: 0, bits: 16, writable: true, signed: false, },
    "Br" => Field { reg: 0x00, offset: 15, bits: 1, writable: true, signed: false, },
    "Smx" => Field { reg: 0x00, offset: 14, bits: 1, writable: true, signed: false, },
    "Tmx" => Field { reg: 0x00, offset: 13, bits: 1, writable: true, signed: false, },
    "Vmx" => Field { reg: 0x00, offset: 12, bits: 1, writable: true, signed: false, },
    "Bi" => Field { reg: 0x00, offset: 11, bits: 1, writable: true, signed: false, },
    "Smn" => Field { reg: 0x00, offset: 10, bits: 1, writable: true, signed: false, },
    "Tmn" => Field { reg: 0x00, offset: 9, bits: 1, writable: true, signed: false, },
    "Vmn" => Field { reg: 0x00, offset: 8, bits: 1, writable: true, signed: false, },
    "dSOCi" => Field { reg: 0x00, offset: 7, bits: 1, writable: true, signed: false, },
    "Imx" => Field { reg: 0x00, offset: 6, bits: 1, writable: true, signed: false, },
    "Bst" => Field { reg: 0x00, offset: 3, bits: 1, writable: true, signed: false, },
    "Imn" => Field { reg: 0x00, offset: 2, bits: 1, writable: true, signed: false, },
    "POR" => Field { reg: 0x00, offset: 1, bits: 1, writable: true, signed: false, },

    // Current & Temperature
    "Current" => Field { reg: 0x0A, offset: 0, bits: 16, writable: true, signed: false, },
    "AvgCurrent" => Field { reg: 0x0B, offset: 0, bits: 16, writable: true, signed: false, },
    "Temperature" => Field { reg: 0x08, offset: 0, bits: 16, writable: true, signed: false, },
    "AvgTemperature" => Field { reg: 0x16, offset: 0, bits: 16, writable: true, signed: false, },

    // Alerts & Design
    "VAlrtTh" => Field { reg: 0x01, offset: 0, bits: 16, writable: true, signed: false, },
    "IChgTerm" => Field { reg: 0x1E, offset: 0, bits: 16, writable: true, signed: false, },
    "DesignCap" => Field { reg: 0x18, offset: 0, bits: 16, writable: true, signed: false, },

    // Voltage & Empty
    "VEmpty" => Field { reg: 0x3A, offset: 0, bits: 16, writable: true, signed: false, },
    "VE" => Field { reg: 0x3A, offset: 7, bits: 9, writable: true, signed: false, },
    "VR" => Field { reg: 0x01, offset: 0, bits: 7, writable: true, signed: false, },

    // Model Config
    "ModelCFG" => Field { reg: 0xDB, offset: 0, bits: 16, writable: true, signed: false, },
    "Refresh" => Field { reg: 0xDB, offset: 15, bits: 1, writable: true, signed: false, },
    "R100" => Field { reg: 0xDB, offset: 13, bits: 1, writable: true, signed: false, },
    "VChg" => Field { reg: 0xDB, offset: 10, bits: 1, writable: true, signed: false, },
    "ModelID" => Field { reg: 0xDB, offset: 4, bits: 4, writable: true, signed: false, },
    "CSEL" => Field { reg: 0xDB, offset: 2, bits: 1, writable: true, signed: false, },

    // Config Register
    "Config" => Field { reg: 0x1D, offset: 0, bits: 16, writable: true, signed: false, },
    "TSel" => Field { reg: 0x1D, offset: 15, bits: 1, writable: true, signed: false, },
    "SS" => Field { reg: 0x1D, offset: 14, bits: 1, writable: true, signed: false, },
    "TS" => Field { reg: 0x1D, offset: 13, bits: 1, writable: true, signed: false, },
    "VS" => Field { reg: 0x1D, offset: 12, bits: 1, writable: true, signed: false, },
    "IS" => Field { reg: 0x1D, offset: 11, bits: 1, writable: true, signed: false, },
    "THSH" => Field { reg: 0x1D, offset: 10, bits: 1, writable: true, signed: false, },
    "Ten" => Field { reg: 0x1D, offset: 9, bits: 1, writable: true, signed: false, },
    "Tex" => Field { reg: 0x1D, offset: 8, bits: 1, writable: true, signed: false, },
    "SHDN" => Field { reg: 0x1D, offset: 7, bits: 1, writable: true, signed: false, },
    "COMMSH" => Field { reg: 0x1D, offset: 6, bits: 1, writable: true, signed: false, },
    "ETHRM" => Field { reg: 0x1D, offset: 4, bits: 1, writable: true, signed: false, },
    "FTHRM" => Field { reg: 0x1D, offset: 3, bits: 1, writable: true, signed: false, },
    "Aen" => Field { reg: 0x1D, offset: 2, bits: 1, writable: true, signed: false, },
    "Bei" => Field { reg: 0x1D, offset: 1, bits: 1, writable: true, signed: false, },
    "Ber" => Field { reg: 0x1D, offset: 0, bits: 1, writable: true, signed: false, },

    // Config2 Register
    "Config2" => Field { reg: 0xBB, offset: 0, bits: 16, writable: true, signed: false, },
    "AltRateEn" => Field { reg: 0xBB, offset: 13, bits: 1, writable: true, signed: false, },
    "DPEn" => Field { reg: 0xBB, offset: 12, bits: 1, writable: true, signed: false, },
    "POWR" => Field { reg: 0xBB, offset: 8, bits: 4, writable: true, signed: false, },
    "dSOCen" => Field { reg: 0xBB, offset: 7, bits: 1, writable: true, signed: false, },
    "TAIrtEn" => Field { reg: 0xBB, offset: 6, bits: 1, writable: true, signed: false, },
    "LDMdl" => Field { reg: 0xBB, offset: 5, bits: 1, writable: true, signed: false, },
    "DRCfg" => Field { reg: 0xBB, offset: 2, bits: 2, writable: true, signed: false, },
    "CPMode" => Field { reg: 0xBB, offset: 1, bits: 1, writable: true, signed: false, },

    // Capacity & Time
    "RepCap" => Field { reg: 0x05, offset: 0, bits: 16, writable: true, signed: false, },
    "RepSOC" => Field { reg: 0x06, offset: 0, bits: 16, writable: true, signed: false, },
    "FullCapRep" => Field { reg: 0x10, offset: 0, bits: 16, writable: true, signed: false, },
    "TTE" => Field { reg: 0x11, offset: 0, bits: 16, writable: true, signed: false, },
    "TTF" => Field { reg: 0x20, offset: 0, bits: 16, writable: true, signed: false, },

    // Cell Voltage
    "VCell" => Field { reg: 0x09, offset: 0, bits: 16, writable: true, signed: false, },
    "AvgVCell" => Field { reg: 0x19, offset: 0, bits: 16, writable: true, signed: false, },
    "MaxMinVolt" => Field { reg: 0xFF, offset: 0, bits: 16, writable: true, signed: false, },
    "MaxVCELL" => Field { reg: 0xFF, offset: 8, bits: 8, writable: true, signed: false, },
    "MinVCELL" => Field { reg: 0xFF, offset: 0, bits: 8, writable: true, signed: false, },

    // Misc
    "FStat" => Field { reg: 0x3D, offset: 0, bits: 16, writable: true, signed: false, },
    "HibCfg" => Field { reg: 0xBA, offset: 0, bits: 16, writable: true, signed: false, },
    "DevName" => Field { reg: 0x21, offset: 0, bits: 16, writable: false, signed: false, },
};
