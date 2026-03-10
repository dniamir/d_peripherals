use heapless::String;
use core::fmt::Write;
use embassy_time::Timer;

use phf::Map;  // Efficient map for register maps
use phf_macros::phf_map;

use crate::d_peripherals::chip::{Chip, CommProvider, ChipError};
use crate::d_peripherals::chip_implementations::Addressable;
use crate::d_peripherals::chip_map::{Field, FieldMapProvider};
use crate::{d_log::dlogger::DLogger, d_info};  // Logging


#[derive(Debug)]
pub enum BME680Error {
    NotFound,
    BusError(ChipError),
}

// Error conversion 
impl From<ChipError> for BME680Error {
    fn from(err: ChipError) -> Self {BME680Error::BusError(err)}
}

type BMEChip<COMM> = Chip<COMM, BME680FieldMap>;

pub struct BME680<COMM> {
    pub chip: BMEChip<COMM>,
    pub cal_codes: CalCodes,
    pub temp_comp: i32,
    pub t_fine: i32,
}

impl <COMM> BME680<COMM> {
    pub const DEFAULT_I2C_ADDRESS: u8 = 0x76;
    pub const WHO_AM_I_REG: u8 = 0xD0;
    pub const WHO_AM_I_VAL: u8 = 0x61;

    // Gas constants
    pub const CONST_ARRAY1_INT: [u32; 16] = [2147483647, 2147483647, 2147483647, 2147483647,
	                2147483647, 2126008810, 2147483647, 2130303777, 2147483647,
	                2147483647, 2143188679, 2136746228, 2147483647, 2126008810,
	                2147483647, 2147483647];
	
    pub const CONST_ARRAY2_INT: [u32; 16] = [4096000000, 2048000000, 1024000000, 512000000,
                      255744255, 127110228, 64000000, 32258064,
                      16016016, 8000000, 4000000, 2000000,
                      1000000, 500000, 250000, 125000];
}

// When Chip is defined using the BME680 FieldMap
impl <COMM> BME680<COMM> 
where
    COMM: CommProvider + Addressable,
{

    // Constructor for when a Chip is not given
    pub async fn new_i2c<T: Into<Option<u8>>>(i2c: COMM, i2c_addr: T) -> Result<Self, BME680Error> {

        // Default i2c address
        let i2c_addr = i2c_addr.into();
        let i2c_addr = i2c_addr.unwrap_or(Self::DEFAULT_I2C_ADDRESS);

        let chip = BMEChip::new_i2c(i2c, i2c_addr);

        let mut this = Self {
            chip,
            cal_codes: CalCodes::default(),
            temp_comp: 0,
            t_fine: 0,
        };

        this.read_cal_codes().await?;

        Ok(this)
    }
}

impl <COMM> BME680<COMM> 
where
    COMM: CommProvider
{
    // Set a wait profile for the gas sensor
    pub async fn set_gas_wait(&mut self, wait_time_ms: u8, profile_num: u8) -> Result<(), BME680Error> {
        let mut buf: String<16> = String::new();
        write!(buf, "gas_wait_{}", profile_num).unwrap();   
        self.chip.write_field_str(&buf, wait_time_ms).await?;
        Ok(())
    }

    // Set a heater profile for the gas sensor
    pub async fn set_heater_temp(&mut self, target_temp: i16, profile_num: u8) -> Result<(), BME680Error> {

        // --- Get calibration values ---
        let par_g1 = self.cal_codes.par_g1;
        let par_g2 = self.cal_codes.par_g2;
        let par_g3 = self.cal_codes.par_g3;

        // --- Ensure temperature compensation is available ---
        if self.temp_comp == 0 {self.read_temperature().await?;}
        let amb_temp = (self.temp_comp / 100) as i32;

        // --- Read intermediates ---
        let res_heat_range = self.chip.read_field_str("res_heat_range").await? as i32;
        let res_heat_val = self.chip.read_field_str("res_heat_val").await? as i32;

        // --- Calculate heater resistance ---
        let var1 = (((amb_temp * par_g3 as i32) / 10) << 8) as i32;
        let var2 = (par_g1 as i32 + 784)* (((((par_g2 as i32 + 154_009) * target_temp as i32 * 5) / 100) + 3_276_800) / 10);
        let var3 = var1 + (var2 >> 1);
        let var4 = var3 / (res_heat_range + 4);
        let var5 = 131 * res_heat_val + 65_536;
        let res_heat_x100 = ((var4 / var5) - 250) * 34;
        let res_heat_x = ((res_heat_x100 + 50) / 100) as u8;

        // Format field name and write
        let mut buf: String<16> = String::new();
        write!(buf, "res_heat_{}", profile_num).unwrap();   
        self.chip.write_field_str(&buf, res_heat_x).await?;

        Ok(())
    }

    // Shortcut function for read field
    async fn rf(&self, name: &str) -> Result<u8, BME680Error> {
        Ok(self.chip.read_field_str(name).await?)
    }

    // Shortcut register for read register
    async fn rr(&self, reg: u8) -> Result<u8, BME680Error> {
        Ok(self.chip.read_reg(reg).await?)
    }

    // Read all the calibration codes from off sensor
    pub async fn read_cal_codes(&mut self) -> Result<(), BME680Error> {

        // Temperature
        self.cal_codes.par_t1 =(self.rf("par_t1").await? as u16) | ((self.rr(0xea).await? as u16) << 8);
        self.cal_codes.par_t2 =(self.rf("par_t2").await? as i16) | ((self.rr(0x8b).await? as i16) << 8);
        self.cal_codes.par_t3 = self.rf("par_t3").await? as i16;

        // Pressure
        self.cal_codes.par_p1 =(self.rf("par_p1").await? as u16) | ((self.rr(0x8f).await? as u16) << 8);
        self.cal_codes.par_p2 =(self.rf("par_p2").await? as i16) | ((self.rr(0x91).await? as i16) << 8);
        self.cal_codes.par_p3 = self.rf("par_p3").await? as i8;
        self.cal_codes.par_p4 =(self.rf("par_p4").await? as i16) | ((self.rr(0x95).await? as i16) << 8);
        self.cal_codes.par_p5 =(self.rf("par_p5").await? as i16) | ((self.rr(0x97).await? as i16) << 8);
        self.cal_codes.par_p6 = self.rf("par_p6").await? as i8;
        self.cal_codes.par_p7 = self.rf("par_p7").await? as i8;
        self.cal_codes.par_p8 =(self.rf("par_p8").await? as i16) | ((self.rr(0x9d).await? as i16) << 8);
        self.cal_codes.par_p9 =(self.rf("par_p9").await? as i16) | ((self.rr(0x9f).await? as i16) << 8);
        self.cal_codes.par_p10 = self.rf("par_p10").await?;

        // Humidity
        self.cal_codes.par_h1 =((self.rf("par_h1").await? & 0x0F) as u16) | ((self.rr(0xe3).await? as u16) << 4);
        self.cal_codes.par_h2 =((self.rf("par_h2").await? as u16) << 4) | ((self.rr(0xe2).await? as u16) >> 4);
        self.cal_codes.par_h3 = self.rf("par_h3").await? as i8;
        self.cal_codes.par_h4 = self.rf("par_h4").await? as i8;
        self.cal_codes.par_h5 = self.rf("par_h5").await? as i8;
        self.cal_codes.par_h6 = self.rf("par_h6").await?;
        self.cal_codes.par_h7 = self.rf("par_h7").await? as i8;

        // Gas
        self.cal_codes.par_g1 = self.rf("par_g1").await? as i8;
        self.cal_codes.par_g2 =(self.rf("par_g2").await? as i16) | ((self.rr(0xec).await? as i16) << 8);
        self.cal_codes.par_g3 = self.rf("par_g3").await? as i8;

        Ok(())
    }

    // Read the sensor temperature output
    pub async fn read_temperature(&mut self) -> Result<i32, BME680Error> {
        DLogger::hold();

        self.chip.write_field_str("mode", 0b01).await?;
        Timer::after_millis(250).await;

        let mut temp_out = [0u8; 3];
        self.chip.read_regs_str("temp_msb", &mut temp_out).await?;

        // 20-bit ADC value
        let temp_adc: u32 =
            ((temp_out[0] as u32) << 12) |
            ((temp_out[1] as u32) << 4)  |
            ((temp_out[2] as u32) >> 4);

        let temp_comp = self.calibrate_temperature(temp_adc);
        DLogger::release();

        // Log statement with decimal points
        let whole = temp_comp / 100;
        let frac  = temp_comp % 100;
        d_info!("Temperature: {}.{:02} °C", whole, frac);

        Ok(temp_comp)
    }

    // Read the sensor pressure output
    pub async fn read_pressure(&mut self) -> Result<u32, BME680Error> {

        // Lock logger while this is being run
        DLogger::hold();
        self.chip.write_field_str("mode", 0b01).await?;
        Timer::after_millis(250).await;

        // Read Pressure
        let mut press_out = [0u8; 3];
        self.chip.read_regs_str("press_msb", &mut press_out).await?;
        DLogger::release();

        // Bit Shift and Calibrate Value
        let press_adc: u32 = 
            ((press_out[0] as u32) << 12) | 
            ((press_out[1] as u32) << 4) | 
            ((press_out[2] as u32) >> 4);
        let press_comp = self.calibrate_pressure(press_adc);

        // Log pressure
        d_info!("Pressure: {} Pa", press_comp);
        
        Ok(press_comp)
    }

    // Read the sensor humidity output
    pub async fn read_humidity(&mut self) -> Result<i32, BME680Error> {

        // Lock logger while this is being run
        DLogger::hold();
        self.chip.write_field_str("mode", 0b01).await?;
        Timer::after_millis(250).await;

        // Read Pressure
        let mut humid_out = [0u8; 2];
        self.chip.read_regs_str("hum_msb", &mut humid_out).await?;
        DLogger::release();

        // Bit Shift and Calibrate Value
        let humid_adc: u16 = 
            ((humid_out[0] as u16) << 8) | 
            ((humid_out[1] as u16));
        let humid_comp = self.calibrate_humidity(humid_adc);

        // Log pressure
        d_info!("Humidity: {}%", humid_comp / 1000);
        
        Ok(humid_comp)
    }

    pub async fn read_low_gas(&mut self) -> Result<u32, BME680Error>  {

        // Force Measurement
        DLogger::hold();
        self.chip.write_field_str("mode", 0x01).await?;
        Timer::after_millis(250).await;

        // Read gas output
        let mut gas_out = [0u8; 2];
        self.chip.read_regs_str("gas_r_msb", &mut gas_out).await?;
        let gas_res_adc = ((gas_out[0] as u16 )<< 2) | ((gas_out[1] as u16) >> 6);

        // Intermediates
        let range_switching_error = self.chip.read_field_str("range_switching_error").await? as i8;
        let gas_range = self.chip.read_field_str("gas_range_r").await?;
        let range = gas_range.min(15) as usize;  // Ensure within lookup table bounds
        DLogger::release();

        // Calculations
        let var1 = ((1340 + (5 * range_switching_error as i64)) * (Self::CONST_ARRAY1_INT[range] as i64)) >> 16;
        let var2 = (((gas_res_adc as i64) << 15) - 16777216) + var1;
        let var3 = ((Self::CONST_ARRAY2_INT[range] as i64) * var1) >> 9;

        let gas_low =((var3 + (var2 >> 1)) / var2) as u32;

        // Log gas
        d_info!("Gas Low: {}", gas_low);

        Ok(gas_low)
    }

    pub async fn read_high_gas(&mut self) -> Result<u32, BME680Error> {

        // Force Measurement
        DLogger::hold();
        self.chip.write_field_str("mode", 0x01).await?;
        Timer::after_millis(250).await;

        // Read gas output
        let mut gas_out = [0u8; 2];
        self.chip.read_regs_str("gas_r_msb", &mut gas_out).await?;
        let gas_res_adc = ((gas_out[0] as u16 )<< 2) | ((gas_out[1] as u16) >> 6);

        // Intermediates
        let gas_range = self.chip.read_field_str("gas_range_r").await?;
        DLogger::release();

        // Gas range is used as a bit-shift factor [cite: 633, 638]
        let var1: u32 = 262144 >> gas_range;
        let mut var2: i32 = (gas_res_adc as i32) - 512;

        var2 *= 3;
        var2 = 4096 + var2;

        // Use the 10000 * 100 scaling strategy to prevent 32-bit overflow [cite: 641, 645]
        let gass_high = (10000 * var1) / (var2 as u32) * 100;

        // Log gas
        d_info!("Gas High: {}", gass_high);
        
        Ok(gass_high)
    }

    // Calibrate the raw temperature output
    pub fn calibrate_temperature(&mut self, temp_adc: u32) -> i32 {

        // Calibration constants
        let par_t1 = self.cal_codes.par_t1; // i16
        let par_t2 = self.cal_codes.par_t2; // i16
        let par_t3 = self.cal_codes.par_t3; // u16

        // Promote to i64 for intermediate math
        let var1 = ((temp_adc as i32 >> 3) - ((par_t1 as i32) << 1)) as i64;
        let var2 = ((var1 * par_t2 as i64) >> 11) as i64;
        let var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((par_t3 as i64) << 4)) >> 14;

        let t_fine = (var2 + var3) as i32;
        let temp_comp = ((t_fine * 5 + 128) >> 8) as i32;

        // Save intermediate values
        self.t_fine = t_fine;
        self.temp_comp = temp_comp;

        temp_comp
    }

    // Calibrate the raw pressure output
    pub fn calibrate_pressure(&mut self, press_adc: u32) -> u32 {
        let par_p1 = self.cal_codes.par_p1;
        let par_p2 = self.cal_codes.par_p2;
        let par_p3 = self.cal_codes.par_p3;
        let par_p4 = self.cal_codes.par_p4;
        let par_p5 = self.cal_codes.par_p5;
        let par_p6 = self.cal_codes.par_p6;
        let par_p7 = self.cal_codes.par_p7;
        let par_p8 = self.cal_codes.par_p8;
        let par_p9 = self.cal_codes.par_p9;
        let par_p10 = self.cal_codes.par_p10;

        let mut var1: i32;
        let mut var2: i32;
        let var3: i32;
        let mut press_comp: i32;

        let t_fine = self.t_fine;

        var1 = (t_fine >> 1) - 64000;

        var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (par_p6 as i32)) >> 2;
        var2 += (var1 * (par_p5 as i32)) << 1;
        var2 = (var2 >> 2) + ((par_p4 as i32) << 16);

        var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
            ((par_p3 as i32) << 5)) >> 3)
            + (((par_p2 as i32) * var1) >> 1);

        var1 >>= 18;
        var1 = ((32768 + var1) * (par_p1 as i32)) >> 15;

        press_comp = 1048576 - press_adc as i32;
        press_comp = ((press_comp - (var2 >> 12)) * 3125) as i32;

        if press_comp >= (1 << 30) {
            press_comp = (press_comp / var1) << 1;
        } else {
            press_comp = (press_comp << 1) / var1;
        }

        var1 = ((par_p9 as i32)
            * (((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;

        var2 = ((press_comp >> 2) * (par_p8 as i32)) >> 13;

        var3 = ((press_comp >> 8)
            * (press_comp >> 8)
            * (press_comp >> 8)
            * (par_p10 as i32)) >> 17;

        press_comp = press_comp
            + ((var1 + var2 + var3 + ((par_p7 as i32) << 7)) >> 4);

        press_comp as u32
    }

    // Calibrate the raw humidity output
    pub fn calibrate_humidity(&mut self, humid_adc: u16) -> i32 {
        let par_h1 = self.cal_codes.par_h1 as i32;
        let par_h2 = self.cal_codes.par_h2 as i32;
        let par_h3 = self.cal_codes.par_h3 as i32;
        let par_h4 = self.cal_codes.par_h4 as i32;
        let par_h5 = self.cal_codes.par_h5 as i32;
        let par_h6 = self.cal_codes.par_h6 as i32;
        let par_h7 = self.cal_codes.par_h7 as i32;

        let temp_scaled = self.temp_comp;

        let var1 = humid_adc as i32 - (par_h1 << 4)
            - ((temp_scaled * par_h3 / 100) >> 1);
        let var2 = par_h2 * ((temp_scaled * par_h4 / 100)
            + (((temp_scaled * (temp_scaled * par_h5 / 100)) >> 6) / 100)
            + (1 << 14)) >> 10;
        let var3 = var1 * var2;
        let var4 = ((par_h6 << 7) + (temp_scaled * par_h7 / 100)) >> 4;
        let var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
        let var6 = (var4 * var5) >> 1;

        (((var3 + var6) >> 10) * 1000) >> 12
    }

}

#[derive(Copy, Clone)]
pub struct BME680FieldMap;

impl FieldMapProvider for BME680FieldMap {
    fn get_read_field(name: &str) -> Option<Field> {
        Some(*FIELD_MAP.get(name)?)
    }
    fn get_write_field(name: &str) -> Option<Field> {
        BME680FieldMap::get_read_field(name)
    }
}

pub static FIELD_MAP: Map<&'static str, Field> = phf_map! {
    "status" => Field { reg: 0x73, offset: 0, bits: 8, writable: true, signed: false, },
    "reset" => Field { reg: 0xe0, offset: 0, bits: 8, writable: true, signed: false, },
    "Id" => Field { reg: 0xd0, offset: 0, bits: 8, writable: true, signed: false, },
    "chip_id" => Field { reg: 0xd0, offset: 0, bits: 8, writable: true, signed: false, },
    "Config" => Field { reg: 0x75, offset: 0, bits: 8, writable: true, signed: false, },
    "filter" => Field { reg: 0x75, offset: 2, bits: 3, writable: true, signed: false, },
    "ctrl_meas" => Field { reg: 0x74, offset: 0, bits: 8, writable: true, signed: false, },
    "osrs_t" => Field { reg: 0x74, offset: 5, bits: 3, writable: true, signed: false, },
    "osrs_p" => Field { reg: 0x74, offset: 2, bits: 3, writable: true, signed: false, },
    "mode" => Field { reg: 0x74, offset: 0, bits: 2, writable: true, signed: false, },

    "Ctrl_hum" => Field { reg: 0x72, offset: 0, bits: 8, writable: true, signed: false, },
    "osrs_h" => Field { reg: 0x72, offset: 0, bits: 3, writable: true, signed: false, },

    "ctrl_gas_1" => Field { reg: 0x71, offset: 0, bits: 8, writable: true, signed: false, },
    "ctrl_gas_0" => Field { reg: 0x70, offset: 4, bits: 2, writable: true, signed: false, },
    "run_gas" => Field { reg: 0x71, offset: 4, bits: 1, writable: true, signed: false, },
    "nb_conv" => Field { reg: 0x71, offset: 0, bits: 4, writable: true, signed: false, },
    "heat_off" => Field { reg: 0x70, offset: 3, bits: 1, writable: true, signed: false, },
    "gas_wait_9" => Field { reg: 0x6d, offset: 0, bits: 8, writable: true, signed: false, },
    "gas_wait_8" => Field { reg: 0x6c, offset: 0, bits: 8, writable: true, signed: false, },
    "gas_wait_7" => Field { reg: 0x6b, offset: 0, bits: 8, writable: true, signed: false, },
    "gas_wait_6" => Field { reg: 0x6a, offset: 0, bits: 8, writable: true, signed: false, },
    "gas_wait_5" => Field { reg: 0x69, offset: 0, bits: 8, writable: true, signed: false, },
    "gas_wait_4" => Field { reg: 0x68, offset: 0, bits: 8, writable: true, signed: false, },
    "gas_wait_3" => Field { reg: 0x67, offset: 0, bits: 8, writable: true, signed: false, },
    "gas_wait_2" => Field { reg: 0x66, offset: 0, bits: 8, writable: true, signed: false, },
    "gas_wait_1" => Field { reg: 0x65, offset: 0, bits: 8, writable: true, signed: false, },
    "gas_wait_0" => Field { reg: 0x64, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_9" => Field { reg: 0x63, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_8" => Field { reg: 0x62, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_7" => Field { reg: 0x61, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_6" => Field { reg: 0x60, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_5" => Field { reg: 0x5f, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_4" => Field { reg: 0x5e, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_3" => Field { reg: 0x5d, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_2" => Field { reg: 0x5c, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_1" => Field { reg: 0x5b, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_0" => Field { reg: 0x5a, offset: 0, bits: 8, writable: true, signed: false, },

    "gas_r_lsb" => Field { reg: 0x2b, offset: 0, bits: 8, writable: true, signed: false, },
    "gas_range_r" => Field { reg: 0x2b, offset: 0, bits: 4, writable: true, signed: false, },
    "heat_stab_r" => Field { reg: 0x2b, offset: 4, bits: 1, writable: true, signed: false, },
    "gas_valid_r" => Field { reg: 0x2b, offset: 5, bits: 1, writable: true, signed: false, },

    "gas_r_msb" => Field { reg: 0x2a, offset: 0, bits: 8, writable: true, signed: false, },
    "hum_lsb" => Field { reg: 0x26, offset: 0, bits: 8, writable: true, signed: false, },
    "hum_msb" => Field { reg: 0x25, offset: 0, bits: 8, writable: true, signed: false, },
    "temp_xlsb" => Field { reg: 0x24, offset: 4, bits: 4, writable: true, signed: false, },
    "temp_lsb" => Field { reg: 0x23, offset: 0, bits: 8, writable: true, signed: false, },
    "temp_msb" => Field { reg: 0x22, offset: 0, bits: 8, writable: true, signed: false, },
    "press_xlsb" => Field { reg: 0x21, offset: 4, bits: 4, writable: true, signed: false, },
    "press_lsb" => Field { reg: 0x20, offset: 0, bits: 8, writable: true, signed: false, },
    "press_msb" => Field { reg: 0x1f, offset: 0, bits: 8, writable: true, signed: false, },

    "par_t1" => Field { reg: 0xe9, offset: 0, bits: 8, writable: true, signed: false, },
    "par_t2" => Field { reg: 0x8a, offset: 0, bits: 8, writable: true, signed: false, },
    "par_t3" => Field { reg: 0x8c, offset: 0, bits: 8, writable: true, signed: false, },
    "par_p1" => Field { reg: 0x8e, offset: 0, bits: 8, writable: true, signed: false, },
    "par_p2" => Field { reg: 0x90, offset: 0, bits: 8, writable: true, signed: false, },
    "par_p3" => Field { reg: 0x92, offset: 0, bits: 8, writable: true, signed: false, },
    "par_p4" => Field { reg: 0x94, offset: 0, bits: 8, writable: true, signed: false, },
    "par_p5" => Field { reg: 0x96, offset: 0, bits: 8, writable: true, signed: false, },
    "par_p6" => Field { reg: 0x99, offset: 0, bits: 8, writable: true, signed: false, },
    "par_p7" => Field { reg: 0x98, offset: 0, bits: 8, writable: true, signed: false, },
    "par_p8" => Field { reg: 0x9c, offset: 0, bits: 8, writable: true, signed: false, },
    "par_p9" => Field { reg: 0x9e, offset: 0, bits: 8, writable: true, signed: false, },
    "par_p10" => Field { reg: 0xa0, offset: 0, bits: 8, writable: true, signed: false, },
    "par_h1" => Field { reg: 0xe2, offset: 0, bits: 8, writable: true, signed: false, },
    "par_h2" => Field { reg: 0xe1, offset: 0, bits: 8, writable: true, signed: false, },
    "par_h3" => Field { reg: 0xe4, offset: 0, bits: 8, writable: true, signed: false, },
    "par_h4" => Field { reg: 0xe5, offset: 0, bits: 8, writable: true, signed: false, },
    "par_h5" => Field { reg: 0xe6, offset: 0, bits: 8, writable: true, signed: false, },
    "par_h6" => Field { reg: 0xe7, offset: 0, bits: 8, writable: true, signed: false, },
    "par_h7" => Field { reg: 0xe8, offset: 0, bits: 8, writable: true, signed: false, },
    "par_g1" => Field { reg: 0xed, offset: 0, bits: 8, writable: true, signed: false, },
    "par_g2" => Field { reg: 0xeb, offset: 0, bits: 8, writable: true, signed: false, },
    "par_g3" => Field { reg: 0xee, offset: 0, bits: 8, writable: true, signed: false, },
    "res_heat_range" => Field { reg: 0x02, offset: 4, bits: 2, writable: true, signed: false, },
    "res_heat_val" => Field { reg: 0x00, offset: 0, bits: 8, writable: true, signed: false, },
    "range_switching_error" => Field { reg: 0x04, offset: 0, bits: 8, writable: true, signed: false, },

};

#[derive(Default)]  // Gives all numeric fields default values of 0
pub struct CalCodes {
    // Pressure
    pub par_p10: u8,
    pub par_p9: i16,
    pub par_p8: i16,
    pub par_p7: i8,
    pub par_p6: i8,
    pub par_p5: i16,
    pub par_p4: i16,
    pub par_p3: i8,
    pub par_p2: i16,
    pub par_p1: u16,

    // Temperature
    pub par_t3: i16,
    pub par_t2: i16,
    pub par_t1: u16,

    // Humidity
    pub par_h7: i8,
    pub par_h6: u8,
    pub par_h5: i8,
    pub par_h4: i8,
    pub par_h3: i8,
    pub par_h2: u16,
    pub par_h1: u16,

    // Gas
    pub par_g3: i8,
    pub par_g2: i16,
    pub par_g1: i8,

    // Misc
    pub res_heat_range: i8,
    pub res_heat_val: i8,
    pub gas_adc: i16,
    pub gas_range: i8,
    pub range_switching_error: i8,
}