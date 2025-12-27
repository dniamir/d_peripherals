use heapless::String;
use core::marker::PhantomData;
use core::fmt::Write;

use phf::Map;  // Efficient map for register maps
use phf_macros::phf_map;

use crate::d_peripherals::chip::{Chip, I2CProvider, I2CError};
use crate::d_peripherals::chip_map::{Field, FieldMapProvider};
use crate::{d_log::dlogger::DLogger, d_info};  // Logging


#[derive(Debug)]
pub enum BME680Error {
    NotFound,
    BusError(I2CError),
}

// Error conversion 
impl From<I2CError> for BME680Error {
    fn from(err: I2CError) -> Self {BME680Error::BusError(err)}
}

pub struct BME680<I2C, ChipGeneric=Chip<I2C, BME680FieldMap>> {
    // pub chip: Chip<I2C, Bme680FieldMap>,
    pub chip: ChipGeneric,
    pub cal_codes: CalCodes,
    pub temp_comp: i32,
    pub t_fine: i32,
    pub _i2c: PhantomData<I2C>,
}

// When Chip is defined using the BME680 FieldMap
impl <I2C> BME680<I2C, Chip<I2C, BME680FieldMap>> 
where
    I2C: I2CProvider,
{

    // Constructor for when a Chip is not given
    pub async fn new(i2c: I2C, i2c_addr: u8) -> Result<Self, BME680Error> {

        let chip: Chip<I2C, BME680FieldMap> = Chip{i2c, i2c_addr, _map: PhantomData};

        let mut this = Self {
            chip,
            cal_codes: CalCodes::default(),
            temp_comp: 0,
            t_fine: 0,
            _i2c: PhantomData
        };

        this.read_cal_codes().await?;

        Ok(this)
    }

    pub async fn config(&mut self, profile_num: u8) -> Result<(), BME680Error> {

        // Other Sensor Settings
        self.chip.write_field("osrs_h", 0b101).await?;  // 16x oversampling
        self.chip.write_field("osrs_t", 0b101).await?;  // 16x oversampling
        self.chip.write_field("osrs_p", 0b101).await?;  // 16x oversampling
        self.chip.write_field("filter", 0b010).await?;  // Filter coefficient of 3 - form of averaging filter

        // Gas Sensor Settings
        self.chip.write_field("gas_range_r", 4).await?; // Set Gas Range
        self.chip.write_field("run_gas", 0b1).await?; // Turn on Gas Sensor
        self.chip.write_field("nb_conv", profile_num).await?; // Set Heater profile to profile 0

        // Set time between beginning of the heat phase and start of resistance conversion
        self.set_gas_wait(0b00011110, profile_num).await?; // This should be 30ms

        // Set heater temperature
        self.set_heater_temp(300, profile_num).await?;  // Set heater profile 0 to 300C

        Ok(())
    }

    pub async fn set_gas_wait(&mut self, wait_time_ms: u8, profile_num: u8) -> Result<(), BME680Error> {
        let mut buf: String<16> = String::new();
        write!(buf, "gas_wait_{}", profile_num).unwrap();   
        self.chip.write_field(&buf, wait_time_ms).await?;
        Ok(())
    }

    pub async fn set_heater_temp(&mut self, target_temp: i16, profile_num: u8) -> Result<(), BME680Error> {

        // --- Get calibration values ---
        let par_g1 = self.cal_codes.par_g1;
        let par_g2 = self.cal_codes.par_g2;
        let par_g3 = self.cal_codes.par_g3;

        // --- Ensure temperature compensation is available ---
        if self.temp_comp == 0 {self.read_temperature().await?;}
        let amb_temp = (self.temp_comp / 100) as i32;

        // --- Read intermediates ---
        let res_heat_range = self.chip.read_field("res_heat_range").await? as i32;
        let res_heat_val = self.chip.read_field("res_heat_val").await? as i32;

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
        self.chip.write_field(&buf, res_heat_x).await?;

        Ok(())
    }

    // async fn self.rf(this: &mut Self, name: &str) -> Result<u8, BME680Error> {
    //     Ok(this.chip.read_field(name).await?)
    // }

    // async fn self.rr(this: &mut Self, reg: u8) -> Result<u8, BME680Error> {
    //     Ok(this.chip.read_reg(reg).await?)
    // }

    async fn rf(&self, name: &str) -> Result<u8, BME680Error> {
        Ok(self.chip.read_field(name).await?)
    }

    async fn rr(&self, reg: u8) -> Result<u8, BME680Error> {
        Ok(self.chip.read_reg(reg).await?)
    }

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

    pub async fn read_temperature(&mut self) -> Result<i32, BME680Error> {
        DLogger::hold();

        self.chip.write_field("mode", 0b01).await?;

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

    pub async fn read_pressure(&mut self) -> Result<u32, BME680Error> {

        // Lock logger while this is being run
        DLogger::hold();
        self.chip.write_field("mode", 0b01).await?;

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
}

#[derive(Copy, Clone)]
pub struct BME680FieldMap;

impl FieldMapProvider for BME680FieldMap {
    fn get_field(name: &str) -> Option<&'static Field> {
        FIELD_MAP.get(name)
    }
}

pub static FIELD_MAP: Map<&'static str, Field> = phf_map! {
    "status" => Field { reg: 0x73, offset: 0, bits: 8, writable: true },
    "reset" => Field { reg: 0xe0, offset: 0, bits: 8, writable: true },
    "Id" => Field { reg: 0xd0, offset: 0, bits: 8, writable: false },
    "chip_id" => Field { reg: 0xd0, offset: 0, bits: 8, writable: false },
    "Config" => Field { reg: 0x75, offset: 0, bits: 8, writable: true },
    "filter" => Field { reg: 0x75, offset: 2, bits: 3, writable: true },
    "ctrl_meas" => Field { reg: 0x74, offset: 0, bits: 8, writable: true },
    "osrs_t" => Field { reg: 0x74, offset: 5, bits: 3, writable: true },
    "osrs_p" => Field { reg: 0x74, offset: 2, bits: 3, writable: true },
    "mode" => Field { reg: 0x74, offset: 0, bits: 2, writable: true },

    "Ctrl_hum" => Field { reg: 0x72, offset: 0, bits: 8, writable: true },
    "osrs_h" => Field { reg: 0x72, offset: 0, bits: 3, writable: true },

    "ctrl_gas_1" => Field { reg: 0x71, offset: 0, bits: 8, writable: true },
    "ctrl_gas_0" => Field { reg: 0x70, offset: 4, bits: 2, writable: true },
    "run_gas" => Field { reg: 0x71, offset: 4, bits: 1, writable: true },
    "nb_conv" => Field { reg: 0x71, offset: 0, bits: 4, writable: true },
    "heat_off" => Field { reg: 0x70, offset: 3, bits: 1, writable: true },
    "gas_wait_9" => Field { reg: 0x6d, offset: 0, bits: 8, writable: true },
    "gas_wait_8" => Field { reg: 0x6c, offset: 0, bits: 8, writable: true },
    "gas_wait_7" => Field { reg: 0x6b, offset: 0, bits: 8, writable: true },
    "gas_wait_6" => Field { reg: 0x6a, offset: 0, bits: 8, writable: true },
    "gas_wait_5" => Field { reg: 0x69, offset: 0, bits: 8, writable: true },
    "gas_wait_4" => Field { reg: 0x68, offset: 0, bits: 8, writable: true },
    "gas_wait_3" => Field { reg: 0x67, offset: 0, bits: 8, writable: true },
    "gas_wait_2" => Field { reg: 0x66, offset: 0, bits: 8, writable: true },
    "gas_wait_1" => Field { reg: 0x65, offset: 0, bits: 8, writable: true },
    "gas_wait_0" => Field { reg: 0x64, offset: 0, bits: 8, writable: true },
    "res_heat_9" => Field { reg: 0x63, offset: 0, bits: 8, writable: true },
    "res_heat_8" => Field { reg: 0x62, offset: 0, bits: 8, writable: true },
    "res_heat_7" => Field { reg: 0x61, offset: 0, bits: 8, writable: true },
    "res_heat_6" => Field { reg: 0x60, offset: 0, bits: 8, writable: true },
    "res_heat_5" => Field { reg: 0x5f, offset: 0, bits: 8, writable: true },
    "res_heat_4" => Field { reg: 0x5e, offset: 0, bits: 8, writable: true },
    "res_heat_3" => Field { reg: 0x5d, offset: 0, bits: 8, writable: true },
    "res_heat_2" => Field { reg: 0x5c, offset: 0, bits: 8, writable: true },
    "res_heat_1" => Field { reg: 0x5b, offset: 0, bits: 8, writable: true },
    "res_heat_0" => Field { reg: 0x5a, offset: 0, bits: 8, writable: true },

    "gas_r_lsb" => Field { reg: 0x2b, offset: 0, bits: 8, writable: false },
    "gas_range_r" => Field { reg: 0x2b, offset: 0, bits: 4, writable: false },
    "heat_stab_r" => Field { reg: 0x2b, offset: 4, bits: 1, writable: false },
    "gas_valid_r" => Field { reg: 0x2b, offset: 5, bits: 1, writable: false },

    "gas_r_msb" => Field { reg: 0x2a, offset: 0, bits: 8, writable: false },
    "hum_lsb" => Field { reg: 0x26, offset: 0, bits: 8, writable: false },
    "hum_msb" => Field { reg: 0x25, offset: 0, bits: 8, writable: false },
    "temp_xlsb" => Field { reg: 0x24, offset: 4, bits: 4, writable: false },
    "temp_lsb" => Field { reg: 0x23, offset: 0, bits: 8, writable: false },
    "temp_msb" => Field { reg: 0x22, offset: 0, bits: 8, writable: false },
    "press_xlsb" => Field { reg: 0x21, offset: 4, bits: 4, writable: false },
    "press_lsb" => Field { reg: 0x20, offset: 0, bits: 8, writable: false },
    "press_msb" => Field { reg: 0x1f, offset: 0, bits: 8, writable: false },

    "par_t1" => Field { reg: 0xe9, offset: 0, bits: 8, writable: false },
    "par_t2" => Field { reg: 0x8a, offset: 0, bits: 8, writable: false },
    "par_t3" => Field { reg: 0x8c, offset: 0, bits: 8, writable: false },
    "par_p1" => Field { reg: 0x8e, offset: 0, bits: 8, writable: false },
    "par_p2" => Field { reg: 0x90, offset: 0, bits: 8, writable: false },
    "par_p3" => Field { reg: 0x92, offset: 0, bits: 8, writable: false },
    "par_p4" => Field { reg: 0x94, offset: 0, bits: 8, writable: false },
    "par_p5" => Field { reg: 0x96, offset: 0, bits: 8, writable: false },
    "par_p6" => Field { reg: 0x99, offset: 0, bits: 8, writable: false },
    "par_p7" => Field { reg: 0x98, offset: 0, bits: 8, writable: false },
    "par_p8" => Field { reg: 0x9c, offset: 0, bits: 8, writable: false },
    "par_p9" => Field { reg: 0x9e, offset: 0, bits: 8, writable: false },
    "par_p10" => Field { reg: 0xa0, offset: 0, bits: 8, writable: false },
    "par_h1" => Field { reg: 0xe2, offset: 0, bits: 8, writable: false },
    "par_h2" => Field { reg: 0xe1, offset: 0, bits: 8, writable: false },
    "par_h3" => Field { reg: 0xe4, offset: 0, bits: 8, writable: false },
    "par_h4" => Field { reg: 0xe5, offset: 0, bits: 8, writable: false },
    "par_h5" => Field { reg: 0xe6, offset: 0, bits: 8, writable: false },
    "par_h6" => Field { reg: 0xe7, offset: 0, bits: 8, writable: false },
    "par_h7" => Field { reg: 0xe8, offset: 0, bits: 8, writable: false },
    "par_g1" => Field { reg: 0xed, offset: 0, bits: 8, writable: false },
    "par_g2" => Field { reg: 0xeb, offset: 0, bits: 8, writable: false },
    "par_g3" => Field { reg: 0xee, offset: 0, bits: 8, writable: false },
    "res_heat_range" => Field { reg: 0x02, offset: 4, bits: 2, writable: false },
    "res_heat_val" => Field { reg: 0x00, offset: 0, bits: 8, writable: false },
    "range_switching_error" => Field { reg: 0x04, offset: 0, bits: 8, writable: false },

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