use embassy_time::Timer;

use phf::Map;  // Efficient map for register maps
use phf_macros::phf_map;

use crate::d_peripherals::chip::{Chip, CommProvider, ChipError};
use crate::d_peripherals::chip_implementations::Addressable;
use crate::d_peripherals::chip_map::{Field, FieldMapProvider};
use crate::{d_log::dlogger_common::DLogger, d_info};  // Logging


#[derive(Debug)]
pub enum ICM42670Error {
    NotFound,
    InvalidInput,
    BusError(ChipError),
}

// Error conversion
impl From<ChipError> for ICM42670Error {
    fn from(err: ChipError) -> Self {ICM42670Error::BusError(err)}
}

#[repr(u8)]
pub enum GyroFs {
    Dps2000 = 0b00,
    Dps1000 = 0b01,
    Dps500  = 0b10,
    Dps250  = 0b11,
}

#[repr(u8)]
pub enum AccelFs {
    G16 = 0b00,
    G8  = 0b01,
    G4  = 0b10,
    G2  = 0b11,
}

#[repr(u8)]
pub enum GyroOdr {
    Hz1600 = 0b0101,
    Hz800  = 0b0110,
    Hz400  = 0b0111,
    Hz200  = 0b1000,
    Hz100  = 0b1001,
    Hz50   = 0b1010,
    Hz25   = 0b1011,
    Hz12_5 = 0b1100,
}

#[repr(u8)]
pub enum AccelOdr {
    Hz1600   = 0b0101,
    Hz800    = 0b0110,
    Hz400    = 0b0111,
    Hz200    = 0b1000,
    Hz100    = 0b1001,
    Hz50     = 0b1010,
    Hz25     = 0b1011,
    Hz12_5   = 0b1100,
    Hz6_25   = 0b1101,
    Hz3_125  = 0b1110,
    Hz1_5625 = 0b1111,
}

#[repr(u8)]
pub enum AccelAvg {
    Avg2x  = 0b000,
    Avg4x  = 0b001,
    Avg8x  = 0b010,
    Avg16x = 0b011,
    Avg32x = 0b100,
    Avg64x = 0b101,
}

#[repr(u8)]
pub enum AccelFilt {
    Bypass = 0b000,
    Hz180  = 0b001,
    Hz121  = 0b010,
    Hz73   = 0b011,
    Hz53   = 0b100,
    Hz34   = 0b101,
    Hz25   = 0b110,
    Hz16   = 0b111,
}

#[repr(u8)]
pub enum GyroFilt {
    Bypass = 0b000,
    Hz180  = 0b001,
    Hz121  = 0b010,
    Hz73   = 0b011,
    Hz53   = 0b100,
    Hz34   = 0b101,
    Hz25   = 0b110,
    Hz16   = 0b111,
}

pub struct ImuReading {
    pub temp_degc: f32,
    pub accel: (f32, f32, f32),
    pub gyro: (f32, f32, f32),
}

type ICM42670Chip<COMM> = Chip<COMM, ICM42670FieldMap>;

pub struct ICM42670<COMM> {
    pub chip: ICM42670Chip<COMM>,
}

impl <COMM> ICM42670<COMM> {
    pub const DEFAULT_I2C_ADDRESS: u8 = 0x69;
    pub const WHO_AM_I_REG: u8 = 0x75;
    pub const WHO_AM_I_VAL: u8 = 0x67;
}

// When Chip is defined using the BME680 FieldMap
impl <COMM> ICM42670<COMM> 
where
    COMM: CommProvider + Addressable,
{

    // Constructor for when a Chip is not given
    pub async fn new_i2c<T: Into<Option<u8>>>(i2c: COMM, i2c_addr: T) -> Result<Self, ICM42670Error> {

        d_info!("Initializing ICM42670");

        // Default i2c address
        DLogger::hold();
        let i2c_addr = i2c_addr.into();
        let i2c_addr = i2c_addr.unwrap_or(Self::DEFAULT_I2C_ADDRESS);

        let chip = ICM42670Chip::new_i2c(i2c, i2c_addr);
        let this = Self {chip};
        DLogger::release();

        Ok(this)
    }
}

impl<COMM> ICM42670<COMM>
where
    COMM: CommProvider,
{
    pub async fn who_am_i(&mut self) -> Result<u8, ICM42670Error> {
        Ok(self.chip.read_field_str("whoami").await?)
    }

    pub async fn reset(&self) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("soft_reset_device_config", 1).await?;
        Timer::after_millis(2).await;
        Ok(())
    }

    // ---- Power control ----
    pub async fn enable_accel_ln(&self) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("accel_mode", 0b11 as u8).await?;
        Timer::after_millis(1).await;
        Ok(())
    }

    pub async fn enable_accel_lp(&self) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("accel_mode", 0b10 as u8).await?;
        Timer::after_millis(1).await;
        Ok(())
    }

    pub async fn disable_accel(&self) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("accel_mode", 0 as u8).await?;
        Timer::after_millis(1).await;
        Ok(())
    }

    pub async fn enable_gyro_ln(&self) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("gyro_mode", 0b11 as u8).await?;
        Timer::after_millis(1).await;
        Ok(())
    }

    pub async fn disable_gyro(&self) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("gyro_mode", 0 as u8).await?;
        Timer::after_millis(1).await;
        Ok(())
    }

    pub async fn power_down(&self) -> Result<(), ICM42670Error> {
        d_info!("Powering down gyro and accel");
        DLogger::hold();
        self.disable_accel().await?;
        self.disable_gyro().await?;
        DLogger::release();
        Ok(())
    }

    // ---- Full-scale range ----
    pub async fn set_accel_fs(&self, fs: AccelFs) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("accel_ui_fs_sel", fs as u8).await?;
        Ok(())
    }

    pub async fn set_gyro_fs(&self, fs: GyroFs) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("gyro_ui_fs_sel", fs as u8).await?;
        Ok(())
    }

    pub async fn get_accel_fs(&self) -> Result<u16, ICM42670Error> {
        let bits = self.chip.read_field_str("accel_ui_fs_sel").await?;
        match bits {
            0b00 => Ok(16),
            0b01 => Ok(8),
            0b10 => Ok(4),
            0b11 => Ok(2),
            _    => Err(ICM42670Error::InvalidInput),
        }
    }

    pub async fn get_gyro_fs(&self) -> Result<u16, ICM42670Error> {
        let bits = self.chip.read_field_str("gyro_ui_fs_sel").await?;
        match bits {
            0b00 => Ok(2000),
            0b01 => Ok(1000),
            0b10 => Ok(500),
            0b11 => Ok(250),
            _    => Err(ICM42670Error::InvalidInput),
        }
    }

    // ---- Output data rate ----
    pub async fn set_accel_odr(&self, odr: AccelOdr) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("accel_odr", odr as u8).await?;
        Ok(())
    }

    pub async fn set_gyro_odr(&self, odr: GyroOdr) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("gyro_odr", odr as u8).await?;
        Ok(())
    }

    // ---- Averaging & filtering ----
    pub async fn set_accel_avg(&self, avg: AccelAvg) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("accel_ui_avg", avg as u8).await?;
        Ok(())
    }

    pub async fn set_accel_filt(&self, bw: AccelFilt) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("accel_ui_filt_bw", bw as u8).await?;
        Ok(())
    }

    pub async fn set_gyro_filt(&self, bw: GyroFilt) -> Result<(), ICM42670Error> {
        self.chip.write_field_str("gyro_ui_filt_bw", bw as u8).await?;
        Ok(())
    }

    // ---- Sensor reads ----
    // Returns raw 16-bit signed temperature. Convert to degrees C: (raw / 128) + 25
    pub async fn read_temperature_raw(&self) -> Result<i16, ICM42670Error> {
        let mut buf = [0u8; 2];
        self.chip.read_regs_str("temp_data1", &mut buf).await?;
        Ok(i16::from_be_bytes(buf))
    }

    // Returns temperature in tenths of a degree Celsius (e.g. 253 = 25.3 C)
    pub async fn read_temperature_cdeg(&self) -> Result<i32, ICM42670Error> {
        DLogger::hold();
        let raw = self.read_temperature_raw().await? as i32;
        DLogger::release();

        let temp_cdeg = (raw * 10 / 128) + 250;
        d_info!("Temp: {} cdegC", temp_cdeg);

        Ok(temp_cdeg)
    }

    // Returns (x, y, z) raw 16-bit signed accelerometer counts
    pub async fn read_accel(&self) -> Result<(i16, i16, i16), ICM42670Error> {
        let mut buf = [0u8; 6];

        DLogger::hold();
        self.chip.read_regs_str("accel_data_x1", &mut buf).await?;
        DLogger::release();

        let accel_x_gee = i16::from_be_bytes([buf[0], buf[1]]);
        let accel_y_gee = i16::from_be_bytes([buf[2], buf[3]]);
        let accel_z_gee = i16::from_be_bytes([buf[4], buf[5]]);

        d_info!("Accel X: {} Gee", accel_x_gee);
        d_info!("Accel Y: {} Gee", accel_y_gee);
        d_info!("Accel Z: {} Gee", accel_z_gee);

        Ok((accel_x_gee, accel_y_gee, accel_z_gee))
    }

    // Returns (x, y, z) raw 16-bit signed gyroscope counts
    pub async fn read_gyro(&self) -> Result<(i16, i16, i16), ICM42670Error> {
        let mut buf = [0u8; 6];

        DLogger::hold();
        self.chip.read_regs_str("gyro_data_x1", &mut buf).await?;
        DLogger::release();

        let gyro_x_dps = i16::from_be_bytes([buf[0], buf[1]]);
        let gyro_y_dps = i16::from_be_bytes([buf[2], buf[3]]);
        let gyro_z_dps = i16::from_be_bytes([buf[4], buf[5]]);

        d_info!("Gyro X: {} dps", gyro_x_dps);
        d_info!("Gyro Y: {} dps", gyro_y_dps);
        d_info!("Gyro Z: {} dps", gyro_z_dps);

        Ok((gyro_x_dps, gyro_y_dps, gyro_z_dps))
    }

    // Burst reads temp + accel + gyro in one transaction (0x09–0x16, 14 bytes)
    pub async fn read_all(&self) -> Result<ImuReading, ICM42670Error> {
        let mut buf = [0u8; 14];

        DLogger::hold();
        self.chip.read_regs_str("temp_data1", &mut buf).await?;

        let temp_raw = i16::from_be_bytes([buf[0], buf[1]]) as i32;
        let temp_degc = (temp_raw as f32 / 128.0) + 25.0;

        // Get LSB readings
        let accel_x_lsb = i16::from_be_bytes([buf[2], buf[3]]);
        let accel_y_lsb = i16::from_be_bytes([buf[4], buf[5]]);
        let accel_z_lsb = i16::from_be_bytes([buf[6], buf[7]]);

        let gyro_x_lsb = i16::from_be_bytes([buf[8],  buf[9]]);
        let gyro_y_lsb = i16::from_be_bytes([buf[10], buf[11]]);
        let gyro_z_lsb = i16::from_be_bytes([buf[12], buf[13]]);

        // Convert to real units
        let accel_fs = self.get_accel_fs().await?;
        let gyro_fs = self.get_gyro_fs().await?;
        DLogger::release();

        let accel_x_gee = accel_x_lsb as f32 * accel_fs as f32 / 32768.0;
        let accel_y_gee = accel_y_lsb as f32 * accel_fs as f32 / 32768.0;
        let accel_z_gee = accel_z_lsb as f32 * accel_fs as f32 / 32768.0;

        let gyro_x_dps = gyro_x_lsb as f32 * gyro_fs as f32 / 32768.0;
        let gyro_y_dps = gyro_y_lsb as f32 * gyro_fs as f32 / 32768.0;
        let gyro_z_dps = gyro_z_lsb as f32 * gyro_fs as f32 / 32768.0;

        d_info!("Temp: {} degC", temp_degc, 1);
        d_info!("Accel X: {} gee", accel_x_gee, 2);
        d_info!("Accel Y: {} gee", accel_y_gee, 2);
        d_info!("Accel Z: {} gee", accel_z_gee, 2);
        d_info!("Gyro X: {} dps", gyro_x_dps, 1);
        d_info!("Gyro Y: {} dps", gyro_y_dps, 1);
        d_info!("Gyro Z: {} dps", gyro_z_dps, 1);

        Ok(ImuReading {
            temp_degc,
            accel: (accel_x_gee, accel_y_gee, accel_z_gee),
            gyro:  (gyro_x_dps,  gyro_y_dps,  gyro_z_dps),
        })
    }
}

#[derive(Copy, Clone)]
pub struct ICM42670FieldMap;

impl FieldMapProvider for ICM42670FieldMap {
    fn get_read_field(name: &str) -> Option<Field> {
        Some(*FIELD_MAP.get(name)?)
    }
    fn get_write_field(name: &str) -> Option<Field> {
        ICM42670FieldMap::get_read_field(name)
    }
}

pub static FIELD_MAP: Map<&'static str, Field> = phf_map! {
    // =========================================================================
    // USER BANK 0 REGISTERS
    // =========================================================================
    "mclk_rdy" => Field { reg: 0x00, offset: 3, bits: 1, writable: false, signed: false, }, //
    "spi_ap_4wire" => Field { reg: 0x01, offset: 2, bits: 1, writable: true, signed: false, }, //
    "spi_mode" => Field { reg: 0x01, offset: 0, bits: 1, writable: true, signed: false, }, //
    "soft_reset_device_config" => Field { reg: 0x02, offset: 4, bits: 1, writable: true, signed: false, }, //
    "fifo_flush" => Field { reg: 0x02, offset: 2, bits: 1, writable: true, signed: false, }, //
    "i3c_ddr_slew_rate" => Field { reg: 0x03, offset: 3, bits: 3, writable: true, signed: false, }, //
    "i3c_sdr_slew_rate" => Field { reg: 0x03, offset: 0, bits: 3, writable: true, signed: false, }, //
    "i2c_slew_rate" => Field { reg: 0x04, offset: 3, bits: 3, writable: true, signed: false, }, //
    "all_slew_rate" => Field { reg: 0x04, offset: 0, bits: 3, writable: true, signed: false, }, //
    "spi_slew_rate" => Field { reg: 0x05, offset: 0, bits: 3, writable: true, signed: false, }, //
    "int2_mode" => Field { reg: 0x06, offset: 5, bits: 1, writable: true, signed: false, }, //
    "int2_drive_circuit" => Field { reg: 0x06, offset: 4, bits: 1, writable: true, signed: false, }, //
    "int2_polarity" => Field { reg: 0x06, offset: 3, bits: 1, writable: true, signed: false, }, //
    "int1_mode" => Field { reg: 0x06, offset: 2, bits: 1, writable: true, signed: false, }, //
    "int1_drive_circuit" => Field { reg: 0x06, offset: 1, bits: 1, writable: true, signed: false, }, //
    "int1_polarity" => Field { reg: 0x06, offset: 0, bits: 1, writable: true, signed: false, }, //
    
    // Sensor Output Channels (Read-Only)
    "temp_data1" => Field { reg: 0x09, offset: 0, bits: 8, writable: false, signed: false, }, //
    "temp_data0" => Field { reg: 0x0a, offset: 0, bits: 8, writable: false, signed: false, }, //
    "accel_data_x1" => Field { reg: 0x0b, offset: 0, bits: 8, writable: false, signed: false, }, //
    "accel_data_x0" => Field { reg: 0x0c, offset: 0, bits: 8, writable: false, signed: false, }, //
    "accel_data_y1" => Field { reg: 0x0d, offset: 0, bits: 8, writable: false, signed: false, }, //
    "accel_data_y0" => Field { reg: 0x0e, offset: 0, bits: 8, writable: false, signed: false, }, //
    "accel_data_z1" => Field { reg: 0x0f, offset: 0, bits: 8, writable: false, signed: false, }, //
    "accel_data_z0" => Field { reg: 0x10, offset: 0, bits: 8, writable: false, signed: false, }, //
    "gyro_data_x1" => Field { reg: 0x11, offset: 0, bits: 8, writable: false, signed: false, }, //
    "gyro_data_x0" => Field { reg: 0x12, offset: 0, bits: 8, writable: false, signed: false, }, //
    "gyro_data_y1" => Field { reg: 0x13, offset: 0, bits: 8, writable: false, signed: false, }, //
    "gyro_data_y0" => Field { reg: 0x14, offset: 0, bits: 8, writable: false, signed: false, }, //
    "gyro_data_z1" => Field { reg: 0x15, offset: 0, bits: 8, writable: false, signed: false, }, //
    "gyro_data_z0" => Field { reg: 0x16, offset: 0, bits: 8, writable: false, signed: false, }, //
    
    "tmst_fsync_data1" => Field { reg: 0x17, offset: 0, bits: 8, writable: false, signed: false, }, //
    "tmst_fsync_data0" => Field { reg: 0x18, offset: 0, bits: 8, writable: false, signed: false, }, //
    "ff_dur_7_0" => Field { reg: 0x1d, offset: 0, bits: 8, writable: false, signed: false, }, //
    "ff_dur_15_8" => Field { reg: 0x1e, offset: 0, bits: 8, writable: false, signed: false, }, //
    
    // Core Configuration & Control
    "accel_lp_clk_sel" => Field { reg: 0x1f, offset: 7, bits: 1, writable: true, signed: false, }, //
    "idle" => Field { reg: 0x1f, offset: 4, bits: 1, writable: true, signed: false, }, //
    "gyro_mode" => Field { reg: 0x1f, offset: 2, bits: 2, writable: true, signed: false, }, //
    "accel_mode" => Field { reg: 0x1f, offset: 0, bits: 2, writable: true, signed: false, }, //
    
    "gyro_ui_fs_sel" => Field { reg: 0x20, offset: 5, bits: 2, writable: true, signed: false, }, //
    "gyro_odr" => Field { reg: 0x20, offset: 0, bits: 4, writable: true, signed: false, }, //
    
    "accel_ui_fs_sel" => Field { reg: 0x21, offset: 5, bits: 2, writable: true, signed: false, }, //
    "accel_odr" => Field { reg: 0x21, offset: 0, bits: 4, writable: true, signed: false, }, //
    
    "temp_filt_bw" => Field { reg: 0x22, offset: 4, bits: 3, writable: true, signed: false, }, //
    
    "gyro_ui_filt_bw" => Field { reg: 0x23, offset: 0, bits: 3, writable: true, signed: false, }, //
    
    "accel_ui_avg" => Field { reg: 0x24, offset: 4, bits: 3, writable: true, signed: false, }, //
    "accel_ui_filt_bw" => Field { reg: 0x24, offset: 0, bits: 3, writable: true, signed: false, }, //
    
    // APEX & FIFO Base Controls
    "dmp_power_save_en" => Field { reg: 0x25, offset: 3, bits: 1, writable: true, signed: false, }, //
    "dmp_init_en" => Field { reg: 0x25, offset: 2, bits: 1, writable: true, signed: false, }, //
    "dmp_mem_reset_en" => Field { reg: 0x25, offset: 0, bits: 1, writable: true, signed: false, }, //
    "smd_enable" => Field { reg: 0x26, offset: 6, bits: 1, writable: true, signed: false, }, //
    "ff_enable" => Field { reg: 0x26, offset: 5, bits: 1, writable: true, signed: false, }, //
    "tilt_enable" => Field { reg: 0x26, offset: 4, bits: 1, writable: true, signed: false, }, //
    "ped_enable" => Field { reg: 0x26, offset: 3, bits: 1, writable: true, signed: false, }, //
    "dmp_odr" => Field { reg: 0x26, offset: 0, bits: 2, writable: true, signed: false, }, //
    "wom_int_dur" => Field { reg: 0x27, offset: 3, bits: 2, writable: true, signed: false, }, //
    "wom_int_mode" => Field { reg: 0x27, offset: 2, bits: 1, writable: true, signed: false, }, //
    "wom_mode" => Field { reg: 0x27, offset: 1, bits: 1, writable: true, signed: false, }, //
    "wom_en" => Field { reg: 0x27, offset: 0, bits: 1, writable: true, signed: false, }, //
    "fifo_mode" => Field { reg: 0x28, offset: 1, bits: 1, writable: true, signed: false, }, //
    "fifo_bypass" => Field { reg: 0x28, offset: 0, bits: 1, writable: true, signed: false, }, //
    "fifo_wm_7_0" => Field { reg: 0x29, offset: 0, bits: 8, writable: true, signed: false, }, //
    "fifo_wm_11_8" => Field { reg: 0x2a, offset: 0, bits: 4, writable: true, signed: false, }, //

    // INT Routing Controls
    "st_int1_en" => Field { reg: 0x2b, offset: 7, bits: 1, writable: true, signed: false, }, //
    "fsync_int1_en" => Field { reg: 0x2b, offset: 6, bits: 1, writable: true, signed: false, }, //
    "pll_rdy_int1_en" => Field { reg: 0x2b, offset: 5, bits: 1, writable: true, signed: false, }, //
    "reset_done_int1_en" => Field { reg: 0x2b, offset: 4, bits: 1, writable: true, signed: false, }, //
    "drdy_int1_en" => Field { reg: 0x2b, offset: 3, bits: 1, writable: true, signed: false, }, //
    "fifo_ths_int1_en" => Field { reg: 0x2b, offset: 2, bits: 1, writable: true, signed: false, }, //
    "fifo_full_int1_en" => Field { reg: 0x2b, offset: 1, bits: 1, writable: true, signed: false, }, //
    "agc_rdy_int1_en" => Field { reg: 0x2b, offset: 0, bits: 1, writable: true, signed: false, }, //
    "i3c_protocol_error_int1_en" => Field { reg: 0x2c, offset: 6, bits: 1, writable: true, signed: false, }, //
    "smd_int1_en" => Field { reg: 0x2c, offset: 3, bits: 1, writable: true, signed: false, }, //
    "wom_z_int1_en" => Field { reg: 0x2c, offset: 2, bits: 1, writable: true, signed: false, }, //
    "wom_y_int1_en" => Field { reg: 0x2c, offset: 1, bits: 1, writable: true, signed: false, }, //
    "wom_x_int1_en" => Field { reg: 0x2c, offset: 0, bits: 1, writable: true, signed: false, }, //
    "st_int2_en" => Field { reg: 0x2d, offset: 7, bits: 1, writable: true, signed: false, }, //
    "fsync_int2_en" => Field { reg: 0x2d, offset: 6, bits: 1, writable: true, signed: false, }, //
    "pll_rdy_int2_en" => Field { reg: 0x2d, offset: 5, bits: 1, writable: true, signed: false, }, //
    "reset_done_int2_en" => Field { reg: 0x2d, offset: 4, bits: 1, writable: true, signed: false, }, //
    "drdy_int2_en" => Field { reg: 0x2d, offset: 3, bits: 1, writable: true, signed: false, }, //
    "fifo_ths_int2_en" => Field { reg: 0x2d, offset: 2, bits: 1, writable: true, signed: false, }, //
    "fifo_full_int2_en" => Field { reg: 0x2d, offset: 1, bits: 1, writable: true, signed: false, }, //
    "agc_rdy_int2_en" => Field { reg: 0x2d, offset: 0, bits: 1, writable: true, signed: false, }, //
    "i3c_protocol_error_int2_en" => Field { reg: 0x2e, offset: 6, bits: 1, writable: true, signed: false, }, //
    "smd_int2_en" => Field { reg: 0x2e, offset: 3, bits: 1, writable: true, signed: false, }, //
    "wom_z_int2_en" => Field { reg: 0x2e, offset: 2, bits: 1, writable: true, signed: false, }, //
    "wom_y_int2_en" => Field { reg: 0x2e, offset: 1, bits: 1, writable: true, signed: false, }, //
    "wom_x_int2_en" => Field { reg: 0x2e, offset: 0, bits: 1, writable: true, signed: false, }, //

    // Tracking & Data Interfaces
    "fifo_lost_pkt_cnt_7_0" => Field { reg: 0x2f, offset: 0, bits: 8, writable: false, signed: false, }, //
    "fifo_lost_pkt_cnt_15_8" => Field { reg: 0x30, offset: 0, bits: 8, writable: false, signed: false, }, //
    "step_cnt_7_0" => Field { reg: 0x31, offset: 0, bits: 8, writable: false, signed: false, }, //
    "step_cnt_15_8" => Field { reg: 0x32, offset: 0, bits: 8, writable: false, signed: false, }, //
    "step_cadence" => Field { reg: 0x33, offset: 0, bits: 8, writable: false, signed: false, }, //
    "dmp_idle" => Field { reg: 0x34, offset: 2, bits: 1, writable: false, signed: false, }, //
    "activity_class" => Field { reg: 0x34, offset: 0, bits: 2, writable: false, signed: false, }, //
    "fifo_count_format" => Field { reg: 0x35, offset: 6, bits: 1, writable: true, signed: false, }, //
    "fifo_count_endian" => Field { reg: 0x35, offset: 5, bits: 1, writable: true, signed: false, }, //
    "sensor_data_endian" => Field { reg: 0x35, offset: 4, bits: 1, writable: true, signed: false, }, //
    "i3c_sdr_en" => Field { reg: 0x36, offset: 3, bits: 1, writable: true, signed: false, }, //
    "i3c_ddr_en" => Field { reg: 0x36, offset: 2, bits: 1, writable: true, signed: false, }, //
    "clksel" => Field { reg: 0x36, offset: 0, bits: 2, writable: true, signed: false, }, //
    
    // Status Registers (Read/Clear on Read)
    "data_rdy_int" => Field { reg: 0x39, offset: 0, bits: 1, writable: false, signed: false, }, //
    "st_int" => Field { reg: 0x3a, offset: 7, bits: 1, writable: false, signed: false, }, //
    "fsync_int" => Field { reg: 0x3a, offset: 6, bits: 1, writable: false, signed: false, }, //
    "pll_rdy_int" => Field { reg: 0x3a, offset: 5, bits: 1, writable: false, signed: false, }, //
    "reset_done_int" => Field { reg: 0x3a, offset: 4, bits: 1, writable: false, signed: false, }, //
    "fifo_ths_int" => Field { reg: 0x3a, offset: 2, bits: 1, writable: false, signed: false, }, //
    "fifo_full_int" => Field { reg: 0x3a, offset: 1, bits: 1, writable: false, signed: false, }, //
    "agc_rdy_int" => Field { reg: 0x3a, offset: 0, bits: 1, writable: false, signed: false, }, //
    "smd_int" => Field { reg: 0x3b, offset: 3, bits: 1, writable: false, signed: false, }, //
    "wom_x_int" => Field { reg: 0x3b, offset: 2, bits: 1, writable: false, signed: false, }, //
    "wom_y_int" => Field { reg: 0x3b, offset: 1, bits: 1, writable: false, signed: false, }, //
    "wom_z_int" => Field { reg: 0x3b, offset: 0, bits: 1, writable: false, signed: false, }, //
    "step_det_int" => Field { reg: 0x3c, offset: 5, bits: 1, writable: false, signed: false, }, //
    "step_cnt_ovf_int" => Field { reg: 0x3c, offset: 4, bits: 1, writable: false, signed: false, }, //
    "tilt_det_int" => Field { reg: 0x3c, offset: 3, bits: 1, writable: false, signed: false, }, //
    "ff_det_int" => Field { reg: 0x3c, offset: 2, bits: 1, writable: false, signed: false, }, //
    "lowg_det_int" => Field { reg: 0x3c, offset: 1, bits: 1, writable: false, signed: false, }, //
    
    // FIFO Ports & ID
    "fifo_count_15_8" => Field { reg: 0x3d, offset: 0, bits: 8, writable: false, signed: false, }, //
    "fifo_count_7_0" => Field { reg: 0x3e, offset: 0, bits: 8, writable: false, signed: false, }, //
    "fifo_data" => Field { reg: 0x3f, offset: 0, bits: 8, writable: false, signed: false, }, //
    "whoami" => Field { reg: 0x75, offset: 0, bits: 8, writable: false, signed: false, }, //
    
    // Indirect Access Hooks
    "blk_sel_w" => Field { reg: 0x79, offset: 0, bits: 8, writable: true, signed: false, }, //
    "maddr_w" => Field { reg: 0x7a, offset: 0, bits: 8, writable: true, signed: false, }, //
    "m_w" => Field { reg: 0x7b, offset: 0, bits: 8, writable: true, signed: false, }, //
    "blk_sel_r" => Field { reg: 0x7c, offset: 0, bits: 8, writable: true, signed: false, }, //
    "maddr_r" => Field { reg: 0x7d, offset: 0, bits: 8, writable: true, signed: false, }, //
    "m_r" => Field { reg: 0x7e, offset: 0, bits: 8, writable: true, signed: false, }, //

    // =========================================================================
    // USER BANK MREG1 REGISTERS (Indirect Access Address Map)
    // =========================================================================
    "tmst_on_sreg_en" => Field { reg: 0x00, offset: 4, bits: 1, writable: true, signed: false, }, //
    "tmst_res" => Field { reg: 0x00, offset: 3, bits: 1, writable: true, signed: false, }, //
    "tmst_delta_en" => Field { reg: 0x00, offset: 2, bits: 1, writable: true, signed: false, }, //
    "tmst_fsync_en" => Field { reg: 0x00, offset: 1, bits: 1, writable: true, signed: false, }, //
    "tmst_en" => Field { reg: 0x00, offset: 0, bits: 1, writable: true, signed: false, }, //
    "fifo_wm_gt_th" => Field { reg: 0x01, offset: 5, bits: 1, writable: true, signed: false, }, //
    "fifo_resume_partial_rd" => Field { reg: 0x01, offset: 4, bits: 1, writable: true, signed: false, }, //
    "fifo_hires_en" => Field { reg: 0x01, offset: 3, bits: 1, writable: true, signed: false, }, //
    "fifo_tmst_fsync_en" => Field { reg: 0x01, offset: 2, bits: 1, writable: true, signed: false, }, //
    "fifo_gyro_en" => Field { reg: 0x01, offset: 1, bits: 1, writable: true, signed: false, }, //
    "fifo_accel_en" => Field { reg: 0x01, offset: 0, bits: 1, writable: true, signed: false, }, //
    "fifo_empty_indicator_dis" => Field { reg: 0x02, offset: 4, bits: 1, writable: true, signed: false, }, //
    "rcosc_req_on_fifo_ths_dis" => Field { reg: 0x02, offset: 0, bits: 1, writable: true, signed: false, }, //
    "fsync_ui_sel" => Field { reg: 0x03, offset: 4, bits: 3, writable: true, signed: false, }, //
    "fsync_ui_flag_clear_sel" => Field { reg: 0x03, offset: 1, bits: 1, writable: true, signed: false, }, //
    "fsync_polarity" => Field { reg: 0x03, offset: 0, bits: 1, writable: true, signed: false, }, //
    "ui_drdy_int_clear" => Field { reg: 0x04, offset: 4, bits: 2, writable: true, signed: false, }, //
    "fifo_ths_int_clear" => Field { reg: 0x04, offset: 2, bits: 2, writable: true, signed: false, }, //
    "fifo_full_int_clear" => Field { reg: 0x04, offset: 0, bits: 2, writable: true, signed: false, }, //
    "int_tpulse_duration" => Field { reg: 0x05, offset: 6, bits: 1, writable: true, signed: false, }, //
    "int_async_reset" => Field { reg: 0x05, offset: 4, bits: 1, writable: true, signed: false, }, //
    "apex_disable" => Field { reg: 0x06, offset: 6, bits: 1, writable: true, signed: false, }, //
    "st_number_sample" => Field { reg: 0x13, offset: 6, bits: 1, writable: true, signed: false, }, //
    "accel_st_lim" => Field { reg: 0x13, offset: 3, bits: 3, writable: true, signed: false, }, //
    "gyro_st_lim" => Field { reg: 0x13, offset: 0, bits: 3, writable: true, signed: false, }, //
    "gyro_st_en" => Field { reg: 0x14, offset: 7, bits: 1, writable: true, signed: false, }, //
    "accel_st_en" => Field { reg: 0x14, offset: 6, bits: 1, writable: true, signed: false, }, //
    "i3c_timeout_en" => Field { reg: 0x23, offset: 4, bits: 1, writable: true, signed: false, }, //
    "i3c_ibi_byte_en" => Field { reg: 0x23, offset: 3, bits: 1, writable: true, signed: false, }, //
    "i3c_ibi_en" => Field { reg: 0x23, offset: 2, bits: 1, writable: true, signed: false, }, //
    "asynctime0_dis" => Field { reg: 0x25, offset: 7, bits: 1, writable: true, signed: false, }, //
    "i3c_ddr_wr_mode" => Field { reg: 0x28, offset: 3, bits: 1, writable: true, signed: false, }, //
    "otp_copy_mode" => Field { reg: 0x2b, offset: 2, bits: 2, writable: true, signed: false, }, //
    
    // MREG1 APEX Motion Parameter Setup
    "ff_int1_en" => Field { reg: 0x2f, offset: 7, bits: 1, writable: true, signed: false, }, //
    "lowg_int1_en" => Field { reg: 0x2f, offset: 6, bits: 1, writable: true, signed: false, }, //
    "step_det_int1_en" => Field { reg: 0x2f, offset: 5, bits: 1, writable: true, signed: false, }, //
    "step_cnt_ofl_int1_en" => Field { reg: 0x2f, offset: 4, bits: 1, writable: true, signed: false, }, //
    "tilt_det_int1_en" => Field { reg: 0x2f, offset: 3, bits: 1, writable: true, signed: false, }, //
    "ff_int2_en" => Field { reg: 0x30, offset: 7, bits: 1, writable: true, signed: false, }, //
    "lowg_int2_en" => Field { reg: 0x30, offset: 6, bits: 1, writable: true, signed: false, }, //
    "step_det_int2_en" => Field { reg: 0x30, offset: 5, bits: 1, writable: true, signed: false, }, //
    "step_cnt_ofl_int2_en" => Field { reg: 0x30, offset: 4, bits: 1, writable: true, signed: false, }, //
    "tilt_det_int2_en" => Field { reg: 0x30, offset: 3, bits: 1, writable: true, signed: false, }, //
    
    // MREG1 In-Band Interrupt (IBI) Maps
    "fsync_ibi_en" => Field { reg: 0x31, offset: 5, bits: 1, writable: true, signed: false, }, //
    "pll_rdy_ibi_en" => Field { reg: 0x31, offset: 4, bits: 1, writable: true, signed: false, }, //
    "ui_drdy_ibi_en" => Field { reg: 0x31, offset: 3, bits: 1, writable: true, signed: false, }, //
    "fifo_ths_ibi_en" => Field { reg: 0x31, offset: 2, bits: 1, writable: true, signed: false, }, //
    "fifo_full_ibi_en" => Field { reg: 0x31, offset: 1, bits: 1, writable: true, signed: false, }, //
    "agc_rdy_ibi_en" => Field { reg: 0x31, offset: 0, bits: 1, writable: true, signed: false, }, //
    "i3c_protocol_error_ibi_en" => Field { reg: 0x32, offset: 7, bits: 1, writable: true, signed: false, }, //
    "ff_ibi_en" => Field { reg: 0x32, offset: 6, bits: 1, writable: true, signed: false, }, //
    "lowg_ibi_en" => Field { reg: 0x32, offset: 5, bits: 1, writable: true, signed: false, }, //
    "smd_ibi_en" => Field { reg: 0x32, offset: 4, bits: 1, writable: true, signed: false, }, //
};