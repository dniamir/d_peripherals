use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_nrf::twim::{Twim};

use crate::d_peripherals::chip::{I2CProvider, I2CError};


// Trait defined for embassy nRF52840 I2C mutex
pub struct I2CMutexWrapper(pub &'static Mutex<ThreadModeRawMutex, Twim<'static>>);

impl I2CProvider for I2CMutexWrapper {
    async fn write_read(&self, i2c_address: u8, reg: u8, reg_vals: &mut [u8]) -> Result<(), I2CError> {
        let mut twim = self.0.lock().await;
        twim.write_read(i2c_address, &[reg], reg_vals).await?;
        Ok(())
    }

    async fn write(&self, i2c_address: u8, reg: u8, reg_val: u8) -> Result<(), I2CError> {
        let mut twim = self.0.lock().await;
        twim.write(i2c_address, &[reg, reg_val]).await?;
        Ok(())
    }
}
