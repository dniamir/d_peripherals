use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_nrf::twim::{Twim};

use crate::d_peripherals::chip::CommProvider;
use crate::{d_log::dlogger::DLogger, d_info};  // Logging


// Trait defined for embassy nRF52840 I2C mutex
#[derive(Clone)]
pub struct I2CMutexWrapper(pub &'static Mutex<ThreadModeRawMutex, Twim<'static>>);


#[derive(Debug)]
pub enum CommError {
    NotFound,
    TwimError(TwimError),
    HangUp,
}

impl From<TwimError> for CommError {
    fn from(err: TwimError) -> Self {
        CommError::TwimError(err)
    }
}
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
