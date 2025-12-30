use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::{Timer, Duration, with_timeout};
use embassy_nrf::twim::{Twim, Error as TwimError};
use core::future::Future;

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

// Async function to add a delay to a command
async fn add_timeout<Fut>(op: Fut, timeout_ms: u64, recovery_delay_ms: u64) -> Result<(), CommError>
where
    Fut: Future<Output = Result<(), CommError>>,
{
    DLogger::hold();
    
    let timeout_func = with_timeout(
        Duration::from_millis(timeout_ms),
        op,
    ).await;

    let _result = match timeout_func {
        Ok(comm_result) => {
            if comm_result.is_err() { 
                d_info!("Comm Error");
            } 
            else { 
                d_info!("Comm Success");
            }
        }
        Err(_) => {
            d_info!("Comm timeout");
            Timer::after_millis(recovery_delay_ms).await;
        }
    };

    DLogger::release();
    Ok(())
}


// write_read and write for nRF52840
impl CommProvider for I2CMutexWrapper {
    async fn write_read(&self, i2c_address: u8, reg: u8, reg_vals: &mut [u8]) -> Result<(), CommError> {

        // Get TWIM from MUTEX
        let mut twim = self.0.lock().await;

        // Define communication without calling it
        let reg_buf: [u8; 1] = [reg];
        let com = twim.write_read(i2c_address, &reg_buf, reg_vals);

        // Call communication with a timeout
        add_timeout(
            async { Ok(com.await?) }, 
            200, 
            200,
        ).await
    }

    async fn write(&self, i2c_address: u8, reg: u8, reg_val: u8) -> Result<(), CommError> {
        
        // Get TWIM from MUTEX
        let mut twim = self.0.lock().await;

        // Define communication without calling it
        let reg_buf: [u8; 2] = [reg, reg_val];
        let com = twim.write(i2c_address, &reg_buf);
        
        // Call communication with a timeout
        add_timeout(
            async { Ok(com.await?) }, 
            200, 
            200,
        ).await                      
    }
}
