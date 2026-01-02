use core::cell::RefCell;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::{Timer, Duration, with_timeout};
use embassy_nrf::twim::{Twim, Error as TwimError};
use core::future::Future;

use crate::d_peripherals::chip::CommProvider;
use crate::{d_log::dlogger::DLogger, d_info, d_force};  // Logging


// Trait defined for embassy nRF52840 I2C mutex
pub trait Addressable {
    fn set_address(&mut self, address: u8);
}

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

#[derive(Clone)]
pub struct NRFI2CMutex {
    pub mutex: &'static Mutex<ThreadModeRawMutex, Twim<'static>>,
    pub i2c_address: Option<u8>,
}
impl Addressable for NRFI2CMutex {
    fn set_address(&mut self, address: u8) {
        self.i2c_address = Some(address);
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
                d_force!("----Comm Error----");
            } 
            else { 
                d_info!("Comm Success");
            }
        }
        Err(_) => {
            d_force!("----Comm timeout----");
            Timer::after_millis(recovery_delay_ms).await;
        }
    };

    DLogger::release();
    Ok(())
}


/// Implementation of CommProvider for I2C mutex on nRF52840
impl CommProvider for NRFI2CMutex {
    async fn write_read(&self, reg: u8, reg_vals: &mut [u8]) -> Result<(), CommError> {

        // Get TWIM from MUTEX
        let mut twim = self.mutex.lock().await;

        // Define communication without calling it
        let reg_buf: [u8; 1] = [reg];
        let i2c_address = self.i2c_address.unwrap();
        let com = twim.write_read(i2c_address, &reg_buf, reg_vals);

        // Call communication with a timeout
        add_timeout(
            async { Ok(com.await?) }, 
            200, 
            200,
        ).await
    }

    async fn write(&self, reg: u8, reg_val: u8) -> Result<(), CommError> {
        
        // Get TWIM from MUTEX
        let mut twim = self.mutex.lock().await;

        // Define communication without calling it
        let reg_buf: [u8; 2] = [reg, reg_val];
        let i2c_address = self.i2c_address.unwrap();
        let com = twim.write(i2c_address, &reg_buf);
        
        // Call communication with a timeout
        add_timeout(
            async { Ok(com.await?) }, 
            200, 
            200,
        ).await                      
    }
}


/// Struct for communicating with a shadow map instead of actual communication
/// eg all reads/writes can be done with a shadow register map, and then later
/// done with i2cmutex

pub struct ShadowComm<COMM> {
    
    pub provider: COMM,                         // A normal comm provider, like the I2C mutex
    pub shadow_registers: RefCell<[u8; 256]>,   // The shadow register map
    pub dirty_bits: RefCell<[u8; 256]>,             // Tracks registers that have been updated
}

impl<COMM> ShadowComm<COMM> {

    /// Checks if a specific register has been modified 
    pub fn is_dirty(&self, reg: u8) -> bool {
        let reg_idx = reg as usize;
        let byte = self.dirty_bits.borrow()[reg_idx];
        byte != 0
    }

    // Resets the entire tracking back to clean
    pub fn clear_dirty(&self) {
        self.dirty_bits.borrow_mut().fill(0);
    }

    /// Resets the shadow register map to 0
    pub fn reset_shadow(&self) {
        // 1. Set all register values to 0
        self.shadow_registers.borrow_mut().fill(0);
        
        // 2. Mark everything as "Clean"
        self.clear_dirty();
    }

}

impl<COMM> Addressable for ShadowComm<COMM>
where 
    COMM: Addressable
{
    fn set_address(&mut self, address: u8) {
        self.provider.set_address(address);
    }
}

impl<Comm> ShadowComm<Comm> 
where 
    Comm: CommProvider
{
    pub fn new(comm: Comm) -> Self {
        Self {
            provider: comm,
            shadow_registers: RefCell::new([0u8; 256]),
            dirty_bits: RefCell::new([0u8; 256]),
        }
    }

    /// Explicitly sync a specific register
    pub async fn true_raw_write(&self, reg: u8, reg_val: u8) -> Result<(), CommError> {
        self.provider.write(reg, reg_val).await
    }

    /// Write all the shadow register values to the device
    /// Only writes dirty registers
    pub async fn sync_all(&self) -> Result<(), CommError> {
        
        let shadow = self.shadow_registers.borrow();
        
        // Iterate directly through the array
        for (reg, &reg_val) in shadow.iter().enumerate() {

            // Only write dirty registers
            if !self.is_dirty(reg as u8) { continue; }

            // Attempt the hardware write
            d_info!("Write Register: 0x{=u8:X}, {=u8:b}, 0x{=u8:X}, {}", reg as u8, reg_val, reg_val, reg_val);        
            self.true_raw_write(reg as u8, reg_val).await?;

        }

        Ok(())
    }
}

impl<COMM> CommProvider for ShadowComm<COMM> {

    // Write and read to and from the shadow register field map
    async fn write_read(&self, reg: u8, reg_vals: &mut [u8]) -> Result<(), CommError> {
        let shadow = self.shadow_registers.borrow();
        let start = reg as usize;
        let end = start + reg_vals.len();
        reg_vals.copy_from_slice(&shadow[start..end]);
        Ok(())
    }

    // Write to shadow register field map
    async fn write(&self, reg: u8, reg_val: u8) -> Result<(), CommError> {

        let reg_idx = reg as usize;

        // Update shadow register value
        self.shadow_registers.borrow_mut()[reg_idx] = reg_val;

        // Mark as dirty (1 = Dirty, 0 = Clean)
        self.dirty_bits.borrow_mut()[reg_idx] = 1;

        Ok(())
    }
}