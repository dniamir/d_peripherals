use core::cell::RefCell;
use embedded_hal_async::i2c::{I2c, ErrorType, Operation};
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
    OutOfBounds,
}

impl From<TwimError> for CommError {
    fn from(err: TwimError) -> Self {
        CommError::TwimError(err)
    }
}

impl embedded_hal::i2c::Error for CommError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

#[derive(Clone, Copy)]
pub struct NRFI2CMutex {
    pub mutex: &'static Mutex<ThreadModeRawMutex, Twim<'static>>,
    pub i2c_address: Option<u8>,
}
impl Addressable for NRFI2CMutex {
    fn set_address(&mut self, address: u8) {
        self.i2c_address = Some(address);
    }
}

// Map the error type so the driver knows how to handle failures
impl ErrorType for NRFI2CMutex {
    type Error = CommError; 
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

    // Capture the result so we can return it after releasing the logger
    let result = match timeout_func {
        Ok(comm_result) => {
            if comm_result.is_err() { 
                d_force!("----Comm Error----");
            } else { 
                d_info!("Comm Success");
            }
            comm_result // Propagate the actual I2C/Comm error
        }
        Err(_) => {
            d_force!("----Comm timeout----");
            // Hardware recovery: wait for the IS3 chip or I2C bus to stabilize
            Timer::after_millis(recovery_delay_ms).await;
            Err(CommError::HangUp) // Return a specific timeout error
        }
    };

    DLogger::release();
    result 
}


/// Implementation of CommProvider for I2C mutex on nRF52840
impl CommProvider for NRFI2CMutex {
    async fn write_read(&self, regs: &[u8], reg_vals: &mut [u8]) -> Result<(), CommError> {

        // Get TWIM from MUTEX
        let mut twim = self.mutex.lock().await;

        // Define communication without calling it
        let i2c_address = self.i2c_address.unwrap();
        let com = twim.write_read(i2c_address, regs, reg_vals);

        // Call communication with a timeout
        add_timeout(
            async { Ok(com.await?) }, 
            200, 
            200,
        ).await
    }

    async fn write(&self, regs: &[u8], reg_val: &[u8]) -> Result<(), CommError> {
        
        // Get TWIM from MUTEX
        let mut twim = self.mutex.lock().await;

        // Add both reg and reg_val together
        let total_len = regs.len() + reg_val.len(); // Calculate the combined length
        let mut reg_buff = [0u8; 32];
        reg_buff[..regs.len()].copy_from_slice(regs);
        reg_buff[regs.len()..total_len].copy_from_slice(reg_val);
        
        // Define communication without calling it
        let i2c_address = self.i2c_address.unwrap();
        let com = twim.write(i2c_address, &reg_buff[..total_len]);
        
        // Call communication with a timeout
        add_timeout(
            async { Ok(com.await?) }, 
            200, 
            200,
        ).await                      
    }
}

// Implementation of embedded_hal_async I2C for NRFI2CMutex
impl I2c for NRFI2CMutex {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        // Bridge to your existing mutex logic
        let mut twim = self.mutex.lock().await;
        add_timeout(async { Ok(twim.read(address, read).await?) }, 200, 200).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        // You can call your existing CommProvider::write here 
        // OR just reimplement the mutex lock logic to match the address argument
        let mut twim = self.mutex.lock().await;
        add_timeout(async { Ok(twim.write(address, write).await?) }, 200, 200).await
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        let mut twim = self.mutex.lock().await;
        add_timeout(async { Ok(twim.write_read(address, write, read).await?) }, 200, 200).await
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut twim = self.mutex.lock().await;

        // Wrap the entire transaction in your timeout logic
        add_timeout(async {
            for op in operations {
                match op {
                    Operation::Read(buf) => twim.read(address, buf).await?,
                    Operation::Write(buf) => twim.write(address, buf).await?,
                }
            }
            Ok(())
        }, 200, 200).await
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
        self.provider.write(&[reg], &[reg_val]).await
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
    async fn write_read(&self, regs: &[u8], reg_vals: &mut [u8]) -> Result<(), CommError> {
        
        // Borrow the shadow and dirty bits
        let shadow = self.shadow_registers.borrow();

        // Pair each register address with each slot in reg_vals
        for (&reg_addr, output_slot) in regs.iter().zip(reg_vals.iter_mut()) {
            let idx = reg_addr as usize;
            
            // Safety check to prevent out-of-bounds panics
            if idx < shadow.len() {
                *output_slot = shadow[idx];
            } else {
                return Err(CommError::OutOfBounds);
            }
        }

        Ok(())
    }

    // Write to shadow register field map
    async fn write(&self, regs: &[u8], reg_vals: &[u8]) -> Result<(), CommError> {

        // Borrow the shadow and dirty bits
        let mut shadow = self.shadow_registers.borrow_mut();
        let mut dirty = self.dirty_bits.borrow_mut();

        // Iterate through the registers and update the shadow
        for (&reg, &reg_val) in regs.iter().zip(reg_vals.iter()) {
            let reg_idx = reg as usize;
            shadow[reg_idx] = reg_val;
            dirty[reg_idx] = reg_val;
        }

        Ok(())
    }
}