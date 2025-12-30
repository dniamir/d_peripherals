// src/led.rs
use embassy_hal_internal::Peri;
use embassy_time::Timer;

use crate::embassy_hal::gpio::{Pin, Output, Level, OutputDrive};
use crate::d_info;

pub struct Led<'a> {
    led_pin: Output<'a>,
}

impl<'d> Led<'d> {
    pub fn new(pin: Peri<'d, impl Pin>) -> Self {
        // Output::new *owns* the pin in embassy-nrf 0.8
        let led_pin = Output::new(pin, Level::Low, OutputDrive::Standard);
        Self { led_pin }
    }

    pub async fn blink(&mut self, delay_ms: u64) {
        d_info!("LED ON");
        self.led_pin.set_high();
        Timer::after_millis(delay_ms).await;

        d_info!("LED OFF");
        self.led_pin.set_low();
        Timer::after_millis(delay_ms).await;
    }
}
