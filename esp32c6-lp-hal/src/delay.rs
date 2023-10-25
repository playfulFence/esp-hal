//! Simple blocking delay functionality
//!
//! This uses the delay functionality provided by the `riscv` crate under the
//! hood.

use embedded_hal::blocking::delay::{DelayMs, DelayUs};

use crate::CPU_CLOCK;

pub struct Delay {
    rv_delay: riscv::delay::McycleDelay,
}

impl Delay {
    pub fn new() -> Self {
        Self {
            rv_delay: riscv::delay::McycleDelay::new(unsafe { CPU_CLOCK }),
        }
    }
}

impl Default for Delay {
    fn default() -> Self {
        Self::new()
    }
}

impl DelayUs<u64> for Delay {
    #[inline(always)]
    fn delay_us(&mut self, us: u64) {
        self.rv_delay.delay_us(us);
    }
}

impl DelayUs<u32> for Delay {
    #[inline(always)]
    fn delay_us(&mut self, us: u32) {
        self.rv_delay.delay_us(us);
    }
}

impl DelayMs<u32> for Delay {
    #[inline(always)]
    fn delay_ms(&mut self, ms: u32) {
        self.rv_delay.delay_ms(ms);
    }
}

impl DelayMs<u16> for Delay {
    #[inline(always)]
    fn delay_ms(&mut self, ms: u16) {
        self.rv_delay.delay_ms(ms);
    }
}

impl DelayUs<u16> for Delay {
    #[inline(always)]
    fn delay_us(&mut self, us: u16) {
        self.rv_delay.delay_us(us);
    }
}
