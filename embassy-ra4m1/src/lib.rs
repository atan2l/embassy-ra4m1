#![no_std]
pub mod gpio;

use embassy_hal_internal::peripherals;
use ra4m1 as pac;

include!(concat!(env!("OUT_DIR"), "/peripherals.rs"));

/// Top-level configuration passed to `init()`.
/// Empty for now, clock config will live here once we tackle that subsystem.
#[derive(Default)]
pub struct Config {
    // pub rcc: rcc::Config, // future
}

/// Initialise the HAL and return all peripheral tokens.
///
/// # Panics
/// Panics if called more than once. This enforces the singleton invariant:
/// peripheral tokens can only be handed out once, ensuring the type system's exclusivity
/// guarantees are backed by a runtime check at the one place where they originate.
pub fn init(_config: Config) -> Peripherals {
    /*
     * Future: configure clocks, enable peripheral clocks, etc
     * For now the chip runs on the default MOCO at 8MHz, sufficient for GPIO
     */

    Peripherals::take()
}
