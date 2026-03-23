#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_ra4m1::Config;

use defmt_rtt as _;
use embassy_ra4m1::gpio::{DriveStrength, Level, Output};
use panic_probe as _;

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let p = embassy_ra4m1::init(Config::default());

    let led = Output::new(p.P111, Level::High, DriveStrength::Low);

    loop {}
}
