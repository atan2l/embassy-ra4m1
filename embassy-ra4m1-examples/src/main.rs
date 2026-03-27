#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_ra4m1::Config;

use defmt_rtt as _;
use embassy_ra4m1::gpio::{DriveStrength, Level, Output};
use panic_probe as _;

#[embassy_executor::main]
async fn main(_s: Spawner) {
    //info!("Before init");
    let p = embassy_ra4m1::init(Config::default());

    //info!("After init");
    let mut led = Output::new(p.P111, Level::High, DriveStrength::Low);
    //info!("LED is now on");

    //info!("LED is now off");

    loop {
        //cortex_m::asm::delay(15000);
        led.set_low();
        //cortex_m::asm::delay(15000);
        led.set_high();
    }
}
