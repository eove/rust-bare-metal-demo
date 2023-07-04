//! Blinks the LED on a NUCLEO-H723ZG board
//!
//! This will blink an LED attached to port B0, which is the pin wired to the on-board green LED.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use stm32h7xx_hal::{pac, prelude::*};
use panic_probe as _;

#[entry]
fn main() -> ! {
    info!("Program start");
    let core = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    let mut delay = core.SYST.delay(ccdr.clocks);

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let mut led_pin = gpiob.pb0.into_push_pull_output();

    loop {
        info!("on!");
        led_pin.set_high();
        delay.delay_ms(500_u16);
        info!("off!");
        led_pin.set_low();
        delay.delay_ms(500_u16);
    }
}
