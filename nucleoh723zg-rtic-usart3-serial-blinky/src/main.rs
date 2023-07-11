//! example of using RTIC to communicate over serial port and blink LED
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_rtt as _;
use panic_probe as _;

defmt::timestamp! {"{=u64}", {
    static COUNT: AtomicUsize = AtomicUsize::new(0); // Transcript from ~40:30
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n as u64
}
}

#[rtic::app(device = stm32h7xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {
    use defmt::*;
    
    use stm32h7xx_hal as hal;
    use crate::app::hal::{
        gpio::{Output, Pin},
        device::USART3,
        serial::{Event, Serial},
        prelude::*,
    };
    use rtic_monotonics::systick::*;
    use core::fmt::Write;
    use hal::nb as nb;
    use nb::block;

    #[shared] // Multiple tasks can access this struct
    struct Shared {
        led_blink: bool,
        led_pause: u32,
        msg_q: heapless::Deque<blink_proto::Message, 10>, // double-ended queue
    }

    #[local] // Local info for the task that blinks the LED
    struct Local {
        led: Pin<'B', 0, Output>,
        serial: Serial<USART3>,
        msg_buf: heapless::Vec<u8, { blink_proto::MSG_BUF_SIZE }>, // message buffer for the blink protocol with a static buffer size
    }

    #[init]
    fn init(mut c: init::Context) -> (Shared, Local) {
        /* Note: RTICv2 example [here](https://rtic.rs/2/book/en/migration_v1_v2/complete_example.html) is a good porting guide and is using STM32 target */
        let dp = c.device;

        c.core.DCB.enable_trace();
        c.core.DWT.enable_cycle_counter();

        // Constrain and Freeze power
        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        // Constrain and Freeze clock
        let rcc = dp.RCC.constrain();

        let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(c.core.SYST, 100_000_000, systick_token);
        
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let led = gpiob.pb0.into_push_pull_output();

        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let tx_pin = gpiod.pd8.into_alternate();
        let rx_pin = gpiod.pd9.into_alternate();
        let mut serial = dp.USART3.serial(
                                          (tx_pin, rx_pin),
                                          19_200.bps(),
                                          ccdr.peripheral.USART3,
                                          &ccdr.clocks,
                                         ).unwrap();
    
        let led_blink = true; // Set our variable => we want to blink

        // Transcript from ~45:43

        led_blinker::spawn().ok(); // Command to spawn the task that blinks the LED

        let msg_buf = heapless::Vec::<u8, { blink_proto::MSG_BUF_SIZE }>::new();
        let msg_q = heapless::Deque::<blink_proto::Message, 10>::new(); // We initialize our double-ended queue for messages

        info!("Send me a message!");
        serial.write_str("Send me a message!\n").expect("Failed writing to USART");

        serial.listen(Event::Rxne);
        // Init function needs to return the information shared between the tasks
        (
            Shared {
                led_blink,
                led_pause: 500,
                msg_q,
            },
            Local {
                led,
                serial,
                msg_buf,
            },
        )
    }

    #[task(priority = 1, local = [led], shared = [led_blink, led_pause])] // We define the local data we want to access: led (note that only one task can have access to local members)
    async fn led_blinker(mut ctx: led_blinker::Context) {
        loop {
            let mut pause = 500;
            // Shared information must be locked. ctx is auto-populated from local and shared args in the task directive
            (&mut ctx.shared.led_blink, &mut ctx.shared.led_pause).lock(|led_blink, led_pause| { // Closure will be run once the lock is grabbed
                if *led_blink {
                    ctx.local.led.toggle();
                }
                pause = *led_pause;
            });
            Systick::delay((pause as u32).millis()).await;
        }
    }
    
    /* We could also try to use DMA (see https://github.com/kalkyl/f303-rtic/blob/main/src/bin/serial.rs) */
    /// USB interrupt handler. Runs every time the host requests new data.
    #[task(binds=USART3, local = [serial, msg_buf], shared = [led_blink, led_pause, msg_q])]
    fn on_usb(mut ctx: on_usb::Context) {
        let serial = ctx.local.serial;
        let msg_buf = ctx.local.msg_buf;
        if serial.is_rxne() {
            match serial.read() {
                Ok(rx_byte) => {
                    info!("Received {:#04x}", rx_byte);
                    if msg_buf.push(rx_byte).is_err() {
                        error!("bufcopy 1 byte");
                        msg_buf.clear();
                    }
                    use blink_proto::ParseResult::*;
                    match blink_proto::parse(msg_buf) {
                        Found(msg) => {
                            if let blink_proto::Message::Ping { id } = msg {
                                info!("{:?}", msg);
                                let pong = blink_proto::Message::Pong { id };
                                ctx.shared.msg_q.lock(|q| q.push_back(pong).ok());
                            }
                            if let blink_proto::Message::Led {
                                id,
                                blinking,
                                pause,
                            } = msg
                            {
                                info!("{:?}", msg);
                                ctx.shared.led_blink.lock(|b| *b = blinking);
                                ctx.shared.led_pause.lock(|p| *p = pause);
                                // confirm execution with the same message
                                let pong = blink_proto::Message::Led {
                                    id,
                                    blinking,
                                    pause,
                                };
                                ctx.shared.msg_q.lock(|q| q.push_back(pong).ok());
                            }
                            msg_buf.clear();
                            // Transcript for ~49:56
                        }
                        Need(b) => {
                            debug!("Need({})", b);
                        } // continue reading
                        HeaderInvalid | DataInvalid => {
                            debug!("invalid");
                            msg_buf.clear();
                        }
                    }

                    // write back to the host
                    loop {
                        let msg = ctx.shared.msg_q.lock(|q| q.pop_front());
                        let bytes = match msg {
                            Some(msg) => match blink_proto::wrap_msg(msg) {
                                Ok(msg_bytes) => msg_bytes,
                                Err(_) => break,
                            },
                            None => break,
                        };
                        let wr_ptr = bytes.as_slice();
                        let _ = wr_ptr.iter().map(|c| block!(serial.write(*c))).last();
                    }
                }
                _ => {}
            }
        }
    }

    /// Task with least priority that only runs when nothing else is running.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        /* Could also be (to avoid rtic sending the board to sleep):
        loop {
            cortex_m::asm::nop();
        }
         */
        // debug::exit(debug::EXIT_SUCCESS);
        loop {
            // hprintln!("idle");
            cortex_m::asm::wfi(); // put the MCU in sleep mode until interrupt occurs
        }
    }
}
