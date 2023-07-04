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
    use core::mem::MaybeUninit;
    use defmt::*;
    
    use stm32h7xx_hal::{
        gpio::{self, gpiob::PB0, PushPull, Output, Pin},
        delay::Delay,
        prelude::*,
    };

    #[shared] // Multiple tasks can access this struct
    struct Shared {
        led_blink: bool,
        led_pause: u32,
        msg_q: heapless::Deque<blink_proto::Message, 10>, // double-ended queue
    }

    #[local] // Local info for the task that blinks the LED
    struct Local {
        led: Pin<'B', 0, Output>,
        //serial: SerialPort<'static, UsbBus>, //FIXME: adapt this for the USART on STM32
        msg_buf: heapless::Vec<u8, { blink_proto::MSG_BUF_SIZE }>, // message buffer for the blink protocol with a static buffer size
        delay: Delay
    }

    // #[monotonic(binds = SysTick, default = true)]
    // type MyMono = DwtSystick<520_000_000>; // Our STM32H723 CPU is running at 520 MHz on (see the system clock setup below)

    #[init]
    fn init(mut c: init::Context) -> (Shared, Local/*, init::Monotonics*/) {
        let dp = c.device;

        c.core.DCB.enable_trace();
        c.core.DWT.enable_cycle_counter();

        // Constrain and Freeze power
        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        // Constrain and Freeze clock
        let rcc = dp.RCC.constrain();
        let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);    
        
        let delay = c.core.SYST.delay(ccdr.clocks);

        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let led = gpiob.pb0.into_push_pull_output();

        /* FIXME: this should be adapted to use the USART on STM32H7 */
        //let serial = SerialPort::new(usb_bus); // Create a serial port running over the USB device

        let led_blink = true; // Set our variable => we want to blink

        // Transcript from ~45:43

        led_blinker::spawn().ok(); // Command to spawn the task that blinks the LED

        let msg_buf = heapless::Vec::<u8, { blink_proto::MSG_BUF_SIZE }>::new();
        let msg_q = heapless::Deque::<blink_proto::Message, 10>::new(); // We initialize our double-ended queue for messages

        info!("Send me a message!");
        // Init function needs to return the information shared between the tasks
        (
            Shared {
                led_blink,
                led_pause: 500,
                msg_q,
            },
            Local {
                led,
                /*serial,*/
                msg_buf,
                delay,
            },
        )
    }

    #[task(priority = 1, local = [led, delay], shared = [led_blink, led_pause])] // We define the local data we want to access: led (note that only one task can have access to local members)
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
            ctx.local.delay.delay_ms(pause as u16);
        }
    }
    
    /// USB interrupt handler. Runs every time the host requests new data.
    // #[task(binds=USBCTRL_IRQ, local = [serial, usb_dev, msg_buf], shared = [led_blink, led_pause, msg_q])]
    // fn on_usb(mut ctx: on_usb::Context) {
    //     let serial = ctx.local.serial;
    //     if !ctx.local.usb_dev.poll(&mut [serial]) {
    //         return;
    //     }
    //     let mut buf = [0u8; blink_proto::MSG_BUF_SIZE];
    //     let msg_buf = ctx.local.msg_buf;
    //     match serial.read(&mut buf) {
    //         Ok(count) if count > 0 => {
    //             if msg_buf.extend_from_slice(&buf[..count]).is_err() {
    //                 error!("bufcopy {}", count);
    //                 msg_buf.clear();
    //             }
    //             use blink_proto::ParseResult::*;
    //             match blink_proto::parse(msg_buf) {
    //                 Found(msg) => {
    //                     if let blink_proto::Message::Ping { id } = msg {
    //                         info!("{:?}", msg);
    //                         let pong = blink_proto::Message::Pong { id };
    //                         ctx.shared.msg_q.lock(|q| q.push_back(pong).ok());
    //                     }
    //                     if let blink_proto::Message::Led {
    //                         id,
    //                         blinking,
    //                         pause,
    //                     } = msg
    //                     {
    //                         info!("{:?}", msg);
    //                         ctx.shared.led_blink.lock(|b| *b = blinking);
    //                         ctx.shared.led_pause.lock(|p| *p = pause);
    //                         // confirm execution with the same message
    //                         let pong = blink_proto::Message::Led {
    //                             id,
    //                             blinking,
    //                             pause,
    //                         };
    //                         ctx.shared.msg_q.lock(|q| q.push_back(pong).ok());
    //                     }
    //                     msg_buf.clear();
    //                     // Transcript for ~49:56
    //                 }
    //                 Need(b) => {
    //                     debug!("Need({})", b);
    //                 } // continue reading
    //                 HeaderInvalid | DataInvalid => {
    //                     debug!("invalid");
    //                     msg_buf.clear();
    //                 }
    //             }

    //             // write back to the host
    //             loop {
    //                 let msg = ctx.shared.msg_q.lock(|q| q.pop_front());
    //                 let bytes = match msg {
    //                     Some(msg) => match blink_proto::wrap_msg(msg) {
    //                         Ok(msg_bytes) => msg_bytes,
    //                         Err(_) => break,
    //                     },
    //                     None => break,
    //                 };
    //                 let mut wr_ptr = bytes.as_slice();
    //                 while !wr_ptr.is_empty() {
    //                     let _ = serial.write(wr_ptr).map(|len| {
    //                         wr_ptr = &wr_ptr[len..];
    //                     });
    //                 }
    //             }
    //         }
    //         _ => {}
    //     }
    // }

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
