# Introduction to Rust programming on bare metal hardware

Rust code accompanying a presentation by Michael Kefeder showing how to:

- compile and run firmware on NUCLEO-H723ZG
- communicate between host to NUCLEO-H723ZG using a custom binary protocol
- introduction into RTIC v2

The original presentation and code mentionned above target a Raspberry Pico board. In this repo, that code has been ported to allow the demo to run on a NUCLEO-H723ZG board

> **Note**
> The initial demo from Mike Kefeder showed how to make a USB CDC compatible device on the RP2040.
> We don't need this part on the NUCLEO-H723ZG because that board provides the host with a virtual serial port directly attached to the STM32 chip's USART

Video of the presentation done at Rust Zürisee March 2023: [Introduction to Rust programming on bare metal hardware](https://youtu.be/KECu_piSM5s)

## pre-requisites

Ater setting up a working [Rust environment](https://rustup.rs/) including nightly compiler:

```shell
# install compilation targets
rustup target add thumbv7em-none-eabihf
# Useful for flashing over the SWD pins using the J-Link probe embedded on the NUCLEO-H723ZG board
cargo install probe-run
# most comfortable way of running and looking at logs
cargo install cargo-embed
```

for GDB debugging environment follow [OS Specific instructions](https://docs.rust-embedded.org/book/intro/install.html#os-specific-instructions)

## overview of code

- `nucleoh723zg-blink` the classic hello world of embedded blinking a LED
- `blink-host` host client for the serial port over USB protocol
- `blink-proto` the protocol implementation for host using `std` and RP2040 `no_std`
- `nucleoh723zg-rtic-usart3-serial-blinky` the usb-serial communication firmware using RTIC v2 alpha (needs rust nightly)
