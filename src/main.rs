#![no_std]
#![no_main]

// Neopixel Rainbow
// This only functions when the --release version is compiled. Using the debug
// version leads to slow pulse durations which results in a straight white LED
// output.
//
// // Needs to be compiled with --release for the timing to be correct

use panic_persist;

mod mpu6050;

use feather_m4 as bsp;
use bsp::hal;

use bsp::{entry, periph_alias, pin_alias};

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::time::Hertz;
use hal::timer::*;
use hal::sercom::{i2c, uart};
use hal::gpio::{Pin, PinId, PushPullOutput};
use hal::qspi;

use core::fmt::Write;
use heapless;

use smart_leds::{
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
    RGB,
};
use ws2812_timer_delay::Ws2812;



#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );
    let pins = bsp::Pins::new(peripherals.PORT);
    let mut delay = Delay::new(core.SYST, &mut clocks);
    let mut red_led = pins.d13.into_push_pull_output();
    red_led.set_low().unwrap();


    // Setup UART peripheral
    let uart_sercom = periph_alias!(peripherals.uart_sercom);
    
    let board_uart = bsp::uart(
        &mut clocks,
        115200.Hz(),
        uart_sercom,
        &mut peripherals.MCLK,
        pin_alias!(pins.uart_rx),
        pin_alias!(pins.uart_tx),
    );
    let (mut _board_uart_rx, mut board_uart_tx) = board_uart.split();
    let mut scratch_string: heapless::String<256> = heapless::String::new();

    // Check if there was a panic message, if so, send to UART and noop loop
    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        write_to_uart(&mut board_uart_tx, b"\r\n\r\n");
        write_to_uart(&mut board_uart_tx, msg);
        write_to_uart(&mut board_uart_tx, b"\r\nHalting for panic\r\n\r\n");
        loop { }
    }


    // set up timer for neopixel
    let gclk0 = clocks.gclk0();
    let timer_clock = clocks.tc2_tc3(&gclk0).unwrap();
    let mut timer = TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK);
    timer.start(Hertz::MHz(3).into_duration());

    // set up neopixel
    let neopixel_pin = pins.neopixel.into_push_pull_output();
    let mut neopixel = Ws2812::new(timer, neopixel_pin);

    // set up MPU6050
    let (sda, scl) = (pins.sda, pins.scl);
    let sercom2_clock = &clocks.sercom2_core(&gclk0).unwrap();
    let i2c_pads = i2c::Pads::new(sda, scl);
    let i2c_sercom = periph_alias!(peripherals.i2c_sercom);
    let mut i2c = i2c::Config::new(&peripherals.MCLK, i2c_sercom, i2c_pads, sercom2_clock.freq())
        .baud(100.kHz())
        .enable();

    neopixel.write([RGB {r:0, g:0, b:20}].into_iter()).unwrap();

    mpu6050::mpu6050_setup(&mut i2c, &mut delay, 0x68);

    neopixel.write([RGB {r:20, g:20, b:0}].into_iter()).unwrap();
    
    neopixel.write([RGB {r:0, g:20, b:0}].into_iter()).unwrap();

    loop {
        let result = mpu6050::mpu6050_read_latest(&mut i2c, 0x68);
        scratch_string.clear();
        core::write!(&mut scratch_string, "qx:{}, qy:{}, qz:{}, qw:{}\r\n", result.qx, result.qy, result.qz, result.qw).unwrap();
        write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());

        scratch_string.clear();
        core::write!(&mut scratch_string, "gx:{}, gy:{}, gz:{}\r\n", result.gyro_x, result.gyro_y, result.gyro_z).unwrap();
        core::write!(&mut scratch_string, "ax:{}, ay:{}, az:{}\r\n", result.accel_x, result.accel_y, result.accel_z).unwrap();
        write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());

        write_to_uart(&mut board_uart_tx, b"\r\n");

        delay.delay_ms(500u16);
    }

    // Loop through all of the available hue values (colors) to make a
    // rainbow effect from the onboard neopixel
    // loop {
    //     for j in 0..255u8 {
    //         let colors = [hsv2rgb(Hsv {
    //             hue: j,
    //             sat: 255,
    //             val: 2,
    //         })];
    //         neopixel.write(colors.iter().cloned()).unwrap();
    //         delay.delay_ms(5u8);
    //     }
    // }
}

fn write_to_uart<T: uart::ValidConfig>(tx: &mut uart::Uart<T, uart::TxDuplex>, 
                 msg: &[T::Word]) {
    for c in msg.iter() {
        nb::block!(tx.write(*c)).unwrap();
    }
}

#[allow(dead_code)]
fn blink_led<P>(ms: u16, n:usize, led: &mut Pin<P, PushPullOutput>, delay: &mut Delay) 
where
    P: PinId,
{
    for _ in 0..n {
        led.set_high().unwrap();
        delay.delay_ms(ms);
        led.set_low().unwrap();
        delay.delay_ms(ms);
    }
}