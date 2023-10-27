#![no_std]
#![no_main]

// // Needs to be compiled with --release for the timing to be correct

use panic_persist;

mod mpu6050;

use cortex_m::asm;

use feather_m4 as bsp;
use bsp::{entry, periph_alias, hal};

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::time::Hertz;
use hal::timer::*;
use hal::sercom::{i2c, uart};
use hal::gpio::{Pin, PinId, PushPullOutput};

//use core::fmt::Write;
//use heapless;

use smart_leds::{
    SmartLedsWrite,
    RGB,
};
use ws2812_timer_delay::Ws2812;

const MPU6050_ADDR: u8 = 0x68;

const TAIL_STRIP_NPIX: usize = 20;
const HEAD_STRIP_NPIX: usize = 35;


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
    
    // cannot use the bsp pins because they don't expose the qspi...
    //let pins = bsp::Pins::new(peripherals.PORT);
    let pins = hal::gpio::Pins::new(peripherals.PORT);
    let pd13 = pins.pa23;
    let puart_rx = pins.pb17;
    let puart_tx = pins.pb16;
    let pneopixel = pins.pb03;
    let pscl = pins.pa13;
    let psda = pins.pa12;
    let d4 = pins.pa14;  //tail strip
    let d5 = pins.pa16;  //head strip
    let mut delay = Delay::new(core.SYST, &mut clocks);
    
    let mut red_led = pd13.into_push_pull_output();
    red_led.set_high().unwrap();

    // set up timer for neopixel
    let gclk0 = clocks.gclk0();
    let timer_clock = clocks.tc2_tc3(&gclk0).unwrap();
    let mut timer = TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK);
    timer.start(Hertz::MHz(3).into_duration());

    // set up built-in neopixel
    let neopixel_pin = pneopixel.into_push_pull_output();
    let mut neopixel = Ws2812::new(timer, neopixel_pin);

    // Neopixel starts blue for setup
    neopixel.write([RGB {r:0, g:0, b:20}].into_iter()).unwrap();

    red_led.set_low().unwrap();

    // Setup UART peripheral
    let uart_sercom = periph_alias!(peripherals.uart_sercom);
    
    let board_uart = bsp::uart(
        &mut clocks,
        115200.Hz(),
        uart_sercom,
        &mut peripherals.MCLK,
        puart_rx,
        puart_tx
    );
    let (mut _board_uart_rx, mut board_uart_tx) = board_uart.split();
    //let mut scratch_string: heapless::String<2560> = heapless::String::new();

    // Check if there was a panic message, if so, send to UART and noop loop
    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        write_to_uart(&mut board_uart_tx, b"\r\n\r\n");
        write_to_uart(&mut board_uart_tx, b"Previous panic message:\r\n");
        write_to_uart(&mut board_uart_tx, msg);
        write_to_uart(&mut board_uart_tx, b"\r\nHalting for panic\r\n\r\n");
        halt(&mut delay);
    }
    

    // set up MPU6050
    let sercom2_clock = &clocks.sercom2_core(&gclk0).unwrap();
    let (sda, scl) = (psda, pscl);
    let i2c_pads = i2c::Pads::new(sda, scl);
    let i2c_sercom = periph_alias!(peripherals.i2c_sercom);
    let mut i2c = i2c::Config::new(&peripherals.MCLK, i2c_sercom, i2c_pads, sercom2_clock.freq())
        .baud(100.kHz())
        .enable();
    
    mpu6050::mpu6050_setup(&mut i2c, &mut delay, MPU6050_ADDR);
    
    // Neopixel turns magenta after main setup
    neopixel.write([RGB {r:0, g:20, b:20}].into_iter()).unwrap();

    // set up the strips, initialize 10% power white.
    let timer_clock45 = clocks.tc4_tc5(&gclk0).unwrap();

    let mut timer4 = TimerCounter::tc4_(&timer_clock45, peripherals.TC4, &mut peripherals.MCLK);
    timer4.start(Hertz::MHz(3).into_duration());
    let mut head_strip = Ws2812::new(timer4, d4.into_push_pull_output());

    let mut timer5 = TimerCounter::tc5_(&timer_clock45, peripherals.TC5, &mut peripherals.MCLK);
    timer5.start(Hertz::MHz(3).into_duration());
    let mut tail_strip = Ws2812::new(timer5, d5.into_push_pull_output());

    let mut head_colors = [RGB {r:25, g:25, b:25} ; HEAD_STRIP_NPIX];
    let mut tail_colors = [RGB {r:25, g:25, b:25} ; TAIL_STRIP_NPIX];

    head_strip.write(head_colors.into_iter()).unwrap();
    tail_strip.write(tail_colors.into_iter()).unwrap();

    // turn off the built-in neopixel
    neopixel.write([RGB {r:0, g:0, b:0}].into_iter()).unwrap();

    let mut i = 1;
    loop {
        let j = i - 1;
        head_colors[j % HEAD_STRIP_NPIX] = RGB {r:25, g:25, b:25};
        head_colors[i % HEAD_STRIP_NPIX] = RGB {r:0, g:0, b:0};
        tail_colors[j % TAIL_STRIP_NPIX] = RGB {r:25, g:25, b:25};
        tail_colors[i % TAIL_STRIP_NPIX] = RGB {r:0, g:0, b:0};
        head_strip.write(head_colors.into_iter()).unwrap();
        tail_strip.write(tail_colors.into_iter()).unwrap();

        i += 1;

        delay.delay_ms(50u32);
   }

}


#[allow(dead_code)]
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

fn halt(delay: &mut Delay) -> ! {
    delay.delay_ms(1000u16);
    asm::wfe();
    loop { }
}
