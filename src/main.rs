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

const MPU6050_ADDR: u8 = 0x68;
const FLASH_BLOCK_SIZE: usize = 256;  // bytes


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
    let pqspi_sck = pins.pb10;
    let pqspi_cs = pins.pb11;
    let pqspi_io0 = pins.pa08;
    let pqspi_io1 = pins.pa09;
    let pqspi_io2 = pins.pa10;
    let pqspi_io3 = pins.pa11;

    let mut delay = Delay::new(core.SYST, &mut clocks);
    let mut red_led = pd13.into_push_pull_output();
    red_led.set_high().unwrap();

    // set up timer for neopixel
    let gclk0 = clocks.gclk0();
    let timer_clock = clocks.tc2_tc3(&gclk0).unwrap();
    let mut timer = TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK);
    timer.start(Hertz::MHz(3).into_duration());

    // set up neopixel
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
        //pin_alias!(pins.uart_rx),
        //pin_alias!(pins.uart_tx),
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

    // set up MPU6050
    let (sda, scl) = (psda, pscl);
    let sercom2_clock = &clocks.sercom2_core(&gclk0).unwrap();
    let i2c_pads = i2c::Pads::new(sda, scl);
    let i2c_sercom = periph_alias!(peripherals.i2c_sercom);
    let mut i2c = i2c::Config::new(&peripherals.MCLK, i2c_sercom, i2c_pads, sercom2_clock.freq())
        .baud(100.kHz())
        .enable();

    mpu6050::mpu6050_setup(&mut i2c, &mut delay, MPU6050_ADDR);
    
    // setup flash
    // https://github.com/atsamd-rs/atsamd/blob/master/hal/src/thumbv7em/qspi.rs and  https://github.com/atsamd-rs/atsamd/blob/master/boards/wio_terminal/examples/qspi.rs are useful here
    // as perhaps is https://cdn-shop.adafruit.com/product-files/4763/4763_GD25Q16CTIGR.pdf
    let mut flash = qspi::Qspi::new(
        &mut peripherals.MCLK,
        peripherals.QSPI,
        pqspi_sck,
        pqspi_cs,
        pqspi_io0,
        pqspi_io1,
        pqspi_io2,
        pqspi_io3,
    );

    flash_wait_ready(&mut flash);
    flash.run_command(qspi::Command::EnableReset).unwrap();
    flash.run_command(qspi::Command::Reset).unwrap();
    delay.delay_ms(15u8);

    // 60 MHz I think? 120/(3-1)
    flash.set_clk_divider(3);

    // enable QSPI
    flash.write_command(qspi::Command::WriteStatus, &[0x00, 0x02]).unwrap();

    flash_wait_ready(&mut flash);
    flash.run_command(qspi::Command::WriteEnable).unwrap();
    write_to_uart(&mut board_uart_tx, b"erasing chip, please wait... ");
    flash.erase_command(qspi::Command::EraseChip, 0x0).unwrap();
    flash_wait_ready(&mut flash);
    write_to_uart(&mut board_uart_tx, b"Done erasing!\r\n");

    // Neopixel turns magenta after setup
    neopixel.write([RGB {r:0, g:20, b:20}].into_iter()).unwrap();

    let mut write_buf = [0u8; FLASH_BLOCK_SIZE];
    let mut buffer_idx = 0usize;
    mpu6050::mpu6050_reset_fifo(&mut i2c, MPU6050_ADDR);
   {
        while mpu6050::mpu6050_get_fifo_count(&mut i2c, MPU6050_ADDR) < 28 {
            delay.delay_ms(1u8);
        }

        let data = mpu6050::mpu6050_read_fifo(&mut i2c, MPU6050_ADDR);
        buffer_idx += data.to_byte_array(&mut write_buf, buffer_idx);

    

    flash_wait_ready(&mut flash);
    flash.run_command(qspi::Command::WriteEnable).unwrap();
    flash.write_memory(0, &write_buf);

    let mut read_buf = [0u8; FLASH_BLOCK_SIZE];
    flash_wait_ready(&mut flash);
    flash.read_memory(0, &mut read_buf);
    scratch_string.clear();
    core::write!(&mut scratch_string, "post-write read value: {:?}\r\n", &read_buf[0..30]).unwrap();
    write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());

   }


    let mut i = 0usize;
    loop {
        neopixel.write([hsv2rgb(Hsv { hue: (i % 256) as u8, sat: 255, val: 30 })].into_iter()).unwrap();
        i +=1;
        delay.delay_ms(10u8);
    } 
//     mpu6050::mpu6050_reset_fifo(&mut i2c, MPU6050_ADDR);
//     let mut j = 0;
//     loop {
//         let mut sbuffer = [0u8; 1];
//         i2c.write_read(MPU6050_ADDR, &[0x3a],   &mut sbuffer).unwrap();
//         let mut out = *b"status: 00000000\r\n";
//         for i in 0..8 {
//             if (sbuffer[0] & (1 << i)) != 0 {
//                 out[out.len()-3-i] = b'1';
//             }
//         }
//         write_to_uart(&mut board_uart_tx, &out);
//         scratch_string.clear();
//         core::write!(&mut scratch_string, "s:{}\r\n", out[0]).unwrap();
//         write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());

//         if (sbuffer[0] & 0b00010000) != 0 {
//             neopixel.write([RGB {r:20, g:0, b:0}].into_iter()).unwrap();
//         } else {
//             neopixel.write([RGB {r:0, g:20, b:0}].into_iter()).unwrap();
//         }

//         i2c.write_read(MPU6050_ADDR, &[0x3a],   &mut sbuffer).unwrap();
//         let mut out = *b"after: 00000000\r\n";
//         for i in 0..8 {
//             if (sbuffer[0] & (1 << i)) != 0 {
//                 out[out.len()-3-i] = b'1';
//             }
//         }
//         write_to_uart(&mut board_uart_tx, &out);

//         let fifocount = mpu6050::mpu6050_get_fifo_count(&mut i2c, MPU6050_ADDR);

//         scratch_string.clear();
//         let delayms = j* 5u32;
//         core::write!(&mut scratch_string, "count was {}, delaying {} ms\r\n", fifocount, delayms).unwrap();
//         write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());

//         mpu6050::mpu6050_reset_fifo(&mut i2c, MPU6050_ADDR);
//         delay.delay_ms(delayms);
//         j += 1;

//     }
//     loop {
//         mpu6050::mpu6050_reset_fifo(&mut i2c, MPU6050_ADDR);
//         let result = mpu6050::mpu6050_read_fifo(&mut i2c, MPU6050_ADDR);
//         scratch_string.clear();
//         core::write!(&mut scratch_string, "qx:{}, qy:{}, qz:{}, qw:{}\r\n", result.qx, result.qy, result.qz, result.qw).unwrap();
//         write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());

//         scratch_string.clear();
//         core::write!(&mut scratch_string, "gx:{}, gy:{}, gz:{}\r\n", result.gyro_x, result.gyro_y, result.gyro_z).unwrap();
//         core::write!(&mut scratch_string, "ax:{}, ay:{}, az:{}\r\n", result.accel_x, result.accel_y, result.accel_z).unwrap();
//         write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());

//         write_to_uart(&mut board_uart_tx, b"\r\n");

//         delay.delay_ms(500u16);
//     }
}


fn flash_wait_ready(flash: &mut qspi::Qspi<qspi::OneShot>) {
    let mut out1 = [0u8; 1];
    let mut out2 = [0u8; 1];

    flash.read_command(qspi::Command::ReadStatus, &mut out1);
    flash.read_command(qspi::Command::ReadStatus2, &mut out2);
    while (out1[0] & 1u8) == 1 || (out1[0] & 0b10000000u8) == 1  {
        flash.read_command(qspi::Command::ReadStatus, &mut out1);
        flash.read_command(qspi::Command::ReadStatus2, &mut out2);
    }
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