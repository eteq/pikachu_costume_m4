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
use hal::sercom::{i2c, uart, spi};
use hal::gpio::{Pin, PinId, PushPullOutput};
use hal::rtc;

use heapless;
#[allow(unused_imports)]
use core::fmt::Write;

use fugit::Rate;

use smart_leds::{
    SmartLedsWrite,
    RGB,
    hsv::{hsv2rgb, Hsv},
};
use ws2812_timer_delay;
use ws2812_spi;


const MPU6050_ADDR: u8 = 0x68;
const RTC_FREQ: Rate<u32, 1, 1> = Rate::<u32, 1, 1>::from_raw(32768);

// pkachu ears N=64
// Note: 200 mA @ 50 px @ 25/255 power of each r,g,b @ 4.13 V battery
const TAIL_STRIP_NPIX: usize = 35; 
const HEAD_STRIP_NPIX: usize = 64;

const PIKACHU_YELLOW: RGB<u8> = RGB{r: 60, g:60, b:0};
const RGB_OFF: RGB<u8> = RGB{r: 0, g:0, b:0};

// testing options
const OVERFLOW_CHECK: bool = true; // whether or not to check for FIFO not keeping up.  Can be turned off once all timing is confirmed
const BUILTIN_NPX_FIFO_LEVEL: bool = false; // whether or not to use the built-in neopixel to show the FIFO fullness
const BUILTIN_NPX_STATUS: bool = true; // whether or not to use the built-in neopixel to show status of the jump/spin state
const STRIP_TEST_MODE: bool = false; // If true, the strips will be auto-triggered for testing
const JUMP_DETECTION_INFO_ON: bool = false; // whether or not to print the detection info to the UART
const SPIN_DETECTION_INFO_ON: bool = false; // whether or not to print the detection info to the UART

// parameters to twiddle
const SPIN_TIME_SECS: f32 = 2.5; // tail
const JUMP_TIME_SECS: f32 = 3.0;  // Head
const ACCEL_BUFFER_SIZE: usize = 25; // determines the accel window for jump detection - 100 Hz
const ACCEL_STD_THRESHOLD: f32 = 0.1; // unclear units...?
const ROT_BUFFER_SIZE: usize = 4; // determines the window for spin detection - 100 Hz.  Note the actual sample size is n-1 since its a diff
const ROT_DIFF_THRESHOLD: f32 = 20.0; // deg per sample = 100 deg/s

const ACCEL_STD_THRESHOLD_SQUARED: f32 = ACCEL_STD_THRESHOLD*ACCEL_STD_THRESHOLD;
const ROT_DIFF_THRESHOLD_SQUARED: f32 = ROT_DIFF_THRESHOLD*ROT_DIFF_THRESHOLD;

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
    let mut delay = Delay::new(core.SYST, &mut clocks);
    
    let mut red_led = pd13.into_push_pull_output();
    red_led.set_high().unwrap();

    let gclk0 = clocks.gclk0();

    // set up timers for neopixels
    let timer_clock23 = clocks.tc2_tc3(&gclk0).unwrap();
    let mut timer2 = TimerCounter::tc2_(&timer_clock23, peripherals.TC2, &mut peripherals.MCLK);
    timer2.start(Hertz::MHz(3).into_duration());
    let mut timer3 = TimerCounter::tc3_(&timer_clock23, peripherals.TC3, &mut peripherals.MCLK);
    timer3.start(Hertz::MHz(3).into_duration());
    let timer_clock45 = clocks.tc4_tc5(&gclk0).unwrap();
    let mut timer4 = TimerCounter::tc4_(&timer_clock45, peripherals.TC4, &mut peripherals.MCLK);
    timer4.start(Hertz::MHz(4).into_duration());
    let mut timer5 = TimerCounter::tc5_(&timer_clock45, peripherals.TC5, &mut peripherals.MCLK);
    timer5.start(Hertz::MHz(3).into_duration());

    // set up built-in neopixel
    let neopixel_pin = pneopixel.into_push_pull_output();
    let mut neopixel = ws2812_timer_delay::Ws2812::new(timer4, neopixel_pin);

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
    let mut scratch_string: heapless::String<2560> = heapless::String::new();

    // Check if there was a panic message, if so, send to UART and noop loop
    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        write_to_uart(&mut board_uart_tx, b"\r\n\r\n");
        write_to_uart(&mut board_uart_tx, b"Previous panic message:\r\n");
        write_to_uart(&mut board_uart_tx, msg);
        write_to_uart(&mut board_uart_tx, b"\r\nHalting for panic\r\n\r\n");
        halt(&mut delay);
    }

    // set up the RTC
    peripherals.OSC32KCTRL.rtcctrl.modify(|_, w| w.rtcsel().xosc32k());
    let rtc = rtc::Rtc::count32_mode(peripherals.RTC, RTC_FREQ, &mut peripherals.MCLK).into_count32_mode();
    

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

    let mut tail_strip = ws2812_timer_delay::Ws2812::new(timer3, pins.pb08.into_push_pull_output()); // a2 - tail
    //let mut head_strip = ws2812_timer_delay::Ws2812::new(timer4, pins.pb23.into_push_pull_output());  //mosi - head

    // note sercom 2 and 5 are already taken above for i2c and uart. SErcom 1 is the default SPI, so we use that plus 4

    let pads_spi1 = spi::Pads::<hal::sercom::Sercom1, hal::sercom::UndocIoSet1>::default()
                    .data_out(pins.pb23).sclk(pins.pa17).data_in(pins.pb22);  // the labeled SPI pins
    let spi_clock1 = clocks.sercom1_core(&gclk0).unwrap();
    let spi1 = spi::Config::new(&peripherals.MCLK, peripherals.SERCOM1, pads_spi1, spi_clock1.freq())
                .baud(2.MHz())
                .spi_mode(spi::MODE_0)
                .enable();
    let mut head_strip = ws2812_spi::Ws2812::new(spi1);


    // let pads_spi4 = spi::Pads::<hal::sercom::Sercom4, hal::sercom::IoSet2>::default()
    //                 .data_out(pins.pb08).sclk(pins.pb09).data_in(pins.pb10); // a2 - tail
    // let spi_clock4 = clocks.sercom4_core(&gclk0).unwrap();
    // let spi4 = spi::Config::new(&peripherals.MCLK, peripherals.SERCOM4, pads_spi4, spi_clock4.freq())
    //              .baud(3.MHz())
    //              .spi_mode(spi::MODE_0)
    //              .enable();
    // let mut tail_strip = ws2812_spi::Ws2812::new(spi4);

    let mut head_colors = [PIKACHU_YELLOW ; HEAD_STRIP_NPIX];
    let mut tail_colors = [PIKACHU_YELLOW ; TAIL_STRIP_NPIX];

    head_strip.write(head_colors.into_iter()).unwrap();
    tail_strip.write(tail_colors.into_iter()).unwrap();
    // quick blink to show startup succeeded w/ strips on for 300 ms
    blink_led(50, 3, &mut red_led, &mut delay);

    // turn off the built-in neopixel
    let mut bi_npx_color = RGB_OFF;
    neopixel.write([bi_npx_color].into_iter()).unwrap();

    // turn off the strips
    for i in 0..head_colors.len() {head_colors[i] = RGB_OFF; }
    for i in 0..tail_colors.len() {tail_colors[i] = RGB_OFF; }
    head_strip.write(head_colors.into_iter()).unwrap();
    tail_strip.write(tail_colors.into_iter()).unwrap();

    let mut zaccel_buffer: heapless::Deque<f32, ACCEL_BUFFER_SIZE>= heapless::Deque::new();
    let mut rot_buffer: heapless::Deque<f32, ROT_BUFFER_SIZE>= heapless::Deque::new();

    let mut jump_end: f32 = -(JUMP_TIME_SECS as f32)-1.;
    //let mut jump_finished = false;
    let mut spin_end: f32 = -(SPIN_TIME_SECS as f32)-1.;
    //let mut spin_finished = false;


    // for test node
    let mut jt_test = false;
    let mut st_test = false;

    mpu6050::mpu6050_reset_fifo(&mut i2c, MPU6050_ADDR);
    loop {
        let mut buffers_full = true;
        let fifo_count = mpu6050::mpu6050_get_fifo_count(&mut i2c, MPU6050_ADDR);
        if mpu6050::mpu6050_get_fifo_count(&mut i2c, MPU6050_ADDR) as usize >= mpu6050::DMP_PACKET_SIZE {
            // THIS BRANCH IS FOR CHECKING FOR NEW DATA
            let data = mpu6050::mpu6050_read_fifo(&mut i2c, MPU6050_ADDR);

            if OVERFLOW_CHECK {
                // reset and repeat if overflow occurred
                let mut status_buffer = [0u8; 1];
                i2c.write_read(MPU6050_ADDR, &[0x3a], &mut status_buffer).unwrap();
                if (status_buffer[0] & 0b00010000) != 0 {
                    write_to_uart(&mut board_uart_tx, b"FIFO overflow! Discarding and resetting.\r\n");
                    mpu6050::mpu6050_reset_fifo(&mut i2c, MPU6050_ADDR);
                    continue;
                }
            }

            if BUILTIN_NPX_FIFO_LEVEL {
                // set neopixel to keep track of whether or not FIFO 
                let hsv = Hsv{hue: (((fifo_count as f32 / 1024.) * 170. + 85.) % 255.) as u8, 
                                    sat: 255, val: 60};
                neopixel.write([hsv2rgb(hsv)].into_iter()).unwrap();
            }

            if zaccel_buffer.is_full(){ zaccel_buffer.pop_back(); } else { buffers_full = false; } //discard oldest data
            //let _ = zaccel_buffer.push_front(data.to_z_accel());
            let _ = zaccel_buffer.push_front(data.accel_to_float().0);

            if rot_buffer.is_full(){ rot_buffer.pop_back(); } else { buffers_full = false; } //discard oldest data
            let _ = rot_buffer.push_front(data.to_xy_rotdeg());

            if !buffers_full { continue } // wait for the buffers to fill before doing anything 

            // process data for action detection
            let mut out = [0.0f32; 2];
            let mut now = get_rtc_secs(&rtc);

            if now > jump_end {
                if detect_jump(&zaccel_buffer, &mut out) {
                    jump_end = now + JUMP_TIME_SECS;
                    if !JUMP_DETECTION_INFO_ON { write_to_uart(&mut board_uart_tx, b"Jump detected.\r\n"); }

                    if JUMP_DETECTION_INFO_ON {
                        scratch_string.clear();
                        write!(scratch_string, "Jump detected.     var: {:+.10} > {} , Mean: {:+.8}", out[1], ACCEL_STD_THRESHOLD_SQUARED, out[0]).unwrap();
                        //write!(scratch_string, "  Buffer: {:?}\r\n", zaccel_buffer).unwrap();
                        write!(scratch_string, "  lastval: {:?}\r\n", zaccel_buffer.front().unwrap()).unwrap();
                        write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());
                    }
                } else if JUMP_DETECTION_INFO_ON{
                    scratch_string.clear();
                    write!(scratch_string, "Jump not detected. var: {:+.10} > {}, Mean: {:+.8}", out[1], ACCEL_STD_THRESHOLD_SQUARED, out[0]).unwrap();
                    write!(scratch_string, "  lastval: {:?}\r\n", zaccel_buffer.front().unwrap()).unwrap();
                    write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());
                }
            }

            now = get_rtc_secs(&rtc);
            if  now > spin_end { 
                if detect_spin(&rot_buffer, &mut out) {
                    spin_end = get_rtc_secs(&rtc) + SPIN_TIME_SECS;
                    if !SPIN_DETECTION_INFO_ON { write_to_uart(&mut board_uart_tx, b"Spin detected.\r\n"); }

                    if SPIN_DETECTION_INFO_ON {
                        scratch_string.clear();
                        write!(scratch_string, "Spin detected.      Mean: {:+012.6} > {}", out[0], ROT_DIFF_THRESHOLD).unwrap();
                        write!(scratch_string, "  Buffer: {:?}\r\n", rot_buffer).unwrap();
                        write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());
                    }
                } else if SPIN_DETECTION_INFO_ON {
                    scratch_string.clear();
                    write!(scratch_string, "Spin not detected.  Mean: {:+012.6} > {}", out[0], ROT_DIFF_THRESHOLD).unwrap();
                    write!(scratch_string, "  Buffer: {:?}\r\n", rot_buffer).unwrap();
                    write_to_uart(&mut board_uart_tx, scratch_string.as_bytes());
                }
            }

            if STRIP_TEST_MODE {
                let rs = get_rtc_secs(&rtc);

                if (rs as u32) % 10 == 2 {
                    if !jt_test {
                        jump_end = get_rtc_secs(&rtc) + JUMP_TIME_SECS;
                        write_to_uart(&mut board_uart_tx, b"Jump triggered.\r\n");
                        jt_test = true;
                    }
                } else {
                    jt_test = false;
                }
                if (rs as u32) % 15 == 10 {
                    if !st_test {
                        spin_end = get_rtc_secs(&rtc) + SPIN_TIME_SECS;
                        write_to_uart(&mut board_uart_tx, b"Spin triggered.\r\n");
                        st_test = true;
                    }
                } else {
                    st_test = false;
                }
            }

            
            
        }

        let rtc_secs = get_rtc_secs(&rtc);

        if rtc_secs < jump_end {
            // do the jump lights
            head_update(&mut head_colors, 1. - (jump_end-rtc_secs)/JUMP_TIME_SECS);
            head_strip.write(head_colors.into_iter()).unwrap();

            //jump_finished = false;
            if BUILTIN_NPX_STATUS { bi_npx_color.b = 25; }
        } else {
            for i in 0..head_colors.len() {head_colors[i] = RGB_OFF; }
            head_strip.write(head_colors.into_iter()).unwrap();
            if BUILTIN_NPX_STATUS { bi_npx_color.b = 0; }
        }
        
        if rtc_secs < spin_end {
            // do the spin lights
            tail_update(&mut tail_colors, 1. - (spin_end-rtc_secs)/SPIN_TIME_SECS);
            tail_strip.write(tail_colors.into_iter()).unwrap();

            //spin_finished = false;
            if BUILTIN_NPX_STATUS { bi_npx_color.g = 25; }
        } else {
            for i in 0..tail_colors.len() {tail_colors[i] = RGB_OFF; }
            tail_strip.write(tail_colors.into_iter()).unwrap();
            if BUILTIN_NPX_STATUS { bi_npx_color.g = 0; }
        }     

        if BUILTIN_NPX_STATUS { neopixel.write([bi_npx_color].into_iter()).unwrap(); }
    }
}
fn head_update(colors: &mut [RGB<u8>], frac: f32) {
    // start with first half on  second half off
    if frac < 0.25f32 {
        for i in 0..(colors.len()/2 - 1) {
            colors[i] = PIKACHU_YELLOW;
        }
        for i in (colors.len()/2 - 1)..colors.len() {
            colors[i] = RGB_OFF;
        }   
    }
    // 1/4 of the way though, first half (ish) off, second half on
    else if frac < 0.5f32 {
        for i in 0..(colors.len()/2 -2) {
            colors[i] = RGB_OFF;
        }
        for i in (colors.len()/2 - 2)..colors.len() {
            colors[i] = PIKACHU_YELLOW;
        }   

    }
    // 1/2 of the way through, all on
    else if frac < 0.75f32 {
        for i in 0..colors.len() {
            colors[i] = PIKACHU_YELLOW;
        } 
    // blink 3x for remainder
    } else {
        let stage: i32 = ((frac-0.75)*4.*6.) as i32;
        if stage % 2 == 0 {
            // on
            for i in 0..colors.len() { colors[i] = RGB_OFF;}
        } else {
            // off
            for i in 0..colors.len() { colors[i] = PIKACHU_YELLOW;}
        }
    }

}

fn tail_update(colors: &mut [RGB<u8>], frac: f32) {
    // first half, yellow dot*3 goes up the tail
    // second half, blink 4 times
    if frac < 0.5 {
        let idx = (frac*2.*(colors.len() as f32)) as usize;
        for i in 0..colors.len() { colors[i] = RGB_OFF;}
        colors[idx] = PIKACHU_YELLOW;
        if idx > 1 {
            colors[idx-1] = PIKACHU_YELLOW;
        }
        if idx > 2 {
            colors[idx-2] = PIKACHU_YELLOW;
        }
    } else {
        let stage: i32 = ((frac-0.5)*2.*8.) as i32;
        if stage % 2 == 0 {
            // on
            for i in 0..colors.len() { colors[i] = PIKACHU_YELLOW;}
        } else {
            // off
            for i in 0..colors.len() { colors[i] = RGB_OFF;}
        }

    }
}

fn detect_jump<const N: usize>(zaccel_buffer: &heapless::Deque<f32, N>, 
    info: &mut [f32; 2]) -> bool {
    let mut accum: f32 = 0.0;
    for elem in zaccel_buffer.iter() {
        accum += elem;
    }
    let mean = accum / (N as f32);

    info[0] = mean;
    accum = 0.0;
    for elem in zaccel_buffer.iter() {
        let em = elem - mean;
        accum += em*em;
    }
    let var = accum / (N as f32);
    info[1] = var;

    var > ACCEL_STD_THRESHOLD_SQUARED
}

fn detect_spin<const N: usize>(rot_buffer: &heapless::Deque<f32, N>, 
                                                     info: &mut [f32; 2]) -> bool {
    let mut diff_buffer = [0.0f32 ; N];
    let mut i = 0;
    for elem in rot_buffer.iter(){
        diff_buffer[i] = elem - diff_buffer[i];
        if i >= N-1 { break; }
        diff_buffer[i+1] = *elem;
        i+= 1;
    }
                                                        
    let mut accum: f32 = 0.0;
    for i in 1..N {
        accum += diff_buffer[i];
    }
    let mean = accum / (N as f32);
    info[0] = accum;

    (mean*mean) > ROT_DIFF_THRESHOLD_SQUARED
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

#[inline]
fn get_rtc_secs(rtc: &rtc::Rtc<rtc::Count32Mode>) -> f32 {
    (rtc.count32() as f32)/(RTC_FREQ.to_Hz() as f32)
}