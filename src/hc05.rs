use core::fmt::Write;

// use cortex_m::{itm::write_fmt, peripheral::mpu};
use nb::block;

use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::serial::*;
use stm32f1xx_hal::{rcc, rcc::APB1};
use stm32f1xx_hal::pac::USART2;
use stm32f1xx_hal::afio::MAPR;

use stm32f1xx_hal::gpio::gpioa::{PA2, PA3};
use stm32f1xx_hal::gpio::{Alternate, PushPull, Input, Floating};

// use embedded_hal::digital::v2::{OutputPin, InputPin};

pub static BAUDRATE: u32 = 19200;

use crate::motion_control;
use crate::mpu6050;


pub struct Pars {
    pub angle_offset: f32,
    pub kp: f32,
    pub kd: f32
}

impl Pars {
    pub fn new() -> Pars {
        Pars {
            angle_offset: motion_control::ANGLE_OFFSET,
            kp: motion_control::KP,
            kd: motion_control::KD
        }
    }
}


pub struct HC05<'a> {
    pub tx: Tx<USART2>,
    rx: Rx<USART2>,
    pub pars: &'a mut Pars,
    data: &'a mpu6050::Data
}



impl<'a> HC05<'a> {
    pub fn init(
        usart: USART2,
        txpin: PA2<Alternate<PushPull>>,
        rxpin: PA3<Input<Floating>>,
        mapr: &mut MAPR,
        clocks: rcc::Clocks,
        apb: &mut APB1,
        pars: &'a mut Pars,
        data: &'a mpu6050::Data
    ) -> Self {
        
        let serial = Serial::usart2(
            usart,
            (txpin, rxpin),
            mapr,
            Config::default().baudrate(BAUDRATE.bps()),
            clocks,
            apb
        );
        let (tx, rx) = serial.split();
    
        HC05 {
            tx,
            rx,
            pars,
            data
        }
    }

    pub fn send_packets(&mut self) {

        let data_buf = f32_to_u8(self.data.angle);
        let mut check: u32 = 0;

        for i in 0..4 {
            self.tx.write(data_buf[i]).ok();
            check += data_buf[i] as u32;
        }

        self.tx.write(get_the_lowest_byte(check)).ok();

        self.tx.write(0x5A).ok();

        let buffer: [u8; 7] = [
            0xA5,
            data_buf[0],
            data_buf[1],
            data_buf[2],
            data_buf[3],
            get_the_lowest_byte(check),
            0x5A
        ];

        let str = unsafe {
            core::intrinsics::transmute::<&[u8], &str>(&buffer)
        };

        self.tx.write_str(str).ok();
    }

    pub fn waiting_data(&mut self) -> u8 {
        block!(self.rx.read()).unwrap()
    }
}



fn f32_to_u8(num: f32) -> [u8;4] {
    let mut u8_buf: [u8; 4] = [0x0; 4];
    let f32_ptr: *const f32 = &num as *const f32;
    let u8_ptr: *const u8 = f32_ptr as *const u8;

    for i in 0..4 {
        u8_buf[i as usize] = unsafe {
            *u8_ptr.offset(i) as u8
        }
    }

    u8_buf
}

fn get_the_lowest_byte(num: u32) -> u8 {
    let u32_ptr: *const u32 = &num as *const u32;
    let u8_ptr: *const u8 = u32_ptr as *const u8;

    unsafe {
        *u8_ptr.offset(0) as u8
    }
}