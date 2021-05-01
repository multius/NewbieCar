use core::{fmt::Write, panic};

// use cortex_m::{itm::write_fmt, peripheral::mpu};
use nb::block;

use stm32f1xx_hal::{dma::Transfer, prelude::*};
use stm32f1xx_hal::serial::*;
use stm32f1xx_hal::{rcc, rcc::APB1};
use stm32f1xx_hal::pac::USART2;
use stm32f1xx_hal::afio::MAPR;
use stm32f1xx_hal::dma::dma1::Channels;
use stm32f1xx_hal::dma;

use stm32f1xx_hal::gpio::gpioa::{PA2, PA3};
use stm32f1xx_hal::gpio::{Alternate, PushPull, Input, Floating};

// use embedded_hal::digital::v2::{OutputPin, InputPin};
use cortex_m::singleton;

pub static BAUDRATE: u32 = 19200;

use crate::motion_control;
use crate::mpu6050;


pub struct Pars {
    pub angle_offset: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32
}

impl Pars {
    pub fn new() -> Pars {
        Pars {
            angle_offset: mpu6050::ANGLE_OFFSET,
            kp: motion_control::KP,
            ki: motion_control::KI,
            kd: motion_control::KD
        }
    }
}

pub struct HC05<'a> {
    pub tx: Tx<USART2>,
    pub rx_circbuf: dma::CircBuffer<[u8; 7], RxDma2>,
    data: &'a mpu6050::Data,
    pub pars: &'a mut Pars
}



impl<'a> HC05<'a> {
    pub fn init(
        usart: USART2,
        txpin: PA2<Alternate<PushPull>>,
        rxpin: PA3<Input<Floating>>,
        mapr: &mut MAPR,
        clocks: rcc::Clocks,
        apb: &mut APB1,
        channels: Channels,
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
        let rx = rx.with_dma(channels.6);

        let buf= singleton!(: [[u8; 7]; 2] = [[0; 7]; 2]).unwrap();
        let rx_circbuf = rx.circ_read(buf);

        HC05 {
            tx,
            rx_circbuf,
            data,
            pars
        }
    }

    pub fn send_packets(&mut self) {

        let angle_buf = f32_to_u8(self.data.angle);
        let kp_buf = f32_to_u8(self.pars.kp);
        let ki_buf = f32_to_u8(self.pars.ki);
        let kd_buf = f32_to_u8(self.pars.kd);
        let mut check: u32 = 0;

        for i in 0..4 {
            check += angle_buf[i] as u32;
            check += kp_buf[i] as u32;
            check += ki_buf[i] as u32;
            check += kd_buf[i] as u32;
        }

        let buffer: [u8; 19] = [
            0xA5,
            angle_buf[0], angle_buf[1], angle_buf[2], angle_buf[3],
            kp_buf[0], kp_buf[1], kp_buf[2], kp_buf[3],
            ki_buf[0], ki_buf[1], ki_buf[2], ki_buf[3],
            kd_buf[0], kd_buf[1], kd_buf[2], kd_buf[3],
            get_the_lowest_byte(check),
            0x5A
        ];

        let str = unsafe {
            core::intrinsics::transmute::<&[u8], &str>(&buffer)
        };

        self.tx.write_str(str).ok();
    }

    // pub fn fuck(&mut self) {
    //     let buf= singleton!(: [u8; 8] = [0; 8]).unwrap();
    //     self.rx.read(buf);
    // }

    pub fn packets_analyse(&mut self) {
        let data = self.rx_circbuf.peek(|half, _| *half).unwrap();
        data_check(&data).unwrap();
        
        if data[1] == 1 {
            self.pars.angle_offset += 0.001;
        } else if data[1] == 2 {
            self.pars.angle_offset -= 0.001;
        }

        if data[2] == 1 {
            self.pars.kp += 10.0;
        } else if data[2] == 2 {
            self.pars.kp -= 10.0;
        }

        if data[3] == 1 {
            self.pars.ki += 1.0;
        } else if data[3] == 2 {
            self.pars.ki -= 1.0;
        }

        if data[4] == 1 {
            self.pars.kd += 5.0;
        } else if data[4] == 2 {
            self.pars.kd -= 5.0;
        }
    }
}

pub fn get_half(result: Result<dma::Half, dma::Error>) -> dma::Half{
    match result {
        Ok(h) => h,
        Err(_) => dma::Half::Second
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

pub fn data_check(data: &[u8; 7]) -> Result<u8,u8> {
    if (data[0] != 0xA5) || (data[6] != 0x5A) {
        return Err(1);
    }

    let check: u32 = (data[1] as u32) + (data[2] as u32) + (data[3] as u32) + (data[4] as u32);
    if get_the_lowest_byte(check) != data[5] {
        return Err(2);
    }

    Ok(1)
}