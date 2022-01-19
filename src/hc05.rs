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
    pub v_kp: f32,
    pub v_kd: f32,
    pub b_kp: f32,
    pub b_ki: f32
}

impl Pars {
    pub fn new() -> Pars {
        Pars {
            angle_offset: mpu6050::ANGLE_OFFSET,
            v_kp: motion_control::V_KP,
            v_kd: motion_control::V_KD,
            b_kp: motion_control::B_KP,
            b_ki: motion_control::B_KI
        }
    }
}

pub struct HC05<'a> {
    pub tx: Tx<USART2>,
    pub rx_circbuf: dma::CircBuffer<[u8; 7], RxDma2>,
    mpu6050_data: &'a mpu6050::Data,
    mc_data:&'a motion_control::Data,
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
        mpu6050_data: &'a mpu6050::Data,
        mc_data: &'a motion_control::Data
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
            mpu6050_data,
            mc_data,
            pars
        }
    }

    pub fn send_packets(&mut self) {

        let angle_buf = f32_to_u8(self.mpu6050_data.angle);
        let angle_i_buf = f32_to_u8(self.mc_data.angle_i);
        let v_kp_buf = f32_to_u8(self.pars.v_kp);
        let v_kd_buf = f32_to_u8(self.pars.v_kd);
        let b_kp_buf = f32_to_u8(self.pars.b_kp);
        let b_ki_buf = f32_to_u8(self.pars.b_ki);
        let mut check: u32 = 0;

        for i in 0..4 {
            check += angle_buf[i] as u32;
            check += angle_i_buf[i] as u32;
            check += v_kp_buf[i] as u32;
            check += v_kd_buf[i] as u32;
            check += b_kp_buf[i] as u32;
            check += b_ki_buf[i] as u32;
        }

        let buffer: [u8; 27] = [
            0xA5,
            angle_buf[0], angle_buf[1], angle_buf[2], angle_buf[3],
            angle_i_buf[0], angle_i_buf[1], angle_i_buf[2], angle_i_buf[3],
            v_kp_buf[0], v_kp_buf[1], v_kp_buf[2], v_kp_buf[3],
            v_kd_buf[0], v_kd_buf[1], v_kd_buf[2], v_kd_buf[3],
            b_kp_buf[0], b_kp_buf[1], b_kp_buf[2], b_kp_buf[3],
            b_ki_buf[0], b_ki_buf[1], b_ki_buf[2], b_ki_buf[3],
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
        if let Ok(_) = data_check(&data)  {
            if data[1] == 1 {
                self.pars.angle_offset += 0.002;
            } else if data[1] == 2 {
                self.pars.angle_offset -= 0.002;
            } 
    
            if data[2] == 1 {
                self.pars.v_kp += 0.02;
            } else if data[2] == 2 {
                self.pars.v_kp -= 0.02;
            } else if data[2] == 3 {
                self.pars.v_kd += 0.02;
            } else if data[2] == 4 {
                self.pars.v_kd -= 0.02;
            }
    
            if data[3] == 1 {
                self.pars.b_kp += 5.0;
            } else if data[3] == 2 {
                self.pars.b_kp -= 5.0;
            }
    
            if data[4] == 1 {
                self.pars.b_ki += 2.0;
            } else if data[4] == 2 {
                self.pars.b_ki -= 2.0;
            }
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

pub fn data_check(data: &[u8;7]) -> Result<u8,u8> {
    if (data[0] != 0xA5) || (data[6] != 0x5A) {
        return Err(1);
    }

    let check: u32 = (data[1] as u32) + (data[2] as u32) + (data[3] as u32) + (data[4] as u32);
    if get_the_lowest_byte(check) != data[5] {
        return Err(2);
    }

    Ok(1)
}