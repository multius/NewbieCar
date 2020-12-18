use crate::mpu6050;

use mpu6050::MPU6050;

use stm32f1xx_hal::serial::*;
use stm32f1xx_hal::{rcc, rcc::APB2};
use stm32f1xx_hal::pac::USART1;
use stm32f1xx_hal::afio::MAPR;
use stm32f1xx_hal::gpio::gpioa::{PA9, PA10};
use stm32f1xx_hal::gpio::{Alternate, PushPull, Input, Floating};

use embedded_hal::serial::Write;

use nb::block;

use core::iter::IntoIterator;


pub struct PC {
    tx: Tx<USART1>,
}

pub fn init(
    usart: USART1,
    pa9: PA9<Alternate<PushPull>>,
    pa10: PA10<Input<Floating>>,
    mapr: &mut MAPR,
    config: Config,
    clocks: rcc::Clocks,
    apb: &mut APB2,
) -> PC {
    let serial = Serial::usart1(
        usart,
        (pa9, pa10),
        mapr,
        config,
        clocks,
        apb
    );
    let (tx, _) = serial.split();
    PC {
        tx,
    }
}

impl PC {
    pub fn send(&mut self, char: u8) {
        block!(self.tx.write(char)).ok();
    }

    pub fn send_str(&mut self, s: &str) {
        for &i in s.as_bytes().into_iter() {
            self.send(i);
        }
    }

    pub fn send_i16(&mut self, mut num: i16) {
        let mut buffer: [u8; 8] = [0x00; 8];

        let mut negative = false;
        if num < 0 {
            negative = true; 
            num = -num;
        }

        for i in (0..7).rev() {
            let buf = num % 10;

            buffer[i] = (buf as u8) + 0x30;
            num /= 10;
        }

        for i in 0..7 {
            if buffer[i] == 0x30 {
                buffer[i] = 0x20;
            } else {
                if negative == true {
                    buffer[i-1] = 0x2d;
                }
                break;
            }
        }


        let str: &str = unsafe {
            core::intrinsics::transmute::<&[u8], &str>(&buffer)
        };

        self.send_str(str);
    }

    pub fn send_f32(&mut self, mut num: f32) {
        let mut buffer: [u8; 8] = [0x00; 8];

        let mut negative = false;
        if num < 0.0 {
            negative = true; 
            num = -num;
        }
        
        let mut biger_1 = false;
        if num >= 1.0 {
            biger_1 = true;
        }
        
        num *= 100000.0;

        let mut num = num as i32;

        for i in (3..8).rev() {
            let buf = num % 10;
            buffer[i] = (buf as u8) + 0x30;
            num /= 10;
        }

        if negative == true {
            buffer[0] = 0x2d;
        } else {
            buffer[0] = 0x20;
        }
        
        if biger_1 == true {
            buffer[1] = 0x31;
        } else {
            buffer[1] = 0x30;
        }

        buffer[2] = 0x2e;

        let str: &str = unsafe {
            core::intrinsics::transmute::<&[u8], &str>(&buffer)
        };

        self.send_str(str);
    }

    pub fn send_all_of_mpu6050(&mut self, data: mpu6050::Data) {
        self.send_str("TEMP: ");
        self.send_i16(data.temp);
        self.send_str("\n");

        self.send_str("ACCEL_X: ");
        self.send_i16(data.acc_x);
        self.send_str("\t");

        self.send_str("ACCEL_Y: ");
        self.send_i16(data.acc_y);
        self.send_str("\t");

        self.send_str("ACCEL_Z: ");
        self.send_i16(data.acc_z);
        self.send_str("\n");

        self.send_str("GYRO_X: ");
        self.send_i16(data.gyro_x);
        self.send_str("\t");

        self.send_str("GYRO_Y: ");
        self.send_i16(data.gyro_y);
        self.send_str("\t");

        self.send_str("GYRO_Z: ");
        self.send_i16(data.gyro_z);
        self.send_str("\n");
    }
}