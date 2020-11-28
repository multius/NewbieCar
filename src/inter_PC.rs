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

use crate::mpu6050;


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
    fn send(&mut self, char: u8) {
        block!(self.tx.write(char)).ok();
    }

    pub fn send_str(&mut self, s: &str) {
        for &i in s.as_bytes().into_iter() {
            self.send(i);
        }
    }

    pub fn send_u16(&mut self, mut num: u16) {
        let mut buffer: [u8; 8] = [0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37];
        
        let str = &"        ";
        unsafe {
            let str = &mut (**str);
            //let mut str = &mut (*(str[0..8])) as &mut str;

        
            let iter = str.as_bytes_mut().iter_mut();
        }
        // unsafe {
        //     for i in (&mut (*str)).as_bytes_mut().iter_mut() {
        //     }
        // }

        //for i in 15..0 {
        //    let buf = num % 10;
        //    buffer[i] = (buf as u8) + 0x30;
        //    num /= 10;
        //}

        // let str: &str = unsafe {
        //     core::intrinsics::transmute::<[u8; 8], &str>(buffer)
        // };

        self.send_str(str);
    }

    pub fn send_all_of_mpu6050(&mut self, mpu6050: &mut MPU6050) {
        self.send_str("TEMP: ");
        self.send_u16(
            mpu6050.get_data(mpu6050::Regs::TEMP_OUT_H.addr())
        );
        self.send_str("\n");
    }
}