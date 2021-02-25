use crate::mpu6050;

// use nb::block;

use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::serial::*;
use stm32f1xx_hal::{rcc, rcc::APB2};
use stm32f1xx_hal::pac::USART1;
use stm32f1xx_hal::afio::MAPR;
use stm32f1xx_hal::gpio::gpioa::{PA9, PA10};
use stm32f1xx_hal::gpio::gpiob;
use stm32f1xx_hal::gpio::{Alternate, PushPull, Input, Floating, Output};

use embedded_hal::digital::v2::OutputPin;

use core::fmt::Write as write;


pub static BAUDRATE: u32 = 28800;

pub struct PC<'a> {
    tx: Tx<USART1>,
    rx: Rx<USART1>,
    data: &'a mpu6050::Data
}


impl<'a> PC<'a> {
    // pub fn send(&mut self, char: u8) {
    //     block!(self.tx.write(char)).ok();
    // }

    pub fn init(
        usart: USART1,
        txpin: PA9<Alternate<PushPull>>,
        rxpin: PA10<Input<Floating>>,
        mapr: &mut MAPR,
        clocks: rcc::Clocks,
        apb: &mut APB2,
        data: &'static mpu6050::Data
    ) -> PC<'a> {
        let serial = Serial::usart1(
            usart,
            (txpin, rxpin),
            mapr,
            Config::default().baudrate(BAUDRATE.bps()),
            clocks,
            apb
        );
        let (tx, rx) = serial.split();
        PC {
            tx,
            rx,
            data
        }
    }

    pub fn send_all_of_mpu6050(&mut self) {
        write!(
            self.tx,
            "\nACCEL_X: {}   ACCEL_Z: {}\nGYRO_Y: {}\nangle: {}\ngyro: {}\n",
            self.data.acc_x,
            self.data.acc_z,
            self.data.gyro_y,
            self.data.angle,
            self.data.gyro,
        ).ok();
    }

    pub fn send_char(&mut self, c: u8) {
        self.tx.write(c).ok();
    }
}