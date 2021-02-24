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

type LEDPIN = gpiob::PB0<Output<PushPull>>;

pub static BAUDRATE: u32 = 28800;

pub struct PC<'a> {
    tx: Tx<USART1>,
    rx: Rx<USART1>,
    led: LEDPIN,
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
        config: Config,
        clocks: rcc::Clocks,
        apb: &mut APB2,
        led: LEDPIN,
        data: &'static mpu6050::Data
    ) -> PC<'a> {
        let serial = Serial::usart1(
            usart,
            (txpin, rxpin),
            mapr,
            config,
            clocks,
            apb
        );
        let (tx, rx) = serial.split();
        PC {
            tx,
            rx,
            led,
            data
        }
    }

    pub fn send_all_of_mpu6050(&mut self) {
        self.led.set_low().ok();
        write!(
            self.tx,
            "\nACCEL_X: {}   ACCEL_Z: {}\nGYRO_Y: {}\nangle: {}\ngyro: {}\n",
            self.data.acc_x,
            self.data.acc_z,
            self.data.gyro_y,
            self.data.angle,
            self.data.gyro,
        ).ok();
        self.led.set_high().ok();
    }

    pub fn send_char(&mut self, c: u8) {
        if c == b'A' {
            self.led.set_high().ok();
        } else {
            self.led.set_low().ok();
        }
        self.tx.write(c).ok();
    }
}