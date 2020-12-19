use crate::mpu6050;

use stm32f1xx_hal::serial::*;
use stm32f1xx_hal::{rcc, rcc::APB2};
use stm32f1xx_hal::pac::USART1;
use stm32f1xx_hal::afio::MAPR;
use stm32f1xx_hal::gpio::gpioa::{PA9, PA10};
use stm32f1xx_hal::gpio::{Alternate, PushPull, Input, Floating};

use embedded_hal::serial::Write;

use nb::block;

use core::{iter::IntoIterator, writeln};
use core::fmt::Write as write;

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

    pub fn send_all_of_mpu6050(&mut self, data: mpu6050::Data) {
        write!(
            self.tx,
            "\n\nTEMP: {} \nACCEL_X: {}   ACCEL_Y: {}   ACCEL_Z: {}\nGYRO_X: {}   GYRO_Y: {}   GYRO_Z: {}\n",
            data.acc_x,
            data.acc_y,
            data.acc_z,
            data.gyro_x,
            data.gyro_y,
            data.gyro_z
        ).ok();
    }
}