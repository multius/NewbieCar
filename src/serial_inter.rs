use crate::mpu6050;

use stm32f1xx_hal::serial::*;
use stm32f1xx_hal::{rcc, rcc::APB2};
use stm32f1xx_hal::pac::USART1;
use stm32f1xx_hal::afio::MAPR;

use stm32f1xx_hal::gpio::gpioa::{PA9, PA10};
use stm32f1xx_hal::gpio::gpiob;
use stm32f1xx_hal::gpio::{Alternate, PushPull, Input, Floating, Output};

use stm32f1xx_hal::timer::CountDownTimer;
use stm32f1xx_hal::pac::TIM3;

use embedded_hal::digital::v2::OutputPin;

// use embedded_hal::serial::Write;

use core::fmt::Write as write;

type LEDPIN = gpiob::PB0<Output<PushPull>>;

pub struct PC {
    tx: Tx<USART1>,
    led: LEDPIN,
    pub tim: CountDownTimer<TIM3>
}



impl PC {
    // pub fn send(&mut self, char: u8) {
    //     block!(self.tx.write(char)).ok();
    // }

    pub fn init(
        usart: USART1,
        tx: PA9<Alternate<PushPull>>,
        rx: PA10<Input<Floating>>,
        mapr: &mut MAPR,
        config: Config,
        clocks: rcc::Clocks,
        apb: &mut APB2,
        led: LEDPIN,
        tim: CountDownTimer<TIM3>
    ) -> PC {
        let serial = Serial::usart1(
            usart,
            (tx, rx),
            mapr,
            config,
            clocks,
            apb
        );
        let (tx, _) = serial.split();
        PC {
            tx,
            led,
            tim
        }
    }

    pub fn send_all_of_mpu6050(&mut self, data: &mpu6050::Data) {
        self.led.set_low().ok();
        write!(
            self.tx,
            "\nACCEL_X: {}   ACCEL_Z: {}\nGYRO_X: {}\nangle: {}du\n",
            data.acc_x,
            data.acc_z,
            data.gyro_x,
            data.angle
        ).ok();
        self.led.set_high().ok();
    }
}