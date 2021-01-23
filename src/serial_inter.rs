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

pub struct PC<'a> {
    tx: Tx<USART1>,
    led: LEDPIN,
    data: &'a mpu6050::Data,
    pub tim: CountDownTimer<TIM3>
}



impl<'a> PC<'a> {
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
        data: &'static mpu6050::Data,
        tim: CountDownTimer<TIM3>
    ) -> PC<'a> {
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
            data,
            tim
        }
    }

    pub fn send_all_of_mpu6050(&mut self) {
        self.led.set_low().ok();
        write!(
            self.tx,
            "\nACCEL_X: {}   ACCEL_Z: {}\nGYRO_Y: {}\nangle: {}\ngyro: {}\nangle_i: {}\n",
            self.data.acc_x,
            self.data.acc_z,
            self.data.gyro_y,
            self.data.angle,
            self.data.gyro,
            self.data.angle_i
        ).ok();
        self.led.set_high().ok();
    }
}