use core::fmt::Write;

use nb::block;

use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::serial::*;
use stm32f1xx_hal::{rcc, rcc::APB1};
use stm32f1xx_hal::pac::USART2;
use stm32f1xx_hal::afio::MAPR;

use stm32f1xx_hal::gpio::gpioa::{PA2, PA3, PA6, PA7};
use stm32f1xx_hal::gpio::gpiob::PB0;
use stm32f1xx_hal::gpio::{Alternate, PushPull, Input, Floating, Output};

use embedded_hal::digital::v2::{OutputPin, InputPin};

pub static BAUDRATE: u32 = 9600;

use crate::motion_control;

pub struct HC05<'a> {
    pub tx: Tx<USART2>,
    rx: Rx<USART2>,
    pub led: PB0<Output<PushPull>>,
    pub pars: &'a mut Pars
}

pub struct Pars {
    pub angle_offset: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32
}

impl Pars {
    pub fn new() -> Pars {
        Pars {
            angle_offset: motion_control::ANGLE_OFFSET,
            kp: motion_control::KP,
            ki: motion_control::KI,
            kd: motion_control::KD
        }
    }
}

impl<'a> HC05<'a> {
    pub fn init(
        usart: USART2,
        txpin: PA2<Alternate<PushPull>>,
        rxpin: PA3<Input<Floating>>,
        mapr: &mut MAPR,
        clocks: rcc::Clocks,
        apb: &mut APB1,
        led: PB0<Output<PushPull>>,
        pars: &'a mut Pars
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

        let mut hc05 = HC05 {
            tx,
            rx,
            led,
            pars
        };


        hc05
    }

    pub fn waiting_data(&mut self) -> u8 {
        block!(self.rx.read()).unwrap()
    }

}