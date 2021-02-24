use core::fmt::Write as write;

use nb::block;
use stm32f1xx_hal::serial::*;
use stm32f1xx_hal::{rcc, rcc::APB1};
use stm32f1xx_hal::pac::USART2;
use stm32f1xx_hal::afio::MAPR;

use stm32f1xx_hal::{
    prelude::*,
    serial::{Config, Serial},
};

use stm32f1xx_hal::gpio::gpioa::{PA2, PA3, PA6, PA7};
use stm32f1xx_hal::gpio::{Alternate, PushPull, Input, Floating, Output};

use embedded_hal::digital::v2::OutputPin;

pub struct HC05 {
    tx: Tx<USART2>,
    rx: Rx<USART2>,
    intpin: PA6<Input<Floating>>,
    keypin: PA7<Output<PushPull>>
}

impl HC05 {
    pub fn init(
        usart: USART2,
        txpin: PA2<Alternate<PushPull>>,
        rxpin: PA3<Input<Floating>>,
        intpin: PA6<Input<Floating>>,
        keypin: PA7<Output<PushPull>>,
        mapr: &mut MAPR,
        config: Config,
        clocks: rcc::Clocks,
        apb: &mut APB1,
    ) -> HC05 {
        let serial = Serial::usart2(
            usart,
            (txpin, rxpin),
            mapr,
            config,
            clocks,
            apb
        );
        let (tx, rx) = serial.split();
        let mut hc05 = HC05 {
            
            tx,
            rx,
            intpin,
            keypin
        };

        hc05.keypin.set_low().unwrap();

        hc05
    }

    pub fn waiting_for_data(&mut self) -> u8 {
        block!(self.rx.read()).unwrap()
    }

}