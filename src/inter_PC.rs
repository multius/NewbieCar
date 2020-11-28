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
    rx: Rx<USART1>
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
    let (tx, rx) = serial.split();
    PC {
        tx,
        rx
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
}