use stm32f1xx_hal::gpio::gpiod;
use stm32f1xx_hal::gpio::{Output, PushPull};
use embedded_hal::digital::v2::OutputPin;

type DIRPIN = gpiod::PD1<Output<PushPull>>;
type STEPPIN = gpiod::PD0<Output<PushPull>>;

pub struct Motor {
    dirpin: DIRPIN,
    steppin: STEPPIN,
}

pub struct State {
    speed: i16,
    time: i16
}

impl Motor{
    pub fn send_pulse(&mut self, state: State) {

    }

}