use stm32f1xx_hal::gpio::{gpiod, gpiob};
use stm32f1xx_hal::gpio::{Output, PushPull};
use embedded_hal::digital::v2::OutputPin;

type DIRPIN = gpiod::PD1<Output<PushPull>>;
type STEPPIN = gpiod::PD0<Output<PushPull>>;
type LEDPIN = gpiob::PB1<Output<PushPull>>;

pub struct Motor {
    dirpin: DIRPIN,
    steppin: STEPPIN,
    ledpin: LEDPIN,
    pulse: bool
}

pub struct State {
    speed: i16,
    counter: i16
}

pub fn init(dirpin: DIRPIN, steppin: STEPPIN, ledpin: LEDPIN) -> Motor {
    Motor {
        dirpin,
        steppin,
        ledpin,
        pulse: true
    }
}

impl Motor{
    pub fn send_pulse(&mut self) {
        self.dirpin.set_high().ok();
        if self.pulse == true {
            self.steppin.set_high().ok();
            self.ledpin.set_high().ok();
            self.pulse = false;
        } else {
            self.steppin.set_low().ok();
            self.ledpin.set_low().ok();
            self.pulse = true;
        }
    }
}