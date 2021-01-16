use stm32f1xx_hal::gpio::{gpiod, gpiob};
use stm32f1xx_hal::gpio::{Output, PushPull};

use stm32f1xx_hal::timer::CountDownTimer;
use stm32f1xx_hal::pac::TIM4;

use embedded_hal::digital::v2::OutputPin;

type DIRPIN = gpiod::PD1<Output<PushPull>>;
type STEPPIN = gpiod::PD0<Output<PushPull>>;
type LEDPIN = gpiob::PB1<Output<PushPull>>;

pub struct Motor {
    dir: DIRPIN,
    step: STEPPIN,
    led: LEDPIN,
    pulse: bool,
    flag: i16,
    pub tim: CountDownTimer<TIM4>
}

#[derive(Clone, Copy)]
pub struct State {
    speed: i16,//脉冲占比,越大速度越少
    forward: bool
}

impl State {
    pub fn new() -> State {
        State {
            speed: 0,
            forward: true
        }
    }
}



impl Motor{
    pub fn init(dir: DIRPIN, step: STEPPIN, led: LEDPIN, tim: CountDownTimer<TIM4>) -> Motor {
        Motor {
            dir,
            step,
            led,
            pulse: true,
            flag: 0,
            tim
        }
    }

    pub fn send_pulse(&mut self, state: State) {
        self.dir.set_high().ok();
        if self.pulse == true {
            self.step.set_low().ok();
            self.led.set_low().ok();
            self.pulse = false;
        } else {
            if self.flag >= state.speed {
                self.step.set_high().ok();
                self.led.set_high().ok();
                self.pulse = true;
                self.flag = 0;
            } else {
                self.pulse = true;
                self.flag += 1;
            }
        }
    }
}