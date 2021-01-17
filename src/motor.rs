use stm32f1xx_hal::gpio::{gpiod, gpiob};
use stm32f1xx_hal::gpio::{Output, PushPull};

use stm32f1xx_hal::timer::CountDownTimer;
use stm32f1xx_hal::pac::TIM4;

use embedded_hal::digital::v2::OutputPin;


#[allow(non_camel_case_types)]
type RIG_DIRPIN = gpiod::PD1<Output<PushPull>>;
#[allow(non_camel_case_types)]
type RIG_STEPPIN = gpiod::PD0<Output<PushPull>>;
#[allow(non_camel_case_types)]
type LEF_DIRPIN = gpiod::PD15<Output<PushPull>>;
#[allow(non_camel_case_types)]
type LEF_STEPPIN = gpiod::PD14<Output<PushPull>>;
type LEDPIN = gpiob::PB1<Output<PushPull>>;

pub struct Motor {
    rig_dir: RIG_DIRPIN,
    rig_step: RIG_STEPPIN,
    lef_dir: LEF_DIRPIN,
    lef_step: LEF_STEPPIN,
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
    pub fn init(
        rig_dir: RIG_DIRPIN,
        rig_step: RIG_STEPPIN,
        lef_dir: LEF_DIRPIN,
        lef_step: LEF_STEPPIN,
        led: LEDPIN,
        tim: CountDownTimer<TIM4>
    ) -> Motor {
        Motor {
            rig_dir,
            rig_step,
            lef_dir,
            lef_step,
            led,
            pulse: true,
            flag: 0,
            tim
        }
    }

    pub fn send_pulse(&mut self, state: State) {
        self.rig_dir.set_high().ok();
        self.lef_dir.set_high().ok();

        if self.pulse == true {

            self.rig_step.set_low().ok();
            self.lef_step.set_low().ok();
            self.led.set_low().ok();
            self.pulse = false;

        } else {
            if self.flag >= state.speed {
                
                self.rig_step.set_high().ok();
                self.lef_step.set_high().ok();
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