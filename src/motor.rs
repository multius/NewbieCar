use stm32f1xx_hal::gpio::{gpiod, gpiob};
use stm32f1xx_hal::gpio::{Output, PushPull};

use stm32f1xx_hal::timer::CountDownTimer;
use stm32f1xx_hal::pac::TIM4;

use embedded_hal::digital::v2::OutputPin;

pub static UNIT_TIME: u32 = 500; //单元时间常数（单位us）


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
    rig_flag: i16,
    lef_flag: i16,
    pulse: bool,
    led: LEDPIN,
    pub tim: CountDownTimer<TIM4>
}

#[derive(Clone, Copy)]
pub struct State {
    rig_speed: i16,//脉冲占比参数，值越大速度越小
    rig_forward: bool,
    lef_speed: i16,//脉冲占比
    lef_forward: bool
}

impl State {
    pub fn new() -> State {
        State {
            rig_speed: 1,
            rig_forward: true,
            lef_speed: 1,
            lef_forward: true
        }
    }

    pub fn set_speed(mut self, speed: i16) -> Self {
        self.rig_speed = speed;
        self.lef_speed = speed;

        self
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
            rig_flag: 0,
            lef_flag: 0,
            pulse: true,
            led,
            tim
        }
    }

    pub fn send_pulse(&mut self, state: &State) {
        match self.pulse {
            true => {
                self.pulse = false;
                self.rig_step.set_low().ok();
            },
            false => {
                self.pulse = true;
                self.send_rig_pulse(&state);
            }
        }

    }

    fn send_rig_pulse(&mut self, state: &State) {
        match state.rig_forward {
            true => {
                self.rig_dir.set_low().ok();
            },
            false => {
                self.lef_dir.set_high().ok();
            }
        }

        if self.rig_flag >= state.rig_speed {
            self.rig_flag = 1;
            self.rig_step.set_high().ok();
        } else {
            self.rig_flag += 1;
        }

    }

}