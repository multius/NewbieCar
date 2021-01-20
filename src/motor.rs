use stm32f1xx_hal::gpio::gpiod;
use stm32f1xx_hal::gpio::{Output, PushPull};

use stm32f1xx_hal::timer::CountDownTimer;
use stm32f1xx_hal::pac::TIM4;

use embedded_hal::digital::v2::OutputPin;

pub static UNIT_TIME: u32 = 8000; //单元时间常数（单位us）


#[allow(non_camel_case_types)]
type RIG_DIRPIN = gpiod::PD1<Output<PushPull>>;
#[allow(non_camel_case_types)]
type RIG_STEPPIN = gpiod::PD0<Output<PushPull>>;
#[allow(non_camel_case_types)]
type LEF_DIRPIN = gpiod::PD15<Output<PushPull>>;
#[allow(non_camel_case_types)]
type LEF_STEPPIN = gpiod::PD14<Output<PushPull>>;

pub struct Motor<'a> {
    rig_dir: RIG_DIRPIN,
    rig_step: RIG_STEPPIN,
    lef_dir: LEF_DIRPIN,
    lef_step: LEF_STEPPIN,
    rig_flag: u16,
    lef_flag: u16,
    pulse: bool,
    state: &'a State,
    pub tim: CountDownTimer<TIM4>
}

#[derive(Clone, Copy)]
pub struct State {
    rig_speed: u16,//脉冲占比参数，值越大速度越小
    rig_forward: bool,
    lef_speed: u16,//脉冲占比
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

    pub fn set_speed(mut self, speed: u16) -> Self {
        self.rig_speed = speed;
        self.lef_speed = speed;

        self
    }

    pub fn set_forward(mut self, forward: bool) -> Self {
        self.rig_forward = forward;
        self.lef_forward = forward;

        self
    }
}


impl<'a> Motor<'a>{
    pub fn init(
        rig_dir: RIG_DIRPIN,
        rig_step: RIG_STEPPIN,
        lef_dir: LEF_DIRPIN,
        lef_step: LEF_STEPPIN,
        state: &'a State,
        tim: CountDownTimer<TIM4>
    ) -> Motor<'a> {
        Motor {
            rig_dir,
            rig_step,
            lef_dir,
            lef_step,
            rig_flag: 0,
            lef_flag: 0,
            pulse: true,
            state,
            tim
        }
    }

    pub fn send_pulse(&mut self) {
        match self.pulse {
            true => {
                self.pulse = false;
                self.rig_step.set_low().ok();
                self.lef_step.set_low().ok();
            },
            false => {
                self.pulse = true;
                self.send_rig_pulse();
                self.send_lef_pulse();
            }
        }

    }

    fn send_rig_pulse(&mut self) {
        match self.state.rig_forward {
            true => {
                self.rig_dir.set_low().ok();
            },
            false => {
                self.rig_dir.set_high().ok();
            }
        }

        if self.rig_flag >= self.state.rig_speed {
            self.rig_flag = 1;
            self.rig_step.set_high().ok();
        } else {
            self.rig_flag += 1;
        }

    }

    fn send_lef_pulse(&mut self) {
        match self.state.lef_forward {
            true => {
                self.lef_dir.set_high().ok();
            },
            false => {
                self.lef_dir.set_low().ok();
            }
        }

        if self.lef_flag >= self.state.lef_speed {
            self.lef_flag = 1;
            self.lef_step.set_high().ok();
        } else {
            self.lef_flag += 1;
        }

    }

}