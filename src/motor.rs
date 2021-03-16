use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::pwm::{Channel, Pwm, C1, C2};
use stm32f1xx_hal::timer::Tim2NoRemap;
use stm32f1xx_hal::pac::TIM2;
use stm32f1xx_hal::gpio::gpioa::{PA0, PA1};
use stm32f1xx_hal::gpio::gpiod::{PD1, PD15};
use stm32f1xx_hal::gpio::{Alternate, PushPull, Output};

use embedded_hal::digital::v2::OutputPin;

static MAX_SPEED: u32 = 9000;
static MAX_ACC: i32 = 100;


pub struct Motors {
    pwm: Pwm<TIM2, Tim2NoRemap, (C1, C2), (
        PA0<Alternate<PushPull>>,
        PA1<Alternate<PushPull>>
    )>,
    dirpins: (PD1<Output<PushPull>>, PD15<Output<PushPull>>),
    speed: i32
}

impl Motors {
    pub fn init(
        mut pwm: Pwm<TIM2, Tim2NoRemap, (C1, C2), (
            PA0<Alternate<PushPull>>,
            PA1<Alternate<PushPull>>
        )>,
        dirpins: (PD1<Output<PushPull>>, PD15<Output<PushPull>>)
    ) -> Self {

        pwm.set_period(1.hz());
        pwm.set_duty(Channel::C1, pwm.get_max_duty()/100);
        pwm.set_duty(Channel::C2, pwm.get_max_duty()/100);
        pwm.enable(Channel::C1);
        pwm.enable(Channel::C2);

        Motors {
            pwm,
            dirpins,
            speed: 0
        }
    }

    fn set_dir(&mut self, forward: bool) {
        if forward {
            self.dirpins.0.set_high().ok();
            self.dirpins.1.set_low().ok();
        } else {
            self.dirpins.0.set_low().ok();
            self.dirpins.1.set_high().ok();
        }
    }

    pub fn set_speed(&mut self, mut speed: i32) {
        if (speed - self.speed).abs() >= MAX_ACC {
            if speed > self.speed {
                speed = self.speed + MAX_ACC
            } else {
                speed = self.speed - MAX_ACC
            }
        }

        if speed > 0 {
            self.set_dir(true)
        } else {
            self.set_dir(false)
        }

        self.speed = speed;
        let speed = speed.abs() as u32;

        if speed == 0 {
            self.pwm.set_period(1.hz())
        } else if speed >= MAX_SPEED {
            self.pwm.set_period(MAX_SPEED.hz())
        } else {
            self.pwm.set_period(speed.hz())
        }
    }
}