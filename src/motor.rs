use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::pwm::{Channel, Pwm, C1, C2};
use stm32f1xx_hal::timer::Tim2NoRemap;
use stm32f1xx_hal::pac::TIM2;
use stm32f1xx_hal::gpio::gpioa::{PA0, PA1};
use stm32f1xx_hal::gpio::gpiod::{PD1, PD15};
use stm32f1xx_hal::gpio::{Alternate, PushPull, Output};

// use embedded_hal::digital::v2::OutputPin;

static MAX_SPEED: u32 = 8000;
static MIN_SPEED: u32 = 15;
static MAX_ACC: i32 = 1000;


pub struct Motors {
    pwm: Pwm<TIM2, Tim2NoRemap, (C1, C2), (
        PA0<Alternate<PushPull>>,
        PA1<Alternate<PushPull>>
    )>,
    dirpins: (PD1<Output<PushPull>>, PD15<Output<PushPull>>),
    velocity: i32
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
            velocity: 0
        }
    }

    fn set_dir(&mut self, forward: bool) {
        if forward {
            self.dirpins.0.set_high();
            self.dirpins.1.set_low();
        } else {
            self.dirpins.0.set_low();
            self.dirpins.1.set_high();
        }
    }

    pub fn set_velocity(&mut self, mut velocity: i32) {
        if (velocity - self.velocity).abs() >= MAX_ACC {
            if velocity > self.velocity {
                velocity = self.velocity + MAX_ACC
            } else {
                velocity = self.velocity - MAX_ACC
            }
        }

        if velocity > 0 {
            self.set_dir(true)
        } else {
            self.set_dir(false)
        }

        self.velocity = velocity;
        let velocity = velocity.abs() as u32;

        if velocity < MIN_SPEED {
            self.pwm.set_period(MIN_SPEED.hz())
        } else if velocity >= MAX_SPEED {
            self.pwm.set_period(MAX_SPEED.hz())
        } else {
            self.pwm.set_period(velocity.hz())
        }
    }
}