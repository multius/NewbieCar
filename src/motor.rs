use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::pwm::{Channel, Pwm, C1, C2};
use stm32f1xx_hal::timer::Tim2NoRemap;
use stm32f1xx_hal::pac::TIM2;
use stm32f1xx_hal::gpio::gpioa::{PA0, PA1};
use stm32f1xx_hal::gpio::gpiod::{PD1, PD15};
use stm32f1xx_hal::gpio::{Alternate, PushPull, Output};

use embedded_hal::digital::v2::OutputPin;

pub struct Motor {
    pwm: Pwm<TIM2, Tim2NoRemap, (C1, C2), (
        PA0<Alternate<PushPull>>,
        PA1<Alternate<PushPull>>
    )>,
    dirpins: (PD1<Output<PushPull>>, PD15<Output<PushPull>>),
}


impl Motor{
    pub fn init(
        mut pwm: Pwm<TIM2, Tim2NoRemap, (C1, C2), (
            PA0<Alternate<PushPull>>,
            PA1<Alternate<PushPull>>
        )>,
        dirpins: (PD1<Output<PushPull>>, PD15<Output<PushPull>>),
    ) -> Motor {

        pwm.set_period(1.hz());
        pwm.set_duty(Channel::C1, pwm.get_max_duty()/2);
        pwm.set_duty(Channel::C2, pwm.get_max_duty()/2);
        pwm.enable(Channel::C1);
        pwm.enable(Channel::C2);

        Motor {
            pwm,
            dirpins
        }
    }

    pub fn set_direction(&mut self, forward: bool) {
        if forward {
            self.dirpins.0.set_high().ok();
            self.dirpins.1.set_low().ok();
        } else {
            self.dirpins.0.set_low().ok();
            self.dirpins.1.set_high().ok();
        }
    }

    pub fn set_speed(&mut self, speed: u32) {
        self.pwm.set_period(speed.hz())
    }
}