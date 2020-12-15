use stm32f1xx_hal::gpio::gpiob::PB5;
use stm32f1xx_hal::gpio::{Output, PushPull};
use embedded_hal::digital::v2::OutputPin;

pub struct blink {
    pin: PB5<Output<PushPull>>,
    state: bool
}

pub fn init(pin: PB5<Output<PushPull>>) -> blink {
    blink {
        pin,
        state: false
    }
}

impl blink {
    pub fn flash(&mut self) {
        if self.state == false {
            self.pin.set_high().unwrap();
            self.state = true;
        } else {
            self.pin.set_low().unwrap();
            self.state = false;
        }
    }
}