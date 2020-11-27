
use stm32f1xx_hal::gpio::gpiob::PB5;
use stm32f1xx_hal::gpio::{Output, PushPull};

struct blink {
    pin: PB5<Output<PushPull>>,
    state: bool
}

impl blink {
    fn flash(&mut self) {
        if self.state == true {
            
        }
    }
}