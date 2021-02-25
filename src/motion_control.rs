use libm::fabsf;

use embedded_hal::digital::v2::OutputPin;

use crate::mpu6050;
use crate::motor;


static KP: f32 = 90.0;
// static KI: f32 = 0.0;//1.30;
// static KD: f32 = 0.00;

pub struct MotionCon<'a> {
    data: &'a mpu6050::Data,
    state: &'a State,
    motor: motor::Motor
}

impl<'a> MotionCon<'a> {
    pub fn init(
        motor: motor::Motor,
        data: &'a mpu6050::Data,
        state: &'a State
    ) -> Self {

        MotionCon {
            data,
            motor,
            state
        }
    }

    pub fn adjust_speed(&mut self) {
        match self.state {
            Balance => {
                self.balance_adjust()
            }
            Forward => {}
            Fuck => {}
        }
    }

    fn balance_adjust(&mut self) {
        if self.data.angle > 0.0 {
            self.motor.set_direction(true)
        } else {
            self.motor.set_direction(false)
        }

        let angle = fabsf(self.data.angle);
        // let gyro = self.data.gyro;
        // let angle_i = self.data.angle_i;

        let mut speed = (KP * angle) as u32;

        if speed >= 490 {
            speed = 490
        }
        if speed <= 1 {
            speed = 1
        }

        self.motor.set_speed(speed)
    }

}

pub enum State {
    Balance,
    Forward,
    Backward
}

impl State {
    pub fn new() -> Self {
        State::Balance
    }
}