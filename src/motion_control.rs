use libm::fabsf;

use embedded_hal::digital::v2::OutputPin;

use crate::mpu6050;
use crate::motor;


static KP: f32 = 90.0;
// static KI: f32 = 0.0;//1.30;
// static KD: f32 = 0.00;

pub struct MotionCon<'a> {
    data: &'a mpu6050::Data,
    state: &'a StateType,
    motor: motor::Motor
}

impl<'a> MotionCon<'a> {
    pub fn init(
        motor: motor::Motor,
        data: &'a mpu6050::Data,
        state: &'a StateType
    ) -> Self {

        MotionCon {
            data,
            motor,
            state
        }
    }

    pub fn adjust_speed(&mut self) {
        match self.state {
            StateType::Balance => { self.balance_adjust() }
            StateType::Forward => { self.forward_adjust() }
            StateType::Backward => { self.backward_adjust() }
            StateType::TurnLeft => { self.turn_left_adjust() }
            StateType::TurnRight => { self.turn_right_adjust() }
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

        // if speed >= 490 {
        //     speed = 490
        // }
        // if speed <= 1 {
        //     speed = 1
        // }

        self.motor.set_speed(speed)
    }

    fn forward_adjust(&mut self) {
    }

    fn backward_adjust(&mut self) {
    }

    fn turn_left_adjust(&mut self) {
    }

    fn turn_right_adjust(&mut self) {
    }
}

pub enum StateType {
    Balance,
    Forward,
    Backward,
    TurnLeft,
    TurnRight
}

impl StateType {
    pub fn new() -> Self {
        StateType::Balance
    }
}