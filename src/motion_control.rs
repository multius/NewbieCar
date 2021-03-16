use crate::{hc05, mpu6050};
use crate::motor;


pub static KP: f32 = 700.0;
// pub static KI: f32 = 0.0;
pub static KD: f32 = 50.0;
pub static ANGLE_OFFSET: f32 = 0.0;


pub struct MotionCon<'a> {
    data: &'a mpu6050::Data,
    state: &'a StateType,
    motors: motor::Motors,
    pars: &'a hc05::Pars
}

impl<'a> MotionCon<'a> {
    pub fn init(
        motors: motor::Motors,
        data: &'a mpu6050::Data,
        state: &'a StateType,
        pars: &'a hc05::Pars
    ) -> Self {

        MotionCon {
            data,
            motors,
            state,
            pars
        }
    }

    pub fn adjust_motion(&mut self) {
        match self.state {
            StateType::Balance => { self.balance_adjust() }
            StateType::Forward => { self.forward_adjust() }
            StateType::Backward => { self.backward_adjust() }
            StateType::TurnLeft => { self.turn_left_adjust() }
            StateType::TurnRight => { self.turn_right_adjust() }
        }
    }

    fn balance_adjust(&mut self) {
        let angle = self.data.angle + self.pars.angle_offset;
        let gyro = self.data.gyro_y;
        // let angle_i = self.data.angle_i;

        let speed = ((self.pars.kp * angle) - (self.pars.kd * gyro)) as i32;

        self.motors.set_speed(speed)
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