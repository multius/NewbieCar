use crate::{hc05, mpu6050};
use crate::motor;


pub static KP: f32 = 800.0;
pub static KI: f32 = 0.0;
pub static KD: f32 = 50.0;


pub struct MotionCon<'a> {
    data: &'a mpu6050::Data,
    state: &'a StateType,
    motors: motor::Motors,
    pars: &'a hc05::Pars,
    mpu6050: mpu6050::MPU6050<'a>
}

impl<'a> MotionCon<'a> {
    pub fn init(
        motors: motor::Motors,
        data: &'a mpu6050::Data,
        state: &'a StateType,
        pars: &'a hc05::Pars,
        mpu6050: mpu6050::MPU6050<'a>
    ) -> Self {

        MotionCon {
            data,
            motors,
            state,
            pars,
            mpu6050
        }
    }

    fn upright_ring(&self) -> i32 {

        let angle = self.data.angle;
        let gyro = self.data.gyro_y;
        let angle_i = self.data.angle_i;

        ((self.pars.kp * angle) + (self.pars.ki * angle_i) - (self.pars.kd * gyro)) as i32
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
        self.mpu6050.refresh(0.0);

        let speed = self.upright_ring();

        self.motors.set_speed(speed)
    }

    fn forward_adjust(&mut self) {
        self.mpu6050.refresh(-2.0);

        let speed = self.upright_ring();

        self.motors.set_speed(speed)
    }

    fn backward_adjust(&mut self) {
        self.mpu6050.refresh(2.0);

        let speed = self.upright_ring();

        self.motors.set_speed(speed)
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