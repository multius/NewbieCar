use crate::mpu6050;
use crate::motor;


static KP: f32 = 280.0;
// static KI: f32 = 0.0;//1.30;
// static KD: f32 = 0.00;

pub struct MotionCon<'a> {
    data: &'a mpu6050::Data,
    state: &'a StateType,
    motors: motor::Motors
}

impl<'a> MotionCon<'a> {
    pub fn init(
        motors: motor::Motors,
        data: &'a mpu6050::Data,
        state: &'a StateType
    ) -> Self {

        MotionCon {
            data,
            motors,
            state
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
        let angle = self.data.angle;
        // let gyro = self.data.gyro;
        // let angle_i = self.data.angle_i;

        let speed = (KP * angle) as i32;

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