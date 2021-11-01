use crate::{hc05, mpu6050};
use crate::motor;


pub static B_KP: f32 = 900.0;
// pub static B_KI: f32 = 100.0;
pub static B_KD: f32 = 0.0;

pub static V_KP: f32 = 1.0;
pub static V_KI: f32 = V_KP / 200.0;

pub struct MotionCon<'a> {
    mpu6050_data: &'a mpu6050::Data,
    motors: motor::Motors,
    pars: &'a hc05::Pars,
    mpu6050: mpu6050::MPU6050<'a>,
    data: &'a mut Data
}

pub struct Data {
    velocity_i: i32,
    velocity_i_buf: [i32;20]
}

impl Data {
    pub fn new() -> Self {
        Data {
            velocity_i: 0,
            velocity_i_buf: [0; 20]
        }
    }
}

impl<'a> MotionCon<'a> {
    pub fn init(
        motors: motor::Motors,
        mpu6050_data: &'a mpu6050::Data,
        pars: &'a hc05::Pars,
        mpu6050: mpu6050::MPU6050<'a>,
        data: &'a mut Data
    ) -> Self {

        MotionCon {
            mpu6050_data,
            motors,
            pars,
            mpu6050,
            data
        }
    }

    fn balance_feedback(&self) -> i32 {

        let angle = self.mpu6050_data.angle;
        let gyro = self.mpu6050_data.gyro;

        ((B_KP * angle) + (B_KD * gyro)) as i32
    }

    fn velocity_feedback(&self, v: i32) -> i32 {
        let v = v as f32;
        (self.pars.v_kp * v + (self.pars.v_kp / 200.0) * v) as i32
    }

    pub fn adjust_motion(&mut self, _target_angle: f32) {
        self.mpu6050.refresh();
        let v = self.velocity_feedback(self.balance_feedback());
        self.update_velocity_i(v);

        self.motors.set_velocity(v);
    }


    fn update_velocity_i(&mut self, new_velocity: i32) {
        for i in 0..19 {
            self.data.velocity_i_buf[i] = self.data.velocity_i_buf[i+1]
        }
        self.data.velocity_i_buf[19] = new_velocity;

        let mut new_velocity_i= 0;
        for i in self.data.velocity_i_buf {
            new_velocity_i += i
        }

        self.data.velocity_i = new_velocity_i
    }

}