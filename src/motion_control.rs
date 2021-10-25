use crate::{hc05, mpu6050};
use crate::motor;


pub static KP: f32 = 800.0;
pub static KI: f32 = 0.0;
pub static KD: f32 = 50.0;


pub struct MotionCon<'a> {
    mpu6050_data: &'a mpu6050::Data,
    motors: motor::Motors,
    pars: &'a hc05::Pars,
    mpu6050: mpu6050::MPU6050<'a>,
    data: Data
}

struct Data {
    angle_i: f32//角度积分
}

impl Data {
    pub fn new() -> Self {
        Data { angle_i: 0.0 }
    }
}

impl<'a> MotionCon<'a> {
    pub fn init(
        motors: motor::Motors,
        mpu6050_data: &'a mpu6050::Data,
        pars: &'a hc05::Pars,
        mpu6050: mpu6050::MPU6050<'a>
    ) -> Self {

        MotionCon {
            mpu6050_data,
            motors,
            pars,
            mpu6050,
            data: Data::new()
        }
    }

    fn upright_ring(&self) -> i32 {

        let angle = self.mpu6050_data.angle;
        let gyro = self.mpu6050_data.gyro_y;
        let angle_i = self.data.angle_i;

        ((self.pars.kp * angle) + (self.pars.ki * angle_i) - (self.pars.kd * gyro)) as i32
    }

    pub fn adjust_motion(&mut self, target_angle: f32) {
        self.mpu6050.refresh();
        self.data.angle_i += self.mpu6050_data.angle - target_angle;
        let speed = self.upright_ring();

        self.motors.set_speed(speed);
    }

}