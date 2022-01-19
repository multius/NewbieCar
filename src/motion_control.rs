use crate::{hc05, mpu6050};
use crate::motor;


pub static B_KP: f32 = 1200.0;
pub static B_KI: f32 = 100.0;
// pub static B_KD: f32 = 0.0;

pub static V_KP: f32 = 1.0;
pub static V_KD: f32 = 0.0;

// static FILTER_PAR: f32 = 0.9;

pub struct MotionCon<'a> {
    mpu6050_data: &'a mpu6050::Data,
    motors: motor::Motors,
    pars: &'a hc05::Pars,
    mpu6050: mpu6050::MPU6050<'a>,
    data: &'a mut Data
}

pub struct Data {
    pub angle_i: f32,
    // angle_i_buf: [f32;60],
    pub velocity: i32
}

impl Data {
    pub fn new() -> Self {
        Data {
            angle_i: 0.0,
            // angle_i_buf: [0.0; 60],
            velocity: 0,
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
        let angle_i = self.data.angle_i as f32;

        ((self.pars.b_kp * angle) + (self.pars.b_ki * angle_i)) as i32
    }

    fn velocity_feedback(&mut self, v: i32) -> i32 {
        
        let last_v = self.data.velocity;
        self.data.velocity = v;
        let d_v = v - last_v;

        let v = v as f32;
        let d_v = d_v as f32;

        (V_KP * v - self.pars.v_kd * d_v) as i32
    }

    pub fn adjust_motion(&mut self, _target_angle: f32) {
        self.mpu6050.refresh();
        self.update_angle_i(self.mpu6050_data.angle);
        let v = self.velocity_feedback(self.balance_feedback());

        self.motors.set_velocity(v);
    }


    fn update_angle_i(&mut self, new_angle: f32) {
        // for i in 0..59 {
        //     self.data.angle_i_buf[i] = self.data.angle_i_buf[i+1]
        // }
        // self.data.angle_i_buf[59] = new_angle;

        // let mut new_angle_i= 0.0;
        // for i in self.data.angle_i_buf {
        //     new_angle_i += i
        // }

        let new_angle_i = self.data.angle_i + new_angle;

        if new_angle_i <= 600.0 && new_angle_i >= -600.0 {
            self.data.angle_i = new_angle_i
        } else if new_angle_i > 0.0 {
            self.data.angle_i = 600.0
        } else if new_angle_i < 0.0 {
            self.data.angle_i = -600.0
        }

    }

}