use libm::fabsf;

use crate::mpu6050;
use crate::motor;

static K: f32 = 7000.0;

pub struct UprightCon<'a> {
    data: &'a mpu6050::Data,
    pub state: &'a mut motor::State,
}

impl<'a> UprightCon<'a> {
    pub fn init(
        data: &'a mpu6050::Data,
        state: &'a mut motor::State
    ) -> Self {
        UprightCon {
            data,
            state
        }
    }

    pub fn cal_to_speed(&mut self) {
        let forward: bool;

        if self.data.angle > 0.0 {
            forward = false
        } else {
            forward = true
        }

        let angle = fabsf(self.data.angle);

        let speed = (0.9 as f32 / (K * angle * 0.000064)) as u16;

        *self.state = motor::State::new().set_forward(forward).set_speed(speed);
    }
}