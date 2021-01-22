use libm::fabsf;

use crate::mpu6050;
use crate::motor;

static KP: f32 = 18000.0;
static KD: f32 = 400.0;

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
        let gyro = self.data.gyro_y;

        let speed = (0.9 as f32 / (
            (KP * angle - KD * gyro) * 0.000004
        )) as u16;

        *self.state = motor::State::new().set_forward(forward).set_speed(speed);
    }
}