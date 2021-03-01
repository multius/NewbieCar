use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::{i2c, i2c::BlockingI2c, pac::I2C1};
use stm32f1xx_hal::{rcc, rcc::APB1};
use stm32f1xx_hal::afio::MAPR;
use stm32f1xx_hal::gpio::gpiob;
use stm32f1xx_hal::gpio::gpiob::{PB6, PB7};
use stm32f1xx_hal::gpio::{Alternate, OpenDrain, Output, PushPull};

use libm::atan2f;

use embedded_hal::digital::v2::OutputPin;

type LEDPIN = gpiob::PB5<Output<PushPull>>;

// static X_GYRO_OFFSET: i16 = 78;
static Y_GYRO_OFFSET: i32 = -127;
// static Z_GYRO_OFFSET: i16 = -44;
static X_ACC_OFFSET: i32 = -900;
// static Y_ACC_OFFSET: i16 = -30;
static Z_ACC_OFFSET: i32 = -955;

pub static UNIT_TIME: u32 = 50;//ms
static UT_S: f32 = 0.05;

pub struct MPU6050<'a> {
    i2c: BlockingI2c<
        I2C1,
        (PB6<Alternate<OpenDrain>>,PB7<Alternate<OpenDrain>>)
    >,
    led: LEDPIN,
    data: &'a mut Data
}

#[derive(Clone, Copy)]
pub struct Data {
    pub acc_x: i32,
    pub acc_z: i32,
    pub gyro_y: f32,
    pub gyro: f32,
    pub angle: f32,
}


impl Data {
    pub fn new() -> Data{
        Data {
            acc_x: 0,
            acc_z: 0,
            gyro_y: 0.0,
            gyro: 0.0,
            angle: 0.0,
        }
    }
}


impl<'a> MPU6050<'a> {
    pub fn init(
        i2c: I2C1,
        mapr: &mut MAPR,
        clocks: rcc::Clocks,
        apb: &mut APB1,
        pb6: PB6<Alternate<OpenDrain>>,
        pb7: PB7<Alternate<OpenDrain>>,
        led: LEDPIN,
        data: &'a mut Data
    ) -> Self {
        let i2c = BlockingI2c::i2c1(
            i2c,
            (pb6, pb7),
            mapr,
            i2c::Mode::fast(400.khz(), i2c::DutyCycle::Ratio2to1),
            clocks,
            apb,
            1000,
            1,
            1000,
            1000
        );

        let mut mpu6050 = MPU6050 {
            i2c,
            led,
            data
        };

        mpu6050.write(Regs::POWER_MGMT_1.addr(), 0x00);
        mpu6050.write(Regs::POWER_MGMT_2.addr(), 0x00);
        mpu6050.write(Regs::SMPLRT_DIV.addr(), 0x07);
        mpu6050.write(Regs::CONFIG.addr(), 0x06);
        mpu6050.write(Regs::GYRO_CONFIG.addr(), 0x00);
        mpu6050.write(Regs::ACCEL_CONFIG.addr(), 0x00);

        mpu6050
    }

    pub fn write(&mut self, addr: u8, data: u8) {
        self.i2c.write(
            Regs::SLAVE_ADDR.addr(), 
            &[addr, data]
        ).ok();
    }

    pub fn read(&mut self, addr: u8) -> u8 {
        let mut buffer: [u8;1] = [0x0];
        self.i2c.write_read(
            Regs::SLAVE_ADDR.addr(),
            &[addr],
            &mut buffer
        ).ok();
        buffer[0]
    }

    pub fn get_data(&mut self, addr: u8) -> i32 {
        let data_h = self.read(addr);
        let data_l = self.read(addr + 1);

        (((data_h as i16) << 8) | data_l as i16) as i32
    }

    // pub fn get_temp(&mut self) -> i16 {
    //     self.get_data(Regs::TEMP_OUT_H.addr()) / 340 + 36
    // }

    pub fn get_accel_x(&mut self) -> i32 {
        self.get_data(Regs::ACC_REGX_H.addr()) + X_ACC_OFFSET// as f32  / 16384 as f32
    }

    // pub fn get_accel_y(&mut self) -> i16 {
    //     self.get_data(Regs::ACC_REGY_H.addr()) + Y_ACC_OFFSET// as f32  / 16384 as f32
    // }

    pub fn get_accel_z(&mut self) -> i32 {
        self.get_data(Regs::ACC_REGZ_H.addr()) + Z_ACC_OFFSET// as f32  / 16384 as f32
    }

    // pub fn get_gyro_x(&mut self) -> i16 {
    //     self.get_data(Regs::GYRO_REGX_H.addr()) + X_GYRO_OFFSET
    // }

    pub fn get_gyro_y(&mut self) -> f32 {
        let gyro_i32 = self.get_data(Regs::GYRO_REGY_H.addr()) + Y_GYRO_OFFSET;
        (gyro_i32 as f32 / 65536.0) * 500.0
    }

    // pub fn get_gyro_z(&mut self) -> i16 {
    //     self.get_data(Regs::GYRO_REGZ_H.addr()) + Z_GYRO_OFFSET
    // }

    pub fn cal_angle(&self, acc_x: i32, acc_z: i32, gyro_y: f32) -> f32 {
        let angle_m = atan2f(
            acc_x as f32,
            acc_z as f32
        ) * (180.0 / 3.1415926);
        let gyro_m = -gyro_y;

        0.10 * angle_m + 0.90 * (self.data.angle + gyro_m * UT_S)
    }

    pub fn cal_gyro(&self, angle: f32, gyro_y: f32) -> f32{
        let angle_d = self.data.angle;

        ((angle - angle_d) / UT_S) * 0.9 + gyro_y * -0.1
    }

    pub fn refresh(&mut self) {
        self.led.set_low().ok();

        let acc_x = self.get_accel_x();
        let acc_z = self.get_accel_z();
        let gyro_y = self.get_gyro_y();
        let angle =  self.cal_angle(acc_x, acc_z, gyro_y);
        let gyro = self.cal_gyro(angle, gyro_y);

        let data = Data {
            acc_x,
            acc_z,
            gyro_y,
            angle,
            gyro,
        };
        self.led.set_high().ok();

        *self.data = data;
    }
}


#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
pub enum Regs {
    /// Slave address of Mpu6050
    SLAVE_ADDR = 0x68,
    /// Internal register to check slave addr
    //WHOAMI = 0x75,
    /// High Bytle Register Gyro x orientation
    // GYRO_REGX_H = 0x43,
    /// High Bytle Register Gyro y orientation
    GYRO_REGY_H = 0x45,
    // /// High Bytle Register Gyro z orientation
    // GYRO_REGZ_H = 0x47,
    // /// High Byte Register Calc roll
    ACC_REGX_H = 0x3b,
    /// High Byte Register Calc pitch
    // ACC_REGY_H = 0x3d,
    /// High Byte Register Calc yaw
    ACC_REGZ_H = 0x3f,
    /// High Byte Register Temperature
    // TEMP_OUT_H = 0x41,
    /// Register to control chip waking from sleep, enabling sensors, default: sleep
    POWER_MGMT_1 = 0x6b,
    /// Internal clock
    POWER_MGMT_2 = 0x6c,
    /// Accelerometer config register
    ACCEL_CONFIG = 0x1c,
    /// gyro config register
    GYRO_CONFIG = 0x1b,
    //陀螺仪采样率，典型值： 0x07(125HZ)
    SMPLRT_DIV = 0x19,

    CONFIG = 0x1a,
}

impl Regs {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}