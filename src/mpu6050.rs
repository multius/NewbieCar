use stm32f1xx_hal::{prelude::*, i2c, i2c::BlockingI2c, pac::I2C1, rcc, rcc::APB1};

use stm32f1xx_hal::afio::MAPR;

use stm32f1xx_hal::gpio::gpiob::{PB6, PB7};
use stm32f1xx_hal::gpio::{Alternate, OpenDrain};

static X_GYRO_OFFSET: i16 = 0;
static Y_GYRO_OFFSET: i16 = 0;
static Z_GYRO_OFFSET: i16 = 0;


pub struct MPU6050 {
    i2c: BlockingI2c<
        I2C1,
        (PB6<Alternate<OpenDrain>>,PB7<Alternate<OpenDrain>>)
    >
}

pub struct Data {
    pub temp: i16,
    pub acc_x: i16,
    pub acc_y: i16,
    pub acc_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
}

pub fn init(
    i2c: I2C1,
    mapr: &mut MAPR,
    clocks: rcc::Clocks,
    apb: &mut APB1,
    pb6: PB6<Alternate<OpenDrain>>,
    pb7: PB7<Alternate<OpenDrain>>
) -> (MPU6050, Data) {
    let i2c = BlockingI2c::i2c1(
        i2c,
        (pb6, pb7),
        mapr,
        i2c::Mode::standard(100.hz()),
        clocks,
        apb,
        100000,
        100,
        100000,
        100000
    );
    let mut mpu6050 = MPU6050 {
        i2c
    };

    mpu6050.write(Regs::POWER_MGMT_1.addr(), 0x00);
    mpu6050.write(Regs::POWER_MGMT_2.addr(), 0x00);
    mpu6050.write(Regs::SMPLRT_DIV.addr(), 0x07);
    mpu6050.write(Regs::CONFIG.addr(), 0x00);
    mpu6050.write(Regs::GYRO_CONFIG.addr(), 0x00);
    mpu6050.write(Regs::ACCEL_CONFIG.addr(), 0x00);

    (
        mpu6050,
        Data {
            temp: 0,
            acc_x: 0,
            acc_y: 0,
            acc_z: 0,
            gyro_x: 0,
            gyro_y: 0,
            gyro_z: 0,
        }
    )
}

impl MPU6050 {
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

    pub fn get_data(&mut self, addr: u8) -> i16 {
        let data_h = self.read(addr);
        let data_l = self.read(addr + 1);

        ((data_h as i16) << 8) | data_l as i16
    }

    pub fn get_temp(&mut self) -> i16 {
        self.get_data(Regs::TEMP_OUT_H.addr()) / 340 + 36
    }

    pub fn get_accel_x(&mut self) -> i16 {
        self.get_data(Regs::ACC_REGX_H.addr())// as f32  / 16384 as f32
    }

    pub fn get_accel_y(&mut self) -> i16 {
        self.get_data(Regs::ACC_REGY_H.addr())// as f32  / 16384 as f32
    }

    pub fn get_accel_z(&mut self) -> i16 {
        self.get_data(Regs::ACC_REGZ_H.addr())// as f32  / 16384 as f32
    }

    pub fn get_gyro_x(&mut self) -> i16 {
        self.get_data(Regs::GYRO_REGX_H.addr()) + X_GYRO_OFFSET
    }

    pub fn get_gyro_y(&mut self) -> i16 {
        self.get_data(Regs::GYRO_REGY_H.addr()) + Y_GYRO_OFFSET
    }

    pub fn get_gyro_z(&mut self) -> i16 {
        self.get_data(Regs::GYRO_REGZ_H.addr()) + Z_GYRO_OFFSET
    }

    // pub fn get_angle(&mut self) -> i16 {

    // }

    pub fn refresh(&mut self) -> Data {
        Data {
            temp: self.get_temp(),
            acc_x: self.get_accel_x(),
            acc_y: self.get_accel_y(),
            acc_z: self.get_accel_z(),
            gyro_x: self.get_gyro_x(),
            gyro_y: self.get_gyro_y(),
            gyro_z: self.get_gyro_z()
        }
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
    GYRO_REGX_H = 0x43,
    /// High Bytle Register Gyro y orientation
    GYRO_REGY_H = 0x45,
    /// High Bytle Register Gyro z orientation
    GYRO_REGZ_H = 0x47,
    /// High Byte Register Calc roll
    ACC_REGX_H = 0x3b,
    /// High Byte Register Calc pitch
    ACC_REGY_H = 0x3d,
    /// High Byte Register Calc yaw
    ACC_REGZ_H = 0x3f,
    /// High Byte Register Temperature
    TEMP_OUT_H = 0x41,
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