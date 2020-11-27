use stm32f1xx_hal::{prelude::*, i2c, i2c::BlockingI2c, pac::{Peripherals, I2C1}, afio, rcc, rcc::APB1};

use stm32f1xx_hal::afio::MAPR;

use stm32f1xx_hal::gpio::gpiob::{PB6, PB7};
use stm32f1xx_hal::gpio::{Alternate, OpenDrain};


pub struct mpu6050 {
    i2c: BlockingI2c<
        I2C1,
        (PB6<Alternate<OpenDrain>>,PB7<Alternate<OpenDrain>>)
    >
}

pub fn init(
    i2c: I2C1,
    mapr: &mut MAPR,
    clocks: rcc::Clocks,
    apb: &mut APB1,
    pb6: PB6<Alternate<OpenDrain>>,
    pb7: PB7<Alternate<OpenDrain>>) -> mpu6050 {
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
    let mut mpu6050 = mpu6050 {
        i2c
    };
    
    mpu6050.write(Regs::POWER_MGMT_1.addr(), 0x01);
    mpu6050.write(Regs::ACCEL_CONFIG.addr(), 0x00);
    mpu6050.write(Regs::POWER_MGMT_2.addr(), 0x00);
    mpu6050.write(Regs::GYRO_CONFIG.addr(), 0x18);

    mpu6050
}

impl mpu6050 {
    pub fn write(&mut self, addr: u8, data: u8) {
        self.i2c.write(Regs::SLAVE_ADDR.addr(), &[addr, data]).ok();
    }

    pub fn read(&mut self, addr: u8, data: &mut [u8]) {
        self.i2c.write_read(Regs::SLAVE_ADDR.addr(),&[addr], data).ok();
    }

}


#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
pub enum Regs {
    /// Slave address of Mpu6050
    SLAVE_ADDR = 0x68,
    /// Internal register to check slave addr
    WHOAMI = 0x75,
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
}

impl Regs {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}