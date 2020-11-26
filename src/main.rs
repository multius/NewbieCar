#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::{pac, prelude::*, timer::Timer, i2c, i2c::BlockingI2c, serial::{Serial, Config}};

mod registers;

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOC peripheral
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpiob.pb5.into_push_pull_output(&mut gpiob.crl);
    let mut PB6 = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let mut PB7 = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let mut pin_TX = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let mut pin_RX = gpioa.pa10;
    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).start_count_down(1.hz());


    let mut i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (PB6, PB7),
        &mut afio.mapr,
        i2c::Mode::standard(100.hz()),
        clocks,
        &mut rcc.apb1,
        100000,
        100,
        100000,
        100000
    );

    let serial = Serial::usart1(
        dp.USART1,
        (pin_TX,pin_RX),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2
    );

    let (mut tx, mut rx) = serial.split();

    block!(tx.write(0x77)).ok();

    let mut buffer:[u8;1] = [0xff];
    
    i2c.write(0x68, &[0x6b, 0x00]).ok();
    i2c.write(0x68, &[0x1c, 0x00]).ok();
    i2c.write(0x68, &[0x6c, 0x00]).ok();
    


    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        block!(timer.wait()).unwrap();
        led.set_high().unwrap();

        i2c.write_read(0x68, &[0x43], &mut buffer).ok();

        let mut data= 0xff;

        for i in buffer.iter() {
            //data <<= 1;
            data = *i;
        }

        block!(tx.write(data)).ok();

        i2c.write_read(0x68, &[0x44], &mut buffer).ok();

        let mut data= 0xff;

        for i in buffer.iter() {
            //data <<= 1;
            data = *i;
        }

        block!(tx.write(data)).ok();

        block!(timer.wait()).unwrap();
        led.set_low().unwrap();
    }
}