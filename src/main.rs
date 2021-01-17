#![no_std]
#![no_main]

use panic_halt as _;

// use nb::block;

use cortex_m_rt::entry;
use cortex_m::interrupt::Mutex;

use stm32f1xx_hal::{pac, prelude::*, serial};
use stm32f1xx_hal::timer::{Event, Timer};
use stm32f1xx_hal::pac::{interrupt, Interrupt};

// use embedded_hal::digital::v2::OutputPin;

use core::cell::RefCell;

mod mpu6050;
mod serial_inter;
mod motor;

use mpu6050::{MPU6050, Data};
use serial_inter::PC;
use motor::{Motor, State};


macro_rules! send_to_global {
    ($val: expr, $addr: expr) => {
        cortex_m::interrupt::free(|cs| *$addr.borrow(cs).borrow_mut() = Some($val))
    };
}

macro_rules! get_from_global {
    ($local:expr, $addr:expr) => {
        $local.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| {
                $addr.borrow(cs).replace(None).unwrap()
            })
        })
    };
}

macro_rules! refresh_with {
    ($addr: expr) => {
        cortex_m::interrupt::free(|cs| {
            $addr.borrow(cs).replace_with(|&mut old| old).unwrap()
        })
    };
}


static G_MPU6050: Mutex<RefCell<Option<mpu6050::MPU6050>>> = Mutex::new(RefCell::new(None));

static G_PC: Mutex<RefCell<Option<serial_inter::PC>>> = Mutex::new(RefCell::new(None));
static G_DATA: Mutex<RefCell<Option<mpu6050::Data>>> = Mutex::new(RefCell::new(None));

static G_MOTOR: Mutex<RefCell<Option<motor::Motor>>> = Mutex::new(RefCell::new(None));
static G_STATE: Mutex<RefCell<Option<motor::State>>> = Mutex::new(RefCell::new(None));

#[interrupt]
unsafe fn TIM2() {
    static mut MPU6050: Option<mpu6050::MPU6050> = None;
    let mpu6050 = get_from_global!(MPU6050, G_MPU6050);

    let data = mpu6050.refresh(); //额外定义一个变量，避免refresh操作锁定Mutex
    send_to_global!(data, G_DATA);

    mpu6050.tim.wait().ok();
}


#[interrupt]
unsafe fn TIM3() {
    static mut PC: Option<serial_inter::PC> = None;
    let pc = get_from_global!(PC, G_PC);

    let data = refresh_with!(&G_DATA);
    pc.send_all_of_mpu6050(data);

    pc.tim.wait().ok();
}


#[interrupt]
unsafe fn TIM4() { //步进电机中断
    static mut MOTOR: Option<motor::Motor> = None;
    let motor = get_from_global!(MOTOR, G_MOTOR);

    let state = refresh_with!(&G_STATE);
    motor.send_pulse(state);

    motor.tim.wait().ok();
}



#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    let clocks = rcc
        .cfgr
        .sysclk(72.mhz())
        .pclk1(8.mhz())
        .freeze(&mut flash.acr);

    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);

    //-------------------------------------定时器初始化
    let mut tim2 = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1)
        .start_count_down(25.ms());

    let mut tim3 = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1)
        .start_count_down(1000.ms());

    let mut tim4 = Timer::tim4(dp.TIM4, &clocks, &mut rcc.apb1)
        .start_count_down(20000.hz());

    tim2.listen(Event::Update);
    tim3.listen(Event::Update);
    tim4.listen(Event::Update);

    //------------------------------------外置模块初始化
    let mpu6050 = MPU6050::init(
        dp.I2C1,
        &mut afio.mapr,
        clocks,
        &mut rcc.apb1,
        gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl),
        gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl),
        gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
        tim2
    );

    let pc = PC::init(
        dp.USART1,
        gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
        gpioa.pa10,
        &mut afio.mapr,
        serial::Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2,
        gpiob.pb0.into_push_pull_output(&mut gpiob.crl),
        tim3
    );

    let motor = Motor::init(
        gpiod.pd1.into_push_pull_output(&mut gpiod.crl),
        gpiod.pd0.into_push_pull_output(&mut gpiod.crl),
        gpiob.pb1.into_push_pull_output(&mut gpiob.crl),
        tim4
    );

    //-----------------------------------初始化全局变量
    send_to_global!(mpu6050, &G_MPU6050);

    send_to_global!(pc, &G_PC);
    send_to_global!(Data::new(), &G_DATA);

    send_to_global!(motor, &G_MOTOR);
    send_to_global!(State::new(), &G_STATE);

    //-----------------------------------启用定时器
    unsafe {
        cp.NVIC.set_priority(Interrupt::TIM2, 0x10);
        cp.NVIC.set_priority(Interrupt::TIM3, 0xf0);
        cp.NVIC.set_priority(Interrupt::TIM4, 0x00);

        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM4);
    }


    loop {
    }
}