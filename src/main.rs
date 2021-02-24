#![no_std]
#![no_main]

use panic_halt as _;
// use nb::block;

//----------------------------引入cortex-m库
use cortex_m_rt::entry;
use cortex_m::interrupt::Mutex;

//----------------------------引入hal库
use stm32f1xx_hal::{pac, prelude::*, serial, timer::Tim2NoRemap};
use stm32f1xx_hal::timer::{Event, Timer};
use stm32f1xx_hal::pac::{interrupt, Interrupt};
use stm32f1xx_hal::delay::Delay;

// use embedded_hal::digital::v2::OutputPin;

//-------------------------引入核心库（core）
use core::cell::RefCell;
use core::mem::MaybeUninit;

//-------------------------引入子模块
mod mpu6050;
mod serial_inter;
mod upright;
mod hc05;

use mpu6050::MPU6050;
use serial_inter::PC;
use upright::UprightCon;

//----------------------------------------全局变量控制宏
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

macro_rules! get_ptr {
    ($addr:expr) => {
        & *$addr.as_ptr()
    };
}

macro_rules! get_mut_ptr {
    ($addr:expr) => {
        &mut *$addr.as_mut_ptr()
    };
}

//------------------------------------全局变量
static G_MPU6050: Mutex<RefCell<Option<mpu6050::MPU6050>>> = Mutex::new(RefCell::new(None));
static mut G_DATA: MaybeUninit<mpu6050::Data> = MaybeUninit::uninit();

static G_PC: Mutex<RefCell<Option<serial_inter::PC>>> = Mutex::new(RefCell::new(None));

static G_UPRIGHTCON: Mutex<RefCell<Option<upright::UprightCon>>> = Mutex::new(RefCell::new(None));

//----------------------------------------定时器中断函数
#[interrupt]
#[allow(non_snake_case)]
unsafe fn TIM3() { //上位机数据传输计时器
    static mut PC: Option<serial_inter::PC> = None;
    let pc = get_from_global!(PC, G_PC);


    pc.tim.wait().ok();
}


#[interrupt]
#[allow(non_snake_case)]
unsafe fn TIM4() {
    static mut MPU6050: Option<mpu6050::MPU6050> = None;
    static mut UPRIGHTCON: Option<upright::UprightCon> = None;
    let mpu6050 = get_from_global!(MPU6050, G_MPU6050);
    let upright_con = get_from_global!(UPRIGHTCON, G_UPRIGHTCON);

    mpu6050.refresh();
    upright_con.cal_speed();

    mpu6050.tim.wait().ok();
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

    let mut delay = Delay::new(cp.SYST, clocks);

    delay.delay_ms(100_u16);  //等待mpu6050模块启动

    //-------------------------------------定时器初始化
    let motor_pwm = Timer::tim2(
        dp.TIM2,
        &clocks,
        &mut rcc.apb1
    ).pwm::<Tim2NoRemap, _, _, _>(
        (
            gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl)
        ),
        &mut afio.mapr,
        1.hz()
    );

    let mut tim3 = Timer::tim3(
        dp.TIM3,
        &clocks,
        &mut rcc.apb1
    ).start_count_down(500.ms());

    let mut tim4 = Timer::tim4(
        dp.TIM4,
        &clocks,
        &mut rcc.apb1
    ).start_count_down(mpu6050::UNIT_TIME.ms());

    // tim3.listen(Event::Update);
    // tim4.listen(Event::Update);

    //------------------------------------全局变量初始化
    {
        let data = unsafe {
            get_mut_ptr!(G_DATA)
        };
        *data = mpu6050::Data::new();
    }

    //-----------------------------------功能模块初始化
    let mpu6050 = MPU6050::init(
        dp.I2C1,
        &mut afio.mapr,
        clocks,
        &mut rcc.apb1,
        gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl),
        gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl),
        gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
        unsafe { get_mut_ptr!(G_DATA) },
        tim4
    );

    let mut pc = PC::init(
        dp.USART1,
        gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
        gpioa.pa10,
        &mut afio.mapr,
        serial::Config::default().baudrate(28800.bps()),
        clocks,
        &mut rcc.apb2,
        gpiob.pb0.into_push_pull_output(&mut gpiob.crl),
        unsafe { get_ptr!(G_DATA) },
        tim3
    );

    let upright_con = UprightCon::init(
        motor_pwm,
        (
            gpiod.pd1.into_push_pull_output(&mut gpiod.crl),
            gpiod.pd15.into_push_pull_output(&mut gpiod.crh)
        ),
        unsafe { get_ptr!(G_DATA) }
    );

    let mut hc05 = hc05::HC05::init(
        dp.USART2,
        gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa3,
        gpioa.pa6,
        gpioa.pa7.into_push_pull_output(&mut gpioa.crl),
        &mut afio.mapr,
        serial::Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb1
    );

    //-----------------------------------功能模块分发
    send_to_global!(mpu6050, &G_MPU6050);

    // send_to_global!(pc, &G_PC);

    send_to_global!(upright_con, &G_UPRIGHTCON);

    //-----------------------------------启用定时器中断
    unsafe {
        // cp.NVIC.set_priority(Interrupt::TIM3, 0x60);
        // cp.NVIC.set_priority(Interrupt::TIM4, 0x10);

        // cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
        // cortex_m::peripheral::NVIC::unmask(Interrupt::TIM4);
    }


    loop {
        let f = hc05.waiting_for_data();
        pc.send_char(f);
    }
} 