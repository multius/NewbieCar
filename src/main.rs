#![no_std]
#![no_main]

use panic_halt as _;
// use nb::block;

//----------------------------引入cortex-m库
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

//----------------------------引入hal库
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::pac::{interrupt, Interrupt};
use stm32f1xx_hal::timer::{Event, Timer, CountDownTimer};
use stm32f1xx_hal::{pac, pac::{TIM3, TIM4}, timer::Tim2NoRemap};
use stm32f1xx_hal::delay::Delay;

// use embedded_hal::digital::v2::OutputPin;

//-------------------------引入核心库（core）
use core::cell::RefCell;
use core::mem::MaybeUninit;

//-------------------------引入子模块
mod hc05;
mod macro_lib;
mod mpu6050;
mod serial_inter;
mod motion_control;
mod motor;

use mpu6050::MPU6050;
use serial_inter::PC;
use motion_control::MotionCon;

//------------------------------------全局变量
static G_TIM3: Mutex<RefCell<Option<CountDownTimer<TIM3>>>> = Mutex::new(RefCell::new(None));
static G_TIM4: Mutex<RefCell<Option<CountDownTimer<TIM4>>>> = Mutex::new(RefCell::new(None));

static G_MPU6050: Mutex<RefCell<Option<mpu6050::MPU6050>>> = Mutex::new(RefCell::new(None));
static mut G_DATA: MaybeUninit<mpu6050::Data> = MaybeUninit::uninit();

static G_MOTIONCON: Mutex<RefCell<Option<motion_control::MotionCon>>> = Mutex::new(RefCell::new(None));
static mut G_STATE: MaybeUninit<motion_control::StateType> = MaybeUninit::uninit();

static G_PC: Mutex<RefCell<Option<serial_inter::PC>>> = Mutex::new(RefCell::new(None));
static G_HC05: Mutex<RefCell<Option<hc05::HC05>>> = Mutex::new(RefCell::new(None));

//----------------------------------------定时器中断函数
#[interrupt]
#[allow(non_snake_case)]
unsafe fn TIM3() {
    static mut TIM3: Option<CountDownTimer<TIM3>> = None;
    let tim3 = get_from_global!(TIM3, G_TIM3);

    tim3.wait().ok();
}

#[interrupt]
#[allow(non_snake_case)]
unsafe fn TIM4() {
    static mut TIM4: Option<CountDownTimer<TIM4>> = None;
    let tim4 = get_from_global!(TIM4, G_TIM4);

    static mut MPU6050: Option<mpu6050::MPU6050> = None;
    static mut MOTIONCON: Option<motion_control::MotionCon> = None;
    let mpu6050 = get_from_global!(MPU6050, G_MPU6050);
    let motion_con = get_from_global!(MOTIONCON, G_MOTIONCON);

    mpu6050.refresh();
    motion_con.adjust_speed();

    tim4.wait().ok();
}


#[entry]
fn main() -> ! {
    init();

    static mut HC05: Option<hc05::HC05> = None;
    let hc05 = unsafe {
        get_from_global!(HC05, G_HC05)
    };

    static mut PC: Option<serial_inter::PC> = None;
    let pc = unsafe {
        get_from_global!(PC, G_PC)
    };

    let state = unsafe {
        get_mut_ptr!(G_STATE)
    };


    loop {
        let flag = hc05.waiting_data();
        pc.send_char(flag);
        
        match flag {
            b'G' => { *state = motion_control::StateType::Forward }
            b'H' => { *state = motion_control::StateType::TurnLeft }
            b'I' => { *state = motion_control::StateType::Balance }
            b'J' => { *state = motion_control::StateType::TurnRight }
            b'K' => { *state = motion_control::StateType::Backward }
            _ => {}
        }
    }
}


//-------------------------------初始化设置
fn init() {
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


    //---------------------------等待外置模块启动
    delay.delay_ms(500_u16);


    //-------------------------------------定时器初始化
    let motor_pwm = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).pwm::<Tim2NoRemap, _, _, _>(
        (
            gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
        ),
        &mut afio.mapr,
        1.hz(),
    );

    let mut tim3 = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1)
        .start_count_down(500.ms());

    let mut tim4 = Timer::tim4(dp.TIM4, &clocks, &mut rcc.apb1)
        .start_count_down(mpu6050::UNIT_TIME.ms());


    //------------------------------------全局变量初始化
    unsafe {
        *get_mut_ptr!(G_DATA) = mpu6050::Data::new();
        *get_mut_ptr!(G_STATE) = motion_control::StateType::new();
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
        unsafe { get_mut_ptr!(G_DATA) }
    );

    let pc = PC::init(
        dp.USART1,
        gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
        gpioa.pa10,
        &mut afio.mapr,
        clocks,
        &mut rcc.apb2,
        unsafe { get_ptr!(G_DATA) },
    );

    let motor = motor::Motor::init(
        motor_pwm,
        (
            gpiod.pd1.into_push_pull_output(&mut gpiod.crl),
            gpiod.pd15.into_push_pull_output(&mut gpiod.crh),
        ),
    );

    let motion_con = MotionCon::init(
        motor,
        unsafe { get_ptr!(G_DATA) },
        unsafe { get_ptr!(G_STATE) }
    );

    let hc05 = hc05::HC05::init(
        dp.USART2,
        gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa3,
        gpioa.pa6,
        gpioa.pa7.into_push_pull_output(&mut gpioa.crl),
        &mut afio.mapr,
        clocks,
        &mut rcc.apb1,
    );


    //----------------------------------------定时器启用
    unsafe {
        tim3.listen(Event::Update);
        tim4.listen(Event::Update);

        cp.NVIC.set_priority(Interrupt::TIM3, 0x60);
        cp.NVIC.set_priority(Interrupt::TIM4, 0x10);

        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM4);
    }


    //-----------------------------------功能模块分发
    send_to_global!(tim3, &G_TIM3);
    send_to_global!(tim4, &G_TIM4);

    send_to_global!(mpu6050, &G_MPU6050);
    send_to_global!(pc, &G_PC);
    send_to_global!(motion_con, &G_MOTIONCON);
    send_to_global!(hc05, &G_HC05);
}