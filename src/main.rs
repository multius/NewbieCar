
#![no_std]
#![no_main]

// use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
// use nb::block;

//----------------------------引入cortex-m库
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

//----------------------------引入hal库
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::pac::{interrupt, Interrupt};
use stm32f1xx_hal::timer::{Event, Timer, CountDownTimer};
use stm32f1xx_hal::{pac, pac::TIM3, timer::Tim2NoRemap};
use stm32f1xx_hal::delay::Delay;

// use embedded_hal::digital::v2::OutputPin;

//-------------------------引入核心库（core）
use core::cell::RefCell;
use core::mem::MaybeUninit;
// use core::fmt::Write as write;

//-------------------------引入子模块
mod hc05;
mod macro_lib;
mod mpu6050;
// mod serial_inter;
mod motion_control;
mod motor;

use mpu6050::MPU6050;
// use serial_inter::PC;
use motion_control::MotionCon;

//------------------------------------全局变量
static G_TIM3: Mutex<RefCell<Option<CountDownTimer<TIM3>>>> = Mutex::new(RefCell::new(None));

static G_MOTIONCON: Mutex<RefCell<Option<motion_control::MotionCon>>> = Mutex::new(RefCell::new(None));
// static G_PC: Mutex<RefCell<Option<serial_inter::PC>>> = Mutex::new(RefCell::new(None));

static mut G_PARS: MaybeUninit<hc05::Pars> = MaybeUninit::uninit();
// static mut G_STATE: MaybeUninit<motion_control::StateType> = MaybeUninit::uninit();
static mut G_MPU6050_DATA: MaybeUninit<mpu6050::Data> = MaybeUninit::uninit();
static mut G_MC_DATA: MaybeUninit<motion_control::Data> = MaybeUninit::uninit();


//----------------------------------------定时器中断函数
#[interrupt]
#[allow(non_snake_case)]
unsafe fn TIM3() {
    static mut TIM3: Option<CountDownTimer<TIM3>> = None;
    let tim3 = get_from_global!(TIM3, G_TIM3);

    static mut MOTIONCON: Option<motion_control::MotionCon> = None;
    let motion_con = get_from_global!(MOTIONCON, G_MOTIONCON);

    tim3.wait().ok();

    motion_con.adjust_motion(0.0);

}

#[entry]
fn main() -> ! {
    let mut master = init();

    // static mut PC: Option<serial_inter::PC> = None;
    // let pc = unsafe {
    //     get_from_global!(PC, G_PC)
    // };

    // let state = unsafe { get_mut_ptr!(G_STATE) };

    let mut last_half = hc05::get_half(master.rx_circbuf.readable_half());

    loop {
        master.send_packets();

        if master.rx_circbuf.readable_half().unwrap() != last_half {

            last_half = master.rx_circbuf.readable_half().unwrap();

            master.packets_analyse();
        }
    }
}


//-------------------------------初始化设置
fn init() -> hc05::HC05<'static> {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let channels = dp.DMA1.split(&mut rcc.ahb);

    let clocks = rcc
        .cfgr
        .sysclk(72.mhz())
        .pclk1(32.mhz())
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
        .start_count_down(mpu6050::UNIT_TIME.ms());


    //------------------------------------全局变量初始化
    unsafe {
        *get_mut_ptr!(G_MPU6050_DATA) = mpu6050::Data::new();
        *get_mut_ptr!(G_MC_DATA) = motion_control::Data::new();
        // *get_mut_ptr!(G_STATE) = motion_control::StateType::new();
        *get_mut_ptr!(G_PARS) = hc05::Pars::new();
    }


    // -----------------------------------功能模块初始化
    let mpu6050 = MPU6050::init(
        dp.I2C1,
        &mut afio.mapr,
        clocks,
        &mut rcc.apb1,
        gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl),
        gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl),
        gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
        unsafe { get_mut_ptr!(G_MPU6050_DATA) }, //获取全局DATA的控制权
        unsafe { get_ptr!(G_PARS) } //获取全局PARS的读取权
    );

    // let pc = PC::init(
    //     dp.USART1,
    //     gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
    //     gpioa.pa10,
    //     &mut afio.mapr,
    //     clocks,
    //     &mut rcc.apb2,
    //     unsafe { get_ptr!(G_DATA) },
    // );

    let motors = motor::Motors::init(
        motor_pwm,
        (
            gpiod.pd1.into_push_pull_output(&mut gpiod.crl),
            gpiod.pd15.into_push_pull_output(&mut gpiod.crh),
        )
    );

    let motion_con = MotionCon::init(
        motors,
        unsafe { get_ptr!(G_MPU6050_DATA) },
        unsafe { get_ptr!(G_PARS) }, //获取全部全局变量的读取权
        mpu6050,
        unsafe { get_mut_ptr!(G_MC_DATA) }
    );

    let hc05 = hc05::HC05::init(
        dp.USART2,
        gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa3,
        &mut afio.mapr,
        clocks,
        &mut rcc.apb1,
        channels,
        unsafe { get_mut_ptr!(G_PARS) },//获取全局PARS的控制权
        unsafe { get_ptr!(G_MPU6050_DATA) },//获取全局MPU6050_DATA的读取权
        unsafe{ get_ptr!(G_MC_DATA) }//获取全局MC_DATA的读取权
    );


    //----------------------------------------定时器启用
    unsafe {
        tim3.listen(Event::Update);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
    }


    //-----------------------------------功能模块分发
    send_to_global!(tim3, &G_TIM3);

    // send_to_global!(pc, &G_PC);
    send_to_global!(motion_con, &G_MOTIONCON);

    hc05
}