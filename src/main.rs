#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use cortex_m::interrupt::Mutex;

use stm32f1xx_hal::{pac, prelude::*, serial};
use stm32f1xx_hal::timer::{CountDownTimer, Event, Timer};
use stm32f1xx_hal::pac::{interrupt,Interrupt,TIM2,TIM3};

use embedded_hal::digital::v2::OutputPin;

use core::cell::RefCell;

mod mpu6050;
mod blinky;
mod serial_inter;
mod motor;



static G_TIM2: Mutex<RefCell<Option<CountDownTimer<TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM3: Mutex<RefCell<Option<CountDownTimer<TIM3>>>> = Mutex::new(RefCell::new(None));
static G_MPU6050: Mutex<RefCell<Option<mpu6050::MPU6050>>> = Mutex::new(RefCell::new(None));
static G_PC: Mutex<RefCell<Option<serial_inter::PC>>> = Mutex::new(RefCell::new(None));
static G_BLINK: Mutex<RefCell<Option<blinky::Blink>>> = Mutex::new(RefCell::new(None));
static G_DATA: Mutex<RefCell<Option<mpu6050::Data>>> = Mutex::new(RefCell::new(None));


#[interrupt]
unsafe fn TIM2() {
    static mut TIM2: Option<CountDownTimer<TIM2>> = None;
    static mut MPU6050: Option<mpu6050::MPU6050> = None;
    static mut BLINK: Option<blinky::Blink> = None;

    let mpu6050 = MPU6050.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            G_MPU6050.borrow(cs).replace(None).unwrap()
        })
    });
    
    let tim2 = TIM2.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            G_TIM2.borrow(cs).replace(None).unwrap()
        })
    });

    let blink = BLINK.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            G_BLINK.borrow(cs).replace(None).unwrap()
        })
    });

    let data = mpu6050.refresh(); //额外定义一个变量，避免refresh操作锁定Mutex
    cortex_m::interrupt::free(|cs| *G_DATA.borrow(cs).borrow_mut() = Some(data));

    
    blink.flash();
    let _ = tim2.wait();
}

#[interrupt]
unsafe fn TIM3() {
    static mut TIM3: Option<CountDownTimer<TIM3>> = None;
    static mut PC: Option<serial_inter::PC> = None;

    let pc = PC.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            G_PC.borrow(cs).replace(None).unwrap()
        })
    });
    
    let tim3 = TIM3.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            G_TIM3.borrow(cs).replace(None).unwrap()
        })
    });

    let data = cortex_m::interrupt::free(|cs| {
        G_DATA.borrow(cs).replace(Some(mpu6050::Data {
            acc_x: 0,
            acc_z: 0,
            gyro_x: 0.0,
            angle: 0.0
        })).unwrap()
    });
    pc.send_all_of_mpu6050(data);

    
    let _ = tim3.wait();
}

#[entry]
fn main() -> ! {
    //let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc
        .cfgr
        .sysclk(72.mhz())
        .pclk1(8.mhz())
        .freeze(&mut flash.acr);


    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);

    
    let pb5 = gpiob.pb5.into_push_pull_output(&mut gpiob.crl);
    let pb6 = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let pb7 = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let mut step = gpiod.pd0.into_push_pull_output(&mut gpiod.crl);
    let mut dir = gpiod.pd1.into_push_pull_output(&mut gpiod.crl);
    let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let rx = gpioa.pa10;
    

    let (mpu6050, mpu6050_data) = mpu6050::init(
        dp.I2C1,
        &mut afio.mapr,
        clocks,
        &mut rcc.apb1,
        pb6,
        pb7
    );


    let pc = serial_inter::init(
        dp.USART1,
        tx,
        rx,
        &mut afio.mapr,
        serial::Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2
    );

    let blink = blinky::init(pb5);


    let mut tim2 = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1)
        .start_count_down(25.ms());
    
    let mut tim3 = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1)
        .start_count_down(1000.ms());

    let mut tim4 = Timer::tim4(dp.TIM4, &clocks, &mut rcc.apb1)
        .start_count_down(5000.us());

    tim2.listen(Event::Update);
    tim3.listen(Event::Update);


    cortex_m::interrupt::free(|cs| *G_TIM2.borrow(cs).borrow_mut() = Some(tim2));
    cortex_m::interrupt::free(|cs| *G_TIM3.borrow(cs).borrow_mut() = Some(tim3));
    cortex_m::interrupt::free(|cs| *G_MPU6050.borrow(cs).borrow_mut() = Some(mpu6050));
    
    cortex_m::interrupt::free(|cs| *G_PC.borrow(cs).borrow_mut() = Some(pc));
    cortex_m::interrupt::free(|cs| *G_BLINK.borrow(cs).borrow_mut() = Some(blink));
    cortex_m::interrupt::free(|cs| *G_DATA.borrow(cs).borrow_mut() = Some(mpu6050_data));

    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
    }


    let mut pb1 = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
    dir.set_high().ok();

    loop {

        step.set_high().ok();
        pb1.set_high().ok();
        block!(tim4.wait()).unwrap();

        step.set_low().ok();
        pb1.set_low().ok();
        block!(tim4.wait()).unwrap();
    }
}