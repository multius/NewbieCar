
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use stm32f1xx_hal::{ pac, prelude::*, serial::Config, timer::{CountDownTimer, Event, Timer}};
use pac::{interrupt,Interrupt,TIM2,TIM3};

use core::cell::RefCell;
use cortex_m::{asm::wfi, interrupt::Mutex};

mod mpu6050;
mod blinky;
mod inter_PC;

static G_TIM2: Mutex<RefCell<Option<CountDownTimer<TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM3: Mutex<RefCell<Option<CountDownTimer<TIM3>>>> = Mutex::new(RefCell::new(None));
static G_MPU6050: Mutex<RefCell<Option<mpu6050::MPU6050>>> = Mutex::new(RefCell::new(None));
static G_PC: Mutex<RefCell<Option<inter_PC::PC>>> = Mutex::new(RefCell::new(None));
static G_BLINK: Mutex<RefCell<Option<blinky::Blink>>> = Mutex::new(RefCell::new(None));
static G_DATA: Mutex<RefCell<Option<mpu6050::Data>>> = Mutex::new(RefCell::new(None));

// #[interrupt]
// unsafe fn TIM2() {
//     static mut TIM2: Option<CountDownTimer<TIM2>> = None;
//     static mut MPU6050: Option<mpu6050::MPU6050> = None;
//     static mut BLINK: Option<blinky::Blink> = None;

//     let mpu6050 = MPU6050.get_or_insert_with(|| {
//         cortex_m::interrupt::free(|cs| {
//             // Move LED pin here, leaving a None in its place
//             G_MPU6050.borrow(cs).replace(None).unwrap()
//         })
//     });
    
//     let tim2 = TIM2.get_or_insert_with(|| {
//         cortex_m::interrupt::free(|cs| {
//             // Move LED pin here, leaving a None in its place
//             G_TIM2.borrow(cs).replace(None).unwrap()
//         })
//     });

//     // let pc = PC.get_or_insert_with(|| {
//     //     cortex_m::interrupt::free(|cs| {
//     //         // Move LED pin here, leaving a None in its place
//     //         G_PC.borrow(cs).replace(None).unwrap()
//     //     })
//     // });

//     let blink = BLINK.get_or_insert_with(|| {
//         cortex_m::interrupt::free(|cs| {
//             // Move LED pin here, leaving a None in its place
//             G_BLINK.borrow(cs).replace(None).unwrap()
//         })
//     });

//     cortex_m::interrupt::free(|cs| *G_DATA.borrow(cs).borrow_mut() = Some(mpu6050.refresh()));

    
//     blink.flash();
//     let _ = tim2.wait();
// }

#[interrupt]
unsafe fn TIM3() {
    static mut TIM3: Option<CountDownTimer<TIM3>> = None;
    static mut PC: Option<inter_PC::PC> = None;

    let pc = PC.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_PC.borrow(cs).replace(None).unwrap()
        })
    });
    
    let tim3 = TIM3.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_TIM3.borrow(cs).replace(None).unwrap()
        })
    });

    pc.send_all_of_mpu6050(
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_DATA.borrow(cs).replace(None).unwrap()
        })
    );

    
    let _ = tim3.wait();
}

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    //let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
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

    
    let pb5 = gpiob.pb5.into_push_pull_output(&mut gpiob.crl);
    let pb6 = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let pb7 = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let rx = gpioa.pa10;
    

    //let mut timer = Timer::syst(cp.SYST, &clocks).start_count_down(1.hz());
    

    let (mut mpu6050, mpu6050_data) = mpu6050::init(
        dp.I2C1,
        &mut afio.mapr,
        clocks,
        &mut rcc.apb1,
        pb6,
        pb7
    );


    let pc = inter_PC::init(
        dp.USART1,
        tx,
        rx,
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2
    );

    let mut blink = blinky::init(pb5);


    // let mut tim2 = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1)
    //     .start_count_down(1000.ms());
    
    let mut tim3 = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1)
        .start_count_down(1000.ms());

    // tim2.listen(Event::Update);
    tim3.listen(Event::Update);


    // cortex_m::interrupt::free(|cs| *G_TIM2.borrow(cs).borrow_mut() = Some(tim2));
    cortex_m::interrupt::free(|cs| *G_TIM3.borrow(cs).borrow_mut() = Some(tim3));
    // cortex_m::interrupt::free(|cs| *G_MPU6050.borrow(cs).borrow_mut() = Some(mpu6050));
    
    cortex_m::interrupt::free(|cs| *G_PC.borrow(cs).borrow_mut() = Some(pc));
    // cortex_m::interrupt::free(|cs| *G_BLINK.borrow(cs).borrow_mut() = Some(blink));
    cortex_m::interrupt::free(|cs| *G_DATA.borrow(cs).borrow_mut() = Some(mpu6050_data));

    unsafe {
        // cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
    }

    loop {

        
        blink.flash();
        cortex_m::interrupt::free(|cs| *G_DATA.borrow(cs).borrow_mut() = Some(mpu6050.refresh()));
    
        // block!(tim3.wait()).unwrap();
    }
}