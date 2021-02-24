#[macro_export]
macro_rules! send_to_global {
    ($val: expr, $addr: expr) => {
        cortex_m::interrupt::free(
            |cs| *$addr.borrow(cs).borrow_mut() = Some($val)
        )
    };
}

#[macro_export]
macro_rules! get_from_global {
    ($local:expr, $addr:expr) => {
        $local.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| {
                $addr.borrow(cs).replace(None).unwrap()
            })
        })
    };
}

#[macro_export]
macro_rules! get_ptr {
    ($addr:expr) => { & *$addr.as_ptr() };
}

#[macro_export]
macro_rules! get_mut_ptr {
    ($addr:expr) => { &mut *$addr.as_mut_ptr() };
}