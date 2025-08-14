pub mod interrupt;
pub mod misc;
pub mod timer;
pub mod uart;

pub fn init_early() {
    uart::init_early();
}

pub fn init() {
    println!("Initializing gic...");
    interrupt::init();
    println!("Initializing gic done.");
    uart::init();
    println!("Initializing uart done.");
    timer::init();
    println!("Initializing timer done.");
}
