//! snps,dw-apb-uart serial driver

use dw_apb_uart::DW8250;
use crate::sync::SpinNoIrqLock;

use crate::mm::{PhysAddr, VirtAddr};
use crate::sync::Mutex;

const UART_BASE: PhysAddr = PhysAddr::new(0xFEB50000);
const UART_IRQ_NUM: usize = 333;

pub static UART: SpinNoIrqLock<DW8250> = SpinNoIrqLock::new(DW8250::new(UART_BASE.into_kvaddr().as_usize()));

/// Writes a byte to the console.
pub fn console_putchar(c: u8) {
    let mut uart = UART.lock();
    match c {
        b'\r' | b'\n' => {
            uart.putchar(b'\r');
            uart.putchar(b'\n');
        }
        c => uart.putchar(c),
    }
}

/// Reads a byte from the console, or returns [`None`] if no input is available.
pub fn console_getchar() -> Option<u8> {
    UART.lock().getchar()
}

/// UART simply initialize
pub fn init_early() {

}


pub fn init() {
    // UART.lock().init();
    // UART interrupts are not supported currently
    crate::drivers::interrupt::set_enable(UART_IRQ_NUM, false);
}
