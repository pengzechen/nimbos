//! snps,dw-apb-uart serial driver

use dw_apb_uart::DW8250;
use crate::sync::SpinNoIrqLock;

use crate::mm::{PhysAddr, VirtAddr};
use crate::sync::Mutex;
use crate::platform::config::UART_BASE_PADDR;

const UART_BASE: PhysAddr = PhysAddr::new(UART_BASE_PADDR);
const UART_IRQ_NUM: usize = 365;

pub static UART: SpinNoIrqLock<DW8250> = SpinNoIrqLock::new(DW8250::new(UART_BASE.into_kvaddr().as_usize()));

fn usize_to_u8_array(n: usize, buffer: &mut [u8; 20]) -> usize { 
      let mut number = n;
      let mut index = 0; 
      if number == 0 {
          buffer[index] = b'0';
          return 1;
      }
      while number > 0 && index < buffer.len() {
          buffer[index] = b'0' + (number % 10) as u8;
          number /= 10;
          index += 1;
      }
      buffer[0..index].reverse();
      index 
  }                                                                                   
fn write_str(s: &[u8]){
    for c in s {
        match c {
            b'\n' => {
                console_putchar(b'\r');
                console_putchar(b'\n');
            }
            _ => console_putchar(*c),
        }
    }
}

pub fn print_num(prefix: &[u8], num: usize) {
    write_str(prefix);
    if num != 0 {
        let mut buffer = [0u8; 20];
        let length = usize_to_u8_array(num, &mut buffer);
        let u8_array = &buffer[0..length];
        write_str(u8_array);
    }
    console_putchar(b'\n');
}

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
