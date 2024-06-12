cfg_if! {
    if #[cfg(target_arch = "x86_64")] {
        mod uart16550;
        use uart16550 as imp;
    } else if #[cfg(target_arch = "aarch64")] {
        //mod pl011;
        //use pl011 as imp;
        mod dw_apb_uart;
        use self::dw_apb_uart as imp;
    } else if #[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))] {
        mod riscv;
        use self::riscv as imp;
    }
}

pub use self::imp::{console_getchar, console_putchar, print_num};
pub(super) use self::imp::{init, init_early};
