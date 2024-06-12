#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(asm_const, naked_functions)]
#![feature(panic_info_message, alloc_error_handler)]
#![feature(const_refs_to_cell)]
#![feature(const_maybe_uninit_zeroed)]
#![feature(get_mut_unchecked)]

extern crate alloc;
#[macro_use]
extern crate cfg_if;
#[macro_use]
extern crate log;

#[macro_use]
mod logging;

mod arch;
mod config;
mod drivers;
mod loader;
mod mm;
mod percpu;
mod platform;
mod sync;
mod syscall;
mod task;
mod timer;
mod utils;

#[cfg(not(test))]
mod lang_items;

extern "C" {
    fn sbss();
    fn ebss();
}
fn clear_bss() {
    unsafe {
        core::slice::from_raw_parts_mut(sbss as usize as *mut u8, ebss as usize - sbss as usize)
            .fill(0);
    }
}

const LOGO: &str = r"
NN   NN  iii               bb        OOOOO    SSSSS
NNN  NN       mm mm mmmm   bb       OO   OO  SS
NN N NN  iii  mmm  mm  mm  bbbbbb   OO   OO   SSSSS
NN  NNN  iii  mmm  mm  mm  bb   bb  OO   OO       SS
NN   NN  iii  mmm  mm  mm  bbbbbb    OOOO0    SSSSS
              ___    ____    ___    ___
             |__ \  / __ \  |__ \  |__ \
             __/ / / / / /  __/ /  __/ /
            / __/ / /_/ /  / __/  / __/
           /____/ \____/  /____/ /____/
";


use core::sync::atomic::Ordering;
use core::arch::asm;

#[inline(always)]
unsafe fn atomic_store(dst: *mut u64, val: u64, order: Ordering) {
    match order {
        Ordering::Relaxed => {
            asm!(
                "str {1}, [{0}]",
                in(reg) dst,
                in(reg) val,
                options(nostack, preserves_flags)
            );
        }
        Ordering::Release => {
            asm!(
                "stlr {1}, [{0}]",
                in(reg) dst,
                in(reg) val,
                options(nostack, preserves_flags)
            );
        }
        _ => unimplemented!("Unsupported ordering"),
    }
}

#[inline(always)]
unsafe fn atomic_load(src: *const u64, order: Ordering) -> u64 {
    let mut val: u64;
    match order {
        Ordering::Relaxed => {
            asm!(
                "ldr {0}, [{1}]",
                out(reg) val,
                in(reg) src,
                options(nostack, preserves_flags)
            );
        }
        Ordering::Acquire => {
            asm!(
                "ldar {0}, [{1}]",
                out(reg) val,
                in(reg) src,
                options(nostack, preserves_flags)
            );
        }
        _ => unimplemented!("Unsupported ordering"),
    }
    val
}

#[inline(always)]
unsafe fn atomic_cmp_exchange(dst: *mut u64, current: u64, new: u64, success: Ordering, failure: Ordering) -> Result<u64, u64> {
    let mut prev: u64;
    let res: u32;

    match (success, failure) {
        (Ordering::SeqCst, Ordering::SeqCst) => {
            asm!(
                "1: ldxr {0}, [{2}]
                    cmp {0}, {3}
                    b.ne 2f
                    stxr {1:w}, {4}, [{2}]
                    cbnz {1:w}, 1b
                    2:",
                out(reg) prev,
                out(reg) res,
                in(reg) dst,
                in(reg) current,
                in(reg) new,
                options(nostack)
            );
        }
        _ => unimplemented!("Unsupported ordering"),
    }

    if res == 0 {
        Ok(prev)
    } else {
        Err(prev)
    }
}

fn guoweikang_test() {
    // Example usage
    let mut data: u64 = 42;
    unsafe {
        atomic_store(&mut data, 100, Ordering::Relaxed);
        let loaded = atomic_load(&data, Ordering::Acquire);
        crate::drivers::uart::print_num("loaded value: ".as_bytes(), loaded as usize);
        match atomic_cmp_exchange(&mut data, 100, 200, Ordering::SeqCst, Ordering::SeqCst) {
            Ok(old) => 
        crate::drivers::uart::print_num("swap success old : ".as_bytes(), old as usize),
            Err(old) =>  crate::drivers::uart::print_num("swap failed old : ".as_bytes(), old as usize),
        }
    }
}

#[inline(always)]
unsafe fn test_ldxr(dst: u64) -> u64 {
    let mut prev: u64;
    asm!(
        "
        ldxr {0}, [{1}]",
            out(reg) prev,
            in(reg) dst,
            options(nostack)
    );
    prev
}

#[inline(never)]
fn guoweikang_test2() {
    // Example usage
    unsafe {
        let loaded = test_ldxr(0xffff_0000_7000_0000);
        crate::drivers::uart::print_num("read value: ".as_bytes(), loaded as usize);
    }
}

#[no_mangle]
pub fn rust_main() -> ! {
    clear_bss();
    drivers::init_early();
    println!("{}", LOGO);
    println!(
        "\
        arch = {}\n\
        platform = {}\n\
        build_mode = {}\n\
        log_level = {}\n\
        ",
        option_env!("ARCH").unwrap_or(""),
        option_env!("PLATFORM").unwrap_or(""),
        option_env!("MODE").unwrap_or(""),
        option_env!("LOG").unwrap_or(""),
    );

    mm::init_heap_early();
    logging::init();
    info!("Logging is enabled.");

    arch::init();
    arch::init_percpu();
    percpu::init_percpu_early();

    mm::init();
    drivers::init();

    percpu::init_percpu();
    timer::init();
    info!("timer init success");
    task::init();
    loader::list_apps();
    info!("task run");
    task::run();
}
