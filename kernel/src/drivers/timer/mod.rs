pub const MSEC_PER_SEC: u64 = 1000;
pub const USEC_PER_SEC: u64 = MSEC_PER_SEC * 1000;
pub const NSEC_PER_SEC: u64 = USEC_PER_SEC * 1000;

cfg_if! {
    if #[cfg(feature = "platform-pc")] {
        mod x86_timer;
        use x86_timer as imp;
    } else if #[cfg(feature = "platform-qemu-virt-arm")] {
        mod generic_timer;
        use generic_timer as imp;
    }
}

pub use self::imp::{get_time_ns, init};