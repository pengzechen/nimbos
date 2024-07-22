//! ARM Generic Timer.

use cortex_a::registers::{CNTFRQ_EL0, CNTVCT_EL0, CNTV_CTL_EL0, CNTV_TVAL_EL0};
use tock_registers::interfaces::{Readable, Writeable};

use crate::drivers::interrupt;
use crate::sync::LazyInit;
use crate::timer::NANOS_PER_SEC;
use crate::utils::ratio::Ratio;

const HV_TIMER_IRQ_NUM: usize = 27;

static CNTVCT_TO_NANOS_RATIO: LazyInit<Ratio> = LazyInit::new();
static NANOS_TO_CNTVCT_RATIO: LazyInit<Ratio> = LazyInit::new();

pub fn current_ticks() -> u64 {
    CNTVCT_EL0.get()
}

pub fn ticks_to_nanos(ticks: u64) -> u64 {
    CNTVCT_TO_NANOS_RATIO.mul(ticks)
}

#[allow(dead_code)]
pub fn nanos_to_ticks(nanos: u64) -> u64 {
    NANOS_TO_CNTVCT_RATIO.mul(nanos)
}

pub fn set_oneshot_timer(deadline_ns: u64) {
    let cnptct = CNTVCT_EL0.get();
    let cnptct_deadline = NANOS_TO_CNTVCT_RATIO.mul(deadline_ns);
    if cnptct < cnptct_deadline {
        let interval = cnptct_deadline - cnptct;
        debug_assert!(interval <= u32::MAX as u64);
        CNTV_TVAL_EL0.set(interval);
    } else {
        CNTV_TVAL_EL0.set(0);
    }
}

pub fn init() {
    CNTVCT_TO_NANOS_RATIO.init_by(Ratio::new(NANOS_PER_SEC as u32, CNTFRQ_EL0.get() as u32));
    NANOS_TO_CNTVCT_RATIO.init_by(CNTVCT_TO_NANOS_RATIO.inverse());

    
    interrupt::register_handler(HV_TIMER_IRQ_NUM, crate::timer::handle_timer_irq);
    interrupt::set_enable(HV_TIMER_IRQ_NUM, true);
    CNTV_CTL_EL0.write(CNTV_CTL_EL0::ENABLE::SET);
}