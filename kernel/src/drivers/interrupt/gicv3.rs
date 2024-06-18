
use arm_gic::GicV3;
use crate::mm::{PhysAddr, VirtAddr, phys_to_virt};
use crate::utils::irq_handler::{IrqHandler, IrqHandlerTable};
use arm_gic::GenericArmGic;

const IRQ_COUNT: usize = 1024;

static HANDLERS: IrqHandlerTable<IRQ_COUNT> = IrqHandlerTable::new();

const GIC_BASE: usize = crate::config::GICD_BASE_PADDR;
const GICD_BASE: PhysAddr = PhysAddr::new(GIC_BASE);
const GICR_BASE: PhysAddr = PhysAddr::new(crate::config::GICR_BASE_PADDR);

pub static mut GIC: GicV3 = GicV3::new(GICD_BASE.into_kvaddr().as_mut_ptr(), 
    GICR_BASE.into_kvaddr().as_mut_ptr());

pub fn set_enable(vector: usize, enable: bool) {
    unsafe {
        if enable {
            GIC.enable_interrupt(vector.into());
        } else {
            GIC.disable_interrupt(vector.into());
        }
    }
}

pub fn handle_irq(_vector: usize) {
    let intid = unsafe { GIC.get_and_acknowledge_interrupt() };
    if let Some(id) = intid {
        HANDLERS.handle(id.into());
        unsafe {
            GIC.end_interrupt(id);
        }
    }
}

pub fn register_handler(vector: usize, handler: IrqHandler) {
    HANDLERS.register_handler(vector, handler);
}

fn gic_global_init() {
    unsafe {  GIC.global_init(); }
}

fn gic_local_init() {
    unsafe { GIC.local_init(); }
    unsafe { GIC.gicc_init(); }
}

pub fn init() {
    // gic_global_init();
    // gic_local_init();
    unsafe { GIC.init_primary(); }
}
