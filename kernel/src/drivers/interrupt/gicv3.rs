

use arm_gic::GicV3;
use crate::mm::{PhysAddr, VirtAddr, phys_to_virt};
use crate::utils::irq_handler::{IrqHandler, IrqHandlerTable};
use arm_gic::GenericArmGic;


const IRQ_COUNT: usize = 1024;

static HANDLERS: IrqHandlerTable<IRQ_COUNT> = IrqHandlerTable::new();

const GIC_BASE: usize = 0x0800_0000;
const GICD_BASE: PhysAddr = PhysAddr::new(GIC_BASE);
const GICC_BASE: PhysAddr = PhysAddr::new(GIC_BASE + 0xa0000);

pub static mut GIC: GicV3 = GicV3::new(GICD_BASE.into_kvaddr().as_mut_ptr(), GICC_BASE.into_kvaddr().as_mut_ptr());

pub fn set_enable(vector: usize, enable: bool) {
    info!("in platform gic set_enable: irq_num {}, enabled {}", vector, enable);
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
        // debug!("id :{:?}", intid);
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
