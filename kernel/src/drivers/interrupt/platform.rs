

pub enum SchedRule {
    /// Round-robin scheduling
    RoundRobin,
    /// No specific scheduling rule
    None,
}

pub const ARM_CORTEX_A57: u8 = 0;
pub const ARM_CORTEX_A55: u8 = 1;
pub const ARM_CORTEX_A76: u8 = 2;

pub struct PlatCpuCoreConfig {
    pub name: u8,
    pub mpidr: usize,
    pub sched: SchedRule,
}

pub struct PlatformConfig {
    pub cpu_desc: PlatCpuConfig,
}

pub struct PlatCpuConfig {
    pub num: usize,
    pub core_list: &'static [PlatCpuCoreConfig],
}

/// Static configuration for the Rockchip RK3588 platform
pub const PLAT_DESC: PlatformConfig = PlatformConfig {
    /// CPU configuration details for RK3588
    cpu_desc: PlatCpuConfig {
        num: 4,
        core_list: &[
            PlatCpuCoreConfig {
                //cluster0
                name: ARM_CORTEX_A55,
                mpidr: 0,
                sched: SchedRule::RoundRobin,
            },
            PlatCpuCoreConfig {
                //cluster0
                name: ARM_CORTEX_A55,
                mpidr: 1,
                sched: SchedRule::RoundRobin,
            },
            PlatCpuCoreConfig {
                //cluster0
                name: ARM_CORTEX_A55,
                mpidr: 2,
                sched: SchedRule::RoundRobin,
            },
            PlatCpuCoreConfig {
                //cluster0
                name: ARM_CORTEX_A55,
                mpidr: 3,
                sched: SchedRule::RoundRobin,
            },
        ]
    }
};


/// Maps CPU ID to CPU interface number for RK3588
pub fn cpuid_to_cpuif(cpuid: usize) -> usize {
    PLAT_DESC.cpu_desc.core_list[cpuid].mpidr
}

pub fn cpuid2mpidr(cpuid: usize) -> usize {
    PLAT_DESC.cpu_desc.core_list[cpuid].mpidr
}


pub const GICD_BASE: usize = 0x08000000;
pub const GICR_BASE: usize = 0x080a0000;
