#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/irqchip/arm-gic.h>

#include <mach/mt_secure_api.h>

extern phys_addr_t INT_POL_CTL0_phys;


void mt_set_pol_reg(u32 reg_index, u32 value)
{	
	mcusys_smc_write_phy((INT_POL_CTL0_phys + (reg_index * 4)), value);
}

