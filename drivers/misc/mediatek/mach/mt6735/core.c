#include <linux/pm.h>
#include <linux/bug.h>
#include <linux/memblock.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/smp_scu.h>
#include <asm/page.h>
#include <mach/mt_reg_base.h>
#include <mach/irqs.h>
#include <linux/irqchip.h>
#include <linux/of_platform.h>

extern struct smp_operations mt_smp_ops;
extern void __init mt_timer_init(void);
extern void mt_fixup(struct tag *tags, char **cmdline, struct meminfo *mi);
#ifdef CONFIG_OF
extern void mt_dt_fixup(struct tag *tags, char **cmdline, struct meminfo *mi);
#endif
extern void mt_reserve(void);

/* FIXME: need to remove */
extern void arm_machine_restart(char mode, const char *cmd);

#if 1
static const struct of_device_id dt_bus_match[] __initconst = {
        { .compatible = "simple-bus", },
        {}
};
#endif
void __init mt_dt_init(void){

        of_platform_populate(NULL, dt_bus_match, NULL, NULL);

}
void __init mt_init(void)
{
            mt_dt_init();
}

static struct map_desc mt_io_desc[] __initdata =
{  
  {
		.virtual = CKSYS_BASE,
		.pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(CKSYS_BASE)),
		.length = SZ_4K * 19,
		.type = MT_DEVICE,
	},
	{
		.virtual = MCUCFG_BASE,
		.pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(MCUCFG_BASE)),
		.length = SZ_4K * 26,
		.type = MT_DEVICE
	},
	{
		.virtual = CA9_BASE,
		.pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(CA9_BASE)),
		.length = SZ_32K,
		.type = MT_DEVICE
	},
	{
		.virtual = DBGAPB_BASE,
		.pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(DBGAPB_BASE)),
		.length = SZ_1M,
		.type = MT_DEVICE,
	},	
	{
		.virtual = CCI400_BASE,
		.pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(CCI400_BASE)),
		.length = SZ_64K,
		.type = MT_DEVICE,
	},
    {
        /* virtual 0xF9000000, physical 0x00100000 */
        .virtual = SYSRAM_BASE,
        .pfn = __phys_to_pfn(0x00100000),
        .length = SZ_64K+SZ_64K+SZ_64K,
        .type = MT_MEMORY_NONCACHED
    },	
};

#ifdef CONFIG_OF
static const char *mt_dt_match[] __initdata =
{
    "mediatek,mt6735",
    NULL
};
#endif

void __init mt_map_io(void)
{
	iotable_init(mt_io_desc, ARRAY_SIZE(mt_io_desc));
}

static void __init mt_dt_init_irq(void)
{
            irqchip_init();
            mt_init_irq();
}


#ifdef CONFIG_OF
DT_MACHINE_START(MT6735_DT, "MT6735")
	.map_io		= mt_map_io,
	.smp		= smp_ops(mt_smp_ops),
	/*.init_irq	= mt_dt_init_irq,*/
	/*.init_time	= mt_timer_init,*/
	.init_machine	= mt_init,
	.fixup		= mt_dt_fixup,
	/* FIXME: need to implement the restart function */
	.restart	= arm_machine_restart,
	.reserve	= mt_reserve,
  .dt_compat  = mt_dt_match,
MACHINE_END
#endif
