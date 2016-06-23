#include <mach/upmu_common.h>
#include <mach/sync_write.h>
#include <mach/mt_clkmgr.h>
#include <mach/emi_mpu.h>

#ifdef CONFIG_OF_RESERVED_MEM
#include <linux/of_reserved_mem.h>
#include <mach/mtk_memcfg.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <linux/gpio.h>

static unsigned long infra_ao_base = 0; // 0x1000_0000
static unsigned long sleep_base = 0; // 0x1000_6000
static unsigned long toprgu_base = 0; // 0x1000_7000
static unsigned long c2k_chip_id_base = 0;
static unsigned long md1_pccif_base = 0;
static unsigned long md3_pccif_base = 0;
static unsigned int  c2k_wdt_irq_id = 0;
static unsigned long c2k_iram_base = 0;
static unsigned long c2k_iram_base_seg2[2] = {0};

#define ENABLE_C2K_JTAG 0

#define MD3_BANK0_MAP0 (0x310)
#define MD3_BANK0_MAP1 (0x314)
#define MD3_BANK4_MAP0 (0x318)
#define MD3_BANK4_MAP1 (0x31C)
#define INFRA_AO_C2K_CONFIG (0x330)
#define INFRA_AO_C2K_STATUS (0x334)
#define INFRA_AO_C2K_SPM_CTRL (0x338)
#define SLEEP_CLK_CON (0x400)
#define TOP_RGU_WDT_MODE (0x0)
#define TOP_RGU_WDT_SWRST (0x14)
#define TOP_RGU_WDT_SWSYSRST (0x18)
#define TOP_RGU_WDT_NONRST_REG (0x20)
#define PCCIF_BUSY (0x4)
#define PCCIF_TCHNUM (0xC)
#define PCCIF_ACK (0x14)
#define PCCIF_CHDATA (0x100)
#define PCCIF_SRAM_SIZE (512)

#define AP_PLATFORM_INFO    "MT6735E1"

extern void mtk_wdt_set_c2k_sysrst(unsigned int flag);

#define ENABLE_C2K_EMI_PROTECTION	(1)
//===================================================
// MPU Region defination
//===================================================
#define MPU_REGION_ID_C2K_ROM       7
#define MPU_REGION_ID_C2K_RW        8
#define MPU_REGION_ID_C2K_SMEM      10

#define MD3_ROM_SIZE				(0x380000)
#define MD3_RAM_SIZE_BOTTOM			(0x180000)
#define MD3_RAM_SIZE_TOP			(0x500000)
#define MD3_SHARE_MEM_SIZE			(0x600000)

#define AP_READY_BIT				(0x1 << 1)
#define AP_WAKE_C2K_BIT				(0x1 << 0)

#define ETS_SEL_BIT					(0x1 << 13)

#define c2k_write32(b, a, v) mt_reg_sync_writel(v, (b)+(a))
#define c2k_write16(b, a, v) mt_reg_sync_writew(v, (b)+(a))
#define c2k_write8(b, a, v)  mt_reg_sync_writeb(v, (b)+(a))
#define c2k_read32(b, a)     ioread32((void __iomem *)((b)+(a)))
#define c2k_read16(b, a)     ioread16((void __iomem *)((b)+(a)))
#define c2k_read8(b, a)      ioread8((void __iomem *)((b)+(a)))

#ifdef CONFIG_OF_RESERVED_MEM
#define CCCI_MD3_MEM_RESERVED_KEY "reserve-memory-ccci_md3"
#define MD3_MEM_SIZE (16*1024*1024)
phys_addr_t md3_mem_base;
static unsigned long md3_mem_base_virt = 0;
#define C2K_IMG_DUMP_SIZE 		(0x1000)
#define C2K_IMG_DUMP_OFFSET		(0x61000)
#define C2K_IRAM_DUMP_SIZE 		(0x200)
static int first_init = 1;

void c2k_platform_restore_first_init(void)
{
	first_init = 1;
}
	
int modem_sdio_reserve_mem_of_init(struct reserved_mem * rmem, unsigned long node, const char * uname)
{
	phys_addr_t rptr = 0;
	unsigned int rsize= 0;
	
	rptr = rmem->base;
	rsize= (unsigned int)rmem->size;	
	MTK_MEMCFG_LOG_AND_PRINTK(KERN_ALERT "%s,uname:%s,base:0x%llx,size:0x%x\n", __func__, rmem->name, (unsigned long long)rptr, rsize);

	if(strcmp(uname, CCCI_MD3_MEM_RESERVED_KEY) == 0){
		if(rsize != MD3_MEM_SIZE)	{
			MTK_MEMCFG_LOG_AND_PRINTK(KERN_ERR "%s: reserve size=0x%x < 0x%x\n", __func__, rsize, MD3_MEM_SIZE);
			return 0;
		}
	}
	md3_mem_base = rmem->base;
	return 0;
}
RESERVEDMEM_OF_DECLARE(ccci_reserve_mem_md3_init,CCCI_MD3_MEM_RESERVED_KEY,modem_sdio_reserve_mem_of_init);
#endif

static void c2k_hw_info_init()
{
#ifdef CONFIG_OF
	struct device_node * node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,INFRACFG_AO");
	infra_ao_base = (unsigned long)of_iomap(node, 0);

	node = of_find_compatible_node(NULL, NULL, "mediatek,SLEEP");
	sleep_base = (unsigned long)of_iomap(node, 0);

	node = of_find_compatible_node(NULL, NULL, "mediatek,TOPRGU");
	toprgu_base = (unsigned long)of_iomap(node, 0);
	
	node = of_find_compatible_node(NULL, NULL, "mediatek,MDC2K");
	c2k_chip_id_base = (unsigned long)of_iomap(node, 0);
	md1_pccif_base = (unsigned long)of_iomap(node, 1);
	md3_pccif_base = (unsigned long)of_iomap(node, 2);
	c2k_wdt_irq_id = irq_of_parse_and_map(node, 0);

#ifdef CONFIG_OF_RESERVED_MEM
	md3_mem_base_virt = ioremap_nocache(md3_mem_base + C2K_IMG_DUMP_OFFSET, C2K_IMG_DUMP_SIZE);
#endif

	printk("[C2K] infra_ao_base=0x%lx, sleep_base=0x%lx, toprgu_base=0x%lx, c2k_chip_id_base=0x%lx, md3_mem_base_virt=0x%lx, c2k_wdt_irq_id=%d\n",
		infra_ao_base, sleep_base, toprgu_base, c2k_chip_id_base, md3_mem_base_virt, c2k_wdt_irq_id);
#endif
}


void c2k_mem_dump(void *start_addr, int len)
{
	unsigned int *curr_p = (unsigned int *)start_addr;
	unsigned char *curr_ch_p;
	int _16_fix_num = len/16;
	int tail_num = len%16;
	char buf[16];
	int i,j;

	if(NULL == curr_p) {
		printk("[C2K-DUMP]NULL point to dump!\n");
		return;
	}
	if(0 == len){
		printk("[C2K-DUMP]Not need to dump\n");
		return;
	}

	printk("[C2K-DUMP]Base: 0x%lx\n", (unsigned long)start_addr);
	// Fix section
	for(i=0; i<_16_fix_num; i++){
		printk("[C2K-DUMP]%03X: %08X %08X %08X %08X\n", 
				i*16, *curr_p, *(curr_p+1), *(curr_p+2), *(curr_p+3) );
		curr_p+=4;
	}

	// Tail section
	if(tail_num > 0){
		curr_ch_p = (unsigned char*)curr_p;
		for(j=0; j<tail_num; j++){
			buf[j] = *curr_ch_p;
			curr_ch_p++;
		}
		for(; j<16; j++)
			buf[j] = 0;
		curr_p = (unsigned int*)buf;
		printk("[C2K-DUMP]%03X: %08X %08X %08X %08X\n", 
				i*16, *curr_p, *(curr_p+1), *(curr_p+2), *(curr_p+3) );
	}
}


void dump_c2k_iram()
{
	if(md3_mem_base_virt == 0){
		printk("[C2K] md3_mem_base_virt is 0, exit without dump...\n");
		return;
	}
	
	printk("[C2K] Dump C2K MEM begin, size 4K\n");
	c2k_mem_dump(md3_mem_base_virt, C2K_IMG_DUMP_SIZE);
	printk("[C2K] Dump C2K MEM end\n");
}

void dump_c2k_iram_seg2()
{
	int i;

	for (i = 0; i < sizeof(c2k_iram_base_seg2)/sizeof(c2k_iram_base_seg2[0]); i++) {
		if(c2k_iram_base_seg2[i] == 0){
			printk("[C2K] c2k_iram_base_seg2[%d] is 0, exit...\n", i);
			return;
		}
		printk("[C2K] Dump C2K IRAM begin, start at %lx\n", c2k_iram_base_seg2[i]);
		c2k_mem_dump(c2k_iram_base_seg2[i], C2K_IRAM_DUMP_SIZE);
	}

	printk("[C2K] Dump C2K MEM end\n");
	
}

#if ENABLE_C2K_EMI_PROTECTION
void set_c2k_mpu()
{
	unsigned int shr_mem_phy_start, shr_mem_phy_end, shr_mem_mpu_id, shr_mem_mpu_attr;
    unsigned int rom_mem_phy_start, rom_mem_phy_end, rom_mem_mpu_id, rom_mem_mpu_attr;
    unsigned int rw_mem_phy_start, rw_mem_phy_end, rw_mem_mpu_id, rw_mem_mpu_attr;
	
	rom_mem_mpu_id = MPU_REGION_ID_C2K_ROM;
	rw_mem_mpu_id = MPU_REGION_ID_C2K_RW;
	shr_mem_mpu_id = MPU_REGION_ID_C2K_SMEM;
	
	rom_mem_mpu_attr = SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, NO_PROTECTION, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, SEC_R_NSEC_R);
	rw_mem_mpu_attr =  SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, NO_PROTECTION, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, SEC_R_NSEC_R);
	shr_mem_mpu_attr = SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, NO_PROTECTION, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, NO_PROTECTION);

	/*
	 * if set start=0x0, end=0x10000, the actural protected area will be 0x0-0x1FFFF,
	 * here we use 64KB align, MPU actually request 32KB align since MT6582, but this works...
	 * we assume emi_mpu_set_region_protection will round end address down to 64KB align.
	 */
	rw_mem_phy_start  = (unsigned int)md3_mem_base;
	rw_mem_phy_end	  = ((rw_mem_phy_start + MD3_RAM_SIZE_BOTTOM + MD3_ROM_SIZE + MD3_RAM_SIZE_TOP + 0xFFFF)&(~0xFFFF)) - 0x1;
	
	rom_mem_phy_start = (unsigned int)(rw_mem_phy_start + MD3_RAM_SIZE_BOTTOM);
	rom_mem_phy_end   = ((rom_mem_phy_start + MD3_ROM_SIZE + 0xFFFF)&(~0xFFFF)) - 0x1;
	
	shr_mem_phy_start = rw_mem_phy_end + 0x1;
	shr_mem_phy_end   = ((shr_mem_phy_start + MD3_SHARE_MEM_SIZE + 0xFFFF)&(~0xFFFF)) - 0x1;

	printk("[C2K] MPU Start protect MD R/W region<%d:%08x:%08x> %x\n", 
			rw_mem_mpu_id, rw_mem_phy_start, rw_mem_phy_end, rw_mem_mpu_attr);
	emi_mpu_set_region_protection(rw_mem_phy_start, 		 /*START_ADDR*/
									rw_mem_phy_end, 	  /*END_ADDR*/
									rw_mem_mpu_id,		  /*region*/
									rw_mem_mpu_attr);
	
	printk("[C2K] MPU Start protect MD ROM region<%d:%08x:%08x> %x\n", 
			rom_mem_mpu_id, rom_mem_phy_start, rom_mem_phy_end, rom_mem_mpu_attr);
	emi_mpu_set_region_protection(rom_mem_phy_start,	  /*START_ADDR*/
									rom_mem_phy_end,	  /*END_ADDR*/
									rom_mem_mpu_id, 	  /*region*/
									rom_mem_mpu_attr);

	printk("[C2K] MPU Start protect MD Share region<%d:%08x:%08x> %x\n", 
			shr_mem_mpu_id, shr_mem_phy_start, shr_mem_phy_end, shr_mem_mpu_attr);
	emi_mpu_set_region_protection(shr_mem_phy_start,	  /*START_ADDR*/
									shr_mem_phy_end,	  /*END_ADDR*/
									shr_mem_mpu_id, 	  /*region*/
									shr_mem_mpu_attr);
}
#endif


static void md_gpio_get(GPIO_PIN pin, char *tag)
{
	printk("GPIO%d(%s): mode=%d,dir=%d,in=%d,out=%d,pull_en=%d,pull_sel=%d,smt=%d\n",
			pin, tag,
			mt_get_gpio_mode(pin),
			mt_get_gpio_dir(pin),
			mt_get_gpio_in(pin),
			mt_get_gpio_out(pin),
			mt_get_gpio_pull_enable(pin),
			mt_get_gpio_pull_select(pin),
			mt_get_gpio_smt(pin));
}

static void md_gpio_set(GPIO_PIN pin, GPIO_MODE mode, GPIO_DIR dir, GPIO_OUT out, GPIO_PULL_EN pull_en, GPIO_PULL pull, GPIO_SMT smt)
{
	mt_set_gpio_mode(pin, mode);
	if(dir != GPIO_DIR_UNSUPPORTED)
		mt_set_gpio_dir(pin, dir);

	if(dir == GPIO_DIR_OUT) {
		mt_set_gpio_out(pin, out);
	}
	if(dir == GPIO_DIR_IN) {
		mt_set_gpio_smt(pin, smt);
	}
	if(pull_en != GPIO_PULL_EN_UNSUPPORTED) {
		mt_set_gpio_pull_enable(pin, pull_en);
		mt_set_gpio_pull_select(pin, pull);
	}
	md_gpio_get(pin, "-");
}

void enable_c2k_jtag(unsigned int mode)
{
	static void __iomem *c2k_jtag_setting;
	if (mode == 1){
		md_gpio_set(0x80000000 | GPIO82, GPIO_MODE_05, GPIO_DIR_IN, GPIO_OUT_UNSUPPORTED, GPIO_PULL_ENABLE, GPIO_PULL_DOWN, GPIO_SMT_ENABLE);
		md_gpio_set(0x80000000 | GPIO81, GPIO_MODE_05, GPIO_DIR_IN, GPIO_OUT_UNSUPPORTED, GPIO_PULL_ENABLE, GPIO_PULL_UP, GPIO_SMT_ENABLE);
		md_gpio_set(0x80000000 | GPIO83, GPIO_MODE_05, GPIO_DIR_IN, GPIO_OUT_UNSUPPORTED, GPIO_PULL_ENABLE, GPIO_PULL_UP, GPIO_SMT_DISABLE);
		md_gpio_set(0x80000000 | GPIO85, GPIO_MODE_05, GPIO_DIR_IN, GPIO_OUT_UNSUPPORTED, GPIO_PULL_ENABLE, GPIO_PULL_UP, GPIO_SMT_DISABLE);
		md_gpio_set(0x80000000 | GPIO84, GPIO_MODE_05, GPIO_DIR_OUT, GPIO_OUT_ZERO, GPIO_PULL_EN_UNSUPPORTED, GPIO_PULL_UNSUPPORTED, GPIO_SMT_UNSUPPORTED);
		md_gpio_set(0x80000000 | GPIO86, GPIO_MODE_05, GPIO_DIR_OUT, GPIO_OUT_ZERO, GPIO_PULL_EN_UNSUPPORTED, GPIO_PULL_UNSUPPORTED, GPIO_SMT_UNSUPPORTED);
	}else if (mode == 2){
		if (first_init){
			first_init = 0;
			c2k_jtag_setting = ioremap_nocache(0x10000700, 0x04);
		}
		c2k_write32(c2k_jtag_setting, 0, c2k_read32(c2k_jtag_setting, 0) | (0x1 << 2));
	}
		
}


void c2k_modem_power_on_platform(void)
{
	int ret;

#if ENABLE_C2K_JTAG
// ARM legacy JTAG for C2K
	md_gpio_set(0x80000000 | GPIO82, GPIO_MODE_05, GPIO_DIR_IN, GPIO_OUT_UNSUPPORTED, GPIO_PULL_ENABLE, GPIO_PULL_DOWN, GPIO_SMT_ENABLE);
	md_gpio_set(0x80000000 | GPIO81, GPIO_MODE_05, GPIO_DIR_IN, GPIO_OUT_UNSUPPORTED, GPIO_PULL_ENABLE, GPIO_PULL_UP, GPIO_SMT_ENABLE);
	md_gpio_set(0x80000000 | GPIO83, GPIO_MODE_05, GPIO_DIR_IN, GPIO_OUT_UNSUPPORTED, GPIO_PULL_ENABLE, GPIO_PULL_UP, GPIO_SMT_DISABLE);
	md_gpio_set(0x80000000 | GPIO85, GPIO_MODE_05, GPIO_DIR_IN, GPIO_OUT_UNSUPPORTED, GPIO_PULL_ENABLE, GPIO_PULL_UP, GPIO_SMT_DISABLE);
	md_gpio_set(0x80000000 | GPIO84, GPIO_MODE_05, GPIO_DIR_OUT, GPIO_OUT_ZERO, GPIO_PULL_EN_UNSUPPORTED, GPIO_PULL_UNSUPPORTED, GPIO_SMT_UNSUPPORTED);
	md_gpio_set(0x80000000 | GPIO86, GPIO_MODE_05, GPIO_DIR_OUT, GPIO_OUT_ZERO, GPIO_PULL_EN_UNSUPPORTED, GPIO_PULL_UNSUPPORTED, GPIO_SMT_UNSUPPORTED);
#endif
	printk("[C2K] c2k_modem_power_on enter\n");
	if (infra_ao_base == 0)
		c2k_hw_info_init();
	printk("[C2K] prepare to power on c2k\n");
	// step 0: power on MTCMOS
	ret = md_power_on(SYS_MD2);
	printk("[C2K] md_power_on %d\n", ret);
	// step 1: set C2K boot mode
	c2k_write32(infra_ao_base, INFRA_AO_C2K_CONFIG, (c2k_read32(infra_ao_base, INFRA_AO_C2K_CONFIG)&(~(0x7<<8))));
	printk("[C2K] C2K_CONFIG = 0x%x\n", c2k_read32(infra_ao_base, INFRA_AO_C2K_CONFIG));
	// step 2: config srcclkena selection mask
	c2k_write32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL, c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL)&(~(0xF<<2)));
	c2k_write32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL, c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL)|(0x9<<2));
	printk("[C2K] C2K_SPM_CTRL = 0x%x\n", c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL));
	c2k_write32(sleep_base, SLEEP_CLK_CON, c2k_read32(sleep_base, SLEEP_CLK_CON)|0xc);
	c2k_write32(sleep_base, SLEEP_CLK_CON, c2k_read32(sleep_base, SLEEP_CLK_CON)&(~(0x1<<14)));
	c2k_write32(sleep_base, SLEEP_CLK_CON, c2k_read32(sleep_base, SLEEP_CLK_CON)|(0x1<<12));
	c2k_write32(sleep_base, SLEEP_CLK_CON, c2k_read32(sleep_base, SLEEP_CLK_CON)|(0x1<<27));
	printk("[C2K] SLEEP_CLK_CON = 0x%x\n", c2k_read32(sleep_base, SLEEP_CLK_CON));
	
	// step 3: PMIC VTCXO_1 enable
	pmic_config_interface(0x0A02, 0xA12E, 0xFFFF, 0x0);
	// step 4: reset C2K
	#if 0
	c2k_write32(toprgu_base, TOP_RGU_WDT_SWSYSRST, (c2k_read32(toprgu_base, TOP_RGU_WDT_SWSYSRST)|0x88000000)&(~(0x1<<15)));
	#else
	mtk_wdt_set_c2k_sysrst(1);
	#endif
	printk("[C2K] TOP_RGU_WDT_SWSYSRST = 0x%x\n", c2k_read32(toprgu_base, TOP_RGU_WDT_SWSYSRST));
	
	// step 5: set memory remap
	if(first_init) {
		first_init = 0;
        c2k_write32(infra_ao_base, MD3_BANK0_MAP0, ((((unsigned int)md3_mem_base - 0x40000000)>>24)|0x1)&0xFF);
#if ENABLE_C2K_EMI_PROTECTION
		set_c2k_mpu();
#endif
		printk("[C2K] MD3_BANK0_MAP0 = 0x%x\n", c2k_read32(infra_ao_base, MD3_BANK0_MAP0));
	}
	
	// step 6: wake up C2K
	c2k_write32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL, c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL)|0x1);
	while(!((c2k_read32(infra_ao_base, INFRA_AO_C2K_STATUS)>>1)&0x1)) {
		printk("[C2K] C2K_STATUS = 0x%x\n", c2k_read32(infra_ao_base, INFRA_AO_C2K_STATUS));
	}
	c2k_write32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL, c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL)&(~0x1));
	printk("[C2K] C2K_SPM_CTRL = 0x%x, C2K_STATUS = 0x%x\n", 
		c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL),
		c2k_read32(infra_ao_base, INFRA_AO_C2K_STATUS));
#if 0
	while(c2k_read32(c2k_chip_id_base, 0) != 0x020AC000) {
		printk("[C2K] C2K_CHIP_ID = 0x%x\n", c2k_read32(c2k_chip_id_base, 0));
	}
	printk("[C2K] C2K_CHIP_ID = 0x%x!!\n", c2k_read32(c2k_chip_id_base, 0));
#endif
}


#define AP_CCIF_BASE	(0x10218000)

#define APCCIF_CON		(0x00)
#define APCCIF_BUSY		(0x04)
#define APCCIF_START	(0x08)
#define APCCIF_TCHNUM	(0x0C)
#define APCCIF_RCHNUM	(0x10)
#define APCCIF_ACK		(0x14)
#define APCCIF_CHDATA	(0x100)

static void __iomem *ap_ccif_base;
int ccif_notify_c2k(int ch_id)
{
	int busy = 0;

	if (first_init){
		first_init = 0;
		ap_ccif_base = ioremap_nocache(AP_CCIF_BASE, 0x100);
	}
	busy = c2k_read32(ap_ccif_base, APCCIF_BUSY);

	if(busy & (1<<ch_id)) {
		printk("[C2K] ccif busy now, ch%d\n", ch_id);
        return -1;
    }
	printk("[C2K] ccif_notify_c2k, ch%d\n", ch_id);
	c2k_write32(ap_ccif_base, APCCIF_BUSY, 1<<ch_id);
	c2k_write32(ap_ccif_base, APCCIF_TCHNUM, ch_id);
	return 0;
}

int dump_ccif()
{
	printk("[C2K] ccif APCCIF_BUSY(%d), APCCIF_START(%d)\n", c2k_read32(ap_ccif_base, APCCIF_BUSY), c2k_read32(ap_ccif_base, APCCIF_START));
}

void c2k_modem_power_off_platform(void)
{
	int ret;
	printk("[C2K] md_power_off begain\n");
	ret = md_power_off(SYS_MD2, 1000);
	printk("[C2K] md_power_off %d\n", ret);
}


void c2k_modem_reset_platform(void)
{	
	int ret = 0;
	
	if (infra_ao_base == 0)
		c2k_hw_info_init();

	c2k_write32(toprgu_base, TOP_RGU_WDT_SWSYSRST, (c2k_read32(toprgu_base, TOP_RGU_WDT_SWSYSRST)|0x88000000)|(0x1<<15));

	ret = md_power_on(SYS_MD2);
	printk("[C2K] md_power_on %d\n", ret);
	
	// step 1: reset C2K
	#if 0
	printk("[C2K] set toprgu wdt");
	c2k_write32(toprgu_base, TOP_RGU_WDT_SWSYSRST, (c2k_read32(toprgu_base, TOP_RGU_WDT_SWSYSRST)|0x88000000)&(~(0x1<<15)));
	#else
	mtk_wdt_set_c2k_sysrst(1);
	#endif
	printk("[C2K] TOP_RGU_WDT_SWSYSRST = 0x%x\n", c2k_read32(toprgu_base, TOP_RGU_WDT_SWSYSRST));

	// step 2: wake up C2K
	c2k_write32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL, c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL)|0x1);
	while(!((c2k_read32(infra_ao_base, INFRA_AO_C2K_STATUS)>>1)&0x1));
	c2k_write32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL, c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL)&(~0x1));
	printk("[C2K] C2K_SPM_CTRL = 0x%x\n", c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL));
	
#if 0
	while(c2k_read32(c2k_chip_id_base, 0) != 0x020AC000) {
		printk("[C2K] C2K_CHIP_ID = 0x%x\n", c2k_read32(c2k_chip_id_base, 0));
	}
	printk("[C2K] C2K_CHIP_ID = 0x%x\n", c2k_read32(c2k_chip_id_base, 0));
#endif
}

extern void ccci_mem_dump(int md_id, void *start_addr, int len);
void c2k_modem_reset_pccif(void)
{
	unsigned int tx_channel = 0;
	int i;
	// clear occupied channel
	while(tx_channel<16) {
		if(c2k_read32(md1_pccif_base, PCCIF_BUSY) & (1<<tx_channel)) {
			c2k_write32(md1_pccif_base, PCCIF_TCHNUM, tx_channel);
		}
		if(c2k_read32(md3_pccif_base, PCCIF_BUSY) & (1<<tx_channel)) {
			c2k_write32(md3_pccif_base, PCCIF_TCHNUM, tx_channel);
		}
		tx_channel++;
	}
	// clear un-ached channel
	c2k_write32(md1_pccif_base, PCCIF_ACK, c2k_read32(md3_pccif_base, PCCIF_BUSY));
	c2k_write32(md3_pccif_base, PCCIF_ACK, c2k_read32(md1_pccif_base, PCCIF_BUSY));
	// clear SRAM
	for(i=0; i<PCCIF_SRAM_SIZE/sizeof(unsigned int); i++) {
		c2k_write32(md1_pccif_base, PCCIF_CHDATA+i*sizeof(unsigned int), 0);
		c2k_write32(md3_pccif_base, PCCIF_CHDATA+i*sizeof(unsigned int), 0);
	}
	// dump
	printk("[C2K] Dump MD1 PCCIF\n");
	ccci_mem_dump(-1, md1_pccif_base, 0x300);
	printk("[C2K] Dump MD3 PCCIF\n");
	ccci_mem_dump(-1, md3_pccif_base, 0x300);
}

void set_ap_ready(int value)
{
	unsigned int reg_value;

	if (infra_ao_base == 0)
		c2k_hw_info_init();
	
	reg_value = c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL);

	if (value)
		reg_value |= AP_READY_BIT;
	else
		reg_value &= ~AP_READY_BIT;		//set 0 to indicate ap ready
	
	c2k_write32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL, reg_value);
	printk("[C2K]%s: set ap ready %d\n", __FUNCTION__, value);
}

void set_ap_wake_cp(int value)
{
	unsigned int reg_value;

	if (infra_ao_base == 0)
		c2k_hw_info_init();

	reg_value = c2k_read32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL);
	
	if (value)
		reg_value |= AP_WAKE_C2K_BIT;
	else
		reg_value &= ~AP_WAKE_C2K_BIT;	//set 0 to wake up cp
	
	c2k_write32(infra_ao_base, INFRA_AO_C2K_SPM_CTRL, reg_value);

	printk("[C2K] ap_wake_cp(%d)\n", value);

	return 0;
}


void set_ets_sel(int value)
{
	unsigned int reg_value;

	if (infra_ao_base == 0)
		c2k_hw_info_init();

	reg_value = c2k_read32(infra_ao_base, INFRA_AO_C2K_CONFIG);
	
	if (value)
		reg_value |= ETS_SEL_BIT;
	else
		reg_value &= ~ETS_SEL_BIT;	//set 0 to wake up cp
	
	c2k_write32(infra_ao_base, INFRA_AO_C2K_CONFIG, reg_value);
	
	printk("[C2K] set ETS SEL to %d (reg %d)\n", value, c2k_read32(infra_ao_base, INFRA_AO_C2K_CONFIG));

	return 0;
}



unsigned int get_c2k_wdt_irq_id()
{
	if (c2k_wdt_irq_id == 0)
		c2k_hw_info_init();

	return c2k_wdt_irq_id;
}

unsigned int get_c2k_reserve_mem_size()
{
	return MD3_MEM_SIZE;
}

char *get_ap_platform()
{
	return AP_PLATFORM_INFO;
}
