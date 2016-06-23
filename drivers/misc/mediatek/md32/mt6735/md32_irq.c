#include <linux/workqueue.h>
#include <linux/aee.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/aee.h>
#include <linux/interrupt.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_reg_base.h>
#include <mach/sync_write.h>
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#include <mach/md32_excep.h>
#include "md32_irq.h"


#define MD32_AP_DEBUG  1
#if(MD32_AP_DEBUG == 1)
#define dbg_msg printk
#else
#define dbg_msg(...)
#endif


//#define MT_IPC_MD2HOST_INTR    198  //from mt_irqs.h
#ifdef MD32_WDT_ASSERT_REGIST
static md32_wdt_func MD32_WDT_FUN;
static md32_assert_func  MD32_ASSERT_FUN;
static struct work_struct work_md32_wdt;
static struct work_struct work_md32_assert;
static struct workqueue_struct *wq_md32_wdt;
static struct workqueue_struct *wq_md32_assert;
#endif
//static struct work_struct work_md32_reboot;
static struct workqueue_struct *wq_md32_reboot;

struct md32_aed_cfg {
    int *log;
    int log_size;
    int *phy;
    int phy_size;
    char *detail;
};

struct md32_reboot_work {
    struct work_struct work;
    struct md32_aed_cfg aed;
};

static struct md32_reboot_work work_md32_reboot;

struct md32_status_reg {
    unsigned int status;
    unsigned int pc;
    unsigned int r14;
    unsigned int r15;
    unsigned int m2h_irq;
};

static struct md32_status_reg md32_aee_status;

struct md32_excep_desc md32_excep_desc[MD32_NR_EXCEP];

extern unsigned int md32_log_buf_addr;
extern unsigned int md32_log_start_idx_addr;
extern unsigned int md32_log_end_idx_addr;
extern unsigned int md32_log_lock_addr;
extern unsigned int md32_log_buf_len_addr;


int md32_excep_registration(md32_excep_id id, md32_excep_handler_t stop_handler, 
                                  md32_excep_handler_t reset_handler, md32_excep_handler_t restart_handler, const char *name)
{
    if(id < MD32_NR_EXCEP)
    {
        md32_excep_desc[id].name = name;
        if(stop_handler != NULL)
            md32_excep_desc[id].stop_handler = stop_handler;
        if(reset_handler != NULL)
            md32_excep_desc[id].reset_handler = reset_handler;
        if(restart_handler != NULL)
            md32_excep_desc[id].restart_handler = restart_handler;
        return 0;
    }
    else
    {
        return -1;
    }
}

void md32_dump_regs(void)
{
    volatile unsigned int *reg;

    printk("md32_dump_regs\n");
    printk("md32 status = 0x%08x\n", *(volatile unsigned int *)MD32_BASE);
    printk("md32 pc=0x%08x, r14=0x%08x, r15=0x%08x\n", MD32_DEBUG_PC_REG, MD32_DEBUG_R14_REG, MD32_DEBUG_R15_REG);
    printk("md32 to host inerrupt = 0x%08x\n", MD32_TO_HOST_REG);
    printk("host to md32 inerrupt = 0x%08x\n", HOST_TO_MD32_REG);

    printk("wdt en=%d, count=0x%08x\n",
           (MD32_WDT_REG & 0x10000000) ? 1 : 0,
           (MD32_WDT_REG & 0xFFFFF));

    /*dump all md32 regs*/
    for(reg = (volatile unsigned int *)MD32_BASE;
        reg < (MD32_BASE + 0xA8); reg++)
    {
        //if(!(reg & 0xF))
        {
            printk("\n");
            printk("[0x%p]   ", reg);
        }

        printk("0x%08x  ", *reg);

    }
    printk("\n");
    printk("md32_dump_regs end\n");
}

void md32_aee_stop(void)
{
    printk("md32_aee_stop\n");

    md32_aee_status.status = *(volatile unsigned int *)MD32_BASE;
    md32_aee_status.pc = MD32_DEBUG_PC_REG;
    md32_aee_status.r14 = MD32_DEBUG_R14_REG;
    md32_aee_status.r15 = MD32_DEBUG_R15_REG;

    mt_reg_sync_writel(0x0, MD32_BASE);

    printk("md32_aee_stop end\n");
}

int md32_aee_dump(char *buf)
{
    volatile unsigned int *reg;
    char *ptr = buf;

    ssize_t i, len;
    unsigned int log_start_idx;
    unsigned int log_end_idx;
    unsigned int log_buf_max_len;
    unsigned char *__log_buf = (unsigned char *)(MD32_DTCM + md32_log_buf_addr);

    if(!buf)
        return 0;

    printk("md32_aee_dump\n");
    ptr += sprintf(ptr, "md32 status = 0x%08x\n", md32_aee_status.status);
    ptr += sprintf(ptr, "md32 pc=0x%08x, r14=0x%08x, r15=0x%08x\n", md32_aee_status.pc, md32_aee_status.r14, md32_aee_status.r15);
    ptr += sprintf(ptr, "md32 to host irq = 0x%08x\n", md32_aee_status.m2h_irq);
    ptr += sprintf(ptr, "host to md32 irq = 0x%08x\n", HOST_TO_MD32_REG);

    ptr += sprintf(ptr, "wdt en=%d, count=0x%08x\n",
           (MD32_WDT_REG & 0x10000000) ? 1 : 0,
           (MD32_WDT_REG & 0xFFFFF));

    /*dump all md32 regs*/
    for(reg = (volatile unsigned int *)MD32_BASE;
        reg < (MD32_BASE + 0xA8); reg++)
    {
        //if(!(reg & 0xF))
        {
            ptr += sprintf(ptr, "\n");
            ptr += sprintf(ptr, "[0x%p]   ", reg);
        }

        ptr += sprintf(ptr, "0x%08x  ", *reg);

    }
    ptr += sprintf(ptr, "\n");

#define LOG_BUF_MASK (log_buf_max_len-1)
#define LOG_BUF(idx) (__log_buf[(idx) & LOG_BUF_MASK])

    log_start_idx = readl((void __iomem *)(MD32_DTCM + md32_log_start_idx_addr));
    log_end_idx = readl((void __iomem *)(MD32_DTCM + md32_log_end_idx_addr));
    log_buf_max_len = readl((void __iomem *)(MD32_DTCM + md32_log_buf_len_addr));

    ptr += sprintf(ptr, "log_buf_addr = 0x%08x\n", (unsigned int)md32_log_buf_addr);
    ptr += sprintf(ptr, "log_start_idx = %u\n", log_start_idx);
    ptr += sprintf(ptr, "log_end_idx = %u\n", log_end_idx);
    ptr += sprintf(ptr, "log_buf_max_len = %u\n", log_buf_max_len);

    ptr += sprintf(ptr, "<<md32 log buf start>>\n");
    len = (log_buf_max_len > 0x1000) ? 0x1000 : log_buf_max_len;
    i = 0;
    while((log_start_idx != log_end_idx) && i < len)
    {
        ptr += sprintf(ptr, "%c", LOG_BUF(log_start_idx));
        log_start_idx++;
        i++;
    }
    ptr += sprintf(ptr, "<<md32 log buf end>>\n");

    printk("md32_aee_dump end\n");

    return ptr - buf;
}

void md32_prepare_aed(char *aed_str, struct md32_aed_cfg *aed)
{
	char *detail;
    u8 *log, *phy, *ptr;
    u32 log_size, phy_size;

    printk("md32_prepare_aed\n");

    detail = (char *)kmalloc(MD32_AED_STR_LEN, GFP_KERNEL);
    if(!detail)
    {
        printk("ap allocate detail buffer fail, size=0x%x\n", MD32_AED_STR_LEN);
    }
    else
    {
        ptr = detail;
        detail[MD32_AED_STR_LEN - 1] = '\0';
        ptr += snprintf(detail, MD32_AED_STR_LEN, "%s", aed_str);
        ptr += sprintf(ptr, " md32 pc=0x%08x, r14=0x%08x, r15=0x%08x\n", md32_aee_status.pc, md32_aee_status.r14, md32_aee_status.r15);
    }

    log_size = 0x4000; /* 16KB */
    log = (u8 *)kmalloc(log_size, GFP_KERNEL);
    if(!log)
    {
        printk("ap allocate log buffer fail, size=0x%x\n", log_size);
        log_size = 0;
    }
    else
    {
        memset(log, 0, log_size);

        ptr = log;

        ptr += md32_aee_dump(ptr);

        /* print log in kernel */
        pr_debug("%s", log);

        ptr += sprintf(ptr, "dump memory info\n");
        ptr += sprintf(ptr, "md32 cfgreg: 0x%08x\n", 0);
        ptr += sprintf(ptr, "md32 ptcm  : 0x%08x\n", MD32_CFGREG_SIZE);
        ptr += sprintf(ptr, "md32 dtcm  : 0x%08x\n", MD32_CFGREG_SIZE + MD32_PTCM_SIZE);

#if 0
        ptr += sprintf(ptr, "<<md32 log buf>>\n");
        int size = log_size - (ptr - log);
        ptr += md32_get_log_buf(ptr, size);
#endif

        *ptr = '\0';
        log_size = ptr - log;
    }

    phy_size = MD32_AED_PHY_SIZE;
    phy = (u8 *)kmalloc(phy_size, GFP_KERNEL);
    if(!phy)
    {
        printk("ap allocate phy buffer fail, size=0x%x\n", phy_size);
        phy_size = 0;
    }
    else
    {
        ptr = phy;

        memcpy_from_md32((void *)ptr, (void *)MD32_BASE, MD32_CFGREG_SIZE);
        ptr += MD32_CFGREG_SIZE;

        memcpy_from_md32((void *)ptr, (void *)MD32_PTCM, MD32_PTCM_SIZE);
        ptr += MD32_PTCM_SIZE;

        memcpy_from_md32((void *)ptr, (void *)MD32_DTCM, MD32_DTCM_SIZE);
        ptr += MD32_DTCM_SIZE;
    }

    aed->log = (int *)log;
    aed->log_size = log_size;
    aed->phy = (int *)phy;
    aed->phy_size = phy_size;
    aed->detail = detail;

    printk("md32_prepare_aed end\n");
}

void md32_dmem_abort_handler(void)
{
    printk("[MD32 EXCEP] DMEM Abort\n");
    md32_dump_regs();
}
void md32_pmem_abort_handler(void)
{
    printk("[MD32 EXCEP] PMEM Abort\n");
    md32_dump_regs();
}

void md32_wdt_handler(void)
{
    printk("[MD32 EXCEP] WDT\n");
    md32_dump_regs();
#if 0
    for(int index=0;index<MD32_MAX_USER;index++)
    {
        dbg_msg("in use - index %d,%d\n",index,MD32_WDT_FUN.in_use[index]);
        if((MD32_WDT_FUN.in_use[index]==1) && (MD32_WDT_FUN.wdt_func[index]!=NULL) )
        {
            dbg_msg("do call back - index %d\n",index);
            MD32_WDT_FUN.wdt_func[index](MD32_WDT_FUN.private_data[index]);
        }
    }
    //queue_work(wq_md32_wdt,&work_md32_wdt);
#endif
}

#ifdef MD32_WDT_ASSERT_REGIST
void md32_wdt_reset_func(void)
{
    int index;
#define TEST_PHY_SIZE 0x10000
    printk("reset_md32_func\n");

    //1. AEE function
    aee_kernel_exception("MD32","MD32 WDT Time out ");
    //2. Reset md32
    printk("WDT Reset md32 ok!\n");
    //3. Call driver's callback
    for(index=0;index<MD32_MAX_USER;index++)
    {
        if(MD32_WDT_FUN.in_use[index] && MD32_WDT_FUN.reset_func[index]!=NULL )
        {
            MD32_WDT_FUN.reset_func[index](MD32_WDT_FUN.private_data[index]);
        }
    }
}

void md32_assert_reset_func(void)
{
    int index;
    printk("reset_md32_func\n");
    //1. AEE function
    aee_kernel_exception("MD32","MD32 ASSERT ");
    //2. Reset md32
    printk("ASSERT Reset md32 ok!\n");
    //3. Call driver's callback
    for(index=0;index<MD32_MAX_USER;index++)
    {
        if(MD32_ASSERT_FUN.in_use[index] && MD32_ASSERT_FUN.reset_func[index]!=NULL )
        {
            MD32_ASSERT_FUN.reset_func[index](MD32_ASSERT_FUN.private_data[index]);
        }
    }
}

int alloc_md32_wdt_func(void)
{
    int index;
    for(index=0;index<MD32_MAX_USER;index++)
    {
        if(!MD32_WDT_FUN.in_use[index])
        {
            MD32_WDT_FUN.in_use[index]=1;
            return index;
        }
    }
    return -1;
}

int alloc_md32_assert_func(void)
{
    int index;
    for(index=0;index<MD32_MAX_USER;index++)
    {
        if(!MD32_ASSERT_FUN.in_use[index])
        {
            MD32_ASSERT_FUN.in_use[index]=1;
            return index;
        }
    }
    return -1;
}

int md32_wdt_register_handler_services( void MD32_WDT_FUNC_PTR(void*),void WDT_RESET(void), void* private_data, char *module_name)
{
    int index_cur;
    index_cur=alloc_md32_wdt_func();
    dbg_msg("wdt register index %d\n",index_cur);
    if(index_cur<0)
    {
        dbg_msg("MD32_WDT_FUNC is full");
        return -1;
    }
    MD32_WDT_FUN.wdt_func[index_cur]=MD32_WDT_FUNC_PTR;
    MD32_WDT_FUN.reset_func[index_cur]=WDT_RESET;
    MD32_WDT_FUN.private_data[index_cur]=private_data;
    strcpy(MD32_WDT_FUN.MODULE_NAME[index_cur],module_name);
    return 1;
}

int md32_assert_register_handler_services( void MD32_ASSERT_FUNC_PTR(void*),void ASSERT_RESET (void), void* private_data, char *module_name)
{
    int index_cur;
    index_cur=alloc_md32_assert_func();
    dbg_msg("assert register index %d\n",index_cur);
    if(index_cur<0)
    {
        dbg_msg("MD32_ASSERT_FUNC is full");
        return -1;
    }
    MD32_ASSERT_FUN.assert_func[index_cur]=MD32_ASSERT_FUNC_PTR;
    MD32_ASSERT_FUN.reset_func[index_cur]=ASSERT_RESET;
    MD32_ASSERT_FUN.private_data[index_cur]=private_data;
    strcpy(MD32_ASSERT_FUN.MODULE_NAME[index_cur],module_name);
    return 1;
}

int free_md32_wdt_func(char *module_name)
{
    int index;
    dbg_msg("Flush works in WDT work queue\n");
    flush_workqueue(wq_md32_wdt);
    dbg_msg("Free md32_wdt structure\n");
    for(index=0;index<MD32_MAX_USER;index++)
    {
        if(strcmp(module_name,MD32_WDT_FUN.MODULE_NAME[index])==0)
        {
            MD32_WDT_FUN.in_use[index]=0;
            MD32_WDT_FUN.wdt_func[index]=NULL;
            MD32_WDT_FUN.reset_func[index]=NULL;
            MD32_WDT_FUN.private_data[index]=NULL;
            return 0;
        }
    }
    dbg_msg("Can't free %s\n",module_name);
    return -1;
}

int free_md32_assert_func(char *module_name)
{
    int index;
    dbg_msg("Flush works in ASSERT work queue\n");
    flush_workqueue(wq_md32_assert);
    dbg_msg("Free md32_assert structure\n");
    for(index=0;index<MD32_MAX_USER;index++)
    {
        if(strcmp(module_name,MD32_ASSERT_FUN.MODULE_NAME[index])==0)
        {
            MD32_ASSERT_FUN.in_use[index]=0;
            MD32_ASSERT_FUN.assert_func[index]=NULL;
            MD32_ASSERT_FUN.reset_func[index]=NULL;
            MD32_ASSERT_FUN.private_data[index]=NULL;
            return 0;
        }
    }
    dbg_msg("Can't free %s\n",module_name);
    return -1;
}
#endif

irqreturn_t md32_irq_handler(int irq, void *dev_id)
{
    struct reg_md32_to_host_ipc *md32_irq;
    int reboot = 0;

    md32_irq = (struct reg_md32_to_host_ipc *)MD32_TO_HOST_ADDR;

    if(md32_irq->wdt_int)
    {
        md32_wdt_handler();
        md32_aee_stop();
#if 0
        md32_prepare_aed("md32 wdt", &work_md32_reboot.aed);
        mt_reg_sync_writel(0x0, MD32_BASE);
#endif
        md32_aee_status.m2h_irq = MD32_TO_HOST_REG;
        md32_irq->wdt_int = 0;
        reboot = 1;
    }

    if(md32_irq->pmem_disp_int)
    {
        md32_pmem_abort_handler();
        md32_aee_stop();
#if 0
        md32_prepare_aed("md32 pmem abort", &work_md32_reboot.aed);
        mt_reg_sync_writel(0x0, MD32_BASE);
#endif
        md32_aee_status.m2h_irq = MD32_TO_HOST_REG;
        md32_irq->pmem_disp_int = 0;
        reboot = 1;
    }

    if(md32_irq->dmem_disp_int)
    {
        md32_dmem_abort_handler();
        md32_aee_stop();
#if 0
        md32_prepare_aed("md32 dmem abort", &work_md32_reboot.aed);
        mt_reg_sync_writel(0x0, MD32_BASE);
#endif
        md32_aee_status.m2h_irq = MD32_TO_HOST_REG;
        md32_irq->dmem_disp_int = 0;
        reboot = 1;
    }

    if(md32_irq->md32_ipc_int)
    {
        md32_ipi_handler();
        md32_irq->ipc_md2host = 0;
        md32_irq->md32_ipc_int = 0;
    }

    MD32_TO_HOST_REG = 0x0;

    if(reboot)
    {
        queue_work(wq_md32_reboot, (struct work_struct *)&work_md32_reboot);
    }

    return IRQ_HANDLED;
}

void md32_reboot_from_irq(struct work_struct * ws)
{
    struct md32_reboot_work *rb_ws = (struct md32_reboot_work *)ws;
    struct md32_aed_cfg *aed = &rb_ws->aed;
    struct reg_md32_to_host_ipc *md32_irq = (struct reg_md32_to_host_ipc *)&md32_aee_status.m2h_irq;

    if(md32_irq->wdt_int)
        md32_prepare_aed("md32 wdt", aed);
    else if(md32_irq->pmem_disp_int)
        md32_prepare_aed("md32 pmem_abort", aed);
    else if(md32_irq->dmem_disp_int)
        md32_prepare_aed("md32 dmem_abort", aed);
    else
        md32_prepare_aed("md32 exception", aed);

    printk("%s", aed->detail);

    aed_md32_exception_api(aed->log, aed->log_size, aed->phy, aed->phy_size, aed->detail, DB_OPT_DEFAULT);

    if(aed->detail)
        kfree(aed->detail);
    if(aed->phy)
        kfree(aed->phy);
    if(aed->log)
        kfree(aed->log);

    printk("[MD32] md32 exception dump is done\n");

    //aee_kernel_exception("MD32","MD32 EXCEPTION ");

#if 0
    if(reboot_load_md32() < 0)
    {
        printk("Reboot MD32 Fail\n");
    }
#endif
}

void md32_irq_init(void)
{
    MD32_TO_HOST_REG = 0;  //clear md32 irq

#ifdef MD32_WDT_ASSERT_REGIST
    // Reset Structures
    memset(&MD32_WDT_FUN,0,sizeof(MD32_WDT_FUN));
    memset(&MD32_ASSERT_FUN,0,sizeof(MD32_ASSERT_FUN));
#endif

    wq_md32_reboot = create_workqueue("MD32_REBOOT_WQ");
#ifdef MD32_WDT_ASSERT_REGIST
    wq_md32_wdt    = create_workqueue("MD32_WDT_WQ");
    wq_md32_assert = create_workqueue("MD32_ASSERT_WQ");
#endif

    INIT_WORK((struct work_struct *)&work_md32_reboot,md32_reboot_from_irq);
#ifdef MD32_WDT_ASSERT_REGIST
    INIT_WORK(&work_md32_wdt,md32_wdt_reset_func);
    INIT_WORK(&work_md32_assert,md32_assert_reset_func);
#endif

    //request_irq(md32reg->irq, md32_irq_handler, IRQF_TRIGGER_HIGH, "MD32 IPC_MD2HOST", NULL);
}
