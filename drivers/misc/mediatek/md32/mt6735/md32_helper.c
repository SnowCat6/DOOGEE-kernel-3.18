#include <linux/module.h>       /* needed by all modules */
#include <linux/init.h>         /* needed by module macros */
#include <linux/fs.h>           /* needed by file_operations* */
#include <linux/miscdevice.h>   /* needed by miscdevice* */
#include <linux/device.h>       /* needed by device_* */
#include <linux/vmalloc.h>      /* needed by kmalloc */
#include <linux/uaccess.h>      /* needed by copy_to_user */
#include <linux/fs.h>           /* needed by file_operations* */
#include <linux/slab.h>         /* needed by kmalloc */
#include <linux/poll.h>         /* needed by poll */
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/io.h>

#include <mach/hardware.h>
#include <mach/mt_reg_base.h>
#include <mach/sync_write.h>
#include <mach/mt_spm_sleep.h>

#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#include <mach/mt_clkmgr.h>


#define TIMEOUT 5000
//#define LOG_TO_AP_UART


extern  irqreturn_t md32_irq_handler(int irq, void *dev_id);


struct md32_regs md32reg;

unsigned int md32_init_done;

unsigned char *md32_data_image;
unsigned char *md32_program_image;
unsigned char *md32_send_buff;
unsigned char *md32_recv_buff;

unsigned int md32_log_buf_addr;
unsigned int md32_log_start_idx_addr;
unsigned int md32_log_end_idx_addr;
unsigned int md32_log_lock_addr;
unsigned int md32_log_buf_len_addr;
unsigned int enable_md32_mobile_log_addr;
unsigned int md32_mobile_log_ready = 0;
unsigned int md32_mobile_log_ipi_check = 0;

unsigned char *g_md32_log_buf = NULL;
unsigned char *g_md32_log_buf_2 = NULL;
unsigned int last_log_buf_max_len = 0;
unsigned int md32_log_read_count = 0;
unsigned int buff_full_count = 0;
unsigned int last_buff_full_count = 0;
struct mutex md32_log_mutex;
static wait_queue_head_t logwait;

MD32_GROUP_INFO g_group_info;
int g_current_group = GROUP_BASIC;
unsigned char **swap_buf;

void memcpy_to_md32 (void __iomem *trg, const void *src, int size)
{
    int i;
    u32 __iomem *t = trg;
    const u32 *s = src;

    for (i = 0; i < ((size + 3) >> 2); i++)
        *t++ = *s++;
}

void memcpy_from_md32 (void *trg, const void __iomem *src, int size)
{
    int i;
    u32 *t = trg;
    const u32 __iomem *s = src;

    for (i = 0; i < ((size + 3) >> 2); i++)
        *t++ = *s++;
}

int get_md32_semaphore(int flag)
{
    int read_back;
    int count = 0;
    int ret = -1;

    flag = (flag * 2) + 1;

    writel((1 << flag), MD32_SEMAPHORE);

    while(count != TIMEOUT)
    {
        /* repeat test if we get semaphore */
        read_back = (readl(MD32_SEMAPHORE) >> flag) & 0x1;
        if(read_back == 1)
        {
            ret = 1;
            break;
        }
        writel((1 << flag), MD32_SEMAPHORE);
        count++;
    }

    if(ret < 0)
        printk("[MD32] get md32 semaphore %d TIMEOUT...!\n", flag);

    return ret;
}

int release_md32_semaphore(int flag)
{
    int read_back;
    int ret = -1;

    flag = (flag * 2) + 1;

    /* Write 1 clear */
    writel((1 << flag), MD32_SEMAPHORE);
    read_back = (readl(MD32_SEMAPHORE) >> flag) & 0x1;
    if(read_back == 0)
        ret = 1;
    else
        printk("[MD32] release md32 semaphore %d failed!\n", flag);

    return ret;
}

static BLOCKING_NOTIFIER_HEAD(md32_notifier_list);

void md32_register_notify (struct notifier_block *nb)
{
    blocking_notifier_chain_register(&md32_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(md32_register_notify);

void md32_unregister_notify(struct notifier_block *nb)
{
    blocking_notifier_chain_unregister(&md32_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(md32_unregister_notify);

static int md32_helper_notify(struct notifier_block *self, unsigned long action, void *dev)
{
    MD32_REQUEST_SWAP *request_swap = (MD32_REQUEST_SWAP*)dev;
    int ret;

    switch (action) {
#ifdef DYNAMIC_TCM_SWAP
    case MD32_SELF_TRIGGER_TCM_SWAP:
        ret = md32_tcm_swap(request_swap->group_start);
        if(ret < 0)
            printk("md32_helper_notify swap tcm failed\n");
        break;
    case APP_TRIGGER_TCM_SWAP_START:
        /* do nothing, the following is only for demo and testing */
        if(request_swap->prepare_result < 0)
        {
            printk("MD32 dynamic swap prepare failed\n");
            break;
        }
        //if(request_swap->current_group == GROUP_BASIC && request_swap->group_start == GROUP_A)
        //{
        /* Group leader need to call swap funtion */
        ret = md32_tcm_swap(request_swap->group_start);
        if(ret < 0)
            blocking_notifier_call_chain(&md32_notifier_list, APP_TRIGGER_TCM_SWAP_FAIL, request_swap);
        else
            blocking_notifier_call_chain(&md32_notifier_list, APP_TRIGGER_TCM_SWAP_DONE, request_swap);
        //}
        break;
    case APP_TRIGGER_TCM_SWAP_DONE:
        /* do nothing */
        break;
    case APP_TRIGGER_TCM_SWAP_FAIL:
        /* do nothing */
        break;
    case APP_TRIGGER_APP_FINISHED:
        /* do nothing */
        break;
#endif
    default:
        printk("[MD32] helper get unkown action %ld\n", action);
        break;
    }
    return NOTIFY_OK;
}

static struct notifier_block md32_helper_nb = {
    .notifier_call =        md32_helper_notify,
};

ssize_t md32_get_log_buf(unsigned char *md32_log_buf, size_t b_len)
{
    ssize_t i = 0;
    unsigned int log_start_idx;
    unsigned int log_end_idx;
    unsigned int log_buf_max_len;
    unsigned char *__log_buf = (unsigned char *)(MD32_DTCM + md32_log_buf_addr);

    //mutex_lock(&md32_log_mutex); //mark for get log buf in irq

    log_start_idx = readl((void __iomem *)(MD32_DTCM + md32_log_start_idx_addr));
    log_end_idx = readl((void __iomem *)(MD32_DTCM + md32_log_end_idx_addr));
    log_buf_max_len = readl((void __iomem *)(MD32_DTCM + md32_log_buf_len_addr));

    //printk("[MD32] log_start_idx = %d\n", log_start_idx);
    //printk("[MD32] log_end_idx = %d\n", log_end_idx);
    //printk("[MD32] log_buf_max_len = %d\n", log_buf_max_len);

    if(!md32_log_buf)
    {
        printk("[MD32] input null buffer\n");
        goto out;
    }

    if(b_len > log_buf_max_len)
    {
        printk("[MD32] b_len %zu > log_buf_max_len %d\n", b_len, log_buf_max_len);
        b_len = log_buf_max_len;
    }

#define LOG_BUF_MASK (log_buf_max_len-1)
#define LOG_BUF(idx) (__log_buf[(idx) & LOG_BUF_MASK])

    /* Read MD32 log */
    /* Lock the log buffer */
    mt_reg_sync_writel(0x1, (MD32_DTCM + md32_log_lock_addr));

    i = 0;
    while( (log_start_idx != log_end_idx) && i < b_len)
    {
        md32_log_buf[i] = LOG_BUF(log_start_idx);
        log_start_idx++;
        i++;
    }
    mt_reg_sync_writel(log_start_idx, (MD32_DTCM + md32_log_start_idx_addr));

    /* Unlock the log buffer */
    mt_reg_sync_writel(0x0, (MD32_DTCM + md32_log_lock_addr));

out:
    //mutex_unlock(&md32_log_mutex);

    return i;
}

int get_current_group(void)
{
    return g_current_group;
}

static int md32_log_open(struct inode *inode, struct file *file)
{
    printk("[MD32] md32_log_open\n");
	return nonseekable_open(inode, file);
}

static ssize_t md32_log_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
    int i;
    int ret_len;
    unsigned long copy_size;
    unsigned int log_buf_max_len;

    //printk("[MD32] Enter md32_log_read, len = %d\n", len);

#ifndef LOG_TO_AP_UART
    log_buf_max_len = readl((void __iomem *)(MD32_DTCM + md32_log_buf_len_addr));

    if(g_md32_log_buf == NULL || log_buf_max_len != last_log_buf_max_len)
    {
        g_md32_log_buf = (unsigned char*)krealloc(g_md32_log_buf, log_buf_max_len+1, GFP_KERNEL);

        last_log_buf_max_len = log_buf_max_len;

        if(!g_md32_log_buf)
        {
            printk("[MD32] %s, kmalloc fail, len=0x%08x\n", __func__, log_buf_max_len);
            return 0;
        }
    }

    ret_len = md32_get_log_buf(g_md32_log_buf, len);

    if(ret_len)
    {
    	g_md32_log_buf[ret_len] = '\0';
        //printk("[MD32 LOG] %s", g_md32_log_buf);
    }
    else
    {
        strcpy(g_md32_log_buf, " ");
        ret_len = strlen(g_md32_log_buf);
        //printk("[MD32 LOG] %s", g_md32_log_buf);
    }

    copy_size = copy_to_user((unsigned char*)data, (unsigned char *)g_md32_log_buf, ret_len);
#else
    ret_len = 0;
#endif

    //printk("[MD32] Exit md32_log_read, ret_len = %d, count = %d\n", ret_len, ++md32_log_read_count);

    return ret_len;
}

static unsigned int md32_poll(struct file *file, poll_table *wait)
{
    unsigned int log_start_idx;
    unsigned int log_end_idx;
    unsigned int ret = 0;  //POLLOUT | POLLWRNORM;

    //printk("[MD32] Enter md32_poll\n");

    if (!(file->f_mode & FMODE_READ)) {
        //printk("[MD32] Exit1 md32_poll, ret = %d\n", ret);
        return ret;
    }

    poll_wait(file, &logwait, wait);

    if(!md32_mobile_log_ipi_check && md32_mobile_log_ready)
    {
        log_start_idx = readl((void __iomem *)(MD32_DTCM + md32_log_start_idx_addr));
        log_end_idx = readl((void __iomem *)(MD32_DTCM + md32_log_end_idx_addr));
        if(log_start_idx != log_end_idx)
            ret |= (POLLIN | POLLRDNORM);
    }
    else if(last_buff_full_count != buff_full_count)
    {
        last_buff_full_count++;
        log_start_idx = readl((void __iomem *)(MD32_DTCM + md32_log_start_idx_addr));
        log_end_idx = readl((void __iomem *)(MD32_DTCM + md32_log_end_idx_addr));
        if(log_start_idx != log_end_idx)
            ret |= (POLLIN | POLLRDNORM);
    }

    //printk("[MD32] Exit2 md32_poll, ret = %d\n", ret);

    return ret;
}

static struct file_operations md32_file_ops = {
    .owner = THIS_MODULE,
    .read = md32_log_read,
    .open = md32_log_open,
    .poll = md32_poll,
};

static struct miscdevice md32_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "md32",
	.fops = &md32_file_ops
};

void logger_ipi_handler(int id, void *data, unsigned int len)
{
    MD32_LOG_INFO *log_info = (MD32_LOG_INFO*)data;

    md32_log_buf_addr = log_info->md32_log_buf_addr;
    md32_log_start_idx_addr = log_info->md32_log_start_idx_addr;
    md32_log_end_idx_addr = log_info->md32_log_end_idx_addr;
    md32_log_lock_addr = log_info->md32_log_lock_addr;
    md32_log_buf_len_addr = log_info->md32_log_buf_len_addr;
    enable_md32_mobile_log_addr =log_info->enable_md32_mobile_log_addr;

    md32_mobile_log_ready = 1;

    printk("[MD32] md32_log_buf_addr = 0x%x\n", md32_log_buf_addr);
    printk("[MD32] md32_log_start_idx_addr = 0x%x\n", md32_log_start_idx_addr);
    printk("[MD32] md32_log_end_idx_addr = 0x%x\n", md32_log_end_idx_addr);
    printk("[MD32] md32_log_lock_addr = 0x%x\n", md32_log_lock_addr);
    printk("[MD32] md32_log_buf_len_addr = 0x%x\n", md32_log_buf_len_addr);
    printk("[MD32] enable_md32_mobile_log_addr = 0x%x\n", enable_md32_mobile_log_addr);
}

void buf_full_ipi_handler(int id, void *data, unsigned int len)
{
    //printk("[MD32] Enter buf_full_ipi_handler\n");

#ifndef LOG_TO_AP_UART
    buff_full_count += 8;
    wake_up(&logwait);
#else
    int ret_len;
    unsigned int log_buf_max_len;
    int print_len, idx = 0;

    log_buf_max_len = readl((void __iomem *)(MD32_DTCM + md32_log_buf_len_addr));

    if(g_md32_log_buf == NULL || log_buf_max_len != last_log_buf_max_len)
    {
        g_md32_log_buf = (unsigned char*)krealloc(g_md32_log_buf, log_buf_max_len+1, GFP_KERNEL);

        last_log_buf_max_len = log_buf_max_len;

        if(!g_md32_log_buf)
        {
            printk("[MD32] %s, kmalloc fail, len=0x%08x\n", __func__, log_buf_max_len);
            return;
        }
    }

    if(g_md32_log_buf_2 == NULL)
    {
        g_md32_log_buf_2 = (unsigned char*)kmalloc(512+1, GFP_KERNEL);

        if(!g_md32_log_buf_2)
        {
            printk("[MD32] %s, kmalloc fail, len=0x%08x\n", __func__, 512+1);
            return;
        }
    }
    
    ret_len = md32_get_log_buf(g_md32_log_buf, log_buf_max_len);

    while(ret_len > 0)
    {
        print_len = (ret_len < 512) ? ret_len : 512;
    	memcpy(g_md32_log_buf_2, g_md32_log_buf+idx, print_len);
        g_md32_log_buf_2[print_len] = '\0';
        printk("[MD32 LOG] %s", g_md32_log_buf_2);
        idx += print_len;
        ret_len -= print_len;
    }
#endif

    //printk("[MD32] Exit buf_full_ipi_handler\n");
}

void apdma_ipi_handler(int id, void *data, unsigned int len)
{
    unsigned int en;
    
    printk("[MD32] apdma_ipi_handler\n");

    en = *(unsigned int *)data;

    if (en == 1) {
    	enable_clock(MT_CG_INFRA_APDMA, "apdma");
    	printk("[MD32] enable APDMA clock = %d\n", en);
    } else {
    	disable_clock(MT_CG_INFRA_APDMA, "apdma");
    	printk("[MD32] disable APDMA clock = %d\n", en);
    }
}

int get_md32_img_sz(const char *IMAGE_PATH)
{
    int ret = 0;
    struct file *filp = NULL;
    const int size_per_read = 16 * 1024;
    struct inode *inode;
    off_t fsize = 0;

    filp = filp_open(IMAGE_PATH, O_RDONLY, 0644);

    if(IS_ERR(filp))
    {
        printk("[MD32] Open MD32 image %s FAIL!\n", IMAGE_PATH);
        return -1;
    }
    else
    {
        inode=filp->f_dentry->d_inode;
        fsize=inode->i_size;
    }

    filp_close(filp, NULL);
    return fsize;
}

#ifdef DYNAMIC_TCM_SWAP
/* @group_id: the group want to swap in tcm and run. */
void md32_prepare_swap(int group_id)
{
    MD32_PREPARE_SWAP prepare_swap;

    /* Prepare object for MD32 */
    prepare_swap.info_type = INFO_PREPARE_SWAP;
    prepare_swap.group_start = group_id;

    md32_ipi_send(IPI_SWAP, (void *)&prepare_swap, sizeof(MD32_PREPARE_SWAP), 0);
}

/* @group_id: the group want to swap in tcm and run. */
int md32_tcm_swap(int group_id)
{
    int ret = 0;
    unsigned int swap_lock;
    unsigned int app_program_start_addr;
    unsigned int app_program_end_addr;
    unsigned int app_program_sz;
    unsigned char *p_buf;
    int p_sz;

    unsigned int app_data_start_addr;
    unsigned int app_data_end_addr;
    unsigned int app_data_sz;
    unsigned char *d_buf;
    //int d_sz;

    unsigned int swap_out_data_start_addr;
    unsigned int swap_out_data_end_addr;
    unsigned int swap_out_data_sz;

    /* Check swap_lock is set by md32 */
    swap_lock = readl(MD32_DTCM + g_group_info.swap_lock_addr);

    if(swap_lock == 1)
    {
        app_program_start_addr = readl(MD32_DTCM + g_group_info.app_ptr_p + sizeof(unsigned int)*group_id*2);
        app_program_end_addr = readl(MD32_DTCM + g_group_info.app_ptr_p + (sizeof(unsigned int)*group_id)*2 + sizeof(unsigned int));
        app_program_sz = app_program_end_addr - app_program_start_addr + 1;

        app_data_start_addr = readl(MD32_DTCM + g_group_info.app_ptr_d + sizeof(unsigned int)*group_id*2);
        app_data_end_addr = readl(MD32_DTCM + g_group_info.app_ptr_d + (sizeof(unsigned int)*group_id*2) + sizeof(unsigned int));
        app_data_sz = app_data_end_addr - app_data_start_addr + 1;

        swap_out_data_start_addr = readl(MD32_DTCM + g_group_info.app_ptr_d + sizeof(unsigned int)*g_current_group*2);
        swap_out_data_end_addr = readl(MD32_DTCM + g_group_info.app_ptr_d + sizeof(unsigned int)*g_current_group*2 + sizeof(unsigned int));
        swap_out_data_sz = swap_out_data_end_addr - swap_out_data_start_addr + 1;

        /* swap out stop-app data into swap_buf */
        memcpy(swap_buf[g_current_group], MD32_DTCM + g_group_info.app_d_area, swap_out_data_sz);

        //p_sz = get_md32_img_sz(MD32_PROGRAM_IMAGE_PATH);
        //d_sz = get_md32_img_sz(MD32_DATA_IMAGE_PATH);

        //p_buf = (unsigned char*)kmalloc((size_t)p_sz+1 , GFP_KERNEL);
        //d_buf = (unsigned char*)kmalloc((size_t)d_sz+1 , GFP_KERNEL);

        /* swap in start-app program/data into tcm */
        //ret = load_md32(MD32_PROGRAM_IMAGE_PATH, p_buf);
        //ret = load_md32(MD32_DATA_IMAGE_PATH, d_buf);

        memcpy((MD32_PTCM + g_group_info.app_p_area), (md32_program_image + app_program_start_addr), app_program_sz);
        memcpy((MD32_DTCM + g_group_info.app_d_area), swap_buf[group_id], app_data_sz);

        /* swap done. clear the swap_lock */
        mt_reg_sync_writel(0x0, (MD32_DTCM + g_group_info.swap_lock_addr));
        g_current_group = group_id;
    }
    else
    {
        ret = -1;
    }

    return ret;
}

void swap_ipi_handler(int id, void *data, unsigned int len)
{
    int info_type;
    int i;
    MD32_GROUP_INFO *group_info;
    MD32_REQUEST_SWAP request_swap;
    MD32_REQUEST_SWAP self_trigger_swap;
    unsigned int app_data_start_addr;
    unsigned int app_data_end_addr;
    unsigned int app_data_sz;

    unsigned int app_program_end_addr;
    //unsigned char *d_buf;
    int d_sz;
    int ret;

    printk("[MD32] Enter swap ipi...\n");

    info_type = ((int *)data)[0];

    printk("info_type = %d\n", info_type);

    /* by info_type to select structure we receive */
    if(info_type == INFO_GROUP)
    {
        group_info = (MD32_GROUP_INFO*)data;
        g_group_info.info_type = group_info->info_type;
        g_group_info.app_ptr_d = group_info->app_ptr_d;
        g_group_info.app_ptr_p = group_info->app_ptr_p;
        g_group_info.swap_lock_addr = group_info->swap_lock_addr;
        g_group_info.group_num = group_info->group_num;
        g_group_info.app_d_area = group_info->app_d_area;
        g_group_info.app_p_area = group_info->app_p_area;

        /* Clear other apps in TCM, only leave GROUP_BASIC in TCM */
        app_program_end_addr = readl(MD32_DTCM + g_group_info.app_ptr_p + (sizeof(unsigned int)*GROUP_BASIC)*2 + sizeof(unsigned int));
        memset(MD32_PTCM + app_program_end_addr, 0, MD32_PTCM_SIZE - app_program_end_addr);
        app_data_end_addr = readl(MD32_DTCM + g_group_info.app_ptr_d + (sizeof(unsigned int)*GROUP_BASIC*2) + sizeof(unsigned int));
        memset(MD32_DTCM + app_data_end_addr, 0, MD32_DTCM_SIZE - app_data_end_addr);

        /* Prepare Swap buffer for Data restore */
        swap_buf = (unsigned char**)kmalloc((g_group_info.group_num)*4, GFP_ATOMIC);
        for(i=0; i<g_group_info.group_num; i++)
        {
            app_data_start_addr = readl(MD32_DTCM + g_group_info.app_ptr_d + sizeof(unsigned int)*i*2);
            app_data_end_addr = readl(MD32_DTCM + g_group_info.app_ptr_d + (sizeof(unsigned int)*i*2) + sizeof(unsigned int));
            app_data_sz = app_data_end_addr - app_data_start_addr + 1;
            swap_buf[i] = (unsigned char*)krealloc(swap_buf[i],app_data_sz, GFP_ATOMIC);
            memcpy(swap_buf[i], md32_data_image+app_data_start_addr, app_data_sz);
            printk("[swap_ipi_handler] group %d, app_data_start_addr = 0x%x\n", i, app_data_start_addr);
            printk("[swap_ipi_handler] group %d, app_data_stop_addr = 0x%x\n", i, app_data_end_addr);
        }

        printk("info_type = %d\n", g_group_info.info_type);
        printk("group_num = %d\n", g_group_info.group_num);
        printk("app_ptr_d = 0x%x\n", g_group_info.app_ptr_d);
        printk("app_ptr_p = 0x%x\n", g_group_info.app_ptr_p);
        printk("swap_lock_addr = 0x%x\n", g_group_info.swap_lock_addr);
    }
    else if(info_type == INFO_REQUEST_SWAP)
    {
        //request_swap = (MD32_REQUEST_SWAP*)data;
        memcpy(&request_swap, data, sizeof(MD32_REQUEST_SWAP));
        if(request_swap.prepare_result == 0) //prepare ready.
        {
            g_current_group = request_swap.current_group;
        }
        /* trigger notifier */
        blocking_notifier_call_chain(&md32_notifier_list, APP_TRIGGER_TCM_SWAP_START, &request_swap);
    }
    else if(info_type == INFO_SELF_TRIGGER_SWAP)
    {
        //self_trigger_swap = (MD32_SELF_TRIGGER_SWAP *)data;
        memcpy(&self_trigger_swap, data, sizeof(MD32_REQUEST_SWAP));
        if(self_trigger_swap.prepare_result == 0) //prepare ready.
        {
            g_current_group = self_trigger_swap.current_group;
        }
        blocking_notifier_call_chain(&md32_notifier_list, MD32_SELF_TRIGGER_TCM_SWAP, &self_trigger_swap);
    }
    else
    {
        printk("[MD32 swap IPI] unknown info type %d !\n", info_type);
    }
}
#endif

int load_md32(const char *IMAGE_PATH, void *dst)
{
    int ret = 0;
    struct file *filp = NULL;
    const int size_per_read = 16 * 1024;
    unsigned char *buf = NULL;
    unsigned char *ptr;
    struct inode *inode;
    off_t fsize;
    mm_segment_t fs;

    ptr = buf;

    filp = filp_open(IMAGE_PATH, O_RDONLY, 0644);

    if(IS_ERR(filp))
    {
        printk("[MD32] Open MD32 image %s FAIL!\n", IMAGE_PATH);
        goto error;
    }
    else
    {
        inode=filp->f_dentry->d_inode;
        fsize=inode->i_size;
        printk("[MD32] file %s size: %i \n", IMAGE_PATH, (int)fsize);
        buf = (unsigned char*)kmalloc((size_t)fsize+1 , GFP_KERNEL);
        fs=get_fs();
        set_fs(KERNEL_DS);
        filp->f_op->read(filp,buf,fsize,&(filp->f_pos));
        set_fs(fs);
        buf[fsize]='\0';
        //printk("<1>The File Content is:\n");
        //printk("<1>%s",buf);
        memcpy(dst, buf, fsize);
    }

    filp_close(filp, NULL);
    kfree(buf);
    return fsize;

error:
    if(filp != NULL)
        filp_close(filp, NULL);

    if(buf)
        kfree(buf);
    return -1;
}

unsigned int is_md32_enable(void)
{
    if (md32_init_done && readl(MD32_BASE))
        return 1;
    else
        return 0;
}

void boot_up_md32(void)
{
    mt_reg_sync_writel(0x1, MD32_BASE);
}

void reset_md32(void)
{
    unsigned int sw_rstn;

    sw_rstn = readl(MD32_BASE);
    if(sw_rstn == 0x0)
        printk("[MD32] MD32 has already been reseted!\n");
    else
        mt_reg_sync_writel(0x0, MD32_BASE);
}

static inline ssize_t md32_log_len_show(struct device *kobj, struct device_attribute *attr, char *buf)
{
    int log_legnth = 0;
    //unsigned int log_start_idx;
    //unsigned int log_end_idx;
    unsigned int log_buf_max_len;

    if(md32_mobile_log_ready)
    {
        //log_start_idx = readl(MD32_DTCM + md32_log_start_idx_addr);
        //log_end_idx = readl(MD32_DTCM + md32_log_end_idx_addr);
        log_buf_max_len = readl(MD32_DTCM + md32_log_buf_len_addr);

        log_legnth = log_buf_max_len;
    }

    return sprintf(buf, "%08x\n", log_legnth);
}

static ssize_t md32_log_len_store(struct device *kobj, struct device_attribute *attr,	const char *buf, size_t n)
{
    /*do nothing*/
    return n;
}

DEVICE_ATTR(md32_log_len, 0644, md32_log_len_show, md32_log_len_store);

int reboot_load_md32(void)
{
    int i;
    unsigned int sw_rstn;
    int ret = 0;
    int d_sz, p_sz;
    volatile unsigned int reg;

    sw_rstn = readl(MD32_BASE);

    if(sw_rstn == 0x1)
    {
        printk("[MD32] MD32 is already running, reboot now...\n");
    }

    /* reset MD32 */
    mt_reg_sync_writel(0x0, MD32_BASE);

    mt_reg_sync_writel(0x1FF, MD32_CLK_CTRL_BASE+0x030);

    mt_reg_sync_writel(0x1, MD32_BASE+0x008);

    reg = readl(MD32_CLK_CTRL_BASE+0x02C);
    for (i = 0; i < 7; ++i){
        reg &= ~(0x1 << i);
        mt_reg_sync_writel(reg, MD32_CLK_CTRL_BASE+0x02C);
    }

    d_sz = get_md32_img_sz(MD32_DATA_IMAGE_PATH);
    if(d_sz < 0)
    {
        printk("[MD32] MD32 boot up failed --> can not get data image size\n");
        ret = -1;
        goto error;
    }
    d_sz = ((d_sz + 63) >> 6) << 6;

    md32_data_image = (unsigned char*)krealloc(md32_data_image,(size_t)d_sz+1 , GFP_KERNEL);
    if(!md32_data_image)
    {
        printk("[MD32] MD32 boot up failed --> can't allocate memory\n");
        ret = -1;
        goto error;
    }

    ret = load_md32(MD32_DATA_IMAGE_PATH, md32_data_image);
    if(ret < 0)
    {
        printk("[MD32] MD32 boot up failed --> load data image failed!\n");
        ret = -1;
        goto error;
    }

    if(d_sz > MD32_DTCM_SIZE)
    {
        d_sz = MD32_DTCM_SIZE;
    }
    memcpy(MD32_DTCM, md32_data_image, d_sz);

    p_sz = get_md32_img_sz(MD32_PROGRAM_IMAGE_PATH);
    if(p_sz < 0)
    {
        printk("[MD32] MD32 boot up failed --> can not get program image size\n");
        ret = -1;
        goto error;
    }
    p_sz = ((p_sz + 63) >> 6) << 6;

    md32_program_image = (unsigned char*)krealloc(md32_program_image, (size_t)p_sz+1 , GFP_KERNEL);
    if(!md32_program_image)
    {
        printk("[MD32] MD32 boot up failed --> can't allocate memory\n");
        ret = -1;
        goto error;
    }

    ret = load_md32(MD32_PROGRAM_IMAGE_PATH, md32_program_image);
    if(ret < 0)
    {
        printk("[MD32] MD32 boot up failed --> load program image failed!\n");
        ret = -1;
        goto error;
    }

    if(p_sz > MD32_PTCM_SIZE)
    {
        p_sz = MD32_PTCM_SIZE;
    }
    memcpy(MD32_PTCM, md32_program_image, p_sz);

#if 0
#ifdef DYNAMIC_TCM_SWAP
    unsigned int app_program_start_addr;
    unsigned int app_program_end_addr;
    unsigned int app_program_sz;

    unsigned int app_data_start_addr;
    unsigned int app_data_end_addr;
    unsigned int app_data_sz;

    int d_sz;
    int p_sz;

    g_current_group = GROUP_BASIC;

    /* Only load Basic Group into TCM at beging */
    app_program_start_addr = readl(MD32_DTCM + g_group_info.app_ptr_p + sizeof(unsigned int)*GROUP_BASIC*2);
    app_program_end_addr = readl(MD32_DTCM + g_group_info.app_ptr_p + (sizeof(unsigned int)*GROUP_BASIC)*2 + sizeof(unsigned int));
    app_program_sz = app_program_end_addr - app_program_start_addr + 1;

    app_data_start_addr = readl(MD32_DTCM + g_group_info.app_ptr_d + sizeof(unsigned int)*GROUP_BASIC*2);
    app_data_end_addr = readl(MD32_DTCM + g_group_info.app_ptr_d + (sizeof(unsigned int)*GROUP_BASIC*2) + sizeof(unsigned int));
    app_data_sz = app_data_end_addr - app_data_start_addr + 1;

    /* Initialize swap buffer & copy each app data into it*/
    d_sz = get_md32_img_sz(MD32_DATA_IMAGE_PATH);
    md32_data_image = (unsigned char*)kmalloc((size_t)d_sz+1 , GFP_KERNEL);
    ret = load_md32(MD32_DATA_IMAGE_PATH, md32_data_image);
    if(ret < 0)
    {
        printk("MD32 boot up failed --> load data image failed!\n");
        return;
    }
    memcpy(MD32_DTCM, md32_data_image, (app_data_sz + g_group_info.app_d_area));

    p_sz = get_md32_img_sz(MD32_PROGRAM_IMAGE_PATH);
    md32_program_image = (unsigned char*)kmalloc((size_t)p_sz+1 , GFP_KERNEL);
    ret = load_md32(MD32_PROGRAM_IMAGE_PATH, md32_program_image);
    if(ret < 0)
    {
        printk("MD32 boot up failed --> load program image failed!\n");
        return;
    }
    memcpy(MD32_PTCM, md32_program_image, (app_program_sz + g_group_info.app_p_area));
#else
    d_sz = load_md32(MD32_DATA_IMAGE_PATH, MD32_DTCM);
    if(d_sz < 0)
    {
        printk("MD32 boot up failed --> load data image failed!\n");
        return n;
    }
    ret = load_md32(MD32_PROGRAM_IMAGE_PATH, MD32_PTCM);
    if(ret < 0)
    {
        printk("MD32 boot up failed --> load program image failed!\n");
        return n;
    }
#endif
#endif

    boot_up_md32();
    return ret;

error:
    printk("[MD32] boot up failed!!! free images\n");
    if(md32_data_image)
        kfree(md32_data_image);
    if(md32_program_image)
        kfree(md32_program_image);

    return ret;
}

static inline ssize_t md32_boot_show(struct device *kobj, struct device_attribute *attr, char *buf)
{
    unsigned int sw_rstn;
    sw_rstn = readl(MD32_BASE);

    if(sw_rstn == 0x0)
    {
        return sprintf(buf, "MD32 not enabled\n");
    }
    else
    {
        return sprintf(buf, "MD32 is running...\n");
    }
}

static ssize_t md32_boot_store(struct device *kobj, struct device_attribute *attr,	const char *buf, size_t n)
{
    unsigned int enable = 0;
    if (sscanf(buf, "%d", &enable) != 1)
        return -EINVAL;

    if(reboot_load_md32() < 0)
        return -EINVAL;

    return n;
}

DEVICE_ATTR(md32_boot, 0644, md32_boot_show, md32_boot_store);

#ifdef DYNAMIC_TCM_SWAP
static inline ssize_t md32_swap_show(struct device *kobj, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Current running group is %d\n", g_current_group);
}

static ssize_t md32_swap_store(struct device *kobj, struct device_attribute *attr,	const char *buf, size_t n)
{
    //MD32_PREPARE_SWAP prepare_swap;
    int group_start;

    if (sscanf(buf, "%d", &group_start) != 1)
        return -EINVAL;

    /* Prepare object for MD32 */
    md32_prepare_swap(group_start);
    //prepare_swap.info_type = INFO_PREPARE_SWAP;
    //prepare_swap.group_start = group_start;

    //md32_ipi_send(IPI_SWAP, (void *)&prepare_swap, sizeof(MD32_PREPARE_SWAP), 0);

    return n;
}


DEVICE_ATTR(md32_swap, 0644, md32_swap_show, md32_swap_store);
#endif

static inline ssize_t md32_mobile_log_show(struct device *kobj, struct device_attribute *attr, char *buf)
{
    unsigned int enable_md32_mobile_log = 0;

    if(md32_mobile_log_ready)
    {
        enable_md32_mobile_log = readl((void __iomem *)(MD32_DTCM + enable_md32_mobile_log_addr));
    }

    if(enable_md32_mobile_log == 0x0)
        return sprintf(buf, "MD32 mobile log is disabled\n");
    else if(enable_md32_mobile_log == 0x1)
        return sprintf(buf, "MD32 mobile log is enabled\n");
    else
        return sprintf(buf, "MD32 mobile log is in unknown state...\n");
}

static ssize_t md32_mobile_log_store(struct device *kobj, struct device_attribute *attr,	const char *buf, size_t n)
{
    unsigned int enable;
    if (sscanf(buf, "%d", &enable) != 1)
        return -EINVAL;

    if(md32_mobile_log_ready)
    {
        mt_reg_sync_writel(enable, (MD32_DTCM + enable_md32_mobile_log_addr));
    }    

    return n;
}

DEVICE_ATTR(md32_mobile_log, 0644, md32_mobile_log_show, md32_mobile_log_store);

static int create_files(void)
{
    int ret = device_create_file(md32_device.this_device, &dev_attr_md32_log_len);
    if (unlikely(ret != 0))
        return ret;

    ret = device_create_file(md32_device.this_device, &dev_attr_md32_boot);
    if (unlikely(ret != 0))
        return ret;

    ret = device_create_file(md32_device.this_device, &dev_attr_md32_mobile_log);
    if (unlikely(ret != 0))
        return ret;

    ret = device_create_file(md32_device.this_device, &dev_attr_md32_ocd);
    if (unlikely(ret != 0))
        return ret;

#ifdef DYNAMIC_TCM_SWAP
    ret = device_create_file(md32_device.this_device, &dev_attr_md32_swap);
    if (unlikely(ret != 0))
        return ret;
#endif

    return 0;
}

int md32_dt_init(void)
{
    int ret = 0;
    struct md32_regs *mreg= &md32reg;
    struct device_node *node = NULL;

    node = of_find_compatible_node(NULL, NULL, "mediatek,SCP_SRAM");
    if(!node)
    {
        printk("[MD32] Can't find node: mediatek,SCP_SRAM\n");
        return -1;
    }

    mreg->sram = of_iomap(node, 0);
    printk("[MD32] md32 sram base=0x%p\n", mreg->sram);

    if (!mreg->sram) {
        printk("[MD32] Unable to ioremap mregisters\n");
        return -1;
    }

    node = of_find_compatible_node(NULL, NULL, "mediatek,SCP_CFGREG");
    if(!node)
    {
        printk("[MD32] Can't find node: mediatek,SCP_CFGRE\n");
    }

    mreg->cfg = of_iomap(node, 0);
    mreg->irq = irq_of_parse_and_map(node, 0);
    printk("[MD32] md32 irq=%d, cfgreg=0x%p\n", mreg->irq, mreg->cfg);

    if (!mreg->cfg) {
        printk("[MD32] Unable to ioremap registers\n");
        return -1;
    }

    node = of_find_compatible_node(NULL, NULL, "mediatek,SCP_CLK_CTRL");
    if(!node)
    {
        printk("[MD32] Can't find node: mediatek,SCP_CLK_CTRL\n");
    }

    mreg->clkctrl = of_iomap(node, 0);

    if (!mreg->clkctrl) {
        printk("[MD32] Unable to ioremap registers\n");
        return -1;
    }

    return ret;
}

static int md32_pm_event(struct notifier_block *notifier, unsigned long pm_event, void *unused)
{
    int retval;

    switch(pm_event) {
        case PM_POST_HIBERNATION:
            printk("[MD32] md32_pm_event MD32 reboot\n");
            retval = reboot_load_md32();
            if(retval < 0){
                retval = -EINVAL;
                printk("[MD32] md32_pm_event MD32 reboot Fail\n");
            }
            return NOTIFY_DONE;
    }
    return NOTIFY_OK;
}

static struct notifier_block md32_pm_notifier_block = {
    .notifier_call = md32_pm_event,
    .priority = 0,
};

/**
 * driver initialization entry point
 */
static int __init md32_init(void)
{
    int i;
    int ret = 0;
    volatile unsigned int reg;

    mutex_init(&md32_log_mutex);
    init_waitqueue_head(&logwait);

    ret = md32_dt_init();
    if(ret)
    {
        printk("[MD32] Device Init Fail\n");
        return -1;
    }

    md32_send_buff = (unsigned char*)kmalloc((size_t)64 , GFP_KERNEL);
    if(!md32_send_buff)
    {
        printk("[MD32] Fail --> can't allocate send buff\n");
        return -1;
    }
    
    md32_recv_buff = (unsigned char*)kmalloc((size_t)64 , GFP_KERNEL);
    if(!md32_recv_buff)
    {
        printk("[MD32] Fail --> can't allocate recv buff\n");
        return -1;
    }

    /*FIXME: need to call SPM API. hard code temply*/
    //mt_reg_sync_writel(0x0b160001, SLEEP_BASE);
    //mt_reg_sync_writel(0xfffffff0, (SLEEP_BASE + 0x2c8));
#if 0
    spm_poweron_config_set();
    spm_md32_sram_con(0xfffffff0);
#endif
    mt_reg_sync_writel(0x1FF, MD32_CLK_CTRL_BASE+0x030);
    
    mt_reg_sync_writel(0x1, MD32_BASE+0x008);
    
    reg = readl(MD32_CLK_CTRL_BASE+0x02C);
    for (i = 0; i < 7; ++i){
        reg &= ~(0x1 << i);
        mt_reg_sync_writel(reg, MD32_CLK_CTRL_BASE+0x02C);
    }
    
    md32_irq_init();
    md32_ipi_init();
    md32_ocd_init();

    /* register logger IPI */
    md32_ipi_registration(IPI_LOGGER, logger_ipi_handler, "logger");

    /* register log buf full IPI */
    md32_ipi_registration(IPI_BUF_FULL, buf_full_ipi_handler, "buf_full");

    /* register apdma IPI */
    md32_ipi_registration(IPI_APDMA, apdma_ipi_handler, "apdma");

    printk("[MD32] helper init\n");

    if (unlikely((ret = misc_register(&md32_device)) != 0))
    {
        printk("[MD32] misc register failed\n");
        ret = -1;
    }

    if (unlikely((ret = create_files()) != 0)) {
        printk("[MD32] create files failed\n");
    }

    /*
     * Hook to the MD32 chain to get notification
     */
    md32_register_notify(&md32_helper_nb);

    ret = register_pm_notifier(&md32_pm_notifier_block);
    if (ret)
        printk("[MD32] failed to register PM notifier %d\n", ret);

    request_irq(md32reg.irq, md32_irq_handler, IRQF_TRIGGER_NONE, "MD32 IPC_MD2HOST", NULL);

    md32_init_done = 1;

	return ret;
}

/**
 * driver exit point
 */
static void __exit md32_exit(void)
{
    if(swap_buf)
        kfree(swap_buf);
    misc_deregister(&md32_device);
    md32_unregister_notify(&md32_helper_nb);
}

module_init(md32_init);
module_exit(md32_exit);

