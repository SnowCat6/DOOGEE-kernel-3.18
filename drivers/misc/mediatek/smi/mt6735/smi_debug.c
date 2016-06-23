#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/aee.h>
#include <linux/timer.h>
//#include <asm/system.h>
#include <asm-generic/irq_regs.h>
//#include <asm/mach/map.h>
#include <mach/sync_write.h>
#include <mach/irqs.h>
#include <asm/cacheflush.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/fb.h>
#include <linux/debugfs.h>
#include <mach/mt_typedefs.h>
#include <mach/m4u.h>
#include <mach/mt_smi.h>

#include "smi_common.h"

#include <linux/xlog.h>

#ifdef D1
    #include "smi_reg_d1.h"
#elif defined D2
    #include "smi_reg_d2.h"
#else
    #include "smi_reg_d3.h"
#endif

#define SMI_LOG_TAG "smi"

static char debug_buffer[4096];

static void process_dbg_opt(const char *opt)
{
    if (0 == strncmp(opt, "set_reg:", 8 ))
    {
        unsigned long addr;
        unsigned int val;
		char *p = (char *)opt + 8;

		addr = (unsigned long) simple_strtoul(p, &p, 16);
		p++;
		val = (unsigned int) simple_strtoul(p, &p, 16);

		SMIMSG("set register: 0x%lx = 0x%x\n", addr, val);

        COM_WriteReg32(addr, val);
    }
    if (0 == strncmp(opt, "get_reg:", 8 ))
    {
        unsigned long addr;
		char *p = (char *)opt + 8;

		addr = (unsigned long) simple_strtoul(p, &p, 16);

		SMIMSG("get register: 0x%lx = 0x%x \n", addr, COM_ReadReg32(addr));
    }
    

    
    return;
}


static void process_dbg_cmd(char *cmd)
{
    char *tok;
    while ((tok = strsep(&cmd, " ")) != NULL)
    {
        process_dbg_opt(tok);
    }
}


// ---------------------------------------------------------------------------
//  Debug FileSystem Routines
// ---------------------------------------------------------------------------

struct dentry *smi_dbgfs = NULL;


static int debug_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}

static ssize_t debug_read(struct file *file,
                          char __user *ubuf, size_t count, loff_t *ppos)
{
    int n = 0;
    return simple_read_from_buffer(ubuf, count, ppos, debug_buffer, n);
}


static ssize_t debug_write(struct file *file,
                           const char __user *ubuf, size_t count, loff_t *ppos)
{
    const int debug_bufmax = sizeof(debug_buffer) - 1;
	size_t ret;

	ret = count;

	if (count > debug_bufmax)
        count = debug_bufmax;

	if (copy_from_user(&debug_buffer, ubuf, count))
		return -EFAULT;

	debug_buffer[count] = 0;

    process_dbg_cmd(debug_buffer);

    return ret;
}


static struct file_operations debug_fops = {
	.read  = debug_read,
    .write = debug_write,
	.open  = debug_open,
};


void SMI_DBG_Init(void)
{
    smi_dbgfs = debugfs_create_file("smi",
        S_IFREG|S_IRUGO, NULL, (void *)0, &debug_fops);
}


void SMI_DBG_Deinit(void)
{
    debugfs_remove(smi_dbgfs);
}


