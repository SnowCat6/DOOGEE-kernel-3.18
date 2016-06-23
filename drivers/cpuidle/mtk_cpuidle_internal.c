#include <asm/uaccess.h>
#include <linux/cpu.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <mach/mt_clkmgr.h>
#include "mtk_cpuidle_internal.h"

/*Idle handler on/off*/
static int idle_switch[NR_TYPES] = {
    1,  //dpidle switch
    1,  //soidle switch
    1,  //slidle switch
    1,  //rgidle switch
};

static const char *idle_name[NR_TYPES] = {
    "dpidle",
    "soidle",
    "slidle",
    "rgidle",
};

const char *reason_name[NR_REASONS] = {
    "by_cpu",
    "by_clk",
    "by_tmr",
    "by_oth",
    "by_vtg",
};

#define INVALID_GRP_ID(grp) (grp < 0 || grp >= NR_GRPS)

unsigned int dpidle_condition_mask[NR_GRPS] = {
    0x0000008A, //INFRA:
//    0x37FA1FFD, //PERI0:
//  TODO: check uart1 CG on D-1 EVB ??
    0x37F21FFD, //PERI0:
    0x000FFFFF, //DISP0:
    0x0000003F, //DISP1:
    0x00000FE1, //IMAGE:
    0x00000001, //MFG:
    0x00000000, //AUDIO:
    0x00000001, //VDEC0:
    0x00000001, //VDEC1:
    0x00001111, //VENC:
};

static unsigned long    dpidle_cnt[NR_CPUS] = {0};
static unsigned long    dpidle_block_cnt[NR_REASONS] = {0};
unsigned int            dpidle_block_mask[NR_GRPS] = {0x0};
unsigned int            dpidle_time_critera = 26000;
unsigned int            dpidle_block_time_critera = 30000;//default 30sec
bool                    dpidle_by_pass_cg = false;

static unsigned long    soidle_cnt[NR_CPUS] = {0};

static unsigned long    slidle_cnt[NR_CPUS] = {0};

static unsigned long    rgidle_cnt[NR_CPUS] = {0};

void idle_cnt_inc(int idle_type, int cpu)
{
    switch (idle_type) {
    case IDLE_TYPE_DP:
        dpidle_cnt[cpu]++;
        break;
    case IDLE_TYPE_SO:
        soidle_cnt[cpu]++;
        break;
    case IDLE_TYPE_SL:
        slidle_cnt[cpu]++;
        break;
    case IDLE_TYPE_RG:
        rgidle_cnt[cpu]++;
        break;
    default:
        break;
    }
}
EXPORT_SYMBOL(idle_cnt_inc);

unsigned long idle_cnt_get(int idle_type, int cpu)
{
    unsigned long   ret = 0;

    switch (idle_type) {
    case IDLE_TYPE_DP:
        ret = dpidle_cnt[cpu];
        break;
    case IDLE_TYPE_SO:
        ret = soidle_cnt[cpu];
        break;
    case IDLE_TYPE_SL:
        ret = slidle_cnt[cpu];
        break;
    case IDLE_TYPE_RG:
        ret = rgidle_cnt[cpu];
        break;
    default:
        break;
    }

    return ret;
}
EXPORT_SYMBOL(idle_cnt_get);

void idle_block_cnt_inc(int idle_type, int reason)
{
    switch (idle_type) {
    case IDLE_TYPE_DP:
        dpidle_block_cnt[reason]++;
        break;
    case IDLE_TYPE_SO:
        /* TODO */
        break;
    case IDLE_TYPE_SL:
        /* TODO */
        break;
    case IDLE_TYPE_RG:
        /* TODO */
        break;
    default:
        break;
    }
}
EXPORT_SYMBOL(idle_block_cnt_inc);

unsigned long idle_block_cnt_get(int idle_type, int reason)
{
    unsigned long   ret = 0;

    switch (idle_type) {
    case IDLE_TYPE_DP:
        ret = dpidle_block_cnt[reason];
        break;
    case IDLE_TYPE_SO:
        /* TODO */
        break;
    case IDLE_TYPE_SL:
        /* TODO */
        break;
    case IDLE_TYPE_RG:
        /* TODO */
        break;
    default:
        break;
    }

    return ret;
}
EXPORT_SYMBOL(idle_block_cnt_get);

void idle_block_cnt_clr(int idle_type)
{
    unsigned long   ret = 0;

    switch (idle_type) {
    case IDLE_TYPE_DP:
        memset(dpidle_block_cnt, 0, sizeof(dpidle_block_cnt));
        break;
    case IDLE_TYPE_SO:
        /* TODO */
        break;
    case IDLE_TYPE_SL:
        /* TODO */
        break;
    case IDLE_TYPE_RG:
        /* TODO */
        break;
    default:
        break;
    }
}
EXPORT_SYMBOL(idle_block_cnt_clr);

int idle_switch_get(int idle_type)
{
    return (idle_type >= 0 && idle_type < NR_TYPES) ?
                idle_switch[idle_type] :
                0;
}
EXPORT_SYMBOL(idle_switch_get);

static DEFINE_MUTEX(dpidle_locked);

static void enable_dpidle_by_mask(int grp, unsigned int mask)
{
    mutex_lock(&dpidle_locked);
    dpidle_condition_mask[grp] &= ~mask;
    mutex_unlock(&dpidle_locked);
}

static void disable_dpidle_by_mask(int grp, unsigned int mask)
{
    mutex_lock(&dpidle_locked);
    dpidle_condition_mask[grp] |= mask;
    mutex_unlock(&dpidle_locked);
}

void enable_dpidle_by_bit(int id)
{
    int grp = id / 32;
    unsigned int mask = 1U << (id % 32);
    BUG_ON(INVALID_GRP_ID(grp));
    enable_dpidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(enable_dpidle_by_bit);

void disable_dpidle_by_bit(int id)
{
    int grp = id / 32;
    unsigned int mask = 1U << (id % 32);
    BUG_ON(INVALID_GRP_ID(grp));
    disable_dpidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(disable_dpidle_by_bit);

/***************************/
/* debugfs                 */
/***************************/
static char dbg_buf[2048] = {0};
static char cmd_buf[512] = {0};

/* idle_state */
static int _idle_state_open(struct seq_file *s, void *data)
{
    return 0;
}

static int idle_state_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, _idle_state_open, inode->i_private);
}

static ssize_t idle_state_read(struct file *filp, 
                                 char __user *userbuf, 
                                 size_t count, 
                                 loff_t *f_pos)
{
    int len = 0;
    char *p = dbg_buf;
    int i;

    p += sprintf(p, "********** idle state dump **********\n");

    for (i = 0; i < nr_cpu_ids; i++) {
        p += sprintf(p, "soidle_cnt[%d]=%lu, dpidle_cnt[%d]=%lu, "
                "slidle_cnt[%d]=%lu, rgidle_cnt[%d]=%lu\n",
                i, soidle_cnt[i], i, dpidle_cnt[i],
                i, slidle_cnt[i], i, rgidle_cnt[i]);
    }

    p += sprintf(p, "\n********** variables dump **********\n");
    for (i = 0; i < NR_TYPES; i++) {
        p += sprintf(p, "%s_switch=%d, ", idle_name[i], idle_switch[i]);
    }
    p += sprintf(p, "\n");

    p += sprintf(p, "\n********** idle command help **********\n");
    p += sprintf(p, "status help:   cat /sys/power/idle_state\n");
    p += sprintf(p, "switch on/off: echo switch mask > /sys/power/idle_state\n");

    p += sprintf(p, "soidle help:   cat /sys/power/soidle_state\n");
    p += sprintf(p, "dpidle help:   cat /sys/power/dpidle_state\n");
    p += sprintf(p, "slidle help:   cat /sys/power/slidle_state\n");
    p += sprintf(p, "rgidle help:   cat /sys/power/rgidle_state\n");

    len = p - dbg_buf;

    return simple_read_from_buffer(userbuf, count, f_pos, dbg_buf, len);
}

static ssize_t idle_state_write(struct file *filp, 
                                  const char __user *userbuf, 
                                  size_t count, 
                                  loff_t *f_pos)
{
    char cmd[32];
    int idx;
    int param;

    count = min(count, sizeof(cmd_buf) - 1);

    if (copy_from_user(cmd_buf, userbuf, count)) {
        return -EFAULT;
    }
    cmd_buf[count] = '\0';

    if (sscanf(cmd_buf, "%s %x", cmd, &param) == 2) {
        if (!strcmp(cmd, "switch")) {
            for (idx = 0; idx < NR_TYPES; idx++) {
                idle_switch[idx] = (param & (1U << idx)) ? 1 : 0;
            }
        }
        return count;
    }

    return -EINVAL;
}

static const struct file_operations idle_state_fops = {
    .open = idle_state_open,
    .read = idle_state_read,
    .write = idle_state_write,
    .llseek = seq_lseek,
    .release = single_release,
};
/* dpidle_state */
static int _dpidle_state_open(struct seq_file *s, void *data)
{
    return 0;
}

static int dpidle_state_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, _dpidle_state_open, inode->i_private);
}

static ssize_t dpidle_state_read(struct file *filp, char __user *userbuf, size_t count, loff_t *f_pos)
{
    int len = 0;
    char *p = dbg_buf;
    int i;
    ssize_t retval = 0;

    p += sprintf(p, "*********** deep idle state ************\n");
    p += sprintf(p, "dpidle_time_critera=%u\n", dpidle_time_critera);

    for (i = 0; i < NR_REASONS; i++) {
        p += sprintf(p, "[%d]dpidle_block_cnt[%s]=%lu\n", i, reason_name[i],
                dpidle_block_cnt[i]);
    }

    p += sprintf(p, "\n");

    for (i = 0; i < NR_GRPS; i++) {
        p += sprintf(p, "[%02d]dpidle_condition_mask[%-8s]=0x%08x\t\t"
                "dpidle_block_mask[%-8s]=0x%08x\n", i,
                grp_get_name(i), dpidle_condition_mask[i],
                grp_get_name(i), dpidle_block_mask[i]);
    }

    p += sprintf(p, "dpidle_bypass_cg=%u\n", dpidle_by_pass_cg);

    p += sprintf(p, "\n*********** dpidle command help  ************\n");
    p += sprintf(p, "dpidle help:   cat /sys/power/dpidle_state\n");
    p += sprintf(p, "switch on/off: echo [dpidle] 1/0 > /sys/power/dpidle_state\n");
    p += sprintf(p, "cpupdn on/off: echo cpupdn 1/0 > /sys/power/dpidle_state\n");
    p += sprintf(p, "en_dp_by_bit:  echo enable id > /sys/power/dpidle_state\n");
    p += sprintf(p, "dis_dp_by_bit: echo disable id > /sys/power/dpidle_state\n");
    p += sprintf(p, "modify tm_cri: echo time value(dec) > /sys/power/dpidle_state\n");
    p += sprintf(p, "bypass cg:     echo bypass 1/0 > /sys/power/dpidle_state\n");

    len = p - dbg_buf;

    return simple_read_from_buffer(userbuf, count, f_pos, dbg_buf, len);
}

static ssize_t dpidle_state_write(struct file *filp, 
                                  const char __user *userbuf, 
                                  size_t count, 
                                  loff_t *f_pos)
{
    char cmd[32];
    int param;

    count = min(count, sizeof(cmd_buf) - 1);

    if (copy_from_user(cmd_buf, userbuf, count)) {
        return -EFAULT;
    }
    cmd_buf[count] = '\0';

    if (sscanf(cmd_buf, "%s %d", cmd, &param) == 2) {
        if (!strcmp(cmd, "dpidle")) {
            idle_switch[IDLE_TYPE_DP] = param;
        } else if (!strcmp(cmd, "enable")) {
            enable_dpidle_by_bit(param);
        } else if (!strcmp(cmd, "disable")) {
            disable_dpidle_by_bit(param);
        } else if (!strcmp(cmd, "time")) {
            dpidle_time_critera = param;
        }else if (!strcmp(cmd, "bypass")) {
            dpidle_by_pass_cg = param;
            printk(KERN_WARNING"bypass = %d\n", dpidle_by_pass_cg);
        }
        return count;
    } else if (sscanf(cmd_buf, "%d", &param) == 1) {
        idle_switch[IDLE_TYPE_DP] = param;
        return count;
    }

    return -EINVAL;
}

static const struct file_operations dpidle_state_fops = {
    .open = dpidle_state_open,
    .read = dpidle_state_read,
    .write = dpidle_state_write,
    .llseek = seq_lseek,
    .release = single_release,
};

static struct dentry *root_entry;

int mtk_cpuidle_debugfs_init(void)
{
    /* TODO: check if debugfs_create_file() failed */
    /* Initialize debugfs */
    root_entry = debugfs_create_dir("cpuidle", NULL);
    if (!root_entry) {
        printk(KERN_WARNING"Can not create debugfs `dpidle_state`\n");
        return 1;
    }

    debugfs_create_file("idle_state", 0644, root_entry, NULL, &idle_state_fops);
    debugfs_create_file("dpidle_state", 0644, root_entry, NULL, &dpidle_state_fops);

    return 0;
}
