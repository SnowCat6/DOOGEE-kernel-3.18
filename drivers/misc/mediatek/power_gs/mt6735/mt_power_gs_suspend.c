#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_power_gs.h>

#include <mach/mt_gpio.h>

extern unsigned int *MT6328_PMIC_REG_gs_flightmode_suspend_mode;
extern unsigned int MT6328_PMIC_REG_gs_flightmode_suspend_mode_len;

static bool gpio_force = 0;

static void mt_power_gpio_golden(void)
{
#if !defined (CONFIG_MTK_FPGA)
	int i = 0;
	//no pull
	for(i=0; i<5; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);
    
    for(i=19; i<21; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);
    
    for(i=38; i<42; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);    
        
    for(i=49; i<55; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);    
    
    for(i=57; i<61; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);    
    
    for(i=64; i<65; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE); 

    for(i=74; i<78; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);
    
    for(i=89; i<87; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);    
    
    for(i=92; i<93; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);
    
    for(i=94; i<96; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);        
        
    for(i=123; i<133; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);    
        
    for(i=198; i<204; i++)
        mt_set_gpio_pull_enable(i, GPIO_PULL_DISABLE);

    //pull up
    for(i=5; i<7; i++) {
        mt_set_gpio_pull_enable(i, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(i , GPIO_PULL_UP);
    }
    for(i=8; i<11; i++) {
        mt_set_gpio_pull_enable(i, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(i , GPIO_PULL_UP);
    }
    for(i=65; i<68; i++) {
        mt_set_gpio_pull_enable(i, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(i , GPIO_PULL_UP);
    }
    for(i=78; i<79; i++) {
        mt_set_gpio_pull_enable(i, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(i , GPIO_PULL_UP);
    }
    for(i=130; i<131; i++) {
        mt_set_gpio_pull_enable(i, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(i , GPIO_PULL_UP);
    }
#endif    
}

void mt_power_gs_dump_suspend(void)
{
    mt_power_gs_compare("Suspend",                                \
			MT6328_PMIC_REG_gs_flightmode_suspend_mode, MT6328_PMIC_REG_gs_flightmode_suspend_mode_len);
    
    if(gpio_force)
        mt_power_gpio_golden();
}

//static int dump_suspend_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
static int dump_suspend_read(struct seq_file *m, void *v)
{
    seq_printf(m, "mt_power_gs : suspend\n");
    mt_power_gs_dump_suspend();

    return 0;
}

static void __exit mt_power_gs_suspend_exit(void)
{
    remove_proc_entry("dump_suspend", mt_power_gs_dir);
}

static int proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, dump_suspend_read, NULL);
}

static const struct file_operations proc_fops =
{
    .owner = THIS_MODULE,
    .open = proc_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static int __init mt_power_gs_suspend_init(void)
{
    struct proc_dir_entry *mt_entry = NULL;

    if (!mt_power_gs_dir)
    {
        printk("[%s]: mkdir /proc/mt_power_gs failed\n", __FUNCTION__);
    }
    else
    {
        mt_entry = proc_create("dump_suspend", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir, &proc_fops);
        if (NULL == mt_entry)
        {
            return -ENOMEM;
        }
    }

    return 0;
}

module_init(mt_power_gs_suspend_init);
module_exit(mt_power_gs_suspend_exit);

module_param(gpio_force, bool, 0644);

MODULE_DESCRIPTION("MT Power Golden Setting - Suspend");
