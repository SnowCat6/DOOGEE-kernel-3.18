/*****************************************************************************/
/*****************************************************************************/
#if defined(CONFIG_MTK_HDMI_SUPPORT)
#define _tx_c_

#include <linux/mm.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/rtpm_prio.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/switch.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
//#include <asm/mach-types.h>
#include <mach/mt_typedefs.h>

#include "extd_utils.h"
#include "mtk_ext_disp_mgr.h"
#include <linux/earlysuspend.h>
#include "extd_factory.h"
#include <linux/suspend.h>


#define HDMI_DEVNAME "hdmitx"
#define EXTD_DEV_ID(id) (((id)>>16)&0x0ff)
#define EXTD_DEV_PARAM(id) ((id)&0x0ff)


static dev_t hdmi_devno;
static struct cdev *hdmi_cdev;
static struct class *hdmi_class = NULL;

static void external_display_enable(unsigned long param)
{
    EXTD_DEV_ID device_id = EXTD_DEV_ID(param);
    int enable = EXTD_DEV_PARAM(param);

    switch(device_id)
    {
        case DEV_MHL:
            hdmi_enable(enable);
            break;
        case DEV_WFD:
            /*enable or diable wifi display*/
            break;
        default:
            break;
    }
}

static void external_display_power_enable(unsigned long param)
{
    EXTD_DEV_ID device_id = EXTD_DEV_ID(param);
    int enable = EXTD_DEV_PARAM(param);

    switch(device_id)
    {
        case DEV_MHL:
            hdmi_power_enable(enable);
            break;
        case DEV_WFD:
            /*enable or diable wifi display power*/
            break;
        default:
            break;
    }
}

static void external_display_force_disable(unsigned long param)
{
    EXTD_DEV_ID device_id = EXTD_DEV_ID(param);
    int enable = EXTD_DEV_PARAM(param);

    switch(device_id)
    {
        case DEV_MHL:
            hdmi_force_disable(enable);
            break;
        case DEV_WFD:
            /*enable or diable wifi display power*/
            break;
        default:
            break;
    }
}

static void external_display_set_resolution(unsigned long param)
{
    EXTD_DEV_ID device_id = EXTD_DEV_ID(param);
    int res = EXTD_DEV_PARAM(param);

    switch(device_id)
    {
        case DEV_MHL:
            hdmi_set_resolution(res);
            break;
        case DEV_WFD:
            /*enable or diable wifi display power*/
            break;
        default:
            break;
    }
}

static int external_display_get_dev_info(unsigned long param, void *info)
{
    int ret = 0;
    EXTD_DEV_ID device_id = EXTD_DEV_ID(param);

    switch(device_id)
    {
        case DEV_MHL:
            ret = hdmi_get_dev_info(info);
            break;
        case DEV_WFD:
            /*enable or diable wifi display power*/
            break;
        default:
            break;
    }

    return ret;
}

static long mtk_ext_disp_mgr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int r = 0;

    printk("[EXTD]ioctl= %s(%d), arg = %lu\n", _hdmi_ioctl_spy(cmd), cmd & 0xff, arg);

    switch (cmd)
    {
        case MTK_HDMI_AUDIO_VIDEO_ENABLE:
        {
            /* 0xXY
                       * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
                       * the device id:
                       * X = 0 - mhl
                       * X = 1 - wifi display
                    */
            external_display_enable(arg);
            break;
        }

        case MTK_HDMI_POWER_ENABLE:
        {
            /* 0xXY
                       * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
                       * the device id:
                       * X = 0 - mhl
                       * X = 1 - wifi display
                    */
            external_display_power_enable(arg);
            break;
        }

        case MTK_HDMI_VIDEO_CONFIG:
        {
            /* 0xXY
                       * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
                       * the device id:
                       * X = 0 - mhl
                       * X = 1 - wifi display
                       */
            external_display_set_resolution(arg);
            break;
        }

        case MTK_HDMI_FORCE_FULLSCREEN_ON:
            //case MTK_HDMI_FORCE_CLOSE:
        {
            /* 0xXY
                       * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
                       * the device id:
                       * X = 0 - mhl
                       * X = 1 - wifi display
                       */
            arg = arg | 0x1;
            external_display_power_enable(arg);
            break;
        }

        case MTK_HDMI_FORCE_FULLSCREEN_OFF:
            //case MTK_HDMI_FORCE_OPEN:
        {
            /* 0xXY
                       * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
                       * the device id:
                       * X = 0 - mhl
                       * X = 1 - wifi display
                     */
            arg = arg & 0x0FF0000;
            external_display_power_enable(arg);
            break;
        }

        case MTK_HDMI_GET_DEV_INFO:
        {
            /* 0xXY
                       * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
                       * the device id:
                       * X = 0 - mhl
                       * X = 1 - wifi display
                     */
            r = external_display_get_dev_info(*((unsigned long *)argp), argp);
            break;
        }

        case MTK_HDMI_USBOTG_STATUS:
        {
            //hdmi_set_USBOTG_status(arg);
            break;
        }

        case MTK_HDMI_AUDIO_ENABLE:
        {
            printk("[EXTD]hdmi_set_audio_enable, arg = %lu\n", arg);            
            hdmi_set_audio_enable(arg);
            break;
        }

        case MTK_HDMI_VIDEO_ENABLE:
        {
            break;
        }

        case MTK_HDMI_AUDIO_CONFIG:
        {
			r = hdmi_audio_config(arg);
            break;
        }

        case MTK_HDMI_IS_FORCE_AWAKE:
        {
            r = hdmi_is_force_awake(argp);
            break;
        }

        case MTK_HDMI_GET_EDID:
        {  
            hdmi_get_edid(argp);
            break;
        }
		
        case MTK_HDMI_SCREEN_CAPTURE:
        {
            r = hdmi_screen_capture(argp);
            break;
        }

		case MTK_HDMI_FACTORY_CHIP_INIT:
		{
			r = hdmi_factory_mode_test(STEP1_CHIP_INIT, NULL);
			break;
		}
		
		case MTK_HDMI_FACTORY_JUDGE_CALLBACK:
		{
			r = hdmi_factory_mode_test(STEP2_JUDGE_CALLBACK, argp);
			break;
		}
		
		case MTK_HDMI_FACTORY_START_DPI_AND_CONFIG:
		{
			r = hdmi_factory_mode_test(STEP3_START_DPI_AND_CONFIG, arg);
			//hdmi_factory_mode_test(STEP1_CHIP_INIT, NULL);
			//hdmi_factory_mode_test(STEP3_START_DPI_AND_CONFIG, arg);
			break;
		}
		case MTK_HDMI_FACTORY_DPI_STOP_AND_POWER_OFF:
		{
			///r = hdmi_factory_mode_test(STEP4_DPI_STOP_AND_POWER_OFF, NULL);
			break;
		}
        case MTK_HDMI_FAKE_PLUG_IN:
        {
         	if(arg)
         	{
				hdmi_cable_fake_plug_in();
			}
			else
			{
				hdmi_cable_fake_plug_out();
			}
            break;
        }
        default:
        {
            printk("[EXTD]ioctl(%d) arguments is not support\n",  cmd & 0x0ff);
            r = -EFAULT;
            break;
        }
    }

    printk("[EXTD]ioctl = %s(%d) done\n",  _hdmi_ioctl_spy(cmd), cmd & 0x0ff);
    return r;
}

static int mtk_ext_disp_mgr_open(struct inode *inode, struct file *file)
{
    printk("[EXTD]%s\n", __func__);
    return 0;
}

static int mtk_ext_disp_mgr_release(struct inode *inode, struct file *file)
{
    printk("[EXTD]%s\n", __func__);
    return 0;
}

#ifdef CONFIG_COMPAT
static long mtk_ext_disp_mgr_compat_ioctl(struct file *file, unsigned int cmd,  unsigned long arg)
{
	long ret = -ENOIOCTLCMD;

	switch(cmd) {	
	
    // add cases here for 32bit/64bit conversion
    // ...
    
	default:
		return mtk_ext_disp_mgr_ioctl(file,  cmd,  arg);
	}
	return ret;
}
#endif

struct file_operations external_display_fops =
{
    .owner   = THIS_MODULE,
    .unlocked_ioctl   = mtk_ext_disp_mgr_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mtk_ext_disp_mgr_compat_ioctl,
#endif	
    .open    = mtk_ext_disp_mgr_open,
    .release = mtk_ext_disp_mgr_release,
};

static const struct of_device_id hdmi_of_ids[] = {
	{.compatible = "mediatek,HDMI", },
	{}
};

static int mtk_ext_disp_mgr_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct class_device *class_dev = NULL;

    printk("[EXTD]%s\n", __func__);

    /* Allocate device number for hdmi driver */
    ret = alloc_chrdev_region(&hdmi_devno, 0, 1, HDMI_DEVNAME);

    if (ret)
    {
        printk("[EXTD]alloc_chrdev_region fail\n");
        return -1;
    }

    /* For character driver register to system, device number binded to file operations */
    hdmi_cdev = cdev_alloc();
    hdmi_cdev->owner = THIS_MODULE;
    hdmi_cdev->ops = &external_display_fops;
    ret = cdev_add(hdmi_cdev, hdmi_devno, 1);

    /* For device number binded to device name(hdmitx), one class is corresponeded to one node */
    hdmi_class = class_create(THIS_MODULE, HDMI_DEVNAME);
    /* mknod /dev/hdmitx */
    class_dev = (struct class_device *)device_create(hdmi_class, NULL, hdmi_devno, NULL, HDMI_DEVNAME);

    printk("[EXTD][%s] out\n", __func__);

    return 0;
}

static int mtk_ext_disp_mgr_remove(struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_PM
int hdmi_pm_suspend(struct device *device)
{
    printk("hdmi_pm_suspend()\n");

    return 0;
}

int hdmi_pm_resume(struct device *device)
{
    printk("HDMI_Npm_resume()\n");

    return 0;
}

struct dev_pm_ops hdmi_pm_ops = {
    .suspend =  hdmi_pm_suspend,
    .resume =   hdmi_pm_resume,
};
#endif

static struct platform_driver external_display_driver =
{
    .probe  = mtk_ext_disp_mgr_probe,
    .remove = mtk_ext_disp_mgr_remove,
    .driver = { 
    	.name = HDMI_DEVNAME,
#ifdef CONFIG_PM
        .pm     = &hdmi_pm_ops,
#endif
    	.of_match_table = hdmi_of_ids,
    }
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hdmi_early_suspend(struct early_suspend *h)
{
    printk(" hdmi_early_suspend \n");
    hdmi_power_enable(0);
#ifndef CONFIG_SINGLE_PANEL_OUTPUT	
    hdmi_factory_mode_test(STEP1_ENABLE, NULL);
    hdmi_factory_mode_test(STEP3_SUSPEND, NULL);
#endif	
}

static void hdmi_late_resume(struct early_suspend *h)
{
    ///printk(" hdmi_late_resume\n"); 
    hdmi_power_enable(1);  
}

static struct early_suspend hdmi_early_suspend_handler =
{
	.level =    EARLY_SUSPEND_LEVEL_BLANK_SCREEN +1 ,
	.suspend =  hdmi_early_suspend,
	.resume =   hdmi_late_resume,
};
#endif

static int __init mtk_ext_disp_mgr_init(void)
{
    printk("[EXTD]%s\n", __func__);

    if (platform_driver_register(&external_display_driver))
    {
        printk("[EXTD]failed to register mtkfb driver\n");
        return -1;
    }

    hdmi_init();

#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend(&hdmi_early_suspend_handler);
#endif        
    return 0;
}

static void __exit mtk_ext_disp_mgr_exit(void)
{

    device_destroy(hdmi_class, hdmi_devno);
    class_destroy(hdmi_class);
    cdev_del(hdmi_cdev);
    unregister_chrdev_region(hdmi_devno, 1);

}

module_init(mtk_ext_disp_mgr_init);
module_exit(mtk_ext_disp_mgr_exit);
MODULE_AUTHOR("www.mediatek.com>");
MODULE_DESCRIPTION("External Display Driver");
MODULE_LICENSE("GPL");

#endif
