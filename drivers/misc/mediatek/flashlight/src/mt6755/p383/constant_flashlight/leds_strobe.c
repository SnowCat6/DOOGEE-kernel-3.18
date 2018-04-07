/**************************************************************************
*  aw3644_flashlight.c
* 
*  Create Date : 
* 
*  Modify Date : 
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 0.9, 2016/05/15
**************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>

#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include "kd_flashlight_type.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
/******************************************************************************
 * Debug configuration
******************************************************************************/

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE

#ifdef DEBUG_LEDS_STROBE
    #define PK_DBG PK_DBG_FUNC
#else
    #define PK_DBG(a, ...)
#endif
/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock);    /* cotta-- SMP proection */

static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;
static BOOL ktd2685_use = 0;
static int g_timeOutTimeMs;

static struct work_struct workTimeOut;
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AW3644_I2C_NAME "AW3644_FLASHLIGHT"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char aw3644_hw_on(void);
unsigned char aw3644_hw_off(void);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw3644_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t aw3644_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t aw3644_set_hwen(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t aw3644_get_hwen(struct device* cd, struct device_attribute *attr, char* buf);

static DEVICE_ATTR(reg, 0660, aw3644_get_reg,  aw3644_set_reg);
static DEVICE_ATTR(hwen, 0660, aw3644_get_hwen,  aw3644_set_hwen);

struct i2c_client *aw3644_flashlight_client;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char i2c_write_reg(unsigned char addr, unsigned char reg_data)
{
	char ret;
	u8 wdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= aw3644_flashlight_client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= wdbuf,
		},
	};

	wdbuf[0] = addr;
	wdbuf[1] = reg_data;

	if(NULL == aw3644_flashlight_client)
	{
		pr_err("msg %s aw3644_flashlight_client is NULL\n", __func__);
		return -1;	
	}

	ret = i2c_transfer(aw3644_flashlight_client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

unsigned char I2C_read_reg(unsigned char addr)
{
	unsigned char ret;
	u8 rdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= aw3644_flashlight_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= aw3644_flashlight_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};

	rdbuf[0] = addr;
	
	if(NULL == aw3644_flashlight_client)
	{
		pr_err("msg %s aw3644_flashlight_client is NULL\n", __func__);
		return -1;	
	}

	ret = i2c_transfer(aw3644_flashlight_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return rdbuf[0];
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AW3644 Debug
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char aw3644_hwen_on(void)
{
	printk("%s enter\n", __func__);
	flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_LOW);
	msleep(5);
	flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_HIGH);
	msleep(10);
	printk("%s exit\n", __func__);
	
	return 0;
}

unsigned char aw3644_hwen_off(void)
{
	printk("%s enter\n", __func__);
	flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_LOW);
	msleep(5);
	printk("%s exit\n", __func__);
	
	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AW3644 Debug file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw3644_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;
	
	for(i=0;i<0x0E;i++)
	{
		reg_val = I2C_read_reg(i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%2X, ", i,reg_val);
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	
	return len;
}

static ssize_t aw3644_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		i2c_write_reg(databuf[0],databuf[1]);
	}
	return len;
}

static ssize_t aw3644_get_hwen(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "aw3644_hwen_on(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 1 > hwen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "aw3644_hwen_off(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 0 > hwen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}

static ssize_t aw3644_set_hwen(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	int databuf[16];
	
	sscanf(buf,"%d",&databuf[0]);
	if(databuf[0] == 0) {			// OFF
		aw3644_hwen_off();	
	} else {				// ON
		aw3644_hwen_on();			
	}
	
	return len;
}


static int aw3644_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	err = device_create_file(dev, &dev_attr_hwen);

	return err;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int aw3644_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned char reg_chipid, reg_devid;
	unsigned char cnt = 3;
	int err = 0;
	
	printk("%s enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	aw3644_flashlight_client = client;

	aw3644_hwen_on();

	while(cnt>0)
	{
		reg_chipid = I2C_read_reg(0x00);
		reg_devid = I2C_read_reg(0x0C);
		printk("aw3644 i2c read chipid=0x%2x, deveice id=0x%02x\n", reg_chipid, reg_devid);
		if((reg_chipid == 0x36) && ((reg_devid&0x18) == 0x00))
		{
			break;
		}
		else
		{
			printk("aw3644 i2c read fail\n");
		}
		cnt --;
		msleep(10);
	}
	if(!cnt)
	{
            ktd2685_use = 1;
		//err = -ENODEV;
		//aw3644_hwen_off();
		//goto exit_create_singlethread;
	}

	aw3644_create_sysfs(client);	

	aw3644_hwen_off();
	
	return 0;

//exit_create_singlethread:
	aw3644_flashlight_client = NULL;
exit_check_functionality_failed:
	return err;	
}

static int aw3644_i2c_remove(struct i2c_client *client)
{
	aw3644_flashlight_client = NULL;
	return 0;
}

static const struct i2c_device_id aw3644_i2c_id[] = {
	{ AW3644_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static const struct of_device_id aw3644_i2c_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver aw3644_i2c_driver = {
        .driver = {
                .name   = AW3644_I2C_NAME,
#ifdef CONFIG_OF
		.of_match_table = aw3644_i2c_of_match,
#endif
},

        .probe          = aw3644_i2c_probe,
        .remove         = aw3644_i2c_remove,
        .id_table       = aw3644_i2c_id,
};
/*
static int aw3644_flashlight_probe(struct platform_device *pdev)
{
	int ret = 0;

	printk("%s enter!\n", __func__);
	
	ret = aw3644_pinctrl_init(pdev);
	if (ret != 0) {
		printk("[%s] failed to init aw3644 pinctrl.\n", __func__);
		return ret;
	} else {
		printk("[%s] Success to init aw3644 pinctrl.\n", __func__);
	}
	
	ret = i2c_add_driver(&aw3644_i2c_driver);
	if (ret != 0) {
		printk("[%s] failed to register aw3644 i2c driver.\n", __func__);
		return ret;
	} else {
		printk("[%s] Success to register aw3644 i2c driver.\n", __func__);
	}

	return ret;
}

static int aw3644_flashlight_remove(struct platform_device *pdev)
{
	printk("%s enter\n", __func__);
	i2c_del_driver(&aw3644_i2c_driver);
	printk("%s exit\n", __func__);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw3644_plt_of_match[] = {
	{.compatible = "mediatek,aw3644_flashlight"},
	{},
};
#endif

static struct platform_driver aw3644_flashlight_driver = {
	.probe	 = aw3644_flashlight_probe,
	.remove	 = aw3644_flashlight_remove,
	.driver = {
		.name   = "aw3644_flashlight",
#ifdef CONFIG_OF
		.of_match_table = aw3644_plt_of_match,
#endif
        }
};
*/
static int __init aw3644_flashlight_init(void) {

	printk("%s enter \n", __func__);
    return i2c_add_driver(&aw3644_i2c_driver);	
/*	ret = platform_driver_register(&aw3644_flashlight_driver);
	if (ret) {
		printk("****[%s] Unable to register driver (%d)\n", __func__, ret);
		return ret;
	}

	return ret;
*/
}

static void __exit aw3644_flashlight_exit(void) {
	printk("%s enter \n", __func__);
    return i2c_del_driver(&aw3644_i2c_driver);	
	//platform_driver_unregister(&aw3644_flashlight_driver);	
}

module_init(aw3644_flashlight_init);
module_exit(aw3644_flashlight_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW3644 Flash Light Driver");
MODULE_LICENSE("GPL v2");


/*****************************************************************************
Dual-flash functions
*****************************************************************************/
enum
{
	e_DutyNum = 26,
};

static bool g_IsTorch[26] = 	{ 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
				  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				  0, 0, 0, 0, 0, 0 };
/*static char g_TorchDutyCode[26] =	{ 0x0F, 0x20, 0x31, 0x42, 0x52, 0x63,
*/
static char g_TorchDutyCode[26] =	{ 0x06, 0x0f, 0x17, 0x1f, 0x27, 0x2f,
					  0x37, 0x00, 0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00 };

static char g_FlashDutyCode[26] =	{ 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B,
					  0x0D, 0x10, 0x14, 0x19, 0x1D, 0x21,
					  0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B,
					  0x3F, 0x43, 0x48, 0x4C, 0x4E, 0x50,
					  0x52, 0x54 };
static char g_EnableReg;
static int g_duty1 = -1;
static int g_duty2 = -1;

int flashEnable_aw3644_2(void)
{
	int ret;
	unsigned char buf[2];

	buf[0] = 0x01; /* Enable Register */
	if (g_IsTorch[g_duty2] == 1) /* LED1 in torch mode */
		g_EnableReg |= (0x09);
	else
		g_EnableReg |= (0x0D);
	buf[1] = g_EnableReg;
	ret = i2c_write_reg(buf[0], buf[1]);
	return ret;
}

int flashEnable_aw3644_1(void)
{
	int ret;
	unsigned char buf[2];

	buf[0] = 0x01; /* Enable Register */
	if (g_IsTorch[g_duty1] == 1) /* LED2 in torch mode */
		g_EnableReg |= (0x0A);
	else
		g_EnableReg |= (0x0E);
	buf[1] = g_EnableReg;
	ret = i2c_write_reg(buf[0], buf[1]);
	return ret;
}

int flashDisable_aw3644_2(void)
{
	int ret;
	unsigned char buf[2];

	buf[0] = 0x01; /* Enable Register */
	if ((g_EnableReg&0x02) == 0x02) /* LED2 enabled */
		g_EnableReg &= (~0x01);
	else
		g_EnableReg &= (~0x0D);
	buf[1] = g_EnableReg;
	ret = i2c_write_reg(buf[0], buf[1]);
	return ret;
}

int flashDisable_aw3644_1(void)
{
	int ret;
	unsigned char buf[2];

	buf[0] = 0x01; /* Enable Register */
	if ((g_EnableReg&0x01) == 0x01) /* LED1 enabled */
		g_EnableReg &= (~0x02);
	else
		g_EnableReg &= (~0x0E);
	buf[1] = g_EnableReg;
	ret = i2c_write_reg(buf[0], buf[1]);
	return ret;
}

int setDuty_aw3644_2(int duty)
{
	int ret;
	unsigned char buf[2];

	if (duty < 0)
		duty = 0;
	else if (duty >= e_DutyNum)
		duty = e_DutyNum-1;

	g_duty2 = duty;
#if 0
	if (duty == 0) /* LED1 in torch mode */
        {
	buf[0] = 0x03; /* LED1 Flash Brightness Register */
	buf[1] = 0x01;
	ret = i2c_write_reg(buf[0], buf[1]);
	buf[0] = 0x05; /* LED1 Torch Brightness Register */
	buf[1] = 0x01;
	ret = i2c_write_reg(buf[0], buf[1]);
        }
        else
#endif
        {
	buf[0] = 0x05; /* LED1 Torch Brightness Register */
	buf[1] = g_TorchDutyCode[duty];
	ret = i2c_write_reg(buf[0], buf[1]);

	buf[0] = 0x03; /* LED1 Flash Brightness Register */
	buf[1] = g_FlashDutyCode[duty];
	ret = i2c_write_reg(buf[0], buf[1]);
        }
	return ret;
}

int setDuty_aw3644_1(int duty)
{
	int ret;
	unsigned char buf[2];

	if (duty < 0)
		duty = 0;
	else if (duty >= e_DutyNum)
		duty = e_DutyNum-1;

	g_duty1 = duty;
	buf[0] = 0x06; /* LED2 Torch Brightness Register */
	buf[1] = g_TorchDutyCode[duty];
	ret = i2c_write_reg(buf[0], buf[1]);

	buf[0] = 0x04; /* LED2 Flash Brightness Register */
	buf[1] = g_FlashDutyCode[duty];
	ret = i2c_write_reg(buf[0], buf[1]);
	return ret;
}

int init_aw3644(void)
{
	int ret;
	char buf[2];

	aw3644_hwen_on();
	
	buf[0] = 0x01; /* Enable Register */
	buf[1] = 0x00;
	g_EnableReg = buf[1];
	ret = i2c_write_reg(buf[0], buf[1]);

	buf[0] = 0x03; /* LED1 Flash Brightness Register */
	buf[1] = 0x3F;
	ret = i2c_write_reg(buf[0], buf[1]);

	buf[0] = 0x05; /* LED2 Flash Brightness Register */
	buf[1] = 0x3F;
	ret = i2c_write_reg(buf[0], buf[1]);

	buf[0] = 0x08; /* Timing Configuration Register */
	buf[1] = 0x1F;
	ret = i2c_write_reg(buf[0], buf[1]);
	return ret;
}

int FL_Enable(void)
{
	PK_DBG(" FL_Enable line=%d\n", __LINE__);
	return flashEnable_aw3644_1();
}



int FL_Disable(void)
{
	PK_DBG(" FL_Disable line=%d\n", __LINE__);
	return flashDisable_aw3644_1();
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	return setDuty_aw3644_1(duty);
}

int FL_Init(void)
{
	PK_DBG(" FL_Init line=%d\n", __LINE__);
	init_aw3644();
    return 0;
}

int FL_Uninit(void)
{
	FL_Disable();
    aw3644_hwen_off();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("LM3643 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
