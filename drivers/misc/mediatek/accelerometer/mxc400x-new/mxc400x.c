/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
 *
 * File Name          : mxc400x_acc.c
 * Description        : MXC400X accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *

 ******************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
	 
#include <accel.h>
#include <linux/batch.h>
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

#define POWER_NONE_MACRO MT65XX_POWER_NONE
//#define KK_OLDARCH 1
#define L_NewArch 1


#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_boot.h>
/*-------------------------MT6516&MT6573 define-------------------------------*/

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "mxc400x.h"
#include <linux/hwmsen_helper.h>


#define I2C_DRIVERID_MXC400X 120
#define DEBUG  1
#define SW_CALIBRATION
//#define SW_SIMULATION   0 
//#define SW_SIMULATION_DEBUG   0
 
#define MXC400X_AXIS_X          0
#define MXC400X_AXIS_Y          1
#define MXC400X_AXIS_Z          2
#define MXC400X_AXES_NUM        3
#define MXC400X_DATA_LEN        6

static s16 cali_sensor_data[MXC400X_AXES_NUM];

static const struct i2c_device_id mxc400x_i2c_id[] = { { MXC400X_DEV_NAME, 0 }, { }, };
static struct i2c_board_info __initdata i2c_mxc400x={ I2C_BOARD_INFO(MXC400X_DEV_NAME, MXC400X_I2C_ADDR)};
static int mxc400x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int mxc400x_i2c_remove(struct i2c_client *client);
static int mxc400x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#ifndef CONFIG_HAS_EARLYSUSPEND
static int mxc400x_suspend(struct i2c_client *client, pm_message_t msg);
static int mxc400x_resume(struct i2c_client *client);
#endif

static int  mxc400x_local_init(void);
static int mxc400x_remove(void);


typedef enum {
	ADX_TRC_FILTER  = 0x01,
	ADX_TRC_RAWDATA = 0x02,
	ADX_TRC_IOCTL	 = 0x04,
	ADX_TRC_CALI	 = 0X08,
	ADX_TRC_INFO	 = 0X10,
} ADX_TRC;

struct scale_factor{
	u8  whole;
	u8  fraction;
};

struct data_resolution {
	struct scale_factor scalefactor;
	int				 sensitivity;
};

#define C_MAX_FIR_LENGTH (32)

struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][MXC400X_AXES_NUM];
	int sum[MXC400X_AXES_NUM];
	int num;
	int idx;
};
static int mxc400x_init_flag = -1;
/*----------------------------------------------------------------------------*/
static struct acc_init_info mxc400x_init_info = {
		.name = "mxc400x",
		.init = mxc400x_local_init,
		.uninit = mxc400x_remove,
};
struct mxc400x_i2c_data {
		 struct i2c_client *client;
		 struct acc_hw *hw;
		 struct hwmsen_convert	 cvt;
		 atomic_t layout; 
		 /*misc*/
		 struct data_resolution *reso;
		 atomic_t				 trace;
		 atomic_t				 suspend;
		 atomic_t				 selftest;
		 atomic_t				 filter;
		 s16					 cali_sw[MXC400X_AXES_NUM+1];
	 
		 /*data*/
		 s16 					 offset[MXC400X_AXES_NUM+1];  /*+1: for 4-byte alignment*/
		 s16					 data[MXC400X_AXES_NUM+1];
#if defined(CONFIG_MXC400X_LOWPASS)
		 atomic_t				 firlen;
		 atomic_t				 fir_en;
		 struct data_filter 	 fir;
#endif 
		 /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
		 struct early_suspend	 early_drv;
#endif 
};

static struct i2c_driver mxc400x_i2c_driver = {
	 .driver = {
		// .owner		   = THIS_MODULE,
		 .name			 = MXC400X_DEV_NAME,
	 },
	 .probe 			 = mxc400x_i2c_probe,
	 .remove			 = mxc400x_i2c_remove,
	 .detect			 = mxc400x_i2c_detect,
	#if !defined(CONFIG_HAS_EARLYSUSPEND)    
	 .suspend			 = mxc400x_suspend,
	 .resume			 = mxc400x_resume,
	#endif
	 .id_table = mxc400x_i2c_id,
};



struct i2c_client      *mxc400x_i2c_client = NULL;
static struct mxc400x_i2c_data *obj_i2c_data = NULL;
static struct platform_driver mxc400x_gsensor_driver;
static bool sensor_power = true;
static GSENSOR_VECTOR3D gsensor_gain;
//static DEFINE_MUTEX(mxc400x_mutex);
static struct mutex mxc400x_mutex;
static bool enable_status = false;

extern struct acc_hw* get_cust_acc_hw(void);

#ifdef  DEBUG
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(GSE_TAG fmt, ##args)
#else
#define GSE_TAG
#define GSE_FUN(f)               do {} while (0)
#define GSE_ERR(fmt, args...)    do {} while (0)
#define GSE_LOG(fmt, args...)    do {} while (0)
#endif

static struct data_resolution mxc400x_data_resolution[] = {
    {{ 0, 9}, 1024},   /*+/-2g  in 12-bit resolution:  0.9 mg/LSB*/
    {{ 1, 9}, 512},   /*+/-4g  in 12-bit resolution:  1.9 mg/LSB*/
    {{ 3, 9},  256},   /*+/-8g  in 12-bit resolution: 3.9 mg/LSB*/      
};
static struct data_resolution mxc400x_offset_resolution = {{3, 9}, 256};


/**********************************************/
/**********SW SIMULATE FUNC********************/
/**********************************************/

#if defined(SW_SIMULATION)	//modified by lyon
#define MXC400X_I2C_SLAVE_WRITE_ADDR	0x2A	
static struct mutex g_gsensor_mutex;
#define GPIO_GSE_SDA_PIN 90
#define GPIO_GSE_SCL_PIN 89
#define GPIO_GSE_SDA_PIN_M_GPIO GPIO_MODE_00
#define GPIO_GSE_SCL_PIN_M_GPIO GPIO_MODE_00
void cust_i2c_scl_set( unsigned char mode)
{
	if(mt_set_gpio_mode(GPIO_GSE_SCL_PIN, GPIO_GSE_SDA_PIN_M_GPIO))
	{
		printk("MXC400X cust_i2c_scl_set mode failed \n");
	}
	if(mt_set_gpio_dir(GPIO_GSE_SCL_PIN, GPIO_DIR_OUT))
	{
		printk("MXC400X cust_i2c_scl_set dir failed \n");
	}
	if( mode == 1)
	{
		if(mt_set_gpio_out(GPIO_GSE_SCL_PIN, GPIO_OUT_ONE))
		{
			printk("MXC400X cust_i2c_scl_set high failed \n");
		}
	}
	else
	{
		if(mt_set_gpio_out(GPIO_GSE_SCL_PIN, GPIO_OUT_ZERO))
		{
			printk("MXC400X cust_i2c_scl_set low failed \n");
		}
	}
}

void cust_i2c_sda_set( unsigned char mode)
{
	if(mt_set_gpio_mode(GPIO_GSE_SDA_PIN, GPIO_GSE_SDA_PIN_M_GPIO))
	{
		printk("MXC400X cust_i2c_sda_set mode failed \n");
	}
	if(mt_set_gpio_dir(GPIO_GSE_SDA_PIN, GPIO_DIR_OUT))
	{
		printk("MXC400X cust_i2c_sda_set dir failed \n");
	}
	if( mode == 1)
	{
		if(mt_set_gpio_out(GPIO_GSE_SDA_PIN, GPIO_OUT_ONE))
		{
			printk("MXC400X cust_i2c_sda_set high failed \n");
		}
	}
	else
	{
		if(mt_set_gpio_out(GPIO_GSE_SDA_PIN, GPIO_OUT_ZERO))
		{
			printk("MXC400X cust_i2c_sda_set low failed \n");
		}
	}
}

void cust_i2c_sda_dir_set( unsigned char mode)
{	
	if(mt_set_gpio_mode(GPIO_GSE_SDA_PIN, GPIO_GSE_SDA_PIN_M_GPIO))
	{
		printk("MXC400X cust_i2c_sda_dir_set mode failed \n");
	}
	if( mode == GPIO_DIR_IN)
	{
		if(mt_set_gpio_dir(GPIO_GSE_SDA_PIN, GPIO_DIR_IN))
		{
			printk("MXC400X cust_i2c_sda_dir_set in failed \n");
		}
	}
	else
	{
		if(mt_set_gpio_dir(GPIO_GSE_SDA_PIN, GPIO_DIR_OUT))
		{
			printk("MXC400X cust_i2c_sda_dir_set out failed \n");
		}
	}
}

unsigned char cust_i2c_sda_get(void)
{
	if(mt_set_gpio_mode(GPIO_GSE_SDA_PIN, GPIO_GSE_SDA_PIN_M_GPIO))
	{
		printk("MXC400X cust_i2c_sda_get mode failed \n");
	}
	if(mt_set_gpio_dir(GPIO_GSE_SDA_PIN,GPIO_DIR_IN))
	{
		printk("MXC400X cust_i2c_sda_get dir failed \n");
	}
	
	return  mt_get_gpio_in(GPIO_GSE_SDA_PIN);
}

void cust_i2c_start(void)
{
	//printk("cust_i2c_start \n");
	cust_i2c_scl_set(1);
	cust_i2c_sda_set(1);
	udelay(5);
	cust_i2c_sda_set(0);
	udelay(5);
	cust_i2c_scl_set(0);
}

void cust_i2c_stop(void)
{
	//printk("cust_i2c_stop \n");
	cust_i2c_scl_set(1);
	cust_i2c_sda_set(0);
	udelay(5);
	cust_i2c_sda_set(1);
	udelay(5);
}

char cust_i2c_get_ack(void)
{
	unsigned char get_bit;
	unsigned char i;
	//printk("cust_i2c_get_ack \n");
	//cust_i2c_sda_set(1);
	//udelay(5);
	cust_i2c_sda_dir_set(GPIO_DIR_IN);
	cust_i2c_scl_set(1);
	udelay(5);
	//cust_i2c_sda_get();
	for(i=0; i<10; i++)
	{
		get_bit =  mt_get_gpio_in(GPIO_GSE_SDA_PIN);
		udelay(5);
		if(0 == get_bit)
		{
			break;
		}
	}
	cust_i2c_scl_set(0);
	cust_i2c_sda_dir_set(GPIO_DIR_OUT);
	if(i == 10)
	{
		return -1;
	}
	
	return 0;
}

char cust_i2c_send_ack(void)
{
	//printk("cust_i2c_send_ack \n");
	cust_i2c_sda_set(0);
	udelay(5);
	cust_i2c_scl_set(1);
	udelay(5);
	cust_i2c_scl_set(0);
	return 0;
}

void cust_i2c_no_ack(void)
{
	//printk("cust_i2c_send_ack \n");
	cust_i2c_sda_set(1);
	cust_i2c_scl_set(1);
	udelay(5);
	cust_i2c_scl_set(0);
	udelay(5);
}

void cust_i2c_send_byte( unsigned char udata)
{
	char i; 

	//printk("cust_i2c_send_byte \n",udata);
	for( i = 0; i<8;i++ )
	{
		if( ((udata>>(7-i)) & 0x01) == 0x01)
		{
			cust_i2c_sda_set(1);
		}
		else
		{
			cust_i2c_sda_set(0);
		}
		
		udelay(2);
		cust_i2c_scl_set(1);
		udelay(5);
		cust_i2c_scl_set(0);
	}	
		udelay(5);
}

unsigned char cust_i2c_get_byte( void )
{
	char i;
	unsigned char data;
	unsigned char get_bit;
	
	data = 0;
	//printk("cust_i2c_get_byte \n");
	cust_i2c_sda_dir_set(GPIO_DIR_IN);	
	for( i = 0; i<8; i++ )
	{
		cust_i2c_scl_set(1);
		udelay(5);
		//get_bit = cust_i2c_sda_get();
		get_bit =  mt_get_gpio_in(GPIO_GSE_SDA_PIN);
		cust_i2c_scl_set(0);
		udelay(5);
		if( 1 == (get_bit &0x01))
		{
			data |= get_bit <<(7-i);
		}
	}	
	udelay(5);
	return data;
}

char cust_i2c_write_byte(unsigned char addr, unsigned char regaddr, unsigned char udata)
{
	char res;

	cust_i2c_start();
	cust_i2c_send_byte(addr);
	res = cust_i2c_get_ack();
	if(0 != res)
	{
		printk("MXC400X cust_i2c_write_byte device addr error \n");
		return -1;
	}
	cust_i2c_send_byte(regaddr);
	res = cust_i2c_get_ack();
	if( 0 != res )
	{
		printk("MXC400X cust_i2c_write_byte reg addr error \n");
		return -1;
	}
	cust_i2c_send_byte(udata);
	res = cust_i2c_get_ack();
	if( 0 != res )
	{
		printk("MXC400X cust_i2c_write_byte reg data error \n");
		return -1;
	}
	cust_i2c_stop();
	return 0;
}

char cust_i2c_read_byte(unsigned char addr, unsigned char regaddr, unsigned char *udata)
{
	unsigned char data;
	char res;

	cust_i2c_start();
	cust_i2c_send_byte( addr );
	res = cust_i2c_get_ack();
	if( 0 != res )
	{
		printk("MXC400X cust_i2c_read_byte device addr error \n");
		return -1;
	}
	cust_i2c_send_byte( regaddr );
	res = cust_i2c_get_ack();
	if( 0 != res )
	{
		printk("MXC400X cust_i2c_read_byte reg addr error \n");
		return -1;
	}
	cust_i2c_start();
	addr |= 0x01;
	cust_i2c_send_byte( addr );
	res = cust_i2c_get_ack();
	if( 0 != res )
	{
		printk("MXC400X cust_i2c_read_byte devce addr error \n");
		return -1;
	}
	*udata = cust_i2c_get_byte();
	cust_i2c_no_ack();
	cust_i2c_stop();

	return 0;
}

char cust_i2c_write_bytes(unsigned char addr, unsigned char regaddr, unsigned char *udata, unsigned int count)
{
	u32 i;
	char res;

	cust_i2c_start();
	cust_i2c_send_byte( addr );
	res = cust_i2c_get_ack();
	if( 0 != res )
	{
		printk("MXC400X cust_i2c_write_bytes device addr error \n");
		return -1;
	}
	cust_i2c_send_byte( regaddr );
	res = cust_i2c_get_ack();
	if( 0 != res )
	{
		printk("MXC400X cust_i2c_write_bytes reg addr error \n");
		return -1;
	}
	for( i = 0; i< count; i++)
	{
		cust_i2c_send_byte(udata[i]);
		res = cust_i2c_get_ack();
		if(0 != res)
		{
			printk("MXC400X cust_i2c_write_bytes reg data error \n",__FUNCTION__,__LINE__);
			return -1;
		}
	}
	cust_i2c_stop();

	return 0;
}

char cust_i2c_read_bytes(unsigned char addr, unsigned char regaddr, unsigned char *udata,unsigned int count)
{
	unsigned char data;
	unsigned int i;
	char res;

	cust_i2c_start();
	cust_i2c_send_byte( addr );
	res = cust_i2c_get_ack();
	if( 0 != res )
	{
		printk("MXC400X cust_i2c_read_bytes device addr error \n");
		return -1;
	}
	cust_i2c_send_byte( regaddr );
	res = cust_i2c_get_ack();
	if( 0 != res )
	{
		printk("MXC400X cust_i2c_read_bytes reg addr error \n");
		return -1;
	}
	cust_i2c_start();
	addr |= 0x01;
	cust_i2c_send_byte( addr );
	res = cust_i2c_get_ack();
	if( 0 != res )
	{
		printk("MXC400X cust_i2c_read_bytes reg addr error \n");
		return -1;
	}

	for( i = 0; i < count-1; i++)
	{
		udata[i] = cust_i2c_get_byte();
		res = cust_i2c_send_ack();
		if(0 != res)
		{
			printk("MXC400X cust_i2c_read_bytes reg data error \n");
			return -1;
		}
	}
	udata[i] = cust_i2c_get_byte();
	cust_i2c_no_ack();

	cust_i2c_stop();

	return data;
}

int cust_i2c_master_send(struct i2c_client *client, u8 *buf, u8 count)
{
	u8 slave_addr;
	u8 reg_addr;

	mutex_lock(&g_gsensor_mutex);
	slave_addr = MXC400X_I2C_SLAVE_WRITE_ADDR ;	
	reg_addr = buf[0];
#if defined(SW_SIMULATION_DEBUG)
	printk("MXC400X cust_i2c_master_send_byte reg_addr=0x x% \n", reg_addr);
#endif
	cust_i2c_write_bytes(slave_addr,reg_addr, &buf[1],count-1);
	mutex_unlock(&g_gsensor_mutex);

	return count;
}

int cust_i2c_master_read(struct i2c_client *client, u8 *buf, u8 count)
{
	u8 slave_addr;
	u8 reg_addr;

	slave_addr = MXC400X_I2C_SLAVE_WRITE_ADDR ;	
	reg_addr = buf[0];
#if defined(SW_SIMULATION_DEBUG)
	printk("MXC400X cust_i2c_master_read_byte reg_addr=0x %x\n", reg_addr);
#endif
	cust_i2c_read_bytes(slave_addr,reg_addr, &buf[0],count);

	return count;
}

int cust_hwmsen_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 slave_addr;
	u8 reg_addr;

	mutex_lock(&g_gsensor_mutex);
	slave_addr = MXC400X_I2C_SLAVE_WRITE_ADDR;	
	reg_addr = addr;
#if defined(SW_SIMULATION_DEBUG)
	printk("MXC400X cust_hwmsen_write_block reg_addr=0x%x\n", reg_addr);
#endif
	cust_i2c_write_bytes(slave_addr,reg_addr, data,len);
	mutex_unlock(&g_gsensor_mutex);

	return 0;
}

int cust_hwmsen_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 buf[64] = {0};
	mutex_lock(&g_gsensor_mutex);
	buf[0] = addr;
	cust_i2c_master_read(client, buf, len);
	mutex_unlock(&g_gsensor_mutex);
#if defined(SW_SIMULATION_DEBUG)	
	printk("MXC400X cust_hwmsen_read_block addr=0x%x, buf=0x%x\n", addr, buf[0]);
#endif
	memcpy(data, buf, len);
	return 0;
}
#endif

/****************************************/
/**********SW SIMULATE FUNC**************/
/****************************************/

static int mxc400x_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
	struct i2c_msg msgs[2] = {
	{
		.addr = client->addr,	.flags = 0,
		.len = 1,	.buf = &beg
	},
	{
		.addr = client->addr,	.flags = I2C_M_RD,
		.len = len,	.buf = data,
	}
	};
	int err;

	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	return err;

}
static void mxc400x_power(struct acc_hw *hw, unsigned int on) 
{
	 static unsigned int power_on = 0;

	 if(hw->power_id != POWER_NONE_MACRO)		 // have externel LDO
	 {		  
		 GSE_LOG("power %s\n", on ? "on" : "off");
		 if(power_on == on)  // power status not change
		 {
			 GSE_LOG("ignore power control: %d\n", on);
		 }
		 else if(on) // power on
		 {
			 if(!hwPowerOn(hw->power_id, hw->power_vol, "MXC400X"))
			 {
				 GSE_ERR("power on fails!!\n");
			 }
		 }
		 else	 // power off
		 {
			 if (!hwPowerDown(hw->power_id, "MXC400X"))
			 {
				 GSE_ERR("power off fail!!\n");
			 }			   
		 }
	 }
	 power_on = on;    
}



static int MXC400X_SaveData(int buf[MXC400X_AXES_NUM])
{ 
   
	mutex_lock(&mxc400x_mutex);
//    memcpy(cali_sensor_data, buf, sizeof(cali_sensor_data));
	cali_sensor_data[0] = buf[0];
	cali_sensor_data[1] = buf[1];
	cali_sensor_data[2] = buf[2];
	
	mutex_unlock(&mxc400x_mutex);

    return 0;
}
static int  MXC400X_RxData(struct i2c_client *client, s16 data[MXC400X_AXES_NUM])
{
	u8 addr = MXC400X_REG_X;
	u8 buf[MXC400X_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
        GSE_LOG("client is null\n");
		err = -EINVAL;
	}
#if defined(SW_SIMULATION)
	else if(err = cust_hwmsen_read_block(client, addr, buf, MXC400X_DATA_LEN))
#else
	else if(err = mxc400x_i2c_read_block(client, addr, buf, MXC400X_DATA_LEN))
#endif
	{
		GSE_ERR("error: %d\n", err);
	}
	else 
    {
        
	 	data[MXC400X_AXIS_X] = (s16)(buf[0] << 8 | buf[1]) >> 4;
		data[MXC400X_AXIS_Y] = (s16)(buf[2] << 8 | buf[3]) >> 4;
		data[MXC400X_AXIS_Z] = (s16)(buf[4] << 8 | buf[5]) >> 4;
       
	}

	return err;	
}
static int MXC400X_ReadTemp(struct i2c_client *client, s8 *data)
{     
	u8 addr = MXC400X_REG_TEMP;
	int err = 0;
    s8 temp = 0;

	if(NULL == client)
	{
        GSE_LOG("client is null\n");
		err = -EINVAL;
	}
#if defined(SW_SIMULATION)
	else if(err = cust_hwmsen_read_block(client, addr, &temp, 0x01))
#else
	else if(err = mxc400x_i2c_read_block(client, addr, &temp, 1))
#endif
	{
		GSE_ERR("error: %d\n", err);
	}
    *data = temp;
  
	return err;
}
static int MXC400X_ReadDant(struct i2c_client *client, int dant[MXC400X_AXES_NUM+1])
{
	u8 addr = MXC400X_REG_X;
	u8 buf[MXC400X_DATA_LEN+1] = {0};
	int err = 0;

	if(NULL == client)
	{
        GSE_LOG("client is null\n");
		err = -EINVAL;
	}
#if defined(SW_SIMULATION)
	else if(err = cust_hwmsen_read_block(client, addr, buf, MXC400X_DATA_LEN+1))
#else
	else if(err = mxc400x_i2c_read_block(client, addr, buf, MXC400X_DATA_LEN+1))
#endif
	{
		GSE_ERR("error: %d\n", err);
	}
	else 
    {        
   
		dant[MXC400X_AXIS_X] = (s16)((s16)buf[0] << 8 | (s16)buf[1]) >> 4;
		dant[MXC400X_AXIS_Y] = (s16)((s16)buf[2] << 8 | (s16)buf[3]) >> 4;
		dant[MXC400X_AXIS_Z] = (s16)((s16)buf[4] << 8 | (s16)buf[5]) >> 4;
		dant[MXC400X_AXIS_Z+1] = (s16)buf[6];

	   
	}
    
	return err;	
}
static int MXC400X_Test1(struct i2c_client *client, u8 data)
{

  	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MXC400X_REG_TEST1;    
	databuf[1] = data;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return MXC400X_ERR_I2C;
	}

    return 0;
}
static int MXC400X_Test2(struct i2c_client *client, u8 data)
{

  	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MXC400X_REG_TEST2;    
	databuf[1] = data;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return MXC400X_ERR_I2C;
	}

    return 0;
}
static int MXC400X_Test3(struct i2c_client *client, u8 data)
{

  	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MXC400X_REG_TEST3;    
	databuf[1] = data;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return MXC400X_ERR_I2C;
	}

    return 0;
}

static int MXC400X_SetDataResolution(struct mxc400x_i2c_data *obj)
{
 	obj->reso = &mxc400x_data_resolution[2];
	return MXC400X_SUCCESS;
}
static int MXC400X_ReadData(struct i2c_client *client, s16 data[MXC400X_AXES_NUM])
{
	struct mxc400x_i2c_data *priv = i2c_get_clientdata(client);        
	u8 addr = MXC400X_REG_X;
	u8 buf[MXC400X_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
        GSE_LOG("client is null\n");
		err = -EINVAL;
	}
#if defined(SW_SIMULATION)
	else if(err = cust_hwmsen_read_block(client, addr, buf, MXC400X_DATA_LEN))
#else
	else if((err = mxc400x_i2c_read_block(client, addr, buf, MXC400X_DATA_LEN))!=0)
#endif
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		data[MXC400X_AXIS_X] = (s16)(buf[0] << 8 | buf[1]) >> 4;
		data[MXC400X_AXIS_Y] = (s16)(buf[2] << 8 | buf[3]) >> 4;
		data[MXC400X_AXIS_Z] = (s16)(buf[4] << 8 | buf[5]) >> 4;
		GSE_LOG("reg data x = %d %d y = %d %d z = %d %d\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

#if 0
		if(data[MXC400X_AXIS_X]&0x80)
		{
			 data[MXC400X_AXIS_X] = ~data[MXC400X_AXIS_X];
			 data[MXC400X_AXIS_X] &= 0xff;
			 data[MXC400X_AXIS_X]+=1;
			 data[MXC400X_AXIS_X] = -data[MXC400X_AXIS_X];
		}
		if(data[MXC400X_AXIS_Y]&0x80)
		{
			 data[MXC400X_AXIS_Y] = ~data[MXC400X_AXIS_Y];
			 data[MXC400X_AXIS_Y] &= 0xff;
			 data[MXC400X_AXIS_Y]+=1;
			 data[MXC400X_AXIS_Y] = -data[MXC400X_AXIS_Y];
		}

        if(data[MXC400X_AXIS_Z]&0x80)
		{
				data[MXC400X_AXIS_Z] = ~data[MXC400X_AXIS_Z];
				data[MXC400X_AXIS_Z] &= 0xff;
				data[MXC400X_AXIS_Z]+=1;
				data[MXC400X_AXIS_Z] = -data[MXC400X_AXIS_Z];
		}
		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
		 	GSE_LOG("[%08X %08X] => [%5d %5d]\n", data[MXC400X_AXIS_X], data[MXC400X_AXIS_Y],
									data[MXC400X_AXIS_X], data[MXC400X_AXIS_Y]);
		}

#endif 
#ifdef CONFIG_MXC400X_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][MXC400X_AXIS_X] = data[MXC400X_AXIS_X];
					priv->fir.raw[priv->fir.num][MXC400X_AXIS_Y] = data[MXC400X_AXIS_Y];
					priv->fir.raw[priv->fir.num][MXC400X_AXIS_Z] = data[MXC400X_AXIS_Z];
					priv->fir.sum[MXC400X_AXIS_X] += data[MXC400X_AXIS_X];
					priv->fir.sum[MXC400X_AXIS_Y] += data[MXC400X_AXIS_Y];
					priv->fir.sum[MXC400X_AXIS_Z] += data[MXC400X_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][MXC400X_AXIS_X], priv->fir.raw[priv->fir.num][MXC400X_AXIS_Y], priv->fir.raw[priv->fir.num][MXC400X_AXIS_Z],
							priv->fir.sum[MXC400X_AXIS_X], priv->fir.sum[MXC400X_AXIS_Y], priv->fir.sum[MXC400X_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[MXC400X_AXIS_X] -= priv->fir.raw[idx][MXC400X_AXIS_X];
					priv->fir.sum[MXC400X_AXIS_Y] -= priv->fir.raw[idx][MXC400X_AXIS_Y];
					priv->fir.sum[MXC400X_AXIS_Z] -= priv->fir.raw[idx][MXC400X_AXIS_Z];
					priv->fir.raw[idx][MXC400X_AXIS_X] = data[MXC400X_AXIS_X];
					priv->fir.raw[idx][MXC400X_AXIS_Y] = data[MXC400X_AXIS_Y];
					priv->fir.raw[idx][MXC400X_AXIS_Z] = data[MXC400X_AXIS_Z];
					priv->fir.sum[MXC400X_AXIS_X] += data[MXC400X_AXIS_X];
					priv->fir.sum[MXC400X_AXIS_Y] += data[MXC400X_AXIS_Y];
					priv->fir.sum[MXC400X_AXIS_Z] += data[MXC400X_AXIS_Z];
					priv->fir.idx++;
					data[MXC400X_AXIS_X] = priv->fir.sum[MXC400X_AXIS_X]/firlen;
					data[MXC400X_AXIS_Y] = priv->fir.sum[MXC400X_AXIS_Y]/firlen;
					data[MXC400X_AXIS_Z] = priv->fir.sum[MXC400X_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][MXC400X_AXIS_X], priv->fir.raw[idx][MXC400X_AXIS_Y], priv->fir.raw[idx][MXC400X_AXIS_Z],
						priv->fir.sum[MXC400X_AXIS_X], priv->fir.sum[MXC400X_AXIS_Y], priv->fir.sum[MXC400X_AXIS_Z],
						data[MXC400X_AXIS_X], data[MXC400X_AXIS_Y], data[MXC400X_AXIS_Z]);
					}
				}
			}
		}	
#endif 
	}
	return err;
}


static int MXC400X_ReadOffset(struct i2c_client *client, s8 ofs[MXC400X_AXES_NUM])
{    
	int err;
	err = 0;
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#endif
	return err;    
}

static int MXC400X_ResetCalibration(struct i2c_client *client)
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	err = 0;

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;    
}

static int MXC400X_ReadCalibration(struct i2c_client *client, int dat[MXC400X_AXES_NUM])
{
    struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
    int  err = 0;
    int mul;

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
	    if ((err = MXC400X_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    	}    
    	mul = obj->reso->sensitivity/mxc400x_offset_resolution.sensitivity;
	#endif

    dat[obj->cvt.map[MXC400X_AXIS_X]] = obj->cvt.sign[MXC400X_AXIS_X]*(obj->offset[MXC400X_AXIS_X]*mul + obj->cali_sw[MXC400X_AXIS_X]);
    dat[obj->cvt.map[MXC400X_AXIS_Y]] = obj->cvt.sign[MXC400X_AXIS_Y]*(obj->offset[MXC400X_AXIS_Y]*mul + obj->cali_sw[MXC400X_AXIS_Y]);
    dat[obj->cvt.map[MXC400X_AXIS_Z]] = obj->cvt.sign[MXC400X_AXIS_Z]*(obj->offset[MXC400X_AXIS_Z]*mul + obj->cali_sw[MXC400X_AXIS_Z]);
                                       
    return err;
}
static int MXC400X_ReadCalibrationEx(struct i2c_client *client, int act[MXC400X_AXES_NUM], int raw[MXC400X_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;
	err = 0;
 

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
		if((err = MXC400X_ReadOffset(client, obj->offset)))
		{
			GSE_ERR("read offset fail, %d\n", err);
			return err;
		}   
		mul = obj->reso->sensitivity/mxc400x_offset_resolution.sensitivity;
	#endif
	
	raw[MXC400X_AXIS_X] = obj->offset[MXC400X_AXIS_X]*mul + obj->cali_sw[MXC400X_AXIS_X];
	raw[MXC400X_AXIS_Y] = obj->offset[MXC400X_AXIS_Y]*mul + obj->cali_sw[MXC400X_AXIS_Y];
	raw[MXC400X_AXIS_Z] = obj->offset[MXC400X_AXIS_Z]*mul + obj->cali_sw[MXC400X_AXIS_Z];

	act[obj->cvt.map[MXC400X_AXIS_X]] = obj->cvt.sign[MXC400X_AXIS_X]*raw[MXC400X_AXIS_X];
	act[obj->cvt.map[MXC400X_AXIS_Y]] = obj->cvt.sign[MXC400X_AXIS_Y]*raw[MXC400X_AXIS_Y];
	act[obj->cvt.map[MXC400X_AXIS_Z]] = obj->cvt.sign[MXC400X_AXIS_Z]*raw[MXC400X_AXIS_Z];                        
	                       
	return 0;
}

static int MXC400X_WriteCalibration(struct i2c_client *client, int dat[MXC400X_AXES_NUM])
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[MXC400X_AXES_NUM], raw[MXC400X_AXES_NUM];
	int lsb; 
	lsb = mxc400x_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;

	if((err = MXC400X_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[MXC400X_AXIS_X], raw[MXC400X_AXIS_Y], raw[MXC400X_AXIS_Z],
		obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z],
		obj->cali_sw[MXC400X_AXIS_X], obj->cali_sw[MXC400X_AXIS_Y], obj->cali_sw[MXC400X_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[MXC400X_AXIS_X] += dat[MXC400X_AXIS_X];
	cali[MXC400X_AXIS_Y] += dat[MXC400X_AXIS_Y];
	cali[MXC400X_AXIS_Z] += dat[MXC400X_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[MXC400X_AXIS_X], dat[MXC400X_AXIS_Y], dat[MXC400X_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[MXC400X_AXIS_X] = obj->cvt.sign[MXC400X_AXIS_X]*(cali[obj->cvt.map[MXC400X_AXIS_X]]);
	obj->cali_sw[MXC400X_AXIS_Y] = obj->cvt.sign[MXC400X_AXIS_Y]*(cali[obj->cvt.map[MXC400X_AXIS_Y]]);
	obj->cali_sw[MXC400X_AXIS_Z] = obj->cvt.sign[MXC400X_AXIS_Z]*(cali[obj->cvt.map[MXC400X_AXIS_Z]]);	
#else
	int divisor = obj->reso->sensitivity/lsb;//modified
	obj->offset[MXC400X_AXIS_X] = (s8)(obj->cvt.sign[MXC400X_AXIS_X]*(cali[obj->cvt.map[MXC400X_AXIS_X]])/(divisor));
	obj->offset[MXC400X_AXIS_Y] = (s8)(obj->cvt.sign[MXC400X_AXIS_Y]*(cali[obj->cvt.map[MXC400X_AXIS_Y]])/(divisor));
	obj->offset[MXC400X_AXIS_Z] = (s8)(obj->cvt.sign[MXC400X_AXIS_Z]*(cali[obj->cvt.map[MXC400X_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[MXC400X_AXIS_X] = obj->cvt.sign[MXC400X_AXIS_X]*(cali[obj->cvt.map[MXC400X_AXIS_X]])%(divisor);
	obj->cali_sw[MXC400X_AXIS_Y] = obj->cvt.sign[MXC400X_AXIS_Y]*(cali[obj->cvt.map[MXC400X_AXIS_Y]])%(divisor);
	obj->cali_sw[MXC400X_AXIS_Z] = obj->cvt.sign[MXC400X_AXIS_Z]*(cali[obj->cvt.map[MXC400X_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[MXC400X_AXIS_X]*divisor + obj->cali_sw[MXC400X_AXIS_X], 
		obj->offset[MXC400X_AXIS_Y]*divisor + obj->cali_sw[MXC400X_AXIS_Y], 
		obj->offset[MXC400X_AXIS_Z]*divisor + obj->cali_sw[MXC400X_AXIS_Z], 
		obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z],
		obj->cali_sw[MXC400X_AXIS_X], obj->cali_sw[MXC400X_AXIS_Y], obj->cali_sw[MXC400X_AXIS_Z]);

#endif
	mdelay(1);
	return err;
}

static int MXC400X_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MXC400X_REG_ID;   
#if defined(SW_SIMULATION)
	res = cust_i2c_master_read(client, databuf, 0x1);
#else
	res = i2c_master_send(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_MXC400X_CheckDeviceID;
	}
	udelay(500);

	databuf[0] = 0x0;        
	res = i2c_master_recv(client, databuf, 0x01);
#endif
	if(res <= 0)
	{
		goto exit_MXC400X_CheckDeviceID;
	}

	
	databuf[0]= (databuf[0]&0x3f);
	
	if(databuf[0]!= MXC400X_ID)
	{
		return MXC400X_ERR_IDENTIFICATION;
	}
	
	GSE_LOG("MXC400X_CheckDeviceID %d done!\n ", databuf[0]);

	return MXC400X_SUCCESS;

	exit_MXC400X_CheckDeviceID:
	if (res <= 0)
	{
		GSE_ERR("MXC400X_CheckDeviceID %d failt!\n ", MXC400X_ERR_I2C);
		return MXC400X_ERR_I2C;
	}

}

static int MXC400X_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	
	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return MXC400X_SUCCESS;
	}
	if(enable == 1)
	{
		databuf[1] = MXC400X_AWAKE;
	}
	else
	{
		databuf[1] = MXC400X_SLEEP;
	}
	databuf[0] = MXC400X_REG_CTRL;
#if defined(SW_SIMULATION)
	res = cust_i2c_master_send(client, databuf, 0x2);
#else	
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if(res <= 0)
	{
		GSE_LOG("set power mode failed!\n");
		return MXC400X_ERR_I2C;
	}
	sensor_power = enable;
	mdelay(300);//lyon
	
	return MXC400X_SUCCESS;    
}
static int MXC400X_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);  
	databuf[0] = MXC400X_REG_CTRL;
	databuf[1] = MXC400X_RANGE_8G;
#if defined(SW_SIMULATION)
	res = cust_i2c_master_send(client, databuf, 0x2);
#else
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if(res <= 0)
	{
		GSE_LOG("set power mode failed!\n");
		return MXC400X_ERR_I2C;
	}  

	return MXC400X_SetDataResolution(obj);    
}
static int MXC400X_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	
	return MXC400X_SUCCESS;    
}

static int mxc400x_init_client(struct i2c_client *client, int reset_cali)
{
	 struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	 int res = 0;

	 res = MXC400X_CheckDeviceID(client); 
	 if(res != MXC400X_SUCCESS)
	 {
	 	 return res;
	 }	 
	 
	res = MXC400X_SetBWRate(client, MXC400X_BW_50HZ);
	if(res != MXC400X_SUCCESS ) 
	{
		return res;
	}

	res = MXC400X_SetDataFormat(client, MXC400X_RANGE_8G);
	if(res != MXC400X_SUCCESS) 
	{
		return res;
	}
	
	res = MXC400X_SetPowerMode(client, enable_status);
	if(res != MXC400X_SUCCESS)
	{
		return res;
	}
	
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;
	 
	if(0 != reset_cali)
	{ 
	 	/*reset calibration only in power on*/
		 res = MXC400X_ResetCalibration(client);
		 if(res != MXC400X_SUCCESS)
		 {
			 return res;
		 }
	}
	GSE_LOG("mxc400x_init_client OK!\n");
#ifdef CONFIG_MXC400X_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif
	mdelay(20);

	return MXC400X_SUCCESS;
}

static int MXC400X_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "MXC400X Chip");
	return 0;
}

static int MXC400X_ReadSensorData(struct i2c_client *client, int *buf, int bufsize)
{
	struct mxc400x_i2c_data *obj = (struct mxc400x_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[MXC400X_AXES_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
        GSE_ERR("buf is null !!!\n");
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
         GSE_ERR("client is null !!!\n");
		return MXC400X_ERR_STATUS;
	}

	if (atomic_read(&obj->suspend))
	{
		GSE_LOG("sensor in suspend read not data!\n");
		return MXC400X_ERR_GETGSENSORDATA;
	}
     
	if(res = MXC400X_ReadData(client, obj->data))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return MXC400X_ERR_I2C;
	}
	else
	{   
        
		//GSE_LOG("raw data x=%d, y=%d, z=%d \n",obj->data[MXC400X_AXIS_X],obj->data[MXC400X_AXIS_Y],obj->data[MXC400X_AXIS_Z]);
		obj->data[MXC400X_AXIS_X] += obj->cali_sw[MXC400X_AXIS_X];
		obj->data[MXC400X_AXIS_Y] += obj->cali_sw[MXC400X_AXIS_Y];
		obj->data[MXC400X_AXIS_Z] += obj->cali_sw[MXC400X_AXIS_Z];

		//printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[MXC400X_AXIS_X],obj->cali_sw[MXC400X_AXIS_Y],obj->cali_sw[MXC400X_AXIS_Z]);
		
		/*remap coordinate*/
		acc[obj->cvt.map[MXC400X_AXIS_X]] = obj->cvt.sign[MXC400X_AXIS_X]*obj->data[MXC400X_AXIS_X];
		acc[obj->cvt.map[MXC400X_AXIS_Y]] = obj->cvt.sign[MXC400X_AXIS_Y]*obj->data[MXC400X_AXIS_Y];
		acc[obj->cvt.map[MXC400X_AXIS_Z]] = obj->cvt.sign[MXC400X_AXIS_Z]*obj->data[MXC400X_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[MXC400X_AXIS_X],obj->cvt.sign[MXC400X_AXIS_Y],obj->cvt.sign[MXC400X_AXIS_Z]);
        
		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[MXC400X_AXIS_X], acc[MXC400X_AXIS_Y], acc[MXC400X_AXIS_Z]);
  
		//Out put the mg
		//printk("mg acc 1 x =%d, GRAVITY=%d, sensityvity=%d \n",acc[MXC400X_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[MXC400X_AXIS_X] = acc[MXC400X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[MXC400X_AXIS_Y] = acc[MXC400X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[MXC400X_AXIS_Z] = acc[MXC400X_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;	
        //printk("mg acc 2 x =%d, y =%d, z =%d \n",acc[MXC400X_AXIS_X],acc[MXC400X_AXIS_Y],acc[MXC400X_AXIS_Z]);
        buf[0] = acc[MXC400X_AXIS_X];
        buf[1] = acc[MXC400X_AXIS_Y];
        buf[2] = acc[MXC400X_AXIS_Z];
		//sprintf(buf, "%04x %04x %04x", acc[MXC400X_AXIS_X], acc[MXC400X_AXIS_Y], acc[MXC400X_AXIS_Z]);
		//printk("mg acc 3 x =%d, y =%d, z =%d \n",buf[MXC400X_AXIS_X],buf[MXC400X_AXIS_Y],buf[MXC400X_AXIS_Z]);
	//	if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
	//	{
	//		GSE_LOG("gsensor data: %s!\n", buf);
	//	}

	}
	
	return res;
}

static int MXC400X_ReadRawData(struct i2c_client *client, int *buf)
{
	struct mxc400x_i2c_data *obj = (struct mxc400x_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
        GSE_ERR(" buf or client is null !!\n");
		return EINVAL;
	}
	
	if(res = MXC400X_ReadData(client, obj->data))
	{        
		GSE_ERR("I2C error: ret value=%d\n", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "MXC400X_ReadRawData %04x %04x %04x", obj->data[MXC400X_AXIS_X], 
			obj->data[MXC400X_AXIS_Y], obj->data[MXC400X_AXIS_Z]);
	
	}
	
	return 0;
}


static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;
	char strbuf[MXC400X_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	MXC400X_ReadChipInfo(client, strbuf, MXC400X_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", (char*)strbuf);        
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;
	int strbuf[MXC400X_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	MXC400X_ReadSensorData(client, strbuf, MXC400X_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%d %d %d \n", strbuf[0],strbuf[1],strbuf[2]);
	//return snprintf(buf, PAGE_SIZE, "%s\n", (char*)strbuf);            
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;
	struct mxc400x_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[MXC400X_AXES_NUM];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return MXC400X_ERR_STATUS;
	}

	obj = i2c_get_clientdata(client);



	if((err = MXC400X_ReadOffset(client, obj->offset)))
	{
		return -EINVAL;
	}
	else if((err = MXC400X_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/mxc400x_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z],
			obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[MXC400X_AXIS_X], obj->cali_sw[MXC400X_AXIS_Y], obj->cali_sw[MXC400X_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[MXC400X_AXIS_X]*mul + obj->cali_sw[MXC400X_AXIS_X],
			obj->offset[MXC400X_AXIS_Y]*mul + obj->cali_sw[MXC400X_AXIS_Y],
			obj->offset[MXC400X_AXIS_Z]*mul + obj->cali_sw[MXC400X_AXIS_Z],
			tmp[MXC400X_AXIS_X], tmp[MXC400X_AXIS_Y], tmp[MXC400X_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = mxc400x_i2c_client;  
	int err, x, y, z;
	int dat[MXC400X_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(err = MXC400X_ResetCalibration(client))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[MXC400X_AXIS_X] = x;
		dat[MXC400X_AXIS_Y] = y;
		dat[MXC400X_AXIS_Z] = z;
		if((err = MXC400X_WriteCalibration(client, dat)))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return 0;
}
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_MXC400X_LOWPASS
	struct i2c_client *client = mxc400x_i2c_client;
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
	 	int idx, len = atomic_read(&obj->firlen);
	 	GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][MXC400X_AXIS_X], obj->fir.raw[idx][MXC400X_AXIS_Y], obj->fir.raw[idx][MXC400X_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[MXC400X_AXIS_X], obj->fir.sum[MXC400X_AXIS_Y], obj->fir.sum[MXC400X_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[MXC400X_AXIS_X]/len, obj->fir.sum[MXC400X_AXIS_Y]/len, obj->fir.sum[MXC400X_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_MXC400X_LOWPASS
	struct i2c_client *client = mxc400x_i2c_client;  
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return 0;
}
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mxc400x_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mxc400x_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	return count;    
}
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct mxc400x_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	
	if(sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, PAGE_SIZE, "%x\n", sensor_power);
}
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;  
	struct mxc400x_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = mxc400x_i2c_client;  
	struct mxc400x_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}
	
	return count;            
}
static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(firlen, 	S_IWUSR | S_IRUGO, show_firlen_value,		 store_firlen_value);
static DRIVER_ATTR(trace,		S_IWUSR | S_IRUGO, show_trace_value,		 store_trace_value);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status, 			  S_IRUGO, show_status_value,		 NULL);
static DRIVER_ATTR(powerstatus,			   S_IRUGO, show_power_status_value,		NULL);

static struct driver_attribute *mxc400x_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,	   /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,		   /*trace log*/
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_powerstatus,
};
static int mxc400x_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(mxc400x_attr_list)/sizeof(mxc400x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, mxc400x_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", mxc400x_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
static int mxc400x_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(mxc400x_attr_list)/sizeof(mxc400x_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, mxc400x_attr_list[idx]);
	}
	
	return err;
}
int gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
			 void* buff_out, int size_out, int* actualout)
{
	 int err = 0;
	 int value, sample_delay;	 
	 struct mxc400x_i2c_data *priv = (struct mxc400x_i2c_data*)self;
	 hwm_sensor_data* gsensor_data;
	 int buff[MXC400X_BUFSIZE] = {0};
 
	 //GSE_FUN(f);
	 switch (command)
	 {
		 case SENSOR_DELAY:
			 GSE_LOG("mxc400x set delay\n");
			 break;

		 case SENSOR_ENABLE:
			 if((buff_in == NULL) || (size_in < sizeof(int)))
			 {
				 GSE_ERR("Enable sensor parameter error!\n");
				 err = -EINVAL;
			 }
			 else
			 {
				value = *(int *)buff_in;
				mutex_lock(&mxc400x_mutex);
                GSE_LOG("Gsensor device enable function enable = %d, sensor_power = %d!\n",value,sensor_power);
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					enable_status = sensor_power;
					GSE_LOG("Gsensor device have updated !\n");
				}
				else
				{
					enable_status = !sensor_power;
					if (atomic_read(&priv->suspend) == 0)
					{
						
						err = MXC400X_SetPowerMode(priv->client, enable_status);
						GSE_LOG("Gsensor not in suspend MXC400X_SetPowerMode!, enable_status = %d\n",enable_status);
					}
					else
					{
					
						GSE_LOG("Gsensor in suspend and can not enable or disable!enable_status = %d\n",enable_status);
					}
					
				}
				
				mutex_unlock(&mxc400x_mutex);
			 }
			 break;

		 case SENSOR_GET_DATA:
			 if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			 {
				 GSE_ERR("get sensor data parameter error!\n");
				 err = -EINVAL;
			 }
			 else
			 {
				 gsensor_data = (hwm_sensor_data *)buff_out;
				 mutex_lock(&mxc400x_mutex);
				 MXC400X_ReadSensorData(priv->client, buff, MXC400X_DATA_LEN);
				 gsensor_data->values[0] = buff[0];
				 gsensor_data->values[1] = buff[1];
				 gsensor_data->values[2] = buff[2];
				 
                 //gsensor_data->values[0] = cali_sensor_data[0];
				 //gsensor_data->values[1] = cali_sensor_data[1];
				 //gsensor_data->values[2] = cali_sensor_data[2];
				 mutex_unlock(&mxc400x_mutex);
                 //sscanf(buff, "%x %x %x", &gsensor_data->values[0], &gsensor_data->values[1], &gsensor_data->values[2]);
                 GSE_LOG("Gsensor DATA X = %d, Y = %d, Z = %d!\n",gsensor_data->values[0],gsensor_data->values[1],gsensor_data->values[2]);
				 gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				 
				 gsensor_data->value_divide = 1000;
		
			 }
			 break;
		 default:
			 GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			 err = -1;
			 break;
	 }
 
	return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int mxc400x_open(struct inode *inode, struct file *file)
{
	 file->private_data = mxc400x_i2c_client;

	 if(file->private_data == NULL)
	 {
		 GSE_ERR("null mxc400x!!\n");
		 return -EINVAL;
	 }
	 return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int mxc400x_release(struct inode *inode, struct file *file)
{
	 file->private_data = NULL;
	 return 0;
}

#ifdef CONFIG_COMPAT
static long mxc400x_compat_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    long err = 0;

	void __user *arg64 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
	switch (cmd)
	{
		case COMPAT_GSENSOR_IOCTL_INIT:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_INIT, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_TM is failed!\n");
				}
				break;
		case COMPAT_GSENSOR_IOCTL_READ_CHIPINFO:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_CHIPINFO, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_READ_CHIPINFO is failed!\n");
				}
				break;
		case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_READ_SENSORDATA is failed!\n");
				}
				break;
		case COMPAT_GSENSOR_IOCTL_READ_GAIN:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_GAIN, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_READ_GAIN is failed!\n");
				}
				break;
		case COMPAT_GSENSOR_IOCTL_READ_RAW_DATA:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_RAW_DATA, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_READ_RAW_DATA is failed!\n");
				}
				break;

		case COMPAT_GSENSOR_IOCTL_SET_CALI:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_SET_CALI is failed!\n");
				}
				break;
		case COMPAT_GSENSOR_IOCTL_CLR_CALI:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_CLR_CALI is failed!\n");
				}
				break;

		case COMPAT_GSENSOR_IOCTL_GET_CALI:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_CALI is failed!\n");
				}
				break;
		case COMPAT_GSENSOR_IOCTL_GET_DELAY:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_DELAY, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_DELAY is failed!\n");
				}
				break;

		case COMPAT_GSENSOR_IOCTL_GET_STATUS:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_STATUS, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_STATUS is failed!\n");
				}
				break;
		case COMPAT_GSENSOR_IOCTL_GET_DATA:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_DATA, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_DATA is failed!\n");
				}
				break;

		case COMPAT_GSENSOR_IOCTL_SET_DATA:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_DATA, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_SET_DATA is failed!\n");
				}
				break;
		case COMPAT_GSENSOR_IOCTL_GET_TEMP:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_TEMP, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_TEMP is failed!\n");
				}
				break;

		case COMPAT_GSENSOR_IOCTL_GET_DANT:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_DANT, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_DANT is failed!\n");
				}
				break;
		case COMPAT_GSENSOR_IOCTL_READ_REG:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_REG, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_READ_REG is failed!\n");
				}
				break;

		case COMPAT_GSENSOR_IOCTL_WRITE_REG:
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_WRITE_REG, (unsigned long)arg64);
			if(err < 0)
				{
					printk(KERN_ERR "COMPAT_GSENSOR_IOCTL_WRITE_REG is failed!\n");
				}
				break;
		default:
			 printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			 return -ENOIOCTLCMD;
			 break;
	}
	return 0;
}
#endif
static long mxc400x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct mxc400x_i2c_data *obj = (struct mxc400x_i2c_data*)i2c_get_clientdata(client);  
	int strbuf[MXC400X_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3];
        int value[3] = {0};
        int vec[3] = {0};
        s8 temp = 0 ;
	int dant[4] = {0};
	u8 test=0;
	u8 buf;
	u8 reg[2];


	//printk(KERN_INFO "%s: %s call with cmd 0x%x and arg 0x%x\n",MXC400X_DEV_NAME, __func__, cmd, (unsigned int)arg);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
	 	err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
	 	err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
	 	GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
	 	return -EFAULT;
	}
	switch (cmd) {
		case GSENSOR_IOCTL_INIT:
			mxc400x_init_client(client, 0); 		 
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			MXC400X_ReadChipInfo(client, strbuf, MXC400X_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			MXC400X_SetPowerMode(client,true);	
			MXC400X_ReadSensorData(client, strbuf, MXC400X_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;
		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;
		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			MXC400X_ReadRawData(client, strbuf);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  
		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				cali[MXC400X_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[MXC400X_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[MXC400X_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = MXC400X_WriteCalibration(client, cali);			 
			}
			break;
		case GSENSOR_IOCTL_CLR_CALI:
			err = MXC400X_ResetCalibration(client);
			break;
		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(err = MXC400X_ReadCalibration(client, cali))
			{
				break;
			}
			
			sensor_data.x = cali[MXC400X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[MXC400X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[MXC400X_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
//  begin.................
		case GSENSOR_IOCTL_GET_DELAY:
           //  GSE_LOG("Gsensor get delay!!\n");
             break;
		case GSENSOR_IOCTL_GET_STATUS:
			 data = (void __user *) arg;
			 if(data == NULL)
			 {
				err = -EINVAL;
			 	break;	  
			 }
			 mutex_lock(&mxc400x_mutex);
			 if(copy_to_user(data, &enable_status, sizeof(enable_status)))
			 {
				err = -EFAULT;
				mutex_unlock(&mxc400x_mutex);
				break;
			 }	
             	
             mutex_unlock(&mxc400x_mutex);
             break;
		case GSENSOR_IOCTL_GET_DATA:
			 data = (void __user *) arg;
			 if(data == NULL)
			 {
				 err = -EINVAL;
				 break;	  
			 }
			 	
			 //MXC400X_RxData(client, vec);
             err = MXC400X_ReadSensorData(client, vec,MXC400X_AXES_NUM);
             
			 if(copy_to_user(data, vec, sizeof(vec)))
			 {
				 err = -EFAULT;
				 break;	  
			 }
             break;
		case GSENSOR_IOCTL_SET_DATA:
			 data = (void __user *) arg;
			 if(data == NULL)
			 {
				 err = -EINVAL;
				 break;	  
			 }
			 
			 if(copy_from_user(value, data, sizeof(value)))
			 {
				 err = -EFAULT;
				 break;	  
			 }
             
			 MXC400X_SaveData(value);
             break;
        case GSENSOR_IOCTL_GET_TEMP:
			 data = (void __user *) arg;
			 if(data == NULL)
			 {
				 err = -EINVAL;
				 break;	  
			 }
            
			 err = MXC400X_ReadTemp(client, &temp);
			
			 if(copy_to_user(data, &temp, sizeof(temp)))
			 {
				 err = -EFAULT;
				 break;	  
			 }
             break;
		case GSENSOR_IOCTL_GET_DANT:
             data = (void __user *) arg;
			 if(data == NULL)
			 {
				 err = -EINVAL;
				 break;	  
			 }
			 err = MXC400X_ReadDant(client, dant);
             //GSE_LOG("dant 2 = %d %d %d %d\n", dant[0], dant[1], dant[2], dant[3]);
			 if(copy_to_user(data, dant, sizeof(dant)))
			 {
				 err = -EFAULT;
				 break;	  
			 } 			
			break;
#if 0
		case GSENSOR_IOCTL_SET_TEST1:
			 data = (void __user *) arg;
			 if(data == NULL)
			 {
				 err = -EINVAL;
				 break;	  
			 }
			 if(copy_from_user(&test, data, sizeof(test)))
			 {
				 err = -EFAULT;
				 break;	  
			 }
			
			err = MXC400X_Test1(client, test);
            break;

		case GSENSOR_IOCTL_SET_TEST2:
			 data = (void __user *) arg;
			 if(data == NULL)
			 {
				 err = -EINVAL;
				 break;	  
			 }
			 if(copy_from_user(&test, data, sizeof(test)))
			 {
				 err = -EFAULT;
				 break;	  
			 }
			
			err = MXC400X_Test2(client, test);
			break;
		case GSENSOR_IOCTL_SET_TEST3:
			 data = (void __user *) arg;
			 if(data == NULL)
			 {
				 err = -EINVAL;
				 break;	  
			 }
			 if(copy_from_user(&test, data, sizeof(test)))
			 {
				 err = -EFAULT;
				 break;	  
			 }
			
			err = MXC400X_Test3(client, test);
			break;
#endif

		case GSENSOR_IOCTL_READ_REG:
			data = (void __user *) arg;
			if(data == NULL)
			{
				 err = -EINVAL;
				 break;	  
			}
		        if(copy_from_user(&test, data, sizeof(test)))
			{
				 err = -EFAULT;
				 break;	  
			}
			if(err = mxc400x_i2c_read_block(client, test, &buf, 1));
			{
				GSE_ERR("error: %d\n", err);
			}
 			if(copy_to_user(data, &buf, 1))
			{
				err = -EFAULT;
				break;	  
			} 
			break;

		case GSENSOR_IOCTL_WRITE_REG:
			data = (void __user *) arg;
			if(data == NULL)
			{
				 err = -EINVAL;
				 break;	  
			}
			if(copy_from_user(reg, data, sizeof(reg)))
			{
				 err = -EFAULT;
				 break;	  
			}
			err = i2c_master_send(client, reg, 0x2);
			if(err <= 0)
			{
				GSE_LOG("write reg failed!\n");
				err = -EFAULT;
				break;
			}
			break;
//  end.....
		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
	}
	
	return err;
}


static struct file_operations mxc400x_fops = {
		 //.owner = THIS_MODULE,
		 .open = mxc400x_open,
		 .release = mxc400x_release,
		 .unlocked_ioctl = mxc400x_unlocked_ioctl,
		#ifdef CONFIG_COMPAT
		.compat_ioctl = mxc400x_compat_ioctl,
		#endif
};

static struct miscdevice mxc400x_device = {
		 .minor = MISC_DYNAMIC_MINOR,
		 .name = "gsensor",
		 .fops = &mxc400x_fops,
};

#ifndef CONFIG_HAS_EARLYSUSPEND

static int mxc400x_suspend(struct i2c_client *client, pm_message_t msg) 
{
	 struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);	  
	 int err = 0; 

	 if(msg.event == PM_EVENT_SUSPEND)
	 {	 
		 if(obj == NULL)
		 {
			 GSE_ERR("null mxc400x!!\n");
			 return -EINVAL;
		 }
		 mutex_lock(&mxc400x_mutex);
		 atomic_set(&obj->suspend, 1);
		 if(err == MXC400X_SetPowerMode(obj->client, false))
		 {
			 GSE_ERR("write power control fail!!\n");
			 mutex_unlock(&mxc400x_mutex);
			 return -EINVAL;
		 } 
		 mutex_unlock(&mxc400x_mutex);
	 }
	 return err;
}

static int mxc400x_resume(struct i2c_client *client)
{
	 struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);		  
	 int err = 0;


	 if(obj == NULL)
	 {
		 GSE_ERR("null mxc400x!!\n");
		 return -EINVAL;
	 }
     mutex_lock(&mxc400x_mutex);
	 if(err = mxc400x_init_client(client, 0))
	 {
		 GSE_ERR("initialize client fail!!\n");
		 mutex_unlock(&mxc400x_mutex);
		 return -EINVAL;		
	 }
	 atomic_set(&obj->suspend, 0);
     mutex_unlock(&mxc400x_mutex);
	 return err;
}

#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
	
static void mxc400x_early_suspend(struct early_suspend *h) 
{
	 struct mxc400x_i2c_data *obj = container_of(h, struct mxc400x_i2c_data, early_drv);	 
	 int err; 

	 if(obj == NULL)
	 {
		 GSE_ERR("null mxc400x!!\n");
		 return;
	 }
	mutex_lock(&mxc400x_mutex);
	atomic_set(&obj->suspend, 1); 
	if(err = MXC400X_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		mutex_unlock(&mxc400x_mutex);
		return;
	}
	mutex_unlock(&mxc400x_mutex);
	mxc400x_power(obj->hw, 0);
}
	
static void mxc400x_late_resume(struct early_suspend *h)
{
	 struct mxc400x_i2c_data *obj = container_of(h, struct mxc400x_i2c_data, early_drv);		   
	 int err;

	 if(obj == NULL)
	 {
		 GSE_ERR("null mxc400x!!\n");
		 return;
	 }

	 mxc400x_power(obj->hw, 1);
	 mutex_lock(&mxc400x_mutex);
	 if(err = mxc400x_init_client(obj->client, 0))
	 {
		 GSE_ERR("initialize client fail!!\n");
		 mutex_unlock(&mxc400x_mutex);
		 return;		
	 }
	 atomic_set(&obj->suspend, 0);	  
	 mutex_unlock(&mxc400x_mutex);
}

#endif /*CONFIG_HAS_EARLYSUSPEND*/

static int mxc400x_i2c_detect(struct i2c_client *client,int kind,  struct i2c_board_info *info) 
{    
	strcpy(info->type, MXC400X_DEV_NAME);
	return 0;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int mxc400x_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int mxc400x_enable_nodata(int en)
{
	int res =0;
	bool power = false;
	
	if(1==en)
	{
		power = true;

	}
	if(0==en)
	{
		power = false;
	}
	res = MXC400X_SetPowerMode(obj_i2c_data->client, power);
	if(res != MXC400X_SUCCESS)
	{
		GSE_ERR("MXC400X_SetPowerMode fail!\n");
		return -1;
	}
	GSE_LOG("MXC400X_enable_nodata OK!\n");
	enable_status = en;
	return 0;
}

static int mxc400x_set_delay(u64 ns)
{
/*
    int value =0;
	int sample_delay=0;
	int err;
	value = (int)ns/1000/1000;
	if(value <= 5)
	{
		sample_delay = LIS3DH_BW_200HZ;
	}
	else if(value <= 10)
	{
		sample_delay = LIS3DH_BW_100HZ;
	}
	else
	{
		sample_delay = LIS3DH_BW_50HZ;
	}
	mutex_lock(&lis3dh_op_mutex);
	err = LIS3DH_SetBWRate(obj_i2c_data->client, sample_delay);
	if(err != LIS3DH_SUCCESS ) //0x2C->BW=100Hz
	{
		GSE_ERR("Set delay parameter error!\n");
	}
	mutex_unlock(&lis3dh_op_mutex);
	if(value >= 50)
	{
		atomic_set(&obj_i2c_data->filter, 0);
	}
	else
	{					
		obj_i2c_data->fir.num = 0;
		obj_i2c_data->fir.idx = 0;
		obj_i2c_data->fir.sum[LIS3DH_AXIS_X] = 0;
		obj_i2c_data->fir.sum[LIS3DH_AXIS_Y] = 0;
		obj_i2c_data->fir.sum[LIS3DH_AXIS_Z] = 0;
		atomic_set(&obj_i2c_data->filter, 1);
	}
	
	GSE_LOG("lis3dh_set_delay (%d)\n",value);
*/
	return 0;
}

static int mxc400x_get_data(int* x ,int* y,int* z, int* status)
{

	struct i2c_client *client = mxc400x_i2c_client;
	int strbuf[MXC400X_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	MXC400X_ReadSensorData(client, strbuf, MXC400X_BUFSIZE);     
	*x = strbuf[0];
	*y = strbuf[1];
	*z = strbuf[2];
	
//	sscanf(buff, "%x %x %x", x, y, z);		
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}


/*----------------------------------------------------------------------------*/
static int mxc400x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	 struct i2c_client *new_client;
	 struct mxc400x_i2c_data *obj;
	 
	 #ifdef KK_OLDARCH
	 struct hwmsen_object sobj;
	 #endif
	 
	 int err = 0;
	 
	#ifdef L_NewArch
	struct acc_control_path ctl={0};
	struct acc_data_path data={0};
    #endif
	
	 if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	 {
		 err = -ENOMEM;
		 goto exit;
	 }
	 
	 memset(obj, 0, sizeof(struct mxc400x_i2c_data));
 
	 obj->hw = get_cust_acc_hw();
	 atomic_set(&obj->layout, obj->hw->direction);
	 if(err = hwmsen_get_convert(obj->hw->direction, &obj->cvt))
	 {
		 GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		 goto exit;
	 }

	 obj_i2c_data = obj;
	 obj->client = client;
	 new_client = obj->client;
	 i2c_set_clientdata(new_client,obj);
	 
	 atomic_set(&obj->trace, 0);
	 atomic_set(&obj->suspend, 0);
 
 
#ifdef CONFIG_MXC400X_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
	 	atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	 
	else
	{
	 	atomic_set(&obj->firlen, obj->hw->firlen);
	}

	if(atomic_read(&obj->firlen) > 0)
	{
	 	atomic_set(&obj->fir_en, 1);
	}

#endif
	 mxc400x_i2c_client = new_client; 
 
	 if(err = mxc400x_init_client(new_client, 1))
	 {
		 goto exit_init_failed;
	 }
	 
 
	 if(err = misc_register(&mxc400x_device))
	 {
		 GSE_ERR("mxc400x_device register failed\n");
		 goto exit_misc_device_register_failed;
	 }
 
	 if(err = mxc400x_create_attr(&mxc400x_init_info.platform_diver_addr->driver))
	 {
		 GSE_ERR("create attribute err = %d\n", err);
		 goto exit_create_attr_failed;
	 }
	 
 #ifdef KK_OLDARCH
	 sobj.self = obj;
	 sobj.polling = 1;
	 sobj.sensor_operate = gsensor_operate;
	 if(err = hwmsen_attach(ID_ACCELEROMETER, &sobj))
	 {
		 GSE_ERR("attach fail = %d\n", err);
		 goto exit_kfree;
	 }
#endif

  #ifdef L_NewArch
 	ctl.open_report_data= mxc400x_open_report_data;
	ctl.enable_nodata = mxc400x_enable_nodata;
	ctl.set_delay  = mxc400x_set_delay;
	ctl.is_report_input_direct = false;
	
	err = acc_register_control_path(&ctl);
	if(err)
	{
	 	GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}
	
	data.get_data = mxc400x_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
	 	GSE_ERR("register acc data path err\n");
		goto exit_kfree;
	}
#endif
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	 obj->early_drv.level	 = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	 obj->early_drv.suspend  = mxc400x_early_suspend,
	 obj->early_drv.resume	 = mxc400x_late_resume,	  
	 register_early_suspend(&obj->early_drv);
#endif 
     mxc400x_init_flag = 0;
	 GSE_LOG("%s: OK\n", __func__);    
	 return 0;
 
	 exit_create_attr_failed:
	 misc_deregister(&mxc400x_device);
	 exit_misc_device_register_failed:
	 exit_init_failed:
	 //i2c_detach_client(new_client);
	 exit_kfree:
	 kfree(obj);
	 exit:
	 GSE_ERR("%s: err = %d\n", __func__, err);
      mxc400x_init_flag = -1;
	 return err;
}
 
static int mxc400x_i2c_remove(struct i2c_client *client)
{
	 int err = 0;	 
	 
	 if(err = mxc400x_delete_attr(&mxc400x_init_info.platform_diver_addr->driver))
	 {
		 GSE_ERR("mxc400x_delete_attr fail: %d\n", err);
	 }
	 
	 if(err = misc_deregister(&mxc400x_device))
	 {
		 GSE_ERR("misc_deregister fail: %d\n", err);
	 }

	 if(err = hwmsen_detach(ID_ACCELEROMETER))
		 

	 mxc400x_i2c_client = NULL;
	 i2c_unregister_device(client);
	 kfree(i2c_get_clientdata(client));
	 return 0;
}


static int  mxc400x_local_init(void)
{
	struct acc_hw *hw = get_cust_acc_hw();

	mxc400x_power(hw, 1);

	if(i2c_add_driver(&mxc400x_i2c_driver))
	{
		 GSE_ERR("add driver error\n");
		 return -1;
	}
	return 0;
}
static int mxc400x_remove(void)
{
	 struct acc_hw *hw = get_cust_acc_hw();
 
	 mxc400x_power(hw, 0);	 
	 i2c_del_driver(&mxc400x_i2c_driver);
	 return 0;
}
#if 0
static struct platform_driver mxc400x_gsensor_driver = {
		 .probe 	 = mxc400x_probe,
		 .remove	 = mxc400x_remove,	 
		 .driver	 = {
			 .name	= "gsensor",
			 //.owner = THIS_MODULE,
		 }
};
#endif

static int __init mxc400x_driver_init(void)
{
   	
	 struct acc_hw *hw = get_cust_acc_hw();
	 GSE_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 

	 i2c_register_board_info(hw->i2c_num, &i2c_mxc400x, 1);
	 acc_driver_add(&mxc400x_init_info);
/*
if(platform_driver_register(&mxc400x_gsensor_driver))
	 {
		GSE_ERR("failed to register driver");
		return -ENODEV;
	 }
*/
     mutex_init(&mxc400x_mutex);
#if defined(SW_SIMULATION)	
     mutex_init(&g_gsensor_mutex);
#endif

	 return 0;	 
}

static void __exit mxc400x_driver_exit(void)
{
	
     platform_driver_unregister(&mxc400x_gsensor_driver);
     mutex_destroy(&mxc400x_mutex);
#if defined(SW_SIMULATION)	//modified by lyon
     mutex_destroy(&g_gsensor_mutex);
#endif

}

module_init(mxc400x_driver_init);
module_exit(mxc400x_driver_exit);


MODULE_AUTHOR("Lyon Miao<xlmiao@memsic.com>");
MODULE_DESCRIPTION("MEMSIC MXC400x Accelerometer Sensor Driver");
MODULE_LICENSE("GPL");
