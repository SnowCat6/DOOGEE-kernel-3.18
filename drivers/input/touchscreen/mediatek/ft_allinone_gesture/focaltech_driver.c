/*
 * This software is licensed under the terms of the GNU General Public 
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms. 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * * VERSION      	DATE			AUTHOR          Note
 *    1.0		  2013-7-16			Focaltech        initial  based on MTK platform
 * 
 */

#include "tpd.h"

#include "tpd_custom_fts.h"
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef TPD_SYSFS_DEBUG
#include "focaltech_ex_fun.h"
#endif
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#include "cust_gpio_usage.h"
#ifdef CONFIG_OF_TOUCH
#include <linux/of.h>
#include <linux/of_irq.h>
#endif
#include <mach/battery_common.h>

#include <linux/dma-mapping.h>

#if defined(CONFIG_HCT_TP_GESTRUE)
#define GTP_GESTURE_WAKEUP    1
#else
#define GTP_GESTURE_WAKEUP    0
#endif

#if GTP_GESTURE_WAKEUP
static char tpgesture_value[10]={};
static char tpgesture_status_value[5] = {};
static char tpgesture_status = 1;
#endif

struct Upgrade_Info fts_updateinfo[] =
{
	{0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
	{0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 1, 1500},
	{0x05,"FT6208",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,60, 30, 0x79, 0x05, 10, 2000},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
};
				
struct Upgrade_Info fts_updateinfo_curr;

#ifdef TPD_PROXIMITY
#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)

static u8 tpd_proximity_flag 			= 0;
static u8 tpd_proximity_flag_one 		= 0; //add for tpd_proximity by wangdongfang
static u8 tpd_proximity_detect 		= 1;//0-->close ; 1--> far away
#endif

#ifdef FTS_GESTRUE
#define GESTURE_LEFT			0x20
#define GESTURE_RIGHT			0x21
#define GESTURE_UP		    	0x22
#define GESTURE_DOWN			0x23
#define GESTURE_DOUBLECLICK		0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_C		    0x34
#define GESTURE_S		    0x46
#define GESTURE_V		    0x54
#define GESTURE_Z		    0x41

#include "ft_gesture_lib.h"

#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

//suspend_state_t get_suspend_state(void);

unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};
#endif
extern struct tpd_device *tpd;
 
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);
 #ifdef CONFIG_OF_TOUCH
unsigned int touch_irq = 0;
u8 int_type = 0;
#endif
 
#ifdef CONFIG_OF_TOUCH
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc);
#else
static void tpd_eint_interrupt_handler(void);
#endif
 
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int  tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

static int tpd_flag 					= 0;
static int tpd_halt						= 0;
static int point_num 					= 0;
static int p_point_num 					= 0;

u8 *I2CDMABuf_va = NULL;
//volatile u32 I2CDMABuf_pa = NULL;
dma_addr_t I2CDMABuf_pa = 0;

//#define TPD_CLOSE_POWER_IN_SLEEP
#define TPD_OK 							0
//register define
#define DEVICE_MODE 					0x00
#define GEST_ID 						0x01
#define TD_STATUS 						0x02
//point1 info from 0x03~0x08
//point2 info from 0x09~0x0E
//point3 info from 0x0F~0x14
//point4 info from 0x15~0x1A
//point5 info from 0x1B~0x20
//register define

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 			3
//extern int tpd_mstar_status ;  // compatible mstar and ft6306 chenzhecong

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local_fwvga[TPD_KEY_COUNT][4] = TPD_KEYS_DIM_FWVGA;
static int tpd_keys_dim_local_qhd[TPD_KEY_COUNT][4] = TPD_KEYS_DIM_QHD;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#define VELOCITY_CUSTOM_FT5206
#ifdef VELOCITY_CUSTOM_FT5206
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 			10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 			10
#endif

#define TOUCH_IOC_MAGIC 				'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)


static int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
static int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;

static int tpd_misc_open(struct inode *inode, struct file *file)
{
/*
	file->private_data = adxl345_i2c_client;

	if(file->private_data == NULL)
	{
		printk("tpd: null pointer!!\n");
		return -EINVAL;
	}
	*/
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int adxl345_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	//struct i2c_client *client = (struct i2c_client*)file->private_data;
	//struct adxl345_i2c_data *obj = (struct adxl345_i2c_data*)i2c_get_clientdata(client);	
	//char strbuf[256];
	void __user *data;
	
	long err = 0;
	
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
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	return err;
}


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TPD_NAME,
	.fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[10];
    int x[10];
    int p[10];
    int id[10];
};
 
static const struct i2c_device_id ft5206_tpd_id[] = {{TPD_NAME,0},{}};
//unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
//static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
static struct i2c_board_info __initdata ft5206_i2c_tpd={ I2C_BOARD_INFO(TPD_NAME, (0x70>>1))};
 
static struct i2c_driver tpd_i2c_driver = {
  	.driver = {
	 	.name 	= TPD_NAME,
	//	.owner 	= THIS_MODULE,
  	},
  	.probe 		= tpd_probe,
  	.remove 	= tpd_remove,
  	.id_table 	= ft5206_tpd_id,
  	.detect 	= tpd_detect,
// 	.shutdown	= tpd_shutdown,
//  .address_data = &addr_data,
};


static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
#if defined(HCT_TPD_ROTATION)
	if(y<TPD_RES_Y)
	{
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, TPD_RES_X-x);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, TPD_RES_Y-y);
	}
	else
	{
	    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	}
#else	
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
#endif
	//printk("D[%4d %4d %4d] ", x, y, p);
	/* track id Start 0 */
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
	input_mt_sync(tpd->dev);
#if 1//ndef MT6572
    if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
#endif	
    {   
      tpd_button(x, y, 1);  
    }
	TPD_EM_PRINT(x, y, x, y, p-1, 1);
}
 
static  void tpd_up(int x, int y) {
	//input_report_abs(tpd->dev, ABS_PRESSURE, 0);
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	//input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//printk("U[%4d %4d %4d] ", x, y, 0);
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, 0, 0);
#if 1//ndef MT6572
    if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
#endif
    {   
       tpd_button(x, y, 0); 
    }   		 
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
//#if (TPD_MAX_POINTS==2)
//	char data[35] = {0};
//#else
//	char data[16] = {0};
//#endif	
    char data[128] = {0};
    u16 high_byte,low_byte,reg;
	u8 report_rate =0;

	p_point_num = point_num;
	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}
	mutex_lock(&i2c_access);


    reg = 0x00;
	fts_i2c_Read(i2c_client, &reg, 1, data, 64);
	mutex_unlock(&i2c_access);
	//TPD_DEBUG("received raw data from touch panel as following:\n");
	//TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5]);
	//TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
	//TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);
   
	/*get the number of the touch points*/
	point_num= data[2] & 0x0f;
	
	//TPD_DEBUG("point_num =%d\n",point_num);
		
	for(i = 0; i < point_num; i++)  
	{
		cinfo->p[i] = data[3+6*i] >> 6; //event flag 
     	cinfo->id[i] = data[3+6*i+2]>>4; //touch id
	   	/*get the X coordinate, 2 bytes*/
		high_byte = data[3+6*i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i + 1];
		cinfo->x[i] = high_byte |low_byte;

		//cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra
	
		/*get the Y coordinate, 2 bytes*/
		
		high_byte = data[3+6*i+2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i+3];
		cinfo->y[i] = high_byte |low_byte;

		 //cinfo->y[i]=  cinfo->y[i] * 800 >> 11;
	}
	TPD_DEBUG(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);	
	//TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
	//TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	
		  
	return true;
};

#ifdef TPD_PROXIMITY
int tpd_read_ps(void)
{
	tpd_proximity_detect;
	return 0;    
}

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	u8 state;
	int ret = -1;

	i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
	printk("[proxi_5206]read: 999 0xb0's value is 0x%02X\n", state);
	if (enable){
		state |= 0x01;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is on\n");	
	}else{
		state &= 0x00;	
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is off\n");
	}

	ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);
	TPD_PROXIMITY_DEBUG("[proxi_5206]write: 0xB0's value is 0x%02X\n", state);
	return 0;
}

int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	
	hwm_sensor_data *sensor_data;
	TPD_DEBUG("[proxi_5206]command = 0x%02X\n", command);		
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{		
					if((tpd_enable_ps(1) != 0))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				sensor_data = (hwm_sensor_data *)buff_out;				
				
				if((err = tpd_read_ps()))
				{
					err = -1;;
				}
				else
				{
					sensor_data->values[0] = tpd_get_ps_value();
					TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}	
				
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;	
}
#endif

#ifdef FTS_GESTRUE
/*
static void check_gesture(int gesture_id)
{
	
    printk("kaka gesture_id==0x%x\n ",gesture_id);
    
	switch(gesture_id)
	{
		case GESTURE_LEFT:		     
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
		     	    input_report_key(tpd->dev, KEY_LEFT, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_LEFT, 0);
			    input_sync(tpd->dev);
			break;
		case GESTURE_RIGHT:	
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
  			input_report_key(tpd->dev, KEY_RIGHT, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_RIGHT, 0);
			    input_sync(tpd->dev);
 
			break;
		case GESTURE_UP:	
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_UP, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_UP, 0);
			    input_sync(tpd->dev);
			    
			break;
		case GESTURE_DOWN:	
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);		
				input_report_key(tpd->dev, KEY_DOWN, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_DOWN, 0);
			    input_sync(tpd->dev);
		    
			break;
		case GESTURE_DOUBLECLICK:	
				input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
	//		input_report_key(tpd->dev, KEY_U, 1);
	//		    input_sync(tpd->dev);
	//		     input_report_key(tpd->dev, KEY_U, 0);
	//		    input_sync(tpd->dev);
			    
			break;
		case GESTURE_O:	
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
					input_report_key(tpd->dev, KEY_O, 1);
			    input_sync(tpd->dev);
			    input_report_key(tpd->dev, KEY_O, 0);
			    input_sync(tpd->dev);
			break;
		case GESTURE_W:			
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_W, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_W, 0);
			    input_sync(tpd->dev);
			    
			break;
		case GESTURE_M:		
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_M, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_M, 0);
			    input_sync(tpd->dev);
			    
			break;
		case GESTURE_E:		
				input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_E, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_E, 0);
			    input_sync(tpd->dev);
			    
			break;
		case GESTURE_C:		
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_C, 1);
			 input_sync(tpd->dev);
			 input_report_key(tpd->dev, KEY_C, 0);
			 input_sync(tpd->dev);
			break;

		case GESTURE_S:		
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_S, 1);
		 input_sync(tpd->dev);
		 input_report_key(tpd->dev, KEY_S, 0);
		 input_sync(tpd->dev);
		break;

		case GESTURE_V:
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_V, 1);
		 input_sync(tpd->dev);
		 input_report_key(tpd->dev, KEY_V, 0);
		 input_sync(tpd->dev);
		break;

		case GESTURE_Z:	
			input_report_key(tpd->dev, KEY_POWER, 1);
			    input_sync(tpd->dev);
			     input_report_key(tpd->dev, KEY_POWER, 0);
			    input_sync(tpd->dev);
		input_report_key(tpd->dev, KEY_Z, 1);
		 input_sync(tpd->dev);
		 input_report_key(tpd->dev, KEY_Z, 0);
		 input_sync(tpd->dev);
			break;
		default:
		
			break;
	}
}
*/
extern void tpgesture_hander();
static void check_gesture(int gesture_id)
{	
    printk("kaka gesture_id==0x%x\n ",gesture_id);
    
	switch(gesture_id)
	{
		case GESTURE_DOUBLECLICK:
            		sprintf(tpgesture_value,"DOUBCLICK");
			tpgesture_hander();
			break;

		case GESTURE_UP:
            		sprintf(tpgesture_value,"UP");
			tpgesture_hander();
			break;

		case GESTURE_DOWN:
			sprintf(tpgesture_value,"DOWN");
			tpgesture_hander();
			break;

		case GESTURE_LEFT:
			sprintf(tpgesture_value,"LEFT");
			tpgesture_hander();
			break;

		case GESTURE_RIGHT:
			sprintf(tpgesture_value,"RIGHT");
			tpgesture_hander();
			break;

		case GESTURE_C:
			sprintf(tpgesture_value,"c");
			tpgesture_hander();
			break;

		case GESTURE_O:
			sprintf(tpgesture_value,"o");
			tpgesture_hander();
			break;

		case GESTURE_W:
			sprintf(tpgesture_value,"w");
			tpgesture_hander();
			break;

		case GESTURE_E:
			sprintf(tpgesture_value,"e");
			tpgesture_hander();
			break;

		case GESTURE_V:
			sprintf(tpgesture_value,"v");
			tpgesture_hander();
			break;

		case GESTURE_M:
			sprintf(tpgesture_value,"m");
			tpgesture_hander();
			break;

		case GESTURE_Z:
			sprintf(tpgesture_value,"z");
			tpgesture_hander();
			break;
		case GESTURE_S:
			sprintf(tpgesture_value,"s");
			tpgesture_hander();
			break;
		default:		
			break;
	}
}

static int ft5x0x_read_Touchdata(void)
{
    unsigned char buf[FTS_GESTRUE_POINTS * 4] = { 0 };
    int ret = -1;
    int i = 0;
    buf[0] = 0xd3;
    int gestrue_id = 0;
    short pointnum = 0;
    ret = fts_i2c_Read(i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }
    /* FW ?±?ó??3?ê?ê? */
  //  if (0x24 == buf[0] || 0x20 == buf[0] || 0x21 == buf[0] || 0x22 == buf[0] || 0x23 == buf[0])
     printk("%s lsm--buf[0]=%x .\n",__func__,buf[0]);
    if(buf[0]!=0xfe)
    {
        gestrue_id =  buf[0];
        check_gesture(gestrue_id);
        return -1;
    }

    pointnum = (short)(buf[1]) & 0xff;
    buf[0] = 0xd3;
    if((pointnum * 4 + 2+6)<255)
    {
    ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 2 + 6));
    }
    else
    {
         ret = fts_i2c_Read(i2c_client, buf, 1, buf, 255);
          ret = fts_i2c_Read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 2 +6)-255);

    }
    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }
    gestrue_id = fetch_object_sample(buf, pointnum);
    for(i = 0;i < pointnum;i++)
    {
        coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
            8 | (((s16) buf[1 + (4 * i)])& 0xFF);
        coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
            8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
    }
	check_gesture(gestrue_id);
    return -1;
}
#endif
 static int touch_event_handler(void *unused)
 { 
   	struct touch_info cinfo, pinfo;
	int i=0;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

#ifdef TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
	u8 proximity_status;

#endif
 	u8 state;
	do
	{
#ifdef CONFIG_OF_TOUCH
	enable_irq(touch_irq);
#else
	  mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);
						 
		tpd_flag = 0;
			 
		set_current_state(TASK_RUNNING);
#ifdef FTS_GESTRUE
		i2c_smbus_read_i2c_block_data(i2c_client, 0xd0, 1, &state);
	  printk("%s lsm--state=%x .\n",__func__,state);
//		if((get_suspend_state() == PM_SUSPEND_MEM) && (state ==1))
		if(state ==1)
		{
			ft5x0x_read_Touchdata();
			continue;
		}
#endif
#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
			TPD_PROXIMITY_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state);

			if(!(state&0x01))
			{
				tpd_enable_ps(1);
			}

			i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
			TPD_PROXIMITY_DEBUG("proxi_5206 0x01 value is 1139 0x%02X\n", proximity_status);
			
			if (proximity_status == 0xC0)
			{
				tpd_proximity_detect = 0;	
			}
			else if(proximity_status == 0xE0)
			{
				tpd_proximity_detect = 1;
			}

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);

			if ((err = tpd_read_ps()))
			{
				TPD_PROXIMITY_DMESG("proxi_5206 read ps data 1156: %d\n", err);	
			}
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			{
				TPD_PROXIMITY_DMESG(" proxi_5206 call hwmsen_get_interrupt_data failed= %d\n", err);	
			}
		}  
#endif
		 
		if (tpd_touchinfo(&cinfo, &pinfo)) 
		{
		    //TPD_DEBUG("point_num = %d\n",point_num);
			TPD_DEBUG_SET_TIME;
			if(point_num >0) 
			{
			    for(i =0; i<point_num; i++)//only support 3 point
			    {
			         tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
			    }
			    input_sync(tpd->dev);
			}
			else  
    		{
			    tpd_up(cinfo.x[0], cinfo.y[0]);
        	    //TPD_DEBUG("release --->\n"); 
        	    //input_mt_sync(tpd->dev);
        	    input_sync(tpd->dev);
        		}
        	}

        	if(tpd_mode==12)
        	{
           //power down for desence debug
           //power off, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
			hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#else
			hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
			hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
#endif 
    		msleep(20);
    	}
 	}while(!kthread_should_stop());
 
	return 0;
}
 
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
{
	strcpy(info->type, TPD_DEVICE);	
	return 0;
}
 
#ifdef CONFIG_OF_TOUCH
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
	TPD_DEBUG_PRINT_INT;
		
	tpd_flag = 1;
	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
	disable_irq_nosync(touch_irq);
	wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
static void tpd_eint_interrupt_handler(void)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}
#endif

void focaltech_get_upgrade_array(void)
{

	u8 chip_id;
	u32 i;

	i2c_smbus_read_i2c_block_data(i2c_client,FT_REG_CHIP_ID,1,&chip_id);

	printk("%s chip_id = %x\n", __func__, chip_id);

	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
	}
}

#ifdef CONFIG_OF_TOUCH 
static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};
	GTP_INFO("Device Tree Tpd_irq_registration!");
	
	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if(node){
		of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);
		
		if (!int_type)	//EINTF_TRIGGER
		{
			ret = request_irq(touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, EINTF_TRIGGER_RISING, "TOUCH_PANEL-eint", NULL);
            //gtp_eint_trigger_type = EINTF_TRIGGER_RISING;
			if(ret > 0){
			    ret = -1;
			    GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		}
		else
		{
			ret = request_irq(touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, EINTF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
            //gtp_eint_trigger_type = EINTF_TRIGGER_FALLING;
			if(ret > 0){
			    ret = -1;
			    GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		}
	}else{
		GTP_ERROR("tpd request_irq can not find touch eint device node!.");
		ret = -1;
	}
	GTP_INFO("[%s]irq:%d, debounce:%d-%d:", __FUNCTION__, touch_irq, ints[0], ints[1]);
	return ret;
}
#endif

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	int retval = TPD_OK;
	char data;
	u8 report_rate=0;
	int err=0;
	int reset_count = 0;
	u8 chip_id,i;

reset_proc:   
	i2c_client = client;
   
	//power on, need confirm with SA
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(5);
	TPD_DMESG(" fts ic reset\n");
	
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif

#if 0 //def TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    
#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	hwPowerDown(TPD_POWER_SOURCE,"TP");
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(100);

#else
       mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
       mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(5);
	TPD_DMESG(" fts ic reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
       mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
       mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
       mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
       mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
#ifdef CONFIG_OF_TOUCH
	/* EINT device tree, default EINT enable */
	tpd_irq_registration();
	enable_irq(touch_irq);
#else
	  mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	  mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	  mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif 
	msleep(400);
 
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        if ( reset_count < TPD_MAX_RESET_COUNT )
        {
            reset_count++;
            goto reset_proc;
        }
#endif
		   return -1; 
	}

	tpd_load_status = 1;
 //   tpd_mstar_status =0 ;  // compatible mstar and ft6306 chenzhecong
 
        focaltech_get_upgrade_array();

	#ifdef FTS_APK_DEBUG
	ft5x0x_create_apk_debug_channel(client);
        #endif
	#ifdef TPD_SYSFS_DEBUG
	fts_create_sysfs(i2c_client);
	#endif

/*
	#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(i2c_client) < 0)
		TPD_DMESG(TPD_DEVICE, "%s:[FTS] create fts control iic driver failed\n",__func__);
	#endif
#ifdef FTS_GESTRUE
	init_para(480,854,60,0,0);
	//fts_write_reg(i2c_client, 0xd0, 0x01);
#endif
*/	
	#ifdef VELOCITY_CUSTOM_FT5206
	if((err = misc_register(&tpd_misc_device)))
	{
		printk("mtk_tpd: tpd_misc_device register failed\n");
		
	}
	#endif

	#ifdef TPD_AUTO_UPGRADE
	printk("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(i2c_client);
	#endif
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{ 
		retval = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}

	TPD_DMESG("FTS Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#ifdef TPD_PROXIMITY
	struct hwmsen_object obj_ps;
	
	obj_ps.polling = 0;//interrupt mode
	obj_ps.sensor_operate = tpd_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("proxi_fts attach fail = %d\n", err);
	}
	else
	{
		APS_ERR("proxi_fts attach ok = %d\n", err);
	}		
#endif
   return 0;
   
 }

 static int tpd_remove(struct i2c_client *client)
{

        #ifdef FTS_APK_DEBUG
	ft5x0x_release_apk_debug_channel();
	#endif
   	#ifdef TPD_SYSFS_DEBUG
	fts_release_sysfs(client);
	#endif

	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	
	TPD_DEBUG("TPD removed\n");
 
   	return 0;
}
 
 #define IIC_DMA_MAX_TRANSFER_SIZE     250
static int tpd_local_init(void)
{
  	TPD_DMESG("FTS I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 #if 1//TPD_SUPPORT_I2C_DMA
	//mutex_init(&dma_mutex);
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	I2CDMABuf_va = (u8 *) dma_alloc_coherent(&tpd->dev->dev, IIC_DMA_MAX_TRANSFER_SIZE, &I2CDMABuf_pa, GFP_KERNEL);
	if (!I2CDMABuf_va) {
		TPD_DMESG("Allocate DMA I2C Buffer failed!");
		return -1;
	}
	memset(I2CDMABuf_va, 0, IIC_DMA_MAX_TRANSFER_SIZE);
#endif
   	if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("FTS unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("FTS add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
#ifdef TPD_HAVE_BUTTON     
	if(TPD_RES_Y > 854)
	{
	    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local_qhd);// initialize tpd button data
	}
	else
	{
	    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local_fwvga);// initialize tpd button data
	}
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
    TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
    tpd_type_cap = 1;
    return 0; 
 }

 static void tpd_resume( struct early_suspend *h )
 {
  //int retval = TPD_OK;
  //char data;
#ifdef TPD_PROXIMITY	
	if (tpd_proximity_flag == 1)
	{
		if(tpd_proximity_flag_one == 1)
		{
			tpd_proximity_flag_one = 0;	
			TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
			return;
		}
	}
#endif	
 
   	TPD_DMESG("TPD wake up\n");
#ifdef FTS_GESTRUE
	if((g_call_state != CALL_ACTIVE)&&(tpgesture_status))
	{
            fts_write_reg(i2c_client,0xD0,0x00);
	    //fts_write_reg(i2c_client,0xD1,0x00);
	    //fts_write_reg(i2c_client,0xD2,0x00);
	    tpd_halt = 0;
	   // return;
	}
#endif
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");

#else

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(2);  

#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
   // mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
   // mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
#ifdef CONFIG_OF_TOUCH
	enable_irq(touch_irq);
#else
	  mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif 
	msleep(30);
	tpd_halt = 0;
	/* for resume debug
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("resume I2C transfer error, line: %d\n", __LINE__);
	}
	*/
	tpd_up(0,0);
	input_sync(tpd->dev);
	TPD_DMESG("TPD wake up done\n");
	 //return retval;
 }

 static void tpd_suspend( struct early_suspend *h )
 {
	// int retval = TPD_OK;
	 static char data = 0x3;

#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		tpd_proximity_flag_one = 1;	
		return;
	}
#endif

#if GTP_GESTURE_WAKEUP
	printk("[xy-tp]%d\n", tpgesture_status);

	if((g_call_state != CALL_ACTIVE)&&(tpgesture_status)) 
	{
	printk("[xy-tp]:gesture mode\n");
		msleep(200);
		mutex_lock(&i2c_access);
		fts_write_reg(i2c_client, 0xd0, 0x01);
	//	if (fts_updateinfo_curr.CHIP_ID==0x54) 
	//	{
			fts_write_reg(i2c_client, 0xd1, 0xff);
			fts_write_reg(i2c_client, 0xd2, 0xff);
			fts_write_reg(i2c_client, 0xd5, 0xff);
			fts_write_reg(i2c_client, 0xd6, 0xff);
			fts_write_reg(i2c_client, 0xd7, 0xff);
			fts_write_reg(i2c_client, 0xd8, 0xff);
	//	}
		 mutex_unlock(&i2c_access);
		tpd_halt = 1;
		return;
	} 
	else 
	{
#endif
	tpd_halt = 1;
	 TPD_DMESG("TPD enter sleep\n");
#ifdef CONFIG_OF_TOUCH
		disable_irq(touch_irq);
#else
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
	mutex_lock(&i2c_access);
	i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
	mutex_unlock(&i2c_access);
#endif
  #ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
    #else
        hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
    #endif
	TPD_DMESG("TPD enter sleep done\n");
	//return retval;
	#if GTP_GESTURE_WAKEUP
	}
	#endif
 } 
#if GTP_GESTURE_WAKEUP
static ssize_t show_tpgesture_value(struct device* dev, struct device_attribute *attr, char *buf)
{
	 printk("show tp gesture value is %s \n",tpgesture_value);
	 return scnprintf(buf, PAGE_SIZE, "%s\n", tpgesture_value);
}
 
static ssize_t show_tpgesture_status_value(struct device* dev, struct device_attribute *attr, char *buf)
{
	 printk("show tp gesture status is %s \n",tpgesture_status_value);
	 return scnprintf(buf, PAGE_SIZE, "%s\n", tpgesture_status_value);
}
 
static ssize_t store_tpgesture_status_value(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	 if(!strncmp(buf, "on", 2))
	 {
		 sprintf(tpgesture_status_value,"on");
		 tpgesture_status = 1;//status --- on
	 }
	 else
	 {
		 sprintf(tpgesture_status_value,"off");
		 tpgesture_status = 0;//status --- off
	 }
 
	 return count;
}
 
static DEVICE_ATTR(tpgesture,	0664, show_tpgesture_value, NULL);
static DEVICE_ATTR(tpgesture_status,  0666, show_tpgesture_status_value, store_tpgesture_status_value);
static struct device_attribute *tpd_attr_list[] = {
	 &dev_attr_tpgesture,
	 &dev_attr_tpgesture_status,
};
#endif


 static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = TPD_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif		
#if GTP_GESTURE_WAKEUP
	.attrs = {
		.attr = &tpd_attr_list,
		.num = (int)(sizeof(tpd_attr_list)/sizeof(tpd_attr_list[0])),
	},
#endif
 };
 /* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	printk("MediaTek FTS touch panel driver init\n");
	i2c_register_board_info(IIC_PORT, &ft5206_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FTS driver failed\n");
	 return 0;
 }
 
 /* should never be called */
static void __exit tpd_driver_exit(void) {
	TPD_DMESG("MediaTek FTS touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}
 
module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


