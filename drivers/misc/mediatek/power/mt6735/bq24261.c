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
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#include <linux/xlog.h>


#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include "bq24261.h"
#include "cust_charging.h"
#include <mach/charging.h>

#if defined(CONFIG_MTK_FPGA)
#else
#include <cust_i2c.h>
#endif

/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#define bq24261_SLAVE_ADDR_WRITE   0xD6
#define bq24261_SLAVE_ADDR_Read    0xD7

#ifdef I2C_SWITHING_CHARGER_CHANNEL
#define bq24261_BUSNUM I2C_SWITHING_CHARGER_CHANNEL
#else
#define bq24261_BUSNUM 0
#endif

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id bq24261_i2c_id[] = {{"bq24261",0},{}};   
kal_bool chargin_hw_init_done = KAL_FALSE; 
static int bq24261_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);


#ifdef CONFIG_OF
static const struct of_device_id bq24261_of_match[] = {
	{ .compatible = "bq24261", },
	{},
};

MODULE_DEVICE_TABLE(of, bq24261_of_match);
#endif

static struct i2c_driver bq24261_driver = {
    .driver = {
        .name    = "bq24261",
#ifdef CONFIG_OF 
        .of_match_table = bq24261_of_match,
#endif
    },
    .probe       = bq24261_driver_probe,
    .id_table    = bq24261_i2c_id,

};

/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/
kal_uint8 bq24261_reg[bq24261_REG_NUM] = {0};

static DEFINE_MUTEX(bq24261_i2c_access);

int g_bq24261_hw_exist=0;

/**********************************************************
  *
  *   [I2C Function For Read/Write bq24261] 
  *
  *********************************************************/
kal_uint32 bq24261_read_byte(kal_uint8 cmd, kal_uint8 *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

    mutex_lock(&bq24261_i2c_access);
    
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        new_client->ext_flag=0;

        mutex_unlock(&bq24261_i2c_access);
        return 0;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    // new_client->addr = new_client->addr & I2C_MASK_FLAG;
    new_client->ext_flag=0;
    
    mutex_unlock(&bq24261_i2c_access);    
    return 1;
}

kal_uint32 bq24261_write_byte(kal_uint8 cmd, kal_uint8 writeData)
{
    char    write_data[2] = {0};
    int     ret=0;
    
    mutex_lock(&bq24261_i2c_access);
    
    write_data[0] = cmd;
    write_data[1] = writeData;
    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) 
    {
       
        new_client->ext_flag=0;
        mutex_unlock(&bq24261_i2c_access);
        return 0;
    }
    
    new_client->ext_flag=0;
    mutex_unlock(&bq24261_i2c_access);
    return 1;
}

/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
kal_uint32 bq24261_read_interface (kal_uint8 RegNum, kal_uint8 *val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24261_reg = 0;
    kal_uint32 ret = 0;

    battery_log(BAT_LOG_FULL,"--------------------------------------------------\n");

    ret = bq24261_read_byte(RegNum, &bq24261_reg);

    battery_log(BAT_LOG_FULL,"[bq24261_read_interface] Reg[%x]=0x%x\n", RegNum, bq24261_reg);
	
    bq24261_reg &= (MASK << SHIFT);
    *val = (bq24261_reg >> SHIFT);
	
    battery_log(BAT_LOG_FULL,"[bq24261_read_interface] val=0x%x\n", *val);
	
    return ret;
}

kal_uint32 bq24261_config_interface (kal_uint8 RegNum, kal_uint8 val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24261_reg = 0;
    kal_uint32 ret = 0;

    battery_log(BAT_LOG_FULL,"--------------------------------------------------\n");

    ret = bq24261_read_byte(RegNum, &bq24261_reg);
    battery_log(BAT_LOG_FULL,"[bq24261_config_interface] Reg[%x]=0x%x\n", RegNum, bq24261_reg);
    
    bq24261_reg &= ~(MASK << SHIFT);
    bq24261_reg |= (val << SHIFT);

    if(RegNum == bq24261_CON1 && val == 1 && MASK ==CON1_RESET_MASK && SHIFT == CON1_RESET_SHIFT)
    {
        // RESET bit
    }
    else if(RegNum == bq24261_CON1)
    {
        bq24261_reg &= ~0x80;	//RESET bit read returs 1, so clear it
    }
	 

    ret = bq24261_write_byte(RegNum, bq24261_reg);
    battery_log(BAT_LOG_FULL,"[bq24261_config_interface] write Reg[%x]=0x%x\n", RegNum, bq24261_reg);

    // Check
    //bq24261_read_byte(RegNum, &bq24261_reg);
    //printk("[bq24261_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq24261_reg);

    return ret;
}

//write one register directly
kal_uint32 bq24261_reg_config_interface (kal_uint8 RegNum, kal_uint8 val)
{   
    kal_uint32 ret = 0;
    
    ret = bq24261_write_byte(RegNum, val);

    return ret;
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
  
//CON0----------------------------------------------------

void bq24261_set_tmr_rst(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_TMR_RST_MASK),
                                    (kal_uint8)(CON0_TMR_RST_SHIFT)
                                    );
}

void bq24261_set_en_boost(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_EN_BOOST_MASK),
                                    (kal_uint8)(CON0_EN_BOOST_SHIFT)
                                    );
}

kal_uint32 bq24261_get_stat(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24261_read_interface(     (kal_uint8)(bq24261_CON0), 
                                    (&val),
                                    (kal_uint8)(CON0_STAT_MASK),
                                    (kal_uint8)(CON0_STAT_SHIFT)
                                    );
    return val;
}

void bq24261_set_en_shipmode(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_EN_SHIPMODE_MASK),
                                    (kal_uint8)(CON0_EN_SHIPMODE_SHIFT)
                                    );
}

kal_uint32 bq24261_get_fault(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24261_read_interface(     (kal_uint8)(bq24261_CON0), 
                                    (&val),
                                    (kal_uint8)(CON0_FAULT_MASK),
                                    (kal_uint8)(CON0_FAULT_SHIFT)
                                    );
    return val;
}

//CON1----------------------------------------------------

void bq24261_set_reset(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_RESET_MASK),
                                    (kal_uint8)(CON1_RESET_SHIFT)
                                    );
}

void bq24261_set_in_limit(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_IN_LIMIT_MASK),
                                    (kal_uint8)(CON1_IN_LIMIT_SHIFT)
                                    );
}

void bq24261_set_en_stat(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_EN_STAT_MASK),
                                    (kal_uint8)(CON1_EN_STAT_SHIFT)
                                    );
}

void bq24261_set_te(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_TE_MASK),
                                    (kal_uint8)(CON1_TE_SHIFT)
                                    );
}

void bq24261_set_dis_ce(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_DIS_CE_MASK),
                                    (kal_uint8)(CON1_DIS_CE_SHIFT)
                                    );
}

void bq24261_set_hz_mode(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_HZ_MODE_MASK),
                                    (kal_uint8)(CON1_HZ_MODE_SHIFT)
                                    );
}

//CON2----------------------------------------------------

void bq24261_set_vbreg(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON2), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON2_VBREG_MASK),
                                    (kal_uint8)(CON2_VBREG_SHIFT)
                                    );
}

void bq24261_set_mod_freq(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON2), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON2_MOD_FREQ_MASK),
                                    (kal_uint8)(CON2_MOD_FREQ_SHIFT)
                                    );
}

//CON3----------------------------------------------------

kal_uint32 bq24261_get_vender_code(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24261_read_interface(     (kal_uint8)(bq24261_CON3), 
                                    (&val),
                                    (kal_uint8)(CON3_VENDER_CODE_MASK),
                                    (kal_uint8)(CON3_VENDER_CODE_SHIFT)
                                    );
    return val;
}

kal_uint32 bq24261_get_pn(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24261_read_interface(     (kal_uint8)(bq24261_CON3), 
                                    (&val),
                                    (kal_uint8)(CON3_PN_MASK),
                                    (kal_uint8)(CON3_PN_SHIFT)
                                    );
    return val;
}

//CON4----------------------------------------------------

void bq24261_set_ichg(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_ICHRG_MASK),
                                    (kal_uint8)(CON4_ICHRG_SHIFT)
                                    );
}

void bq24261_set_iterm(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_ITERM_MASK),
                                    (kal_uint8)(CON4_ITERM_SHIFT)
                                    );
}

//CON5----------------------------------------------------

kal_uint32 bq24261_get_minsys_status(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24261_read_interface(     (kal_uint8)(bq24261_CON5), 
                                    (&val),
                                    (kal_uint8)(CON5_MINSYS_STATUS_MASK),
                                    (kal_uint8)(CON5_MINSYS_STATUS_SHIFT)
                                    );
    return val;
}

kal_uint32 bq24261_get_vindpm_status(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24261_read_interface(     (kal_uint8)(bq24261_CON5), 
                                    (&val),
                                    (kal_uint8)(CON5_VINDPM_STATUS_MASK),
                                    (kal_uint8)(CON5_VINDPM_STATUS_SHIFT)
                                    );
    return val;
}

void bq24261_set_low_chg(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_LOW_CHG_MASK),
                                    (kal_uint8)(CON5_LOW_CHG_SHIFT)
                                    );
}

void bq24261_set_dpdm_en(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_DPDM_EN_MASK),
                                    (kal_uint8)(CON5_DPDM_EN_SHIFT)
                                    );
}

kal_uint32 bq24261_get_cd_status(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24261_read_interface(     (kal_uint8)(bq24261_CON5), 
                                    (&val),
                                    (kal_uint8)(CON5_CD_STATUS_MASK),
                                    (kal_uint8)(CON5_CD_STATUS_SHIFT)
                                    );
    return val;
}

void bq24261_set_vindpm(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_VINDPM_MASK),
                                    (kal_uint8)(CON5_VINDPM_SHIFT)
                                    );
}

//CON6----------------------------------------------------

void bq24261_set_2xtmr_en(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_2XTMR_EN_MASK),
                                    (kal_uint8)(CON6_2XTMR_EN_SHIFT)
                                    );
}

void bq24261_set_tmr(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_TMR_MASK),
                                    (kal_uint8)(CON6_TMR_SHIFT)
                                    );
}

void bq24261_set_boost_ilim(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_BOOST_ILIM_MASK),
                                    (kal_uint8)(CON6_BOOST_ILIM_SHIFT)
                                    );
}

void bq24261_set_ts_en(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_TS_EN_MASK),
                                    (kal_uint8)(CON6_TS_EN_SHIFT)
                                    );
}

kal_uint32 bq24261_get_ts_fault(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24261_read_interface(     (kal_uint8)(bq24261_CON6), 
                                    (&val),
                                    (kal_uint8)(CON6_TS_FAULT_MASK),
                                    (kal_uint8)(CON6_TS_FAULT_SHIFT)
                                    );
    return val;
}

void bq24261_set_vindpm_off(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24261_config_interface(   (kal_uint8)(bq24261_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_VINDPM_OFF_MASK),
                                    (kal_uint8)(CON6_VINDPM_OFF_SHIFT)
                                    );
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
void bq24261_hw_component_detect(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24261_read_interface(0x03, &val, 0xFF, 0x0);
    
    if(val == 0)
        g_bq24261_hw_exist=0;
    else
        g_bq24261_hw_exist=1;

    printk("[bq24261_hw_component_detect] exist=%d, Reg[0x03]=0x%x\n", 
        g_bq24261_hw_exist, val);
}

int is_bq24261_exist(void)
{
    printk("[is_bq24261_exist] g_bq24261_hw_exist=%d\n", g_bq24261_hw_exist);
    
    return g_bq24261_hw_exist;
}

void bq24261_dump_register(void)
{
    kal_uint8 i=0;
    printk("[bq24261] ");
    for (i=0;i<bq24261_REG_NUM;i++)
    {
        bq24261_read_byte(i, &bq24261_reg[i]);
        printk("[0x%x]=0x%x ", i, bq24261_reg[i]);        
    }
    printk("\n");
}

void bq24261_hw_init(void)
{   
    battery_log(BAT_LOG_CRTI,"[bq24261_hw_init] After HW init\n");    
    bq24261_dump_register();
}

static int bq24261_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    int err=0; 

    battery_log(BAT_LOG_CRTI,"[bq24261_driver_probe] \n");

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }    
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;    

    //---------------------
    bq24261_hw_component_detect();
    bq24261_dump_register();
    //bq24261_hw_init(); //move to charging_hw_xxx.c   
    chargin_hw_init_done = KAL_TRUE;
	
    return 0;                                                                                       

exit:
    return err;

}

/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/
kal_uint8 g_reg_value_bq24261=0;
static ssize_t show_bq24261_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    battery_log(BAT_LOG_CRTI,"[show_bq24261_access] 0x%x\n", g_reg_value_bq24261);
    return sprintf(buf, "%u\n", g_reg_value_bq24261);
}
static ssize_t store_bq24261_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int ret=0;
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;
    
    battery_log(BAT_LOG_CRTI,"[store_bq24261_access] \n");
    
    if(buf != NULL && size != 0)
    {
        battery_log(BAT_LOG_CRTI,"[store_bq24261_access] buf is %s and size is %zu \n",buf,size);
        reg_address = simple_strtoul(buf,&pvalue,16);
        
        if(size > 3)
        {        
            reg_value = simple_strtoul((pvalue+1),NULL,16);        
            battery_log(BAT_LOG_CRTI,"[store_bq24261_access] write bq24261 reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            ret=bq24261_config_interface(reg_address, reg_value, 0xFF, 0x0);
        }
        else
        {    
            ret=bq24261_read_interface(reg_address, &g_reg_value_bq24261, 0xFF, 0x0);
            battery_log(BAT_LOG_CRTI,"[store_bq24261_access] read bq24261 reg 0x%x with value 0x%x !\n",reg_address,g_reg_value_bq24261);
            battery_log(BAT_LOG_CRTI,"[store_bq24261_access] Please use \"cat bq24261_access\" to get value\r\n");
        }        
    }    
    return size;
}
static DEVICE_ATTR(bq24261_access, 0664, show_bq24261_access, store_bq24261_access); //664

static int bq24261_user_space_probe(struct platform_device *dev)    
{    
    int ret_device_file = 0;

    battery_log(BAT_LOG_CRTI,"******** bq24261_user_space_probe!! ********\n" );
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq24261_access);
    
    return 0;
}

struct platform_device bq24261_user_space_device = {
    .name   = "bq24261-user",
    .id     = -1,
};

static struct platform_driver bq24261_user_space_driver = {
    .probe      = bq24261_user_space_probe,
    .driver     = {
        .name = "bq24261-user",
    },
};


static struct i2c_board_info __initdata i2c_bq24261 = { I2C_BOARD_INFO("bq24261", (bq24261_SLAVE_ADDR_WRITE>>1))};


static int __init bq24261_subsys_init(void)
{    
    int ret=0;
    
    battery_log(BAT_LOG_CRTI,"[bq24261_init] init start. ch=%d\n", bq24261_BUSNUM);
    
    i2c_register_board_info(bq24261_BUSNUM, &i2c_bq24261, 1);

    if(i2c_add_driver(&bq24261_driver)!=0)
    {
        battery_log(BAT_LOG_CRTI,"[bq24261_init] failed to register bq24261 i2c driver.\n");
    }
    else
    {
        battery_log(BAT_LOG_CRTI,"[bq24261_init] Success to register bq24261 i2c driver.\n");
    }

    // bq24261 user space access interface
    ret = platform_device_register(&bq24261_user_space_device);
    if (ret) {
        battery_log(BAT_LOG_CRTI,"****[bq24261_init] Unable to device register(%d)\n", ret);
        return ret;
    }    
    ret = platform_driver_register(&bq24261_user_space_driver);
    if (ret) {
        battery_log(BAT_LOG_CRTI,"****[bq24261_init] Unable to register driver (%d)\n", ret);
        return ret;
    }
    
    return 0;        
}

static void __exit bq24261_exit(void)
{
    i2c_del_driver(&bq24261_driver);
}

//module_init(bq24261_init);
//module_exit(bq24261_exit);
subsys_initcall(bq24261_subsys_init);
   
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq24261 Driver");
MODULE_AUTHOR("James Lo<james.lo@mediatek.com>");
