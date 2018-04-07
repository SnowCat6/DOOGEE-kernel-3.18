#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/init.h>         /* For init/exit macros */
#include <linux/module.h>       /* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/charging.h>

#include "bq24158.h"

/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#define bq24158_SLAVE_ADDR_WRITE   0xD4
#define bq24158_SLAVE_ADDR_Read    0xD5

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id bq24158_i2c_id[] = {{"bq24158",0},{}};   
kal_bool chargin_hw_init_done = KAL_FALSE; 
static int bq24158_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
#ifdef CONFIG_OF
static const struct of_device_id bq24158_of_match[] = { 
            {.compatible = "mediatek,swithing_charger",},
                    {}, 
};

MODULE_DEVICE_TABLE(of, bq24158_of_match);
#endif

static struct i2c_driver bq24158_driver = {
    .driver = {
        .name    = "bq24158",
#ifdef CONFIG_OF
        .of_match_table = bq24158_of_match,
#endif

    },
    .probe       = bq24158_driver_probe,
    .id_table    = bq24158_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/
unsigned char bq24158_reg[bq24158_REG_NUM] = {0};

static DEFINE_MUTEX(bq24158_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write bq24158] 
  *
  *********************************************************/
int bq24158_read_byte(unsigned char cmd, unsigned char *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

    if (new_client == NULL)
        return -1;

    mutex_lock(&bq24158_i2c_access);
    
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], ((1 << 8 ) | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        new_client->ext_flag=0;

        mutex_unlock(&bq24158_i2c_access);
        return 0;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    //new_client->addr = new_client->addr & I2C_MASK_FLAG;
    new_client->ext_flag=0;
    
    mutex_unlock(&bq24158_i2c_access);    
    return 1;
}

int bq24158_write_byte(unsigned char cmd, unsigned char writeData)
{
    char    write_data[2] = {0};
    int     ret=0;

    if (new_client == NULL)
        return -1;

    mutex_lock(&bq24158_i2c_access);
    
    write_data[0] = cmd;
    write_data[1] = writeData;
    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) 
    {
       
        new_client->ext_flag=0;
        mutex_unlock(&bq24158_i2c_access);
        return 0;
    }
    
    new_client->ext_flag=0;
    mutex_unlock(&bq24158_i2c_access);
    return 1;
}

/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
int bq24158_read_interface (unsigned char RegNum, unsigned char *val, unsigned char MASK, unsigned char SHIFT)
{
    unsigned char bq24158_reg = 0;
    int ret = 0;

   battery_xlog_printk(BAT_LOG_FULL,"--------------------------------------------------\n");

    ret = bq24158_read_byte(RegNum, &bq24158_reg);

	battery_xlog_printk(BAT_LOG_FULL,"[bq24158_read_interface] Reg[%x]=0x%x\n", RegNum, bq24158_reg);
	
    bq24158_reg &= (MASK << SHIFT);
    *val = (bq24158_reg >> SHIFT);
	
	battery_xlog_printk(BAT_LOG_FULL,"[bq24158_read_interface] val=0x%x\n", *val);
	
    return ret;
}

int bq24158_config_interface (unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT)
{
    unsigned char bq24158_reg = 0;
    int ret = 0;

    battery_xlog_printk(BAT_LOG_FULL,"--------------------------------------------------\n");

    ret = bq24158_read_byte(RegNum, &bq24158_reg);
    battery_xlog_printk(BAT_LOG_FULL,"[bq24158_config_interface] Reg[%x]=0x%x\n", RegNum, bq24158_reg);
    
    bq24158_reg &= ~(MASK << SHIFT);
    bq24158_reg |= (val << SHIFT);

	if(RegNum == bq24158_CON4 && val == 1 && MASK ==CON4_RESET_MASK && SHIFT == CON4_RESET_SHIFT)
	{
		// RESET bit
	}
	else if(RegNum == bq24158_CON4)
	{
		bq24158_reg &= ~0x80;	//RESET bit read returs 1, so clear it
	}
	 

    ret = bq24158_write_byte(RegNum, bq24158_reg);
    battery_xlog_printk(BAT_LOG_FULL,"[bq24158_config_interface] write Reg[%x]=0x%x\n", RegNum, bq24158_reg);

    // Check
    //bq24158_read_byte(RegNum, &bq24158_reg);
    //printk("[bq24158_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq24158_reg);

    return ret;
}

//write one register directly
int bq24158_config_interface_liao (unsigned char RegNum, unsigned char val)
{   
    int ret = 0;
    
    ret = bq24158_write_byte(RegNum, val);

    return ret;
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
//CON0----------------------------------------------------

void bq24158_set_tmr_rst(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON0), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON0_TMR_RST_MASK),
                                    (unsigned char)(CON0_TMR_RST_SHIFT)
                                    );
}

 unsigned int bq24158_get_otg_status(void)
{
    int ret=0;
    unsigned char val=0;

    ret=bq24158_read_interface(     (unsigned char)(bq24158_CON0), 
                                    (&val),
                                    (unsigned char)(CON0_OTG_MASK),
                                    (unsigned char)(CON0_OTG_SHIFT)
                                    );
    return val;
}

void bq24158_set_en_stat(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON0), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON0_EN_STAT_MASK),
                                    (unsigned char)(CON0_EN_STAT_SHIFT)
                                    );
}

unsigned int bq24158_get_chip_status(void)
{
    int ret=0;
    unsigned char val=0;

    ret=bq24158_read_interface(     (unsigned char)(bq24158_CON0), 
                                    (&val),
                                    (unsigned char)(CON0_STAT_MASK),
                                    (unsigned char)(CON0_STAT_SHIFT)
                                    );
    return val;
}

unsigned int bq24158_get_boost_status(void)
{
    int ret=0;
    unsigned char val=0;

    ret=bq24158_read_interface(     (unsigned char)(bq24158_CON0), 
                                    (&val),
                                    (unsigned char)(CON0_BOOST_MASK),
                                    (unsigned char)(CON0_BOOST_SHIFT)
                                    );
    return val;
}

unsigned int bq24158_get_fault_status(void)
{
    int ret=0;
    unsigned char val=0;

    ret=bq24158_read_interface(     (unsigned char)(bq24158_CON0), 
                                    (&val),
                                    (unsigned char)(CON0_FAULT_MASK),
                                    (unsigned char)(CON0_FAULT_SHIFT)
                                    );
    return val;
}

//CON1----------------------------------------------------

void bq24158_set_input_charging_current(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_LIN_LIMIT_MASK),
                                    (unsigned char)(CON1_LIN_LIMIT_SHIFT)
                                    );
}

void bq24158_set_v_low(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_LOW_V_MASK),
                                    (unsigned char)(CON1_LOW_V_SHIFT)
                                    );
}

void bq24158_set_te(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_TE_MASK),
                                    (unsigned char)(CON1_TE_SHIFT)
                                    );
}

void bq24158_set_ce(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_CE_MASK),
                                    (unsigned char)(CON1_CE_SHIFT)
                                    );
}

void bq24158_set_hz_mode(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_HZ_MODE_MASK),
                                    (unsigned char)(CON1_HZ_MODE_SHIFT)
                                    );
}

void bq24158_set_opa_mode(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_OPA_MODE_MASK),
                                    (unsigned char)(CON1_OPA_MODE_SHIFT)
                                    );
}

//CON2----------------------------------------------------

void bq24158_set_oreg(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON2), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON2_OREG_MASK),
                                    (unsigned char)(CON2_OREG_SHIFT)
                                    );
}

void bq24158_set_otg_pl(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON2), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON2_OTG_PL_MASK),
                                    (unsigned char)(CON2_OTG_PL_SHIFT)
                                    );
}

void bq24158_set_otg_en(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON2), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON2_OTG_EN_MASK),
                                    (unsigned char)(CON2_OTG_EN_SHIFT)
                                    );
}

//CON3----------------------------------------------------

unsigned int bq24158_get_vender_code(void)
{
    int ret=0;
    unsigned char val=0;

    ret=bq24158_read_interface(     (unsigned char)(bq24158_CON3), 
                                    (&val),
                                    (unsigned char)(CON3_VENDER_CODE_MASK),
                                    (unsigned char)(CON3_VENDER_CODE_SHIFT)
                                    );
    return val;
}

unsigned int bq24158_get_pn(void)
{
    int ret=0;
    unsigned char val=0;

    ret=bq24158_read_interface(     (unsigned char)(bq24158_CON3), 
                                    (&val),
                                    (unsigned char)(CON3_PIN_MASK),
                                    (unsigned char)(CON3_PIN_SHIFT)
                                    );
    return val;
}

unsigned int bq24158_get_revision(void)
{
    int ret=0;
    unsigned char val=0;

    ret=bq24158_read_interface(     (unsigned char)(bq24158_CON3), 
                                    (&val),
                                    (unsigned char)(CON3_REVISION_MASK),
                                    (unsigned char)(CON3_REVISION_SHIFT)
                                    );
    return val;
}

//CON4----------------------------------------------------

void bq24158_set_reset(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON4), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON4_RESET_MASK),
                                    (unsigned char)(CON4_RESET_SHIFT)
                                    );
}

void bq24158_set_iocharge(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON4), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON4_I_CHR_MASK),
                                    (unsigned char)(CON4_I_CHR_SHIFT)
                                    );
}

void bq24158_set_iterm(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON4), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON4_I_TERM_MASK),
                                    (unsigned char)(CON4_I_TERM_SHIFT)
                                    );
}

//CON5----------------------------------------------------

void bq24158_set_dis_vreg(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON5), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON5_DIS_VREG_MASK),
                                    (unsigned char)(CON5_DIS_VREG_SHIFT)
                                    );
}

void bq24158_set_io_level(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON5), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON5_IO_LEVEL_MASK),
                                    (unsigned char)(CON5_IO_LEVEL_SHIFT)
                                    );
}

unsigned int bq24158_get_sp_status(void)
{
    int ret=0;
    unsigned char val=0;

    ret=bq24158_read_interface(     (unsigned char)(bq24158_CON5), 
                                    (&val),
                                    (unsigned char)(CON5_SP_STATUS_MASK),
                                    (unsigned char)(CON5_SP_STATUS_SHIFT)
                                    );
    return val;
}

unsigned int bq24158_get_en_level(void)
{
    int ret=0;
    unsigned char val=0;

    ret=bq24158_read_interface(     (unsigned char)(bq24158_CON5), 
                                    (&val),
                                    (unsigned char)(CON5_EN_LEVEL_MASK),
                                    (unsigned char)(CON5_EN_LEVEL_SHIFT)
                                    );
    return val;
}

void bq24158_set_vsp(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON5), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON5_VSP_MASK),
                                    (unsigned char)(CON5_VSP_SHIFT)
                                    );
}

//CON6----------------------------------------------------

void bq24158_set_i_safe(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON6), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON6_ISAFE_MASK),
                                    (unsigned char)(CON6_ISAFE_SHIFT)
                                    );
}

void bq24158_set_v_safe(unsigned int val)
{
    int ret=0;    

    ret=bq24158_config_interface(   (unsigned char)(bq24158_CON6), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON6_VSAFE_MASK),
                                    (unsigned char)(CON6_VSAFE_SHIFT)
                                    );
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
void bq24158_dump_register(void)
{
    int i=0;
    printk("[bq24158] ");
    for (i=0;i<bq24158_REG_NUM;i++)
    {
        bq24158_read_byte(i, &bq24158_reg[i]);
        printk("[0x%x]=0x%x ", i, bq24158_reg[i]);        
    }
    printk("\n");
}

#if 0
extern int g_enable_high_vbat_spec;
extern int g_pmic_cid;

void bq24158_hw_init(void)
{    
    if(g_enable_high_vbat_spec == 1)
    {
        if(g_pmic_cid == 0x1020)
        {
            printk("[bq24158_hw_init] (0x06,0x70) because 0x1020\n");
            bq24158_config_interface_liao(0x06,0x70); // set ISAFE
        }
        else
        {
            printk("[bq24158_hw_init] (0x06,0x77)\n");
            bq24158_config_interface_liao(0x06,0x77); // set ISAFE and HW CV point (4.34)
        }
    }
    else
    {
        printk("[bq24158_hw_init] (0x06,0x70) \n");
        bq24158_config_interface_liao(0x06,0x70); // set ISAFE
    }
}
#endif

static int bq24158_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    int err=0; 

    battery_xlog_printk(BAT_LOG_CRTI,"[bq24158_driver_probe] \n");

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }    
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;    

    //---------------------
  //  bq24158_hw_init();
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
    bq24158_config_interface_liao(0x06,0x77);
#else
    bq24158_config_interface_liao(0x06,0x70);
#endif
    bq24158_dump_register();
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
unsigned char g_reg_value_bq24158=0;
static ssize_t show_bq24158_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    battery_xlog_printk(BAT_LOG_FULL,"[show_bq24158_access] 0x%x\n", g_reg_value_bq24158);
    return sprintf(buf, "%u\n", g_reg_value_bq24158);
}
static ssize_t store_bq24158_access(struct device *dev, struct device_attribute *attr,
                                                  const char *buf, size_t count)
{
    int ret=0;
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;
    
    battery_xlog_printk(BAT_LOG_FULL,"[store_bq24158_access] \n");
    
    if(buf != NULL && count != 0)
    {
        //battery_xlog_printk(BAT_LOG_FULL,"[store_bq24158_access] buf is %s and size is %d \n",buf,count);
        reg_address = simple_strtoul(buf,&pvalue,16);
        
        if(count> 3)
        {        
            reg_value = simple_strtoul((pvalue+1),NULL,16);        
            battery_xlog_printk(BAT_LOG_FULL,"[store_bq24158_access] write bq24158 reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            ret=bq24158_config_interface(reg_address, reg_value, 0xFF, 0x0);
        }
        else
        {    
            ret=bq24158_read_interface(reg_address, &g_reg_value_bq24158, 0xFF, 0x0);
            battery_xlog_printk(BAT_LOG_FULL,"[store_bq24158_access] read bq24158 reg 0x%x with value 0x%x !\n",reg_address,g_reg_value_bq24158);
            battery_xlog_printk(BAT_LOG_FULL,"[store_bq24158_access] Please use \"cat bq24158_access\" to get value\r\n");
        }        
    }    
    return count;
}
static DEVICE_ATTR(bq24158_access, 0664, show_bq24158_access, store_bq24158_access); //664

static int bq24158_user_space_probe(struct platform_device *dev)    
{    
    int ret_device_file = 0;

    battery_xlog_printk(BAT_LOG_CRTI,"******** bq24158_user_space_probe!! ********\n" );
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq24158_access);
    
    return 0;
}

struct platform_device bq24158_user_space_device = {
    .name   = "bq24158-user",
    .id     = -1,
};

static struct platform_driver bq24158_user_space_driver = {
    .probe      = bq24158_user_space_probe,
    .driver     = {
        .name = "bq24158-user",
    },
};


//static struct i2c_board_info __initdata i2c_bq24158 = { I2C_BOARD_INFO("bq24158", (bq24158_SLAVE_ADDR_WRITE>>1))};

static int __init bq24158_init(void)
{    
    int ret=0;
    
    battery_xlog_printk(BAT_LOG_CRTI,"[bq24158_init] init start\n");
    
    //i2c_register_board_info(BQ24158_BUSNUM, &i2c_bq24158, 1);

    if(i2c_add_driver(&bq24158_driver)!=0)
    {
        battery_xlog_printk(BAT_LOG_CRTI,"[bq24158_init] failed to register bq24158 i2c driver.\n");
    }
    else
    {
        battery_xlog_printk(BAT_LOG_CRTI,"[bq24158_init] Success to register bq24158 i2c driver.\n");
    }

    // bq24158 user space access interface
    ret = platform_device_register(&bq24158_user_space_device);
    if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI,"****[bq24158_init] Unable to device register(%d)\n", ret);
        return ret;
    }    
    ret = platform_driver_register(&bq24158_user_space_driver);
    if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI,"****[bq24158_init] Unable to register driver (%d)\n", ret);
        return ret;
    }
    
    return 0;        
}

static void __exit bq24158_exit(void)
{
    i2c_del_driver(&bq24158_driver);
}

module_init(bq24158_init);
module_exit(bq24158_exit);
   
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq24158 Driver");
MODULE_AUTHOR("James Lo<james.lo@mediatek.com>");
