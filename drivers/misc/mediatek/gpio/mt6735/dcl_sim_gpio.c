//#include <mach/sync_write.h>
#include <mach/mt_typedefs.h>
#include <mach/dcl_sim_gpio.h>
#include <linux/module.h>
#include <linux/i2c.h>

static struct mutex		 sim_gpio_lock;

#define MT6306_I2C_NUMBER                 0
/******************************************************************************
 MACRO Definition
******************************************************************************/
#define MT6306_GPIOTAG                "[MT6306_GPIO] "
#define MT6306_GPIOLOG(fmt, arg...)   printk(MT6306_GPIOTAG fmt, ##arg)
#define MT6306_GPIOMSG(fmt, arg...)   printk(fmt, ##arg)
/*----------------------------------------------------------------------------*/
static struct i2c_client *mt6306_i2c_client = NULL;
static const struct i2c_device_id mt6306_i2c_id[] = {{"mt6306", 0}, {}};
//#ifndef CONFIG_OF
static struct i2c_board_info __initdata i2c_6306 = { I2C_BOARD_INFO("mt6306", 0x64)}; //another is 0x65 if connecting CE0 pin to VIO high
//#endif
/*----------------------------------------------------------------------------*/
void MT6306_Write_Register8(UINT8 var,  UINT8 addr)
{
	char buffer[2];
	buffer[0] = addr;
	buffer[1] = var;
	i2c_master_send(mt6306_i2c_client, buffer, 2);
}

UINT8 MT6306_Read_Register8(UINT8 addr)
{
	UINT8 var;
	i2c_master_send(mt6306_i2c_client, &addr, 1);
	i2c_master_recv(mt6306_i2c_client, &var, 1);
	return var;
}
/*---------------------------------------------------------------------------*/
int mt6306_set_gpio_dir(unsigned long pin, unsigned long dir)
{
    UINT8 val;
    UINT8 mask;

    mutex_lock(&sim_gpio_lock);
    switch (pin) {
    case 2:
	mask = (0x1 << 2);
	val = MT6306_Read_Register8(0x00);
	val &= ~(mask);
 	if (dir == GPIO_DIR_IN)
            val |= ((0x0 << 2)&mask);
    	else
            val |= ((0x1 << 2)&mask);
	MT6306_Write_Register8(val, 0x00);
 	break;
    case 3:
	mask = (0x1 << 0);
	val = MT6306_Read_Register8(0x08);
	val &= ~(mask);
 	if (dir == GPIO_DIR_IN)
            val |= ((0x0 << 0)&mask);
    	else
            val |= ((0x1 << 0)&mask);
	MT6306_Write_Register8(val, 0x08);
 	break;
    case 4:
	mask = (0x1 << 3);
	val = MT6306_Read_Register8(0x01);
	val &= ~(mask);
 	if (dir == GPIO_DIR_IN)
            val |= ((0x0 << 3)&mask);
    	else
            val |= ((0x1 << 3)&mask);
	MT6306_Write_Register8(val, 0x01);
	break;
    case 7:
	mask = (0x1 << 2);
	val = MT6306_Read_Register8(0x05);
	val &= ~(mask);
 	if (dir == GPIO_DIR_IN)
            val |= ((0x0 << 2)&mask);
    	else
            val |= ((0x1 << 2)&mask);
	MT6306_Write_Register8(val, 0x05);
 	break;
    case 10:
	mask = (0x1 << 3);
	val = MT6306_Read_Register8(0x05);
	val &= ~(mask);
 	if (dir == GPIO_DIR_IN)
            val |= ((0x0 << 3)&mask);
    	else
            val |= ((0x1 << 3)&mask);
	MT6306_Write_Register8(val, 0x05);
 	break;
    case 11:
	mask = (0x1 << 3);
	val = MT6306_Read_Register8(0x04);
	val &= ~(mask);
 	if (dir == GPIO_DIR_IN)
            val |= ((0x0 << 3)&mask);
    	else
            val |= ((0x1 << 3)&mask);
	MT6306_Write_Register8(val, 0x04);
 	break;
    case 12:
	mask = (0x1 << 3);
	val = MT6306_Read_Register8(0x08);
	val &= ~(mask);
 	if (dir == GPIO_DIR_IN)
            val |= ((0x0 << 3)&mask);
    	else
            val |= ((0x1 << 3)&mask);
	MT6306_Write_Register8(val, 0x08);
 	break;
    default:
	MT6306_GPIOLOG("pin%ld do not support\n", pin);
  	break;
    }
    mutex_unlock(&sim_gpio_lock);
    MT6306_GPIOLOG("set dir (%ld, %ld) done\n", pin, dir);
    
    return 0/*RSUCCESS*/;
}
/*---------------------------------------------------------------------------*/
int mt6306_get_gpio_dir(unsigned long pin)
{
#if 0    
    unsigned long pos;
    unsigned long bit;
    unsigned long data;
    GPIO_REGS *reg = gpio_reg;

    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;
    
    data = GPIO_RD32(&reg->dir[pos].val);
    return (((data & (1L << bit)) != 0)? 1: 0);        
#endif
    return -1;
}
/*---------------------------------------------------------------------------*/
int mt6306_set_gpio_out(unsigned long pin, unsigned long output)
{
    UINT8 val;
    UINT8 mask;

    mutex_lock(&sim_gpio_lock);
    switch (pin) {
    case 2:
	mask = (0x1 << 0);
	val = MT6306_Read_Register8(0x00);
	val &= ~(mask);
 	if (output == GPIO_OUT_ZERO)
            val |= ((0x0 << 0)&mask);
    	else
            val |= ((0x1 << 0)&mask);
	MT6306_Write_Register8(val, 0x00);
 	break;
    case 3:
	mask = (0x1 << 0);
	val = MT6306_Read_Register8(0x02);
	val &= ~(mask);
 	if (output == GPIO_OUT_ZERO)
            val |= ((0x0 << 0)&mask);
    	else
            val |= ((0x1 << 0)&mask);
	MT6306_Write_Register8(val, 0x02);
 	break;
    case 4:
	mask = (0x1 << 1);
	val = MT6306_Read_Register8(0x01);
	val &= ~(mask);
 	if (output == GPIO_OUT_ZERO)
            val |= ((0x0 << 1)&mask);
    	else
            val |= ((0x1 << 1)&mask);
	MT6306_Write_Register8(val, 0x01);
	break;
    case 7:
	mask = (0x1 << 0);
	val = MT6306_Read_Register8(0x05);
	val &= ~(mask);
 	if (output == GPIO_OUT_ZERO)
            val |= ((0x0 << 0)&mask);
    	else
            val |= ((0x1 << 0)&mask);
	MT6306_Write_Register8(val, 0x05);
 	break;
    case 10:
	mask = (0x1 << 1);
	val = MT6306_Read_Register8(0x05);
	val &= ~(mask);
 	if (output == GPIO_OUT_ZERO)
            val |= ((0x0 << 1)&mask);
    	else
            val |= ((0x1 << 1)&mask);
	MT6306_Write_Register8(val, 0x05);
 	break;
    case 11:
	mask = (0x1 << 1);
	val = MT6306_Read_Register8(0x04);
	val &= ~(mask);
 	if (output == GPIO_OUT_ZERO)
            val |= ((0x0 << 1)&mask);
    	else
            val |= ((0x1 << 1)&mask);
	MT6306_Write_Register8(val, 0x04);
 	break;
    case 12:
	mask = (0x1 << 1);
	val = MT6306_Read_Register8(0x06);
	val &= ~(mask);
 	if (output == GPIO_OUT_ZERO)
            val |= ((0x0 << 1)&mask);
    	else
            val |= ((0x1 << 1)&mask);
	MT6306_Write_Register8(val, 0x06);
 	break;
    default:
	MT6306_GPIOLOG("pin%ld do not support\n", pin);
  	break;
    }
    mutex_unlock(&sim_gpio_lock);
    MT6306_GPIOLOG("set dout(%ld, %ld) done\n", pin, output);
    
    return 0;
}
/*---------------------------------------------------------------------------*/
int mt6306_get_gpio_out(unsigned long pin)
{
#if 0
    unsigned long pos;
    unsigned long bit;
    unsigned long data;
    GPIO_REGS *reg = gpio_reg;

    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;

    data = GPIO_RD32(&reg->dout[pos].val);
    return (((data & (1L << bit)) != 0)? 1: 0);        
#endif
    return -1;
}
/*---------------------------------------------------------------------------*/
int mt6306_set_GPIO_pin_group_power(unsigned long group, unsigned long on)
{
    UINT8 val;
    UINT8 mask;

    switch (group) {
    case 1:
	mask = (0x1 << 2);
	val = MT6306_Read_Register8(0x03);
	val &= ~(mask);
 	if (on == 0)
            val |= ((0x0 << 2)&mask);
    	else
            val |= ((0x1 << 2)&mask);
	MT6306_Write_Register8(val, 0x03);
 	break;
    case 2:
	mask = (0x1 << 3);
	val = MT6306_Read_Register8(0x03);
	val &= ~(mask);
 	if (on == 0)
            val |= ((0x0 << 3)&mask);
    	else
            val |= ((0x1 << 3)&mask);
	MT6306_Write_Register8(val, 0x03);
 	break;
    case 3:
	mask = (0x1 << 2);
	val = MT6306_Read_Register8(0x07);
	val &= ~(mask);
 	if (on == 0)
            val |= ((0x0 << 2)&mask);
    	else
            val |= ((0x1 << 2)&mask);
	MT6306_Write_Register8(val, 0x07);
 	break;
    case 4:
	mask = (0x1 << 3);
	val = MT6306_Read_Register8(0x07);
	val &= ~(mask);
 	if (on == 0)
            val |= ((0x0 << 3)&mask);
    	else
            val |= ((0x1 << 3)&mask);
	MT6306_Write_Register8(val, 0x07);
 	break;
    case 5:
	mask = (0x1 << 3);
	val = MT6306_Read_Register8(0x09);
	val &= ~(mask);
 	if (on == 0)
            val |= ((0x0 << 3)&mask);
    	else
            val |= ((0x1 << 3)&mask);
	MT6306_Write_Register8(val, 0x09);
 	break;
    case 6:
	mask = (0x1 << 2);
	val = MT6306_Read_Register8(0x09);
	val &= ~(mask);
 	if (on == 0)
            val |= ((0x0 << 2)&mask);
    	else
            val |= ((0x1 << 2)&mask);
	MT6306_Write_Register8(val, 0x09);
 	break;
    case 7:
	mask = (0x1 << 1);
	val = MT6306_Read_Register8(0x09);
	val &= ~(mask);
 	if (on == 0)
            val |= ((0x0 << 1)&mask);
    	else
            val |= ((0x1 << 1)&mask);
	MT6306_Write_Register8(val, 0x09);
 	break;
    case 8:
	mask = (0x1 << 0);
	val = MT6306_Read_Register8(0x09);
	val &= ~(mask);
 	if (on == 0)
            val |= ((0x0 << 0)&mask);
    	else
            val |= ((0x1 << 0)&mask);
	MT6306_Write_Register8(val, 0x09);
 	break;
    default:
  	break;
    }

    return 0;
}
/*---------------------------------------------------------------------------*/

static int mt6306_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	mt6306_i2c_client = client;


	/* mt6306 GPIO initial sequence
	   Set pin group 1,2,3,4 to GPIO mode */
	MT6306_Write_Register8(0x0F, 0x1A);


	MT6306_GPIOLOG("addr: 0x1A, value: %x\n", MT6306_Read_Register8(0x1A));

	/* Power on GPIO pin group 1,2 */
	MT6306_Write_Register8(0x0C, 0x03);

	/* Power on GPIO pin group 3,4 */
	MT6306_Write_Register8(0x0C, 0x07);

	/* Switch VIO to 1.8v */
	MT6306_Write_Register8(0x08, 0x10);
#if 0
	/* Power select 2.8v on GPIO pin group 1,2*/
	MT6306_Write_Register8(0x0C, 0x02);

	/* Power select 2.8v on GPIO pin group 3,4*/
	MT6306_Write_Register8(0x0C, 0x06);

	/* Power select 3.0v on GPIO pin group 1,2*/
	MT6306_Write_Register8(0x0F, 0x03);

	/* Power select 3.0v on GPIO pin group 3,4*/
	MT6306_Write_Register8(0x0F, 0x07);
#endif

	MT6306_GPIOLOG("mt6306_i2c_probe, end\n");
    return 0;

}
/*---------------------------------------------------------------------------*/
#ifdef CONFIG_PM 
/*---------------------------------------------------------------------------*/
void mt6306_gpio_suspend(void)
{
	/* Set clock stop H/L to keep/unkeep SCLK/SRST/SIO value */
	MT6306_Write_Register8(0x0F, 0x09);
}
/*---------------------------------------------------------------------------*/
void mt6306_gpio_resume(void)
{
	/* Set clock stop H/L to keep/unkeep SCLK/SRST/SIO value */
	MT6306_Write_Register8(0x00, 0x09);
}
/*---------------------------------------------------------------------------*/
#endif /*CONFIG_PM*/
/*---------------------------------------------------------------------------*/

static int mt6306_i2c_remove(struct i2c_client *client) {return 0;}

#ifdef CONFIG_OF
static const struct of_device_id mt6306_of_match[] = {
	{ .compatible = "mediatek,mt6306",},
	{ },
};
#endif

struct i2c_driver mt6306_i2c_driver = {
    .probe = mt6306_i2c_probe,
    .remove = mt6306_i2c_remove,
    .driver = {
    	.name = "mt6306",
#ifdef CONFIG_OF
	.of_match_table = mt6306_of_match,
#endif
    },
    .id_table = mt6306_i2c_id,
};

/* called when loaded into kernel */
static int __init mt6306_gpio_driver_init(void)
{
    MT6306_GPIOLOG("MT6306 GPIO driver init\n");
    mutex_init(&sim_gpio_lock);

//#ifndef CONFIG_OF
    i2c_register_board_info(MT6306_I2C_NUMBER, &i2c_6306, 1);
//#endif

    if (i2c_add_driver(&mt6306_i2c_driver)!=0) {
    	MT6306_GPIOLOG("mt6306_i2c_driver initialization failed!!\n");
    	return -1;
    } else {
    	MT6306_GPIOLOG("mt6306_i2c_driver initialization succeed!!\n");
    }
    return 0;
}

/* should never be called */
static void __exit mt6306_gpio_driver_exit(void)
{
    MT6306_GPIOLOG("MT6306 GPIO driver exit\n");
}

module_init(mt6306_gpio_driver_init);
module_exit(mt6306_gpio_driver_exit);
