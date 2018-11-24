#include <linux/interrupt.h>
#include <mt_boot_common.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>

#include "tpd.h"
#include "ft5x0x_i2c.h"

//#define TPD_CLOSE_POWER_IN_SLEEP

#if !defined(CONFIG_TPD_CLOSE_POWER_IN_SLEEP) || defined(CONFIG_HCT_TP_GESTRUE)
#include "ft5x0x_getsure.h"
#endif

#include "ft5x0x_util.h"

extern struct tpd_device *tpd;
struct i2c_client *i2c_client = NULL;

static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);
	return 0;
}

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	i2c_client = client;
	if (tpd_power_on(client) == 0)
		return -1;
	tpd_irq_registration(client);
	// Extern variable MTK touch driver
    	tpd_load_status = 1;
	return 0;
}

static int tpd_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");
	return 0;
}

static const struct i2c_device_id ft5x0x_tpd_id[] = {{"ft5x0x",0},{}};
static const struct of_device_id ft5x0x_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};

MODULE_DEVICE_TABLE(of, ft5x0x_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = ft5x0x_dt_match,
		.name = "ft5x0x",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft5x0x_tpd_id,
	.detect = tpd_detect,
};
/********************************************/
static int tpd_local_init(void)
{
	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}

	TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

static void tpd_resume(struct device *h)
{
#ifdef FTS_GESTRUE
	if (tpd_getsure_resume(i2c_client)) return;
#endif

#ifdef TPD_CLOSE_POWER_IN_SLEEP
   	TPD_DMESG("TPD wake up\n");
	ft5x0x_set_rst(false, 5);
	ft5x0x_power(true);
	ft5x0x_set_rst(true, 20);
#else

#endif
	TPD_DMESG("TPD wake up done\n");
}

static void tpd_suspend(struct device *h)
{
//	static char data;

#ifdef FTS_GESTRUE
	if (tpd_getsure_suspend(i2c_client)) return;
#endif

	TPD_DEBUG("TPD enter sleep\n");
//	data = 0x3;
//	i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
	fts_write_reg(i2c_client, 0xA5, 0x3);

#ifdef TPD_CLOSE_POWER_IN_SLEEP
	ft5x0x_power(false);
#endif
}


static struct device_attribute *ft5x0x_attrs[] = {

#ifdef CONFIG_HCT_TP_GESTRUE
	 &dev_attr_tpgesture,
	 &dev_attr_tpgesture_status,
#endif
};

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "FT5x0x",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
	.attrs = {
		.attr = ft5x0x_attrs,
		.num  = ARRAY_SIZE(ft5x0x_attrs),
	},
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	printk("MediaTek FT5x0x touch panel driver init\n");
	tpd_get_dts_info();
	tpd_apply_settings();

	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FT5x0x driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("MediaTek FT5x0x touch panel driver exit\n");
       	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

