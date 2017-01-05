

extern struct i2c_client *i2c_client;
extern struct tpd_device *tpd;
/***********************************************
//	SET RESET PIN
*/
#define TPD_PIN_RST 	(0)
#define TPD_PIN_EINT 	(1)

static void ft5x0x_set_rst(bool bSet, int nDelay)
{
	tpd_gpio_output(TPD_PIN_RST, bSet?1:0);
	if (nDelay) mdelay(nDelay);
}

/***********************************************
//	SET POWER TOUCHPANEL
*/
extern struct tpd_device* tpd;
static void ft5x0x_power(bool bOn)
{
	int retval;
	if (!tpd->reg)
	{
		tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
		retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	}

	if  (bOn)	{
		retval = regulator_enable(tpd->reg);
	}else{
		retval = regulator_disable(tpd->reg);
	}
}

/***********************************************
//	GET PANEL PRESSURE STSTE
*/
#define TPD_SUPPORT_POINTS	2
struct touch_info
{
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};
static int tpd_touchinfo(struct i2c_client* i2c_client, struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[40] = {0};
	u8 report_rate = 0;
	u16 high_byte, low_byte;

	mutex_lock(&i2c_rw_access);

	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));

	i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[32]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 1, &report_rate);

	mutex_unlock(&i2c_rw_access);

	TPD_DEBUG("FW version=%x]\n",data[32]);

	if(report_rate < 8)
	{
		report_rate = 0x8;
		if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate)) < 0)
			TPD_DMESG("I2C write report rate error, line: %d\n", __LINE__);
	}

	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if((data[0] & 0x70) != 0)
		return false;

	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));
	for(i = 0; i < TPD_SUPPORT_POINTS; i++)
		cinfo->p[i] = 1;	//Put up

	/*get the number of the touch points*/
	cinfo->count = data[2] & 0x0f;

	TPD_DEBUG("Number of touch points = %d\n", cinfo->count);
	TPD_DEBUG("Procss raw data...\n");

	for(i = 0; i < cinfo->count; i++)
	{
		cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; //event flag
		cinfo->id[i] = data[3 + 6 * i + 2] >> 4; //touch id

		/*get the X coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i + 0];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 1];
		low_byte &= 0x00FF;
		cinfo->x[i] = high_byte | low_byte;

		/*get the Y coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 3];
		low_byte &= 0x00FF;
		cinfo->y[i] = high_byte | low_byte;

		TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i , cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}

	return true;
};

/***********************************************
//	CHECK KEYS
*/

extern void tpd_button(unsigned int x, unsigned int y, unsigned int down);
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag = 0;

static void tpd_down(int x, int y, int id)
{
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(tpd->dev);

	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
	{
		tpd_button(x, y, 1);
	}
}

static void tpd_up(int x, int y)
{
    TPD_DEBUG("%s x:%d y:%d\n", __func__, x, y);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_mt_sync(tpd->dev);

	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
	{
		tpd_button(x, y, 0);
	}
}

static int touch_event_handler(void *unused)
{
	int i=0;
	struct touch_info cinfo, pinfo;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };

	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;
		set_current_state(TASK_RUNNING);

#ifdef FTS_GESTRUE
		if (touch_getsure_event_handler(i2c_client)) continue;
#endif

		TPD_DEBUG("touch_event_handler start \n");
		if(tpd_touchinfo(i2c_client, &cinfo, &pinfo) == 0)
			continue;
#ifdef TS_MIUI8_KEYS
		if ((cinfo.y[0] >= TPD_RES_Y) && (pinfo.y[0] < TPD_RES_Y)
		&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
			TPD_DEBUG("Dummy release --->\n");
			tpd_up(pinfo.x[0], pinfo.y[0]);
			input_sync(tpd->dev);
			continue;
		}

		if ((cinfo.y[0] <= TPD_RES_Y && cinfo.y[0] != 0) && (pinfo.y[0] > TPD_RES_Y)
		&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
			TPD_DEBUG("Dummy key release --->\n");
			tpd_button(pinfo.x[0], pinfo.y[0], 0);
			input_sync(tpd->dev);
			continue;
		}
		if ((cinfo.y[0] > TPD_RES_Y) || (pinfo.y[0] > TPD_RES_Y)) 
		{

				if ((cinfo.p[0] == 0) || (cinfo.p[0] == 2)) {
						TPD_DEBUG("Key press --->\n");
						tpd_button(pinfo.x[0], pinfo.y[0], 1);
				} else if ((cinfo.p[0] == 1) &&
					((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
						TPD_DEBUG("Key release --->\n");
						tpd_button(pinfo.x[0], pinfo.y[0], 0);
				}
				input_sync(tpd->dev);

			continue;
		}
#endif
		if(cinfo.count > 0)
		{
		    for(i =0; i < cinfo.count; i++)
		    {
		         tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
		    }
		}else{
		    tpd_up(cinfo.x[0], cinfo.y[0]);
		}
	   	input_sync(tpd->dev);

	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit \n");

	return 0;
}

/***********************************************
//	ATTACH TOUCHPANEL EVENT
*/
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	TPD_DEBUG("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}
static void tpd_irq_registration(struct i2c_client *client)
{
	struct device_node *node = NULL;
	struct task_struct *thread;
	unsigned int touch_irq;
	int retval;

	node = of_find_matching_node(node, touch_of_match);
	if (node)
	{
		tpd_gpio_as_int(TPD_PIN_EINT);
		touch_irq = irq_of_parse_and_map(node, 0);
		retval = request_irq(touch_irq, tpd_eint_interrupt_handler,
						IRQF_TRIGGER_FALLING, TPD_DEVICE, NULL);
	}

#ifdef FTS_GESTRUE
	fts_Gesture_init(tpd->dev);
#endif

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
}

static int tpd_power_on(struct i2c_client *client)
{
	int reset_count;
	u8 retval = 0;

	TPD_DMESG("mtk_tpd: tpd_probe ft5x0x \n");
	ft5x0x_power(true);
	for (reset_count = 0; reset_count < 3; ++reset_count)
	{
		ft5x0x_set_rst(false, 5);
		ft5x0x_set_rst(true, 20);

		if(fts_read_reg(client, 0x00, &retval) >= 0)
			return 1;

		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
	};
	ft5x0x_power(false);
	return 0;
}

extern int tpd_v_magnify_x;
extern int tpd_v_magnify_y;
void tpd_apply_settings(void)
{
	tpd_button_setting(tpd_dts_data.tpd_key_num,
			tpd_dts_data.tpd_key_local,
			tpd_dts_data.tpd_key_dim_local);

	if (tpd_dts_data.touch_filter.enable)
	{
		tpd_v_magnify_x = tpd_dts_data.touch_filter.VECLOCITY_THRESHOLD[0];
		tpd_v_magnify_y = tpd_dts_data.touch_filter.VECLOCITY_THRESHOLD[1];
	}
}

