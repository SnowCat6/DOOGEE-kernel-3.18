#include <linux/gpio.h>
#include <linux/interrupt.h>
#include "c2k_hw.h"


//#if defined(CONFIG_MTK_C2K_SUPPORT)
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <cust_eint.h>


/*config the gpio to be input for irq if the SOC need*/
int c2k_gpio_direction_input_for_irq(int gpio)
{
#ifdef CONFIG_EVDO_DT_VIA_SUPPORT
	switch(gpio)
	{
		case GPIO_C2K_MDM_RST_IND:
			mt_set_gpio_mode(gpio, GPIO_VIA_MDM_RST_IND_M_EINT);
			break;
		case GPIO_C2K_SDIO_DATA_ACK:
			mt_set_gpio_mode(gpio, GPIO_VIA_SDIO_ACK_M_EINT);
			break;
		case GPIO_C2K_SDIO_FLOW_CTRL:
			mt_set_gpio_mode(gpio, GPIO_VIA_FLOW_CTRL_M_EINT);
			break;		
		case GPIO_C2K_SDIO_MDM_WAKE_AP:
			mt_set_gpio_mode(gpio, GPIO_VIA_MDM_WAKE_AP_M_EINT);
			break;
		case GPIO_C2K_SDIO_MDM_RDY:
			mt_set_gpio_mode(gpio, GPIO_VIA_MDM_RDY_M_EINT);
			break;
	}
    mt_set_gpio_dir(gpio, GPIO_DIR_IN);
#endif
    return 0;
}

//this routine will not be called unless GPIO is valid. So don't worry.
int c2k_gpio_direction_output(int gpio, int value)
{
#ifdef CONFIG_EVDO_DT_VIA_SUPPORT
    mt_set_gpio_mode(gpio, GPIO_MODE_GPIO);
    mt_set_gpio_dir(gpio, GPIO_DIR_OUT);
    mt_set_gpio_out(gpio, !!value);
#endif
    return 0;
}

#ifndef CONFIG_EVDO_DT_VIA_SUPPORT
int c2k_gpio_get_ls(int gpio);
#endif

int c2k_gpio_get_value(int gpio)
{
#ifdef CONFIG_EVDO_DT_VIA_SUPPORT
    if(GPIO_DIR_IN == mt_get_gpio_dir(gpio)){
        return mt_get_gpio_in(gpio);
    }else{
        return mt_get_gpio_out(gpio);
    }
#else
	return c2k_gpio_get_ls(gpio);
	
#endif
}

typedef struct mtk_c2k_gpio_des{
    int gpio;
    int irq;
	unsigned int deb_en;
	unsigned int pol;
    void (*redirect)(void);
    irq_handler_t handle;
    void *data;
#ifndef CONFIG_EVDO_DT_VIA_SUPPORT
	unsigned int irq_type;
	unsigned int eint_ls;		//in order to save the current EINT line status, because there is no way to directly get internal EINT line status.
#endif
}mtk_c2k_gpio_des;

static void gpio_irq_handle_sdio_mdm_rdy(void);
static void gpio_irq_handle_sdio_mdm_wake_ap(void);
static void gpio_irq_handle_rst_ind(void);
static void gpio_irq_handle_data_ack(void);
static void gpio_irq_handle_flow_crtl(void);
#ifndef CONFIG_EVDO_DT_VIA_SUPPORT
static void gpio_irq_handle_excp(void);
#endif

#ifdef CONFIG_EVDO_DT_VIA_SUPPORT
mtk_c2k_gpio_des c2k_gpio_list[] = {
    {GPIO_C2K_SDIO_MDM_RDY, CUST_EINT_EVDO_DT_EXT_MDM_RDY_NUM, 
	CUST_EINT_EVDO_DT_EXT_MDM_RDY_DEBOUNCE_EN,2,
	gpio_irq_handle_sdio_mdm_rdy, NULL, NULL},
    {GPIO_C2K_SDIO_MDM_WAKE_AP, CUST_EINT_EVDO_DT_EXT_MDM_WAKE_AP_NUM, 
    CUST_EINT_EVDO_DT_EXT_MDM_WAKE_AP_DEBOUNCE_EN,2,
    gpio_irq_handle_sdio_mdm_wake_ap, NULL, NULL},
    {GPIO_C2K_MDM_RST_IND,  CUST_EINT_EVDO_DT_EXT_MDM_RST_IND_NUM, 
	CUST_EINT_EVDO_DT_EXT_MDM_RST_IND_DEBOUNCE_EN,2,
	gpio_irq_handle_rst_ind, NULL, NULL},
    {GPIO_C2K_SDIO_DATA_ACK, CUST_EINT_EVDO_DT_EXT_MDM_ACK_NUM, 
	CUST_EINT_EVDO_DT_EXT_MDM_ACK_DEBOUNCE_EN, 2,
	gpio_irq_handle_data_ack, NULL, NULL},
    {GPIO_C2K_SDIO_FLOW_CTRL, CUST_EINT_EVDO_DT_EXT_MDM_FLOW_CTRL_NUM, 
	CUST_EINT_EVDO_DT_EXT_MDM_FLOW_CTRL_DEBOUNCE_EN, 1,
	gpio_irq_handle_flow_crtl, NULL, NULL},
};
#else
mtk_c2k_gpio_des c2k_gpio_list[] = {
	/* gpio						irq							deb_en, pol, resirect,								handle, data, eint_ls*/
    {GPIO_C2K_SDIO_MDM_RDY, 	GPIO_C2K_SDIO_MDM_RDY, 		0, 		2, gpio_irq_handle_sdio_mdm_rdy, 		NULL, NULL, IRQ_TYPE_EDGE_FALLING, 1},
    {GPIO_C2K_SDIO_MDM_WAKE_AP, GPIO_C2K_SDIO_MDM_WAKE_AP, 	0, 		2, gpio_irq_handle_sdio_mdm_wake_ap, 	NULL, NULL, IRQ_TYPE_EDGE_FALLING, 1},
    {GPIO_C2K_MDM_RST_IND,		GPIO_C2K_MDM_RST_IND,		0, 		2, gpio_irq_handle_rst_ind, 			NULL, NULL, IRQ_TYPE_EDGE_FALLING, 1},
    {GPIO_C2K_SDIO_DATA_ACK, 	GPIO_C2K_SDIO_DATA_ACK,		0, 		2, gpio_irq_handle_data_ack, 			NULL, NULL, IRQ_TYPE_EDGE_FALLING, 1},
    {GPIO_C2K_SDIO_FLOW_CTRL, 	GPIO_C2K_SDIO_FLOW_CTRL,	0, 		1, gpio_irq_handle_flow_crtl, 			NULL, NULL, IRQ_TYPE_EDGE_RISING,  1},
	{GPIO_C2K_EXCEPTION,		GPIO_C2K_EXCEPTION,			0,		2, gpio_irq_handle_excp, 				NULL, NULL, IRQ_TYPE_EDGE_FALLING, 1},
};
#endif

static mtk_c2k_gpio_des* gpio_des_find_by_gpio(int gpio)
{
    int i = 0;
    mtk_c2k_gpio_des *des = NULL;

    /*if(gpio < 0){
        return NULL;
    }*/
    
    for(i=0; i < sizeof(c2k_gpio_list) / sizeof(mtk_c2k_gpio_des); i++){
        des = c2k_gpio_list + i;
        if(des->gpio == gpio){
            return des;
        }
    }

    return NULL;
}

static mtk_c2k_gpio_des* gpio_des_find_by_irq(int irq)
{
    int i = 0;
    mtk_c2k_gpio_des *des = NULL;

    for(i=0; i < sizeof(c2k_gpio_list) / sizeof(mtk_c2k_gpio_des); i++){
        des = c2k_gpio_list + i;
        if(des->irq == irq){
            return des;
        }
    }

    return NULL;
}
static void gpio_irq_handle_sdio_mdm_rdy(void)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_gpio(GPIO_C2K_SDIO_MDM_RDY);
    if(des && des->handle){
        des->handle(des->irq, des->data);
    }
}
static void gpio_irq_handle_sdio_mdm_wake_ap(void)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_gpio(GPIO_C2K_SDIO_MDM_WAKE_AP);
    if(des && des->handle){
        des->handle(des->irq, des->data);
    }
}

static void gpio_irq_handle_rst_ind(void)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_gpio(GPIO_C2K_MDM_RST_IND);
    if(des && des->handle){
        des->handle(des->irq, des->data);
    }
}
static void gpio_irq_handle_data_ack(void)
{
    mtk_c2k_gpio_des *des = NULL;
    
    des = gpio_des_find_by_gpio(GPIO_C2K_SDIO_DATA_ACK);
    if(des && des->handle){
        des->handle(des->irq, des->data);
    }
}

static void gpio_irq_handle_flow_crtl(void)
{
    mtk_c2k_gpio_des *des = NULL;
	
    des = gpio_des_find_by_gpio(GPIO_C2K_SDIO_FLOW_CTRL);
    if(des && des->handle){
        des->handle(des->irq, des->data);
    }
}

int c2k_gpio_to_irq(int gpio)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_gpio(gpio);
    if(NULL == des){
        printk("%s: no irq for gpio %d\n", __FUNCTION__, gpio);
        return -1;
    }else{
        return des->irq;
    }
}

int c2k_irq_to_gpio(int irq)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_irq(irq);
    if(NULL == des){
        printk("%s: no gpio for irq %d\n", __FUNCTION__, irq);
        return -1;
    }else{
        return des->gpio;
    }
}

#ifndef CONFIG_EVDO_DT_VIA_SUPPORT

static void gpio_irq_handle_excp(void)
{
    mtk_c2k_gpio_des *des = NULL;
	
    des = gpio_des_find_by_gpio(GPIO_C2K_EXCEPTION);
    if(des && des->handle){
        des->handle(des->irq, des->data);
    }
}

int c2k_irq_set_type(int irq, unsigned int type)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_irq(irq);
    if(NULL == des){
        printk("%s: no gpio for irq %d\n", __FUNCTION__, irq);
        return -1;
    }else{
        des->irq_type = type;
        //printk("[C2K]set irq(%d) type %d\n", irq, type);
    }
	return 0;
}

int c2k_gpio_get_irq_type(int gpio)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_gpio(gpio);
    if(NULL == des){
        printk("%s: no irqtype for gpio %d\n", __FUNCTION__, gpio);
        return -1;
    }else{
        return des->irq_type;
    }
}


int c2k_gpio_set_ls(int gpio, unsigned int ls)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_gpio(gpio);
    if(NULL == des){
        printk("%s: no des for gpio %d\n", __FUNCTION__, gpio);
        return -1;
    }else{
        des->eint_ls = ls;;
    }
	return 0;
}

int c2k_gpio_get_ls(int gpio)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_gpio(gpio);
    if(NULL == des){
        printk("%s: no des for gpio %d\n", __FUNCTION__, gpio);
        return -1;
    }else{
        return des->eint_ls;
    }
}

int c2k_gpio_to_ls(int gpio)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_gpio(gpio);
    if(NULL == des){
        printk("%s: no des for gpio %d\n", __FUNCTION__, gpio);
        return -1;
    }else{
		if (des->irq_type == IRQ_TYPE_EDGE_FALLING)
			des->eint_ls = 0;	//irq triggerred by edge falling interrupt, so line status should be low
		else if (des->irq_type == IRQ_TYPE_EDGE_RISING)
			des->eint_ls = 1;	//irq triggerred by edge rising interrupt, so line status should be high
		else if (des->irq_type == IRQ_TYPE_LEVEL_LOW)
			des->eint_ls = 0;
		else if (des->irq_type == IRQ_TYPE_LEVEL_HIGH)
			des->eint_ls = 1;
    }
    //printk("[C2K]gpio(%d) ls(%d)", gpio, des->eint_ls);
	return des->eint_ls;
}

extern void set_ap_ready(int value);
extern void set_ap_wake_cp(int value);

int c2k_ap_ready_indicate(int value)
{
	if (value != 0 && value != 1){
		printk("%s: invalid para %d\n", __FUNCTION__, value);
		return (-1);
	}
	
	set_ap_ready(value);

	return 0;
}

int c2k_ap_wake_cp(int value)
{
	if (value != 0 && value != 1){
		printk("%s: invalid para %d\n", __FUNCTION__, value);
		return (-1);
	}

	set_ap_wake_cp(value);

	return 0;
}
#endif

int c2k_gpio_set_irq_type(int gpio, unsigned int type)
{
    int irq, level;

    irq = c2k_gpio_to_irq(gpio);
    if(irq < 0){
        return irq;
    }
   
    level = c2k_gpio_get_value(gpio);
 
    if(type == IRQ_TYPE_EDGE_BOTH){
        if(level){
            type = IRQ_TYPE_EDGE_FALLING;
        }else{
            type = IRQ_TYPE_EDGE_RISING;
        }
    }

    if(type == IRQ_TYPE_LEVEL_MASK){
        if(level){
            type = IRQ_TYPE_LEVEL_LOW;
        }else{
            type = IRQ_TYPE_LEVEL_HIGH;
        }
    }
#ifndef CONFIG_EVDO_DT_VIA_SUPPORT
	c2k_irq_set_type(irq, type);
#endif

    mt_eint_set_hw_debounce(irq, 0);
    switch(type){
        case IRQ_TYPE_EDGE_RISING:
            mt_eint_set_sens(irq, MT_EDGE_SENSITIVE);
            mt_eint_set_polarity(irq, MT_POLARITY_HIGH);
            break;
        case IRQ_TYPE_EDGE_FALLING:
            mt_eint_set_sens(irq, MT_EDGE_SENSITIVE);
            mt_eint_set_polarity(irq, MT_POLARITY_LOW);
            break;
        case IRQ_TYPE_LEVEL_HIGH:
            mt_eint_set_sens(irq, MT_LEVEL_SENSITIVE);
            mt_eint_set_polarity(irq, MT_POLARITY_HIGH);
            break;
        case IRQ_TYPE_LEVEL_LOW:
            mt_eint_set_sens(irq, MT_LEVEL_SENSITIVE);
            mt_eint_set_polarity(irq, MT_POLARITY_LOW);
            break;
        default:
            return -EINVAL;
   }
   //printk("[C2K]set irq(%d) type(%d) done\n", irq, type);

   return 0;
}

int c2k_gpio_request_irq(int gpio, irq_handler_t handler, unsigned long flags,
	    const char *name, void *dev)
{
    mtk_c2k_gpio_des *des = NULL;

    des = gpio_des_find_by_gpio(gpio);
    if(des == NULL){
        return -1;
    }
    des->data = dev;
    des->handle = handler;

	printk("[C2K] c2k_gpio_request_irq eintnum %d\n", des->irq);
    //mt_eint_registration(des->irq, des->deb_en, des->pol, des->redirect, 0);
    mt_eint_registration(des->irq, des->pol, des->redirect, 0);
	
    return 0;
}

void c2k_gpio_irq_mask(int gpio)
{
    int irq;

    irq = c2k_gpio_to_irq(gpio);
    if(irq < 0){
        return ;
    }

    mt_eint_mask(irq);
}

void c2k_gpio_irq_unmask(int gpio)
{
    int irq;

    irq = c2k_gpio_to_irq(gpio);
    if(irq < 0){
        return ;
    }

    mt_eint_unmask(irq);
}
//#endif

void c2k_reset_tx_gpio_ready(int gpio)
{
	mtk_c2k_gpio_des *des;
	des = gpio_des_find_by_gpio(gpio);
	des->eint_ls = 1;
	des->irq_type = IRQ_TYPE_EDGE_FALLING;
	c2k_gpio_set_irq_type(gpio, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
}