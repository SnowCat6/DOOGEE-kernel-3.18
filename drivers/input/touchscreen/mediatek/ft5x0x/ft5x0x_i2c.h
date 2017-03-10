#include <linux/dma-mapping.h>

extern struct tpd_device *tpd;
 #define IIC_DMA_MAX_TRANSFER_SIZE     250

u8 *I2CDMABuf_va = NULL;
dma_addr_t I2CDMABuf_pa = 0;
static DEFINE_MUTEX(i2c_rw_access);

void fts_i2c_Init(void)
{
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	I2CDMABuf_va = (u8 *) dma_alloc_coherent(&tpd->dev->dev, IIC_DMA_MAX_TRANSFER_SIZE, &I2CDMABuf_pa, GFP_KERNEL);
	if (I2CDMABuf_va)
		memset(I2CDMABuf_va, 0, IIC_DMA_MAX_TRANSFER_SIZE);
}

int fts_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret = 0;
	int i,i2;

	mutex_lock(&i2c_rw_access);

	if (I2CDMABuf_va == NULL)
		fts_i2c_Init();

	client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;

	for(i = 0; i < writelen; i += ret)
	{
		for(i2 = 0; i2 < min(IIC_DMA_MAX_TRANSFER_SIZE, writelen - i); i2++){
			I2CDMABuf_va[i2] = writebuf[i + i2];
		}
		ret = i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, i2);
		if(ret != i2){
			mutex_unlock(&i2c_rw_access);
			return -1;
		}
	}

	for(i = 0; i < readlen; i += ret)
	{
		ret = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa,
					min(IIC_DMA_MAX_TRANSFER_SIZE, readlen - i));
		if (ret <= 0) return -1;

		for(i2 = 0; i2 < ret; i2++){
			readbuf[i + i2] = I2CDMABuf_va[i2];
		}
	}

	client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);

	mutex_unlock(&i2c_rw_access);

	return i;
}

int fts_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	return fts_i2c_Read(client, writebuf, writelen, writebuf, 0);
}

int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_Read(client, buf, sizeof(buf), buf, 0);
}
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_Read(client, &regaddr, 1, regvalue, 1);
}
