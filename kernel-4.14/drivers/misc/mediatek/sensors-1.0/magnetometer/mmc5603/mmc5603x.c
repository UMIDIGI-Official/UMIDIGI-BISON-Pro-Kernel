/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information and source code
 *  contained herein is confidential. The software including the source code
 *  may not be copied and the information contained herein may not be used or
 *  disclosed except with the written permission of MEMSIC Inc. (C) 2017
 *****************************************************************************/
/*
 *
 * mmc5603x.c - mmc5603x magnetic sensor chip driver.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <hwmsensor.h>
#include <sensors_io.h>
#include <linux/types.h>


#include <hwmsen_helper.h>

#include <cust_mag.h>
#include "mmc5603x.h"

#include "mag.h"

/*-------------------------MT6516&MT6573 define-------------------------------*/
#define POWER_NONE_MACRO MT65XX_POWER_NONE

/*----------------------------------------------------------------------------*/
#define MMC5603X_DEV_NAME       "mmc5603x"
#define DRIVER_VERSION          "V80.97.00.10"
/*----------------------------------------------------------------------------*/
#define MEMSIC_DEBUG_ON          		0
#define MEMSIC_DEBUG_FUNC_ON     		0
/* Log define */
#if 0
#define MEMSIC_INFO(fmt, arg...)      	pr_warn("<<-MMC5603X INFO->> "fmt"\n", ##arg)
#else
#define MEMSIC_INFO(fmt, arg...)
#endif
#define MEMSIC_ERR(fmt, arg...)          	pr_err("<<-MMC5603X ERROR->> [line=%d]"fmt"\n",__LINE__,##arg)
#define MEMSIC_DEBUG(fmt, arg...)		do {\
						if (MEMSIC_DEBUG_ON)\
							pr_warn("<<-MMC5603X DEBUG->> [%d]"fmt"\n", __LINE__, ##arg);\
					} while (0)
#define MEMSIC_DEBUG_FUNC()		do {\
						if (MEMSIC_DEBUG_FUNC_ON)\
							pr_debug("<<-MMC5603X FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);\
					} while (0)


#define MMC5603X_RETRY_COUNT	3
#define MMC5603X_DEFAULT_DELAY	100
#define MMC5603X_BUFSIZE  	0x50
#define READMD			0


/* Define Delay time */
#define MMC5603X_DELAY_TM		10	/* ms */
#define MMC5603X_DELAY_SET		50	/* ms */
#define MMC5603X_DELAY_RESET		50  	/* ms */
#define MMC5603X_DELAY_STDN		1	/* ms */

#define MMC5603X_RESET_INTV		250
#define MEMSIC_ABS(a)	((a) > 0 ? (a):(-a))
//static int otpMatrix[3] = {0};

static struct i2c_client *this_client = NULL;

static u32 read_idx = 0;

// calibration msensor and orientation data
static struct mutex sensor_data_mutex;
static struct mutex read_i2c_xyz;


/* static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq); */
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static int mmcd_delay = MMC5603X_DEFAULT_DELAY;

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

static const struct i2c_device_id mmc5603x_i2c_id[] = {{MMC5603X_DEV_NAME,0},{}};

/* Maintain  cust info here */
static struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

/*----------------------------------------------------------------------------*/
static int mmc5603x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mmc5603x_i2c_remove(struct i2c_client *client);
static int mmc5603x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#if 0
static int mmc5603x_suspend(struct i2c_client *client, pm_message_t msg);
static int mmc5603x_resume(struct i2c_client *client);
#endif
static int mmc5603x_local_init(void);
static int mmc5603x_remove(void);
static int mmc5603x_config(int status);

typedef enum {
	MEMSIC_FUN_DEBUG  = 0x01,
	MEMSIC_DATA_DEBUG = 0X02,
	MEMSIC_HWM_DEBUG  = 0X04,
	MEMSIC_CTR_DEBUG  = 0X08,
	MEMSIC_I2C_DEBUG  = 0x10,
} MMC_TRC;

/*----------------------------------------------------------------------------*/
struct mmc5603x_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw;
    atomic_t layout;
    atomic_t trace;
	struct hwmsen_convert   cvt;
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};

static int mmc5603x_init_flag = 0; // 0<==>OK -1 <==> fail



static struct mag_init_info mmc5603x_init_info = {
	 .name = "mmc5603x",
	 .init = mmc5603x_local_init,
	 .uninit = mmc5603x_remove,
};

#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,mag_mmc5603x"},
	{},
};
#endif

static struct i2c_driver mmc5603x_i2c_driver = {
    .driver = {
     //   .owner = THIS_MODULE,
        .name  = MMC5603X_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = mag_of_match,
#endif
    },
	.probe      = mmc5603x_i2c_probe,
	.remove     = mmc5603x_i2c_remove,
	.detect     = mmc5603x_i2c_detect,
#if 0 //!defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend    = mmc5603x_suspend,
	.resume     = mmc5603x_resume,
#endif
	.id_table = mmc5603x_i2c_id,
};

static atomic_t dev_open_count;

static DEFINE_MUTEX(mmc5603x_i2c_mutex);

#ifndef CONFIG_MTK_I2C_EXTENSION
int mag_read_byte(struct i2c_client *client, u8 addr, u8 *data)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,	.flags = 0,
			.len = 1,	.buf = &beg
		},
		{
			.addr = client->addr,	.flags = I2C_M_RD,
			.len = 1,	.buf = data,
		}
	};

	if (!client)
		return -EINVAL;

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		MEMSIC_ERR("i2c_transfer error: (%d %p) %d\n", addr, data, err);
		err = -EIO;
	}

	err = 0;

	return err;
}
static int mag_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	if(len == 1){
		return mag_read_byte(client, addr, data);
	}
	mutex_lock(&mmc5603x_i2c_mutex);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&mmc5603x_i2c_mutex);
		MEMSIC_ERR("Client is Empty\n");
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&mmc5603x_i2c_mutex);
		MEMSIC_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (err != 2) {
		MEMSIC_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&mmc5603x_i2c_mutex);
	return err;

}

static int mag_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0, idx = 0, num = 0;
	char buf[C_I2C_FIFO_SIZE];

	mutex_lock(&mmc5603x_i2c_mutex);
	if (!client) {
		mutex_unlock(&mmc5603x_i2c_mutex);
		MEMSIC_ERR("Client is Empty\n");
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&mmc5603x_i2c_mutex);
		MEMSIC_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&mmc5603x_i2c_mutex);
		MEMSIC_ERR("send command error!!\n");
		return -EFAULT;
	}
	mutex_unlock(&mmc5603x_i2c_mutex);
	return err;
}
#endif

static void mmc5603x_power(struct mag_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;
#if 0
	if(hw->power_id != MT65XX_POWER_NONE)
	{
		//MEMSIC_DEBUG("power %s\n", on ? "on" : "off");
		if(power_on == on)
		{
			//MEMSIC_DEBUG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "mmc5603x"))
			{
				MEMSIC_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "mmc5603x"))
			{
				MEMSIC_ERR("power off fail!!\n");
			}
		}
	}
#endif
	power_on = on;
}

static int I2C_RxData(char *rxData, int length)
{
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_client *client = this_client;
	int res = 0;
	char addr = rxData[0];

	if ((rxData == NULL) || (length < 1))
	{
		MEMSIC_ERR("Invalid param\n");
		return -EINVAL;
	}
	res = mag_i2c_read_block(client, addr, rxData, length);
	if (res < 0)
	{
		MEMSIC_ERR("mag_i2c_read_block error\n");
		return -1;
	}
	return 0;
#else
	uint8_t loop_i = 0;

	int i;
	struct i2c_client *client = this_client;
	struct mmc5603x_i2c_data *data = i2c_get_clientdata(client);
	char addr = rxData[0];

	/* Caller should check parameter validity.*/

	if((rxData == NULL) || (length < 1))
	{
		MEMSIC_ERR("Invalid param\n");
		return -EINVAL;
	}

	for(loop_i = 0; loop_i < MMC5603X_RETRY_COUNT; loop_i++)
	{
		this_client->addr = (this_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG;
		if(i2c_master_send(this_client, (const char*)rxData, ((length<<0X08) | 0X01)))
		{
			break;
		}
		MEMSIC_DEBUG("I2C_RxData delay!\n");
		mdelay(10);
	}

	if(loop_i >= MMC5603X_RETRY_COUNT)
	{
		MEMSIC_ERR("%s retry over %d\n", __func__, MMC5603X_RETRY_COUNT);
		return -EIO;
	}

	if(atomic_read(&data->trace) == MEMSIC_I2C_DEBUG)
	{
		MEMSIC_INFO("RxData: len=%02x, addr=%02x\n  data=", length, addr);
		for(i = 0; i < length; i++)
		{
			MEMSIC_INFO(" %02x", rxData[i]);
		}
	    MEMSIC_INFO("\n");
	}

	return 0;
#endif
}

static int I2C_TxData(char *txData, int length)
{
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_client *client = this_client;
	int res = 0;
	char addr = txData[0];
	u8 *buff = &txData[1];

	if ((txData == NULL) || (length < 2))
	{
		MEMSIC_ERR("Invalid param\n");
		return -EINVAL;
	}
	res = mag_i2c_write_block(client, addr, buff, (length - 1));
	if (res < 0)
		return -1;
	return 0;
#else
	uint8_t loop_i;
	int i;
	struct i2c_client *client = this_client;
	struct mmc5603x_i2c_data *data = i2c_get_clientdata(client);

	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2))
	{
		return -EINVAL;
	}
	this_client->addr = this_client->addr & I2C_MASK_FLAG;
	for(loop_i = 0; loop_i < MMC5603X_RETRY_COUNT; loop_i++)
	{
		if(i2c_master_send(this_client, (const char*)txData, length) > 0)
		{
			break;
		}
		MEMSIC_DEBUG("I2C_TxData delay!\n");
		mdelay(10);
	}

	if(loop_i >= MMC5603X_RETRY_COUNT)
	{
		MEMSIC_ERR("%s retry over %d\n", __func__, MMC5603X_RETRY_COUNT);
		return -EIO;
	}

	if(atomic_read(&data->trace) == MEMSIC_I2C_DEBUG)
	{
		MEMSIC_INFO("TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for(i = 0; i < (length-1); i++)
		{
			MEMSIC_INFO(" %02x", txData[i + 1]);
		}
		MEMSIC_INFO("\n");
	}

	return 0;
#endif
}


static int ECS_ReadXYZData(int *vec, int size)
{
	unsigned char data[6] = {0,0,0,0,0,0};
//	int magraw[3] = {0};
	struct i2c_client *client = this_client;
	struct mmc5603x_i2c_data *clientdata = i2c_get_clientdata(client);

	if(size < 3)
	{
		MEMSIC_ERR("Invalid size value\n");
		return -1;
	}
	mutex_lock(&read_i2c_xyz);

	if (!(read_idx % MMC5603X_RESET_INTV))
	{
		/* Reset Sensor Periodly SET */
	}


	read_idx++;
	data[0] = MMC5603X_REG_DATA;
	if(I2C_RxData(data, 6) < 0)
	{
		mutex_unlock(&read_i2c_xyz);
		MEMSIC_ERR("i2c rxdata failed\n");
		return -EFAULT;
	}
	vec[0] = (uint16_t)(data[0] << 8 | data[1]);
	vec[1] = (uint16_t)(data[2] << 8 | data[3]);
	vec[2] = (uint16_t)(data[4] << 8 | data[5]);

	if(atomic_read(&clientdata->trace) == MEMSIC_DATA_DEBUG)
	{
		MEMSIC_INFO("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
	}

	mutex_unlock(&read_i2c_xyz);
	return 0;
}

static int ECS_GetRawData(int data[3])
{
	int err = 0;
    int data_temp[3] = {0};

	struct mmc5603x_i2c_data *obj = i2c_get_clientdata(this_client);
	if(NULL == obj)
	{
		MEMSIC_ERR("mmc5603x_i2c_data is null!!\n");
		return 0;
	}

	err = ECS_ReadXYZData(data, 3);
	if(err !=0 )
	{
		MEMSIC_ERR("MMC5603X_IOC_TM failed\n");
		return -1;
	}
    data_temp[0] = data[0] - 32768;
    data_temp[1] = data[1] - 32768;
    data_temp[2] = data[2] - 32768;

    //MEMSIC_INFO("coridate before %d %d %d\n",data[0],data[1],data[2]);
    data[obj->cvt.map[0]] = obj->cvt.sign[0] * data_temp[0];
    data[obj->cvt.map[1]] = obj->cvt.sign[1] * data_temp[1];
    data[obj->cvt.map[2]] = obj->cvt.sign[2] * data_temp[2];
   // MEMSIC_INFO("coridate after %d %d %d\n",data[0],data[1],data[2]);

	return err;
}

static int mmc5603x_ReadChipInfo(char *buf, int bufsize)
{
	if((!buf)||(bufsize <= MMC5603X_BUFSIZE -1))
	{
		MEMSIC_ERR("Invalid buff size\n");
		return -1;
	}
	if(!this_client)
	{
		*buf = 0;
		MEMSIC_ERR("Invalid client\n");
		return -2;
	}

	sprintf(buf, "mmc5603x Chip");
	return 0;
}


static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC5603X_BUFSIZE];
	mmc5603x_ReadChipInfo(strbuf, MMC5603X_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	int sensordata[3];
	char strbuf[MMC5603X_BUFSIZE];

	ECS_GetRawData(sensordata);

	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct mmc5603x_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;
	struct mmc5603x_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			MEMSIC_ERR("hwmsen_get_convert function error!\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			MEMSIC_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			MEMSIC_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		MEMSIC_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mmc5603x_i2c_data *obj = i2c_get_clientdata(this_client);
	if(NULL == obj)
	{
		MEMSIC_ERR("mmc5603x_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mmc5603x_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;
	if(NULL == obj)
	{
		MEMSIC_ERR("mmc5603x_i2c_data is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		MEMSIC_ERR("invalid content: '%s', length = %ld\n", buf, count);
	}

	return count;
}


static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC5603X_BUFSIZE];
	sprintf(strbuf, "memsicd5603x");
	return sprintf(buf, "%s", strbuf);
}

static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;
	u8 uData = 0;
	struct mmc5603x_i2c_data *obj = i2c_get_clientdata(this_client);

	if (obj == NULL) {
		MEMSIC_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	uData = atomic_read(&m_flag);

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", uData);
	return res;
}

static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	return 1;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,	 S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,	 S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,   S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(layout,	 S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(trace,	 S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
static DRIVER_ATTR(powerstatus,        S_IRUGO, show_power_status, NULL);
static DRIVER_ATTR(selftest,    S_IRUGO, show_selftest_value, NULL);

static struct driver_attribute *mmc5603x_attr_list[] = {
    	&driver_attr_daemon,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_layout,
	&driver_attr_trace,
	&driver_attr_powerstatus,
	&driver_attr_selftest,
};

static int mmc5603x_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mmc5603x_attr_list)/sizeof(mmc5603x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, mmc5603x_attr_list[idx])))
		{
			MEMSIC_ERR("driver_create_file (%s) = %d\n", mmc5603x_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}

static int mmc5603x_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(mmc5603x_attr_list)/sizeof(mmc5603x_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, mmc5603x_attr_list[idx]);
	}

	return 0;
}


/*----------------------------------------------------------------------------*/

#ifndef	CONFIG_HAS_EARLYSUSPEND
#if 0
static int mmc5603x_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct mmc5603x_i2c_data *obj = i2c_get_clientdata(client);


	if(msg.event == PM_EVENT_SUSPEND)
	{
		mmc5603x_power(obj->hw, 0);
		mmc5603x_config(0);
	}
	return 0;
}
static int mmc5603x_resume(struct i2c_client *client)
{
	struct mmc5603x_i2c_data *obj = i2c_get_clientdata(client);


	mmc5603x_power(obj->hw, 1);


	return 0;
}
#endif
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
static void mmc5603x_early_suspend(struct early_suspend *h)
{
	struct mmc5603x_i2c_data *obj = container_of(h, struct mmc5603x_i2c_data, early_drv);

	if(NULL == obj)
	{
		MEMSIC_ERR("null pointer!!\n");
		return;
	}
	
	mmc5603x_power(obj->hw, 0);
	mmc5603x_config(0);
}

static void mmc5603x_late_resume(struct early_suspend *h)
{
	struct mmc5603x_i2c_data *obj = container_of(h, struct mmc5603x_i2c_data, early_drv);


	if(NULL == obj)
	{
		MEMSIC_ERR("null pointer!!\n");
		return;
	}

	mmc5603x_power(obj->hw, 1);

}
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
static int mmc5603x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, MMC5603X_DEV_NAME);
	return 0;
}

static int check_device(void)
{
	char product_id[2] = {0};

	product_id[0] = MMC5603X_REG_PRODUCTID;
	if(I2C_RxData(product_id, 1) < 0)
	{
		MEMSIC_ERR("[mmc5603x] read id fail\n");
		//read again
		I2C_RxData(product_id, 1);
	}

	MEMSIC_INFO("[mmc5603x] product_id[0] = %d\n",product_id[0]);
	if(product_id[0] != MMC5603x_DEVICE_ID)
	{
		MEMSIC_ERR("Got memsic mmc5603x id failed");
		return -1;
	}
	return 0;
}
static int init_device(void)
{

	return 0;
}

static int mmc5603x_enable(int en)
{
	int value = 0;
	int err = 0;
	value = en;

	if(value == 1)
	{
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&m_flag, 0);
		if(atomic_read(&o_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}
	}
	
	if (atomic_read(&open_flag))
	{
		mmc5603x_config(1);
	}
	else
	{
		mmc5603x_config(0);
	}
	wake_up(&open_wq);
	return err;
}

static int mmc5603x_set_delay(u64 ns)
{
    int value=0;
	value = (int)ns/1000/1000;
	if(value <= 10)
    {
        mmcd_delay = 10;
    }
    else{
        mmcd_delay = value;
	}

	return 0;
}	

static int mmc5603x_open_report_data(int open)
{
	return 0;
}

static int mmc5603x_get_data(int* x ,int* y,int* z, int* status)
{
	int data[3] = {0};
	
	ECS_GetRawData(data);
	
	*x = data[0] * 100;
	*y = data[1] * 100;
	*z = data[2] * 100;
	*status = 3;
	//MEMSIC_INFO("memsic 5603 data x/y/z/status=%d %d %d %d\n",*x,*y,*z,*status);
	return 0;
}

static int mmc5603x_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs / 1000 / 1000;

	if (value <= 10)
		mmcd_delay = 10;
	else
		mmcd_delay = value;
	return 0;
}

static int mmc5603x_flush(void)
{
	return mag_flush_report();
}


static int mmc5603x_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = mmc5603x_enable(enabledisable == true ? 1 : 0);
	if (err) {
		MEMSIC_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = mmc5603x_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		MEMSIC_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
static int mmc5603x_factory_get_data(int32_t data[3], int *status)
{
	/* get raw data */
	return  mmc5603x_get_data(&data[0], &data[1], &data[2], status);
}
static int mmc5603x_factory_get_raw_data(int32_t data[3])
{
	MEMSIC_INFO("do not support mmc5603x_factory_get_raw_data!\n");
	return 0;
}
static int mmc5603x_factory_enable_calibration(void)
{
	return 0;
}
static int mmc5603x_factory_clear_cali(void)
{
	return 0;
}
static int mmc5603x_factory_set_cali(int32_t data[3])
{
	return 0;
}
static int mmc5603x_factory_get_cali(int32_t data[3])
{
	return 0;
}
static int mmc5603x_factory_do_self_test(void)
{
	return 0;
}

static struct mag_factory_fops mmc5603x_factory_fops = {
	.enable_sensor = mmc5603x_factory_enable_sensor,
	.get_data = mmc5603x_factory_get_data,
	.get_raw_data = mmc5603x_factory_get_raw_data,
	.enable_calibration = mmc5603x_factory_enable_calibration,
	.clear_cali = mmc5603x_factory_clear_cali,
	.set_cali = mmc5603x_factory_set_cali,
	.get_cali = mmc5603x_factory_get_cali,
	.do_self_test = mmc5603x_factory_do_self_test,
};

static struct mag_factory_public mmc5603x_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &mmc5603x_factory_fops,
};

static int mmc5603x_config(int en)
{
	unsigned char reg[2] = {0};
    
	reg[0] = MMC5603X_REG_CTRL1;
    reg[1] = MMC5603X_CMD_BW_01;
	if (I2C_TxData(reg, 2) < 0) 
	{
		MEMSIC_ERR("i2c transfer error func: %s  line: %d\n", __func__, __LINE__);
		return -1;
	}
	
	reg[0] = MMC5603X_REG_ODR;
    reg[1] = MMC5603X_SAMPLE_RATE + MMC5603X_SAMPLE_RATE/10;
	if (I2C_TxData(reg, 2) < 0) 
	{
		MEMSIC_ERR("i2c transfer error func: %s  line: %d\n", __func__, __LINE__);
		return -1;
	}

	reg[0] = MMC5603X_REG_CTRL0;
    reg[1] = MMC5603X_CMD_AUTO_SR;
	if (I2C_TxData(reg, 2) < 0) 
	{
		MEMSIC_ERR("i2c transfer error func: %s  line: %d\n", __func__, __LINE__);
		return -1;
	}

	if (en)
	{
		/*turn on continuous mode*/
		reg[0] = MMC5603X_REG_CTRL2;
		reg[1] = MMC5603X_CMD_CM;
		if (I2C_TxData(reg, 2) < 0) 
		{
			MEMSIC_ERR("i2c transfer error func: %s  line: %d\n", __func__, __LINE__);
			return -1;
		}
		MEMSIC_INFO("enable sensor\n");
	} 
	else
	{
		/*turn off continuous mode*/
		reg[0] = MMC5603X_REG_CTRL2;
		reg[1] = 0;
		if (I2C_TxData(reg, 2) < 0) 
		{
			MEMSIC_ERR("i2c transfer error func: %s  line: %d\n", __func__, __LINE__);
			return -1;
		}
		MEMSIC_INFO("disaable sensor\n");
	}

	msleep(1);
	return 1;
}
static int mmc5603x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct mmc5603x_i2c_data *data;
	int err = 0;
	struct mag_control_path ctl={0};
	struct mag_data_path mag_data={0};

	MEMSIC_INFO("%s: enter probe,driver version=%s\n", __func__,DRIVER_VERSION);

	//hw = get_mag_dts_func(client->dev.of_node, hw);
	//if (!hw) {
	//	MEMSIC_ERR("get dts info fail\n");
	//	err = -EFAULT;
	//	goto exit;
	//}
    MEMSIC_INFO("direction =%d\n",hw->direction);


	if(!(data = kmalloc(sizeof(struct mmc5603x_i2c_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		MEMSIC_ERR("Allocate memory to struct failed\n");
		goto exit;
	}
	memset(data, 0, sizeof(struct mmc5603x_i2c_data));

	data->hw = hw;

    err = hwmsen_get_convert(data->hw->direction, &data->cvt);
    if(err) {
        MEMSIC_ERR("invalid direction: %d\n", data->hw->direction);
        goto exit;
    }

	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);

	mutex_init(&sensor_data_mutex);
	mutex_init(&read_i2c_xyz);

	/*init_waitqueue_head(&data_ready_wq);*/
	init_waitqueue_head(&open_wq);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	this_client = new_client;
    msleep(10);

	err = check_device();
	if(err < 0)
	{
		MEMSIC_ERR("mmc5603x probe: check device connect error\n");
		goto exit_kfree;
	}

    err = init_device();
	if(err < 0)
	{
		MEMSIC_ERR("mmc5603x probe: init_device error\n");
		goto exit_kfree;
	}
	
	/* Register sysfs attribute */
	if((err = mmc5603x_create_attr(&(mmc5603x_init_info.platform_diver_addr->driver))))
	{
		MEMSIC_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	err = mag_factory_device_register(&mmc5603x_factory_device);
	if (err) {
		MEMSIC_ERR("misc device register failed, err = %d\n", err);
		goto exit_factory_device_register_failed;
	}
	ctl.is_use_common_factory = false;
	ctl.enable = mmc5603x_enable;
	ctl.set_delay = mmc5603x_set_delay;
	ctl.open_report_data = mmc5603x_open_report_data;
	ctl.batch = mmc5603x_batch;
	ctl.flush = mmc5603x_flush;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = data->hw->is_batch_supported;
	#if 1 //vsun,wzd modify
	strlcpy(ctl.libinfo.libname, "memsicd5603x", sizeof(ctl.libinfo.libname));
	//ctl.libinfo.layout = AKECS_SetCert();
	ctl.libinfo.deviceid = MMC5603x_DEVICE_ID; //akm_device;	
	#else
	ctl.lib_name = "memsicd5603x";
	#endif

	err = mag_register_control_path(&ctl);
	if(err)
	{
		MEMSIC_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

	mag_data.div = CONVERT_M_DIV;
	mag_data.get_data = mmc5603x_get_data;

	err = mag_register_data_path(&mag_data);
	if(err)
	{
		MEMSIC_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

#if defined CONFIG_HAS_EARLYSUSPEND
	data->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	data->early_drv.suspend  = mmc5603x_early_suspend,
	data->early_drv.resume   = mmc5603x_late_resume,
	register_early_suspend(&data->early_drv);
#endif

	MEMSIC_INFO("mmc5603X IIC probe successful !");

	mmc5603x_init_flag = 1;
	return 0;

	exit_sysfs_create_group_failed:
	exit_factory_device_register_failed:

	exit_kfree:
	kfree(data);
	exit:
	MEMSIC_ERR("%s: err = %d\n", __func__, err);
	mmc5603x_init_flag = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int mmc5603x_i2c_remove(struct i2c_client *client)
{
	int err;

	if((err = mmc5603x_delete_attr(&(mmc5603x_init_info.platform_diver_addr->driver))))
	{
		MEMSIC_ERR("mmc5603x_delete_attr fail: %d\n", err);
	}

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/

static int mmc5603x_local_init(void)
{

	mmc5603x_power(hw, 1);

	atomic_set(&dev_open_count, 0);


	if(i2c_add_driver(&mmc5603x_i2c_driver))
	{
		MEMSIC_ERR("add driver error\n");
		return -1;
	}
	if(-1 == mmc5603x_init_flag)
	{
	   i2c_del_driver(&mmc5603x_i2c_driver);
	   MEMSIC_ERR("mmc5603x init failed\n");
	   return -1;
	}

	return 0;
}

static int mmc5603x_remove(void)
{

	mmc5603x_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&mmc5603x_i2c_driver);
	return 0;
}



/*----------------------------------------------------------------------------*/
static int __init mmc5603x_init(void)
{
#if 1
	const char *name = "mediatek,mmc5603x";

	hw = get_mag_dts_func(name, hw);

	MEMSIC_INFO("mmc5603x_init addr0 = 0x%x,addr1 = 0x%x,i2c_num = %d \n",hw->i2c_addr[0],hw->i2c_addr[1],hw->i2c_num);

	if (!hw)
	{
		MEMSIC_ERR("get dts info fail\n");
	}
#endif

	mag_driver_add(&mmc5603x_init_info);

	return 0;
}


/*----------------------------------------------------------------------------*/
static void __exit mmc5603x_exit(void)
{
	//platform_driver_unregister(&mmc5603x_platform_driver);
}
/*----------------------------------------------------------------------------*/
module_init(mmc5603x_init);
module_exit(mmc5603x_exit);

MODULE_AUTHOR("Aaron Peng<hcpeng@memsic.cn>");
MODULE_DESCRIPTION("MEMSIC mmc5603KJ Magnetic Sensor Chip Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
