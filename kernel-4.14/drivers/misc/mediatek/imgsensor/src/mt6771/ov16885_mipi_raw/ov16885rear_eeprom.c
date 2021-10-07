//Gionee <Amigo_Skip> <yangym> <20170509>  begin
/*
 * Driver for CAM_CAL
 *
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include <linux/workqueue.h>
#include "cam_cal_define.h"
#include <linux/dma-mapping.h>
#include <linux/regulator/consumer.h>
#include <linux/time.h> 

#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#include "ov16885rear_eeprom.h"
#include "kd_imgsensor.h"
#include "kd_camera_feature.h"
#include "kd_camera_typedef.h"

#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG

#define PFX "OV16885REAR_CamCal"
#define CAM_CALINF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)
#define CAM_CALERR(format, args...)    pr_err(KERN_ERR format, ##args)

#ifdef CAM_CAL_DEBUG
#include <linux/kern_levels.h>

#define CAM_CALDB(format, args...)     pr_debug(PFX "[%s] " format, __func__, ##args)
#else
#define CAM_CALDB(x, ...)
#ifndef BOOL
typedef unsigned char   BOOL;
#endif
#endif

#ifndef CAM_CAL_DTNAME
#define CAM_CAL_DTNAME 	"mediatek,cam_cal_drv1" //CAM_CAL_DRV0
#endif
#ifndef CAM_CAL_DTNAME_I2C
#define CAM_CAL_DTNAME_I2C	"mediatek,cam_cal_ov16885rear"
#endif

#define CAM_CAL_DRVNAME "CAM_CAL_DRV1"  //CAM_CAL_DRV0
#define CAM_CAL_CLASSNAME "CAM_CAL1"  //CAM_CAL0

#define EEPROM_READ_ID  0xA8
#define EEPROM_WRITE_ID  0xA9

#define EEPROM_DATA_SIZE  0x2000

#define VOL_1800 1800000

#ifndef I2C_WR_FLAG
#define I2C_WR_FLAG		(0x1000)
#define I2C_MASK_FLAG	(0x00ff)
#endif
/*******************************************************************************
*
********************************************************************************/
#define BYTE               unsigned char
/*******************************************************************************
*
********************************************************************************/
static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP

static struct workqueue_struct *CAM_CAL2_wq1 = NULL;
static struct delayed_work CAM_CAL12_work;

static struct i2c_client * g_pstI2Cclient = NULL;

static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
static u8 eeprom_Data[EEPROM_DATA_SIZE];

static bool is_preload_eeprom = false;
struct device* CAM_CAL_device = NULL;
static struct regulator *regVCAMIO;
static int g_regVCAMIOEn;

static int selective_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size);

void eepromRegulatorCtrl(int Stage)
{
	if (Stage == 0) {
		if (regVCAMIO == NULL) {
			struct device_node *node, *kd_node;

			/* check if customer camera node defined */
			node = of_find_compatible_node(NULL, NULL, CAM_CAL_DTNAME);

			if (node) {
				kd_node = CAM_CAL_device->of_node;
				CAM_CAL_device->of_node = node;

				regVCAMIO = regulator_get(CAM_CAL_device, "vcamio");

				CAM_CALDB("[Init] regulator_get %p\n", regVCAMIO);

				CAM_CAL_device->of_node = kd_node;
			}
		}
	} else if (Stage == 1) {
		if (regVCAMIO != NULL && g_regVCAMIOEn == 0) {
			int Status = regulator_is_enabled(regVCAMIO);

			CAM_CALDB("regulator_is_enabled %d\n", Status);

			if (!Status) {
				Status = regulator_set_voltage(regVCAMIO, 1800000, 1800000);

				CAM_CALDB("regulator_set_voltage %d\n", Status);

				if (Status != 0)
					CAM_CALDB("regulator_set_voltage fail\n");

				Status = regulator_enable(regVCAMIO);
				CAM_CALDB("regulator_enable %d\n", Status);

				if (Status != 0)
					CAM_CALDB("regulator_enable fail\n");

				g_regVCAMIOEn = 1;
				usleep_range(5000, 5500);
			} else {
				CAM_CALDB("eeprom Power on\n");
			}
		}
	} else {
		if (regVCAMIO != NULL && g_regVCAMIOEn == 1) {
			int Status = regulator_is_enabled(regVCAMIO);

			CAM_CALDB("regulator_is_enabled %d\n", Status);

			if (Status) {
				CAM_CALDB("Camera Power enable\n");

				Status = regulator_disable(regVCAMIO);
				CAM_CALDB("regulator_disable %d\n", Status);
				if (Status != 0)
					CAM_CALDB("Fail to regulator_disable\n");
			}
			/* regulator_put(regVCAMIO); */
			CAM_CALDB("eepromRegulatorCtrl regulator_put %p\n", regVCAMIO);
			/* regVCAMIO = NULL; */
			g_regVCAMIOEn = 0;
		}
	}
}

static int iReadCAM_CAL(u16 a_u2Addr, u32 ui4_length, u8 *a_puBuff)
{
	int  i4RetValue = 0;
	char puReadCmd[2] = {(char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF)};

	/* CAM_CALDB("[CAM_CAL] iReadCAM_CAL!!\n"); */

	if (ui4_length > 8) {
		CAM_CALDB("[BRCB032GWZ] exceed I2c-mt65xx.c 8 bytes limitation\n");
		return -1;
	}

	spin_lock(&g_CAM_CALLock); /* for SMP */
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
	spin_unlock(&g_CAM_CALLock); /* for SMP */

	/* CAM_CALDB("[CAM_CAL] i2c_master_send\n"); */
	i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
	if (i4RetValue != 2) {
		CAM_CALDB("[CAM_CAL] I2C send read address failed!!\n");
		return -1;
	}

	/* CAM_CALDB("[CAM_CAL] i2c_master_recv\n"); */
	i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
	if (i4RetValue != ui4_length) {
		CAM_CALDB("[CAM_CAL] I2C read data failed!!\n");
		return -1;
	}

	spin_lock(&g_CAM_CALLock); /* for SMP */
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & I2C_MASK_FLAG;
	spin_unlock(&g_CAM_CALLock); /* for SMP */

	/* CAM_CALDB("[CAM_CAL] iReadCAM_CAL done!!\n"); */
	return 0;
}

static int iReadData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char *pinputdata)
{
	int  i4RetValue = 0;
	int  i4ResidueDataLength;
	u32 u4IncOffset = 0;
	u32 u4CurrentOffset;
	u8 *pBuff;
	CAM_CALDB("[S24EEPORM] iReadData\n" );

	i4ResidueDataLength = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;
	do {
		if (i4ResidueDataLength >= 8) {
			i4RetValue = iReadCAM_CAL((u16)u4CurrentOffset, 8, pBuff);
			if (i4RetValue != 0) {
				CAM_CALDB("[BRCB032GWZ] I2C iReadData failed!!\n");
				return -1;
			}
			u4IncOffset += 8;
			i4ResidueDataLength -= 8;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
		} else {
			i4RetValue = iReadCAM_CAL((u16)u4CurrentOffset, i4ResidueDataLength, pBuff);
			if (i4RetValue != 0) {
				CAM_CALDB("[BRCB032GWZ] I2C iReadData failed!!\n");
				return -1;
			}
			u4IncOffset += 8;
			i4ResidueDataLength -= 8;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
			/* break; */
		}
	} while (i4ResidueDataLength > 0);

	return 0;
}

static int selective_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size)
{
	unsigned short curAddr = (unsigned short)addr;
	u8 *buff = data;
	u32 size_to_read = size;
	int ret = 0;

	CAM_CALDB("Before byteread_cmos_sensor curAddr =%x count=%d buffData=%x\n", curAddr,
	size - size_to_read, *buff);

	if (iReadData(curAddr, size_to_read, buff) == 0)
		ret = size;
	CAM_CALDB("\n selective_read_region addr =%x size %d readSize = %d\n", addr, size, ret);
	return ret;
}

static void preload_eeprom_data(void)
{
	int ret = 0;

	if(!is_preload_eeprom)
	{
		CAM_CALINF("ov16885rear start!\n");

		ret = selective_read_region(0, eeprom_Data, EEPROM_READ_ID, EEPROM_DATA_SIZE);
        if(ret == EEPROM_DATA_SIZE) {
			CAM_CALINF("eeprom_Data=[0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", 
						eeprom_Data[0], eeprom_Data[1], eeprom_Data[2], eeprom_Data[3], eeprom_Data[4],
						eeprom_Data[5], eeprom_Data[6], eeprom_Data[7], eeprom_Data[8], eeprom_Data[9]);
#if 1
CAM_CALINF("eeprom_Data[0x77A]=[0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", 
			eeprom_Data[0x77A], eeprom_Data[0x77B], eeprom_Data[0x77C], eeprom_Data[0x77D], eeprom_Data[0x77E],
			eeprom_Data[0x77F], eeprom_Data[0x780], eeprom_Data[0x781], eeprom_Data[0x782], eeprom_Data[0x783]);
#endif
			CAM_CALINF("Successed!\n");
            is_preload_eeprom = true;
        }
		else {
			CAM_CALINF("eeprom_Data=[0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", 
						eeprom_Data[0], eeprom_Data[1], eeprom_Data[2], eeprom_Data[3], eeprom_Data[4],
						eeprom_Data[5], eeprom_Data[6], eeprom_Data[7], eeprom_Data[8], eeprom_Data[9]);

			CAM_CALINF("Failed!\n");

			memset(eeprom_Data,0x00,EEPROM_DATA_SIZE);
			is_preload_eeprom = false;
		}

	}
}

static int eeprom_read_region(u32 addr, u8 *data, u32 size)
{
	unsigned short curAddr = (unsigned short)addr;
	u8 *buff = data;

	if (CAM_CAL2_wq1 != NULL)
		cancel_delayed_work_sync(&CAM_CAL12_work);

	preload_eeprom_data();
	memcpy(buff,eeprom_Data + curAddr,size);	

	return size;
}

/* Burst Write Data */
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length,
unsigned char *pinputdata)
{
	CAM_CALDB("[CAM_CAL] not implemented!");
	return 0;
}

static void CAM_CAL12_work_func(struct work_struct *data)
{
	static int retry = 3;

	
	CAM_CALINF("ov16885rear start!\n");

    eepromRegulatorCtrl(1);
	mdelay(1);

	is_preload_eeprom = false;
	preload_eeprom_data();

    eepromRegulatorCtrl(2);

	if(( eeprom_Data[0] == 0x00 ) && ( retry > 0 )){
		retry--;
		CAM_CALERR("Error : ov16885rear read eeprom fail\n");
		if (CAM_CAL2_wq1 != NULL)
			queue_delayed_work(CAM_CAL2_wq1, &CAM_CAL12_work, 1*HZ);
	}
}



/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
	u8 *pBuff = NULL;
	u8 *pu1Params = NULL;
    struct stCAM_CAL_INFO_STRUCT *ptempbuf;		
#ifdef CAM_CALGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif

	CAM_CALDB("[CAM_CAL] ioctl\n"); 

	/*if (_IOC_NONE == _IOC_DIR(a_u4Command)) {
	} else {*/
	if (_IOC_NONE != _IOC_DIR(a_u4Command)) {
        pBuff = kmalloc(sizeof(struct stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
			CAM_CALDB(" ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(struct stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
				CAM_CALDB("[CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (struct stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pu1Params)
    {
        kfree(pBuff);
		CAM_CALDB("[CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
	//CAM_CALDB(" init Working buffer address 0x%p  command is 0x%x\n", pu1Params, a_u4Command);

 
    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
		kfree(pu1Params);
		CAM_CALDB("[CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:    
			CAM_CALDB("[CAM_CAL] Write CMD\n");
			#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
			#endif            
			i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
			#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
			CAM_CALDB("Write data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
			#endif            
            break;
        case CAM_CALIOC_G_READ:
			CAM_CALDB("[CAM_CAL] Read CMD\n");
			#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
			#endif     
            CAM_CALDB("[CAM_CAL] offset=0x%04x length=%d\n", ptempbuf->u4Offset, ptempbuf->u4Length);
			i4RetValue = eeprom_read_region(ptempbuf->u4Offset, pu1Params, ptempbuf->u4Length);
			CAM_CALDB("[CAM_CAL] data[0] 0x%x \n", *(pu1Params+0));
			#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
			CAM_CALDB("Read data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
			#endif            

            break;
        default :
			CAM_CALDB("[CAM_CAL] No CMD\n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
		//CAM_CALDB("[CAM_CAL] to user length %d\n", ptempbuf->u4Length);
		//CAM_CALDB("[CAM_CAL] to user  Working buffer address 0x%p\n", pu1Params);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
			CAM_CALDB("[CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
	kfree(pu1Params);
    return i4RetValue;
}

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            struct COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            struct stCAM_CAL_INFO_STRUCT __user *data)
{
    //compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not unchange */
#if 0
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            struct COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            struct stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);
	return err;
}

static long CAM_CAL_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    //CAM_CALDB("[CAMERA SENSOR] COMPAT_CAM_CALIOC_G_READ\n");
    struct COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    struct stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	CAM_CALDB("[CAMERA SENSOR] CAM_CAL_Ioctl_Compat,%p %p %x ioc size %d\n", filp->f_op ,
	filp->f_op->unlocked_ioctl, cmd, _IOC_SIZE(cmd));

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
    return -ENOTTY;

    switch (cmd) {
		case COMPAT_CAM_CALIOC_G_READ:
		{
			CAM_CALDB("[CAM_CAL] COMPAT_CAM_CALIOC_G_READ\n");

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			err = compat_get_cal_info_struct(data32, data);
			if (err)
				return err;

			ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
			err = compat_put_cal_info_struct(data32, data);


			if(err != 0)
				CAM_CALERR("[CAM_CAL] compat_put_acdk_sensor_getinfo_struct failed\n");
			return ret;
		}
        default:
       
            return -ENOIOCTLCMD;

    }
}
#endif

static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
	CAM_CALDB("[CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[CAM_CAL] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
	
    spin_unlock(&g_CAM_CALLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
    .unlocked_ioctl = CAM_CAL_Ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = CAM_CAL_Ioctl_Compat,
#endif
};

#define GN_BSP_CAM_CAL_OV16885REAR
#ifdef GN_BSP_CAM_CAL_OV16885REAR 

static u16 Camcaldata = 0;
static ssize_t Camcal1_state_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	CAM_CALDB("Camcaldata = 0x%x\n", Camcaldata);
    return sprintf(buf, "0x%x\n", Camcaldata);
}

static ssize_t Camcal1_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	u16 addr;
//	int ret;
	char* addr_end = NULL;

    if ((buf != NULL) && (size != 0))
    {
		if (buf != NULL && strlen(buf) > 0) {
			addr = simple_strtoul(buf, &addr_end, 16);
			if((addr > 0) && (addr < EEPROM_DATA_SIZE))
				Camcaldata = eeprom_Data[addr];			
		} 
    }

#if 1
	{
		int i=0;
		printk("\n\n=====================OTP start cam_cal_ov16885rear=========================\n");
		for(i=0; i<EEPROM_DATA_SIZE; i++){
			if(i == 0x0020)
				printk("\n=====AF data start \n\taddr[0x%04x]:", i);
			else if(i == 0x0030)
				printk("\n=====AWB data start \n\taddr[0x%04x]:", i);
			else if(i == 0x0040)
				printk("\n=====LSC data start \n\taddr[0x%04x]:", i);
			else if(i == 0x0900)
				printk("\n=====PDAF data start \n\taddr[0x%04x]:", i);
			else if(i == 0x0AF8)
				printk("\n=====PDAF Step2 start \n\taddr[0x%04x]:", i);
			else if(i == 0x0E68)
				printk("\n=====SPC data start \n\taddr[0x%04x]:", i);
			else if(i%16 == 0)
				printk("\n\taddr[0x%04x]:", i);
	
			printk("0x%02x ", eeprom_Data[i]);
		}
		printk("\n=====================OTP end cam_cal_ov16885rear=========================\n\n");
	}
#endif

    return size;
}

DEVICE_ATTR(camcal1, S_IWUSR | S_IRUGO, Camcal1_state_show, Camcal1_store);

static struct device_attribute *camcal1_attr[]=
{   
    &dev_attr_camcal1,
};
#endif//GN_BSP_CAM_CAL_OV16885REAR


#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
#ifdef GN_BSP_CAM_CAL_OV16885REAR
	int idx = 0;
	int num = (int)(sizeof(camcal1_attr)/sizeof(camcal1_attr[0]));
#endif

	#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
		CAM_CALDB("[CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
	#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
		CAM_CALDB("[CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
	#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

		CAM_CALDB("[CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
		CAM_CALDB("[CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, CAM_CAL_CLASSNAME);
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
		CAM_CALDB("[CAM_CAL] Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

#ifdef GN_BSP_CAM_CAL_OV16885REAR
	for(idx = 0; idx<num; idx++ )
	{
		int ret = device_create_file(CAM_CAL_device,camcal1_attr[idx]);
		if(ret)
		{
			CAM_CALDB("[CAM_CAL_device] create device file fail!!!\n");
		}
	}
#endif


    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

static int CAM_CAL1_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
	int i4RetValue = 0;
	CAM_CALDB("[CAM_CAL] CAM_CAL_i2c_probe Start!\n");
//    spin_lock_init(&g_CAM_CALLock);

    //get sensor i2c client
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = EEPROM_WRITE_ID>>1;
//	g_pstI2Cclient->timing = 400;
    spin_unlock(&g_CAM_CALLock); // for SMP    

	CAM_CALDB("[CAM_CAL] i2c addr=0x%2x\n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
		CAM_CALDB("[CAM_CAL] register char device failed!\n");
        return i4RetValue;
    }
	eepromRegulatorCtrl(0);
	if (CAM_CAL2_wq1 == NULL)
		CAM_CAL2_wq1 = create_singlethread_workqueue("CAM_CAL2_wq1");
	if (CAM_CAL2_wq1 != NULL) {
		INIT_DELAYED_WORK(&CAM_CAL12_work, CAM_CAL12_work_func);
		queue_delayed_work(CAM_CAL2_wq1, &CAM_CAL12_work, 1*HZ);
	}
	else
	{
		CAM_CALDB("create_freezable_workqueue CAM_CAL2_wq1 failed!\n");
	}

	CAM_CALDB("[CAM_CAL] CAM_CAL_i2c_probe End!\n");
    return 0;                                                                                       
} 

static int CAM_CAL1_i2c_remove(struct i2c_client *client)
{
    
	if (CAM_CAL2_wq1 != NULL) {
			/* flush work queue */
			cancel_delayed_work_sync(&CAM_CAL12_work);
			destroy_workqueue(CAM_CAL2_wq1);
			CAM_CAL2_wq1 = NULL;
		}
    return 0;
}

static const struct i2c_device_id CAM_CAL_i2c_id[] = {
	{CAM_CAL_DRVNAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id CAM_CAL_i2c_of_match[] = {
	{.compatible = CAM_CAL_DTNAME},
	{},
};
#endif

static struct i2c_driver CAM_CAL1_i2c_driver = {
	.driver = {
		.name = CAM_CAL_DRVNAME,
#ifdef CONFIG_OF
		.of_match_table = CAM_CAL_i2c_of_match,
#endif
	},
	.probe = CAM_CAL1_i2c_probe,
	.remove = CAM_CAL1_i2c_remove,
	.id_table = CAM_CAL_i2c_id,
};

static int CAM_CAL1_probe(struct platform_device *pdev)
{
	CAM_CALDB("[CAM_CAL] CAM_CAL1_probe Start!\n");
    return i2c_add_driver(&CAM_CAL1_i2c_driver);
}

static int CAM_CAL1_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL1_i2c_driver);
    return 0;
}

static struct platform_device g_stCAM_CAL1_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

MODULE_DEVICE_TABLE(platform, g_stCAM_CAL1_Device);

// platform structure
static struct platform_driver g_stCAM_CAL1_Driver = {
    .probe		= CAM_CAL1_probe,
    .remove		= CAM_CAL1_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};

static int __init CAM_CAL1_i2C_init(void)
{
	CAM_CALDB("[CAM_CAL] CAM_CAL_i2C_init Start!\n");
	printk("===CAM_CAL1_i2C_init====\n");

    if (platform_device_register(&g_stCAM_CAL1_Device))
    {
		CAM_CALDB("[CAM_CAL1] failed to register CAM_CAL1 driver, 2nd time\n");
        return -ENODEV;
    }
	printk("===CAM_CAL1_i2C_init= 1===\n");

    if(platform_driver_register(&g_stCAM_CAL1_Driver)){
		CAM_CALDB("[CAM_CAL1] failed to register CAM_CAL1 driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit CAM_CAL1_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL1_Driver);
}

module_init(CAM_CAL1_i2C_init);
module_exit(CAM_CAL1_i2C_exit);

MODULE_DESCRIPTION("CAM_CAL1 driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


