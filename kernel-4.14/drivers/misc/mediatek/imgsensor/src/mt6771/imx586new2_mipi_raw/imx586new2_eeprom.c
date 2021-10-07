#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>


#define PFX "imx586new2_pdafotp"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_typedef.h"
#include "kd_imgsensor_errcode.h"

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);

#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define imx586new2_EEPROM_READ_ID  0xB0
#define imx586new2_EEPROM_WRITE_ID   0xB1
#define imx586new2_I2C_SPEED        100  
#define imx586new2_MAX_OFFSET		0xFFFF

static int BGr_ratio_Typical = 569;
static int RGr_ratio_Typical = 643;
static int GbGr_ratio_Typical = 1022;
#define AWB1X 0x100
#define LRC_SIZE 384
#define DCC_SIZE 96
#define MTK_IDENTITY_VALUE 0x010B00FF
struct EEPROM_PDAF_INFO {
	kal_uint16 LRC_addr;
	unsigned int LRC_size;
	kal_uint16 DCC_addr;
	unsigned int DCC_size;
};

enum EEPROM_PDAF_INFO_FMT {
	MTK_FMT = 0,
	OP_FMT,
	FMT_MAX
};

static DEFINE_MUTEX(gimx586new2_eeprom_mutex);

typedef struct IMX586NEW2_MIPI_otp_struct{
	kal_uint16 RGr_ratio;
	kal_uint16 BGr_ratio;
	kal_uint16 GbGr_ratio;
}IMX586NEW2_OTP_TYPE;

static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;
static kal_uint8 imx586new2_write_id = 0;

static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > imx586new2_MAX_OFFSET){
        return false;
	}

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, imx586new2_EEPROM_READ_ID)<0){
		return false;
	}
    return true;
}

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para , kal_uint8 i2c_write_id)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    
    iWriteRegI2C(pu_send_cmd, 3, i2c_write_id);
}

static kal_uint8 eeprom_read_byte(kal_uint32 addr)
{
    kal_uint8 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imx586new2_EEPROM_READ_ID);
    return get_byte;
}

static bool _read_imx586new2_eeprom(kal_uint16 addr, BYTE* data, int size ){
	int i = 0;
	int offset = addr;
	LOG_INF("enter _read_eeprom size = %d\n",size);

	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			printk("[MaJian][%s][%d]read eeprom failed\n", __func__, __LINE__);
			return false;
		}
		LOG_INF("read_eeprom 0x%0x %d\n",offset, data[i]);
		offset++;
	}

	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

/*If the function is needed to use, please rewrite the context.*/
static void IMX586NEW2_MIPI_read_otp_wb(IMX586NEW2_OTP_TYPE *otp)
{	
   	kal_uint16 RGr_ratio, BGr_ratio, GbGr_ratio;
    kal_uint8 awb_flag=0;
    
    
    awb_flag = eeprom_read_byte(0x0021);
    if(awb_flag)
    {
        RGr_ratio = (eeprom_read_byte(0x0022)<<8) | eeprom_read_byte(0x0023);
	  	BGr_ratio = (eeprom_read_byte(0x0024)<<8) | eeprom_read_byte(0x0025);
	  	GbGr_ratio = (eeprom_read_byte(0x0026)<<8) | eeprom_read_byte(0x0027);
		LOG_INF("RGr_ratio=0x%x,BGr_ratio=0x%x,GbGr_ratio=0x%x,BGr_ratio_Typical=0x%x,RGr_ratio_Typical=%x\n",RGr_ratio,BGr_ratio,GbGr_ratio,BGr_ratio_Typical,RGr_ratio_Typical);    //247,2d5,400
       	otp->RGr_ratio = RGr_ratio;
       	otp->BGr_ratio = BGr_ratio;	
       	otp->GbGr_ratio = GbGr_ratio;
       	
       	RGr_ratio_Typical = (eeprom_read_byte(0x0028)<<8) | eeprom_read_byte(0x0029);
	  	BGr_ratio_Typical = (eeprom_read_byte(0x002A)<<8) | eeprom_read_byte(0x002B);
	  	GbGr_ratio_Typical = (eeprom_read_byte(0x002C)<<8) | eeprom_read_byte(0x002D);
		LOG_INF("RGr_ratio_Typical=0x%x,BGr_ratio_Typical=0x%x,RGr_ratio_Typical=%x\n",RGr_ratio_Typical,BGr_ratio_Typical,RGr_ratio_Typical);
    }
    else 
    {
       LOG_INF("err : read imx586new2 eeprom no awb data\n");
    }	
}

void IMX586NEW2_MIPI_write_otp_wb(IMX586NEW2_OTP_TYPE *otp,kal_uint8 i2c_write_id)
{
    kal_uint16 RGr_ratio, BGr_ratio, GbGr_ratio;

    kal_uint16 Gb_test1,Gr_test1,R_test,B_test,Gr_test,Gb_test;
    kal_uint16 Gr_test_R, Gr_test_B;
    
    imx586new2_write_id = i2c_write_id;    
    
    RGr_ratio = otp->RGr_ratio;
    BGr_ratio = otp->BGr_ratio;
    GbGr_ratio = otp->GbGr_ratio;
   
    LOG_INF("GbGr_ratio=0x%x,BGr_ratio=0x%x,RGr_ratio=0x%x\n",GbGr_ratio,BGr_ratio,RGr_ratio);

    if(GbGr_ratio<GbGr_ratio_Typical)
    {
        Gr_test1 = AWB1X;
        Gb_test1 = AWB1X *GbGr_ratio_Typical/GbGr_ratio;
    }
    else
    {
        Gb_test1 = AWB1X;
        Gr_test1 = AWB1X *GbGr_ratio/GbGr_ratio_Typical;
    }
    
    if(BGr_ratio<BGr_ratio_Typical)
    {
        if(RGr_ratio<RGr_ratio_Typical)
        {
            Gr_test = AWB1X;
            B_test = AWB1X * BGr_ratio_Typical / BGr_ratio;
            R_test = AWB1X * RGr_ratio_Typical / RGr_ratio;    
        }
        else
        {
            R_test = AWB1X;
            Gr_test = AWB1X * RGr_ratio / RGr_ratio_Typical;
            B_test = Gr_test * BGr_ratio_Typical / BGr_ratio;
        }
    }
    else
    {
        if(RGr_ratio<RGr_ratio_Typical)
        {
            B_test = AWB1X;
            Gr_test = AWB1X * BGr_ratio / BGr_ratio_Typical;
            R_test = Gr_test * RGr_ratio_Typical / RGr_ratio;
        }
        else
        {
            Gr_test_B = AWB1X * BGr_ratio / BGr_ratio_Typical;
            Gr_test_R = AWB1X * RGr_ratio / RGr_ratio_Typical;
            if(Gr_test_B > Gr_test_R)
            {
                B_test = AWB1X;
                Gr_test = Gr_test_B;
                R_test = Gr_test * RGr_ratio_Typical / RGr_ratio;
            }
            else
            {
                R_test = AWB1X;
                Gr_test = Gr_test_R;
                B_test = Gr_test * BGr_ratio_Typical / BGr_ratio;
            }
        }
    }
    R_test = R_test*Gr_test1/AWB1X;
    B_test = B_test*Gr_test1/AWB1X;
    Gr_test = Gr_test*Gr_test1/AWB1X;
    Gb_test = Gr_test*Gb_test1/AWB1X;
    
    write_cmos_sensor_byte(0x020e,Gr_test>>8,imx586new2_write_id);
    write_cmos_sensor_byte(0x020F,Gr_test&0xff,imx586new2_write_id);
    write_cmos_sensor_byte(0x0210,R_test>>8,imx586new2_write_id);
    write_cmos_sensor_byte(0x0211,R_test&0xff,imx586new2_write_id);
    write_cmos_sensor_byte(0x0212,B_test>>8,imx586new2_write_id);
    write_cmos_sensor_byte(0x0213,B_test&0xff,imx586new2_write_id);
    write_cmos_sensor_byte(0x0214,Gb_test>>8,imx586new2_write_id);
    write_cmos_sensor_byte(0x0215,Gb_test&0xff,imx586new2_write_id);
	
    LOG_INF("IMX586NEW2_OTP:Gr_test=0x%x\n",Gr_test);
    LOG_INF("IMX586NEW2_OTP:R_test=0x%x\n",R_test);
    LOG_INF("IMX586NEW2_OTP:B_test=0x%x\n",B_test);
    LOG_INF("IMX586NEW2_OTP:Gb_test=0x%x\n",Gb_test);
    LOG_INF("IMX586NEW2_OTP:End.\n");
}

void IMX586NEW2_MIPI_update_awb(kal_uint8 i2c_write_id)
{
   IMX586NEW2_OTP_TYPE current_otp;  
   IMX586NEW2_MIPI_read_otp_wb(&current_otp);
   IMX586NEW2_MIPI_write_otp_wb(&current_otp,i2c_write_id);
}

bool imx586new2_read_otp_qsc(BYTE* data)
{
	int addr = 0x990;
	int size = 2304;
	int i = 0;	
	
	LOG_INF("read imx586new2 QSC, size = %d, get_done = %d, last_size = %d, last_offset = %d\n", size, get_done, last_size, last_offset);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx586new2_eeprom(addr, data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	
	for (i = 0; i < 4; i++)
		LOG_INF("[%s][%d]data[%d]:0x%x\n", __func__, __LINE__, i, data[i]);

    return true;
}

bool imx586new2_read_otp_lrc(BYTE* data)
{
	int addr = 0x7AF;
	int size = 384;
	int i = 0;	
	
	LOG_INF("read imx586new2 LRC, size = %d, get_done = %d, last_size = %d, last_offset = %d\n", size, get_done, last_size, last_offset);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx586new2_eeprom(addr, data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	
	for (i = 0; i < 4; i++)
		LOG_INF("[%s][%d]data[%d]:0x%x\n", __func__, __LINE__, i, data[i]);

    return true;
}

bool imx586new2CheckVersion(kal_uint32 imgsensor_writeid)
{
	kal_uint8 otp_flag = 0; 
	kal_uint8 checksum = 0;
	kal_uint8 num = 0;
	kal_uint16 sum = 0;
	kal_uint8 data[32] = { 0 };

	_read_imx586new2_eeprom(0x0000,&otp_flag,1);
	LOG_INF("read imx586new2 otp flag = %d\n", otp_flag);
        
	if(!otp_flag)
	{
	    LOG_INF("read otp failed!\n");
	    return false;
	}
	
	_read_imx586new2_eeprom(0x0001,data,32);
	
	for (num = 0;num < 31; num++)
	{		
		sum += data[num];
		LOG_INF("sum:%d data[%d]:%d(0x%x)\n", sum, num, data[num], data[num]);
	}	
	checksum = sum % 255 + 1;
	
	if (checksum != data[31])
	{
	    LOG_INF("checksum failed!checksum:%d(0x%x) data[31]:%d\n", checksum, checksum, data[31]);
	    return false;
	}	

	
	return true;
}

static bool read_imx586new2_eeprom(kal_uint16 addr, BYTE *data, int size)
{
	int i = 0;
	int offset = addr;

	/*LOG_INF("enter read_eeprom size = %d\n", size);*/
	for (i = 0; i < size; i++) {
		if (!selective_read_eeprom(offset, &data[i]))
			return false;
		/*LOG_INF("read_eeprom 0x%0x %d\n", offset, data[i]);*/
		offset++;
	}
	return true;
}

static struct EEPROM_PDAF_INFO eeprom_pdaf_info[] = {
	{/* MTK_FMT */
		.LRC_addr = 0x14FE,
		.LRC_size = LRC_SIZE,
		.DCC_addr = 0x763,
		.DCC_size = DCC_SIZE
	},
	{/* OP_FMT */
		.LRC_addr = 0x1620,
		.LRC_size = LRC_SIZE,
		.DCC_addr = 0x18D0,
		.DCC_size = DCC_SIZE
	},
};

static struct EEPROM_PDAF_INFO *get_eeprom_pdaf_info(void)
{
	static struct EEPROM_PDAF_INFO *pinfo;
	BYTE read_data[4];

	mutex_lock(&gimx586new2_eeprom_mutex);
	if (pinfo == NULL) {
		read_imx586new2_eeprom(0x1, read_data, 4);
		if (((read_data[3] << 24) |
		     (read_data[2] << 16) |
		     (read_data[1] << 8) |
		     read_data[0]) == MTK_IDENTITY_VALUE) {
			pinfo = &eeprom_pdaf_info[MTK_FMT];
		} else {
			pinfo = &eeprom_pdaf_info[OP_FMT];
		}
	}
	mutex_unlock(&gimx586new2_eeprom_mutex);

	return pinfo;
}

unsigned int read_imx586new2_LRC(BYTE *data)
{
	static BYTE IMX586NEW2_LRC_data[LRC_SIZE] = { 0 };
	static unsigned int readed_size;
	struct EEPROM_PDAF_INFO *pinfo = get_eeprom_pdaf_info();

	LOG_INF("read imx586new2 LRC, addr = %d, size = %u\n",
		pinfo->LRC_addr, pinfo->LRC_size);

	mutex_lock(&gimx586new2_eeprom_mutex);
	if ((readed_size == 0) &&
	    read_imx586new2_eeprom(pinfo->LRC_addr,
			       IMX586NEW2_LRC_data, pinfo->LRC_size)) {
		readed_size = pinfo->LRC_size;
	}
	mutex_unlock(&gimx586new2_eeprom_mutex);

	memcpy(data, IMX586NEW2_LRC_data, pinfo->LRC_size);
	return readed_size;
}



