/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX586NEW2mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx586new2mipiraw_Sensor.h"

#ifdef XUNHU_LPS_TEKHW_SUPPORT
#include <teksunhw.h>/*add by xunhu andy andy20160720 at 22:49*/
static int tekhw_imx586new2_mirror = 0xff;/*add by xunhu andy andy20160720 at 22:48*/
static int tekhw_imx586new2_color =0xff;
static int tekhw_times=0;
#endif
#define PFX "IMX586NEW2_camera_sensor"

#define DEVICE_VERSION_IMX586NEW2     "imx586new2"
#define LOG_INF(format, args...)	printk(PFX "[%s] " format, __func__, ##args)
static DEFINE_SPINLOCK(imgsensor_drv_lock);

//extern void IMX586NEW2_MIPI_update_awb(kal_uint8 i2c_write_id);
//extern bool Imx586CheckVersion(kal_uint32 imgsensor_writeid);
//#define BENJA_IMX586NEW2_LRC

#define DATA_SIZE 2304
#define BYTE      unsigned char
static BYTE imx586new2_qsc_data[DATA_SIZE] = { 0 };
//extern bool imx586new2_read_otp_qsc(BYTE* data);

#define LRC_SIZE 384
#ifdef BENJA_IMX586NEW2_LRC
static BYTE imx586new2_lrc_data[LRC_SIZE] = { 0 };
#endif

//extern bool imx586new2_read_otp_lrc(BYTE* data);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX586NEW2_SENSOR_ID,

	.module_id = 0x01,  /* 0x01 Sunny,0x05 QTEK */

	.checksum_value = 0xffb1ec31,

	.pre = {
		/*setting for normal binning*/
		.pclk = 864000000,
		.linelength = 8976,
		.framelength = 3208,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 534400000,
		.max_framerate = 300,
	},
	.cap = {
		/*setting for normal binning*/
		.pclk = 734400000,
		.linelength = 7872,
		.framelength = 3108,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 394400000,
		.max_framerate = 300,
	},
	.normal_video = {
		/*setting for normal binning*/
		.pclk = 864000000,
		.linelength = 8976,
		.framelength = 3208,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 534400000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 552000000,
		.linelength = 5376,
		.framelength = 854,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 216000000,
		.max_framerate = 1200,

	},
	.slim_video = {
		/*setting for normal binning*/
		.pclk = 864000000,
		.linelength = 8976,
		.framelength = 3208,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 534400000,
		.max_framerate = 300,
	},
#if 0	
	.custom1 = {		/*32M */
		.pclk = 1483200000,
		.linelength = 14704,
		.framelength = 5933,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 6560,
		.grabwindow_height = 4928,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 696000000,
		.max_framerate = 170,
	},
#else
	.custom1 = {
		/*setting for normal binning*/
		.pclk = 864000000,
		.linelength = 8976,
		.framelength = 3208,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 534400000,
		.max_framerate = 300,
	},
#endif
	.custom2 = {		/*48M@15fps*/
		.pclk = 864000000,
		.linelength = 8976,
		.framelength = 3208,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 534400000,
		.max_framerate = 300,

	},
	.custom3 = {		/*stero@34fps*/
		.pclk = 432000000,
		.linelength = 5376,
		.framelength = 2678,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2000,
		.grabwindow_height = 1500,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 219600000,
		.max_framerate = 300,
	},	
	
	.margin = 48,
	.min_shutter = 16,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  /* 1, support; 0,not support */
	.ihdr_le_firstline = 0,  /* 1,le first ; 0, se first */
	.sensor_mode_num = 8,	  /* support sensor mode num */
	.cap_delay_frame = 3,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	
	.custom1_delay_frame = 2,	/*32M */
	.custom2_delay_frame = 2,	/*48M@15fps*/
	.custom3_delay_frame = 2,	/*stero@34fps*/
	
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, /* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0, /* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */

	//[agold][xfl][20190313][start]
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	//[agold][xfl][20190313][end]

	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x34, 0xff},
	.i2c_speed = 320, /* i2c read/write speed */
};


static struct imgsensor_struct imgsensor = {
	//[agold][xfl][20190313][start]
	.mirror = IMAGE_NORMAL,				/* mirrorflip information */
	//[agold][xfl][20190313][end]
	
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview,
	*  Capture, Video,High Speed Video, Slim Video
	*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,					/* current shutter */
	.gain = 0x100,						/* current gain */
	.dummy_pixel = 0,					/* current dummypixel */
	.dummy_line = 0,					/* current dummyline */
	.current_fps = 0,  /* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	/* auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker */
	.autoflicker_en = KAL_FALSE,
	/* test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output */
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/* current scenario id */
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.hdr_mode = 0, /* HDR mODE : 0: disable HDR, 1:IHDR, 2:HDR, 9:ZHDR */
	.i2c_write_id = 0x34,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] = {
{ 8000,  6000,  0,  0,  8000,  6000,  4000,  3000,  0000,  0000,  4000,  3000,  0,  0,  4000,  3000},/*Preview*/
{ 8000,  6000,  0,  0,  8000,  6000,  4000,  3000,  0000,  0000,  4000,  3000,  0,  0,  4000,  3000}, /* capture*/
{ 8000,  6000,  0,  0,  8000,  6000,  4000,  3000,  0000,  0000,  4000,  3000,  0,  0,  4000,  3000}, /* video */
{ 8000,  6000,  0,  0,  8000,  6000,  2000,  1500,  0360,  390,  1280,  0720,  0,  0,  1280,  0720}, /*hs video*/
{ 8000,  6000,  0,  0,  8000,  6000,  4000,  3000,  0000,  0000,  4000,  3000,  0,  0,  4000,  3000}, /* slim video*/
#if 0
{ 8000,  6000,  0,  0,  8000,  6000,  6736,  5052,  88,  	62,   6560,  4928,  0,  0,  6560,  4928}, /*custom1 32M */
#else
{ 8000,  6000,  0,  0,  8000,  6000,  4000,  3000,  0000,  0000,  4000,  3000,  0,  0,  4000,  3000}, /*custom1 12M */
#endif
{ 8000,  6000,  0,  0,  8000,  6000,  8000,  6000,  0000,  0000,  8000,  6000,  0,  0,  8000,  6000}, /*custom2 48M@15fps*/
{ 8000,  6000,  0,  0,  8000,  6000,  2000,  1500,  0000,  0000,  2000,  1500,  0,  0,  2000,  1500}, /*custom3 stero@34fps*/
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting */
	{0x05, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0B40, 0x086C, 0x00, 0x12, 0x0E10, 0x0002, /*VC0:raw, VC1:Embedded data*/
	 0x00, 0x34, 0x04D8, 0x05D0, 0x00, 0x00, 0x0000, 0x0000}, /*VC2:Y HIST(3HDR), VC3:AE HIST(3HDR)*/
	 //0x00, 0x33, 0x0E10, 0x0001, 0x00, 0x00, 0x0000, 0x0000}, /*VC4:Flicker(3HDR), VC5:no data*/
	/* Capture mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1680, 0x10D8, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x34, 0x04D8, 0x05D0, 0x00, 0x00, 0x0000, 0x0000},
	/* Video mode setting */
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0B40, 0x086C, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x34, 0x04D8, 0x05D0, 0x00, 0x00, 0x0000, 0x0000}
};
/* If mirror flip */
static struct SET_PD_BLOCK_INFO_T  imgsensor_pd_info = {
	.i4OffsetX = 15,
	.i4OffsetY = 12,
	.i4PitchX  =  8,
	.i4PitchY  = 16,
	.i4PairNum  = 8,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = { {15, 12}, {17, 14}, {21, 16}, {19, 18},
		   {15, 20}, {17, 22}, {21, 24}, {19, 26} },
	.i4PosR = { {16, 12}, {18, 14}, {22, 16}, {20, 19},
		   {16, 20}, {18, 22}, {22, 24}, {20, 26} },
	.i4BlockNumX = 496,
	.i4BlockNumY = 186,
	//.iMirrorFlip = 3,
	//.i4Crop = { {0, 0}, {0, 0}, {0, 200}, {0, 0}, {0, 372},
	//	    {0, 0}, {80, 420}, {0, 0}, {0, 0}, {0, 0} },
};


#define IMX586NEW2MIPI_MaxGainIndex (223)
kal_uint16 IMX586NEW2MIPI_sensorGainMapping[IMX586NEW2MIPI_MaxGainIndex][2] ={
	{72,114},
	{76,162},
	{80,205},
	{84,244},
	{88,279},
	{92,312},
	{96,341},
	{100,369},
	{104,394},
	{108,417},
	{112,439},
	{116,459},
	{120,478},
	{124,495},
	{128,512},
	{132,528},
	{136,542},
	{140,556},
	{144,569},
	{148,581},
	{152,593},
	{156,604},
	{160,614},
	{164,624},
	{168,634},
	{172,643},
	{176,652},
	{180,660},
	{184,668},
	{188,675},
	{192,683},
	{196,690},
	{200,696},
	{204,703},
	{208,709},
	{212,715},
	{216,721},
	{220,726},
	{224,731},
	{228,737},
	{232,742},
	{236,746},
	{240,751},
	{244,755},
	{248,760},
	{252,764},
	{256,768},
	{260,772},
	{264,776},
	{267,779},
	{272,783},
	{277,787},
	{280,790},
	{284,793},
	{287,796},
	{293,800},
	{297,803},
	{301,806},
	{303,808},
	{308,811},
	{312,814},
	{317,817},
	{320,819},
	{324,822},
	{328,824},
	{333,827},
	{336,829},
	{340,831},
	{343,833},
	{349,836},
	{352,838},
	{356,840},
	{360,842},
	{364,844},
	{368,846},
	{372,848},
	{377,850},
	{381,852},
	{383,853},
	{388,855},
	{392,857},
	{397,859},
	{400,860},
	{405,862},
	{407,863},
	{412,865},
	{415,866},
	{420,868},
	{423,869},
	{428,871},
	{431,872},
	{437,874},
	{440,875},
	{443,876},
	{449,878},
	{452,879},
	{455,880},
	{462,882},
	{465,883},
	{468,884},
	{471,885},
	{475,886},
	{478,887},
	{485,889},
	{489,890},
	{493,891},
	{496,892},
	{500,893},
	{504,894},
	{508,895},
	{512,896},
	{516,897},
	{520,898},
	{524,899},
	{529,900},
	{533,901},
	{537,902},
	{542,903},
	{546,904},
	{551,905},
	{555,906},
	{560,907},
	{565,908},
	{570,909},
	{575,910},
	{580,911},
	{585,912},
	{590,913},
	{596,914},
	{601,915},
	{607,916},
	{612,917},
	{618,918},
	{624,919},
	{630,920},
	{636,921},
	{643,922},
	{649,923},
	{655,924},
	{662,925},
	{669,926},
	{676,927},
	{683,928},
	{690,929},
	{697,930},
	{705,931},
	{712,932},
	{720,933},
	{728,934},
	{736,935},
	{745,936},
	{753,937},
	{762,938},
	{771,939},
	{780,940},
	{790,941},
	{799,942},
	{809,943},
	{819,944},
	{830,945},
	{840,946},
	{851,947},
	{862,948},
	{874,949},
	{886,950},
	{898,951},
	{910,952},
	{923,953},
	{936,954},
	{950,955},
	{964,956},
	{978,957},
	{993,958},
	{1008,959},
	{1024,960},
	{1040,961},
	{1057,962},
	{1074,963},
	{1092,964},
	{1111,965},
	{1130,966},
	{1150,967},
	{1170,968},
	{1192,969},
	{1214,970},
	{1237,971},
	{1260,972},
	{1285,973},
	{1311,974},
	{1337,975},
	{1365,976},
	{1394,977},
	{1425,978},
	{1456,979},
	{1489,980},
	{1524,981},
	{1560,982},
	{1598,983},
	{1638,984},
	{1680,985},
	{1725,986},
	{1771,987},
	{1820,988},
	{1872,989},
	{1928,990},
	{1986,991},
	{2048,992},
	{2114,993},
	{2185,994},
	{2260,995},
	{2341,996},
	{2427,997},
	{2521,998},
	{2621,999},
	{2731,1000},
	{2849,1001},
	{2979,1002},
	{3121,1003},
	{3277,1004},
	{3449,1005},
	{3641,1006},
	{3855,1007},
	{4096,1008},
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/ /* Add this func to set i2c speed by each sensor */
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/ /* Add this func to set i2c speed by each sensor */
	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("frame_length = %d, line_length = %d\n",
		imgsensor.frame_length,
		imgsensor.line_length);

	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
//	write_cmos_sensor_8(0x0342, imgsensor.line_length >> 8);
//	write_cmos_sensor_8(0x0343, imgsensor.line_length & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
}	/*	set_dummy  */

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;


	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
		/* Extend frame length */
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}
	} else {
		/* Extend frame length */
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 i;

	for (i = 0; i < IMX586NEW2MIPI_MaxGainIndex; i++) {
		if(gain <= IMX586NEW2MIPI_sensorGainMapping[i][0]){
			break;
		}
	}
	if(gain != IMX586NEW2MIPI_sensorGainMapping[i][0])
		LOG_INF("Gain mapping don't correctly:%d %d \n", gain, IMX586NEW2MIPI_sensorGainMapping[i][0]);
	return IMX586NEW2MIPI_sensorGainMapping[i][1];
}


/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

    /* gain=1024;//for test */
    /* return; //for test */

	//if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
	if (gain < 72 || gain > 64 * BASEGAIN) {
	LOG_INF("Error gain setting");

	if (gain < 72)
		gain = 72;
	else if (gain > 64 * BASEGAIN)
		gain = 64 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
	write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	return gain;
}	/*	set_gain  */

#ifdef XUNHU_LPS_TEKHW_SUPPORT
static void set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 itemp;

	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp = read_cmos_sensor_8(0x0101);
	itemp &= ~0x03;

	switch (image_mirror) {

	case IMAGE_NORMAL:
	write_cmos_sensor_8(0x0101, itemp | 0x00);
	break;

	case IMAGE_V_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x02);
	break;

	case IMAGE_H_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x01);
	break;

	case IMAGE_HV_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x03);
	break;
	}
}
#endif
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif

#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3

#endif

static kal_uint16 imx586new2_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 3 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 3 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd,
								tosend,
								imgsensor.i2c_write_id,
								3,
								imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif
	}
	return 0;
}
#ifdef BENJA_IMX586NEW2_LRC
extern unsigned int read_imx586new2_LRC(BYTE *data);
static void write_imx586new2_LRC_Data(void)
{
	kal_uint16 i;	

	for (i = 0; i < 192; i++) {
		write_cmos_sensor(0x7510 + i, imx586new2_lrc_data[i]);
		/* LOG_INF("imx586new2_lrc_data[%d] = 0x%x\n", i, imx586new2_lrc_data[i]); */
	}
	
	for (i = 192; i < LRC_SIZE; i++) {
		write_cmos_sensor(0x7600 + i-192, imx586new2_lrc_data[i]);
		/* LOG_INF("imx586new2_lrc_data[%d] = 0x%x\n", i, imx586new2_lrc_data[i]); */
	}
}
#endif
static void write_imx586new2_QSC_Data(void)
{
	kal_uint16 i;	

	for (i = 0; i < DATA_SIZE; i++) {
		write_cmos_sensor(0x7F00 + i, imx586new2_qsc_data[i]);
		/* LOG_INF("imx586new2_qsc_data[%d] = 0x%x\n", i, imx586new2_qsc_data[i]); */
	}
}

kal_uint16 addr_data_pair_init_imx586new2[] = {
	0x0136, 0x18,
	0x0137, 0x00,
	0x3C7E, 0x07,
	0x3C7F, 0x08,
	0x0111, 0x02,
	0x380C, 0x00,
	0x3C00,	0x10,
	0x3C01,	0x10,
	0x3C02,	0x10,
	0x3C03,	0x10,
	0x3C04,	0x10,
	0x3C05,	0x01,
	0x3C06,	0x00,
	0x3C07,	0x00,
	0x3C08,	0x03,
	0x3C09,	0xFF,
	0x3C0A,	0x01,
	0x3C0B,	0x00,
	0x3C0C,	0x00,
	0x3C0D,	0x03,
	0x3C0E,	0xFF,
	0x3C0F,	0x20,
	0x3F88, 0x00,
	0x3F8E, 0x00,
	0x5282, 0x01,
	0x9004,	0x14,
	0x9200,	0xF4,
	0x9201,	0xA7,
	0x9202,	0xF4,
	0x9203,	0xAA,
	0x9204,	0xF4,
	0x9205,	0xAD,
	0x9206,	0xF4,
	0x9207,	0xB0,
	0x9208,	0xF4,
	0x9209,	0xB3,
	0x920A,	0xB7,
	0x920B,	0x34,
	0x920C,	0xB7,
	0x920D,	0x36,
	0x920E,	0xB7,
	0x920F,	0x37,
	0x9210,	0xB7,
	0x9211,	0x38,
	0x9212,	0xB7,
	0x9213,	0x39,
	0x9214,	0xB7,
	0x9215,	0x3A,
	0x9216,	0xB7,
	0x9217,	0x3C,
	0x9218,	0xB7,
	0x9219,	0x3D,
	0x921A,	0xB7,
	0x921B,	0x3E,
	0x921C,	0xB7,
	0x921D,	0x3F,
	0x921E,	0x77,
	0x921F,	0x77,
	0x9222,	0xC4,
	0x9223, 0x4B,
	0x9224, 0xC4,
	0x9225, 0x4C,
	0x9226, 0xC4,
	0x9227, 0x4D,

	0x9810,	0x14,
	0x9814,	0x14,
	0x99B2,	0x20,
	0x99B3,	0x0F,
	0x99B4,	0x0F,
	0x99B5,	0x0F,
	0x99B6,	0x0F,
	0x99E4,	0x0F,
	0x99E5,	0x0F,
	0x99E6,	0x0F,
	0x99E7,	0x0F,
	0x99E8,	0x0F,
	0x99E9,	0x0F,
	0x99EA,	0x0F,
	0x99EB, 0x0F,
	0x99EC, 0x0F,
	0x99ED, 0x0F,
	0xA569, 0x06,
	0xA679, 0x20,
	0xC020, 0x01,
	0xC61D, 0x00,
	0xC625, 0x00,
	0xC638, 0x03,
	0xC63B, 0x01,
	0xE286, 0x31,
	0xE2A6, 0x32,
	0xE2C6, 0x33,
	0x9852, 0x00,
	0x9954, 0x0F,
	0xA7AD, 0x01,
	0xA7CB, 0x01,
	0xAE09, 0xFF,
	0xAE0A, 0xFF,
	0xAE12, 0x58,
	0xAE13, 0x58,
	0xAE15, 0x10,
	0xAE16, 0x10,
	0xAF05, 0x48,
	0xB07C, 0x02,
	0xE186, 0x34,

};

kal_uint16 addr_data_pair_preview_imx586new2[] = {
	//2x2 Bining(4000x3000)
	//From reg tool at 2019-2-27 15:39 
	0x0112,0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x88,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222,0x01,
	0x0900,0x01,
	0x0901,0x22,
	0x0902,0x08,
	0x3140,0x00,
	0x3246,0x81,
	0x3247,0x81,
	0x3F15,0x00,	
	0x0401,0x00,
	0x0404,0x00,
	0x0405,0x10,
	0x0408,0x00,
	0x0409,0x00,
	0x040A,0x00,
	0x040B,0x00,
	0x040C,0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB4,
	0x030B, 0x01,
	0x030D, 0x03,
	0x030E, 0x00,
	0x030F, 0xA7,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x58,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x323A, 0x00,
	0x323B, 0x00,
	0x323C, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};

kal_uint16 addr_data_pair_lrc_imx586new2[] = {
 0x7510,0x6e,
 0x7511,0x70,
 0x7512,0x73,
 0x7513,0x74,
 0x7514,0x75,
 0x7515,0x76,
 0x7516,0x77,
 0x7517,0x77,
 0x7518,0x76,
 0x7519,0x75,
 0x751a,0x75,
 0x751b,0x74,
 0x751c,0x75,
 0x751d,0x77,
 0x751e,0x79,
 0x751f,0x7c,
 0x7520,0x6e,
 0x7521,0x71,
 0x7522,0x74,
 0x7523,0x75,
 0x7524,0x76,
 0x7525,0x77,
 0x7526,0x77,
 0x7527,0x77,
 0x7528,0x77,
 0x7529,0x75,
 0x752a,0x75,
 0x752b,0x74,
 0x752c,0x75,
 0x752d,0x76,
 0x752e,0x78,
 0x752f,0x7c,
 0x7530,0x6f,
 0x7531,0x73,
 0x7532,0x75,
 0x7533,0x76,
 0x7534,0x77,
 0x7535,0x78,
 0x7536,0x78,
 0x7537,0x78,
 0x7538,0x77,
 0x7539,0x76,
 0x753a,0x75,
 0x753b,0x75,
 0x753c,0x75,
 0x753d,0x76,
 0x753e,0x77,
 0x753f,0x7c,
 0x7540,0x71,
 0x7541,0x75,
 0x7542,0x76,
 0x7543,0x78,
 0x7544,0x79,
 0x7545,0x79,
 0x7546,0x79,
 0x7547,0x79,
 0x7548,0x78,
 0x7549,0x77,
 0x754a,0x76,
 0x754b,0x75,
 0x754c,0x76,
 0x754d,0x76,
 0x754e,0x77,
 0x754f,0x7b,
 0x7550,0x73,
 0x7551,0x76,
 0x7552,0x78,
 0x7553,0x79,
 0x7554,0x7a,
 0x7555,0x7a,
 0x7556,0x7b,
 0x7557,0x7a,
 0x7558,0x79,
 0x7559,0x78,
 0x755a,0x77,
 0x755b,0x76,
 0x755c,0x76,
 0x755d,0x76,
 0x755e,0x77,
 0x755f,0x7a,
 0x7560,0x73,
 0x7561,0x76,
 0x7562,0x78,
 0x7563,0x7a,
 0x7564,0x7a,
 0x7565,0x7b,
 0x7566,0x7b,
 0x7567,0x7b,
 0x7568,0x79,
 0x7569,0x78,
 0x756a,0x77,
 0x756b,0x76,
 0x756c,0x76,
 0x756d,0x76,
 0x756e,0x77,
 0x756f,0x7a,
 0x7570,0x74,
 0x7571,0x77,
 0x7572,0x79,
 0x7573,0x7a,
 0x7574,0x7b,
 0x7575,0x7b,
 0x7576,0x7c,
 0x7577,0x7b,
 0x7578,0x79,
 0x7579,0x78,
 0x757a,0x77,
 0x757b,0x76,
 0x757c,0x76,
 0x757d,0x76,
 0x757e,0x77,
 0x757f,0x7a,
 0x7580,0x75,
 0x7581,0x78,
 0x7582,0x79,
 0x7583,0x7b,
 0x7584,0x7c,
 0x7585,0x7c,
 0x7586,0x7c,
 0x7587,0x7c,
 0x7588,0x7a,
 0x7589,0x78,
 0x758a,0x77,
 0x758b,0x76,
 0x758c,0x76,
 0x758d,0x76,
 0x758e,0x77,
 0x758f,0x7a,
 0x7590,0x75,
 0x7591,0x79,
 0x7592,0x7a,
 0x7593,0x7c,
 0x7594,0x7d,
 0x7595,0x7d,
 0x7596,0x7d,
 0x7597,0x7c,
 0x7598,0x7b,
 0x7599,0x79,
 0x759a,0x78,
 0x759b,0x77,
 0x759c,0x77,
 0x759d,0x77,
 0x759e,0x78,
 0x759f,0x7c,
 0x75a0,0x75,
 0x75a1,0x79,
 0x75a2,0x7b,
 0x75a3,0x7c,
 0x75a4,0x7d,
 0x75a5,0x7d,
 0x75a6,0x7e,
 0x75a7,0x7d,
 0x75a8,0x7c,
 0x75a9,0x7a,
 0x75aa,0x79,
 0x75ab,0x78,
 0x75ac,0x78,
 0x75ad,0x78,
 0x75ae,0x7a,
 0x75af,0x7e,
 0x75b0,0x75,
 0x75b1,0x79,
 0x75b2,0x7b,
 0x75b3,0x7d,
 0x75b4,0x7d,
 0x75b5,0x7e,
 0x75b6,0x7e,
 0x75b7,0x7d,
 0x75b8,0x7c,
 0x75b9,0x7b,
 0x75ba,0x79,
 0x75bb,0x79,
 0x75bc,0x79,
 0x75bd,0x79,
 0x75be,0x7c,
 0x75bf,0x80,
 0x75c0,0x76,
 0x75c1,0x78,
 0x75c2,0x7b,
 0x75c3,0x7d,
 0x75c4,0x7e,
 0x75c5,0x7e,
 0x75c6,0x7f,
 0x75c7,0x7e,
 0x75c8,0x7d,
 0x75c9,0x7b,
 0x75ca,0x7a,
 0x75cb,0x79,
 0x75cc,0x7a,
 0x75cd,0x7b,
 0x75ce,0x7e,
 0x75cf,0x81,
 0x7600,0x78,
 0x7601,0x76,
 0x7602,0x74,
 0x7603,0x73,
 0x7604,0x74,
 0x7605,0x74,
 0x7606,0x75,
 0x7607,0x75,
 0x7608,0x76,
 0x7609,0x75,
 0x760a,0x74,
 0x760b,0x73,
 0x760c,0x71,
 0x760d,0x6f,
 0x760e,0x6d,
 0x760f,0x6a,
 0x7610,0x76,
 0x7611,0x73,
 0x7612,0x72,
 0x7613,0x72,
 0x7614,0x72,
 0x7615,0x73,
 0x7616,0x74,
 0x7617,0x75,
 0x7618,0x75,
 0x7619,0x75,
 0x761a,0x74,
 0x761b,0x73,
 0x761c,0x72,
 0x761d,0x70,
 0x761e,0x6d,
 0x761f,0x69,
 0x7620,0x74,
 0x7621,0x71,
 0x7622,0x70,
 0x7623,0x71,
 0x7624,0x71,
 0x7625,0x72,
 0x7626,0x73,
 0x7627,0x74,
 0x7628,0x75,
 0x7629,0x75,
 0x762a,0x74,
 0x762b,0x73,
 0x762c,0x71,
 0x762d,0x70,
 0x762e,0x6e,
 0x762f,0x69,
 0x7630,0x72,
 0x7631,0x6f,
 0x7632,0x6f,
 0x7633,0x6f,
 0x7634,0x70,
 0x7635,0x71,
 0x7636,0x72,
 0x7637,0x74,
 0x7638,0x74,
 0x7639,0x75,
 0x763a,0x74,
 0x763b,0x73,
 0x763c,0x72,
 0x763d,0x70,
 0x763e,0x6e,
 0x763f,0x6a,
 0x7640,0x70,
 0x7641,0x6e,
 0x7642,0x6e,
 0x7643,0x6e,
 0x7644,0x6f,
 0x7645,0x71,
 0x7646,0x72,
 0x7647,0x73,
 0x7648,0x75,
 0x7649,0x74,
 0x764a,0x74,
 0x764b,0x73,
 0x764c,0x72,
 0x764d,0x70,
 0x764e,0x6e,
 0x764f,0x6b,
 0x7650,0x6f,
 0x7651,0x6e,
 0x7652,0x6d,
 0x7653,0x6e,
 0x7654,0x6f,
 0x7655,0x71,
 0x7656,0x72,
 0x7657,0x73,
 0x7658,0x75,
 0x7659,0x75,
 0x765a,0x74,
 0x765b,0x74,
 0x765c,0x72,
 0x765d,0x71,
 0x765e,0x6e,
 0x765f,0x6b,
 0x7660,0x6e,
 0x7661,0x6d,
 0x7662,0x6c,
 0x7663,0x6d,
 0x7664,0x6e,
 0x7665,0x70,
 0x7666,0x72,
 0x7667,0x73,
 0x7668,0x75,
 0x7669,0x75,
 0x766a,0x74,
 0x766b,0x73,
 0x766c,0x72,
 0x766d,0x70,
 0x766e,0x6e,
 0x766f,0x6b,
 0x7670,0x6e,
 0x7671,0x6c,
 0x7672,0x6c,
 0x7673,0x6c,
 0x7674,0x6d,
 0x7675,0x70,
 0x7676,0x71,
 0x7677,0x72,
 0x7678,0x74,
 0x7679,0x74,
 0x767a,0x73,
 0x767b,0x73,
 0x767c,0x72,
 0x767d,0x70,
 0x767e,0x6e,
 0x767f,0x6b,
 0x7680,0x6e,
 0x7681,0x6c,
 0x7682,0x6b,
 0x7683,0x6b,
 0x7684,0x6d,
 0x7685,0x6e,
 0x7686,0x70,
 0x7687,0x71,
 0x7688,0x72,
 0x7689,0x73,
 0x768a,0x73,
 0x768b,0x72,
 0x768c,0x71,
 0x768d,0x6f,
 0x768e,0x6d,
 0x768f,0x6a,
 0x7690,0x6f,
 0x7691,0x6c,
 0x7692,0x6b,
 0x7693,0x6b,
 0x7694,0x6c,
 0x7695,0x6d,
 0x7696,0x6f,
 0x7697,0x70,
 0x7698,0x72,
 0x7699,0x72,
 0x769a,0x72,
 0x769b,0x71,
 0x769c,0x70,
 0x769d,0x6f,
 0x769e,0x6d,
 0x769f,0x69,
 0x76a0,0x70,
 0x76a1,0x6d,
 0x76a2,0x6b,
 0x76a3,0x6b,
 0x76a4,0x6c,
 0x76a5,0x6d,
 0x76a6,0x6e,
 0x76a7,0x70,
 0x76a8,0x71,
 0x76a9,0x72,
 0x76aa,0x71,
 0x76ab,0x70,
 0x76ac,0x70,
 0x76ad,0x6f,
 0x76ae,0x6c,
 0x76af,0x69,
 0x76b0,0x6f,
 0x76b1,0x6e,
 0x76b2,0x6c,
 0x76b3,0x6b,
 0x76b4,0x6c,
 0x76b5,0x6d,
 0x76b6,0x6e,
 0x76b7,0x6f,
 0x76b8,0x71,
 0x76b9,0x71,
 0x76ba,0x71,
 0x76bb,0x71,
 0x76bc,0x70,
 0x76bd,0x6e,
 0x76be,0x6c,
 0x76bf,0x69,
};

#if 0
kal_uint16 addr_data_pair_custom1_imx586new2[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x88,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB4,
	0x030B, 0x01,
	0x030D, 0x03,
	0x030E, 0x00,
	0x030F, 0xA7,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x58,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x323A, 0x00,
	0x323B, 0x00,
	0x323C, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,

};
#endif

kal_uint16 addr_data_pair_custom2_imx586new2[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x88,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB4,
	0x030B, 0x01,
	0x030D, 0x03,
	0x030E, 0x00,
	0x030F, 0xA7,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x58,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x323A, 0x00,
	0x323B, 0x00,
	0x323C, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,

};

kal_uint16 addr_data_pair_custom3_imx586new2[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x15,
	0x0343, 0x00,
	0x0340, 0x0A,
	0x0341, 0x76,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x07,
	0x040D, 0xD0,
	0x040E, 0x05,
	0x040F, 0xDC,
	0x034C, 0x07,
	0x034D, 0xD0,
	0x034E, 0x05,
	0x034F, 0xDC,
	0x0301, 0x05,
	0x0303, 0x04,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB4,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xB7,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x0C,
	0x3C12, 0x05,
	0x3C13, 0x2C,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x02,
	0x3F81, 0x67,
	0x3F8C, 0x02,
	0x3F8D, 0x44,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x01,
	0x3FFF, 0x90,
	0x0202, 0x0A,
	0x0203, 0x46,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x323A, 0x00,
	0x323B, 0x00,
	0x323C, 0x00,
	0x3E20, 0x01,
	0x3E3B, 0x00,
	0x4434, 0x00,
	0x4435, 0xF8,

};


kal_uint16 addr_data_pair_hs_video_imx586new2[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x15,
	0x0343, 0x00,
	0x0340, 0x03,
	0x0341, 0x56,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x06,
	0x0347, 0x18,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x11,
	0x034B, 0x57,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x01,
	0x0409, 0x68,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x05,
	0x040D, 0x00,
	0x040E, 0x02,
	0x040F, 0xD0,
	0x034C, 0x05,
	0x034D, 0x00,
	0x034E, 0x02,
	0x034F, 0xD0,
	0x0301, 0x05,
	0x0303, 0x04,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xE6,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xB4,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x0C,
	0x3C12, 0x05,
	0x3C13, 0x2C,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x02,
	0x3F81, 0x67,
	0x3F8C, 0x02,
	0x3F8D, 0x44,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x01,
	0x3FFF, 0x90,
	0x0202, 0x03,
	0x0203, 0x26,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x323A, 0x00,
	0x323B, 0x00,
	0x323C, 0x00,
	0x3E20, 0x01,
	0x3E3B, 0x00,
	0x4434, 0x00,
	0x4435, 0xF8,
};

static void sensor_init(void)
{
	LOG_INF("E init\n");
	imx586new2_table_write_cmos_sensor(addr_data_pair_init_imx586new2,
		sizeof(addr_data_pair_init_imx586new2) / sizeof(kal_uint16));
	LOG_INF("L\n");
} /*	sensor_init  */

static void preview_setting(void)
{
	LOG_INF("E binning_normal_setting\n");

#if 1	
	write_cmos_sensor(0x0100,0x00); //standby
#if 1//def BENJA_IMX586NEW2_LRC
	//imx586new2_table_write_cmos_sensor(addr_data_pair_lrc_imx586new2,
      //   sizeof(addr_data_pair_lrc_imx586new2) / sizeof(kal_uint16));  
#endif

	imx586new2_table_write_cmos_sensor(addr_data_pair_preview_imx586new2,
         sizeof(addr_data_pair_preview_imx586new2) / sizeof(kal_uint16));  
/*
	write_cmos_sensor(0x38a3,0x02); //0:16x12; 1:8x6; 2:Flex
	write_cmos_sensor(0x3e37,0x01); //0:PD stop; 1:PD out
	write_cmos_sensor(0x38ac,0x01); //1:Flex win0 En; 0:Dis

	write_cmos_sensor(0x38b4,0x05); //Win0 Xtart H
	write_cmos_sensor(0x38B5,0x36); //Win0 Xtart L
	write_cmos_sensor(0x38B6,0x03); //Win0 Ytart H
	write_cmos_sensor(0x38B7,0xE8); //Win0 Ytart L
	write_cmos_sensor(0x38B8,0x0A); //Win0 Xend H
	write_cmos_sensor(0x38B9,0x6B); //Win0 Xend L
	write_cmos_sensor(0x38BA,0x07); //Win0 Yend H
	write_cmos_sensor(0x38BB,0xD0); //Win0 Yend L */

	write_cmos_sensor(0x0100,0x01); //steaming
	
#else
	write_cmos_sensor(0x0100,0x00); //standy
	
	imx586new2_table_write_cmos_sensor(addr_data_pair_preview_imx586new2,
		sizeof(addr_data_pair_preview_imx586new2) / sizeof(kal_uint16));	
	
	write_cmos_sensor(0x3e37,0x01); //0:PD stop; 1:PD out
	//write_cmos_sensor(0x0100,0x00); //stream ON
	write_cmos_sensor(0x38a3,0x02); //0:16x12; 1:8x6; 2:Flex
	write_cmos_sensor(0x38ac,0x01); //1:Flex win0 En; 0:Dis
	write_cmos_sensor(0x38ad,0x01); //1:Flex win1 En; 0:Dis
	write_cmos_sensor(0x38ae,0x01); //1:Flex win2 En; 0:Dis
	write_cmos_sensor(0x38af,0x01); //1:Flex win3 En; 0:Dis
	write_cmos_sensor(0x38b0,0x01); //1:Flex win4 En; 0:Dis
	write_cmos_sensor(0x38b1,0x01); //1:Flex win5 En; 0:Dis
	write_cmos_sensor(0x38b2,0x01); //1:Flex win6 En; 0:Dis
	write_cmos_sensor(0x38b3,0x01); //1:Flex win7 En; 0:Dis
	write_cmos_sensor(0x38b4,0x05); //Win0 Xtart H
	write_cmos_sensor(0x38B5,0xF0); //Win0 Xtart L
	write_cmos_sensor(0x38B6,0x03); //Win0 Ytart H
	write_cmos_sensor(0x38B7,0xFC); //Win0 Ytart L
	write_cmos_sensor(0x38B8,0x09); //Win0 Xend H
	write_cmos_sensor(0x38B9,0xB0); //Win0 Xend L
	write_cmos_sensor(0x38BA,0x07); //Win0 Yend H
	write_cmos_sensor(0x38BB,0xBC); //Win0 Yend L
	write_cmos_sensor(0x38BC,0x00); //Win1 Xtart H
	write_cmos_sensor(0x38BD,0x11); //Win1 Xtart L
	write_cmos_sensor(0x38BE,0x00); //Win1 Ytart H
	write_cmos_sensor(0x38BF,0x0C); //Win1 Ytart L
	write_cmos_sensor(0x38C0,0x05); //Win1 Xend H
	write_cmos_sensor(0x38C1,0xF0); //Win1 Xend L
	write_cmos_sensor(0x38C2,0x05); //Win1 Yend H
	write_cmos_sensor(0x38C3,0xDC); //Win1 Yend L
	write_cmos_sensor(0x38C4,0x05); //Win2 Xtart H
	write_cmos_sensor(0x38C5,0xF0); //Win2 Xtart L
	write_cmos_sensor(0x38C6,0x00); //Win2 Ytart H
	write_cmos_sensor(0x38C7,0x0C); //Win2 Ytart L
	write_cmos_sensor(0x38C8,0x09); //Win2 Xend H
	write_cmos_sensor(0x38C9,0xB0); //Win2 Xend L
	write_cmos_sensor(0x38CA,0x03); //Win2 Yend H
	write_cmos_sensor(0x38CB,0xFC); //Win2 Yend L
	write_cmos_sensor(0x38CC,0x09); //Win3 Xtart H
	write_cmos_sensor(0x38CD,0xB0); //Win3 Xtart L
	write_cmos_sensor(0x38CE,0x00); //Win3 Ytart H
	write_cmos_sensor(0x38CF,0x0C); //Win3 Ytart L
	write_cmos_sensor(0x38D0,0x0F); //Win3 Xend H
	write_cmos_sensor(0x38D1,0x8E); //Win3 Xend L
	write_cmos_sensor(0x38D2,0x05); //Win3 Yend H
	write_cmos_sensor(0x38D3,0xDC); //Win3 Yend L
	write_cmos_sensor(0x38D4,0x00); //Win4 Yend H
	write_cmos_sensor(0x38D5,0x11); //Win4 Yend L
	write_cmos_sensor(0x38D6,0x05); //Win4 Ytart H
	write_cmos_sensor(0x38D7,0xDC); //Win4 Ytart L
	write_cmos_sensor(0x38D8,0x05); //Win4 Xend H
	write_cmos_sensor(0x38D9,0xF0); //Win4 Xend L
	write_cmos_sensor(0x38DA,0x0B); //Win4 Yend H
	write_cmos_sensor(0x38DB,0xAB); //Win4 Yend L
	write_cmos_sensor(0x38DC,0x05); //Win5 Yend H
	write_cmos_sensor(0x38DD,0xF0); //Win5 Yend L
	write_cmos_sensor(0x38DE,0x07); //Win5 Ytart H
	write_cmos_sensor(0x38DF,0xBC); //Win5 Ytart L
	write_cmos_sensor(0x38E0,0x09); //Win5 Xend H
	write_cmos_sensor(0x38E1,0xB0); //Win5 Xend L
	write_cmos_sensor(0x38E2,0x0B); //Win5 Yend H
	write_cmos_sensor(0x38E3,0xAB); //Win5 Yend L
	write_cmos_sensor(0x38E4,0x09); //Win6 Yend H
	write_cmos_sensor(0x38E5,0xB0); //Win6 Yend L
	write_cmos_sensor(0x38E6,0x05); //Win6 Ytart H
	write_cmos_sensor(0x38E7,0xDC); //Win6 Ytart L
	write_cmos_sensor(0x38E8,0x0F); //Win6 Xend H
	write_cmos_sensor(0x38E9,0x8E); //Win6 Xend L
	write_cmos_sensor(0x38EA,0x0B); //Win6 Yend H
	write_cmos_sensor(0x38EB,0xAB); //Win6 Yend L
	
	write_cmos_sensor(0x0100,0x01);	//stream ON

#endif
	
	LOG_INF("L\n");
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps, kal_bool stream_on)
{
	LOG_INF("E currefps:%d\n", currefps);
	preview_setting();
	LOG_INF("L!\n");
}

static void normal_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
	imx586new2_table_write_cmos_sensor(addr_data_pair_hs_video_imx586new2,
		sizeof(addr_data_pair_hs_video_imx586new2) / sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
}

static void custom1_setting(void)
{
	/* custom1 32M setting */
	LOG_INF("E\n");
#if 0
	imx586new2_table_write_cmos_sensor(addr_data_pair_custom1_imx586new2,
		sizeof(addr_data_pair_custom1_imx586new2) / sizeof(kal_uint16));
#else
	preview_setting();
#endif
}

static void custom2_setting(void)
{
	/* custom2 48M@15fps setting */
	LOG_INF("E\n");
	imx586new2_table_write_cmos_sensor(addr_data_pair_custom2_imx586new2,
		sizeof(addr_data_pair_custom2_imx586new2) / sizeof(kal_uint16));
}

static void custom3_setting(void)
{
	/* custom3 stero@34fps setting */
	LOG_INF("E\n");
	imx586new2_table_write_cmos_sensor(addr_data_pair_custom3_imx586new2,
		sizeof(addr_data_pair_custom3_imx586new2) / sizeof(kal_uint16));
}

#define EEPROM_READ_ID    	(0xB0)
static kal_uint16 read_eeprom_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,EEPROM_READ_ID);
    return get_byte;
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint16 moduleID=0;
    /*sensor have two i2c address 0x34 & 0x20,
     *we should detect the module used i2c address
     */
    moduleID = read_eeprom_8(0x01);
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = ((read_cmos_sensor_8(0x0016) << 8) | read_cmos_sensor_8(0x0017)) + 2;
            /*LOG_INF("[getid imx586new2]read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",
                read_cmos_sensor_8(0x0016),
                read_cmos_sensor_8(0x0017),
                read_cmos_sensor(0x0000));*/
            if (moduleID == 0x04) {
	            if (*sensor_id == imgsensor_info.sensor_id) {
	                LOG_INF("[getid imx586new2]i2c write id: 0x%x, sensor id: 0x%x,moduleID: 0x%x\n",imgsensor.i2c_write_id, *sensor_id,moduleID);
	                     return ERROR_NONE;
				    }
            }
            LOG_INF("[getid imx586new2]Read sensor id fail, addr id: 0x%x, sensor id: 0x%x,moduleID: 0x%x\n",imgsensor.i2c_write_id, *sensor_id,moduleID);
            retry--;
        } while (retry > 0);
        i++;
        retry = 2;
    }
	if ((*sensor_id != imgsensor_info.sensor_id)  || (moduleID != 0x04)){
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
			*sensor_id = 0xFFFFFFFF;
			return ERROR_SENSOR_CONNECT_FAIL;
	  } 

	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

extern void AFRegulatorCtrl(int Stage);
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
	kal_uint16 moduleID=0;
    //kal_uint16 j = 0;
	/* sensor have two i2c address 0x35 0x34 & 0x21 0x20, we should detect the module used i2c address */
	moduleID = read_eeprom_8(0x01);
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = ((read_cmos_sensor_8(0x0016) << 8) | read_cmos_sensor_8(0x0017)) + 2;
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("[open imx586new2]i2c write id: 0x%x, sensor id: 0x%x,moduleID: 0x%x\n",
                    imgsensor.i2c_write_id, sensor_id,moduleID);
	                break;
            }
            LOG_INF("[open imx586new2]Read sensor id fail, id: 0x%x,moduleID: 0x%x\n",
                imgsensor.i2c_write_id,moduleID);
            retry--;
        } while (retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
                break;
        retry = 2;
    }
    
	 //~ for (j=0;j<1000;j++)
    //~ {
     //~ printk("otp addr0x%X: 0x%x,   otp  value: 0x%x\n", EEPROM_READ_ID,j,read_eeprom_8(j));
    //~ }
    
	if ((imgsensor_info.sensor_id != sensor_id) || (moduleID != 0x04))
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail sequence write in  */
	AFRegulatorCtrl(1);

	sensor_init();
	imx586new2_table_write_cmos_sensor(addr_data_pair_lrc_imx586new2,sizeof(addr_data_pair_lrc_imx586new2) / sizeof(kal_uint16));  
#ifdef BENJA_IMX586NEW2_LRC	
	read_imx586new2_LRC(imx586new2_lrc_data);
	LOG_INF("[%s][%d]imx586new2_lrc_data[0]:0x%x   imx586new2_lrc_data[383]:0x%x\n", __func__, __LINE__,imx586new2_lrc_data[0],imx586new2_lrc_data[383]);
	write_imx586new2_LRC_Data();
#endif	
	//IMX586NEW2_MIPI_update_awb(imgsensor.i2c_write_id);

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");
	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E imgsensor.hdr_mode=%d\n", imgsensor.hdr_mode);

	LOG_INF("E preview normal\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
#ifdef XUNHU_LPS_TEKHW_SUPPORT
    if (0xff != tekhw_imx586new2_mirror)
    {
	    set_mirror_flip(tekhw_imx586new2_mirror);
    }
#else
	set_mirror_flip(imgsensor.mirror);
#endif

	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps, 1);
#ifdef XUNHU_LPS_TEKHW_SUPPORT
    if (0xff != tekhw_imx586new2_mirror)
    {
	    set_mirror_flip(tekhw_imx586new2_mirror);
    }
#else
	set_mirror_flip(imgsensor.mirror);
#endif

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

		LOG_INF("E preview normal\n");
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
		imgsensor.pclk = imgsensor_info.normal_video.pclk;
		imgsensor.line_length = imgsensor_info.normal_video.linelength;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength;
		imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);
		normal_video_setting();
#ifdef XUNHU_LPS_TEKHW_SUPPORT
    if (0xff != tekhw_imx586new2_mirror)
    {
	    set_mirror_flip(tekhw_imx586new2_mirror);
    }
#else
	set_mirror_flip(imgsensor.mirror);
#endif

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
#ifdef XUNHU_LPS_TEKHW_SUPPORT
    if (0xff != tekhw_imx586new2_mirror)
    {
	    set_mirror_flip(tekhw_imx586new2_mirror);
    }
#else
	set_mirror_flip(imgsensor.mirror);
#endif

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
#ifdef XUNHU_LPS_TEKHW_SUPPORT
    if (0xff != tekhw_imx586new2_mirror)
    {
	    set_mirror_flip(tekhw_imx586new2_mirror);
    }
#else
	set_mirror_flip(imgsensor.mirror);
#endif

	return ERROR_NONE;
}	/*	slim_video	 */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	if (imgsensor.current_fps == imgsensor_info.custom1.max_framerate) {
		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom1.linelength;
		imgsensor.frame_length = imgsensor_info.custom1.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();
#ifdef XUNHU_LPS_TEKHW_SUPPORT
    if (0xff != tekhw_imx586new2_mirror)
    {
	    set_mirror_flip(tekhw_imx586new2_mirror);
    }
#else
	set_mirror_flip(imgsensor.mirror);
#endif

	return ERROR_NONE;
}

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	if (imgsensor.current_fps == imgsensor_info.custom2.max_framerate) {
		imgsensor.pclk = imgsensor_info.custom2.pclk;
		imgsensor.line_length = imgsensor_info.custom2.linelength;
		imgsensor.frame_length = imgsensor_info.custom2.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	
	write_imx586new2_QSC_Data();
	
	custom2_setting();
#ifdef XUNHU_LPS_TEKHW_SUPPORT
    if (0xff != tekhw_imx586new2_mirror)
    {
	    set_mirror_flip(tekhw_imx586new2_mirror);
    }
#else
	set_mirror_flip(imgsensor.mirror);
#endif

	return ERROR_NONE;
}

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	if (imgsensor.current_fps == imgsensor_info.custom3.max_framerate) {
		imgsensor.pclk = imgsensor_info.custom3.pclk;
		imgsensor.line_length = imgsensor_info.custom3.linelength;
		imgsensor.frame_length = imgsensor_info.custom3.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	custom3_setting();
#ifdef XUNHU_LPS_TEKHW_SUPPORT
    if (0xff != tekhw_imx586new2_mirror)
    {
	    set_mirror_flip(tekhw_imx586new2_mirror);
    }
#else
	set_mirror_flip(imgsensor.mirror);
#endif

	return ERROR_NONE;
}


#ifdef XUNHU_LPS_TEKHW_SUPPORT    
static void   xun_hardware_getparams(void) 
{
    tekhw_camera_data* tekhw_imx586new2_ptr = tekhw_get_appointed_camera_ptr(SENSOR_DRVNAME_IMX586NEW2_MIPI_RAW);/*add by xunhu andy andy20160720 at 22:56*/
    if ((tekhw_imx586new2_ptr != NULL) && (tekhw_imx586new2_ptr->set_mirror.size > 0))
    {
        tekhw_imx586new2_mirror = *(tekhw_u16*)tekhw_imx586new2_ptr->set_mirror.buffer;
    }
    if ((tekhw_imx586new2_ptr != NULL) && (tekhw_imx586new2_ptr->set_color.size > 0))
    {
		tekhw_imx586new2_color= *(tekhw_u16*)tekhw_imx586new2_ptr->set_color.buffer;
    }
}
#endif

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	
	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;
	sensor_resolution->SensorCustom2Width = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.custom2.grabwindow_height;
	sensor_resolution->SensorCustom3Width = imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height = imgsensor_info.custom3.grabwindow_height;


	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

#ifdef XUNHU_LPS_TEKHW_SUPPORT    
	if(tekhw_times == 0)
		xun_hardware_getparams() ;
#endif

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;
#if defined(XUNHU_LPS_TEKHW_SUPPORT)
	if(tekhw_imx586new2_color!=0xff)
	{
		sensor_info->SensorOutputDataFormat = tekhw_imx586new2_color;
	}
	tekhw_times =1;
#endif

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 2;
	sensor_info->HDR_Support = 0;	/*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR, 4:four-cell mVHDR*/

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

		switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
				
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
				
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
			break;

		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		custom3(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d, hdr_mode = %d\n", scenario_id, framerate, imgsensor.hdr_mode);

		switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:

				frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
				(frame_length - imgsensor_info.pre.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if (framerate == 0)
				return ERROR_NONE;
			if (imgsensor.hdr_mode) {
				frame_length =
				imgsensor_info.pre_3HDR.pclk / framerate * 10 / imgsensor_info.pre_3HDR.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.pre_3HDR.framelength) ?
				(frame_length - imgsensor_info.pre_3HDR.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.pre_3HDR.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			} else {
				frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
					imgsensor_info.normal_video.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
					(frame_length - imgsensor_info.normal_video.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			}
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
				frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
				spin_lock(&imgsensor_drv_lock);
					imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ?
						(frame_length - imgsensor_info.cap1.framelength) : 0;
					imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
					imgsensor.min_frame_length = imgsensor.frame_length;
					spin_unlock(&imgsensor_drv_lock);
			} else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
			frame_length = imgsensor_info.cap2.pclk / framerate * 10 / imgsensor_info.cap2.linelength;
			spin_lock(&imgsensor_drv_lock);
					imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength) ?
						(frame_length - imgsensor_info.cap2.framelength) : 0;
					imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
					imgsensor.min_frame_length = imgsensor.frame_length;
					spin_unlock(&imgsensor_drv_lock);
			} else {
				if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
					LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate/10);
					frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
					spin_lock(&imgsensor_drv_lock);
					imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
						(frame_length - imgsensor_info.cap.framelength) : 0;
					imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
					imgsensor.min_frame_length = imgsensor.frame_length;
					spin_unlock(&imgsensor_drv_lock);
			}
			set_dummy();
	    	break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 /
				imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
				(frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 /
				imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
				(frame_length - imgsensor_info.slim_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;			
		case MSDK_SCENARIO_ID_CUSTOM1:
			frame_length = imgsensor_info.custom1.pclk / framerate * 10 / 
				imgsensor_info.custom1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? 
				(frame_length - imgsensor_info.custom1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			frame_length = imgsensor_info.custom2.pclk / framerate * 10 / 
				imgsensor_info.custom2.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? 
				(frame_length - imgsensor_info.custom2.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			frame_length = imgsensor_info.custom3.pclk / framerate * 10 / 
				imgsensor_info.custom3.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? 
				(frame_length - imgsensor_info.custom3.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		
		default:  /* coding with  preview scenario by default */
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
				(frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		write_cmos_sensor(0x0601, 0x0002);
	} else {
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		write_cmos_sensor(0x0601, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static void hdr_write_tri_shutter(kal_uint16 le, kal_uint16 me, kal_uint16 se)
{
	kal_uint16 realtime_fps = 0;

	LOG_INF("E! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);
	spin_lock(&imgsensor_drv_lock);
	if (le > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = le + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (le < imgsensor_info.min_shutter)
		le = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8); /*FRM_LENGTH_LINES[15:8]*/
			write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF); /*FRM_LENGTH_LINES[7:0]*/
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}

	write_cmos_sensor_8(0x0104, 0x01);
	/* Long exposure */
	write_cmos_sensor_8(0x0202, (le >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, le & 0xFF);
	/* Muddle exposure */
	write_cmos_sensor_8(0x3FE0, (me >> 8) & 0xFF); /*MID_COARSE_INTEG_TIME[15:8]*/
	write_cmos_sensor_8(0x3FE1, me & 0xFF); /*MID_COARSE_INTEG_TIME[7:0]*/
	/* Short exposure */
	write_cmos_sensor_8(0x0224, (se >> 8) & 0xFF);
	write_cmos_sensor_8(0x0225, se & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	LOG_INF("L! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);

}

static void hdr_write_tri_gain(kal_uint16 lg, kal_uint16 mg, kal_uint16 sg)
{
	kal_uint16 reg_lg, reg_mg, reg_sg;

	if (lg < BASEGAIN || lg > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (lg < BASEGAIN)
			lg = BASEGAIN;
		else if (lg > 16 * BASEGAIN)
			lg = 16 * BASEGAIN;
	}

	reg_lg = gain2reg(lg);
	reg_mg = gain2reg(mg);
	reg_sg = gain2reg(sg);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_lg;
	spin_unlock(&imgsensor_drv_lock);
	write_cmos_sensor_8(0x0104, 0x01);
	/* Long Gian */
	write_cmos_sensor_8(0x0204, (reg_lg>>8) & 0xFF);
	write_cmos_sensor_8(0x0205, reg_lg & 0xFF);
	/* Middle Gian */
	write_cmos_sensor_8(0x3FE2, (reg_mg>>8) & 0xFF);
	write_cmos_sensor_8(0x3FE3, reg_mg & 0xFF);
	/* Short Gian */
	write_cmos_sensor_8(0x0216, (reg_sg>>8) & 0xFF);
	write_cmos_sensor_8(0x0217, reg_sg & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	if (lg > mg) {
		LOG_INF("long gain > medium gain\n");
		write_cmos_sensor_8(0xEB06, 0x00);
		write_cmos_sensor_8(0xEB08, 0x00);
		write_cmos_sensor_8(0xEB0A, 0x00);
		write_cmos_sensor_8(0xEB12, 0x00);
		write_cmos_sensor_8(0xEB14, 0x00);
		write_cmos_sensor_8(0xEB16, 0x00);

		write_cmos_sensor_8(0xEB07, 0x08);
		write_cmos_sensor_8(0xEB09, 0x08);
		write_cmos_sensor_8(0xEB0B, 0x08);
		write_cmos_sensor_8(0xEB13, 0x10);
		write_cmos_sensor_8(0xEB15, 0x10);
		write_cmos_sensor_8(0xEB17, 0x10);
	} else {
		LOG_INF("long gain <= medium gain\n");
		write_cmos_sensor_8(0xEB06, 0x00);
		write_cmos_sensor_8(0xEB08, 0x00);
		write_cmos_sensor_8(0xEB0A, 0x00);
		write_cmos_sensor_8(0xEB12, 0x01);
		write_cmos_sensor_8(0xEB14, 0x01);
		write_cmos_sensor_8(0xEB16, 0x01);

		write_cmos_sensor_8(0xEB07, 0xC8);
		write_cmos_sensor_8(0xEB09, 0xC8);
		write_cmos_sensor_8(0xEB0B, 0xC8);
		write_cmos_sensor_8(0xEB13, 0x2C);
		write_cmos_sensor_8(0xEB15, 0x2C);
		write_cmos_sensor_8(0xEB17, 0x2C);
	}

	LOG_INF("lg:0x%x, mg:0x%x, sg:0x%x, reg_lg:0x%x, reg_mg:0x%x, reg_sg:0x%x\n",
			lg, mg, sg, reg_lg, reg_mg, reg_sg);

}

static void imx586new2_set_lsc_reg_setting(kal_uint8 index, kal_uint16 *regDa, MUINT32 regNum)
{



}

static void set_imx586new2_ATR(kal_uint16 LimitGain, kal_uint16 LtcRate, kal_uint16 PostGain)
{


}

static kal_uint32 imx586new2_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{


	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor_8(0x0100, 0x01);
	else
		write_cmos_sensor_8(0x0100, 0x00);

	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB = (struct SET_SENSOR_AWB_GAIN *) feature_para;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
				imgsensor.pclk, imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			 /* night_mode((BOOL) *feature_data); */
			break;
		case SENSOR_FEATURE_SET_GAIN:
			set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
			/* if EEPROM does not exist in camera module. */
			*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			/* read_3P3_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),
			 *(kal_uint32)(*(feature_data+2)));
			 */
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /* for factory mode auto testing */
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
			LOG_INF("current fps :%d\n", *feature_data_32);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = (UINT16)*feature_data_32;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
			/* LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data); */
			wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
			switch (*feature_data_32) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
					sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
					sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
					sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
					sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5],
					sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[6],
					sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_CUSTOM3:
				memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[7],
					sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
					sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			}
			break;
		/*HDR CMD */
			case SENSOR_FEATURE_GET_PDAF_INFO:
				LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%llu\n", *feature_data);
				PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
			
				switch (*feature_data) {
					case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					case MSDK_SCENARIO_ID_SLIM_VIDEO:
					case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
						memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
						break;
					default:
						break;
				}
				break;

		case SENSOR_FEATURE_SET_HDR_ATR:
			LOG_INF("SENSOR_FEATURE_SET_HDR_ATR Limit_Gain=%d, LTC Rate=%d, Post_Gain=%d\n",
					(UINT16)*feature_data,
					(UINT16)*(feature_data + 1),
					(UINT16)*(feature_data + 2));
			set_imx586new2_ATR((UINT16)*feature_data,
						(UINT16)*(feature_data + 1),
						(UINT16)*(feature_data + 2));
			break;
		case SENSOR_FEATURE_SET_HDR:
			LOG_INF("hdr enable :%d\n", *feature_data_32);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.hdr_mode = (UINT8)*feature_data_32;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR_SHUTTER:
			LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d, no support\n",
				(UINT16) *feature_data,	(UINT16) *(feature_data + 1));
			/*hdr_write_shutter((UINT16) *feature_data, (UINT16) *(feature_data + 1),
			*	(UINT16) *(feature_data + 2));
			*/
			break;
		case SENSOR_FEATURE_SET_HDR_TRI_SHUTTER:
			LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_SHUTTER LE=%d, ME=%d, SE=%d\n",
					(UINT16) *feature_data,
					(UINT16) *(feature_data + 1),
					(UINT16) *(feature_data + 2));
			hdr_write_tri_shutter((UINT16)*feature_data,
								(UINT16)*(feature_data+1),
								(UINT16)*(feature_data+2));
			break;
		case SENSOR_FEATURE_SET_HDR_TRI_GAIN:
			LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_GAIN LGain=%d, SGain=%d, MGain=%d\n",
					(UINT16) *feature_data,
					(UINT16) *(feature_data + 1),
					(UINT16) *(feature_data + 2));
			hdr_write_tri_gain((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
			break;
		case SENSOR_FEATURE_GET_VC_INFO:
            LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", 
			(UINT16)*feature_data);
            pvcinfo = 
			(struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch (*feature_data_32) {
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
                break;
            }
            break;
		case SENSOR_FEATURE_SET_AWB_GAIN:
			imx586new2_awb_gain(pSetSensorAWB);
			break;
		case SENSOR_FEATURE_SET_LSC_TBL:
			{
				kal_uint8 index = *(((kal_uint8 *)feature_para) + (*feature_para_len));

				imx586new2_set_lsc_reg_setting(index, feature_data_16, (*feature_para_len)/sizeof(UINT16));
			}
			break;
		case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
			/*
			  * SENSOR_VHDR_MODE_NONE  = 0x0,
			  * SENSOR_VHDR_MODE_IVHDR = 0x01,
			  * SENSOR_VHDR_MODE_MVHDR = 0x02,
			  * SENSOR_VHDR_MODE_ZVHDR = 0x09
			  * SENSOR_VHDR_MODE_4CELL_MVHDR = 0x0A
			*/
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x2;
				break;
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CUSTOM1:
			case MSDK_SCENARIO_ID_CUSTOM2:
			case MSDK_SCENARIO_ID_CUSTOM3:
			default:
				*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
				break;
			}
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n"
				, *feature_data, *(feature_data+1));
			break;
			/*END OF HDR CMD */
		case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				rate = imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;			
			case MSDK_SCENARIO_ID_CUSTOM1:
				rate = imgsensor_info.custom2.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				rate = imgsensor_info.custom2.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CUSTOM3:
				rate = imgsensor_info.custom3.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
		
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			pr_debug(
			"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
				(UINT16) *feature_data);
			/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CUSTOM1:
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			}
			break;

		case SENSOR_FEATURE_SET_PDAF:
			LOG_INF("PDAF mode :%d\n", *feature_data_16);
			imgsensor.pdaf_mode= *feature_data_16;
			break;
		case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
			LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
			streaming_control(KAL_FALSE);
			break;
		case SENSOR_FEATURE_SET_STREAMING_RESUME:
			LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
			if (*feature_data != 0)
				set_shutter(*feature_data);
			streaming_control(KAL_TRUE);
			break;
		default:
			break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX586NEW2_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{

	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	
