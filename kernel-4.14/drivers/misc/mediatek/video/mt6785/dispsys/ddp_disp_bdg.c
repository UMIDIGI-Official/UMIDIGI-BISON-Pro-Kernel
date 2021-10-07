/*
 * Copyright (C) 2015 MediaTek Inc.
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


#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/string.h>
#include "ddp_disp_bdg.h"
#include "ddp_reg_disp_bdg.h"
#include "disp_drv_log.h"
#include "ddp_reg.h"
#include "mt6382.h"
#include "disp_dts_gpio.h"
#include "../../../base/power/include/clkbuf_v1/mt6785/mtk_clkbuf_hw.h"
#include <linux/of.h>
#include <linux/of_irq.h>
/***** NFC SRCLKENAI0 Interrupt Handler +++ *****/
#include <linux/gpio.h>
#include <linux/of_gpio.h>
/***** NFC SRCLKENAI0 Interrupt Handler --- *****/
#include "cmdq-bdg.h"
//#include "ddp_log.h"
//#include <linux/math.h>

#define SPI_EN
struct BDG_SYSREG_CTRL_REGS *SYS_REG;		/* 0x00000000 */
struct BDG_TOPCKGEN_REGS *TOPCKGEN;		/* 0x00003000 */
struct BDG_APMIXEDSYS_REGS *APMIXEDSYS;		/* 0x00004000 */
struct BDG_GPIO_REGS *GPIO;			/* 0x00007000 */
struct BDG_EFUSE_REGS *EFUSE;			/* 0x00009000 */
struct BDG_MIPIDSI2_REGS *DSI2_REG;		/* 0x0000d000 */
struct BDG_GCE_REGS *GCE_REG;			/* 0x00010000 */
struct BDG_OCLA_REGS *OCLA_REG;			/* 0x00014000 */
struct BDG_DISP_DSC_REGS *DSC_REG;		/* 0x00020000 */
struct BDG_TX_REGS *TX_REG[HW_NUM];		/* 0x00021000 */
struct DSI_TX_CMDQ_REGS *TX_CMDQ_REG[HW_NUM];	/* 0x00021d00 */
struct BDG_MIPI_TX_REGS *MIPI_TX_REG;		/* 0x00022000 */
struct BDG_DISPSYS_CONFIG_REGS *DISPSYS_REG;	/* 0x00023000 */
struct BDG_RDMA0_REGS *RDMA_REG;		/* 0x00024000 */
struct BDG_MUTEX_REGS *MUTEX_REG;		/* 0x00025000 */
struct DSI_TX_PHY_TIMCON0_REG timcon0;
struct DSI_TX_PHY_TIMCON1_REG timcon1;
struct DSI_TX_PHY_TIMCON2_REG timcon2;
struct DSI_TX_PHY_TIMCON3_REG timcon3;
unsigned int bg_tx_data_phy_cycle = 0, tx_data_rate = 0, ap_tx_data_rate = 0;
unsigned int ap_tx_dyn_data_rate = 0, tx_dyn_data_rate = 0, bg_mipi_clk_change_sta = 0;
//unsigned int ap_tx_data_phy_cycle = 0;
unsigned int hsa_byte = 0, hbp_byte = 0, hfp_byte = 0, bllp_byte = 0, bg_tx_line_cycle = 0;
//unsigned int ap_tx_hsa_wc = 0, ap_tx_hbp_wc = 0, ap_tx_hfp_wc = 0, ap_tx_bllp_wc = 0;
unsigned int dsc_en;
unsigned int mt6382_init;
unsigned int bdg_tx_mode;
static int bdg_eint_irq;
/***** NFC SRCLKENAI0 Interrupt Handler +++ *****/
static int nfc_eint_irq;
static int mt6382_nfc_srclk;
static bool nfc_irq_already_requested;
static int mt6382_nfc_gpio_value;
/***** NFC SRCLKENAI0 Interrupt Handler --- *****/
static bool irq_already_requested;

#define T_DCO		5  // nominal: 200MHz
int hsrx_clk_div;
int hs_thssettle, fjump_deskew_reg, eye_open_deskew_reg;
int cdr_coarse_trgt_reg, en_dly_deass_thresh_reg;
int max_phase, dll_fbk, coarse_bank, sel_fast;
int post_rcvd_rst_val, post_det_dly_thresh_val;
unsigned int post_rcvd_rst_reg, post_det_dly_thresh_reg;
unsigned int ddl_cntr_ref_reg;

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY		0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

struct lcm_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[256];
};

#define MM_CLK			546 //fpga=26
#define NS_TO_CYCLE(n, c)	((n) / (c) + (((n) % (c)) ? 1 : 0))

#define DSI_MODULE_to_ID(x)	(x == DISP_BDG_DSI0 ? 0 : 1)
#define DSI_MODULE_BEGIN(x)	(x == DISP_BDG_DSIDUAL ? 0 : DSI_MODULE_to_ID(x))
#define DSI_MODULE_END(x)	(x == DISP_BDG_DSIDUAL ? 1 : DSI_MODULE_to_ID(x))

#define DSI_SET_VAL(cmdq, addr, val) \
			((*(volatile unsigned int *)(addr)) = (unsigned int)val)
#define DSI_GET_VAL(addr) (*(volatile unsigned int *)(addr))

//#define SW_EARLY_PORTING
unsigned int mtk_spi_read(u32 addr)
{
	unsigned int value = 0;

#ifdef SW_EARLY_PORTING
#else
	spislv_read(addr, &value, 4);
#endif
//	DISPMSG("%s, addr=0x%08x, value=0x%08x\n", __func__, addr, value);
	return value;
}

int mtk_spi_write(u32 addr, unsigned int regval)
{
	unsigned int value = regval;
	int ret = 0;

//	DISPMSG("mt6382, %s, addr=0x%08x, value=0x%x\n", __func__, addr, value);
#ifdef SW_EARLY_PORTING
#else
	ret = spislv_write(addr, &value, 4);
#endif
	return ret;
}

int mtk_spi_mask_write(u32 addr, u32 msk, u32 value)
{
	unsigned int i = 0;
	unsigned int temp;

	for (i = 0; i < 32; i++) {
		temp = 1 << i;
		if ((msk & temp) == temp)
			break;
	}
	value = value << i;
//	DISPMSG("mt6382, %s, i=%02d, temp=0x%08x, addr=0x%08x, msk=0x%08x, value=0x%x\n",
//		__func__, i, temp, addr, msk, value);
#ifdef SW_EARLY_PORTING
	return 0;
#else
	return spislv_write_register_mask(addr, value, msk);
#endif
}

int mtk_spi_mask_field_write(u32 addr, u32 msk, u32 value)
{
	unsigned int i = 0;
	unsigned int temp;

	for (i = 0; i < 32; i++) {
		temp = 1 << i;
		if ((msk & temp) == temp)
			break;
	}
	value = value << i;
//	DISPMSG("mt6382, %s, i=%02d, temp=0x%08x, addr=0x%08x, msk=0x%08x, value=0x%x\n",
//		__func__, i, temp, addr, msk, value);
#ifdef SW_EARLY_PORTING
	return 0;
#else
	return spislv_write_register_mask(addr, value, msk);
#endif
}

#ifdef SPI_EN
#define DSI_OUTREGBIT(cmdq, TYPE, REG, bit, value) \
do { \
	TYPE r; \
	*(unsigned int *)(&r) = (0x00000000); \
	r.bit = ~(r.bit); \
	mtk_spi_mask_write((unsigned long)(&REG), AS_UINT32(&r), value); \
} while (0)

#define DSI_OUTREG32(cmdq, addr, val) \
do { \
	mtk_spi_write((unsigned long)(&addr), val); \
} while (0)

#define DSI_MASKREG32(cmdq, addr, mask, val) \
do { \
	mtk_spi_mask_write((unsigned long)(addr), mask, val); \
} while (0)
#else
#define DSI_OUTREGBIT(spi_en, cmdq, TYPE, REG, bit, value) \
do { \
	TYPE r; \
	TYPE v; \
	if (spi_en) { \
		*(unsigned int *)(&r) = (0x00000000); \
		r.bit = ~(r.bit); \
		mtk_spi_mask_write((unsigned long)(&REG), AS_UINT32(&r), value); \
	} \
	else { \
		if (cmdq) { \
			*(unsigned int *)(&r) = (0x00000000); \
			r.bit = ~(r.bit);  \
			*(unsigned int *)(&v) = (0x00000000); \
			v.bit = value; \
			DISP_REG_MASK(cmdq, &REG, AS_UINT32(&v), AS_UINT32(&r)); \
		} else { \
			DSI_SET_VAL(NULL, &r, INREG32(&REG)); \
			r.bit = (value); \
			DISP_REG_SET(cmdq, &REG, DSI_GET_VAL(&r)); \
		} \
	} \
} while (0)

#define DSI_OUTREG32(spi_en, cmdq, addr, val) \
do { \
	if (spi_en) { \
		mtk_spi_write((unsigned long)(&addr), val); \
	} \
	else { \
		DISP_REG_SET(cmdq, addr, val); \
	} \
} while (0)

#define DSI_MASKREG32(spi_en, cmdq, addr, mask, val) \
do { \
	if (spi_en) { \
		mtk_spi_mask_write((unsigned long)(addr), mask, val); \
	} \
	else { \
		DISP_REG_SET(cmdq, addr, val); \
	} \
} while (0)
#endif

struct lcm_setting_table nt36672c_60hz[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	//DSC on
	{0xC0, 1, {0x03} },
	{0xC1, 16, {0x89, 0x28, 0x00, 0x08, 0x00, 0xAA,
		0x02, 0x0E, 0x00, 0x2B, 0x00, 0x07,
		0x0D, 0xB7, 0x0C, 0xB7} },
	{0xC2, 2, {0x1B, 0xA0} },

	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0x01, 1, {0x66} },
	{0x32, 1, {0x4D} },
	{0x69, 1, {0xD1} },
	{0xF2, 1, {0x64} },
	{0xF4, 1, {0x64} },
	{0xF6, 1, {0x64} },
	{0xF9, 1, {0x64} },

	{0xFF, 1, {0x26} },
	{0xFB, 1, {0x01} },
	{0x81, 1, {0x0E} },
	{0x84, 1, {0x03} },
	{0x86, 1, {0x03} },
	{0x88, 1, {0x07} },

	{0xFF, 1, {0x27} },
	{0xFB, 1, {0x01} },
	{0xE3, 1, {0x01} },
	{0xE4, 1, {0xEC} },
	{0xE5, 1, {0x02} },
	{0xE6, 1, {0xE3} },
	{0xE7, 1, {0x01} },
	{0xE8, 1, {0xEC} },
	{0xE9, 1, {0x02} },
	{0xEA, 1, {0x22} },
	{0xEB, 1, {0x03} },
	{0xEC, 1, {0x32} },
	{0xED, 1, {0x02} },
	{0xEE, 1, {0x22} },

	{0xFF, 1, {0x2A} },
	{0xFB, 1, {0x01} },
	{0x0C, 1, {0x04} },
	{0x0F, 1, {0x01} },
	{0x11, 1, {0xE0} },
	{0x15, 1, {0x0E} },
	{0x16, 1, {0x78} },
	{0x19, 1, {0x0D} },
	{0x1A, 1, {0xF4} },
	{0x37, 1, {0X6E} },
	{0x88, 1, {0X76} },

	{0xFF, 1, {0x2C} },
	{0xFB, 1, {0x01} },
	{0x4D, 1, {0x1E} },
	{0x4E, 1, {0x04} },
	{0x4F, 1, {0X00} },
	{0x9D, 1, {0X1E} },
	{0x9E, 1, {0X04} },

	{0xFF, 1, {0xF0} },
	{0xFB, 1, {0x01} },
	{0x5A, 1, {0x00} },

	{0xFF, 1, {0xE0} },
	{0xFB, 1, {0x01} },
	{0x25, 1, {0x02} },
	{0x4E, 1, {0x02} },
	{0x85, 1, {0x02} },

	{0xFF, 1, {0XD0} },
	{0xFB, 1, {0x01} },
	{0X09, 1, {0XAD} },

	{0xFF, 1, {0X20} },
	{0xFB, 1, {0x01} },
	{0XF8, 1, {0X64} },

	{0xFF, 1, {0X2A} },
	{0xFB, 1, {0x01} },
	{0X1A, 1, {0XF0} },
	{0x30, 1, {0x5E} },
	{0x31, 1, {0xCA} },
	{0x34, 1, {0xFE} },
	{0x35, 1, {0x35} },
	{0x36, 1, {0xA2} },
	{0x37, 1, {0xF8} },
	{0x38, 1, {0x37} },
	{0x39, 1, {0xA0} },
	{0x3A, 1, {0x5E} },
	{0x53, 1, {0xD7} },
	{0x88, 1, {0x72} },
	{0x88, 1, {0x72} },

	{0xFF, 1, {0x24} },
	{0xFB, 1, {0x01} },
	{0xC6, 1, {0xC0} },

	{0xFF, 1, {0xE0} },
	{0xFB, 1, {0x01} },
	{0x25, 1, {0x00} },
	{0x4E, 1, {0x02} },
	{0x35, 1, {0x82} },

	{0xFF, 1, {0xC0} },
	{0xFB, 1, {0x01} },
	{0x9C, 1, {0x11} },
	{0x9D, 1, {0x11} },
	//60HZ VESA DSC
	{0xFF, 1, {0x25} },
	{0xFB, 1, {0x01} },
	{0x18, 1, {0x22} },

	//CCMRUN
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0xC0, 1, {0x03} },
	{0x51, 1, {0x00} },
	{0x35, 1, {0x00} },
	{0x53, 1, {0x24} },
	{0x55, 1, {0x00} },
	{0xFF, 1, {0x10} },
	{0x11, 0, {} },
#ifndef LCM_SET_DISPLAY_ON_DELAY
	{REGFLAG_DELAY, 120, {} },
	/* Display On*/
	{0x29, 0, {} },
#endif
};

struct lcm_setting_table nt36672c_90hz[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	//DSC on
	{0xC0, 1, {0x03} },
	{0xC1, 16, {0x89, 0x28, 0x00, 0x08, 0x00, 0xAA,
		0x02, 0x0E, 0x00, 0x2B, 0x00, 0x07,
		0x0D, 0xB7, 0x0C, 0xB7} },
	{0xC2, 2, {0x1B, 0xA0} },

	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0x01, 1, {0x66} },
	{0x32, 1, {0x4D} },
	{0x69, 1, {0xD1} },
	{0xF2, 1, {0x64} },
	{0xF4, 1, {0x64} },
	{0xF6, 1, {0x64} },
	{0xF9, 1, {0x64} },

	{0xFF, 1, {0x26} },
	{0xFB, 1, {0x01} },
	{0x81, 1, {0x0E} },
	{0x84, 1, {0x03} },
	{0x86, 1, {0x03} },
	{0x88, 1, {0x07} },

	{0xFF, 1, {0x27} },
	{0xFB, 1, {0x01} },
	{0xE3, 1, {0x01} },
	{0xE4, 1, {0xEC} },
	{0xE5, 1, {0x02} },
	{0xE6, 1, {0xE3} },
	{0xE7, 1, {0x01} },
	{0xE8, 1, {0xEC} },
	{0xE9, 1, {0x02} },
	{0xEA, 1, {0x22} },
	{0xEB, 1, {0x03} },
	{0xEC, 1, {0x32} },
	{0xED, 1, {0x02} },
	{0xEE, 1, {0x22} },

	{0xFF, 1, {0x2A} },
	{0xFB, 1, {0x01} },
	{0x0C, 1, {0x04} },
	{0x0F, 1, {0x01} },
	{0x11, 1, {0xE0} },
	{0x15, 1, {0x0E} },
	{0x16, 1, {0x78} },
	{0x19, 1, {0x0D} },
	{0x1A, 1, {0xF4} },
	{0x37, 1, {0x6E} },
	{0x88, 1, {0x76} },

	{0xFF, 1, {0x2C} },
	{0xFB, 1, {0x01} },
	{0x4D, 1, {0x1E} },
	{0x4E, 1, {0x04} },
	{0x4F, 1, {0x00} },
	{0x9D, 1, {0x1E} },
	{0x9E, 1, {0x04} },

	{0xFF, 1, {0xF0} },
	{0xFB, 1, {0x01} },
	{0x5A, 1, {0x00} },

	{0xFF, 1, {0xE0} },
	{0xFB, 1, {0x01} },
	{0x25, 1, {0x02} },
	{0x4E, 1, {0x02} },
	{0x85, 1, {0x02} },

	{0xFF, 1, {0xD0} },
	{0xFB, 1, {0x01} },
	{0X09, 1, {0xAD} },

	{0xFF, 1, {0X20} },
	{0xFB, 1, {0x01} },
	{0XF8, 1, {0x64} },

	{0xFF, 1, {0x2A} },
	{0xFB, 1, {0x01} },
	{0X1A, 1, {0xF0} },
	{0x30, 1, {0x5E} },
	{0x31, 1, {0xCA} },
	{0x34, 1, {0xFE} },
	{0x35, 1, {0x35} },
	{0x36, 1, {0xA2} },

	{0x36, 1, {0xA2} },
	{0x37, 1, {0xF8} },
	{0x38, 1, {0x37} },
	{0x39, 1, {0xA0} },
	{0x3A, 1, {0x5E} },
	{0x53, 1, {0xD7} },
	{0x88, 1, {0x72} },
	{0x88, 1, {0x72} },

	{0xFF, 1, {0x24} },
	{0xFB, 1, {0x01} },
	{0xC6, 1, {0xC0} },

	{0xFF, 1, {0xE0} },
	{0xFB, 1, {0x01} },
	{0x25, 1, {0x00} },
	{0x4E, 1, {0x02} },
	{0x35, 1, {0x82} },
	{0xFF, 1, {0xC0} },

	{0xFF, 1, {0xC0} },
	{0xFB, 1, {0x01} },
	{0x9C, 1, {0x11} },
	{0x9D, 1, {0x11} },
	//90HZ VESA DSC
	{0xFF, 1, {0x25} },
	{0xFB, 1, {0x01} },
	{0x18, 1, {0x21} },

	//CCMRUN
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0xC0, 1, {0x03} },
	{0x51, 1, {0x00} },
	{0x35, 1, {0x00} },
	{0x53, 1, {0x24} },

	{0x53, 1, {0x24} },
	{0x55, 1, {0x00} },
	{0xFF, 1, {0x10} },
	{0x11, 0, {} },
#ifndef LCM_SET_DISPLAY_ON_DELAY
	{REGFLAG_DELAY, 120, {} },
	/* Display On*/
	{0x29, 0, {} },
#endif
};

struct lcm_setting_table nt36672c_120hz[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	//DSC on
	{0xC0, 1, {0x03} },
	{0xC1, 16, {0x89, 0x28, 0x00, 0x08, 0x00, 0xAA,
		0x02, 0x0E, 0x00, 0x2B, 0x00, 0x07,
		0x0D, 0xB7, 0x0C, 0xB7} },
	{0xC2, 2, {0x1B, 0xA0} },

	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0x01, 1, {0x66} },
	{0x32, 1, {0x4D} },
	{0x69, 1, {0xD1} },
	{0xF2, 1, {0x64} },
	{0xF4, 1, {0x64} },
	{0xF6, 1, {0x64} },
	{0xF9, 1, {0x64} },

	{0xFF, 1, {0x26} },
	{0xFB, 1, {0x01} },
	{0x81, 1, {0x0E} },
	{0x84, 1, {0x03} },
	{0x86, 1, {0x03} },
	{0x88, 1, {0x07} },

	{0xFF, 1, {0x27} },
	{0xFB, 1, {0x01} },
	{0xE3, 1, {0x01} },
	{0xE4, 1, {0xEC} },
	{0xE5, 1, {0x02} },
	{0xE6, 1, {0xE3} },
	{0xE7, 1, {0x01} },
	{0xE8, 1, {0xEC} },
	{0xE9, 1, {0x02} },
	{0xEA, 1, {0x22} },
	{0xEB, 1, {0x03} },
	{0xEC, 1, {0x32} },
	{0xED, 1, {0x02} },
	{0xEE, 1, {0x22} },

	{0xFF, 1, {0x2A} },
	{0xFB, 1, {0x01} },
	{0x0C, 1, {0x04} },
	{0x0F, 1, {0x01} },
	{0x11, 1, {0xE0} },
	{0x15, 1, {0x0E} },
	{0x16, 1, {0x78} },
	{0x19, 1, {0x0D} },
	{0x1A, 1, {0xF4} },
	{0x37, 1, {0X6E} },
	{0x88, 1, {0X76} },

	{0xFF, 1, {0x2C} },
	{0xFB, 1, {0x01} },
	{0x4D, 1, {0x1E} },
	{0x4E, 1, {0x04} },
	{0x4F, 1, {0X00} },
	{0x9D, 1, {0X1E} },
	{0x9E, 1, {0X04} },

	{0xFF, 1, {0xF0} },
	{0xFB, 1, {0x01} },
	{0x5A, 1, {0x00} },

	{0xFF, 1, {0xE0} },
	{0xFB, 1, {0x01} },
	{0x25, 1, {0x02} },
	{0x4E, 1, {0x02} },
	{0x85, 1, {0x02} },

	{0xFF, 1, {0XD0} },
	{0xFB, 1, {0x01} },
	{0X09, 1, {0XAD} },

	{0xFF, 1, {0X20} },
	{0xFB, 1, {0x01} },
	{0XF8, 1, {0X64} },

	{0xFF, 1, {0X2A} },
	{0xFB, 1, {0x01} },
	{0X1A, 1, {0XF0} },
	{0x30, 1, {0x5E} },
	{0x31, 1, {0xCA} },
	{0x34, 1, {0xFE} },
	{0x35, 1, {0x35} },
	{0x36, 1, {0xA2} },
	{0x37, 1, {0xF8} },
	{0x38, 1, {0x37} },
	{0x39, 1, {0xA0} },
	{0x3A, 1, {0x5E} },
	{0x53, 1, {0xD7} },
	{0x88, 1, {0x72} },
	{0x88, 1, {0x72} },

	{0xFF, 1, {0x24} },
	{0xFB, 1, {0x01} },
	{0xC6, 1, {0xC0} },

	{0xFF, 1, {0xE0} },
	{0xFB, 1, {0x01} },
	{0x25, 1, {0x00} },
	{0x4E, 1, {0x02} },
	{0x35, 1, {0x82} },

	{0xFF, 1, {0xC0} },
	{0xFB, 1, {0x01} },
	{0x9C, 1, {0x11} },
	{0x9D, 1, {0x11} },

	//CCMRUN
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0xC0, 1, {0x03} },
	{0x51, 1, {0x00} },
	{0x35, 1, {0x00} },
	{0x53, 1, {0x24} },
	{0x55, 1, {0x00} },
	{0xFF, 1, {0x10} },
	{0x11, 0, {} },
#ifndef LCM_SET_DISPLAY_ON_DELAY
	{REGFLAG_DELAY, 120, {} },
	/* Display On*/
	{0x29, 0, {} },
#endif
};

struct lcm_setting_table nt36672c_wo_dsc[] = {
	{0xC0, 1, {0x00} },
};

struct lcm_setting_table nt35695b_cmd_mode[] = {
	{0xFF, 1, {0x24} },
	{0xFB, 1, {0x01} },
	{0xC5, 1, {0x31} },


	{0xFF, 1, {0x20} },
	{0x00, 1, {0x01} },
	{0x01, 1, {0x55} },
	{0x02, 1, {0x45} },
	{0x03, 1, {0x55} },
	{0x05, 1, {0x40} },/* VGH=2xAVDD, VGL=2xAVEE */
	{0x06, 1, {0x99} },
	{0x07, 1, {0x9E} },
	{0x08, 1, {0x0C} },
	{0x0B, 1, {0x87} },
	{0x0C, 1, {0x87} },
	{0x0E, 1, {0xAB} },
	{0x0F, 1, {0xA9} },
	{0x11, 1, {0x0D} },/* VCOM */
	{0x12, 1, {0x10} },/* VCOM */
	{0x13, 1, {0x03} },
	{0x14, 1, {0x4A} },
	{0x15, 1, {0x12} },
	{0x16, 1, {0x12} },
	{0x30, 1, {0x01} },
	{0x58, 1, {0x00} },
	{0x59, 1, {0x00} },
	{0x5A, 1, {0x02} },
	{0x5B, 1, {0x00} },
	{0x5C, 1, {0x00} },
	{0x5D, 1, {0x00} },
	{0x5E, 1, {0x02} },
	{0x5F, 1, {0x02} },
	{0x72, 1, {0x31} },
	{0xFB, 1, {0x01} },

	{0xFF, 1, {0x24} },
	{0x00, 1, {0x01} },
	{0x01, 1, {0x0B} },
	{0x02, 1, {0x0C} },
	{0x03, 1, {0x03} },
	{0x04, 1, {0x05} },
	{0x05, 1, {0x1C} },
	{0x06, 1, {0x10} },
	{0x07, 1, {0x00} },
	{0x08, 1, {0x1C} },
	{0x09, 1, {0x00} },
	{0x0A, 1, {0x00} },
	{0x0B, 1, {0x00} },
	{0x0C, 1, {0x00} },
	{0x0D, 1, {0x13} },
	{0x0E, 1, {0x15} },
	{0x0F, 1, {0x17} },
	{0x10, 1, {0x01} },
	{0x11, 1, {0x0B} },
	{0x12, 1, {0x0C} },
	{0x13, 1, {0x04} },
	{0x14, 1, {0x06} },
	{0x15, 1, {0x1C} },
	{0x16, 1, {0x10} },
	{0x17, 1, {0x00} },
	{0x18, 1, {0x1C} },
	{0x19, 1, {0x00} },
	{0x1A, 1, {0x00} },
	{0x1B, 1, {0x00} },
	{0x1C, 1, {0x00} },
	{0x1D, 1, {0x13} },
	{0x1E, 1, {0x15} },
	{0x1F, 1, {0x17} },
	{0x20, 1, {0x00} },/* STV */
	{0x21, 1, {0x03} },
	{0x22, 1, {0x01} },
	{0x23, 1, {0x4A} },
	{0x24, 1, {0x4A} },
	{0x25, 1, {0x6D} },
	{0x26, 1, {0x40} },
	{0x27, 1, {0x40} },
	{0x32, 1, {0x7B} },
	{0x33, 1, {0x00} },
	{0x34, 1, {0x01} },
	{0x35, 1, {0x8E} },
	{0x39, 1, {0x01} },
	{0x3A, 1, {0x8E} },

	{0xBD, 1, {0x20} },/* VEND */
	{0xB6, 1, {0x22} },
	{0xB7, 1, {0x24} },
	{0xB8, 1, {0x07} },
	{0xB9, 1, {0x07} },
	{0xC1, 1, {0x6D} },
	/* disable Vblank protection for low fps power saving (for vdo mode)*/
	{0xC2, 1, {0x00} },
	{0xC4, 1, {0x24} },/* updated */

	{0xBE, 1, {0x07} },
	{0xBF, 1, {0x07} },
	{0x29, 1, {0xD8} },/* UD */
	{0x2A, 1, {0x2A} },

	{0x5B, 1, {0x43} },/* CTRL */
	{0x5C, 1, {0x00} },
	{0x5F, 1, {0x73} },
	{0x60, 1, {0x73} },
	{0x63, 1, {0x22} },
	{0x64, 1, {0x00} },
	{0x67, 1, {0x08} },
	{0x68, 1, {0x04} },

	{0x7A, 1, {0x80} },/* MUX */
	{0x7B, 1, {0x91} },
	{0x7C, 1, {0xD8} },
	{0x7D, 1, {0x60} },
	{0x74, 1, {0x09} },
	{0x7E, 1, {0x09} },
	{0x75, 1, {0x21} },
	{0x7F, 1, {0x21} },
	{0x76, 1, {0x05} },
	{0x81, 1, {0x05} },
	{0x77, 1, {0x04} },
	{0x82, 1, {0x04} },

	{0x93, 1, {0x06} },/* FP,BP */
	{0x94, 1, {0x06} },
	{0xB3, 1, {0x00} },
	{0xB4, 1, {0x00} },
	{0xB5, 1, {0x00} },

	{0x78, 1, {0x00} },/* SOURCE EQ */
	{0x79, 1, {0x00} },
	{0x80, 1, {0x00} },
	{0x83, 1, {0x00} },
	{0x84, 1, {0x04} },


	{0x8A, 1, {0x33} },/* /Inversion Type// pixel column driving */
	{0x8B, 1, {0xF0} },
	{0x9B, 1, {0x0F} },
	{0xC6, 1, {0x09} },
	{0xFB, 1, {0x01} },
	{0xEC, 1, {0x00} },


	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0x75, 1, {0x00} },
	{0x76, 1, {0x49} },
	{0x77, 1, {0x00} },
	{0x78, 1, {0x78} },
	{0x79, 1, {0x00} },
	{0x7A, 1, {0xA4} },
	{0x7B, 1, {0x00} },
	{0x7C, 1, {0xC2} },
	{0x7D, 1, {0x00} },
	{0x7E, 1, {0xDA} },
	{0x7F, 1, {0x00} },
	{0x80, 1, {0xED} },
	{0x81, 1, {0x00} },
	{0x82, 1, {0xFE} },
	{0x83, 1, {0x01} },
	{0x84, 1, {0x0E} },
	{0x85, 1, {0x01} },
	{0x86, 1, {0x1B} },
	{0x87, 1, {0x01} },
	{0x88, 1, {0x48} },
	{0x89, 1, {0x01} },
	{0x8A, 1, {0x6C} },
	{0x8B, 1, {0x01} },
	{0x8C, 1, {0xA2} },
	{0x8D, 1, {0x01} },
	{0x8E, 1, {0xCD} },
	{0x8F, 1, {0x02} },
	{0x90, 1, {0x0F} },
	{0x91, 1, {0x02} },
	{0x92, 1, {0x42} },
	{0x93, 1, {0x02} },
	{0x94, 1, {0x43} },
	{0x95, 1, {0x02} },
	{0x96, 1, {0x71} },
	{0x97, 1, {0x02} },
	{0x98, 1, {0xA3} },
	{0x99, 1, {0x02} },
	{0x9A, 1, {0xC5} },
	{0x9B, 1, {0x02} },
	{0x9C, 1, {0xF3} },
	{0x9D, 1, {0x03} },
	{0x9E, 1, {0x12} },
	{0x9F, 1, {0x03} },
	{0xA0, 1, {0x3A} },
	{0xA2, 1, {0x03} },
	{0xA3, 1, {0x46} },
	{0xA4, 1, {0x03} },
	{0xA5, 1, {0x52} },
	{0xA6, 1, {0x03} },
	{0xA7, 1, {0x60} },
	{0xA9, 1, {0x03} },
	{0xAA, 1, {0x6E} },
	{0xAB, 1, {0x03} },
	{0xAC, 1, {0x7D} },
	{0xAD, 1, {0x03} },
	{0xAE, 1, {0x8B} },
	{0xAF, 1, {0x03} },
	{0xB0, 1, {0x91} },
	{0xB1, 1, {0x03} },
	{0xB2, 1, {0xCF} },
	{0xB3, 1, {0x00} },/* RN GAMMA SETTING */
	{0xB4, 1, {0x49} },
	{0xB5, 1, {0x00} },
	{0xB6, 1, {0x78} },
	{0xB7, 1, {0x00} },
	{0xB8, 1, {0xA4} },
	{0xB9, 1, {0x00} },
	{0xBA, 1, {0xC2} },
	{0xBB, 1, {0x00} },
	{0xBC, 1, {0xDA} },
	{0xBD, 1, {0x00} },
	{0xBE, 1, {0xED} },
	{0xBF, 1, {0x00} },
	{0xC0, 1, {0xFE} },
	{0xC1, 1, {0x01} },
	{0xC2, 1, {0x0E} },
	{0xC3, 1, {0x01} },
	{0xC4, 1, {0x1B} },
	{0xC5, 1, {0x01} },
	{0xC6, 1, {0x48} },
	{0xC7, 1, {0x01} },
	{0xC8, 1, {0x6C} },
	{0xC9, 1, {0x01} },
	{0xCA, 1, {0xA2} },
	{0xCB, 1, {0x01} },
	{0xCC, 1, {0xCD} },
	{0xCD, 1, {0x02} },
	{0xCE, 1, {0x0F} },
	{0xCF, 1, {0x02} },
	{0xD0, 1, {0x42} },
	{0xD1, 1, {0x02} },
	{0xD2, 1, {0x43} },
	{0xD3, 1, {0x02} },
	{0xD4, 1, {0x71} },
	{0xD5, 1, {0x02} },
	{0xD6, 1, {0xA3} },
	{0xD7, 1, {0x02} },
	{0xD8, 1, {0xC5} },
	{0xD9, 1, {0x02} },
	{0xDA, 1, {0xF3} },
	{0xDB, 1, {0x03} },
	{0xDC, 1, {0x12} },
	{0xDD, 1, {0x03} },
	{0xDE, 1, {0x3A} },
	{0xDF, 1, {0x03} },
	{0xE0, 1, {0x46} },
	{0xE1, 1, {0x03} },
	{0xE2, 1, {0x52} },
	{0xE3, 1, {0x03} },
	{0xE4, 1, {0x60} },
	{0xE5, 1, {0x03} },
	{0xE6, 1, {0x6E} },
	{0xE7, 1, {0x03} },
	{0xE8, 1, {0x7D} },
	{0xE9, 1, {0x03} },
	{0xEA, 1, {0x8B} },
	{0xEB, 1, {0x03} },
	{0xEC, 1, {0x91} },
	{0xED, 1, {0x03} },
	{0xEE, 1, {0xCF} },

	{0xEF, 1, {0x00} },/* GP GAMMA SETTING */
	{0xF0, 1, {0x49} },
	{0xF1, 1, {0x00} },
	{0xF2, 1, {0x78} },
	{0xF3, 1, {0x00} },
	{0xF4, 1, {0xA4} },
	{0xF5, 1, {0x00} },
	{0xF6, 1, {0xC2} },
	{0xF7, 1, {0x00} },
	{0xF8, 1, {0xDA} },
	{0xF9, 1, {0x00} },
	{0xFA, 1, {0xED} },

	{0xFF, 1, {0x21} },/* CMD2 PAGE1 */
	{0xFB, 1, {0x01} },

	{0x00, 1, {0x00} },
	{0x01, 1, {0xFE} },
	{0x02, 1, {0x01} },
	{0x03, 1, {0x0E} },
	{0x04, 1, {0x01} },
	{0x05, 1, {0x1B} },
	{0x06, 1, {0x01} },
	{0x07, 1, {0x48} },
	{0x08, 1, {0x01} },
	{0x09, 1, {0x6C} },
	{0x0A, 1, {0x01} },
	{0x0B, 1, {0xA2} },
	{0x0C, 1, {0x01} },
	{0x0D, 1, {0xCD} },
	{0x0E, 1, {0x02} },
	{0x0F, 1, {0x0F} },
	{0x10, 1, {0x02} },
	{0x11, 1, {0x42} },
	{0x12, 1, {0x02} },
	{0x13, 1, {0x43} },
	{0x14, 1, {0x02} },
	{0x15, 1, {0x71} },
	{0x16, 1, {0x02} },
	{0x17, 1, {0xA3} },
	{0x18, 1, {0x02} },
	{0x19, 1, {0xC5} },
	{0x1A, 1, {0x02} },
	{0x1B, 1, {0xF3} },
	{0x1C, 1, {0x03} },
	{0x1D, 1, {0x12} },
	{0x1E, 1, {0x03} },
	{0x1F, 1, {0x3A} },
	{0x20, 1, {0x03} },
	{0x21, 1, {0x46} },
	{0x22, 1, {0x03} },
	{0x23, 1, {0x52} },
	{0x24, 1, {0x03} },
	{0x25, 1, {0x60} },
	{0x26, 1, {0x03} },
	{0x27, 1, {0x6E} },
	{0x28, 1, {0x03} },
	{0x29, 1, {0x7D} },
	{0x2A, 1, {0x03} },
	{0x2B, 1, {0x8B} },
	{0x2D, 1, {0x03} },
	{0x2F, 1, {0x91} },
	{0x30, 1, {0x03} },
	{0x31, 1, {0xCF} },
	{0x32, 1, {0x00} },
	{0x33, 1, {0x49} },
	{0x34, 1, {0x00} },
	{0x35, 1, {0x78} },
	{0x36, 1, {0x00} },
	{0x37, 1, {0xA4} },
	{0x38, 1, {0x00} },
	{0x39, 1, {0xC2} },
	{0x3A, 1, {0x00} },
	{0x3B, 1, {0xDA} },
	{0x3D, 1, {0x00} },
	{0x3F, 1, {0xED} },
	{0x40, 1, {0x00} },
	{0x41, 1, {0xFE} },
	{0x42, 1, {0x01} },
	{0x43, 1, {0x0E} },
	{0x44, 1, {0x01} },
	{0x45, 1, {0x1B} },
	{0x46, 1, {0x01} },
	{0x47, 1, {0x48} },
	{0x48, 1, {0x01} },
	{0x49, 1, {0x6C} },
	{0x4A, 1, {0x01} },
	{0x4B, 1, {0xA2} },
	{0x4C, 1, {0x01} },
	{0x4D, 1, {0xCD} },
	{0x4E, 1, {0x02} },
	{0x4F, 1, {0x0F} },
	{0x50, 1, {0x02} },
	{0x51, 1, {0x42} },
	{0x52, 1, {0x02} },
	{0x53, 1, {0x43} },
	{0x54, 1, {0x02} },
	{0x55, 1, {0x71} },
	{0x56, 1, {0x02} },
	{0x58, 1, {0xA3} },
	{0x59, 1, {0x02} },
	{0x5A, 1, {0xC5} },
	{0x5B, 1, {0x02} },
	{0x5C, 1, {0xF3} },
	{0x5D, 1, {0x03} },
	{0x5E, 1, {0x12} },
	{0x5F, 1, {0x03} },
	{0x60, 1, {0x3A} },
	{0x61, 1, {0x03} },
	{0x62, 1, {0x46} },
	{0x63, 1, {0x03} },
	{0x64, 1, {0x52} },
	{0x65, 1, {0x03} },
	{0x66, 1, {0x60} },
	{0x67, 1, {0x03} },
	{0x68, 1, {0x6E} },
	{0x69, 1, {0x03} },
	{0x6A, 1, {0x7D} },
	{0x6B, 1, {0x03} },
	{0x6C, 1, {0x8B} },
	{0x6D, 1, {0x03} },
	{0x6E, 1, {0x91} },
	{0x6F, 1, {0x03} },
	{0x70, 1, {0xCF} },
	{0x71, 1, {0x00} },/* BP GAMMA SETTING */
	{0x72, 1, {0x49} },
	{0x73, 1, {0x00} },
	{0x74, 1, {0x78} },
	{0x75, 1, {0x00} },
	{0x76, 1, {0xA4} },
	{0x77, 1, {0x00} },
	{0x78, 1, {0xC2} },
	{0x79, 1, {0x00} },
	{0x7A, 1, {0xDA} },
	{0x7B, 1, {0x00} },
	{0x7C, 1, {0xED} },
	{0x7D, 1, {0x00} },
	{0x7E, 1, {0xFE} },
	{0x7F, 1, {0x01} },
	{0x80, 1, {0x0E} },
	{0x81, 1, {0x01} },
	{0x82, 1, {0x1B} },
	{0x83, 1, {0x01} },
	{0x84, 1, {0x48} },
	{0x85, 1, {0x01} },
	{0x86, 1, {0x6C} },
	{0x87, 1, {0x01} },
	{0x88, 1, {0xA2} },
	{0x89, 1, {0x01} },
	{0x8A, 1, {0xCD} },
	{0x8B, 1, {0x02} },
	{0x8C, 1, {0x0F} },
	{0x8D, 1, {0x02} },
	{0x8E, 1, {0x42} },
	{0x8F, 1, {0x02} },
	{0x90, 1, {0x43} },
	{0x91, 1, {0x02} },
	{0x92, 1, {0x71} },
	{0x93, 1, {0x02} },
	{0x94, 1, {0xA3} },
	{0x95, 1, {0x02} },
	{0x96, 1, {0xC5} },
	{0x97, 1, {0x02} },
	{0x98, 1, {0xF3} },
	{0x99, 1, {0x03} },
	{0x9A, 1, {0x12} },
	{0x9B, 1, {0x03} },
	{0x9C, 1, {0x3A} },
	{0x9D, 1, {0x03} },
	{0x9E, 1, {0x46} },
	{0x9F, 1, {0x03} },
	{0xA0, 1, {0x52} },
	{0xA2, 1, {0x03} },
	{0xA3, 1, {0x60} },
	{0xA4, 1, {0x03} },
	{0xA5, 1, {0x6E} },
	{0xA6, 1, {0x03} },
	{0xA7, 1, {0x7D} },
	{0xA9, 1, {0x03} },
	{0xAA, 1, {0x8B} },
	{0xAB, 1, {0x03} },
	{0xAC, 1, {0x91} },
	{0xAD, 1, {0x03} },
	{0xAE, 1, {0xCF} },
	{0xAF, 1, {0x00} },
	{0xB0, 1, {0x49} },
	{0xB1, 1, {0x00} },
	{0xB2, 1, {0x78} },
	{0xB3, 1, {0x00} },
	{0xB4, 1, {0xA4} },
	{0xB5, 1, {0x00} },
	{0xB6, 1, {0xC2} },
	{0xB7, 1, {0x00} },
	{0xB8, 1, {0xDA} },
	{0xB9, 1, {0x00} },
	{0xBA, 1, {0xED} },
	{0xBB, 1, {0x00} },
	{0xBC, 1, {0xFE} },
	{0xBD, 1, {0x01} },
	{0xBE, 1, {0x0E} },
	{0xBF, 1, {0x01} },
	{0xC0, 1, {0x1B} },
	{0xC1, 1, {0x01} },
	{0xC2, 1, {0x48} },
	{0xC3, 1, {0x01} },
	{0xC4, 1, {0x6C} },
	{0xC5, 1, {0x01} },
	{0xC6, 1, {0xA2} },
	{0xC7, 1, {0x01} },
	{0xC8, 1, {0xCD} },
	{0xC9, 1, {0x02} },
	{0xCA, 1, {0x0F} },
	{0xCB, 1, {0x02} },
	{0xCC, 1, {0x42} },
	{0xCD, 1, {0x02} },
	{0xCE, 1, {0x43} },
	{0xCF, 1, {0x02} },
	{0xD0, 1, {0x71} },
	{0xD1, 1, {0x02} },
	{0xD2, 1, {0xA3} },
	{0xD3, 1, {0x02} },
	{0xD4, 1, {0xC5} },
	{0xD5, 1, {0x02} },
	{0xD6, 1, {0xF3} },
	{0xD7, 1, {0x03} },
	{0xD8, 1, {0x12} },
	{0xD9, 1, {0x03} },
	{0xDA, 1, {0x3A} },
	{0xDB, 1, {0x03} },
	{0xDC, 1, {0x46} },
	{0xDD, 1, {0x03} },
	{0xDE, 1, {0x52} },
	{0xDF, 1, {0x03} },
	{0xE0, 1, {0x60} },
	{0xE1, 1, {0x03} },
	{0xE2, 1, {0x6E} },
	{0xE3, 1, {0x03} },
	{0xE4, 1, {0x7D} },
	{0xE5, 1, {0x03} },
	{0xE6, 1, {0x8B} },
	{0xE7, 1, {0x03} },
	{0xE8, 1, {0x91} },
	{0xE9, 1, {0x03} },
	{0xEA, 1, {0xCF} },

	{0xFF, 1, {0x10} }, /* Return  To CMD1 */
	{REGFLAG_UDELAY, 1, {} },
	{0x3B, 3, {0x03, 0x0a, 0x0a} },

	{0x35, 1, {0x00} },
	/* set TE event @ line 0x778(1912) for partial update */
	{0x44, 2, {0x07, 0x78} },

	/* don't reload cmd1 setting from MTP when exit sleep.
	 * (or C9 will be overwritten)
	 */
	{0xFB, 1, {0x01} },
	/* set partial update option */
	{0xC9, 11, {0x49, 0x02, 0x05, 0x00,
				0x0F, 0x06, 0x67, 0x03,
				0x2E, 0x10, 0xF0} },

	{0xBB, 1, {0x10} },/* 0x03:video mode  0x10:command mode */

	/*{REGFLAG_DELAY, 200, {} },*/
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{0x29, 0, {} },
	/*{REGFLAG_DELAY, 200, {} },*/
	/* ///////////////////CABC SETTING///////// */
	{0x51, 1, {0x00} },
	{0x5E, 1, {0x00} },
	{0x53, 1, {0x24} },
	{0x55, 1, {0x00} },
};

void bdg_tx_pull_6382_reset_pin(void)
{
	DISPFUNCSTART();
	bdg_tx_set_6382_reset_pin(1);
	udelay(10);
	bdg_tx_set_6382_reset_pin(0);
	udelay(10);
	bdg_tx_set_6382_reset_pin(1);
	DISPFUNCEND();
}

void bdg_tx_set_6382_reset_pin(unsigned int value)
{
	if (value)
		disp_dts_gpio_select_state(DTS_GPIO_STATE_6382_RST_OUT1);
	else
		disp_dts_gpio_select_state(DTS_GPIO_STATE_6382_RST_OUT0);
}

void set_LDO_on(void *cmdq)
{
	unsigned int reg1, reg2, reg3;
	unsigned int timeout = 100;

	DISPFUNCSTART();

	DSI_OUTREGBIT(cmdq, struct SYSREG_LDO_CTRL0_REG,
			SYS_REG->SYSREG_LDO_CTRL0, RG_PHYLDO_MASKB, 1);
	DSI_OUTREGBIT(cmdq, struct SYSREG_LDO_CTRL0_REG,
			SYS_REG->SYSREG_LDO_CTRL0, RG_LDO_TRIM_BY_EFUSE, 0);

	while (timeout) {
		reg1 = (mtk_spi_read((unsigned long)(&SYS_REG->LDO_STATUS)) & 0x300);
//		DISPMSG("mt6382, %s, LDO_STATUS=0x%x\n", __func__, reg1);

		if (reg1 == 0x300)
			break;
		udelay(10);
		timeout--;
	}

	if (timeout == 0)
		return;

	reg1 = (mtk_spi_read((unsigned long)(&EFUSE->STATUS)) & 0x7fffff);
	DISPMSG("mt6382, %s, EFUSE_STATUS=0x%x\n", __func__, reg1);

	if (reg1 != 0) {
		reg1 = (mtk_spi_read((unsigned long)(&EFUSE->TRIM1)) & 0x3f);
		reg2 = (mtk_spi_read((unsigned long)(&EFUSE->TRIM2)) & 0xf);
		reg3 = (mtk_spi_read((unsigned long)(&EFUSE->TRIM3)) & 0xf);
		DISPMSG("mt6382, %s, TRIM1=0x%x, TRIM2=0x%x, TRIM3=0x%x\n",
			__func__, reg1, reg2, reg3);

		if ((reg1 != 0) | (reg2 != 0) | (reg2 != 0)) {
			DSI_OUTREGBIT(cmdq, struct SYSREG_LDO_CTRL0_REG,
					SYS_REG->SYSREG_LDO_CTRL0, RG_LDO_TRIM_BY_EFUSE, 1);
		}
	}

	DSI_OUTREGBIT(cmdq, struct SYSREG_LDO_CTRL1_REG,
			SYS_REG->SYSREG_LDO_CTRL1, RG_PHYLDO1_LP_EN, 0);
	DSI_OUTREGBIT(cmdq, struct SYSREG_LDO_CTRL1_REG,
			SYS_REG->SYSREG_LDO_CTRL1, RG_PHYLDO2_EN, 1);
	DISPFUNCEND();
}

void set_LDO_off(void *cmdq)
{
	DISPFUNCSTART();

	DSI_OUTREGBIT(cmdq, struct SYSREG_LDO_CTRL1_REG,
			SYS_REG->SYSREG_LDO_CTRL1, RG_PHYLDO2_EN, 0);
	DSI_OUTREGBIT(cmdq, struct SYSREG_LDO_CTRL1_REG,
			SYS_REG->SYSREG_LDO_CTRL1, RG_PHYLDO1_LP_EN, 1);
	DISPFUNCEND();
}

void set_mtcmos_on(void *cmdq)
{
	unsigned int reg = 0;

	DISPFUNCSTART();

	DSI_OUTREGBIT(cmdq, struct SYSREG_PWR_CTRL_REG,
			SYS_REG->SYSREG_PWR_CTRL, REG_PWR_ON, 1);
	DSI_OUTREGBIT(cmdq, struct SYSREG_PWR_CTRL_REG,
			SYS_REG->SYSREG_PWR_CTRL, REG_PWR_ON_2ND, 1);
	DSI_OUTREGBIT(cmdq, struct SYSREG_RST_CTRL_REG,
			SYS_REG->SYSREG_RST_CTRL, REG_SW_RST_EN_DISP_PWR_WRAP, 0);
	DSI_OUTREGBIT(cmdq, struct SYSREG_PWR_CTRL_REG,
			SYS_REG->SYSREG_PWR_CTRL, REG_PISO_EN, 0);
	DSI_OUTREGBIT(cmdq, struct SYSREG_PWR_CTRL_REG,
			SYS_REG->SYSREG_PWR_CTRL, REG_DISP_PWR_CLK_DIS, 0);
	DSI_OUTREGBIT(cmdq, struct SYSREG_PWR_CTRL_REG,
			SYS_REG->SYSREG_PWR_CTRL, REG_SYSBUF_SRAM_SLEEP_B, 1);
	DSI_OUTREGBIT(cmdq, struct SYSREG_PWR_CTRL_REG,
			SYS_REG->SYSREG_PWR_CTRL, REG_GCE_SRAM_SLEEP_B, 1);

	reg = mtk_spi_read((unsigned long)(&SYS_REG->SYSREG_PWR_CTRL));
	reg = reg | 0x00880000;
	reg = reg & 0xffbbefff;
	DSI_OUTREG32(cmdq, SYS_REG->SYSREG_PWR_CTRL, reg);

	DISPFUNCEND();
}

void set_mtcmos_off(void *cmdq)
{
	unsigned int reg = 0;

	DISPFUNCSTART();

	reg = mtk_spi_read((unsigned long)(&SYS_REG->SYSREG_PWR_CTRL));
	reg = reg & 0x00000fff;
	reg = reg | 0x00661000;
	DSI_OUTREG32(cmdq, SYS_REG->SYSREG_PWR_CTRL, reg);

	reg = mtk_spi_read((unsigned long)(&SYS_REG->SYSREG_PWR_CTRL));
	reg = reg & 0xffddffff;
	DSI_OUTREG32(cmdq, SYS_REG->SYSREG_PWR_CTRL, reg);

	DSI_OUTREGBIT(cmdq, struct SYSREG_PWR_CTRL_REG,
			SYS_REG->SYSREG_PWR_CTRL, REG_DISP_PWR_CLK_DIS, 1);
	DSI_OUTREGBIT(cmdq, struct SYSREG_PWR_CTRL_REG,
			SYS_REG->SYSREG_PWR_CTRL, REG_PISO_EN, 1);
	DSI_OUTREGBIT(cmdq, struct SYSREG_RST_CTRL_REG,
			SYS_REG->SYSREG_RST_CTRL, REG_SW_RST_EN_DISP_PWR_WRAP, 1);
	DSI_OUTREGBIT(cmdq, struct SYSREG_PWR_CTRL_REG,
			SYS_REG->SYSREG_PWR_CTRL, REG_PWR_ON_2ND, 0);
	DSI_OUTREGBIT(cmdq, struct SYSREG_PWR_CTRL_REG,
			SYS_REG->SYSREG_PWR_CTRL, REG_PWR_ON, 0);
	DISPFUNCEND();
}

void set_pll_on(void *cmdq)
{
	unsigned int reg = 0;

	DISPFUNCSTART();

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->AP_PLL_CON3));
	reg = reg | 0x0000001e;
	DSI_OUTREG32(cmdq, APMIXEDSYS->AP_PLL_CON3, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->AP_PLL_CON5));
	reg = reg | 0x00016bf0;
	DSI_OUTREG32(cmdq, APMIXEDSYS->AP_PLL_CON5, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->AP_PLL_CON5));
	reg = reg & 0xfffdffff;
	DSI_OUTREG32(cmdq, APMIXEDSYS->AP_PLL_CON5, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->PLLON_CON0));
	reg = reg | 0x07ffffff;
	DSI_OUTREG32(cmdq, APMIXEDSYS->PLLON_CON0, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->PLLON_CON1));
	reg = reg | 0x07fffe00;
	DSI_OUTREG32(cmdq, APMIXEDSYS->PLLON_CON1, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->PLLON_CON1));
	reg = reg & 0xfffffe00;
	DSI_OUTREG32(cmdq, APMIXEDSYS->PLLON_CON1, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->MAINPLL_CON3));
	reg = reg | 0x00000001;
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON3, reg);

	udelay(1);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->MAINPLL_CON3));
	reg = reg & 0xfffffffd;
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON3, reg);

	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON1, 0x800fc000);

	udelay(1);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->MAINPLL_CON0));
	reg = reg | 0x00000001;
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON0, reg);

	udelay(20);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->MAINPLL_CON0));
	reg = reg | 0x00800001;
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON0, reg);

	DISPFUNCEND();
}

void set_pll_off(void *cmdq)
{
	unsigned int reg = 0;

	DISPFUNCSTART();

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->AP_PLL_CON3));
	reg = reg | 0x00000000;
	DSI_OUTREG32(cmdq, APMIXEDSYS->AP_PLL_CON3, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->AP_PLL_CON3));
	reg = reg & 0xffffffe1;
	DSI_OUTREG32(cmdq, APMIXEDSYS->AP_PLL_CON3, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->AP_PLL_CON5));
	reg = reg | 0x000020a0;
	DSI_OUTREG32(cmdq, APMIXEDSYS->AP_PLL_CON5, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->AP_PLL_CON5));
	reg = reg & 0xfffcb4af;
	DSI_OUTREG32(cmdq, APMIXEDSYS->AP_PLL_CON5, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->PLLON_CON0));
	reg = reg | 0x07ffffff;
	DSI_OUTREG32(cmdq, APMIXEDSYS->PLLON_CON0, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->PLLON_CON1));
	reg = reg | 0x07fffe00;
	DSI_OUTREG32(cmdq, APMIXEDSYS->PLLON_CON1, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->PLLON_CON1));
	reg = reg & 0xfffffe00;
	DSI_OUTREG32(cmdq, APMIXEDSYS->PLLON_CON1, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->MAINPLL_CON0));
	reg = reg | 0x00000000;
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON0, reg);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->MAINPLL_CON0));
	reg = reg & 0xff7ffffe;
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON0, reg);

	udelay(1);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->MAINPLL_CON3));
	reg = reg | 0x00000003;
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON3, reg);

	udelay(1);

	reg = mtk_spi_read((unsigned long)(&APMIXEDSYS->MAINPLL_CON3));
	reg = reg & 0xfffffffe;
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON3, reg);
}

void ana_macro_on(void *cmdq)
{
	unsigned int reg = 0;

	DISPFUNCSTART();
	//select pll power on `MAINPLL_CON3
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON3, 3);
	udelay(1);
//	DISPMSG("mt6382, %s, delay 1us\n", __func__);
	//select pll iso enable `MAINPLL_CON3
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON3, 1);
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON0, 0xff000000);
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON1, 0x000fc000);
	udelay(2);
//	DISPMSG("mt6382, %s, delay 2us\n", __func__);
	//reset setting `MAINPLL_CON0
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON0, 0xff000001);
	udelay(25);
//	DISPMSG("mt6382, %s, delay 25us\n", __func__);
	DSI_OUTREG32(cmdq, APMIXEDSYS->MAINPLL_CON0, 0xff800001);

	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_0_CLR, 0xffffffff);
	/* set clock source
	 * bit 16-17 is display mm clk 1(270m)/2(405m)/3(540m)
	 * dsc_on:vact * hact * vrefresh * (vtotal / vact) * bubble_ratio
	 */
#ifdef _90HZ_
	reg = (3 << 24) | (1 << 16) | (1 << 8) | (1 << 0); //270M for 90Hz
#else
	reg = (3 << 24) | (2 << 16) | (1 << 8) | (1 << 0); //405M for 120Hz
#endif
//	reg = (3 << 24) | (3 << 16) | (1 << 8) | (1 << 0); //540M
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_0_SET, reg);
	//config update
	reg = (1 << 4) | (1 << 3) | (1 << 1) | (1 << 0);
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_UPDATE, reg);

	udelay(6);
//	DISPMSG("mt6382, %s, delay 6us\n", __func__);
	DISPFUNCEND();
}

void ana_macro_off(void *cmdq)
{
	unsigned int reg = 0;

	DISPFUNCSTART();
//	ANA_MIPI_DSI_OFF ();
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_PHY_SHUTDOWNZ_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_PHY_RSTZ_OS, 0);

	//config update
	reg = (3 << 24) | (3 << 16) | (1 << 8) | (3 << 0);
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_0_CLR, reg);
	//config update
	reg = (1 << 4) | (1 << 3) | (1 << 1) | (1 << 0);
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_UPDATE, reg);

//	#5000ns; //wait clk mux stable
	udelay(6);
//	spis_low_speed_cfg();
//	DISPMSG("mt6382, %s, delay 6us\n", __func__);
	set_pll_off(cmdq);
}

void set_topck_on(void *cmdq)
{
	unsigned int reg = 0;

	DISPFUNCSTART();

	reg = mtk_spi_read((unsigned long)(&TOPCKGEN->CLK_MODE));
	reg = reg & 0xfffffffd;
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_MODE, reg);

	reg = mtk_spi_read((unsigned long)(&TOPCKGEN->CLK_CFG_0));
	reg = reg & 0xdf7f7f7f;
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_0, reg);
	DISPFUNCEND();
}

void set_topck_off(void *cmdq)
{
	unsigned int reg = 0;

	DISPFUNCSTART();
	//select clock source
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_0_CLR, 0x03030103);
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_0_SET, 0);
	//config update
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_UPDATE, 0x0000001b);

//	#5000ns; //wait clk mux stable
	udelay(6);
//	DISPMSG("mt6382, %s, delay 6us\n", __func__);
//	spis_low_speed_cfg();
//	set_pll_off(cmdq);

	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_0_CLR, 0x00808080);
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_0_SET, 0x00808080);
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_CFG_UPDATE, 0x0000001b);
	reg = mtk_spi_read((unsigned long)(&TOPCKGEN->CLK_MODE));
	reg = reg | 0x00000001;
	DSI_OUTREG32(cmdq, TOPCKGEN->CLK_MODE, reg);
}

void set_subsys_on(void *cmdq)
{
	unsigned int reg = 0;

	DISPFUNCSTART();

	reg = mtk_spi_read((unsigned long)(&GCE_REG->GCE_CTL_INT0));
	reg = reg & 0x0000ffff;
	DSI_OUTREG32(cmdq, GCE_REG->GCE_CTL_INT0, reg);

	//Clear CG
	DSI_OUTREG32(cmdq, DISPSYS_REG->MMSYS_CG_CON0, 0);
	//Turn off all DCM, Clock on
	reg = mtk_spi_read((unsigned long)(&EFUSE->DCM_ON));
	reg = reg & 0xfffffff8;
	DSI_OUTREG32(cmdq, EFUSE->DCM_ON, reg);

	reg = mtk_spi_read((unsigned long)(&GCE_REG->GCE_CTL_INT0));
	reg = reg | 0x0000ffff;
	DSI_OUTREG32(cmdq, GCE_REG->GCE_CTL_INT0, reg);

	DSI_OUTREG32(cmdq, DISPSYS_REG->MMSYS_HW_DCM_1ST_DIS0, 0xffffffff);
	DSI_OUTREG32(cmdq, DISPSYS_REG->MMSYS_HW_DCM_2ND_DIS0, 0xffffffff);

	// enable 26m clock
	DSI_OUTREG32(cmdq, SYS_REG->DISP_MISC1, 1);
	DISPFUNCEND();
}

void set_subsys_off(void *cmdq)
{
	unsigned int reg = 0;

	DISPFUNCSTART();

	reg = mtk_spi_read((unsigned long)(&GCE_REG->GCE_CTL_INT0));
	reg = reg | 0x00010000;
	DSI_OUTREG32(cmdq, GCE_REG->GCE_CTL_INT0, reg);

	reg = mtk_spi_read((unsigned long)(&GCE_REG->GCE_CTL_INT0));
	reg = reg & 0x0001ffff;
	DSI_OUTREG32(cmdq, GCE_REG->GCE_CTL_INT0, reg);

	//Set CG
	DSI_OUTREG32(cmdq, DISPSYS_REG->MMSYS_CG_CON0, 0xffffffff);
	//Turn on all DCM, Clock off
	reg = mtk_spi_read((unsigned long)(&EFUSE->DCM_ON));
	reg = reg | 0x00000007;
	DSI_OUTREG32(cmdq, EFUSE->DCM_ON, reg);

	reg = mtk_spi_read((unsigned long)(&GCE_REG->GCE_CTL_INT0));
	reg = reg | 0x0000ffff;
	DSI_OUTREG32(cmdq, GCE_REG->GCE_CTL_INT0, reg);

	DSI_OUTREG32(cmdq, DISPSYS_REG->MMSYS_HW_DCM_1ST_DIS0, 0);
	DSI_OUTREG32(cmdq, DISPSYS_REG->MMSYS_HW_DCM_2ND_DIS0, 0);
}

void set_ana_mipi_dsi_off(void *cmdq)
{
	unsigned int timeout = 5000;

	DISPFUNCSTART();

	//Tx mac enter ulps sequence:
	//Disable clock lane high speed mode
	DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_LCCON_REG,
		TX_REG[0]->DSI_TX_PHY_LCCON, LC_HS_TX_EN, 0); //[0]: lc_hstx_en

	//Disable all lane ultra-low power state mode
	DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_LD0CON_REG,
		TX_REG[0]->DSI_TX_PHY_LD0CON, L0_ULPM_EN, 0); //[0]: lc_hstx_en

	DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_LCCON_REG,
		TX_REG[0]->DSI_TX_PHY_LCCON, LC_ULPM_EN, 0); //[1]: lc_ulpm_en

	//Enable SLEEPIN_ULPS_INT_EN
	DSI_OUTREGBIT(cmdq, struct DSI_TX_INTEN_REG,
		TX_REG[0]->DSI_TX_INTEN, SLEEPIN_ULPS_INT_EN, 1); //[15]: sleepin_ulps_int_en


	//Enalbe all lane ultra-low power state mode
	DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_LD0CON_REG,
		TX_REG[0]->DSI_TX_PHY_LD0CON, LX_ULPM_AS_L0, 1); //[3]: lx_ulpm_as_l0
	DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_LD0CON_REG,
		TX_REG[0]->DSI_TX_PHY_LD0CON, L0_ULPM_EN, 1); //[1]: l0_ulpm_en
	DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_LCCON_REG,
		TX_REG[0]->DSI_TX_PHY_LCCON, LC_ULPM_EN, 1); //[1]: lc_ulpm_ens

	//Wait sleep in irq and clear irq state
	while ((mtk_spi_read((unsigned long)(&TX_REG[0]->DSI_TX_INTSTA)) & 0x8000) != 0x8000) {
		udelay(1);
		timeout--;

		if (timeout == 0) {
			DISPMSG("%s, wait timeout!\n", __func__);
			break;
		}
	}
	//disable SLEEPIN_ULPS_INT_EN
	DSI_OUTREGBIT(cmdq, struct DSI_TX_INTEN_REG,
		TX_REG[0]->DSI_TX_INTEN, SLEEPIN_ULPS_INT_EN, 0); //[15]: sleepin_ulps_int_en

	//mipi_dsi_tx_phy_sw_lp00_enable
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_PRE_OE_REG,
		MIPI_TX_REG->MIPI_TX_D2_SW_LPTX_PRE_OE, DSI_SW_LPTX_PRE_OE, 0); //[0]
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_OE_REG,
		MIPI_TX_REG->MIPI_TX_D2_SW_LPTX_OE, DSI_SW_LPTX_OE, 0); //[0]: dsi_d2_sw_lptx_oe
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_DP_REG,
		MIPI_TX_REG->MIPI_TX_D2_SW_LPTX_DP, DSI_SW_LPTX_DP, 0); //[0]: dsi_d2_sw_lptx_dp
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_DN_REG,
		MIPI_TX_REG->MIPI_TX_D2_SW_LPTX_DN, DSI_SW_LPTX_DN, 0); //[0]: dsi_d2_sw_lptx_dn

	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_PRE_OE_REG,
		MIPI_TX_REG->MIPI_TX_D0_SW_LPTX_PRE_OE, DSI_SW_LPTX_PRE_OE, 0); //[0]
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_OE_REG,
		MIPI_TX_REG->MIPI_TX_D0_SW_LPTX_OE, DSI_SW_LPTX_OE, 0); //[0]: dsi_d0_sw_lptx_oe
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_DP_REG,
		MIPI_TX_REG->MIPI_TX_D0_SW_LPTX_DP, DSI_SW_LPTX_DP, 0); //[0]: dsi_d0_sw_lptx_dp
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_DN_REG,
		MIPI_TX_REG->MIPI_TX_D0_SW_LPTX_DN, DSI_SW_LPTX_DN, 0); //[0]: dsi_d0_sw_lptx_dn

	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_PRE_OE_REG,
		MIPI_TX_REG->MIPI_TX_CK_SW_LPTX_PRE_OE, DSI_SW_LPTX_PRE_OE, 0); //[0]
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_OE_REG,
		MIPI_TX_REG->MIPI_TX_CK_SW_LPTX_OE, DSI_SW_LPTX_OE, 0); //[0]: dsi_ck_sw_lptx_oe
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_DP_REG,
		MIPI_TX_REG->MIPI_TX_CK_SW_LPTX_DP, DSI_SW_LPTX_DP, 0); //[0]: dsi_ck_sw_lptx_dp
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_DN_REG,
		MIPI_TX_REG->MIPI_TX_CK_SW_LPTX_DN, DSI_SW_LPTX_DN, 0); //[0]: dsi_ck_sw_lptx_dn

	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_PRE_OE_REG,
		MIPI_TX_REG->MIPI_TX_D1_SW_LPTX_PRE_OE, DSI_SW_LPTX_PRE_OE, 0); //[0]
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_OE_REG,
		MIPI_TX_REG->MIPI_TX_D1_SW_LPTX_OE, DSI_SW_LPTX_OE, 0); //[0]: dsi_d1_sw_lptx_oe
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_DP_REG,
		MIPI_TX_REG->MIPI_TX_D1_SW_LPTX_DP, DSI_SW_LPTX_DP, 0); //[0]: dsi_d1_sw_lptx_dp
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_DN_REG,
		MIPI_TX_REG->MIPI_TX_D1_SW_LPTX_DN, DSI_SW_LPTX_DN, 0); //[0]: dsi_d1_sw_lptx_dn

	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_PRE_OE_REG,
		MIPI_TX_REG->MIPI_TX_D3_SW_LPTX_PRE_OE, DSI_SW_LPTX_PRE_OE, 0); //[0]
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_OE_REG,
		MIPI_TX_REG->MIPI_TX_D3_SW_LPTX_OE, DSI_SW_LPTX_OE, 0); //[0]: dsi_d3_sw_lptx_oe
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_DP_REG,
		MIPI_TX_REG->MIPI_TX_D3_SW_LPTX_DP, DSI_SW_LPTX_DP, 0); //[0]: dsi_d3_sw_lptx_dp
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_LPTX_DN_REG,
		MIPI_TX_REG->MIPI_TX_D3_SW_LPTX_DN, DSI_SW_LPTX_DN, 0); //[0]: dsi_d3_sw_lptx_dn

	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
		MIPI_TX_REG->MIPI_TX_D2_SW_CTL_EN, DSI_SW_CTL_EN, 1); //[0]: dsi_d2_sw_ctl_en
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
		MIPI_TX_REG->MIPI_TX_D0_SW_CTL_EN, DSI_SW_CTL_EN, 1); //[0]: dsi_d0_sw_ctl_en
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
		MIPI_TX_REG->MIPI_TX_CK_SW_CTL_EN, DSI_SW_CTL_EN, 1); //[0]: dsi_ck_sw_ctl_en
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
		MIPI_TX_REG->MIPI_TX_D1_SW_CTL_EN, DSI_SW_CTL_EN, 1); //[0]: dsi_d1_sw_ctl_en
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
		MIPI_TX_REG->MIPI_TX_D3_SW_CTL_EN, DSI_SW_CTL_EN, 1); //[0]: dsi_d3_sw_ctl_en

	//Clear lane_num when enter ulps
	DSI_OUTREGBIT(cmdq, struct DSI_TX_TXRX_CON_REG,
		TX_REG[0]->DSI_TX_TXRX_CON, LANE_NUM, 0);

	//tx_phy_disalbe:
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_CON1_REG,
		MIPI_TX_REG->MIPI_TX_PLL_CON1, RG_DSI_PLL_EN, 0);

	//pll_sdm_iso_en = 1, pll_sdm_pwr_on = 0
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_PWR_REG,
		MIPI_TX_REG->MIPI_TX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN, 1); //[1]
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_PWR_REG,
		MIPI_TX_REG->MIPI_TX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON, 0); //[0]

	// TIEL_SEL=1
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_LANE_CON_REG,
		MIPI_TX_REG->MIPI_TX_LANE_CON, RG_DSI_PAD_TIEL_SEL, 1);

	// BG_LPF_EN=0
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_LANE_CON_REG,
		MIPI_TX_REG->MIPI_TX_LANE_CON, RG_DSI_BG_LPF_EN, 0);

	// BG_CORE_EN=0
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_LANE_CON_REG,
		MIPI_TX_REG->MIPI_TX_LANE_CON, RG_DSI_BG_CORE_EN, 0);

	DISPFUNCEND();
}

void bdg_clk_buf_nfc(bool onoff)
{
//	DISPFUNCSTART();

	if (onoff) {
		mtk_spi_write(0x000000a0, 0x00000022);
//		DSI_OUTREGBIT(cmdq, struct CKBUF_CTRL_REG,
//				SYS_REG->CKBUF_CTRL, NFC_CK_OUT_EN, 1);
	} else {
		mtk_spi_write(0x000000a0, 0x00000002);
//		DSI_OUTREGBIT(cmdq, struct CKBUF_CTRL_REG,
//				SYS_REG->CKBUF_CTRL, NFC_CK_OUT_EN, 0);
	}
//	DISPFUNCEND();
}

int dsi_get_pcw(int data_rate, int pcw_ratio)
{
	int pcw, tmp, pcw_floor;

	/**
	 * PCW bit 24~30 = floor(pcw)
	 * PCW bit 16~23 = (pcw - floor(pcw))*256
	 * PCW bit 8~15 = (pcw*256 - floor(pcw)*256)*256
	 * PCW bit 0~7 = (pcw*256*256 - floor(pcw)*256*256)*256
	 */
	pcw = data_rate * pcw_ratio / 26;
	pcw_floor = data_rate * pcw_ratio % 26;
	tmp = ((pcw & 0xFF) << 24) | (((256 * pcw_floor / 26) & 0xFF) << 16) |
		(((256 * (256 * pcw_floor % 26) / 26) & 0xFF) << 8) |
		((256 * (256 * (256 * pcw_floor % 26) % 26) / 26) & 0xFF);

	return tmp;
}

int bdg_mipi_tx_dphy_clk_setting(enum DISP_BDG_ENUM module,
				 void *cmdq,
				 struct LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;
	unsigned int j = 0;
	unsigned int data_Rate;
//	unsigned int pll_clock;
	unsigned int pcw_ratio = 0;
	unsigned int posdiv = 0;
	unsigned int prediv = 0;
	unsigned int delta1 = 2; /* Delta1 is SSC range, default is 0%~-5% */
	unsigned int pdelta1 = 0;
	enum MIPITX_PHY_LANE_SWAP *swap_base;
	enum MIPI_TX_PAD_VALUE pad_mapping[MIPITX_PHY_LANE_NUM] = {
					PAD_D0P, PAD_D1P, PAD_D2P,
					PAD_D3P, PAD_CKP, PAD_CKP};

	DISPFUNCSTART();

	data_Rate = bg_mipi_clk_change_sta == 0 ? tx_data_rate : tx_dyn_data_rate;

	DISPINFO("%s, data_Rate=%d\n",	__func__, data_Rate);

	/* DPHY SETTING */
	/* MIPITX lane swap setting */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/* step 0 MIPITX lane swap setting */
		swap_base = dsi_params->lane_swap[i];
		DISPINFO(
			"MIPITX Lane Swap mapping: 0=%d|1=%d|2=%d|3=%d|CK=%d|RX=%d\n",
			 swap_base[MIPITX_PHY_LANE_0],
			 swap_base[MIPITX_PHY_LANE_1],
			 swap_base[MIPITX_PHY_LANE_2],
			 swap_base[MIPITX_PHY_LANE_3],
			 swap_base[MIPITX_PHY_LANE_CK],
			 swap_base[MIPITX_PHY_LANE_RX]);

		if (unlikely(dsi_params->lane_swap_en)) {
			DISPINFO("MIPITX Lane Swap Enabled for DSI Port %d\n",
				 i);
			DISPINFO(
				"MIPITX Lane Swap mapping: %d|%d|%d|%d|%d|%d\n",
				 swap_base[MIPITX_PHY_LANE_0],
				 swap_base[MIPITX_PHY_LANE_1],
				 swap_base[MIPITX_PHY_LANE_2],
				 swap_base[MIPITX_PHY_LANE_3],
				 swap_base[MIPITX_PHY_LANE_CK],
				 swap_base[MIPITX_PHY_LANE_RX]);

			/* CKMODE_EN */
			for (j = MIPITX_PHY_LANE_0; j < MIPITX_PHY_LANE_CK;
			     j++) {
				if (dsi_params->lane_swap[i][j] ==
				    MIPITX_PHY_LANE_CK)
					break;
			}
			switch (j) {
			case MIPITX_PHY_LANE_0:
				DSI_OUTREGBIT(cmdq, struct MIPI_TX_CKMODE_EN_REG,
						MIPI_TX_REG->MIPI_TX_D0_CKMODE_EN,
						DSI_CKMODE_EN, 1);
				break;
			case MIPITX_PHY_LANE_1:
				DSI_OUTREGBIT(cmdq, struct MIPI_TX_CKMODE_EN_REG,
						MIPI_TX_REG->MIPI_TX_D1_CKMODE_EN,
						DSI_CKMODE_EN, 1);
				break;
			case MIPITX_PHY_LANE_2:
				DSI_OUTREGBIT(cmdq, struct MIPI_TX_CKMODE_EN_REG,
						MIPI_TX_REG->MIPI_TX_D2_CKMODE_EN,
						DSI_CKMODE_EN, 1);
				break;
			case MIPITX_PHY_LANE_3:
				DSI_OUTREGBIT(cmdq, struct MIPI_TX_CKMODE_EN_REG,
						MIPI_TX_REG->MIPI_TX_D3_CKMODE_EN,
						DSI_CKMODE_EN, 1);
				break;
			case MIPITX_PHY_LANE_CK:
				DSI_OUTREGBIT(cmdq, struct MIPI_TX_CKMODE_EN_REG,
						MIPI_TX_REG->MIPI_TX_CK_CKMODE_EN,
						DSI_CKMODE_EN, 1);
				break;
			default:
				break;
			}

			/* LANE_0 */
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL0_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL0,
					MIPI_TX_PHY0_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_0]]);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL0_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL0,
					MIPI_TX_PHY1AB_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_0]] + 1);

			/* LANE_1 */
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL0_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL0,
					MIPI_TX_PHY1_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_1]]);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL1_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL1,
					MIPI_TX_PHY2BC_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_1]] + 1);

			/* LANE_2 */
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL0_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL0,
					MIPI_TX_PHY2_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_2]]);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL0_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL0,
					MIPI_TX_CPHY0BC_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_2]] + 1);

			/* LANE_3 */
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL1_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL1,
					MIPI_TX_PHY3_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_3]]);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL1_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL1,
					MIPI_TX_CPHYXXX_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_3]] + 1);

			/* CK LANE */
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL0_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL0,
					MIPI_TX_PHYC_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_CK]]);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL0_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL0,
					MIPI_TX_CPHY1CA_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_CK]] + 1);

			/* LPRX SETTING */
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL1_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL1,
					MIPI_TX_LPRX0AB_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_RX]]);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL1_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL1,
					MIPI_TX_LPRX0BC_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_RX]] + 1);

			/* HS_DATA SETTING */
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL2_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL2,
					MIPI_TX_PHY2_HSDATA_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_2]]);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL2_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL2,
					MIPI_TX_PHY0_HSDATA_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_0]]);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL2_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL2,
					MIPI_TX_PHYC_HSDATA_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_CK]]);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL2_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL2,
					MIPI_TX_PHY1_HSDATA_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_1]]);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PHY_SEL3_REG,
					MIPI_TX_REG->MIPI_TX_PHY_SEL3,
					MIPI_TX_PHY3_HSDATA_SEL,
					pad_mapping[swap_base[MIPITX_PHY_LANE_3]]);
		} else {
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_CKMODE_EN_REG,
					MIPI_TX_REG->MIPI_TX_CK_CKMODE_EN,
					DSI_CKMODE_EN, 1);
		}
	}
//	refill_mipitx_impedance(NULL);

	/* MIPI INIT */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		unsigned int tmp = 0;

		/* step 0: RG_DSI0_PLL_IBIAS = 0*/
		DSI_OUTREG32(cmdq, MIPI_TX_REG->MIPI_TX_PLL_CON4, 0x00FF12E0);
		/* BG_LPF_EN / BG_CORE_EN */
		DSI_OUTREG32(cmdq, MIPI_TX_REG->MIPI_TX_LANE_CON, 0x3FFF0180);
		udelay(2);
		/* BG_LPF_EN=1,TIEL_SEL=0 */
		DSI_OUTREG32(cmdq, MIPI_TX_REG->MIPI_TX_LANE_CON, 0x3FFF0080);
//		if (atomic_read(&dsi_idle_flg) == 0) {
			/* Switch OFF each Lane */
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
					MIPI_TX_REG->MIPI_TX_D0_SW_CTL_EN,
					DSI_SW_CTL_EN, 0);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
					MIPI_TX_REG->MIPI_TX_D1_SW_CTL_EN,
					DSI_SW_CTL_EN, 0);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
					MIPI_TX_REG->MIPI_TX_D2_SW_CTL_EN,
					DSI_SW_CTL_EN, 0);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
					MIPI_TX_REG->MIPI_TX_D3_SW_CTL_EN,
					DSI_SW_CTL_EN, 0);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
					MIPI_TX_REG->MIPI_TX_CK_SW_CTL_EN,
					DSI_SW_CTL_EN, 0);
//		}

		/* step 1: SDM_RWR_ON / SDM_ISO_EN */
		DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_PWR_REG,
				MIPI_TX_REG->MIPI_TX_PLL_PWR,
				AD_DSI_PLL_SDM_PWR_ON, 1);
		udelay(2); /* 1us */
		DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_PWR_REG,
				MIPI_TX_REG->MIPI_TX_PLL_PWR,
				AD_DSI_PLL_SDM_ISO_EN, 0);

		if (data_Rate > 2500) {
			DISPINFO("mipitx Data Rate exceed limitation(%d)\n",
				    data_Rate);
			ASSERT(0);
			return -2;
		} else if (data_Rate >= 2000) { /* 2G ~ 2.5G */
			pcw_ratio = 1;
			posdiv    = 0;
			prediv    = 0;
		} else if (data_Rate >= 1000) { /* 1G ~ 2G */
			pcw_ratio = 2;
			posdiv    = 1;
			prediv    = 0;
		} else if (data_Rate >= 500) { /* 500M ~ 1G */
			pcw_ratio = 4;
			posdiv    = 2;
			prediv    = 0;
		} else if (data_Rate > 250) { /* 250M ~ 500M */
			pcw_ratio = 8;
			posdiv    = 3;
			prediv    = 0;
		} else if (data_Rate >= 125) { /* 125M ~ 250M */
			pcw_ratio = 16;
			posdiv    = 4;
			prediv    = 0;
		} else {
			DISPINFO("dataRate is too low(%d)\n", data_Rate);
			ASSERT(0);
			return -3;
		}

		/* step 3 */
		/* PLL PCW config */
		tmp = dsi_get_pcw(data_Rate, pcw_ratio);
		DSI_OUTREG32(cmdq, MIPI_TX_REG->MIPI_TX_PLL_CON0, tmp);
		DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_CON1_REG,
				MIPI_TX_REG->MIPI_TX_PLL_CON1,
				RG_DSI_PLL_POSDIV, posdiv);

		/* SSC config */
		if (dsi_params->bdg_ssc_disable != 1) {
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_CON2_REG,
					MIPI_TX_REG->MIPI_TX_PLL_CON2,
					RG_DSI_PLL_SDM_SSC_PH_INIT, 1);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_CON2_REG,
					MIPI_TX_REG->MIPI_TX_PLL_CON2,
					RG_DSI_PLL_SDM_SSC_PRD, 0x1B1);

			delta1 = (dsi_params->ssc_range == 0) ?
				delta1 : dsi_params->ssc_range;
			ASSERT(delta1 <= 8);
			pdelta1 = (delta1 * (data_Rate / 2) * pcw_ratio *
				   262144 + 281664) / 563329;

			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_CON3_REG,
					MIPI_TX_REG->MIPI_TX_PLL_CON3,
					RG_DSI_PLL_SDM_SSC_DELTA, pdelta1);
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_CON3_REG,
					MIPI_TX_REG->MIPI_TX_PLL_CON3,
					RG_DSI_PLL_SDM_SSC_DELTA1, pdelta1);
			DISPINFO(
				"%s, PLL config:data_rate=%d,pcw_ratio=%d, delta1=%d,pdelta1=0x%x\n",
				__func__, data_Rate, pcw_ratio, delta1, pdelta1);
		}
	}

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (data_Rate && (dsi_params->bdg_ssc_disable != 1))
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_CON2_REG,
					MIPI_TX_REG->MIPI_TX_PLL_CON2,
					RG_DSI_PLL_SDM_SSC_EN, 1);
		else
			DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_CON2_REG,
					MIPI_TX_REG->MIPI_TX_PLL_CON2,
					RG_DSI_PLL_SDM_SSC_EN, 0);
	}

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/* PLL EN */
		DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_CON1_REG,
				MIPI_TX_REG->MIPI_TX_PLL_CON1,
				RG_DSI_PLL_EN, 1);

		udelay(50);
		DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTRL_CON4_REG,
				MIPI_TX_REG->MIPI_TX_SW_CTRL_CON4,
				MIPI_TX_SW_ANA_CK_EN, 1);
	}

	DISPFUNCEND();
	return 0;
}

int bdg_tx_phy_config(enum DISP_BDG_ENUM module,
			void *cmdq, struct LCM_DSI_PARAMS *tx_params)
{
	int i;
	u32 ui, cycle_time;
	unsigned int hs_trail;
	unsigned int data_Rate;
//	unsigned char timcon_temp;

	DISPFUNCSTART();
	data_Rate = bg_mipi_clk_change_sta == 0 ? tx_data_rate : tx_dyn_data_rate;

	ui = 1000 / data_Rate;
	cycle_time = 8000 / data_Rate;

	DISPINFO(
		"%s, tx_data_rate=%d, cycle_time=%d, ui=%d\n",
		__func__, data_Rate, cycle_time, ui);

#if 1
	/* lpx >= 50ns (spec) */
	/* lpx = 60ns */
	timcon0.LPX = NS_TO_CYCLE(60, cycle_time);
	if (timcon0.LPX < 2)
		timcon0.LPX = 2;

	/* hs_prep = 40ns+4*UI ~ 85ns+6*UI (spec) */
	/* hs_prep = 64ns+5*UI */
	timcon0.HS_PRPR = NS_TO_CYCLE((64 + 5 * ui), cycle_time) + 1;

	/* hs_zero = (200+10*UI) - hs_prep */
	timcon0.HS_ZERO = NS_TO_CYCLE((200 + 10 * ui), cycle_time);
	timcon0.HS_ZERO = timcon0.HS_ZERO > timcon0.HS_PRPR ?
			timcon0.HS_ZERO - timcon0.HS_PRPR : timcon0.HS_ZERO;
	if (timcon0.HS_ZERO < 1)
		timcon0.HS_ZERO = 1;

	/* hs_trail > max(8*UI, 60ns+4*UI) (spec) */
	/* hs_trail = 80ns+4*UI */
	hs_trail = 80 + 4 * ui;
	timcon0.HS_TRAIL = (hs_trail > cycle_time) ?
				NS_TO_CYCLE(hs_trail, cycle_time) + 1 : 2;

	/* hs_exit > 100ns (spec) */
	/* hs_exit = 120ns */
	/* timcon1.DA_HS_EXIT = NS_TO_CYCLE(120, cycle_time); */
	/* hs_exit = 2*lpx */
	timcon1.DA_HS_EXIT = 2 * timcon0.LPX;

	/* ta_go = 4*lpx (spec) */
	timcon1.TA_GO = 4 * timcon0.LPX;

	/* ta_get = 5*lpx (spec) */
	timcon1.TA_GET = 5 * timcon0.LPX;

	/* ta_sure = lpx ~ 2*lpx (spec) */
	timcon1.TA_SURE = 3 * timcon0.LPX / 2;

	/* clk_hs_prep = 38ns ~ 95ns (spec) */
	/* clk_hs_prep = 80ns */
	timcon3.CLK_HS_PRPR = NS_TO_CYCLE(80, cycle_time);

	/* clk_zero + clk_hs_prep > 300ns (spec) */
	/* clk_zero = 400ns - clk_hs_prep */
	timcon2.CLK_ZERO = NS_TO_CYCLE(400, cycle_time) -
				timcon3.CLK_HS_PRPR;
	if (timcon2.CLK_ZERO < 1)
		timcon2.CLK_ZERO = 1;

	/* clk_trail > 60ns (spec) */
	/* clk_trail = 100ns */
	timcon2.CLK_TRAIL = NS_TO_CYCLE(100, cycle_time) + 1;
	if (timcon2.CLK_TRAIL < 2)
		timcon2.CLK_TRAIL = 2;

	/* clk_exit > 100ns (spec) */
	/* clk_exit = 200ns */
	/* timcon3.CLK_EXIT = NS_TO_CYCLE(200, cycle_time); */
	/* clk_exit = 2*lpx */
	timcon3.CLK_HS_EXIT = 2 * timcon0.LPX;

	/* clk_post > 60ns+52*UI (spec) */
	/* clk_post = 96ns+52*UI */
	timcon3.CLK_HS_POST = NS_TO_CYCLE((96 + 52 * ui), cycle_time);
#else
	hs_trail = (tx_params->HS_TRAIL == 0) ?
				(NS_TO_CYCLE(((1 * 4 * ui) + 80)
				* tx_params->PLL_CLOCK * 2, 8000) + 1) :
				tx_params->HS_TRAIL;
	/* +3 is recommended from designer becauase of HW latency */
	timcon0.HS_TRAIL = (hs_trail < 1) ? 1 : hs_trail;

	timcon0.HS_PRPR = (tx_params->HS_PRPR == 0) ?
			(NS_TO_CYCLE((64 + 5 * ui), cycle_time) + 1) :
			tx_params->HS_PRPR;
	/* HS_PRPR can't be 1. */
	if (timcon0.HS_PRPR < 1)
		timcon0.HS_PRPR = 1;

	timcon0.HS_ZERO = (tx_params->HS_ZERO == 0) ?
				NS_TO_CYCLE((200 + 10 * ui), cycle_time) :
				tx_params->HS_ZERO;
	timcon_temp = timcon0.HS_PRPR;
	if (timcon_temp < timcon0.HS_ZERO)
		timcon0.HS_ZERO -= timcon0.HS_PRPR;

	timcon0.LPX = (tx_params->LPX == 0) ?
		(NS_TO_CYCLE(tx_params->PLL_CLOCK * 2 * 75, 8000) + 1) :
								tx_params->LPX;
	if (timcon0.LPX < 1)
		timcon0.LPX = 1;

	timcon1.TA_GET = (tx_params->TA_GET == 0) ?  (5 * timcon0.LPX) :
							tx_params->TA_GET;
	timcon1.TA_SURE = (tx_params->TA_SURE == 0) ?
				(3 * timcon0.LPX / 2) : tx_params->TA_SURE;
	timcon1.TA_GO = (tx_params->TA_GO == 0) ?  (4 * timcon0.LPX) :
							tx_params->TA_GO;
	/* --------------------------------------------------------------
	 * NT35510 need fine tune timing
	 * Data_hs_exit = 60 ns + 128UI
	 * Clk_post = 60 ns + 128 UI.
	 * --------------------------------------------------------------
	 */
	timcon1.DA_HS_EXIT = (tx_params->DA_HS_EXIT == 0) ?
				(2 * timcon0.LPX) : tx_params->DA_HS_EXIT;

	timcon2.CLK_TRAIL = ((tx_params->CLK_TRAIL == 0) ?
				NS_TO_CYCLE(100 * tx_params->PLL_CLOCK * 2,
				8000) : tx_params->CLK_TRAIL) + 1;
	/* CLK_TRAIL can't be 1. */
	if (timcon2.CLK_TRAIL < 2)
		timcon2.CLK_TRAIL = 2;

//	timcon2.CONT_DET = tx_params->CONT_DET;
	timcon2.CLK_ZERO = (tx_params->CLK_ZERO == 0) ?
						NS_TO_CYCLE(400, cycle_time) :
						tx_params->CLK_ZERO;

	timcon3.CLK_HS_PRPR = (tx_params->CLK_HS_PRPR == 0) ?
				NS_TO_CYCLE(80 * tx_params->PLL_CLOCK * 2,
				8000) : tx_params->CLK_HS_PRPR;

	if (timcon3.CLK_HS_PRPR < 1)
		timcon3.CLK_HS_PRPR = 1;

	timcon3.CLK_HS_EXIT = (tx_params->CLK_HS_EXIT == 0) ?
				(2 * timcon0.LPX) : tx_params->CLK_HS_EXIT;
	timcon3.CLK_HS_POST = (tx_params->CLK_HS_POST == 0) ?
				NS_TO_CYCLE((96 + 52 * ui), cycle_time) :
				tx_params->CLK_HS_POST;

#endif
	bg_tx_data_phy_cycle = (timcon1.DA_HS_EXIT + 1) + timcon0.LPX +
					timcon0.HS_PRPR + timcon0.HS_ZERO + 1;

	DISPINFO(
		"%s, bg_tx_data_phy_cycle=%d, LPX=%d, HS_PRPR=%d, HS_ZERO=%d, HS_TRAIL=%d, DA_HS_EXIT=%d\n",
		__func__, bg_tx_data_phy_cycle, timcon0.LPX, timcon0.HS_PRPR,
		timcon0.HS_ZERO, timcon0.HS_TRAIL, timcon1.DA_HS_EXIT);

	DISPINFO(
		"%s, TA_GO=%d, TA_GET=%d, TA_SURE=%d, CLK_HS_PRPR=%d, CLK_ZERO=%d, CLK_TRAIL=%d, CLK_HS_EXIT=%d, CLK_HS_POST=%d\n",
		__func__, timcon1.TA_GO, timcon1.TA_GET, timcon1.TA_SURE,
		timcon3.CLK_HS_PRPR, timcon2.CLK_ZERO, timcon2.CLK_TRAIL,
		timcon3.CLK_HS_EXIT, timcon3.CLK_HS_POST);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON0_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON0, LPX,
				timcon0.LPX);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON0_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON0, HS_PRPR,
				timcon0.HS_PRPR);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON0_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON0, HS_ZERO,
				timcon0.HS_ZERO);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON0_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON0, HS_TRAIL,
				timcon0.HS_TRAIL);

		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON1_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON1, TA_GO,
				timcon1.TA_GO);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON1_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON1, TA_SURE,
				timcon1.TA_SURE);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON1_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON1, TA_GET,
				timcon1.TA_GET);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON1_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON1, DA_HS_EXIT,
				timcon1.DA_HS_EXIT);

		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON2_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON2, CLK_ZERO,
				timcon2.CLK_ZERO);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON2_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON2, CLK_TRAIL,
				timcon2.CLK_TRAIL);

		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON3_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON3, CLK_HS_PRPR,
				timcon3.CLK_HS_PRPR);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON3_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON3, CLK_HS_POST,
				timcon3.CLK_HS_POST);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_TIMCON3_REG,
				TX_REG[i]->DSI_TX_PHY_TIMECON3, CLK_HS_EXIT,
				timcon3.CLK_HS_EXIT);

		DISPINFO("%s, PHY_TIMECON0=0x%08x,PHY_TIMECON1=0x%08x\n",
			__func__,
			mtk_spi_read((unsigned long)(&TX_REG[i]->DSI_TX_PHY_TIMECON0)),
			mtk_spi_read((unsigned long)(&TX_REG[i]->DSI_TX_PHY_TIMECON1)));
		DISPINFO("%s, PHY_TIMECON2=0x%08x,PHY_TIMECON3=0x%08x\n",
			__func__,
			mtk_spi_read((unsigned long)(&TX_REG[i]->DSI_TX_PHY_TIMECON2)),
			mtk_spi_read((unsigned long)(&TX_REG[i]->DSI_TX_PHY_TIMECON3)));
	}


	return 0;
}

int bdg_tx_txrx_ctrl(enum DISP_BDG_ENUM module,
			void *cmdq, struct LCM_DSI_PARAMS *tx_params)
{
	int i;
	int lane_num = tx_params->LANE_NUM;
	bool hstx_cklp_en = tx_params->cont_clock ? FALSE : TRUE;
	bool dis_eotp_en = tx_params->IsCphy ? TRUE : FALSE;
	bool ext_te_en = tx_params->mode ? FALSE : TRUE;

	DISPFUNCSTART();

	switch (lane_num) {
	case ONE_LANE:
		lane_num = 0x1;
		break;
	case TWO_LANE:
		lane_num = 0x3;
		break;
	case THREE_LANE:
		lane_num = 0x7;
		break;
	case FOUR_LANE:
	default:
		lane_num = 0xF;
		break;
	}

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_TX_TXRX_CON_REG,
				TX_REG[i]->DSI_TX_TXRX_CON, LANE_NUM, lane_num);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_TXRX_CON_REG,
				TX_REG[i]->DSI_TX_TXRX_CON, HSTX_CKLP_EN,
				hstx_cklp_en);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_TXRX_CON_REG,
				TX_REG[i]->DSI_TX_TXRX_CON, DIS_EOT,
				dis_eotp_en);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_TXRX_CON_REG,
				TX_REG[i]->DSI_TX_TXRX_CON, EXT_TE_EN,
				ext_te_en);
		/* fpga mode */
		//DSI_OUTREG32(cmdq, &TX_REG[i]->DSI_PHY_LCPAT, 0x55);
	}

	return 0;
}

int bdg_tx_ps_ctrl(enum DISP_BDG_ENUM module,
			void *cmdq, struct LCM_DSI_PARAMS *tx_params)
{
	int i;
	unsigned int ps_wc, width, bpp, ps_sel;
#ifdef _LINE_BACK_TO_LP_
	unsigned int line_back_to_LP = 6;
#else
	unsigned int line_back_to_LP = 1;
#endif

	DISPFUNCSTART();

	width = tx_params->horizontal_active_pixel / 1;

	switch (tx_params->data_format.format) {
	case LCM_DSI_FORMAT_RGB565:
		ps_sel = PACKED_PS_16BIT_RGB565;
		bpp = 16;
		break;
	case LCM_DSI_FORMAT_RGB666_LOOSELY:
		ps_sel = LOOSELY_PS_24BIT_RGB666;
		bpp = 24;
		break;
	case LCM_DSI_FORMAT_RGB666:
		ps_sel = PACKED_PS_18BIT_RGB666;
		bpp = 18;
		break;
	case LCM_DSI_FORMAT_RGB888:
		ps_sel = PACKED_PS_24BIT_RGB888;
		bpp = 24;
		break;
	case LCM_DSI_FORMAT_RGB101010:
		ps_sel = PACKED_PS_30BIT_RGB101010;
		bpp = 30;
		break;
	default:
		DISPMSG("format not support!!!\n");
		return -1;
	}
	if (dsc_en) {
		ps_sel = PACKED_COMPRESSION;
		width = (width + 2) / 3;
	}
	ps_wc = (width * bpp) / 8;
	DISPINFO(
		"%s, DSI_WIDTH=%d, DSI_HEIGHT=%d, ps_sel=%d, ps_wc=%d, line_back_to_LP=%d\n",
		__func__, width, tx_params->vertical_active_line, ps_sel, ps_wc, line_back_to_LP);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_TX_VACT_NL_REG,
				TX_REG[i]->DSI_TX_VACT_NL, VACT_NL,
				tx_params->vertical_active_line / line_back_to_LP);

		DSI_OUTREGBIT(cmdq, struct DSI_TX_PSCON_REG,
				TX_REG[i]->DSI_TX_PSCON, CUSTOM_HEADER, 0);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PSCON_REG,
				TX_REG[i]->DSI_TX_PSCON, DSI_PS_WC, ps_wc * line_back_to_LP);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_PSCON_REG,
				TX_REG[i]->DSI_TX_PSCON, DSI_PS_SEL, ps_sel);

		DSI_OUTREGBIT(cmdq, struct DSI_TX_SIZE_CON_REG,
				TX_REG[i]->DSI_TX_SIZE_CON, DSI_WIDTH,
				width * line_back_to_LP);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_SIZE_CON_REG,
				TX_REG[i]->DSI_TX_SIZE_CON, DSI_HEIGHT,
				tx_params->vertical_active_line / line_back_to_LP);
	}

	return 0;
}

int bdg_tx_vdo_timing_set(enum DISP_BDG_ENUM module,
			void *cmdq, struct LCM_DSI_PARAMS *tx_params)
{
	int i;
	u32 dsi_buf_bpp, data_init_byte;

	DISPFUNCSTART();

	switch (tx_params->data_format.format) {
	case LCM_DSI_FORMAT_RGB565:
		dsi_buf_bpp = 16;
		break;
	case LCM_DSI_FORMAT_RGB666:
		dsi_buf_bpp = 18;
		break;
	case LCM_DSI_FORMAT_RGB666_LOOSELY:
	case LCM_DSI_FORMAT_RGB888:
		dsi_buf_bpp = 24;
		break;
	case LCM_DSI_FORMAT_RGB101010:
		dsi_buf_bpp = 30;
		break;
	default:
		DISPMSG("format not support!!!\n");
		return -1;
	}

	if (tx_params->IsCphy) {
		DISPMSG("C-PHY mode, need check!!!\n");
	} else {
		data_init_byte = bg_tx_data_phy_cycle * tx_params->LANE_NUM;

		switch (tx_params->mode) {
		case DSI_CMD_MODE:
			break;
		case DSI_SYNC_PULSE_VDO_MODE:
			hsa_byte = (((tx_params->horizontal_sync_active * dsi_buf_bpp)
					/ 8) - 10);
			hbp_byte = (((tx_params->horizontal_backporch * dsi_buf_bpp)
					/ 8) - 10);
			hfp_byte = (((tx_params->horizontal_frontporch * dsi_buf_bpp)
					/ 8) - 12);
			break;
		case DSI_SYNC_EVENT_VDO_MODE:
			hsa_byte = 0;	/* don't care */
			hbp_byte = (((tx_params->horizontal_backporch +
					tx_params->horizontal_sync_active) *
					dsi_buf_bpp) / 8) - 10;
			hfp_byte = (((tx_params->horizontal_frontporch * dsi_buf_bpp) / 8)
					- 12);
			break;
		case DSI_BURST_VDO_MODE:
			hsa_byte = 0;	/* don't care */
			hbp_byte = (((tx_params->horizontal_backporch +
					tx_params->horizontal_sync_active) *
					dsi_buf_bpp) / 8) - 10;
			hfp_byte = (((tx_params->horizontal_frontporch * dsi_buf_bpp) / 8)
					- 12 - 6);
			break;
		}

		bllp_byte = 16 * tx_params->LANE_NUM;
	}

	if (hsa_byte < 0) {
		DISPMSG("error!hsa = %d < 0!\n", hsa_byte);
		hsa_byte = 0;
//		return -1;
	}

	if (hfp_byte > data_init_byte) {
		hfp_byte -= data_init_byte;
	} else {
		hfp_byte = 4;
		DISPMSG("hfp is too short!\n");
//		return -2;
	}

	DISPINFO(
		"%s, mode=0x%x, t_vsa=%d, t_vbp=%d, t_vfp=%d, hsa_byte=%d, hbp_byte=%d, hfp_byte=%d, bllp_byte=%d\n",
		__func__, tx_params->mode, tx_params->vertical_sync_active,
		tx_params->vertical_backporch, tx_params->vertical_frontporch,
		hsa_byte, hbp_byte, hfp_byte, bllp_byte);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_VSA_NL,
					tx_params->vertical_sync_active);
		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_VBP_NL,
					(tx_params->vertical_backporch));
		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_VFP_NL,
					(tx_params->vertical_frontporch));

		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_HSA_WC, hsa_byte);
		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_HBP_WC, hbp_byte);
		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_HFP_WC, hfp_byte);
		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_BLLP_WC, bllp_byte);
	}

	return 0;
}

int bdg_tx_buf_rw_set(enum DISP_BDG_ENUM module,
			void *cmdq, struct LCM_DSI_PARAMS *tx_params)
{
	int i;
	unsigned int width, height, rw_times, tmp;

	DISPFUNCSTART();

	tmp = (mtk_spi_read((unsigned long)
			(&TX_REG[0]->DSI_TX_COM_CON)) & 0x01000000) >> 24;
	width = tx_params->horizontal_active_pixel / 1;
	height = tx_params->vertical_active_line;

	if (dsc_en)
		width = (width + 2) / 3;

	if (tmp == 1 && tx_params->mode == 0) {
		if ((width * 3 % 9) == 0)
			rw_times = (width * 3 / 9) * height;
		else
			rw_times = (width * 3 / 9 + 1) * height;
	} else {
		if ((width * height * 3 % 9) == 0)
			rw_times = width * height * 3 / 9;
		else
			rw_times = width * height * 3 / 9 + 1;
	}

	DISPINFO(
		"%s, mode=0x%x, tmp=%d, width=%d, height=%d, rw_times=%d\n",
		__func__, tx_params->mode, tmp, width, height, rw_times);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_BUF_RW_TIMES, rw_times);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_BUF_CON0_REG,
			TX_REG[i]->DSI_TX_BUF_CON0, ANTI_LATENCY_BUF_EN, 1);

		if (tx_params->mode == CMD_MODE) {
#ifdef _CMD_120FPS_
#if 0
			if (bg_mipi_clk_change_sta)
				DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_BUF_CON1, 0x0dfd0745);
			else
				DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_BUF_CON1, 0x0dfd0026);
#else
			DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_BUF_CON1, 0x0dfd0a00);
#endif
#else
			DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_BUF_CON1, 0x0dfd0300);
#endif
		}
	}

	return 0;
}

int bdg_tx_enable_hs_clk(enum DISP_BDG_ENUM module,
				void *cmdq, bool enable)
{
	int i;

	DISPFUNCSTART();

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enable) {
			DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_LCCON_REG,
				TX_REG[i]->DSI_TX_PHY_LCCON, LC_HS_TX_EN, 1);
		} else {
			DSI_OUTREGBIT(cmdq, struct DSI_TX_PHY_LCCON_REG,
				TX_REG[i]->DSI_TX_PHY_LCCON, LC_HS_TX_EN, 0);
		}
	}
	return 0;
}

#if 0
int dsi_set_fps(lcm_dsi_params *dsi_params, enum dsi_fps_enum fps)
{
	int i;
	u32 dsi_buf_bpp, hactive;
	u32 line_time, frame_time;

	DISPFUNCSTART();

	if ((fps > MTK_DSI_FPS_90) || (fps == 0))
		return -DSI_STATUS_ERROR;

	hactive = dsi_params->hactive_pixel / 1;

	dsi_buf_bpp = 24;

	line_time =
	    (hactive + dsi_params->hsync_active + dsi_params->hback_porch +
	     dsi_params->hfront_porch) * dsi_buf_bpp;
	frame_time =
	    line_time * (dsi_params->vactive_line + dsi_params->vsync_active +
			 dsi_params->vback_porch + dsi_params->vfront_porch);

	dsi_params->data_rate =
	    ((frame_time / dsi_params->lane_num) * ((u32)fps) + 999) / 1000;

	DDPDUMP("fps=%d, lanes=%d, data rate=%d.%03d MHz\n",
		fps, dsi_params->lane_num, dsi_params->data_rate / 1000,
		dsi_params->data_rate % 1000);

	return 0;
}
#endif

int bdg_tx_set_mode(enum DISP_BDG_ENUM module,
				void *cmdq, unsigned int mode)
{
	int i = 0;

	DISPINFO("%s, mode=%d\n", __func__, mode);
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_TX_MODE_CON_REG,
				TX_REG[i]->DSI_TX_MODE_CON, MODE, mode);
	}
	if (mode == CMD_MODE)
		DSI_OUTREGBIT(cmdq, struct MIPI_RX_POST_CTRL_REG,
			DISPSYS_REG->MIPI_RX_POST_CTRL, MIPI_RX_MODE_SEL, 0);
	else
		DSI_OUTREGBIT(cmdq, struct MIPI_RX_POST_CTRL_REG,
			DISPSYS_REG->MIPI_RX_POST_CTRL, MIPI_RX_MODE_SEL, 1);

	return 0;
}

int bdg_tx_bist_pattern(enum DISP_BDG_ENUM module,
				void *cmdq, bool enable, unsigned int sel,
				unsigned int red, unsigned int green,
				unsigned int blue)
{
	int i;

	DISPFUNCSTART();

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enable) {
			DSI_OUTREGBIT(cmdq, struct DSI_TX_SELF_PAT_CON0_REG,
					TX_REG[i]->DSI_TX_SELF_PAT_CON0,
					SELF_PAT_R, (red & 0xff) << 4);
			DSI_OUTREGBIT(cmdq, struct DSI_TX_SELF_PAT_CON1_REG,
					TX_REG[i]->DSI_TX_SELF_PAT_CON1,
					SELF_PAT_G, (green & 0xff) << 4);
			DSI_OUTREGBIT(cmdq, struct DSI_TX_SELF_PAT_CON1_REG,
					TX_REG[i]->DSI_TX_SELF_PAT_CON1,
					SELF_PAT_B, (blue & 0xff) << 4);
			DSI_OUTREGBIT(cmdq, struct DSI_TX_SELF_PAT_CON0_REG,
					TX_REG[i]->DSI_TX_SELF_PAT_CON0,
					SELF_PAT_SEL, sel);
			DSI_OUTREGBIT(cmdq, struct DSI_TX_SELF_PAT_CON0_REG,
					TX_REG[i]->DSI_TX_SELF_PAT_CON0,
					SELF_PAT_PRE_MODE, 1);
		} else {
			DSI_OUTREGBIT(cmdq, struct DSI_TX_SELF_PAT_CON0_REG,
					TX_REG[i]->DSI_TX_SELF_PAT_CON0,
					SELF_PAT_PRE_MODE, 0);
		}

	}

	return 0;
}

int bdg_tx_start(enum DISP_BDG_ENUM module, void *cmdq)
{
	int i;

	DISPFUNCSTART();

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_TX_START_REG,
				TX_REG[i]->DSI_TX_START, DSI_TX_START, 0);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_START_REG,
				TX_REG[i]->DSI_TX_START, DSI_TX_START, 1);
	}

	return 0;
}

int bdg_set_dcs_read_cmd(bool enable, void *cmdq)
{
	DISPFUNCSTART();

	if (enable) {
		DSI_OUTREGBIT(cmdq, struct DSI_TX_RACK_REG,
				TX_REG[0]->DSI_TX_RACK, DSI_TX_RACK_BYPASS, 1);
		DSI_OUTREGBIT(cmdq, struct MIPI_RX_POST_CTRL_REG,
				DISPSYS_REG->MIPI_RX_POST_CTRL, MIPI_RX_MODE_SEL, 0);
	} else {
		DSI_OUTREGBIT(cmdq, struct DSI_TX_RACK_REG,
				TX_REG[0]->DSI_TX_RACK, DSI_TX_RACK_BYPASS, 0);
		DSI_OUTREGBIT(cmdq, struct MIPI_RX_POST_CTRL_REG,
				DISPSYS_REG->MIPI_RX_POST_CTRL, MIPI_RX_MODE_SEL, 1);
	}

	return 0;
}

int bdg_tx_stop(enum DISP_BDG_ENUM module, void *cmdq)
{
	int i;

	DISPFUNCSTART();

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		DSI_OUTREGBIT(cmdq, struct DSI_TX_START_REG,
				TX_REG[i]->DSI_TX_START, DSI_TX_START, 0);

	return 0;
}

int bdg_tx_reset(enum DISP_BDG_ENUM module, void *cmdq)
{
	int i;

	DISPFUNCSTART();

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_TX_COM_CON_REG,
				TX_REG[i]->DSI_TX_COM_CON, DSI_RESET, 1);
		DSI_OUTREGBIT(cmdq, struct DSI_TX_COM_CON_REG,
				TX_REG[i]->DSI_TX_COM_CON, DSI_RESET, 0);
	}

	return 0;
}

int bdg_vm_mode_set(enum DISP_BDG_ENUM module, bool enable,
			unsigned int long_pkt, void *cmdq)
{
	int i;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enable) {
			DSI_OUTREG32(cmdq, TX_REG[i]->DSI_VM_CMD_CON1, 0x10);
			DSI_OUTREG32(cmdq, DISPSYS_REG->DDI_POST_CTRL, 0x00001110);
			if (long_pkt > 1)
				DSI_OUTREG32(cmdq, TX_REG[i]->DSI_VM_CMD_CON0, 0x37);
			else
				DSI_OUTREG32(cmdq, TX_REG[i]->DSI_VM_CMD_CON0, 0x35);

		} else {
			DSI_OUTREG32(cmdq, TX_REG[i]->DSI_VM_CMD_CON0, 0);
			DSI_OUTREG32(cmdq, DISPSYS_REG->DDI_POST_CTRL, 0x00001100);
		}
	}
	return 0;
}

int bdg_tx_cmd_mode(enum DISP_BDG_ENUM module, void *cmdq)
{
	int i;

	DISPFUNCSTART();

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_MEM_CONTI, 0x3c);
		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_CMDQ, 0x2c3909);
		DSI_OUTREG32(cmdq, TX_REG[i]->DSI_TX_CMDQ_CON, 1);
	}

	return 0;
}

int bdg_mutex_trigger(enum DISP_BDG_ENUM module, void *cmdq)
{
	DISPFUNCSTART();

	DSI_OUTREGBIT(cmdq, struct DISP_MUTEX0_EN_REG,
			MUTEX_REG->DISP_MUTEX0_EN, MUTEX0_EN, 0);
	DSI_OUTREGBIT(cmdq, struct DISP_MUTEX0_EN_REG,
			MUTEX_REG->DISP_MUTEX0_EN, MUTEX0_EN, 1);

	return 0;
}

int bdg_dsi_dump_reg(enum DISP_BDG_ENUM module, unsigned int level)
{
	unsigned int i, k;
	unsigned int tmp;

#if 0
	DISPMSG("0x%08x: 0x%08x\n", 0x0000d314, mtk_spi_read(0x0000d314));
	DISPMSG("0x%08x: 0x%08x\n", 0x00007310, mtk_spi_read(0x00007310));
	DISPMSG("0x%08x: 0x%08x\n", 0x000231a8, mtk_spi_read(0x000231a8));
	DISPMSG("0x%08x: 0x%08x\n", 0x00023174, mtk_spi_read(0x00023174));
	DISPMSG("0x%08x: 0x%08x\n", 0x0002106c, mtk_spi_read(0x0002106c));
	DISPMSG("0x%08x: 0x%08x\n", 0x00021300, mtk_spi_read(0x00021300));
	DISPMSG("0x%08x: 0x%08x\n", 0x00003010, mtk_spi_read(0x00003010));
#endif
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		unsigned long dsc_base_addr = (unsigned long)DSC_REG;
		unsigned long dsi_base_addr = (unsigned long)TX_REG[i];
		unsigned long mipi_base_addr = (unsigned long)MIPI_TX_REG;

		DISPMSG("========================== mt6382 RX REGS ==\n", i);
		tmp = mtk_spi_read(0x0000d00c);
		DISPMSG("INT_ST_MAIN(0x0000d00c): 0x%08x\n", tmp);
		if (tmp != 0) {
			if (tmp & (1 << 0))
				DISPMSG("INT_ST_MAIN(bit0), int_st_phy_fatal\n");
			if (tmp & (1 << 1))
				DISPMSG("INT_ST_MAIN(bit1), int_st_dsi_fatal\n");
			if (tmp & (1 << 2))
				DISPMSG("INT_ST_MAIN(bit2), int_st_ddi_fatal\n");
			if (tmp & (1 << 3))
				DISPMSG("INT_ST_MAIN(bit3), int_st_ipi_fatal\n");
			if (tmp & (1 << 4))
				DISPMSG("INT_ST_MAIN(bit4), int_st_fifo_fatal\n");
			if (tmp & (1 << 16))
				DISPMSG("INT_ST_MAIN(bit16), int_st_phy\n");
			if (tmp & (1 << 17))
				DISPMSG("INT_ST_MAIN(bit17), int_st_dsi\n");
			if (tmp & (1 << 18))
				DISPMSG("INT_ST_MAIN(bit18), int_st_ddi\n");
			if (tmp & (1 << 19))
				DISPMSG("INT_ST_MAIN(bit19), int_st_ipi\n");
			if (tmp & (1 << 21))
				DISPMSG("INT_ST_MAIN(bit21), int_st_err_rpt\n");
			if (tmp & (1 << 22))
				DISPMSG("INT_ST_MAIN(bit22), int_st_rx_triggers\n");
		}

		tmp = mtk_spi_read(0x0000d200);
		if (tmp != 0) {
			DISPMSG("INT_ST_PHY_FATAL(0x0000d200): 0x%08x\n", tmp);
			if (tmp & (1 << 0))
				DISPMSG("Lane 0 Start-of-Transmission Synchronization Error.\n");
			if (tmp & (1 << 1))
				DISPMSG("Lane 0 Escape Entry Error.\n");
			if (tmp & (1 << 2))
				DISPMSG("Lane 0 Control Error.\n");
			if (tmp & (1 << 3))
				DISPMSG("Lane 0 LP Data Transmission Synchronization Error.\n");
			if (tmp & (1 << 4))
				DISPMSG("Lane 0 LP0 Contention Error.\n");
			if (tmp & (1 << 5))
				DISPMSG("Lane 0 LP1 Contention Error.\n");
			if (tmp & (1 << 7)) {
				DISPMSG("Peripheral LPTX timeout defined in DSI ");
				DISPMSG("protocol, avoid to bus contention.\n");
			}
			if (tmp & (1 << 8))
				DISPMSG("Lane 1 Start-of-Transmission Synchronization Error.\n");
			if (tmp & (1 << 9))
				DISPMSG("Lane 1 Escape Entry Error.\n");
			if (tmp & (1 << 10))
				DISPMSG("Lane 1 Control Error.\n");
			if (tmp & (1 << 11)) {
				DISPMSG("Skew between lanes is greater than ");
				DISPMSG("maximum allowed value.\n");
			}
			if (tmp & (1 << 16))
				DISPMSG("Lane 2 Start-of-Transmission Synchronization Error.\n");
			if (tmp & (1 << 17))
				DISPMSG("Lane 2 Escape Entry Error.\n");
			if (tmp & (1 << 18))
				DISPMSG("Lane 2 Control Error.\n");
			if (tmp & (1 << 24))
				DISPMSG("Lane 3 Start-of-Transmission Synchronization Error.\n");
		}

		tmp = mtk_spi_read(0x0000d220);
		if (tmp != 0) {
			DISPMSG("INT_ST_DSI_FATAL(0x0000d220): 0x%08x\n", tmp);
			if (tmp & (1 << 0))
				DISPMSG("Payload Checksum error detected on VC 0.\n");
			if (tmp & (1 << 1))
				DISPMSG("Payload Checksum error detected on VC 1.\n");
			if (tmp & (1 << 2))
				DISPMSG("Payload Checksum error detected on VC 2.\n");
			if (tmp & (1 << 3))
				DISPMSG("Payload Checksum error detected on VC 3.\n");
			if (tmp & (1 << 4)) {
				DISPMSG("LPRX only. Payload size Mismatch error ");
				DISPMSG("detected on VC 0. Causes Payload Checksum error.\n");
			}
			if (tmp & (1 << 5)) {
				DISPMSG("LPRX only. Payload size Mismatch error ");
				DISPMSG("detected on VC 1. Causes Payload Checksum error.\n");
			}
			if (tmp & (1 << 6)) {
				DISPMSG("LPRX only. Payload size Mismatch error ");
				DISPMSG("detected on VC 2. Causes Payload Checksum error.\n");
			}
			if (tmp & (1 << 7)) {
				DISPMSG("LPRX only. Payload size Mismatch error ");
				DISPMSG("detected on VC 3. Causes Payload Checksum error.\n");
			}
			if (tmp & (1 << 16))
				DISPMSG("Header error unrecoverable.\n");
			if (tmp & (1 << 17)) {
				DISPMSG("Read or TE command is followed by another read ");
				DISPMSG("or write command instead of BTA.\n");
			}
			if (tmp & (1 << 18))
				DISPMSG("Invalid data type detected.\n");
		}

		tmp = mtk_spi_read(0x0000d240);
		DISPMSG("INT_ST_DDI_FATAL(0x0000d240): 0x%08x\n", tmp);
		if (tmp & (1 << 0)) {
			DISPMSG("This timeout should rise when the DSI Display panel is not ");
			DISPMSG("outputting response data during programmed value.\n");
			DISPMSG("Data received after this timeout are ignored and ");
			DISPMSG("related logic will be reset.\n");
		}
		if (tmp & (1 << 1))
			DISPMSG("Reports error when panel responds an invalid data type.\n");

		tmp = mtk_spi_read(0x0000d260);
		DISPMSG("INT_ST_IPI_FATAL(0x0000d260): 0x%08x\n", tmp);
		if (tmp & (1 << 0)) {
			DISPMSG("This bit asserts when receiving error IPI timing information, ");
			DISPMSG("e.g. no HSE after HSS in pulse mode.\n");
		}
		if (tmp & (1 << 1)) {
			DISPMSG("This bit asserts when receiving non-integer pixel ");
			DISPMSG("number video packet.\n");
		}
		tmp = mtk_spi_read(0x0000d280);
		if (tmp != 0) {
			DISPMSG("INT_ST_FIFO_FATAL(0x0000d280): 0x%08x\n", tmp);
			if (tmp & (1 << 0))
				DISPMSG("LPRX FIFO overflow, caused by LPRX from PHY.\n");
			if (tmp & (1 << 1))
				DISPMSG("LPTX FIFO overflow, caused by LPTX from DDI.\n");
			if (tmp & (1 << 4))
				DISPMSG("D-PHY HSRX elastbuf overflow, caused by D-PHY HSRX.\n");
			if (tmp & (1 << 8))
				DISPMSG("DDI Header FIFO overflow, caused by HSRX from PPI.\n");
			if (tmp & (1 << 9))
				DISPMSG("DDI Payload FIFO overflow, caused by HSRX from PPI.\n");
			if (tmp & (1 << 16))
				DISPMSG("IPI Control FIFO overflow, caused by HSRX from PPI.\n");
			if (tmp & (1 << 17))
				DISPMSG("IPI Pixel FIFO overflow, caused by HSRX from PPI.\n");
			if (tmp & (1 << 18))
				DISPMSG("IPI Pixel FIFO overflow, caused by read from IPI.\n");
		}

		tmp = mtk_spi_read(0x0000d210);
		if (tmp != 0) {
			DISPMSG("INT_ST_PHY(0x0000d210): 0x%08x\n", tmp);
			if (tmp & (1 << 0))
				DISPMSG("Lane 0 Start-of-Transmission(SoT) Error.\n");
			if (tmp & (1 << 1))
				DISPMSG("Lane 1 Start-of-Transmission(SoT) Error.\n");
			if (tmp & (1 << 2))
				DISPMSG("Lane 2 Start-of-Transmission(SoT) Error.\n");
			if (tmp & (1 << 3))
				DISPMSG("Lane 3 Start-of-Transmission(SoT) Error.\n");
			if (tmp & (1 << 4))
				DISPMSG("HS Invalid Code Word Detection on lane 0.\n");
			if (tmp & (1 << 5))
				DISPMSG("HS Invalid Code Word Detection on lane 1.\n");
			if (tmp & (1 << 6))
				DISPMSG("HS Invalid Code Word Detection on lane 2.\n");
			if (tmp & (1 << 16)) {
				DISPMSG("This timeout is to inform the user when the LPDT ");
				DISPMSG(
					"transmission duration is more than the programmed time.\n");
			}
			if (tmp & (1 << 17)) {
				DISPMSG("This timeout is to inform the user when the high-speed ");
				DISPMSG("reception duration is more than the programmed time.\n");
			}
		}

		tmp = mtk_spi_read(0x0000d230);
		if (tmp != 0) {
			DISPMSG("INT_ST_DSI(0x0000d230): 0x%08x\n", tmp);
			if (tmp & (1 << 0))
				DISPMSG("Header ECC Check error detected on VC 0.\n");
			if (tmp & (1 << 1))
				DISPMSG("Header ECC Check error detected on VC 1.\n");
			if (tmp & (1 << 2))
				DISPMSG("Header ECC Check error detected on VC 2.\n");
			if (tmp & (1 << 3))
				DISPMSG("Header ECC Check error detected on VC 3.\n");
			if (tmp & (1 << 4)) {
				DISPMSG("EoT packet detected error, according the configuration ");
				DISPMSG("EOTP_CFG.\n");
			}
		}
		tmp = mtk_spi_read(0x0000d250);
		if (tmp != 0) {
			DISPMSG("INT_ST_DDI(0x0000d250): 0x%08x\n", tmp);
			if (tmp & (1 << 0)) {
				DISPMSG("This timeout should rise when the DSI Display panel is ");
				DISPMSG(
					"not accepting the incoming data during programmed value.\n");
			}
			if (tmp & (1 << 1)) {
				DISPMSG("This field reports when DSI2 host sends an invalid ");
				DISPMSG("virtual channel for DDI command packets.\n");
			}
			if (tmp & (1 << 4)) {
				DISPMSG("This field reports when received response packet number ");
				DISPMSG("mismatch with the value in WC.\n");
			}
			if (tmp & (1 << 5))
				DISPMSG("This field reports when panel responds an invalid VC.\n");
			if (tmp & (1 << 6)) {
				DISPMSG("This field reports when panel responds data after ");
				DISPMSG("response timeout is fired.\n");
			}
		}

		tmp = mtk_spi_read(0x0000d270);
		DISPMSG("INT_ST_IPI(0x0000d270): 0x%08x\n", tmp);
		if (tmp & (1 << 0)) {
			DISPMSG("This bit asserts when receiving invalid virtual channel packets ");
			DISPMSG("according the configuration IPI_VALID_VC_CFG.\n");
		}
		if (tmp & (1 << 1)) {
			DISPMSG("This bit asserts when frame timing information is corrupted ");
			DISPMSG("because of back pressure.\n");
		}
		tmp = mtk_spi_read(0x0000d2b0);
		if (tmp != 0) {
			DISPMSG("INT_ST_ERR_RPT(0x0000d2b0): 0x%08x\n", tmp);
			if (tmp & (1 << 0))
				DISPMSG("SOT Error.\n");
			if (tmp & (1 << 1))
				DISPMSG("SOT Sync Error.\n");
			if (tmp & (1 << 2))
				DISPMSG("EOT Error.\n");
			if (tmp & (1 << 3))
				DISPMSG("Escape mode Entry Command Error.\n");
			if (tmp & (1 << 4))
				DISPMSG("Low-power Transmit Sync Error.\n");
			if (tmp & (1 << 5))
				DISPMSG("Peripheral Timeout Error.\n");
			if (tmp & (1 << 6))
				DISPMSG("False Control Error.\n");
			if (tmp & (1 << 7))
				DISPMSG("Contention detected.\n");
			if (tmp & (1 << 8))
				DISPMSG("ECC Error, single-bit(detected and corrected).\n");
			if (tmp & (1 << 9))
				DISPMSG("ECC Error, multi-bit(detected and not corrected).\n");
			if (tmp & (1 << 10))
				DISPMSG("Payload Checksum Error(long packet only).\n");
			if (tmp & (1 << 11))
				DISPMSG("DSI Data Type Not Recognized.\n");
			if (tmp & (1 << 12)) {
				DISPMSG("DSI VC ID invalid, both DDI_VALID_VC_CFG & ");
				DISPMSG("IPI_VALID_VC_CFG related error will assert this bit.\n");
			}
			if (tmp & (1 << 13))
				DISPMSG("Invalid Transmission Length.\n");
			if (tmp & (1 << 15))
				DISPMSG("DSI Protocol Violation.\n");
			if (tmp & (1 << 16))
				DISPMSG("High-speed Invalid Code Word Detection on lane.\n");
		}

		tmp = mtk_spi_read(0x0000d2d0);
		if (tmp != 0) {
			DISPMSG("INT_ST_RX_TRIGGERS(0x0000d2d0): 0x%08x\n", tmp);
			if (tmp & (1 << 0))
				DISPMSG("Trigger 0\n");
			if (tmp & (1 << 1))
				DISPMSG("Trigger 1\n");
			if (tmp & (1 << 2))
				DISPMSG("Trigger 2\n");
			if (tmp & (1 << 3))
				DISPMSG("Trigger 3\n");
		}

		DISPMSG("========================== mt6382 DSI%d REGS ==\n", i);
//		for (k = 0; k < sizeof(struct BDG_TX_REGS); k += 16) {
		for (k = 0; k < 0x210; k += 16) {
			DISPMSG("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x\n", k,
				mtk_spi_read(dsi_base_addr + k),
				mtk_spi_read(dsi_base_addr + k + 0x4),
				mtk_spi_read(dsi_base_addr + k + 0x8),
				mtk_spi_read(dsi_base_addr + k + 0xc));
		}

		DISPMSG("========================== mt6382 DSI%d CMD REGS ==\n", i);
		for (k = 0; k < 32; k += 16) { /* only dump first 32 bytes cmd */
			DISPMSG("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x\n", 0xd00 + k,
				mtk_spi_read((dsi_base_addr + 0xd00 + k)),
				mtk_spi_read((dsi_base_addr + 0xd00 + k + 0x4)),
				mtk_spi_read((dsi_base_addr + 0xd00 + k + 0x8)),
				mtk_spi_read((dsi_base_addr + 0xd00 + k + 0xc)));
		}

		if (level) {
			DISPMSG("========================== mt6382 MIPI%d REGS ==\n", i);
//			for (k = 0; k < sizeof(struct BDG_MIPI_TX_REGS); k += 16) {
			for (k = 0; k < 0x100; k += 16) {
				DISPMSG("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x\n", k,
					mtk_spi_read(mipi_base_addr + k),
					mtk_spi_read(mipi_base_addr + k + 0x4),
					mtk_spi_read(mipi_base_addr + k + 0x8),
					mtk_spi_read(mipi_base_addr + k + 0xc));
			}

			if (dsc_en) {
				DISPMSG("========================== mt6382 DSC%d REGS ==\n", i);
				for (k = 0; k < sizeof(struct BDG_DISP_DSC_REGS); k += 16) {
					DISPMSG("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x\n", k,
						mtk_spi_read(dsc_base_addr + k),
						mtk_spi_read(dsc_base_addr + k + 0x4),
						mtk_spi_read(dsc_base_addr + k + 0x8),
						mtk_spi_read(dsc_base_addr + k + 0xc));
				}
			}
		}
	}

	return 0;
}

int bdg_tx_wait_for_idle(enum DISP_BDG_ENUM module)
{
	int i;
	unsigned int timeout = 5000; /* unit: usec */
	unsigned int status;

	DISPFUNCSTART();

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		while (timeout) {
			udelay(1);
			status = mtk_spi_read((unsigned long)(&TX_REG[i]->DSI_TX_INTSTA));
//			DISPMSG("%s, i=%d, status=0x%x, timeout=%d\n",
//				__func__, i, status, timeout);

			if (!(status & 0x80000000))
				break;
			timeout--;
		}
	}

	if (timeout == 0) {
//		bdg_dsi_dump_reg(module, 0);
		bdg_tx_reset(module, NULL);
		DISPMSG("%s, wait timeout!\n", __func__);
		return -1;
	}

	return 0;
}

int ap_tx_phy_config(enum DISP_BDG_ENUM module,
			void *cmdq, struct LCM_DSI_PARAMS *tx_params)
{
	return 0;
}

int bdg_dsi_line_timing_dphy_setting(enum DISP_BDG_ENUM module,
			void *cmdq, struct LCM_DSI_PARAMS *tx_params)
{
	unsigned int width, height, lanes, ps_wc, new_hfp_byte;
	unsigned int bg_tx_total_word_cnt = 0;
	unsigned int bg_tx_line_time = 0, disp_pipe_line_time = 0;
	unsigned int rxtx_ratio = 0;
//	unsigned int ap_tx_total_word_cnt = 0, ap_tx_total_word_cnt_no_hfp_wc = 0;
	unsigned int data_Rate;

	DISPFUNCSTART();
	data_Rate = bg_mipi_clk_change_sta == 0 ? tx_data_rate : tx_dyn_data_rate;

	width = tx_params->horizontal_active_pixel / 1;
	height = tx_params->vertical_active_line;
	lanes = tx_params->LANE_NUM;

	/* Step 1. Show Bridge DSI TX Setting. */
	/* get bdg-tx hsa_byte, hbp_byte, new_hfp_byte and bllp_byte */
	if (dsc_en) {
		ps_wc = width * 24 / 8 / 3;	/* for 8bpp, 1/3 compression */
		rxtx_ratio = RXTX_RATIO;	/* ratio=2.25 */
	} else {
		ps_wc = width * 24 / 8;	/* for 8bpp, 1/3 compression */
		rxtx_ratio = 100;
	}
	new_hfp_byte = hfp_byte;

	DISPMSG("%s, dsc_en=%d, hsa_byte=%d, hbp_byte=%d\n",
		__func__, dsc_en, hsa_byte, hbp_byte);
	DISPMSG("%s, new_hfp_byte=%d, bllp_byte=%d, ps_wc=%d\n",
		__func__, new_hfp_byte, bllp_byte, ps_wc);

	/* get lpx, hs_prep, hs_zero,... refer to bdg_tx_phy_config()*/

	/* Step 2. Bridge DSI TX Cycle Count. */
	/* get bg_tx_line_cycle, bg_tx_total_word_cnt */
	/* D-PHY, DSI TX (Bridge IC) back to LP mode during V-Active */
	switch (tx_params->mode) {
	case CMD_MODE:
		bg_tx_total_word_cnt = 0;
		break;
	case SYNC_PULSE_VDO_MODE:
		bg_tx_total_word_cnt = 4 +	/* hss packet */
			(4 + hsa_byte + 2) +	/* hsa packet */
			4 +			/* hse packet */
			(4 + hbp_byte + 2) +	/* hbp packet */
			(4 + ps_wc + 2) +	/* rgb packet */
			(4 + new_hfp_byte + 2) +/* hfp packet */
			bg_tx_data_phy_cycle * lanes;
		break;
	case SYNC_EVENT_VDO_MODE:
		bg_tx_total_word_cnt = 4 +	/* hss packet */
			(4 + hbp_byte + 2) +	/* hbp packet */
			(4 + ps_wc + 2) +	/* rgb packet */
			(4 + new_hfp_byte + 2) +/* hfp packet */
			bg_tx_data_phy_cycle * lanes;
		break;
	case BURST_VDO_MODE:
		bg_tx_total_word_cnt = 4 +	/* hss packet */
			(4 + hbp_byte + 2) +	/* hbp packet */
			(4 + ps_wc + 2) +	/* rgb packet */
			(4 + bllp_byte + 2) +	/* bllp packet*/
			(4 + new_hfp_byte + 2) +/* hfp packet */
			bg_tx_data_phy_cycle * lanes;
		break;
	}

	bg_tx_line_cycle = (bg_tx_total_word_cnt + (lanes - 1)) / lanes;
	bg_tx_line_time = bg_tx_line_cycle * 8000 / data_Rate;

	disp_pipe_line_time = width * 1000 / MM_CLK;

	DISPMSG("bg_tx_total_word_cnt=%d, bg_tx_line_cycle=%d\n",
		bg_tx_total_word_cnt, bg_tx_line_cycle);
	DISPMSG("disp_pipe_line_time=%d, bg_tx_line_time=%d\n",
		disp_pipe_line_time, bg_tx_line_time);

	if ((tx_params->mode != CMD_MODE) && (disp_pipe_line_time > bg_tx_line_time)) {
		DISPMSG("error!! disp_pipe_line_time(%d) > bg_tx_line_time(%d)\n",
			disp_pipe_line_time, bg_tx_line_time);

		return -1;
	}
	/* Step 3. Decide AP DSI TX Data Rate and PHY Timing */
	/* get ap_tx_data_rate and set back to ap */
	/* get lpx, hs_prep, hs_zero,... refer to DSI_DPHY_TIMCONFIG()*/
	ap_tx_data_rate = tx_data_rate * rxtx_ratio / 100;

	/* Step 4. Decide AP DSI TX Blanking */
	/* get ap-tx hsa_byte, hbp_byte, new_hfp_byte and bllp_byte */
	/* refer to DSI_DPHY_Calc_VDO_Timing() */

	/* Step 5. fine-tune data rate */
#if 0
	if (dsc_en) {
		ap_tx_hsa_wc = 4;
		ap_tx_hbp_wc = 4;
	} else {
		ap_tx_hsa_wc = hsa_byte;
		ap_tx_hbp_wc = hbp_byte;
		ap_tx_data_phy_cycle = bg_tx_data_phy_cycle;
	}
	ap_tx_bllp_wc  = bllp_byte;
	ap_tx_total_word_cnt = bg_tx_line_cycle * lanes * rxtx_ratio / 100;

	switch (tx_params->mode) {
	case CMD_MODE:
		ap_tx_total_word_cnt_no_hfp_wc = 0;
		break;
	case SYNC_PULSE_VDO_MODE:
		ap_tx_total_word_cnt_no_hfp_wc = 4 +	/* hss packet */
			(4 + ap_tx_hsa_wc + 2) +	/* hsa packet */
			4 +				/* hse packet */
			(4 + ap_tx_hbp_wc + 2) +	/* hbp packet */
			(4 + ps_wc + 2) +		/* rgb packet */
			(4 + 2) +			/* hfp packet */
			ap_tx_data_phy_cycle * lanes;
		break;
	case SYNC_EVENT_VDO_MODE:
		ap_tx_total_word_cnt_no_hfp_wc = 4 +	/* hss packet */
			(4 + ap_tx_hbp_wc + 2) +	/* hbp packet */
			(4 + ps_wc + 2) +	/* rgb packet */
			(4 + 2) +/* hfp packet */
			ap_tx_data_phy_cycle * lanes;
		break;
	case BURST_VDO_MODE:
		ap_tx_total_word_cnt_no_hfp_wc = 4 +	/* hss packet */
			(4 + ap_tx_hbp_wc + 2) +	/* hbp packet */
			(4 + ps_wc + 2) +		/* rgb packet */
			(4 + ap_tx_bllp_wc + 2) +	/* bllp packet*/
			(4 + 2) +			/* hfp packet */
			ap_tx_data_phy_cycle * lanes;
		break;
	}

	ap_tx_hfp_wc = ap_tx_total_word_cnt - ap_tx_total_word_cnt_no_hfp_wc;

	DISPMSG(
		"%s, ap_tx_hsa_wc=%d, ap_tx_hbp_wc=%d, ap_tx_bllp_wc=%d, ap_tx_data_phy_cycle=%d\n",
		__func__, ap_tx_hsa_wc, ap_tx_hbp_wc, ap_tx_bllp_wc, ap_tx_data_phy_cycle);
	DISPMSG(
		"%s, ap_tx_hfp_wc=%d, ap_tx_total_word_cnt=%d, ap_tx_total_word_cnt_no_hfp_wc=%d\n",
		__func__, ap_tx_hfp_wc, ap_tx_total_word_cnt, ap_tx_total_word_cnt_no_hfp_wc);

#endif
	return 0;
}

unsigned int get_ap_data_rate(void)
{
	DISPMSG("%s, ap_tx_data_rate=%d\n", __func__, ap_tx_data_rate);

	return ap_tx_data_rate;
}

unsigned int get_ap_dyn_data_rate(int en)
{
	if (en)
		return ap_tx_dyn_data_rate;
	else
		return ap_tx_data_rate;
}

unsigned int get_bdg_dyn_data_rate(int en)
{
	if (en)
		return tx_dyn_data_rate;
	else
		return tx_data_rate;
}

void cal_dyn_data_rate(struct LCM_DSI_PARAMS *lcm_params)
{
	ap_tx_dyn_data_rate = lcm_params->data_rate_dyn ? lcm_params->data_rate_dyn :
		lcm_params->PLL_CLOCK_dyn * 2;

	ap_tx_dyn_data_rate = ap_tx_dyn_data_rate * RXTX_RATIO / 100;

	DISPMSG("%s, ap_tx_dyn_data_rate=%d\n",
		__func__, ap_tx_dyn_data_rate);
}

unsigned int get_bdg_data_rate(void)
{
	DISPMSG("%s, tx_data_rate=%d\n", __func__, tx_data_rate);

	return tx_data_rate;
}

int set_bdg_data_rate(unsigned int data_rate)
{

	if (data_rate > 2500 || data_rate < 100) {
		DISPINFO("error, please check data rate=%d MHz\n", data_rate);
		return -1;
	}
	tx_data_rate = data_rate;

	return 0;
}

unsigned int get_bdg_line_cycle(void)
{
	DISPMSG("%s, bg_tx_line_cycle=%d\n", __func__, bg_tx_line_cycle);

	return bg_tx_line_cycle;
}

unsigned int get_dsc_state(void)
{
	DISPMSG("%s, dsc_en=%d\n", __func__, dsc_en);

	return dsc_en;
}

void set_mt6382_init(unsigned int value)
{
	DISPMSG("%s, mt6382_init=%d->%d\n", __func__, mt6382_init, value);

	mt6382_init = value;
}

unsigned int get_mt6382_init(void)
{
	DISPMSG("%s, mt6382_init=%d\n", __func__, mt6382_init);

	return mt6382_init;
}

void set_bdg_tx_mode(unsigned int value)
{
	DISPMSG("%s: %u\n", __func__, value);

	bdg_tx_mode = value;
}

unsigned int get_bdg_tx_mode(void)
{
	DISPMSG("%s, bdg_tx_mode=%d\n", __func__, bdg_tx_mode);

	return bdg_tx_mode;
}

#define DELAY_US 1
void mt6382_nt36672c_fhd_vdo_init(bool dsc_on)
{
	DISPFUNCSTART();
	//lcm_dcs_write_seq_static(ctx, 0xFF, 0X10);
	mtk_spi_write(0x00021d00, 0x10ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		DISPMSG("%s, status=0x%x\n",
			__func__, mtk_spi_read((unsigned long)(&TX_REG[0]->DSI_TX_INTSTA)));
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		DISPMSG("%s, status=0x%x\n",
			__func__, mtk_spi_read((unsigned long)(&TX_REG[0]->DSI_TX_INTSTA)));
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	DISPMSG("%s, dsc_en=%d, tx_data_rate=%d\n", __func__, dsc_on, tx_data_rate);
	//DSC ON && set PPS
	if (1) {
		//lcm_dcs_write_seq_static(ctx, 0xC0, 0x03);
		mtk_spi_write(0x00021d00, 0x03c02300);
		mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
		mtk_spi_write(0x00021000, 0x00000000); //DSI_START
		mtk_spi_write(0x00021000, 0x00000001); //DSI_START
		while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
			DISPMSG("%s, status=0x%x\n",
				__func__, mtk_spi_read((unsigned long)(&TX_REG[0]->DSI_TX_INTSTA)));
			udelay(DELAY_US);
		}
		mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
		udelay(0);
	} else {
		//lcm_dcs_write_seq_static(ctx, 0xC0, 0x00);
		mtk_spi_write(0x00021d00, 0x00c02300);
		mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
		mtk_spi_write(0x00021000, 0x00000000); //DSI_START
		mtk_spi_write(0x00021000, 0x00000001); //DSI_START
		while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
			udelay(DELAY_US);
		}
		mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
		udelay(0);
	}

	//lcm_dcs_write_seq_static(ctx, 0xC1, 0x89, 0x28, 0x00, 0x08, 0x00, 0xAA,
	//0x02, 0x0E, 0x00, 0x2B, 0x00, 0x07, 0x0D, 0xB7,
	//0x0C, 0xB7);
	mtk_spi_write(0x00021d00, 0x00112902);
	mtk_spi_write(0x00021d04, 0x002889c1);
	mtk_spi_write(0x00021d08, 0x02aa0008);
	mtk_spi_write(0x00021d0c, 0x002b000e);
	mtk_spi_write(0x00021d10, 0x0cb70d07);
	mtk_spi_write(0x00021d14, 0x000000b7);
	mtk_spi_write(0x00021060, 0x00000006); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		DISPMSG("%s, status=0x%x\n",
			__func__, mtk_spi_read((unsigned long)(&TX_REG[0]->DSI_TX_INTSTA)));
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0xC2, 0x1B, 0XA0);
	mtk_spi_write(0x00021d00, 0x00032902);
	mtk_spi_write(0x00021d04, 0x00a01bc2);
	mtk_spi_write(0x00021060, 0x00000002); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0xFF, 0X20);
	mtk_spi_write(0x00021d00, 0x20ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X01, 0X66);
	mtk_spi_write(0x00021d00, 0x66011500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X32, 0X4D);
	mtk_spi_write(0x00021d00, 0x4d321500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X69, 0XD1);
	mtk_spi_write(0x00021d00, 0xd1691500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XF2, 0X64);
	mtk_spi_write(0x00021d00, 0x64f22300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XF4, 0X64);
	mtk_spi_write(0x00021d00, 0x64f42300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XF6, 0X64);
	mtk_spi_write(0x00021d00, 0x64f62300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XF9, 0X64);
	mtk_spi_write(0x00021d00, 0x64f92300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0X26);
	mtk_spi_write(0x00021d00, 0x26ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X81, 0X0E);
	mtk_spi_write(0x00021d00, 0x0e811500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X84, 0X03);
	mtk_spi_write(0x00021d00, 0x03841500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X86, 0X03);
	mtk_spi_write(0x00021d00, 0x03861500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X88, 0X07);
	mtk_spi_write(0x00021d00, 0x07881500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0X27);
	mtk_spi_write(0x00021d00, 0x27ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XE3, 0X01);
	mtk_spi_write(0x00021d00, 0x01e32300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XE4, 0XEC);
	mtk_spi_write(0x00021d00, 0xece42300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XE5, 0X02);
	mtk_spi_write(0x00021d00, 0x02e52300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XE6, 0XE3);
	mtk_spi_write(0x00021d00, 0xe3e62300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XE7, 0X01);
	mtk_spi_write(0x00021d00, 0x01e72300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XE8, 0XEC);
	mtk_spi_write(0x00021d00, 0xece82300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XE9, 0X02);
	mtk_spi_write(0x00021d00, 0x02e92300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XEA, 0X22);
	mtk_spi_write(0x00021d00, 0x22ea2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XEB, 0X03);
	mtk_spi_write(0x00021d00, 0x03eb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XEC, 0X32);
	mtk_spi_write(0x00021d00, 0x32ec2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XED, 0X02);
	mtk_spi_write(0x00021d00, 0x02ed2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XEE, 0X22);
	mtk_spi_write(0x00021d00, 0x22ee2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0X2A);
	mtk_spi_write(0x00021d00, 0x2aff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X0C, 0X04);
	mtk_spi_write(0x00021d00, 0x040c1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X0F, 0X01);
	mtk_spi_write(0x00021d00, 0x010f1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X11, 0XE0);
	mtk_spi_write(0x00021d00, 0xe0111500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X15, 0X0E);
	mtk_spi_write(0x00021d00, 0x0e151500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X16, 0X78);
	mtk_spi_write(0x00021d00, 0x78161500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X19, 0X0D);
	mtk_spi_write(0x00021d00, 0x0d191500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X1A, 0XF4);
	mtk_spi_write(0x00021d00, 0xf41a1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X37, 0X6E);
	mtk_spi_write(0x00021d00, 0x6e371500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X88, 0X76);
	mtk_spi_write(0x00021d00, 0x76881500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0X2C);
	mtk_spi_write(0x00021d00, 0x2cff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X4D, 0X1E);
	mtk_spi_write(0x00021d00, 0x1e4d1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X4E, 0X04);
	mtk_spi_write(0x00021d00, 0x044e1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X4F, 0X00);
	mtk_spi_write(0x00021d00, 0x004f1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X9D, 0X1E);
	mtk_spi_write(0x00021d00, 0x1e9d1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X9E, 0X04);
	mtk_spi_write(0x00021d00, 0x049e1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X9F, 0X17);
	mtk_spi_write(0x00021d00, 0x179f1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0XF0);
	mtk_spi_write(0x00021d00, 0xf0ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X5A, 0X00);
	mtk_spi_write(0x00021d00, 0x005a1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0XE0);
	mtk_spi_write(0x00021d00, 0xe0ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X25, 0X02);
	mtk_spi_write(0x00021d00, 0x02251500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X4E, 0X02);
	mtk_spi_write(0x00021d00, 0x024e1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X85, 0X02);
	mtk_spi_write(0x00021d00, 0x02851500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0XD0);
	mtk_spi_write(0x00021d00, 0xd0ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X09, 0XAD);
	mtk_spi_write(0x00021d00, 0xad091500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0X20);
	mtk_spi_write(0x00021d00, 0x20ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XF8, 0X64);
	mtk_spi_write(0x00021d00, 0x64f82300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0X2A);
	mtk_spi_write(0x00021d00, 0x2aff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X1A, 0XF0);
	mtk_spi_write(0x00021d00, 0xf01a1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);
	//lcm_dcs_write_seq_static(ctx, 0X30, 0X5E);
	mtk_spi_write(0x00021d00, 0x5e301500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X31, 0XCA);
	mtk_spi_write(0x00021d00, 0xca311500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X34, 0XFE);
	mtk_spi_write(0x00021d00, 0xfe341500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X35, 0X35);
	mtk_spi_write(0x00021d00, 0x35351500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X36, 0XA2);
	mtk_spi_write(0x00021d00, 0xa2361500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X37, 0XF8);
	mtk_spi_write(0x00021d00, 0xf8371500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X38, 0X37);
	mtk_spi_write(0x00021d00, 0x37381500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X39, 0XA0);
	mtk_spi_write(0x00021d00, 0xa0391500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X3A, 0X5E);
	mtk_spi_write(0x00021d00, 0x5e3a1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X53, 0XD7);
	mtk_spi_write(0x00021d00, 0xd7531500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X88, 0X72);
	mtk_spi_write(0x00021d00, 0x72881500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X88, 0X72);
	mtk_spi_write(0x00021d00, 0x72881500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0X24);
	mtk_spi_write(0x00021d00, 0x24ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XC6, 0XC0);
	mtk_spi_write(0x00021d00, 0xc0c62300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0XE0);
	mtk_spi_write(0x00021d00, 0xe0ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X25, 0X00);
	mtk_spi_write(0x00021d00, 0x00251500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X4E, 0X02);
	mtk_spi_write(0x00021d00, 0x024e1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X35, 0X82);
	mtk_spi_write(0x00021d00, 0x82351500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0XC0);
	mtk_spi_write(0x00021d00, 0xc0ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X9C, 0X11);
	mtk_spi_write(0x00021d00, 0x119c1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0X9D, 0X11);
	mtk_spi_write(0x00021d00, 0x119d1500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	if (dsc_on) {
		if (tx_data_rate < 600) {
		//60HZ
			//lcm_dcs_write_seq_static(ctx, 0XFF, 0X25);
			mtk_spi_write(0x00021d00, 0x25ff2300);
			mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
			mtk_spi_write(0x00021000, 0x00000000); //DSI_START
			mtk_spi_write(0x00021000, 0x00000001); //DSI_START
			while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
				udelay(DELAY_US);
			}
			mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
			udelay(0);

			//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
			mtk_spi_write(0x00021d00, 0x01fb2300);
			mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
			mtk_spi_write(0x00021000, 0x00000000); //DSI_START
			mtk_spi_write(0x00021000, 0x00000001); //DSI_START
			while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
				udelay(DELAY_US);
			}
			mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
			udelay(0);

			//lcm_dcs_write_seq_static(ctx, 0X18, 0X22);
			mtk_spi_write(0x00021d00, 0x22181500);
			mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
			mtk_spi_write(0x00021000, 0x00000000); //DSI_START
			mtk_spi_write(0x00021000, 0x00000001); //DSI_START
			while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
				udelay(DELAY_US);
			}
			mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
			udelay(0);
		} else if (tx_data_rate < 900) {
		//90HZ
			//lcm_dcs_write_seq_static(ctx, 0XFF, 0X25);
			mtk_spi_write(0x00021d00, 0x25ff2300);
			mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
			mtk_spi_write(0x00021000, 0x00000000); //DSI_START
			mtk_spi_write(0x00021000, 0x00000001); //DSI_START
			while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
				udelay(DELAY_US);
			}
			mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
			udelay(0);

			//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
			mtk_spi_write(0x00021d00, 0x01fb2300);
			mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
			mtk_spi_write(0x00021000, 0x00000000); //DSI_START
			mtk_spi_write(0x00021000, 0x00000001); //DSI_START
			while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
				udelay(DELAY_US);
			}
			mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
			udelay(0);

			//lcm_dcs_write_seq_static(ctx, 0X18, 0X21);
			mtk_spi_write(0x00021d00, 0x21181500);
			mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
			mtk_spi_write(0x00021000, 0x00000000); //DSI_START
			mtk_spi_write(0x00021000, 0x00000001); //DSI_START
			while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
				udelay(DELAY_US);
			}
			mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
			udelay(0);
		}
	} else {
		//60HZ
		//lcm_dcs_write_seq_static(ctx, 0XFF, 0X25);
		mtk_spi_write(0x00021d00, 0x25ff2300);
		mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
		mtk_spi_write(0x00021000, 0x00000000); //DSI_START
		mtk_spi_write(0x00021000, 0x00000001); //DSI_START
		while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
			udelay(DELAY_US);
		}
		mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
		udelay(0);

		//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
		mtk_spi_write(0x00021d00, 0x01fb2300);
		mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
		mtk_spi_write(0x00021000, 0x00000000); //DSI_START
		mtk_spi_write(0x00021000, 0x00000001); //DSI_START
		while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
			udelay(DELAY_US);
		}
		mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
		udelay(0);

		//lcm_dcs_write_seq_static(ctx, 0X18, 0X22);
		mtk_spi_write(0x00021d00, 0x22181500);
		mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
		mtk_spi_write(0x00021000, 0x00000000); //DSI_START
		mtk_spi_write(0x00021000, 0x00000001); //DSI_START
		while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
			udelay(DELAY_US);
		}
		mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
		udelay(0);
	}

	//lcm_dcs_write_seq_static(ctx, 0XFF, 0X10);
	mtk_spi_write(0x00021d00, 0x10ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0XFB, 0X01);
	mtk_spi_write(0x00021d00, 0x01fb2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//Set DSC ON/OFF
	if (dsc_on) {
		//lcm_dcs_write_seq_static(ctx, 0xC0, 0x03);
		mtk_spi_write(0x00021d00, 0x03c02300);
		mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
		mtk_spi_write(0x00021000, 0x00000000); //DSI_START
		mtk_spi_write(0x00021000, 0x00000001); //DSI_START
		while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
			DISPMSG("%s, status=0x%x\n",
				__func__, mtk_spi_read((unsigned long)(&TX_REG[0]->DSI_TX_INTSTA)));
			udelay(DELAY_US);
		}
		mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
		udelay(0);
	} else {
		//lcm_dcs_write_seq_static(ctx, 0xC0, 0x00);
		mtk_spi_write(0x00021d00, 0x00c02300);
		mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
		mtk_spi_write(0x00021000, 0x00000000); //DSI_START
		mtk_spi_write(0x00021000, 0x00000001); //DSI_START
		while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
			udelay(DELAY_US);
		}
		mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
		udelay(0);
	}

	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0x51, 0x00);
	mtk_spi_write(0x00021d00, 0x00511500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	mtk_spi_write(0x00021d00, 0x00351500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0x53, 0x24);
	mtk_spi_write(0x00021d00, 0x24531500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
	mtk_spi_write(0x00021d00, 0x00551500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
	mtk_spi_write(0x00021d00, 0x10ff2300);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	//lcm_dcs_write_seq_static(ctx, 0x11);
	mtk_spi_write(0x00021d00, 0x00110500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);

	msleep(120);
	//lcm_dcs_write_seq_static(ctx, 0x29);
	mtk_spi_write(0x00021d00, 0x00290500);
	mtk_spi_write(0x00021060, 0x00000001); //DSI_CMDQ_CON
	mtk_spi_write(0x00021000, 0x00000000); //DSI_START
	mtk_spi_write(0x00021000, 0x00000001); //DSI_START
	while ((mtk_spi_read(0x0002100c) & 0x2) != 0x2) { //wait dsi is not busy
		udelay(DELAY_US);
	}
	mtk_spi_write(0x0002100c, 0xfffd); //write 0 clear
	udelay(0);
	DISPFUNCEND();
}

void dsi_set_cmdq_v2(enum DISP_BDG_ENUM module, void *cmdq,
			unsigned int cmd, unsigned char count,
		     unsigned char *para_list, unsigned char force_update)
{
	UINT32 i = 0;
	unsigned long goto_addr, mask_para, set_para;
	struct DSI_TX_T0_INS t0;
	struct DSI_TX_T2_INS t2;
	struct DSI_TX_CMDQ *tx_cmdq;

	memset(&t0, 0, sizeof(struct DSI_TX_T0_INS));
	memset(&t2, 0, sizeof(struct DSI_TX_T2_INS));

	tx_cmdq = TX_CMDQ_REG[0]->data;

	bdg_tx_wait_for_idle(module);

	if (count > 1) {
		t2.CONFG = 2;
		if (cmd < 0xB0)
			t2.Data_ID = TX_DCS_LONG_PACKET_ID;
		else
			t2.Data_ID = TX_GERNERIC_LONG_PACKET_ID;
		t2.WC16 = count + 1;

		DSI_OUTREG32(cmdq, tx_cmdq[0], AS_UINT32(&t2));

		goto_addr = (unsigned long)(&tx_cmdq[1].byte0);
		mask_para = (0xFFu << ((goto_addr & 0x3u) * 8));
		set_para = cmd;
		goto_addr = (goto_addr & (~0x3UL));

//		DISPMSG("[%s][%d]goto_addr=0x%x, mask_para=0x%x, set_para=0x%x\n",
//			__func__, __LINE__, goto_addr, mask_para, set_para);

		DSI_MASKREG32(cmdq, goto_addr, mask_para, set_para);

		for (i = 0; i < count; i++) {
			goto_addr = (unsigned long)(&tx_cmdq[1].byte1) + i;
			mask_para = (0xFFu << ((goto_addr & 0x3u) * 8));
			set_para = (unsigned long)para_list[i];
			goto_addr = (goto_addr & (~0x3UL));

//			DISPMSG("[%s][%d]i=%d, goto_addr=0x%x, mask_para=0x%x, set_para=0x%x\n",
//				__func__, __LINE__, i, goto_addr, mask_para, set_para);

			DSI_MASKREG32(cmdq, goto_addr, mask_para, set_para);
		}

		DSI_OUTREG32(cmdq, TX_REG[0]->DSI_TX_CMDQ_CON,
				(1 << 15) | (2 + (count) / 4));
	} else {
		t0.CONFG = 0;
		t0.Data0 = cmd;
		if (count) {
			if (cmd < 0xB0)
				t0.Data_ID = TX_DCS_SHORT_PACKET_ID_1;
			else
				t0.Data_ID = TX_GERNERIC_SHORT_PACKET_ID_2;
			t0.Data1 = para_list[0];
		} else {
			if (cmd < 0xB0)
				t0.Data_ID = TX_DCS_SHORT_PACKET_ID_0;
			else
				t0.Data_ID = TX_GERNERIC_SHORT_PACKET_ID_1;
			t0.Data1 = 0;
		}

		DSI_OUTREG32(cmdq, tx_cmdq[0], AS_UINT32(&t0));
		DSI_OUTREG32(cmdq, TX_REG[0]->DSI_TX_CMDQ_CON, 1);
	}

	if (force_update) {
		bdg_tx_start(module, cmdq);
		bdg_tx_wait_for_idle(module);
	}
}

void push_table(struct lcm_setting_table *table, unsigned int count,
			unsigned char force_update)
{
	unsigned int i, cmd;

	DISPFUNCSTART();

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				mdelay(table[i].count);
			else
				mdelay(table[i].count);
			break;
		case REGFLAG_UDELAY:
			udelay(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_v2(DISP_BDG_DSI0, NULL, cmd,
				table[i].count, table[i].para_list,
				force_update);
			break;
		}
	}
}

int lcm_init(enum DISP_BDG_ENUM module)
{
	DISPFUNCSTART();

//	mt6382_nt36672c_fhd_vdo_init(dsc_en);

	if (dsc_en) {
		if (tx_data_rate < 601)
			push_table(nt36672c_60hz, sizeof(nt36672c_60hz) /
				sizeof(struct lcm_setting_table), 1);
		else if (tx_data_rate < 901)
			push_table(nt36672c_90hz, sizeof(nt36672c_90hz) /
				sizeof(struct lcm_setting_table), 1);
		else
			push_table(nt36672c_120hz, sizeof(nt36672c_120hz) /
				sizeof(struct lcm_setting_table), 1);
	} else {
		if (tx_data_rate < 1201)
			push_table(nt36672c_60hz, sizeof(nt36672c_60hz) /
				sizeof(struct lcm_setting_table), 1);
		else if (tx_data_rate < 1601)
			push_table(nt36672c_90hz, sizeof(nt36672c_90hz) /
				sizeof(struct lcm_setting_table), 1);
		else
			push_table(nt36672c_120hz, sizeof(nt36672c_120hz) /
				sizeof(struct lcm_setting_table), 1);

		push_table(nt36672c_wo_dsc, sizeof(nt36672c_wo_dsc) /
			sizeof(struct lcm_setting_table), 1);
	}

//	push_table(nt35695b_cmd_mode, sizeof(nt35695b_cmd_mode) /
//		sizeof(struct lcm_setting_table), 1);

	DISPFUNCEND();

	return 0;
}

int bdg_tx_init(enum DISP_BDG_ENUM module,
		   struct disp_ddp_path_config *config, void *cmdq)
{
	int ret = 0;
	struct LCM_DSI_PARAMS *tx_params;
	unsigned int data_rate;

	DISPFUNCSTART();

	tx_params = &(config->dispif_config.dsi);
	dsc_en = tx_params->bdg_dsc_enable;
	bdg_tx_mode = tx_params->mode;
	if (tx_params->PLL_CLOCK) {
		tx_data_rate = tx_params->PLL_CLOCK * 2;
		tx_dyn_data_rate = tx_params->PLL_CLOCK_dyn * 2;
	} else {
		DISPMSG("PLL clock should not be 0!!!\n");
		return -1;
	}
	cal_dyn_data_rate(tx_params);
	data_rate = bg_mipi_clk_change_sta == 0 ? tx_data_rate : tx_dyn_data_rate;
	DISPMSG("%s, tx_data_rata = %d, tx_dyn_data_rate = %d\n", __func__,
		tx_data_rate, tx_dyn_data_rate);
	DISPMSG("%s, bg_mipi_change_sta=%d, data_rate=%d, bdg_ssc_disable=%d",
		__func__, bg_mipi_clk_change_sta, data_rate, tx_params->bdg_ssc_disable);
	DISPMSG("%s, ssc_disable=%d, dsc_enable=%d, mode=%d\n", __func__,
		tx_params->ssc_disable, dsc_en, bdg_tx_mode);

	ret |= bdg_mipi_tx_dphy_clk_setting(module, cmdq, tx_params);
	udelay(20);

	ret |= bdg_tx_phy_config(module, cmdq, tx_params);
	ret |= bdg_tx_txrx_ctrl(module, cmdq, tx_params);
	ret |= bdg_tx_ps_ctrl(module, cmdq, tx_params);
	ret |= bdg_tx_vdo_timing_set(module, cmdq, tx_params);
	ret |= bdg_tx_buf_rw_set(module, cmdq, tx_params);
	ret |= bdg_tx_enable_hs_clk(module, cmdq, TRUE);
	ret |= bdg_tx_set_mode(module, cmdq, CMD_MODE);
	ret |= bdg_dsi_line_timing_dphy_setting(module, cmdq, tx_params);

	/* panel init*/
//	lcm_init(module);

//	ret |= bdg_tx_set_mode(module, cmdq, tx_params->mode);

	DISPFUNCEND();
	return ret;
}

int bdg_tx_deinit(enum DISP_BDG_ENUM module, void *cmdq)
{
	int i;

	DISPFUNCSTART();

#if 0
	/* step 0: PLL DISABLE */
//	MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON1,
//			 FLD_RG_DSI_PLL_EN, 0);
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_CON1_REG,
		MIPI_TX_REG->MIPI_TX_PLL_CON1, RG_DSI_PLL_EN, 0);

	/* step 1: SDM_RWR_ON / SDM_ISO_EN */
//	MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_PWR,
//			 FLD_AD_DSI_PLL_SDM_ISO_EN, 1);
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_PWR_REG,
		MIPI_TX_REG->MIPI_TX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN, 1);
//	MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_PWR,
//			 FLD_AD_DSI_PLL_SDM_PWR_ON, 0);
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_PLL_PWR_REG,
		MIPI_TX_REG->MIPI_TX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON, 0);

	/* Switch ON each Lane */
//	MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D0_SW_CTL_EN,
//		FLD_DSI_D0_SW_CTL_EN, 1);
//	MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D1_SW_CTL_EN,
//		FLD_DSI_D1_SW_CTL_EN, 1);
//	MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D2_SW_CTL_EN,
//		FLD_DSI_D2_SW_CTL_EN, 1);
//	MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D3_SW_CTL_EN,
//		FLD_DSI_D3_SW_CTL_EN, 1);
//	MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_CK_SW_CTL_EN,
//		FLD_DSI_CK_SW_CTL_EN, 1);
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
		MIPI_TX_REG->MIPI_TX_D0_SW_CTL_EN, DSI_SW_CTL_EN, 1);
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
		MIPI_TX_REG->MIPI_TX_D1_SW_CTL_EN, DSI_SW_CTL_EN, 1);
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
		MIPI_TX_REG->MIPI_TX_D2_SW_CTL_EN, DSI_SW_CTL_EN, 1);
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
		MIPI_TX_REG->MIPI_TX_D3_SW_CTL_EN, DSI_SW_CTL_EN, 1);
	DSI_OUTREGBIT(cmdq, struct MIPI_TX_SW_CTL_EN_REG,
		MIPI_TX_REG->MIPI_TX_CK_SW_CTL_EN, DSI_SW_CTL_EN, 1);

	/* step 2 */
	/* BG_LPF_EN=0, TIEL_SEL=1 */
//	MIPITX_OUTREG32(DSI_PHY_REG[i] + MIPITX_LANE_CON,
//		0x3FFF0180);
	DSI_OUTREG32(cmdq, MIPI_TX_REG->MIPI_TX_LANE_CON, 0x3FFF0180);
	/* BG_CORE_EN=0 */
//	MIPITX_OUTREG32(DSI_PHY_REG[i] + MIPITX_LANE_CON,
//		0x3FFF0100);
	DSI_OUTREG32(cmdq, MIPI_TX_REG->MIPI_TX_LANE_CON, 0x3FFF0100);
#endif
	bdg_tx_enable_hs_clk(module, cmdq, FALSE);
	bdg_tx_set_mode(module, cmdq, CMD_MODE);
	bdg_tx_reset(module, cmdq);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_TX_TXRX_CON_REG,
			TX_REG[i]->DSI_TX_TXRX_CON, LANE_NUM, 0);
		/* a0 mode */
//		DSI_OUTREG32(cmdq, &TX_REG[i]->DSI_PHY_LCPAT, 0xAA);
	}

//	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
//		DSI_INT_DONE[i] = 0;

	/* dsi_dump_info(module, 1); */

	DISPFUNCEND();
	return 0;
}

void calculate_datarate_cfgs_rx(unsigned int data_rate)
{
	unsigned int timebase = 1000, itminrx = 4; // 1us
	unsigned int des_div_en_dly_deass_th = 1;
	unsigned int hs_clk_freq = data_rate * 1000 / 2;

	DISPFUNCSTART();
//	ddl_cntr_ref_reg = 5 * timebase * hs_clk_freq *
//		2 / 1000000 / 2 / 16 / 2; //round up
	ddl_cntr_ref_reg = 5 * hs_clk_freq / 1000 / 2 / 16;

	if (data_rate >= 4500) {
		sel_fast = 1;
		max_phase = 63;
		dll_fbk = 7;
		coarse_bank = 0;
	} else if (data_rate >= 4000) {
		sel_fast = 1;
		max_phase = 71;
		dll_fbk = 8;
		coarse_bank = 1;
	} else if (data_rate >= 3600) {
		sel_fast = 1;
		max_phase = 79;
		dll_fbk = 9;
		coarse_bank = 1;
	} else if (data_rate >= 3230) {
		sel_fast = 1;
		max_phase = 87;
		dll_fbk = 10;
		coarse_bank = 1;
	} else if (data_rate >= 3000) {
		sel_fast = 0;
		max_phase = 71;
		dll_fbk = 8;
		coarse_bank = 0;
	} else if (data_rate >= 2700) {
		sel_fast = 0;
		max_phase = 79;
		dll_fbk = 9;
		coarse_bank = 1;
	} else if (data_rate >= 2455) {
		sel_fast = 0;
		max_phase = 87;
		dll_fbk = 10;
		coarse_bank = 1;
	} else if (data_rate >= 2250) {
		sel_fast = 0;
		max_phase = 95;
		dll_fbk = 11;
		coarse_bank = 1;
	} else if (data_rate >= 2077) {
		sel_fast = 0;
		max_phase = 103;
		dll_fbk = 12;
		coarse_bank = 1;
	} else if (data_rate >= 1929) {
		sel_fast = 0;
		max_phase = 111;
		dll_fbk = 13;
		coarse_bank = 2;
	} else if (data_rate >= 1800) {
		sel_fast = 0;
		max_phase = 119;
		dll_fbk = 14;
		coarse_bank = 2;
	} else if (data_rate >= 1688) {
		sel_fast = 0;
		max_phase = 127;
		dll_fbk = 15;
		coarse_bank = 2;
	} else if (data_rate >= 1588) {
		sel_fast = 0;
		max_phase = 135;
		dll_fbk = 16;
		coarse_bank = 2;
	} else if (data_rate >= 1500) {
		sel_fast = 0;
		max_phase = 143;
		dll_fbk = 17;
		coarse_bank = 3;
	} else {
		sel_fast = 0;
		max_phase = 0;
		dll_fbk = 0;
		coarse_bank = 0;
	}

	if (hs_clk_freq * 2 < 160000)
		hsrx_clk_div = 1;
	else if (hs_clk_freq * 2 < 320000)
		hsrx_clk_div = 2;
	else if (hs_clk_freq * 2 < 640000)
		hsrx_clk_div = 3;
	else if (hs_clk_freq * 2 < 1280000)
		hsrx_clk_div = 4;
	else if (hs_clk_freq * 2 < 2560000)
		hsrx_clk_div = 5;
	else
		hsrx_clk_div = 6;

	hs_thssettle = 115 + 1000 * 6 / data_rate;
	hs_thssettle = (hs_thssettle - T_DCO - (itminrx + 3) * T_DCO - 2 * T_DCO) / T_DCO - 1;
	fjump_deskew_reg = max_phase / 10 / 4;
	eye_open_deskew_reg = max_phase * 4 / 10 / 2;

	DISPMSG("data_rate=%d, ddl_cntr_ref_reg=%d, hs_thssettle=%d, fjump_deskew_reg=%d\n",
		data_rate, ddl_cntr_ref_reg, hs_thssettle, fjump_deskew_reg);

	if (hs_clk_freq * 2 >= 900000)
		cdr_coarse_trgt_reg = timebase * hs_clk_freq * 2 * 2 / 32 / 1000000 - 1;
	else
		cdr_coarse_trgt_reg = timebase * 900000 * 2 / 32 / 1000000 - 1;

	en_dly_deass_thresh_reg = 1000000 * 7 * 6 / hs_clk_freq / 2 / T_DCO +
		des_div_en_dly_deass_th;

	post_rcvd_rst_val = 2 * T_DCO * hs_clk_freq * 2 / 7 / 1000000 - 1;
	post_rcvd_rst_reg = (post_rcvd_rst_val > 0) ? post_rcvd_rst_val : 1;

	post_det_dly_thresh_val = ((189 * 1000000 / hs_clk_freq / 2) -
				(9 * 7 * 1000000 / hs_clk_freq / 2)) / T_DCO - 7;
	post_det_dly_thresh_reg = (post_det_dly_thresh_val > 0) ?
		post_det_dly_thresh_val : 1;

	DISPMSG("cdr_coarse_trgt_reg=%d, post_rcvd_rst_val=%d, fjump_deskew_reg=%d\n",
		cdr_coarse_trgt_reg, post_rcvd_rst_val, post_rcvd_rst_reg);
	DISPMSG(
		"en_dly_deass_thresh_reg=%d, post_det_dly_thresh_val=%d, post_det_dly_thresh_reg=%d\n",
		en_dly_deass_thresh_reg, post_det_dly_thresh_val, post_det_dly_thresh_reg);
}

void mipi_rx_enable(void *cmdq)
{
	DSI_OUTREG32(cmdq, OCLA_REG->OCLA_LANEC_CON, 0);
	DSI_OUTREG32(cmdq, OCLA_REG->OCLA_LANE0_CON, 0);
	DSI_OUTREG32(cmdq, OCLA_REG->OCLA_LANE1_CON, 0);
	DSI_OUTREG32(cmdq, OCLA_REG->OCLA_LANE2_CON, 0);
	DSI_OUTREG32(cmdq, OCLA_REG->OCLA_LANE3_CON, 0);
}

void mipi_rx_unset_forcerxmode(void *cmdq)
{
	DSI_OUTREGBIT(cmdq, struct OCLA_LANE_CON_REG,
			OCLA_REG->OCLA_LANEC_CON, FORCE_RX_MODE, 0);
	DSI_OUTREGBIT(cmdq, struct OCLA_LANE_CON_REG,
			OCLA_REG->OCLA_LANE0_CON, FORCE_RX_MODE, 0);
	DSI_OUTREGBIT(cmdq, struct OCLA_LANE_CON_REG,
			OCLA_REG->OCLA_LANE1_CON, FORCE_RX_MODE, 0);
	DSI_OUTREGBIT(cmdq, struct OCLA_LANE_CON_REG,
			OCLA_REG->OCLA_LANE2_CON, FORCE_RX_MODE, 0);
	DSI_OUTREGBIT(cmdq, struct OCLA_LANE_CON_REG,
			OCLA_REG->OCLA_LANE3_CON, FORCE_RX_MODE, 0);
}

void mipi_rx_set_forcerxmode(void *cmdq)
{
	DSI_OUTREGBIT(cmdq, struct OCLA_LANE_CON_REG,
			OCLA_REG->OCLA_LANEC_CON, FORCE_RX_MODE, 1);
	DSI_OUTREGBIT(cmdq, struct OCLA_LANE_CON_REG,
			OCLA_REG->OCLA_LANE0_CON, FORCE_RX_MODE, 1);
	DSI_OUTREGBIT(cmdq, struct OCLA_LANE_CON_REG,
			OCLA_REG->OCLA_LANE1_CON, FORCE_RX_MODE, 1);
	DSI_OUTREGBIT(cmdq, struct OCLA_LANE_CON_REG,
			OCLA_REG->OCLA_LANE2_CON, FORCE_RX_MODE, 1);
	DSI_OUTREGBIT(cmdq, struct OCLA_LANE_CON_REG,
			OCLA_REG->OCLA_LANE3_CON, FORCE_RX_MODE, 1);
}

void startup_seq_common(void *cmdq)
{
	DISPFUNCSTART();

	DSI_OUTREG32(cmdq, OCLA_REG->OCLA_LANE_SWAP, 0x30213102);
	mipi_rx_enable(cmdq);
	mipi_rx_set_forcerxmode(cmdq);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_0 * 4,
		CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_0_CB_LP_DCO_EN_DLY_MASK, 63);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_STARTUP_RW_COMMON_STARTUP_1_1 * 4,
		PPI_STARTUP_RW_COMMON_STARTUP_1_1_PHY_READY_DLY_MASK, 563);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_STARTUP_RW_COMMON_DPHY_6 * 4,
		PPI_STARTUP_RW_COMMON_DPHY_6_LP_DCO_CAL_addr_MASK, 39);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_CALIBCTRL_RW_COMMON_BG_0 * 4,
		PPI_CALIBCTRL_RW_COMMON_BG_0_BG_MAX_COUNTER_MASK, 500);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_TERMCAL_CFG_0 * 4,
		PPI_RW_TERMCAL_CFG_0_TERMCAL_TIMER_MASK,
		25); // cfg_clk = 26 MHz

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_OFFSETCAL_CFG_0 * 4,
		PPI_RW_OFFSETCAL_CFG_0_OFFSETCAL_WAIT_THRESH_MASK,
		5); // cfg_clk = 26 MHz

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_LPDCOCAL_TIMEBASE * 4,
		PPI_RW_LPDCOCAL_TIMEBASE_LPCDCOCAL_TIMEBASE_MASK,
		103); // cfg_clk = 26 MHz

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_LPDCOCAL_NREF * 4,
		PPI_RW_LPDCOCAL_NREF_LPDCOCAL_NREF_MASK, 800);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_LPDCOCAL_NREF_RANGE * 4,
		PPI_RW_LPDCOCAL_NREF_RANGE_LPDCOCAL_NREF_RANGE_MASK, 15);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_LPDCOCAL_TWAIT_CONFIG * 4,
		PPI_RW_LPDCOCAL_TWAIT_CONFIG_LPDCOCAL_TWAIT_PON_MASK, 127);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_LPDCOCAL_TWAIT_CONFIG * 4,
		PPI_RW_LPDCOCAL_TWAIT_CONFIG_LPDCOCAL_TWAIT_COARSE_MASK,
		32); // cfg_clk = 26 MHz

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_LPDCOCAL_VT_CONFIG * 4,
		PPI_RW_LPDCOCAL_VT_CONFIG_LPDCOCAL_TWAIT_FINE_MASK,
		32); // cfg_clk = 26 MHz

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_LPDCOCAL_VT_CONFIG * 4,
		PPI_RW_LPDCOCAL_VT_CONFIG_LPCDCOCAL_VT_NREF_RANGE_MASK, 15);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_LPDCOCAL_VT_CONFIG * 4,
		PPI_RW_LPDCOCAL_VT_CONFIG_LPCDCOCAL_USE_IDEAL_NREF_MASK, 0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_LPDCOCAL_VT_CONFIG * 4,
		PPI_RW_LPDCOCAL_VT_CONFIG_LPCDCOCAL_VT_TRACKING_EN_MASK, 0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_LPDCOCAL_COARSE_CFG * 4,
		PPI_RW_LPDCOCAL_COARSE_CFG_NCOARSE_START_MASK, 1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6 * 4,
		CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6_OA_CB_HSTXLB_DCO_PON_OVR_EN_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_COMMON_CFG * 4,
		PPI_RW_COMMON_CFG_CFG_CLK_DIV_FACTOR_MASK, 3);
	DISPFUNCEND();
}

int check_stopstate(void *cmdq)
{
	unsigned int timeout = 5000;
	unsigned int stop_state = 0, count = 0;

	DISPFUNCSTART();
	while (timeout) {
		stop_state = mtk_spi_read((unsigned long)(&OCLA_REG->OCLA_LANE0_STOPSTATE));

		DISPMSG("stop_state=0x%x, timeout=%d\n", stop_state, timeout);

		if ((stop_state & 0x1) == 0x1)
			count++;

		if (count > 5)
			break;

		udelay(1);
		timeout--;
	}

	if (timeout == 0) {
		DISPMSG("%s, wait timeout!\n", __func__);
		return -1;
	}
	mipi_rx_unset_forcerxmode(cmdq);

/* Disable extra design */
	mtk_spi_write(0x00047080, 0x0);
	mtk_spi_write(0x00047084, 0x102);

	mt6382_init = 1;

	DISPFUNCEND();
	return 0;
}

int polling_status(void)
{
	unsigned int timeout = 5000;
	unsigned int status = 0;

	DISPFUNCSTART();
	while (timeout) {
		status = mtk_spi_read((unsigned long)(&TX_REG[0]->DSI_TX_STATE_DBG7));

		DISPMSG("%s, status=0x%x, timeout=%d\n", __func__, status, timeout);

		if ((status & 0x800) == 0x800)
			break;

		udelay(1);
		timeout--;
	}

	if (timeout == 0) {
		DISPMSG("%s, wait timeout!\n", __func__);
		bdg_dsi_dump_reg(DISP_BDG_DSI0, 0);
		return -1;
	}

	DISPFUNCEND();
	return 0;
}

int bdg_dsc_init(enum DISP_BDG_ENUM module,
			void *cmdq, struct LCM_DSI_PARAMS *tx_params)
{
	unsigned long width = tx_params->horizontal_active_pixel / 1;
	unsigned long height = tx_params->vertical_active_line;
	unsigned int init_delay_limit, init_delay_height;
	unsigned int pic_group_width_m1;
	unsigned int pic_height_m1, pic_height_ext_m1, pic_height_ext_num;
	unsigned int slice_group_width_m1;
	unsigned int pad_num;
	unsigned int val;
	struct LCM_DSC_CONFIG_PARAMS *params = &tx_params->dsc_params;

	DISPFUNCSTART();
	if (!dsc_en) {
		DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PIC_H, height - 1);
		DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PIC_W, width);
		return 0;
	}

	if (params->pic_width != width || params->pic_height != height) {
		DISPMSG("%s size mismatch...", __func__);
		return 1;
	}

	/* DSC Empty flag always high */
	DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
		DSC_REG->DISP_DSC_CON, DSC_EMPTY_FLAG_SEL, 1);

	/* DSC output buffer as FHD(plus) */
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_OBUF, 0x800002C2);

	init_delay_limit =
		((128 + (params->xmit_delay + 2) / 3) * 3 +
		params->slice_width - 1) / params->slice_width;
	init_delay_height =
		(init_delay_limit > 15) ? 15 : init_delay_limit;

	val = params->slice_mode + (init_delay_height << 8) + (1 << 16);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_MODE, val);

	pic_group_width_m1 = (width + 2) / 3 - 1;
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PIC_W,
		(pic_group_width_m1 << 16) + width);

	pic_height_m1 = height - 1;
	pic_height_ext_num = (height + params->slice_height - 1) /
	    params->slice_height;
	pic_height_ext_m1 = pic_height_ext_num * params->slice_height - 1;
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PIC_H,
		(pic_height_ext_m1 << 16) + pic_height_m1);

	slice_group_width_m1 = (params->slice_width + 2) / 3 - 1;
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_SLICE_W,
		(slice_group_width_m1 << 16) + params->slice_width);

	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_SLICE_H,
		((params->slice_width % 3) << 30) +
		((pic_height_ext_num - 1) << 16) + params->slice_height - 1);

	DSI_OUTREG32(cmdq,  DSC_REG->DISP_DSC_CHUNK_SIZE,
		params->chunk_size);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_BUF_SIZE,
		params->chunk_size * params->slice_height);

	pad_num = (params->chunk_size + 2) / 3 * 3 - params->chunk_size;
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PAD, pad_num);
	if (params->dsc_cfg)
		DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_CFG, params->dsc_cfg);
	else
		DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_CFG, 0x22);

	if ((params->ver & 0xf) == 2)
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_SHADOW_REG,
			DSC_REG->DISP_DSC_SHADOW, DSC_VERSION_MINOR, 0x2);
	else
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_SHADOW_REG,
			DSC_REG->DISP_DSC_SHADOW, DSC_VERSION_MINOR, 0x1);

	/* set PPS */
	val = params->dsc_line_buf_depth + (params->bit_per_channel << 4) +
		(params->bit_per_pixel << 8) + (params->rct_on << 18) +
		(params->bp_enable << 19);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[0], val);

	val = (params->xmit_delay) + (params->dec_delay << 16);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[1], val);

	val = (params->scale_value) + (params->increment_interval << 16);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[2], val);

	val = (params->decrement_interval) + (params->line_bpg_offset << 16);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[3], val);

	val = (params->nfl_bpg_offset) + (params->slice_bpg_offset << 16);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[4], val);

	val = (params->initial_offset) + (params->final_offset << 16);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[5], val);

	val = (params->flatness_minqp) + (params->flatness_maxqp << 8) +
		(params->rc_model_size << 16);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[6], val);

	val = (params->rc_edge_factor) + (params->rc_quant_incr_limit0 << 8) +
		(params->rc_quant_incr_limit1 << 16) +
		(params->rc_tgt_offset_hi << 24) +
		(params->rc_tgt_offset_lo << 28);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[7], val);

	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[8], 0x382a1c0e);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[9], 0x69625446);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[10], 0x7b797770);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[11], 0x7e7d);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[12], 0x800880);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[13], 0xf8c100a1);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[14], 0xe8e3f0e3);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[15], 0xe103e0e3);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[16], 0xd943e123);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[17], 0xd185d965);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[18], 0xd1a7d1a5);
	DSI_OUTREG32(cmdq, DSC_REG->DISP_DSC_PPS[19], 0xd1ed);

	DISPFUNCEND();
	return 0;
}

int mipi_dsi_rx_mac_init(enum DISP_BDG_ENUM module,
			struct disp_ddp_path_config *config, void *cmdq)
{
	int ret = 0;
	unsigned int lanes = 0, ipi_mode_qst, out_type;
	unsigned int temp, frame_width;
	unsigned int ipi_tx_delay_qst, t_ipi_tx_delay;
	unsigned int t_ppi_clk, t_ipi_clk, t_hact_ppi, t_hact_ipi;
	/* bit2: HSRX EoTp enable, bit1: LPTX EoTp enable, bit0: LPRX EoTp enable */
	unsigned int eotp_cfg = 4;
	unsigned int timeout = 5000;
	unsigned int phy_ready = 0, count = 0;
	unsigned int ap_data_rate;
	struct LCM_DSI_PARAMS *tx_params;

	DISPFUNCSTART();
	tx_params = &(config->dispif_config.dsi);
	lanes = tx_params->LANE_NUM > 0 ? tx_params->LANE_NUM - 1 : 0;
	ipi_mode_qst = tx_params->mode == SYNC_PULSE_VDO_MODE ? 0 : 1;
	out_type = tx_params->mode == CMD_MODE ? 1 : 0;
	frame_width = tx_params->horizontal_active_pixel;

	DSI_OUTREG32(cmdq, SYS_REG->DISP_MISC0, 1);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_N_LANES_OS, 3);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_PHY_SHUTDOWNZ_OS, 1);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_PHY_RSTZ_OS, 1);

	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_SOFT_RSTN_OS, 0);
	udelay(1);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_SOFT_RSTN_OS, 1);

	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_VERSION_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_N_LANES_OS, lanes);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_MAIN_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_EOTP_CFG_OS, eotp_cfg);
//	if (out_type)
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_CTRL_CFG_OS, 0);

	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_FIFO_STATUS_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_PHY_MODE_OS, tx_params->IsCphy);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_PHY_TEST_CTRL0_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_PHY_TEST_CTRL1_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_PHY_DATA_STATUS_OS, 0);
//	if (out_type) {
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_LPTXRDY_TO_CNT_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_LPTX_TO_CNT_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_HSRX_TO_CNT_OS, 0);
//	}

	//Interrupt Registers
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_PHY_FATAL_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_MASK_N_PHY_FATAL_OS,
		0xffffffff);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_FORCE_PHY_FATAL_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_PHY_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_MASK_N_PHY_OS,
		0xffffffff);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_FORCE_PHY_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_DSI_FATAL_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_MASK_N_DSI_FATAL_OS,
		0xffffffff);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_FORCE_DSI_FATAL_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_DSI_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_MASK_N_DSI_OS,
		0xffffffff);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_FORCE_DSI_OS, 0);

//	if (out_type) {
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_DDI_FATAL_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_MASK_N_DDI_FATAL_OS,
		0xffffffff);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_FORCE_DDI_FATAL_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_DDI_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_MASK_N_DDI_OS,
		0xffffffff);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_FORCE_DDI_OS, 0);
//	}
	//video mode/ipi
//	if (!out_type) {
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_IPI_FATAL_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_MASK_N_IPI_FATAL_OS,
		0xffffffff);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_FORCE_IPI_FATAL_OS, 0);
//	}
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_FIFO_FATAL_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_MASK_N_FIFO_FATAL_OS,
		0xffffffff);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_FORCE_FIFO_FATAL_OS, 0);

//	if (out_type) {
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_ERR_RPT_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_MASK_N_ERR_RPT_OS,
		0xffffffff);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_FORCE_ERR_RPT_OS, 0);
//	}
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_ST_RX_TRIGGERS_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_MASK_N_RX_TRIGGERS_OS,
		0xffffffff);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_INT_FORCE_RX_TRIGGERS_OS, 0);

//	if (out_type) {
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_DDI_RDY_TO_CNT_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_DDI_RESP_TO_CNT_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_DDI_VALID_VC_CFG_OS, 0xf);
	/* 0x1b for MMCLK 270M 0x37 for MMCLK 407M */
#ifdef _90HZ_
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_DDI_CLK_MGR_CFG_OS, 0x1b);
#else
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_DDI_CLK_MGR_CFG_OS, 0x37);
#endif
//	}
	ap_data_rate = bg_mipi_clk_change_sta == 0 ? ap_tx_data_rate : ap_tx_dyn_data_rate;

	//video mode/ipi
//	if (!out_type) {
		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_MODE_CFG_OS, 0);

		if (ipi_mode_qst)
			DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_MODE_CFG_OS, 1);

		t_ipi_clk  = 1000 / MM_CLK;
		//t_hact_ipi = frame_width * t_ipi_clk;
		t_hact_ipi = frame_width * 1000 / MM_CLK;
		if (tx_params->IsCphy) { //c-phy
			temp = 7000;
			t_ppi_clk = temp / ap_data_rate;
		//t_hact_ppi = ((6 + frame_width * 3) / (n_lanes + 1) / 2) * t_ppi_clk;
			t_hact_ppi = ((6 + frame_width * 3) * temp /
				ap_data_rate / (lanes + 1) / 2);
		} else {
			temp = 8000;
			t_ppi_clk  = temp / ap_data_rate;
		//t_hact_ppi = ((6 + frame_width * 3) / (n_lanes + 1)) * t_ppi_clk;
			t_hact_ppi = ((6 + frame_width * 3) * temp / ap_data_rate /
				(lanes + 1));
		}

		if (t_hact_ppi > t_hact_ipi)
//ipi_tx_delay_qst = ((t_hact_ppi - t_hact_ipi) / t_ipi_clk + 20 * (t_ppi_clk / t_ipi_clk) + 4);
//ipi_tx_delay_qst = ((t_hact_ppi - t_hact_ipi) * MM_CLK / 1000 + 20 *
//(temp * MM_CLK / tx_data_rate / 1000) + 4);
			ipi_tx_delay_qst = ((t_hact_ppi - t_hact_ipi) * MM_CLK +
					20 * temp * MM_CLK / ap_data_rate) / 1000 + 4;
		else
		//ipi_tx_delay_qst =  (20 * (temp * MM_CLK / tx_data_rate / 1000) + 4);
			ipi_tx_delay_qst =  20 * temp * MM_CLK /
				ap_data_rate / 1000 + 4;

		DISPINFO("ap_tx_data_rate=%d, temp=%d, t_ppi_clk=%d, t_ipi_clk=%d\n",
			ap_data_rate, temp, t_ppi_clk, t_ipi_clk);
		DISPINFO("t_hact_ppi=%d, t_hact_ipi=%d\n", t_hact_ppi, t_hact_ipi);

		//t_ipi_tx_delay = ipi_tx_delay_qst_i * t_ipi_clk;
		t_ipi_tx_delay = ipi_tx_delay_qst * 1000 / MM_CLK;

		DISPINFO("ipi_tx_delay_qst=%d, t_ipi_tx_delay=%d\n",
			ipi_tx_delay_qst, t_ipi_tx_delay);

		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_VALID_VC_CFG_OS, 0);
		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_TX_DELAY_OS, ipi_tx_delay_qst);
//	}

	while (timeout) {
		phy_ready = mtk_spi_read((unsigned long)(&OCLA_REG->OCLA_PHY_READY));
		DISPMSG("phy_ready=0x%x, timeout=%d\n", phy_ready, timeout);

		if ((phy_ready & 0x1) == 0x1)
			count++;

		if (count > 5)
			break;

		udelay(1);
		timeout--;
	}

	if (timeout == 0) {
		ret = -1;
		DISPMSG("%s, wait timeout!\n", __func__);
	}

	DISPFUNCEND();
	return ret;
}

void startup_seq_dphy_specific(unsigned int data_rate)
{
	DISPMSG("%s, data_rate=%d\n", __func__, data_rate);
	DISPFUNCSTART();

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_RW_COMMON_7 * 4,
		CORE_DIG_RW_COMMON_7_LANE0_HSRX_WORD_CLK_SEL_GATING_REG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_RW_COMMON_7 * 4,
		CORE_DIG_RW_COMMON_7_LANE1_HSRX_WORD_CLK_SEL_GATING_REG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_RW_COMMON_7 * 4,
				CORE_DIG_RW_COMMON_7_LANE2_HSRX_WORD_CLK_SEL_GATING_REG_MASK,
				0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_RW_COMMON_7 * 4,
		CORE_DIG_RW_COMMON_7_LANE3_HSRX_WORD_CLK_SEL_GATING_REG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_RW_COMMON_7 * 4,
		CORE_DIG_RW_COMMON_7_LANE4_HSRX_WORD_CLK_SEL_GATING_REG_MASK,
		0);

	if (data_rate > RX_V12)
		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		PPI_STARTUP_RW_COMMON_DPHY_7 * 4,
		PPI_STARTUP_RW_COMMON_DPHY_7_DPHY_DDL_CAL_addr_MASK,
		40);
	else
		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		PPI_STARTUP_RW_COMMON_DPHY_7 * 4,
		PPI_STARTUP_RW_COMMON_DPHY_7_DPHY_DDL_CAL_addr_MASK,
		104);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		PPI_STARTUP_RW_COMMON_DPHY_8 * 4,
		PPI_STARTUP_RW_COMMON_DPHY_8_CPHY_DDL_CAL_addr_MASK,
		80);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_0 * 4,
		PPI_RW_DDLCAL_CFG_0_DDLCAL_TIMEBASE_TARGET_MASK,
		125); // cfg_clk = 26 MHz

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_7 * 4,
		PPI_RW_DDLCAL_CFG_7_DDLCAL_DECR_WAIT_MASK,
		34);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_1 * 4,
		PPI_RW_DDLCAL_CFG_1_DDLCAL_DISABLE_TIME_MASK,
		25);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_2 * 4,
		PPI_RW_DDLCAL_CFG_2_DDLCAL_WAIT_MASK,
		4);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_2 * 4,
		PPI_RW_DDLCAL_CFG_2_DDLCAL_TUNE_MODE_MASK,
		2);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_2 * 4,
		PPI_RW_DDLCAL_CFG_2_DDLCAL_DDL_DLL_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_2 * 4,
		PPI_RW_DDLCAL_CFG_2_DDLCAL_ENABLE_WAIT_MASK,
		25); // cfg_clk = 26 MHz

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_2 * 4,
		PPI_RW_DDLCAL_CFG_2_DDLCAL_UPDATE_SETTINGS_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_4 * 4,
		PPI_RW_DDLCAL_CFG_4_DDLCAL_STUCK_THRESH_MASK,
		10);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_6 * 4,
		PPI_RW_DDLCAL_CFG_6_DDLCAL_MAX_DIFF_MASK,
		10);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_7 * 4,
		PPI_RW_DDLCAL_CFG_7_DDLCAL_START_DELAY_MASK,
		12); // cfg_clk = 26 MHz

	if (data_rate > 1500) {
		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_3 * 4,
			PPI_RW_DDLCAL_CFG_3_DDLCAL_COUNTER_REF_MASK,
			ddl_cntr_ref_reg);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_1 * 4,
					PPI_RW_DDLCAL_CFG_1_DDLCAL_MAX_PHASE_MASK,
					max_phase);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_5 * 4,
			PPI_RW_DDLCAL_CFG_5_DDLCAL_DLL_FBK_MASK,
			dll_fbk);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + PPI_RW_DDLCAL_CFG_5 * 4,
			PPI_RW_DDLCAL_CFG_5_DDLCAL_DDL_COARSE_BANK_MASK,
			coarse_bank);
	}

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_8 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_8_OA_LANE0_HSRX_CDPHY_SEL_FAST_MASK,
		sel_fast);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_8 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_8_OA_LANE1_HSRX_CDPHY_SEL_FAST_MASK,
		sel_fast);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_8 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_8_OA_LANE2_HSRX_CDPHY_SEL_FAST_MASK,
		sel_fast);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_8 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_8_OA_LANE3_HSRX_CDPHY_SEL_FAST_MASK,
		sel_fast);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_8 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_8_OA_LANE4_HSRX_CDPHY_SEL_FAST_MASK,
		sel_fast);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_DLANE_0_RW_LP_0 * 4,
		CORE_DIG_DLANE_0_RW_LP_0_LP_0_TTAGO_REG_MASK,
		6);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_DLANE_1_RW_LP_0 * 4,
		CORE_DIG_DLANE_1_RW_LP_0_LP_0_TTAGO_REG_MASK,
		6);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_DLANE_2_RW_LP_0 * 4,
		CORE_DIG_DLANE_2_RW_LP_0_LP_0_TTAGO_REG_MASK,
		6);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_DLANE_3_RW_LP_0 * 4,
		CORE_DIG_DLANE_3_RW_LP_0_LP_0_TTAGO_REG_MASK,
		6);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2_OA_LANE0_SEL_LANE_CFG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2_OA_LANE1_SEL_LANE_CFG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2_OA_LANE2_SEL_LANE_CFG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2_OA_LANE3_SEL_LANE_CFG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2_OA_LANE4_SEL_LANE_CFG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_RW_COMMON_6 * 4,
		CORE_DIG_RW_COMMON_6_DESERIALIZER_EN_DEASS_COUNT_THRESH_D_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + CORE_DIG_RW_COMMON_6 * 4,
		CORE_DIG_RW_COMMON_6_DESERIALIZER_DIV_EN_DELAY_THRESH_D_MASK,
		1);

	if (data_rate > RX_V12) {
		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12_OA_L0_HSRX_DPHY_DDL_BYPASS_EN_OVR_VAL_MASK,
		0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_12 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_12_OA_L1_HSRX_DPHY_DDL_BYPASS_EN_OVR_VAL_MASK,
		0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_12 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_12_OA_L2_HSRX_DPHY_DDL_BYPASS_EN_OVR_VAL_MASK,
		0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_12 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_12_OA_L3_HSRX_DPHY_DDL_BYPASS_EN_OVR_VAL_MASK,
		0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_12 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_12_OA_L4_HSRX_DPHY_DDL_BYPASS_EN_OVR_VAL_MASK,
		0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13_OA_LANE0_HSRX_DPHY_DDL_BYPASS_EN_OVR_EN_MASK,
		0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_13 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_13_OA_LANE1_HSRX_DPHY_DDL_BYPASS_EN_OVR_EN_MASK,
		0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_13 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_13_OA_LANE2_HSRX_DPHY_DDL_BYPASS_EN_OVR_EN_MASK,
		0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_13 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_13_OA_LANE3_HSRX_DPHY_DDL_BYPASS_EN_OVR_EN_MASK,
		0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_13 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_13_OA_LANE4_HSRX_DPHY_DDL_BYPASS_EN_OVR_EN_MASK,
		0);
	} else {
		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12_OA_L0_HSRX_DPHY_DDL_BYPASS_EN_OVR_VAL_MASK,
		1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_12 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_12_OA_L1_HSRX_DPHY_DDL_BYPASS_EN_OVR_VAL_MASK,
		1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_12 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_12_OA_L2_HSRX_DPHY_DDL_BYPASS_EN_OVR_VAL_MASK,
		1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_12 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_12_OA_L3_HSRX_DPHY_DDL_BYPASS_EN_OVR_VAL_MASK,
		1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_12 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_12_OA_L4_HSRX_DPHY_DDL_BYPASS_EN_OVR_VAL_MASK,
		1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13_OA_LANE0_HSRX_DPHY_DDL_BYPASS_EN_OVR_EN_MASK,
		1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_13 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_13_OA_LANE1_HSRX_DPHY_DDL_BYPASS_EN_OVR_EN_MASK,
		1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_13 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_13_OA_LANE2_HSRX_DPHY_DDL_BYPASS_EN_OVR_EN_MASK,
		1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_13 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_13_OA_LANE3_HSRX_DPHY_DDL_BYPASS_EN_OVR_EN_MASK,
		1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_13 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_13_OA_LANE4_HSRX_DPHY_DDL_BYPASS_EN_OVR_EN_MASK,
		1);
	}

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9 * 4,
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9_OA_LANE2_HSRX_HS_CLK_DIV_MASK,
		hsrx_clk_div);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_CLK_RW_HS_RX_0 * 4,
		CORE_DIG_DLANE_CLK_RW_HS_RX_0_HS_RX_0_TCLKSETTLE_REG_MASK,
		28);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_CLK_RW_HS_RX_7 * 4,
		CORE_DIG_DLANE_CLK_RW_HS_RX_7_HS_RX_7_TCLKMISS_REG_MASK,
		6);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_0 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_0_HS_RX_0_THSSETTLE_REG_MASK,
		hs_thssettle);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_0 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_0_HS_RX_0_THSSETTLE_REG_MASK,
		hs_thssettle);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_0 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_0_HS_RX_0_THSSETTLE_REG_MASK,
		hs_thssettle);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_0 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_0_HS_RX_0_THSSETTLE_REG_MASK,
		hs_thssettle);

	if (data_rate > RX_V12) {
		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_0_RW_CFG_1 * 4,
			CORE_DIG_DLANE_0_RW_CFG_1_CFG_1_DESKEW_SUPPORTED_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_1_RW_CFG_1 * 4,
			CORE_DIG_DLANE_1_RW_CFG_1_CFG_1_DESKEW_SUPPORTED_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_2_RW_CFG_1 * 4,
			CORE_DIG_DLANE_2_RW_CFG_1_CFG_1_DESKEW_SUPPORTED_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_3_RW_CFG_1 * 4,
			CORE_DIG_DLANE_3_RW_CFG_1_CFG_1_DESKEW_SUPPORTED_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_0_RW_CFG_1 * 4,
			CORE_DIG_DLANE_0_RW_CFG_1_CFG_1_SOT_DETECTION_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_1_RW_CFG_1 * 4,
			CORE_DIG_DLANE_1_RW_CFG_1_CFG_1_SOT_DETECTION_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_2_RW_CFG_1 * 4,
			CORE_DIG_DLANE_2_RW_CFG_1_CFG_1_SOT_DETECTION_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_3_RW_CFG_1 * 4,
			CORE_DIG_DLANE_3_RW_CFG_1_CFG_1_SOT_DETECTION_REG_MASK,
			0);
	} else {
		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_0_RW_CFG_1 * 4,
			CORE_DIG_DLANE_0_RW_CFG_1_CFG_1_DESKEW_SUPPORTED_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_1_RW_CFG_1 * 4,
			CORE_DIG_DLANE_1_RW_CFG_1_CFG_1_DESKEW_SUPPORTED_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_2_RW_CFG_1 * 4,
			CORE_DIG_DLANE_2_RW_CFG_1_CFG_1_DESKEW_SUPPORTED_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_3_RW_CFG_1 * 4,
			CORE_DIG_DLANE_3_RW_CFG_1_CFG_1_DESKEW_SUPPORTED_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_0_RW_CFG_1 * 4,
			CORE_DIG_DLANE_0_RW_CFG_1_CFG_1_SOT_DETECTION_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_1_RW_CFG_1 * 4,
			CORE_DIG_DLANE_1_RW_CFG_1_CFG_1_SOT_DETECTION_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_2_RW_CFG_1 * 4,
			CORE_DIG_DLANE_2_RW_CFG_1_CFG_1_SOT_DETECTION_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_3_RW_CFG_1 * 4,
			CORE_DIG_DLANE_3_RW_CFG_1_CFG_1_SOT_DETECTION_REG_MASK,
			1);
	}

	if (data_rate > 2500) {
		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_0_RW_HS_RX_2 * 4,
			CORE_DIG_DLANE_0_RW_HS_RX_2_HS_RX_2_IGNORE_ALTERNCAL_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_1_RW_HS_RX_2 * 4,
			CORE_DIG_DLANE_1_RW_HS_RX_2_HS_RX_2_IGNORE_ALTERNCAL_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_2_RW_HS_RX_2 * 4,
			CORE_DIG_DLANE_2_RW_HS_RX_2_HS_RX_2_IGNORE_ALTERNCAL_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_3_RW_HS_RX_2 * 4,
			CORE_DIG_DLANE_3_RW_HS_RX_2_HS_RX_2_IGNORE_ALTERNCAL_REG_MASK,
			0);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_CLK_RW_HS_RX_2 * 4,
			CORE_DIG_DLANE_CLK_RW_HS_RX_2_HS_RX_2_IGNORE_ALTERNCAL_REG_MASK,
			0);
	} else {
		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_0_RW_HS_RX_2 * 4,
			CORE_DIG_DLANE_0_RW_HS_RX_2_HS_RX_2_IGNORE_ALTERNCAL_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_1_RW_HS_RX_2 * 4,
			CORE_DIG_DLANE_1_RW_HS_RX_2_HS_RX_2_IGNORE_ALTERNCAL_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_2_RW_HS_RX_2 * 4,
			CORE_DIG_DLANE_2_RW_HS_RX_2_HS_RX_2_IGNORE_ALTERNCAL_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_3_RW_HS_RX_2 * 4,
			CORE_DIG_DLANE_3_RW_HS_RX_2_HS_RX_2_IGNORE_ALTERNCAL_REG_MASK,
			1);

		mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
			CORE_DIG_DLANE_CLK_RW_HS_RX_2 * 4,
			CORE_DIG_DLANE_CLK_RW_HS_RX_2_HS_RX_2_IGNORE_ALTERNCAL_REG_MASK,
			1);
	}

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_2 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_2_HS_RX_2_UPDATE_SETTINGS_DESKEW_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_2 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_2_HS_RX_2_UPDATE_SETTINGS_DESKEW_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_2 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_2_HS_RX_2_UPDATE_SETTINGS_DESKEW_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_2 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_2_HS_RX_2_UPDATE_SETTINGS_DESKEW_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_CLK_RW_HS_RX_2 * 4,
		CORE_DIG_DLANE_CLK_RW_HS_RX_2_HS_RX_2_UPDATE_SETTINGS_DESKEW_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_1 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_1_HS_RX_1_FILTER_SIZE_DESKEW_REG_MASK,
		16);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_1 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_1_HS_RX_1_FILTER_SIZE_DESKEW_REG_MASK,
		16);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_1 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_1_HS_RX_1_FILTER_SIZE_DESKEW_REG_MASK,
		16);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_1 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_1_HS_RX_1_FILTER_SIZE_DESKEW_REG_MASK,
		16);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_2 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_2_HS_RX_2_WINDOW_SIZE_DESKEW_REG_MASK,
		3);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_2 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_2_HS_RX_2_WINDOW_SIZE_DESKEW_REG_MASK,
		3);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_2 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_2_HS_RX_2_WINDOW_SIZE_DESKEW_REG_MASK,
		3);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_2 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_2_HS_RX_2_WINDOW_SIZE_DESKEW_REG_MASK,
		3);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_CLK_RW_HS_RX_2 * 4,
		CORE_DIG_DLANE_CLK_RW_HS_RX_2_HS_RX_2_WINDOW_SIZE_DESKEW_REG_MASK,
		3);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_3 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_3_HS_RX_3_STEP_SIZE_DESKEW_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_3 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_3_HS_RX_3_STEP_SIZE_DESKEW_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_3 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_3_HS_RX_3_STEP_SIZE_DESKEW_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_3 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_3_HS_RX_3_STEP_SIZE_DESKEW_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_4 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_4_HS_RX_4_MAX_ITERATIONS_DESKEW_REG_MASK,
		150);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_4 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_4_HS_RX_4_MAX_ITERATIONS_DESKEW_REG_MASK,
		150);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_4 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_4_HS_RX_4_MAX_ITERATIONS_DESKEW_REG_MASK,
		150);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_4 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_4_HS_RX_4_MAX_ITERATIONS_DESKEW_REG_MASK,
		150);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_5 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_5_HS_RX_5_DDL_LEFT_INIT_REG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_5 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_5_HS_RX_5_DDL_LEFT_INIT_REG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_5 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_5_HS_RX_5_DDL_LEFT_INIT_REG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_5 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_5_HS_RX_5_DDL_LEFT_INIT_REG_MASK,
		0);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_5 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_5_HS_RX_5_DDL_MID_INIT_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_5 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_5_HS_RX_5_DDL_MID_INIT_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_5 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_5_HS_RX_5_DDL_MID_INIT_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_5 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_5_HS_RX_5_DDL_MID_INIT_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_6 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_6_HS_RX_6_DDL_RIGHT_INIT_REG_MASK,
		2);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_6 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_6_HS_RX_6_DDL_RIGHT_INIT_REG_MASK,
		2);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_6 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_6_HS_RX_6_DDL_RIGHT_INIT_REG_MASK,
		2);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_6 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_6_HS_RX_6_DDL_RIGHT_INIT_REG_MASK,
		2);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_7 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_7_HS_RX_7_DESKEW_AUTO_ALGO_SEL_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_7 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_7_HS_RX_7_DESKEW_AUTO_ALGO_SEL_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_7 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_7_HS_RX_7_DESKEW_AUTO_ALGO_SEL_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_7 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_7_HS_RX_7_DESKEW_AUTO_ALGO_SEL_REG_MASK,
		1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_3 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_3_HS_RX_3_FJUMP_DESKEW_REG_MASK,
		fjump_deskew_reg);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_3 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_3_HS_RX_3_FJUMP_DESKEW_REG_MASK,
		fjump_deskew_reg);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_3 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_3_HS_RX_3_FJUMP_DESKEW_REG_MASK,
		fjump_deskew_reg);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_3 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_3_HS_RX_3_FJUMP_DESKEW_REG_MASK,
		fjump_deskew_reg);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_0_RW_HS_RX_6 * 4,
		CORE_DIG_DLANE_0_RW_HS_RX_6_HS_RX_6_MIN_EYE_OPENING_DESKEW_REG_MASK,
		eye_open_deskew_reg);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_1_RW_HS_RX_6 * 4,
		CORE_DIG_DLANE_1_RW_HS_RX_6_HS_RX_6_MIN_EYE_OPENING_DESKEW_REG_MASK,
		eye_open_deskew_reg);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_2_RW_HS_RX_6 * 4,
		CORE_DIG_DLANE_2_RW_HS_RX_6_HS_RX_6_MIN_EYE_OPENING_DESKEW_REG_MASK,
		eye_open_deskew_reg);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_DLANE_3_RW_HS_RX_6 * 4,
		CORE_DIG_DLANE_3_RW_HS_RX_6_HS_RX_6_MIN_EYE_OPENING_DESKEW_REG_MASK,
		eye_open_deskew_reg);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4, 0x0404);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4, 0x040C);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4, 0x0414);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4, 0x041C);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4, 0x0423);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0429);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0430);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x043A);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0445);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x044A);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0450);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x045A);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0465);

	mtk_spi_write(MIPI_RX_PHY_BASE +
		CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0469);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0472);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x047A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0485);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0489);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0490);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x049A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04A4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04AC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04B4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04BC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04C4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04CC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04D4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04DC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04E4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04EC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04F4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x04FC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0504);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x050C);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0514);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x051C);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0523);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0529);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0530);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x053A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0545);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x054A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0550);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x055A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0565);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0569);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0572);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x057A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0585);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0589);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0590);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x059A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05A4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05AC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05B4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05BC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05C4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05CC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05D4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05DC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05E4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05EC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05F4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x05FC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0604);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x060C);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0614);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x061C);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0623);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0629);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0632);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x063A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0645);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x064A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0650);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x065A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0665);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0669);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0672);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x067A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0685);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0689);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0690);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x069A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06A4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06AC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06B4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06BC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06C4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06CC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06D4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06DC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06E4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06EC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06F4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x06FC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0704);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x070C);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0714);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x071C);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0723);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x072A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0730);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x073A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0745);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x074A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0750);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x075A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0765);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0769);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0772);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x077A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0785);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0789);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x0790);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x079A);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07A4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07AC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07B4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07BC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07C4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07CC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07D4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07DC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07E4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07EC);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07F4);

	mtk_spi_write(MIPI_RX_PHY_BASE + CORE_DIG_COMMON_RW_DESKEW_FINE_MEM * 4,
		0x07FC);

#ifdef _Disable_HS_DCO_
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6 * 4,
		CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6_OA_CB_HSTXLB_DCO_EN_OVR_EN_MASK,
		1);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6 * 4,
		CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6_OA_CB_HSTXLB_DCO_PON_OVR_EN_MASK,
		1);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6 * 4,
		CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6_OA_CB_LP_DCO_EN_OVR_VAL_MASK,
		0);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6 * 4,
		CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6_OA_CB_LP_DCO_PON_OVR_VAL_MASK,
		0);
#endif
#ifdef _Disable_LP_TX_L123_
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1228 * 4,	0x80, 1);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1228 * 4,	0x40, 1);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1428 * 4,	0x80, 1);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1428 * 4,	0x40, 1);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1628 * 4,	0x80, 1);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1628 * 4,	0x40, 1);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1828 * 4,	0x80, 1);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1828 * 4,	0x40, 1);

	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1227 * 4, 0xf, 0);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1427 * 4, 0xf, 0);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1627 * 4, 0xf, 0);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE + 0x1827 * 4, 0xf, 0);
#endif
#ifdef _G_MODE_EN_
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9 * 4, 0x18, 0);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9 * 4, 0x18, 0);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9 * 4, 0x18, 0);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9 * 4, 0x18, 0);
	mtk_spi_mask_field_write(MIPI_RX_PHY_BASE +
		CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9 * 4, 0x18, 0);
#endif

	DISPFUNCEND();
}

/* for debug use */
void output_debug_signal(void)
{
	//Mutex thread 0 remove mod_sof[1]
	mtk_spi_write(0x00025030, 0x0000001D);

	//Mutex thread 1 use IPI_VSYNC falling and mod_sof[1]
	mtk_spi_write(0x00025050, 0x00000002);
	mtk_spi_write(0x0002504C, 0x0000004a);
	mtk_spi_write(0x00025040, 0x00000001);

	//DSI DBG Setting
	mtk_spi_write(0x00021170, 0x00001001);

	//MM DBG Setting
	mtk_spi_write(0x00023300, 0x00000003);
	mtk_spi_write(0x000231a8, 0x00000021);

	//DBGSYS Setting
	mtk_spi_write(0x000076d0, 0x00000001);

	//GPIO Mode
	mtk_spi_write(0x00007310, 0x17711111);
	mtk_spi_write(0x00007300, 0x77701111);
}

void bdg_register_init(void)
{
	DISPFUNCSTART();

	DISPSYS_REG = (struct BDG_DISPSYS_CONFIG_REGS *)DISPSYS_BDG_MMSYS_CONFIG_BASE;
	DSI2_REG = (struct BDG_MIPIDSI2_REGS *)DISPSYS_BDG_MIPIDSI2_DEVICE_BASE;
	SYS_REG = (struct BDG_SYSREG_CTRL_REGS *)DISPSYS_BDG_SYSREG_CTRL_BASE;
	RDMA_REG = (struct BDG_RDMA0_REGS *)DISPSYS_BDG_RDMA0_REGS_BASE;
	MUTEX_REG = (struct BDG_MUTEX_REGS *)DISPSYS_BDG_MUTEX_REGS_BASE;
	OCLA_REG = (struct BDG_OCLA_REGS *)DISPSYS_BDG_OCLA_BASE;
	TX_REG[0] = (struct BDG_TX_REGS *)DISPSYS_BDG_TX_DSI0_BASE;
	MIPI_TX_REG = (struct BDG_MIPI_TX_REGS *)DISPSYS_BDG_MIPI_TX_BASE;
	DSC_REG = (struct BDG_DISP_DSC_REGS *)DISPSYS_BDG_DISP_DSC_BASE;
	APMIXEDSYS = (struct BDG_APMIXEDSYS_REGS *)DISPSYS_BDG_APMIXEDSYS_BASE;
	TOPCKGEN = (struct BDG_TOPCKGEN_REGS *)DISPSYS_BDG_TOPCKGEN_BASE;
	GCE_REG = (struct BDG_GCE_REGS *)DISPSYS_BDG_GCE_BASE;
	EFUSE = (struct BDG_EFUSE_REGS *)DISPSYS_BDG_EFUSE_BASE;
	GPIO = (struct BDG_GPIO_REGS *)DISPSYS_BDG_GPIO_BASE;
	TX_CMDQ_REG[0] = (struct DSI_TX_CMDQ_REGS *)(DISPSYS_BDG_TX_DSI0_BASE + 0xd00);
}

int bdg_common_init(enum DISP_BDG_ENUM module,
			struct disp_ddp_path_config *config, void *cmdq)
{
	int ret = 0;
	struct LCM_DSI_PARAMS *tx_params;
	unsigned int ap_data_rate;

	DISPFUNCSTART();

	bdg_register_init();

	clk_buf_disp_ctrl(true);
	mdelay(2);
	bdg_tx_pull_6382_reset_pin();
	spislv_init();
	spislv_switch_speed_hz(SPI_TX_LOW_SPEED_HZ, SPI_RX_LOW_SPEED_HZ);

	set_LDO_on(cmdq);
	set_mtcmos_on(cmdq);
	ana_macro_on(cmdq);
	set_subsys_on(cmdq);

	spislv_switch_speed_hz(SPI_TX_MAX_SPEED_HZ, SPI_RX_MAX_SPEED_HZ);
	// Disable reset sequential de-glitch circuit
	DSI_OUTREG32(cmdq, SYS_REG->RST_DG_CTRL, 0);
	// Set GPIO to active IRQ
	DSI_OUTREGBIT(cmdq, struct GPIO_MODE1_REG, GPIO->GPIO_MODE1, GPIO12, 1);

	tx_params = &(config->dispif_config.dsi);

	// clear CG
//	DSI_OUTREG32(cmdq, DISPSYS_REG->MMSYS_CG_CON0, 0);
	// for release rx mac reset
	DSI_OUTREG32(cmdq, SYS_REG->DISP_MISC0, 1);
	DSI_OUTREG32(cmdq, SYS_REG->DISP_MISC0, 0);
	// switch DDI(cmd mode) or IPI(vdo mode) path
	DSI_OUTREG32(cmdq, DISPSYS_REG->DDI_POST_CTRL, 0x00001100);
	if (tx_params->mode == CMD_MODE) {
		DSI_OUTREGBIT(cmdq, struct GPIO_MODE1_REG, GPIO->GPIO_MODE1, RSV_24, 0x31);
		DSI_OUTREGBIT(cmdq, struct MIPI_RX_POST_CTRL_REG,
				DISPSYS_REG->MIPI_RX_POST_CTRL, MIPI_RX_MODE_SEL, 0);
	} else
		DSI_OUTREGBIT(cmdq, struct MIPI_RX_POST_CTRL_REG,
				DISPSYS_REG->MIPI_RX_POST_CTRL, MIPI_RX_MODE_SEL, 1);

	// RDMA setting
	DSI_OUTREG32(cmdq, RDMA_REG->DISP_RDMA_SIZE_CON_0,
			tx_params->horizontal_active_pixel);
	DSI_OUTREG32(cmdq, RDMA_REG->DISP_RDMA_SIZE_CON_1,
			tx_params->vertical_active_line);

	DSI_OUTREG32(cmdq, RDMA_REG->DISP_RDMA_FIFO_CON, 0xf30 << 16);
	DSI_OUTREG32(cmdq, RDMA_REG->DISP_RDMA_STALL_CG_CON, 0);
	DSI_OUTREGBIT(cmdq, struct DISP_RDMA_SRAM_SEL_REG,
			RDMA_REG->DISP_RDMA_SRAM_SEL, RDMA_SRAM_SEL, 0);
	DSI_OUTREGBIT(cmdq, struct DISP_RDMA_GLOBAL_CON_REG,
			RDMA_REG->DISP_RDMA_GLOBAL_CON, ENGINE_EN, 1);

	// MUTEX setting
	DSI_OUTREG32(cmdq, MUTEX_REG->DISP_MUTEX_INTEN, 0x1 << 16);
	DSI_OUTREG32(cmdq, MUTEX_REG->DISP_MUTEX0_MOD0, 0x1f);
	if (tx_params->mode == CMD_MODE)
		DSI_OUTREG32(cmdq, MUTEX_REG->DISP_MUTEX0_CTL, 0);
	else {
		DSI_OUTREG32(cmdq, MUTEX_REG->DISP_MUTEX0_CTL,
				0x1 << 0 | 0x0 << 3 | 0x1 << 6 | 0x0 << 9);
		DSI_OUTREGBIT(cmdq, struct DISP_MUTEX0_EN_REG,
				MUTEX_REG->DISP_MUTEX0_EN, MUTEX0_EN, 1);
	}

	// DSI-TX setting
	bdg_tx_init(module, config, NULL);
	DSI_OUTREG32(cmdq, TX_REG[0]->DSI_RESYNC_CON, 0x50007);

	// DSC setting
	if (dsc_en) {
		bdg_dsc_init(module, cmdq, tx_params);
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_PT_MEM_EN, 1);
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_EN, 1);
	} else {
	#ifdef DSC_RELAY_MODE_EN
		bdg_dsc_init(module, cmdq, tx_params);
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_PT_MEM_EN, 1);
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_RELAY, 1);
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_EN, 1);
	#else
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_ALL_BYPASS, 1);
	#endif
	}
	ap_data_rate = bg_mipi_clk_change_sta == 0 ? ap_tx_data_rate : ap_tx_dyn_data_rate;
	calculate_datarate_cfgs_rx(ap_data_rate);
	startup_seq_common(cmdq);

	if (tx_params->IsCphy) {
		DISPINFO("%s rx cphy\n", __func__);
//		startup_seq_cphy_specific();
	} else
		startup_seq_dphy_specific(ap_data_rate);

//	output_debug_signal();

	// request eint irq
	bdg_request_eint_irq();
	/***** NFC SRCLKENAI0 Interrupt Handler +++ *****/
	nfc_request_eint_irq();
	/***** NFC SRCLKENAI0 Interrupt Handler --- *****/

	DISPFUNCEND();

	return ret;
}

int bdg_common_deinit(enum DISP_BDG_ENUM module, void *cmdq)
{
	int ret = 0;

	DISPFUNCSTART();

	if (mt6382_init) {
		spislv_switch_speed_hz(SPI_TX_LOW_SPEED_HZ, SPI_RX_LOW_SPEED_HZ);
		set_ana_mipi_dsi_off(cmdq);
		ana_macro_off(cmdq);
		set_mtcmos_off(cmdq);
		set_LDO_off(cmdq);
		clk_buf_disp_ctrl(false);
		mt6382_init = 0;
	} else
		DISPMSG("%s, 6382 not init\n", __func__);

	DISPFUNCEND();

	return ret;
}

int bdg_common_init_for_rx_pat(enum DISP_BDG_ENUM module,
			struct disp_ddp_path_config *config, void *cmdq)
{
	int ret = 0;
	struct LCM_DSI_PARAMS *tx_params;

	DISPFUNCSTART();

	DISPSYS_REG = (struct BDG_DISPSYS_CONFIG_REGS *)DISPSYS_BDG_MMSYS_CONFIG_BASE;
	DSI2_REG = (struct BDG_MIPIDSI2_REGS *)DISPSYS_BDG_MIPIDSI2_DEVICE_BASE;
	SYS_REG = (struct BDG_SYSREG_CTRL_REGS *)DISPSYS_BDG_SYSREG_CTRL_BASE;
	RDMA_REG = (struct BDG_RDMA0_REGS *)DISPSYS_BDG_RDMA0_REGS_BASE;
	MUTEX_REG = (struct BDG_MUTEX_REGS *)DISPSYS_BDG_MUTEX_REGS_BASE;
	OCLA_REG = (struct BDG_OCLA_REGS *)DISPSYS_BDG_OCLA_BASE;
	TX_REG[0] = (struct BDG_TX_REGS *)DISPSYS_BDG_TX_DSI0_BASE;
	MIPI_TX_REG = (struct BDG_MIPI_TX_REGS *)DISPSYS_BDG_MIPI_TX_BASE;
	DSC_REG = (struct BDG_DISP_DSC_REGS *)DISPSYS_BDG_DISP_DSC_BASE;
	APMIXEDSYS = (struct BDG_APMIXEDSYS_REGS *)DISPSYS_BDG_APMIXEDSYS_BASE;
	TOPCKGEN = (struct BDG_TOPCKGEN_REGS *)DISPSYS_BDG_TOPCKGEN_BASE;
	GCE_REG = (struct BDG_GCE_REGS *)DISPSYS_BDG_GCE_BASE;
	EFUSE = (struct BDG_EFUSE_REGS *)DISPSYS_BDG_EFUSE_BASE;
	GPIO = (struct BDG_GPIO_REGS *)DISPSYS_BDG_GPIO_BASE;
	TX_CMDQ_REG[0] = (struct DSI_TX_CMDQ_REGS *)(DISPSYS_BDG_TX_DSI0_BASE + 0xd00);

	clk_buf_disp_ctrl(true);
	mdelay(2);
	bdg_tx_pull_6382_reset_pin();
	set_LDO_on(cmdq);
	set_mtcmos_on(cmdq);
	ana_macro_on(cmdq);
	set_subsys_on(cmdq);

	// Disable reset sequential de-glitch circuit
	DSI_OUTREG32(cmdq, SYS_REG->RST_DG_CTRL, 0);
	// Set GPIO to active IRQ
	DSI_OUTREGBIT(cmdq, struct GPIO_MODE1_REG, GPIO->GPIO_MODE1, GPIO12, 1);

	tx_params = &(config->dispif_config.dsi);

	// clear CG
//	DSI_OUTREG32(cmdq, DISPSYS_REG->MMSYS_CG_CON0, 0);
	// for release rx mac reset
	DSI_OUTREG32(cmdq, SYS_REG->DISP_MISC0, 1);
	DSI_OUTREG32(cmdq, SYS_REG->DISP_MISC0, 0);
	// switch DDI(cmd mode) or IPI(vdo mode) path
	DSI_OUTREG32(cmdq, DISPSYS_REG->DDI_POST_CTRL, 0x00001100);
	if (tx_params->mode == CMD_MODE) {
		DSI_OUTREGBIT(cmdq, struct GPIO_MODE1_REG, GPIO->GPIO_MODE1, RSV_24, 0x31);
		DSI_OUTREGBIT(cmdq, struct MIPI_RX_POST_CTRL_REG,
				DISPSYS_REG->MIPI_RX_POST_CTRL, MIPI_RX_MODE_SEL, 0);
	} else
		DSI_OUTREGBIT(cmdq, struct MIPI_RX_POST_CTRL_REG,
				DISPSYS_REG->MIPI_RX_POST_CTRL, MIPI_RX_MODE_SEL, 1);

	// RDMA setting
	DSI_OUTREG32(cmdq, RDMA_REG->DISP_RDMA_SIZE_CON_0,
			tx_params->horizontal_active_pixel);
	DSI_OUTREG32(cmdq, RDMA_REG->DISP_RDMA_SIZE_CON_1,
			tx_params->vertical_active_line);

	DSI_OUTREG32(cmdq, RDMA_REG->DISP_RDMA_FIFO_CON, 0xf30 << 16);
	DSI_OUTREG32(cmdq, RDMA_REG->DISP_RDMA_STALL_CG_CON, 0);
	DSI_OUTREGBIT(cmdq, struct DISP_RDMA_SRAM_SEL_REG,
			RDMA_REG->DISP_RDMA_SRAM_SEL, RDMA_SRAM_SEL, 0);
	DSI_OUTREGBIT(cmdq, struct DISP_RDMA_GLOBAL_CON_REG,
			RDMA_REG->DISP_RDMA_GLOBAL_CON, ENGINE_EN, 1);

	// MUTEX setting
	DSI_OUTREG32(cmdq, MUTEX_REG->DISP_MUTEX_INTEN, 0x1 << 16);
	DSI_OUTREG32(cmdq, MUTEX_REG->DISP_MUTEX0_MOD0, 0x1f);
	if (tx_params->mode == CMD_MODE)
		DSI_OUTREG32(cmdq, MUTEX_REG->DISP_MUTEX0_CTL, 0);
	else {
		DSI_OUTREG32(cmdq, MUTEX_REG->DISP_MUTEX0_CTL,
				0x1 << 0 | 0x0 << 3 | 0x1 << 6 | 0x0 << 9);
		DSI_OUTREGBIT(cmdq, struct DISP_MUTEX0_EN_REG,
				MUTEX_REG->DISP_MUTEX0_EN, MUTEX0_EN, 1);
	}

	// DSI-TX setting
	bdg_tx_init(module, config, NULL);
	/* panel init*/
	lcm_init(module);
	bdg_tx_set_mode(module, cmdq, tx_params->mode);

	DSI_OUTREG32(cmdq, TX_REG[0]->DSI_RESYNC_CON, 0x50007);

	// DSC setting
	if (dsc_en) {
		bdg_dsc_init(module, cmdq, tx_params);
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_PT_MEM_EN, 1);
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_EN, 1);
	} else {
#ifdef DSC_RELAY_MODE_EN
		bdg_dsc_init(module, cmdq, tx_params);
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_PT_MEM_EN, 1);
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_RELAY, 1);
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_EN, 1);
#else
		DSI_OUTREGBIT(cmdq, struct DISP_DSC_CON_REG,
				DSC_REG->DISP_DSC_CON, DSC_ALL_BYPASS, 1);
#endif
	}

	bdg_tx_start(DISP_BDG_DSI0, NULL);

	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_SOFT_RSTN_OS, 0);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_CFG_OS, 4);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_PIXEL_NUM_OS,
			(tx_params->horizontal_active_pixel));
	if (dsc_en) {
//tx:500M
//		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_HSA_TIME_OS, 145);
//		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_HBP_TIME_OS, 145);
//		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_HLINE_TIME_OS,
//			(1080 + 145 + 145 + 2361));
//tx:760 MHz
		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_HSA_TIME_OS, 22);
		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_HBP_TIME_OS, 22);
		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_HLINE_TIME_OS,
			(1080 + 22 + 22 + 1331));

	} else {
//tx:1G
		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_HSA_TIME_OS, 73);
		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_HBP_TIME_OS, 73);
		DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_HLINE_TIME_OS,
			(1080 + 73 + 73 + 2998));
	}
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_VSA_LINES_OS,
			(tx_params->vertical_sync_active));
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_VBP_LINES_OS,
			(tx_params->vertical_backporch));
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_VFP_LINES_OS,
			(tx_params->vertical_frontporch));
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_VACTIVE_LINES_OS,
			(tx_params->vertical_active_line));
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_IPI_PG_EN_OS, 1);
	DSI_OUTREG32(cmdq, DSI2_REG->DSI2_DEVICE_SOFT_RSTN_OS, 1);

//	output_debug_signal();

	DISPFUNCEND();

	return ret;
}

#if 0
irqreturn_t bdg_eint_irq_handler(int irq, void *data)
{

	return IRQ_HANDLED;
}
#endif

irqreturn_t bdg_eint_thread_handler(int irq, void *data)
{
	unsigned int irq_ctrl3 = 0;

	//DISPMSG("%s, mt6382 enter eint thread\n", __func__);

	irq_ctrl3 = mtk_spi_read((unsigned long)(&SYS_REG->SYSREG_IRQ_CTRL3));
	//DISPMSG("%s, mt6382 irq_ctrl3: (0x%x)\n", irq_ctrl3, __func__);

	if ((irq_ctrl3 & 0x80000000) == 0x80000000) {
		//IRQ mask for MTCMOS_PWR_ACK (reg_irq_mask_set bit31)
		DSI_OUTREGBIT(NULL, struct IRQ_MSK_CLR_SET_REG,
			SYS_REG->IRQ_MSK_SET, REG_31, 1);

		// callback function for checking module's rg status
		// todo..
	}

	if (irq_ctrl3 & BIT(10)) {
		s32 ret;

		ret = cmdq_bdg_irq_handler();
		DISPMSG("%s: irq_ctrl3:%#x ret:%d", __func__, irq_ctrl3, ret);
	}

	// disable irq (can't use disable_irq() in ISR)
	disable_irq_nosync(bdg_eint_irq);

	return IRQ_HANDLED;
}

void bdg_request_eint_irq(void)
{
	struct device_node *node;

	if (irq_already_requested) {
		enable_irq(bdg_eint_irq);
		return;
	}

	// get compatible node
	node = of_find_compatible_node(NULL, NULL, "mediatek, mt6382_eint");
	if (!node) {
		DISPMSG("%s, mt6382 can't find mt6382_eint compatible node\n", __func__);
		return;
	}

	// get irq number
	bdg_eint_irq = irq_of_parse_and_map(node, 0);
	DISPMSG("%s, mt6382 EINT irq number: (%d)\n", __func__, bdg_eint_irq);

	// register irq thread handler
	if (request_threaded_irq(bdg_eint_irq, NULL/*dbg_eint_irq_handler*/,
				bdg_eint_thread_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"MT6382_EINT", NULL)) {
		DISPMSG("%s, mt6382 request EINT irq failed!\n", __func__);
		return;
	}

	irq_already_requested = true;

	// enable irq
	enable_irq(bdg_eint_irq);
}

/***** NFC SRCLKENAI0 Interrupt Handler +++ *****/
void nfc_work_func(void)
{
	int nfc_srclk;

	nfc_srclk = gpio_get_value(mt6382_nfc_srclk);
	//DISPMSG("%s, NFC SRCLK GPIO Value = %d\n", __func__, nfc_srclk);

	//suspend need to disable MT6382 first and enable SRCLK first
	if (nfc_srclk != mt6382_nfc_gpio_value) { //the state of gpio has been updated
		mt6382_nfc_gpio_value = nfc_srclk;

		if (nfc_srclk == 1) {
			//switch the mt6382 clock
			//DISPMSG("%s, NFC SRCLK switch the display clock = %d\n",
			//	__func__, nfc_srclk);
			bdg_clk_buf_nfc(nfc_srclk);
		} else {
			//switch the mt6382 clock
			//DISPMSG("%s, NFC SRCLK switch the display clock = %d\n",
			//	__func__, nfc_srclk);
			bdg_clk_buf_nfc(nfc_srclk);
		}
	}
}

irqreturn_t nfc_eint_thread_handler(int irq, void *data)
{
	nfc_work_func();

	return IRQ_HANDLED;
}

void nfc_request_eint_irq(void)
{
	struct device_node *node;

	if (nfc_irq_already_requested) {
		enable_irq(nfc_eint_irq);
		return;
	}

	// get compatible node
	node = of_find_compatible_node(NULL, NULL, "mediatek, mt6382_nfc-eint");
	if (!node) {
		DISPMSG("%s, mt6382 can't find mt6382_nfc_eint compatible node\n", __func__);
		return;
	}

	//get gpio
	mt6382_nfc_srclk = of_get_named_gpio(node, "mt6382_nfc_srclk", 0);
	if (mt6382_nfc_srclk < 0)
		DISPMSG("%s: get NFC SRCLK GPIO failed (%d)", __func__, mt6382_nfc_srclk);
	else
		DISPMSG("%s: get NFC SRCLK GPIO Success (%d)", __func__, mt6382_nfc_srclk);

	// get irq number
	nfc_eint_irq = irq_of_parse_and_map(node, 0);
	DISPMSG("%s, mt6382 NFC EINT irq number: (%d)\n", __func__, nfc_eint_irq);

	// register irq thread handler
	if (request_threaded_irq(nfc_eint_irq, NULL/*dbg_eint_irq_handler*/,
				nfc_eint_thread_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"MT6382_NFC_EINT", NULL)) {
		DISPMSG("%s, mt6382 request NFC EINT irq failed!\n", __func__);
		return;
	}

	nfc_irq_already_requested = true;
	mt6382_nfc_gpio_value = 0;

	//get SRCLK status
	nfc_work_func();

	// enable irq
	enable_irq(nfc_eint_irq);
	// enable irq wake
	irq_set_irq_wake(nfc_eint_irq, 1);
}
/***** NFC SRCLKENAI0 Interrupt Handler --- *****/

#if 0
void bdg_free_eint_irq(void)
{
	free_irq(mt6382_eint_irq, NULL);
}
#endif

void bdg_dsi_mipi_clk_change(enum DISP_BDG_ENUM module, void *handle, int clk)
{
	unsigned int pcw_ratio = 0;
	unsigned int pcw = 0;
	unsigned int pcw_floor = 0;
	unsigned int posdiv    = 0;
	unsigned int prediv    = 0;
	unsigned int status = 0;

	if (clk != 0) {
		unsigned int tmp = 0;

		if (clk > 2500) {
			DISPINFO("mipitx Data Rate exceed limit(%d)\n",
			clk);
			ASSERT(0);
		} else if (clk >= 2000) { /* 2G ~ 2.5G */
			pcw_ratio = 1;
			posdiv    = 0;
			prediv    = 0;
		} else if (clk >= 1000) { /* 1G ~ 2G */
			pcw_ratio = 2;
			posdiv    = 1;
			prediv    = 0;
		} else if (clk >= 500) { /* 500M ~ 1G */
			pcw_ratio = 4;
			posdiv    = 2;
			prediv    = 0;
		} else if (clk > 250) { /* 250M ~ 500M */
			pcw_ratio = 8;
			posdiv    = 3;
			prediv    = 0;
		} else if (clk >= 125) { /* 125M ~ 250M */
			pcw_ratio = 16;
			posdiv    = 4;
			prediv    = 0;
		} else {
			DISPINFO("dataRate is too low(%d)\n", clk);
			ASSERT(0);
		}

		pcw = clk * pcw_ratio / 26;
		pcw_floor = clk * pcw_ratio % 26;
		tmp = ((pcw & 0xFF) << 24) |
		(((256 * pcw_floor / 26) & 0xFF) << 16) |
		(((256 * (256 * pcw_floor % 26) / 26) & 0xFF) << 8) |
		((256 * (256 * (256 * pcw_floor % 26) % 26) / 26)
		& 0xFF);

		DSI_OUTREG32(handle, MIPI_TX_REG->MIPI_TX_PLL_CON0, tmp);
		DSI_OUTREGBIT(handle, struct MIPI_TX_PLL_CON1_REG,
				MIPI_TX_REG->MIPI_TX_PLL_CON1,
				RG_DSI_PLL_POSDIV, posdiv);

		status = mtk_spi_read((unsigned long)(&MIPI_TX_REG->MIPI_TX_PLL_CON1));
		if ((status & 0x1) == 0x1)
			DSI_OUTREGBIT(handle, struct MIPI_TX_PLL_CON1_REG,
				MIPI_TX_REG->MIPI_TX_PLL_CON1,
				RG_DSI_PLL_SDM_PCW_CHG, 0);
		else
			DSI_OUTREGBIT(handle, struct MIPI_TX_PLL_CON1_REG,
				MIPI_TX_REG->MIPI_TX_PLL_CON1,
				RG_DSI_PLL_SDM_PCW_CHG, 1);
	}
}

int bdg_dsi_porch_setting(enum DISP_BDG_ENUM module, void *handle,
	 unsigned int value)
{
	int  ret = 0;

	DSI_OUTREG32(handle, TX_REG[0]->DSI_TX_HBP_WC, value);
	DISPINFO("set dsi0 hbp to %d\n", value);

	return ret;
}

int bdg_mipi_clk_change(int msg, int en)
{
	unsigned int data_rate = 0;
	unsigned int dsi_hbp = 0; /* adaptive HBP value */

	if (bg_mipi_clk_change_sta == en) {
		DISPMSG("%s, hopping status not change\n", __func__);
		return 0;
	}

	data_rate = get_bdg_dyn_data_rate(en);
	if (bdg_tx_mode != 0) {
		if (en) {
			data_rate = 1030;
			dsi_hbp = 0x47;
		} else {
			data_rate = 1020;
			dsi_hbp = 0x38;
		}
	}
	DISPMSG("%s, bdg_tx_data_rate=%d\n", __func__, data_rate);
	/* wait 6382 dsi revsync state */
	if (bdg_tx_mode == CMD_MODE)
		bdg_tx_wait_for_idle(DISP_BDG_DSI0);
	else
		polling_status();

	/* change mipi clk & hbp porch params*/
	bdg_dsi_mipi_clk_change(DISP_BDG_DSI0, NULL, data_rate);
	if (bdg_tx_mode != CMD_MODE)
		bdg_dsi_porch_setting(DISP_BDG_DSI0, NULL, dsi_hbp);

	/* mipi clk setting need 28us */
	udelay(28);
	bg_mipi_clk_change_sta = en;

	return 0;
}
