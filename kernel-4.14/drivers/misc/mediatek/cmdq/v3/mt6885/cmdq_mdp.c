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
#include "cmdq_reg.h"
#include "cmdq_mdp_common.h"
#ifdef CMDQ_MET_READY
#include <linux/met_drv.h>
#endif
#include <linux/slab.h>
#ifdef COFNIG_MTK_IOMMU
#include "mtk_iommu.h"
#elif defined(CONFIG_MTK_M4U)
#include "m4u.h"
#endif
#ifdef CONFIG_MTK_SMI_EXT
#include "smi_public.h"
#endif

#include "cmdq_device.h"
struct CmdqMdpModuleBaseVA {
	long MDP_RDMA0;
	long MDP_RDMA1;
	long MDP_RDMA2;
	long MDP_RDMA3;
	long MDP_FG0;
	long MDP_FG1;
	long MDP_HDR0;
	long MDP_HDR1;
	long MDP_COLOR0;
	long MDP_COLOR1;
	long MDP_AAL0;
	long MDP_AAL1;
	long MDP_AAL2;
	long MDP_AAL3;
	long MDP_RSZ0;
	long MDP_RSZ1;
	long MDP_RSZ2;
	long MDP_RSZ3;
	long MDP_TDSHP0;
	long MDP_TDSHP1;
	long MDP_TDSHP2;
	long MDP_TDSHP3;
	long MDP_TCC0;
	long MDP_TCC1;
	long MDP_TCC2;
	long MDP_TCC3;
	long MDP_WROT0;
	long MDP_WROT1;
	long MDP_WROT2;
	long MDP_WROT3;
	long VENC;
	long MM_MUTEX;
};
static struct CmdqMdpModuleBaseVA gCmdqMdpModuleBaseVA;

struct CmdqMdpModuleClock {
	struct clk *clk_CAM_MDP;
	struct clk *clk_IMG_DL_RELAY;
	struct clk *clk_IMG_DL_ASYNC_TOP;
	struct clk *clk_MDP_RDMA0;
	struct clk *clk_MDP_RDMA1;
	struct clk *clk_MDP_RDMA2;
	struct clk *clk_MDP_RDMA3;
	struct clk *clk_MDP_FG0;
	struct clk *clk_MDP_FG1;
	struct clk *clk_MDP_HDR0;
	struct clk *clk_MDP_HDR1;
	struct clk *clk_MDP_COLOR0;
	struct clk *clk_MDP_COLOR1;
	struct clk *clk_MDP_AAL0;
	struct clk *clk_MDP_AAL1;
	struct clk *clk_MDP_AAL2;
	struct clk *clk_MDP_AAL3;
	struct clk *clk_MDP_RSZ0;
	struct clk *clk_MDP_RSZ1;
	struct clk *clk_MDP_RSZ2;
	struct clk *clk_MDP_RSZ3;
	struct clk *clk_MDP_TDSHP0;
	struct clk *clk_MDP_TDSHP1;
	struct clk *clk_MDP_TDSHP2;
	struct clk *clk_MDP_TDSHP3;
	struct clk *clk_MDP_TCC0;
	struct clk *clk_MDP_TCC1;
	struct clk *clk_MDP_TCC2;
	struct clk *clk_MDP_TCC3;
	struct clk *clk_MDP_WROT0;
	struct clk *clk_MDP_WROT1;
	struct clk *clk_MDP_WROT2;
	struct clk *clk_MDP_WROT3;
};
static struct CmdqMdpModuleClock gCmdqMdpModuleClock;
#define IMP_ENABLE_MDP_HW_CLOCK(FN_NAME, HW_NAME)	\
uint32_t cmdq_mdp_enable_clock_##FN_NAME(bool enable)	\
{		\
	return cmdq_dev_enable_device_clock(enable,	\
		gCmdqMdpModuleClock.clk_##HW_NAME, #HW_NAME "-clk");	\
}
#define IMP_MDP_HW_CLOCK_IS_ENABLE(FN_NAME, HW_NAME)	\
bool cmdq_mdp_clock_is_enable_##FN_NAME(void)	\
{		\
	return cmdq_dev_device_clock_is_enable(		\
		gCmdqMdpModuleClock.clk_##HW_NAME);	\
}

IMP_ENABLE_MDP_HW_CLOCK(CAM_MDP, CAM_MDP);
IMP_ENABLE_MDP_HW_CLOCK(IMG_DL_RELAY, IMG_DL_RELAY);
IMP_ENABLE_MDP_HW_CLOCK(IMG_DL_ASYNC_TOP, IMG_DL_ASYNC_TOP);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RDMA0, MDP_RDMA0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RDMA1, MDP_RDMA1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RDMA2, MDP_RDMA2);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RDMA3, MDP_RDMA3);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RSZ0, MDP_RSZ0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RSZ1, MDP_RSZ1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RSZ2, MDP_RSZ2);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RSZ3, MDP_RSZ3);
IMP_ENABLE_MDP_HW_CLOCK(MDP_FG0, MDP_FG0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_FG1, MDP_FG1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_HDR0, MDP_HDR0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_HDR1, MDP_HDR1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_COLOR0, MDP_COLOR0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_COLOR1, MDP_COLOR1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_WROT0, MDP_WROT0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_WROT1, MDP_WROT1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_WROT2, MDP_WROT2);
IMP_ENABLE_MDP_HW_CLOCK(MDP_WROT3, MDP_WROT3);
IMP_ENABLE_MDP_HW_CLOCK(MDP_TDSHP0, MDP_TDSHP0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_TDSHP1, MDP_TDSHP1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_TDSHP2, MDP_TDSHP2);
IMP_ENABLE_MDP_HW_CLOCK(MDP_TDSHP3, MDP_TDSHP3);
IMP_ENABLE_MDP_HW_CLOCK(MDP_TCC0, MDP_TCC0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_TCC1, MDP_TCC1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_TCC2, MDP_TCC2);
IMP_ENABLE_MDP_HW_CLOCK(MDP_TCC3, MDP_TCC3);
IMP_ENABLE_MDP_HW_CLOCK(MDP_AAL0, MDP_AAL0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_AAL1, MDP_AAL1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_AAL2, MDP_AAL2);
IMP_ENABLE_MDP_HW_CLOCK(MDP_AAL3, MDP_AAL3);
IMP_MDP_HW_CLOCK_IS_ENABLE(CAM_MDP, CAM_MDP);
IMP_MDP_HW_CLOCK_IS_ENABLE(IMG_DL_RELAY, IMG_DL_RELAY);
IMP_MDP_HW_CLOCK_IS_ENABLE(IMG_DL_ASYNC_TOP, IMG_DL_ASYNC_TOP);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RDMA0, MDP_RDMA0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RDMA1, MDP_RDMA1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RDMA2, MDP_RDMA2);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RDMA3, MDP_RDMA3);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RSZ0, MDP_RSZ0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RSZ1, MDP_RSZ1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RSZ2, MDP_RSZ2);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RSZ3, MDP_RSZ3);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_HDR0, MDP_HDR0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_HDR1, MDP_HDR1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_FG0, MDP_FG0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_FG1, MDP_FG1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_COLOR0, MDP_COLOR0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_COLOR1, MDP_COLOR1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_WROT0, MDP_WROT0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_WROT1, MDP_WROT1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_WROT2, MDP_WROT2);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_WROT3, MDP_WROT3);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_TDSHP0, MDP_TDSHP0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_TDSHP1, MDP_TDSHP1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_TDSHP2, MDP_TDSHP2);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_TDSHP3, MDP_TDSHP3);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_TCC0, MDP_TCC0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_TCC1, MDP_TCC1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_TCC2, MDP_TCC2);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_TCC3, MDP_TCC3);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_AAL0, MDP_AAL0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_AAL1, MDP_AAL1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_AAL2, MDP_AAL2);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_AAL3, MDP_AAL3);
#undef IMP_ENABLE_MDP_HW_CLOCK
#undef IMP_MDP_HW_CLOCK_IS_ENABLE

static const uint64_t gCmdqEngineGroupBits[CMDQ_MAX_GROUP_COUNT] = {
	CMDQ_ENG_ISP_GROUP_BITS,
	CMDQ_ENG_MDP_GROUP_BITS,
	CMDQ_ENG_DISP_GROUP_BITS,
	CMDQ_ENG_JPEG_GROUP_BITS,
	CMDQ_ENG_VENC_GROUP_BITS,
	CMDQ_ENG_DPE_GROUP_BITS,
	CMDQ_ENG_RSC_GROUP_BITS,
	CMDQ_ENG_GEPF_GROUP_BITS,
	CMDQ_ENG_EAF_GROUP_BITS
};


long cmdq_mdp_get_module_base_VA_MDP_RDMA0(void)
{
	return gCmdqMdpModuleBaseVA.MDP_RDMA0;
}

long cmdq_mdp_get_module_base_VA_MDP_RDMA1(void)
{
	return gCmdqMdpModuleBaseVA.MDP_RDMA1;
}

long cmdq_mdp_get_module_base_VA_MDP_RDMA2(void)
{
	return gCmdqMdpModuleBaseVA.MDP_RDMA2;
}

long cmdq_mdp_get_module_base_VA_MDP_RDMA3(void)
{
	return gCmdqMdpModuleBaseVA.MDP_RDMA3;
}

long cmdq_mdp_get_module_base_VA_MDP_RSZ0(void)
{
	return gCmdqMdpModuleBaseVA.MDP_RSZ0;
}

long cmdq_mdp_get_module_base_VA_MDP_RSZ1(void)
{
	return gCmdqMdpModuleBaseVA.MDP_RSZ1;
}

long cmdq_mdp_get_module_base_VA_MDP_RSZ2(void)
{
	return gCmdqMdpModuleBaseVA.MDP_RSZ2;
}

long cmdq_mdp_get_module_base_VA_MDP_RSZ3(void)
{
	return gCmdqMdpModuleBaseVA.MDP_RSZ3;
}

long cmdq_mdp_get_module_base_VA_MDP_TDSHP0(void)
{
	return gCmdqMdpModuleBaseVA.MDP_TDSHP0;
}

long cmdq_mdp_get_module_base_VA_MDP_TDSHP1(void)
{
	return gCmdqMdpModuleBaseVA.MDP_TDSHP1;
}

long cmdq_mdp_get_module_base_VA_MDP_TDSHP2(void)
{
	return gCmdqMdpModuleBaseVA.MDP_TDSHP2;
}

long cmdq_mdp_get_module_base_VA_MDP_TDSHP3(void)
{
	return gCmdqMdpModuleBaseVA.MDP_TDSHP3;
}

long cmdq_mdp_get_module_base_VA_MDP_COLOR0(void)
{
	return gCmdqMdpModuleBaseVA.MDP_COLOR0;
}

long cmdq_mdp_get_module_base_VA_MDP_COLOR1(void)
{
	return gCmdqMdpModuleBaseVA.MDP_COLOR1;
}

long cmdq_mdp_get_module_base_VA_MDP_WROT0(void)
{
	return gCmdqMdpModuleBaseVA.MDP_WROT0;
}

long cmdq_mdp_get_module_base_VA_MDP_WROT1(void)
{
	return gCmdqMdpModuleBaseVA.MDP_WROT1;
}

long cmdq_mdp_get_module_base_VA_MDP_WROT2(void)
{
	return gCmdqMdpModuleBaseVA.MDP_WROT2;
}

long cmdq_mdp_get_module_base_VA_MDP_WROT3(void)
{
	return gCmdqMdpModuleBaseVA.MDP_WROT3;
}

long cmdq_mdp_get_module_base_VA_MDP_AAL0(void)
{
	return gCmdqMdpModuleBaseVA.MDP_AAL0;
}

long cmdq_mdp_get_module_base_VA_MDP_AAL1(void)
{
	return gCmdqMdpModuleBaseVA.MDP_AAL1;
}

long cmdq_mdp_get_module_base_VA_MDP_AAL2(void)
{
	return gCmdqMdpModuleBaseVA.MDP_AAL2;
}

long cmdq_mdp_get_module_base_VA_MDP_AAL3(void)
{
	return gCmdqMdpModuleBaseVA.MDP_AAL3;
}

long cmdq_mdp_get_module_base_VA_MDP_TCC0(void)
{
	return gCmdqMdpModuleBaseVA.MDP_TCC0;
}

long cmdq_mdp_get_module_base_VA_MDP_TCC1(void)
{
	return gCmdqMdpModuleBaseVA.MDP_TCC1;
}

long cmdq_mdp_get_module_base_VA_MDP_TCC2(void)
{
	return gCmdqMdpModuleBaseVA.MDP_TCC2;
}

long cmdq_mdp_get_module_base_VA_MDP_TCC3(void)
{
	return gCmdqMdpModuleBaseVA.MDP_TCC3;
}

long cmdq_mdp_get_module_base_VA_MDP_HDR0(void)
{
	return gCmdqMdpModuleBaseVA.MDP_HDR0;
}

long cmdq_mdp_get_module_base_VA_MDP_HDR1(void)
{
	return gCmdqMdpModuleBaseVA.MDP_HDR1;
}

long cmdq_mdp_get_module_base_VA_MDP_FG0(void)
{
	return gCmdqMdpModuleBaseVA.MDP_FG0;
}

long cmdq_mdp_get_module_base_VA_MDP_FG1(void)
{
	return gCmdqMdpModuleBaseVA.MDP_FG1;
}

long cmdq_mdp_get_module_base_VA_VENC(void)
{
	return gCmdqMdpModuleBaseVA.VENC;
}

long cmdq_mdp_get_module_base_VA_MM_MUTEX(void)
{
	return gCmdqMdpModuleBaseVA.MM_MUTEX;
}

#define MMSYS_CONFIG_BASE	cmdq_mdp_get_module_base_VA_MMSYS_CONFIG()
#define MDP_RDMA0_BASE		cmdq_mdp_get_module_base_VA_MDP_RDMA0()
#define MDP_RDMA1_BASE		cmdq_mdp_get_module_base_VA_MDP_RDMA1()
#define MDP_RDMA2_BASE		cmdq_mdp_get_module_base_VA_MDP_RDMA2()
#define MDP_RDMA3_BASE		cmdq_mdp_get_module_base_VA_MDP_RDMA3()
#define MDP_AAL0_BASE		cmdq_mdp_get_module_base_VA_MDP_AAL0()
#define MDP_AAL1_BASE		cmdq_mdp_get_module_base_VA_MDP_AAL1()
#define MDP_AAL2_BASE		cmdq_mdp_get_module_base_VA_MDP_AAL2()
#define MDP_AAL3_BASE		cmdq_mdp_get_module_base_VA_MDP_AAL3()
#define MDP_FG0_BASE		cmdq_mdp_get_module_base_VA_MDP_FG0()
#define MDP_FG1_BASE		cmdq_mdp_get_module_base_VA_MDP_FG1()
#define MDP_HDR0_BASE		cmdq_mdp_get_module_base_VA_MDP_HDR0()
#define MDP_HDR1_BASE		cmdq_mdp_get_module_base_VA_MDP_HDR1()
#define MDP_TCC0_BASE		cmdq_mdp_get_module_base_VA_MDP_TCC0()
#define MDP_TCC1_BASE		cmdq_mdp_get_module_base_VA_MDP_TCC1()
#define MDP_TCC2_BASE		cmdq_mdp_get_module_base_VA_MDP_TCC2()
#define MDP_TCC3_BASE		cmdq_mdp_get_module_base_VA_MDP_TCC3()
#define MDP_RSZ0_BASE		cmdq_mdp_get_module_base_VA_MDP_RSZ0()
#define MDP_RSZ1_BASE		cmdq_mdp_get_module_base_VA_MDP_RSZ1()
#define MDP_RSZ2_BASE		cmdq_mdp_get_module_base_VA_MDP_RSZ2()
#define MDP_RSZ3_BASE		cmdq_mdp_get_module_base_VA_MDP_RSZ3()
#define MDP_TDSHP0_BASE		cmdq_mdp_get_module_base_VA_MDP_TDSHP0()
#define MDP_TDSHP1_BASE		cmdq_mdp_get_module_base_VA_MDP_TDSHP1()
#define MDP_TDSHP2_BASE		cmdq_mdp_get_module_base_VA_MDP_TDSHP2()
#define MDP_TDSHP3_BASE		cmdq_mdp_get_module_base_VA_MDP_TDSHP3()
#define MDP_COLOR0_BASE		cmdq_mdp_get_module_base_VA_MDP_COLOR0()
#define MDP_COLOR1_BASE		cmdq_mdp_get_module_base_VA_MDP_COLOR1()
#define MDP_WROT0_BASE		cmdq_mdp_get_module_base_VA_MDP_WROT0()
#define MDP_WROT1_BASE		cmdq_mdp_get_module_base_VA_MDP_WROT1()
#define MDP_WROT2_BASE		cmdq_mdp_get_module_base_VA_MDP_WROT2()
#define MDP_WROT3_BASE		cmdq_mdp_get_module_base_VA_MDP_WROT3()
#define VENC_BASE			cmdq_mdp_get_module_base_VA_VENC()
#define MM_MUTEX_BASE		cmdq_mdp_get_module_base_VA_MM_MUTEX()

struct RegDef {
	int offset;
	const char *name;
};

void cmdq_mdp_dump_mmsys_config(void)
{
	int i = 0;
	uint32_t value = 0;

	static const struct RegDef configRegisters[] = {
		{0x100,	"MDPSYS_CG_CON0"},
		{0x104,	"MDPSYS_CG_SET0"},
		{0x108,	"MDPSYS_CG_CLR0"},
		{0x110,	"MDPSYS_CG_CON1"},
		{0x114,	"MDPSYS_CG_SET1"},
		{0x118,	"MDPSYS_CG_CLR1"},
		{0x120,	"MDPSYS_CG_CON2"},
		{0x124,	"MDPSYS_CG_SET2"},
		{0x128,	"MDPSYS_CG_CLR2"},
		{0x130,	"MDPSYS_CG_CON3"},
		{0x134,	"MDPSYS_CG_SET3"},
		{0x138,	"MDPSYS_CG_CLR3"},
		{0x140,	"MDPSYS_CG_CON4"},
		{0x144,	"MDPSYS_CG_SET4"},
		{0x148,	"MDPSYS_CG_CLR4"},
		{0x1f4,	"MDPSYS_PROC_TRACK_EMI_BUSY_CON"},
		{0x240,	"MDPSYS_APMCU_GALS_ctrl"},
		{0x300,	"MDPSYS_DEBUG_OUT_SEL"},
		{0x804,	"MDPSYS_SMI_BIST"},
		{0x808,	"MDPSYS_SMI_TX_IDLE"},
		{0x8DC,	"MDPSYS_SMI_LARB_GREQ"},
		{0x8F0,	"MDPSYS_HRT_WEIGHT_READ"},
		{0x900,	"MDPSYS_PWR_METER_CTL0"},
		{0x904,	"MDPSYS_PWR_METER_CTL1"},
		{0x920,	"MDP_DL_RELAY0_CFG_WD"},
		{0x924,	"MDP_DL_RELAY1_CFG_WD"},
		{0x928,	"MDP_DL_RELAY2_CFG_WD"},
		{0x92c,	"MDP_DL_RELAY3_CFG_WD"},
		{0x930,	"MDP_DL_RELAY0_CFG_RD"},
		{0x934,	"MDP_DL_RELAY1_CFG_RD"},
		{0x938,	"MDP_DL_RELAY2_CFG_RD"},
		{0x93C,	"MDP_DL_RELAY3_CFG_RD"},
		{0x940, "MDP_DL_ASYNC_CFG_RD0"},
		{0x948,	"MDP_DL_ASYNC_CFG_RD1"},
		{0x950,	"MDP_DL_ASYNC1_CFG_RD0"},
		{0x954,	"MDP_DL_ASYNC1_CFG_RD1"},
		{0x960,	"MDP_DL_ASYNC2_CFG_RD0"},
		{0x964,	"MDP_DL_ASYNC2_CFG_RD1"},
		{0x970,	"MDP_DL_ASYNC3_CFG_RD0"},
		{0x974,	"MDP_DL_ASYNC3_CFG_RD1"},
		{0xE00,	"MDPSYS_BUF_UNDERRUN"},
		{0xE04,	"MDPSYS_BUF_UNDERRUN_ID"},
		{0xF10,	"ISP0_MOUT_EN"},
		{0xF14,	"ISP1_MOUT_EN"},
		{0xF18,	"ISP2_MOUT_EN"},
		{0xF1C,	"ISP3_MOUT_EN"},
		{0xF20,	"MDP_RDMA0_MOUT_EN"},
		{0xF24,	"MDP_RDMA1_MOUT_EN"},
		{0xF28,	"MDP_RDMA2_MOUT_EN"},
		{0xF2C,	"MDP_RDMA3_MOUT_EN"},
		{0xF30,	"MDP_PQ0_SEL_IN"},
		{0xF34,	"MDP_PQ1_SEL_IN"},
		{0xF38,	"MDP_PQ2_SEL_IN"},
		{0xF3C,	"MDP_PQ3_SEL_IN"},
		{0xF40,	"MDP_PQ0_SOUT_SEL"},
		{0xF44,	"MDP_PQ1_SOUT_SEL"},
		{0xF48,	"MDP_AAL0_MOUT_EN"},
		{0xF4C,	"MDP_AAL1_MOUT_EN"},
		{0xF50,	"MDP_TCC0_SOUT_SEL"},
		{0xF54,	"MDP_TCC1_SOUT_SEL"},
		{0xF58,	"MDP_HDR0_SEL_IN"},
		{0xF5C,	"MDP_HDR1_SEL_IN"},
		{0xF60,	"MDP_RSZ0_SEL_IN"},
		{0xF64,	"MDP_RSZ1_SEL_IN"},
		{0xF68,	"MDP_RSZ2_SEL_IN"},
		{0xF6C,	"MDP_RSZ3_SEL_IN"},
		{0xF70,	"MDP_WROT0_SEL_IN"},
		{0xF74,	"MDP_WROT1_SEL_IN"},
		{0xF78,	"MDP_WROT2_SEL_IN"},
		{0xF7C,	"MDP_WROT3_SEL_IN"},
		{0xFD0,	"MDPSYS_MOUT_MASK0"},
		{0xFD4,	"MDPSYS_MOUT_MASK1"},
		{0xFD8,	"MDPSYS_MOUT_MASK2"},
		{0xFE0,	"MDPSYS_DL_VALID0"},
		{0xFE4,	"MDPSYS_DL_VALID1"},
		{0xFE8,	"MDPSYS_DL_VALID2"},
		{0xFF0,	"MDPSYS_DL_READY0"},
		{0xFF4,	"MDPSYS_DL_READY1"},
		{0xFF8,	"MDPSYS_DL_READY2"},
		{0x0F0, "MMSYS_MISC"},
		{0x0F4,	"MDPSYS_MODULE_DBG"}
	};

	if (!MMSYS_CONFIG_BASE) {
		CMDQ_ERR("mmsys not porting\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(configRegisters); i++) {
		value = CMDQ_REG_GET32(MMSYS_CONFIG_BASE +
			configRegisters[i].offset);
		CMDQ_ERR("%s: 0x%08x\n", configRegisters[i].name, value);
	}

	/*DISP_MUTEX MOD*/
	value = CMDQ_REG_GET32(MM_MUTEX_BASE + 0x0D0);
	CMDQ_ERR("%s: 0x%08x\n", "DISP_MUTEX5_MOD0", value);
}

int32_t cmdq_mdp_reset_with_mmsys(const uint64_t engineToResetAgain)
{
	long MMSYS_SW0_RST_B_REG = MMSYS_CONFIG_BASE + (0x700);
	long MMSYS_SW1_RST_B_REG = MMSYS_CONFIG_BASE + (0x704);
	int i = 0;
	uint64_t reset_bits0 = 0ULL;
	uint64_t reset_bits1 = 0ULL;
	int engineResetBit[48] = {
		CMDQ_ENG_MDP_RDMA0,	/* bit  0 : MDP_RDMA0 */
		CMDQ_ENG_MDP_FG0,	/* bit  1 : MDP_FG0 */
		CMDQ_ENG_MDP_HDR0,	/* bit  2 : MDP_HDR0  */
		CMDQ_ENG_MDP_AAL0,	/* bit  3 : MDP_AAL0  */
		CMDQ_ENG_MDP_RSZ0,	/* bit  4 : MDP_RSZ0 */
		CMDQ_ENG_MDP_TDSHP0,	/* bit  5 : MDP_TDSHP0 */
		CMDQ_ENG_MDP_TCC0,	/* bit  6 : MDP_TCC0 */
		CMDQ_ENG_MDP_WROT0,	/* bit  7 : MDP_WROT0 */
		-1,//CMDQ_ENG_MDP_RDMA2,	/* bit  8 : MDP_RDMA2 */
		-1,//CMDQ_ENG_MDP_AAL2,	/* bit  9 : MDP_AAL2  */
		-1,//CMDQ_ENG_MDP_RSZ2,	/* bit 10 : MDP_RSZ2 */
		CMDQ_ENG_MDP_COLOR0,	/* bit 11 : MDP_COLOR0 */
		-1,//CMDQ_ENG_MDP_TDSHP2,	/* bit 12 : MDP_TDSHP2 */
		-1,//CMDQ_ENG_MDP_TCC2,	/* bit 13 : MDP_TCC2 */
		-1,//CMDQ_ENG_MDP_WROT2,	/* bit 14 : MDP_WROT2 */
		-1,			/* bit 15 : mutex0_*/
		CMDQ_ENG_MDP_RDMA1,	/* bit 16 : MDP_RDMA1 */
		CMDQ_ENG_MDP_FG1,	/* bit 17 : MDP_FG1 */
		CMDQ_ENG_MDP_HDR1,	/* bit 18 : MDP_HDR1  */
		CMDQ_ENG_MDP_AAL1,	/* bit 19 : MDP_AAL1  */
		CMDQ_ENG_MDP_RSZ1,	/* bit 20 : MDP_RSZ1 */
		CMDQ_ENG_MDP_TDSHP1,	/* bit 21 : MDP_TDSHP1 */
		CMDQ_ENG_MDP_TCC1,	/* bit 22 : MDP_TCC1 */
		CMDQ_ENG_MDP_WROT1,	/* bit 23 : MDP_WROT1 */
		-1,//CMDQ_ENG_MDP_RDMA3,	/* bit 24 : MDP_RDMA3 */
		-1,//CMDQ_ENG_MDP_AAL3,	/* bit 25 : MDP_AAL3  */
		-1,//CMDQ_ENG_MDP_RSZ3,	/* bit 26 : MDP_RSZ3 */
		CMDQ_ENG_MDP_COLOR1,	/* bit 27 : MDP_COLOR1 */
		-1,//CMDQ_ENG_MDP_TDSHP3,	/* bit 28 : MDP_TDSHP3 */
		-1,//CMDQ_ENG_MDP_TCC3,	/* bit 29 : MDP_TCC3 */
		-1,//CMDQ_ENG_MDP_WROT3,	/* bit 30 : MDP_WROT3 */
		-1,			/* bit 31 : apb_bus */
		-1,			/* bit 32 : mmsysram */
		-1,			/* bit 33 : apmcu_gals */
		-1,			/* bit 34 : fake_eng0 */
		-1,			/* bit 35 : fake_eng1 */
		-1,			/* bit 36 : smi0 */
		-1,			/* bit 37 : img_dl_async0 */
		-1,			/* bit 38 : img_dl_async1 */
		-1,			/* bit 39 : img_dl_async2 */
		-1,			/* bit 40 : smi1 */
		-1,			/* bit 41 : img_dl_async3 */
		-1,			/* bit 42 : empty42 */
		-1,			/* bit 43 : empty43 */
		-1,			/* bit 44 : smi2 */
		-1,			/* bit 45 : empty45 */
		-1,			/* bit 46 : empty46 */
		-1,			/* bit 47 : empty47 */
	};

	for (i = 0; i < 32; ++i) {
		if (engineResetBit[i] < 0)
			continue;

		if (engineToResetAgain & (1LL << engineResetBit[i]))
			reset_bits0 |= (1 << i);
	}
	for (i = 32; i < 48; ++i) {
		if (engineResetBit[i] < 0)
			continue;

		if (engineToResetAgain & (1LL << engineResetBit[i]))
			reset_bits1 |= (1ULL << i);
	}

	if (reset_bits0 != 0) {
		/* 0: reset */
		/* 1: not reset */
		/* so we need to reverse the bits */
		reset_bits0 = ~reset_bits0;

		CMDQ_REG_SET32(MMSYS_SW0_RST_B_REG, reset_bits0);
		CMDQ_REG_SET32(MMSYS_SW0_RST_B_REG, ~0);
		/* This takes effect immediately, no need to poll state */
	}
	if (reset_bits1 != 0) {
		/* 0: reset */
		/* 1: not reset */
		/* so we need to reverse the bits */
		reset_bits1 = ~reset_bits1;

		CMDQ_REG_SET32(MMSYS_SW1_RST_B_REG, reset_bits1);
		CMDQ_REG_SET32(MMSYS_SW1_RST_B_REG, ~0);
		/* This takes effect immediately, no need to poll state */
	}

	return 0;
}

#ifdef COFNIG_MTK_IOMMU
mtk_iommu_callback_ret_t cmdq_TranslationFault_callback(
	int port, unsigned int mva, void *data)
{
	char dispatchModel[MDP_DISPATCH_KEY_STR_LEN] = "MDP";

	CMDQ_ERR("================= [MDP M4U] Dump Begin ================\n");
	CMDQ_ERR("[MDP M4U]fault call port=%d, mva=0x%x", port, mva);

	cmdq_core_dump_tasks_info();

	switch (port) {
	case M4U_PORT_MDP_RDMA0:
		cmdq_mdp_dump_rdma(MDP_RDMA0_BASE, "RDMA0");
		break;
	case M4U_PORT_MDP_RDMA1:
		cmdq_mdp_dump_rdma(MDP_RDMA1_BASE, "RDMA1");
		break;
	case M4U_PORT_MDP_WROT0:
		cmdq_mdp_dump_rot(MDP_WROT0_BASE, "WROT0");
		break;
	default:
		CMDQ_ERR("[MDP M4U]fault callback function");
		break;
	}

	CMDQ_ERR(
		"=============== [MDP] Frame Information Begin ====================================\n");
	/* find dispatch module and assign dispatch key */
	cmdq_mdp_check_TF_address(mva, dispatchModel);
	memcpy(data, dispatchModel, sizeof(dispatchModel));
	CMDQ_ERR(
		"=============== [MDP] Frame Information End ====================================\n");
	CMDQ_ERR("================= [MDP M4U] Dump End ================\n");

	return MTK_IOMMU_CALLBACK_HANDLED;
}
#elif defined(CONFIG_MTK_M4U)
enum m4u_callback_ret_t cmdq_TranslationFault_callback(
	int port, unsigned int mva, void *data)
{
	char dispatchModel[MDP_DISPATCH_KEY_STR_LEN] = "MDP";

	CMDQ_ERR("================= [MDP M4U] Dump Begin ================\n");
	CMDQ_ERR("[MDP M4U]fault call port=%d, mva=0x%x", port, mva);

	cmdq_core_dump_tasks_info();

	switch (port) {
	case M4U_PORT_MDP_RDMA0:
		cmdq_mdp_dump_rdma(MDP_RDMA0_BASE, "RDMA0");
		break;
	case M4U_PORT_MDP_RDMA1:
		cmdq_mdp_dump_rdma(MDP_RDMA1_BASE, "RDMA1");
		break;
	case M4U_PORT_MDP_WROT0:
		cmdq_mdp_dump_rot(MDP_WROT0_BASE, "WROT0");
		break;
	default:
		CMDQ_ERR("[MDP M4U]fault callback function");
		break;
	}

	CMDQ_ERR(
		"=============== [MDP] Frame Information Begin ====================================\n");
	/* find dispatch module and assign dispatch key */
	cmdq_mdp_check_TF_address(mva, dispatchModel);
	memcpy(data, dispatchModel, sizeof(dispatchModel));
	CMDQ_ERR(
		"=============== [MDP] Frame Information End ====================================\n");
	CMDQ_ERR(
		"================= [MDP M4U] Dump End ================\n");

	return M4U_CALLBACK_HANDLED;
}
#endif

int32_t cmdqVEncDumpInfo(uint64_t engineFlag, int logLevel)
{
//	if (engineFlag & (1LL << CMDQ_ENG_VIDEO_ENC))
//		cmdq_mdp_dump_venc(VENC_BASE, "VENC");

	return 0;
}

void cmdq_mdp_init_module_base_VA(void)
{
	memset(&gCmdqMdpModuleBaseVA, 0, sizeof(struct CmdqMdpModuleBaseVA));

	gCmdqMdpModuleBaseVA.MDP_RDMA0 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_rdma0");
	gCmdqMdpModuleBaseVA.MDP_RDMA1 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_rdma1");
	gCmdqMdpModuleBaseVA.MDP_RDMA2 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_rdma2");
	gCmdqMdpModuleBaseVA.MDP_RDMA3 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_rdma3");
	gCmdqMdpModuleBaseVA.MDP_RSZ0 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_rsz0");
	gCmdqMdpModuleBaseVA.MDP_RSZ1 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_rsz1");
	gCmdqMdpModuleBaseVA.MDP_RSZ2 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_rsz2");
	gCmdqMdpModuleBaseVA.MDP_RSZ3 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_rsz3");
	gCmdqMdpModuleBaseVA.MDP_WROT0 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_wrot0");
	gCmdqMdpModuleBaseVA.MDP_WROT1 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_wrot1");
	gCmdqMdpModuleBaseVA.MDP_WROT2 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_wrot2");
	gCmdqMdpModuleBaseVA.MDP_WROT3 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_wrot3");
	gCmdqMdpModuleBaseVA.MDP_TDSHP0 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_tdshp0");
	gCmdqMdpModuleBaseVA.MDP_TDSHP1 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_tdshp1");
	gCmdqMdpModuleBaseVA.MDP_TDSHP2 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_tdshp2");
	gCmdqMdpModuleBaseVA.MDP_TDSHP3 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_tdshp3");
	gCmdqMdpModuleBaseVA.MDP_TCC0 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_tcc0");
	gCmdqMdpModuleBaseVA.MDP_TCC1 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_tcc1");
	gCmdqMdpModuleBaseVA.MDP_TCC2 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_tcc2");
	gCmdqMdpModuleBaseVA.MDP_TCC3 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_tcc3");
	gCmdqMdpModuleBaseVA.MDP_AAL0 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_aal0");
	gCmdqMdpModuleBaseVA.MDP_AAL1 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_aal1");
	gCmdqMdpModuleBaseVA.MDP_AAL2 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_aal2");
	gCmdqMdpModuleBaseVA.MDP_AAL3 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_aal3");
	gCmdqMdpModuleBaseVA.MDP_COLOR0 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_color0");
	gCmdqMdpModuleBaseVA.MDP_COLOR1 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_color1");
	gCmdqMdpModuleBaseVA.MDP_HDR0 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_hdr0");
	gCmdqMdpModuleBaseVA.MDP_HDR1 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_hdr1");
	gCmdqMdpModuleBaseVA.MDP_FG0 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_fg0");
	gCmdqMdpModuleBaseVA.MDP_FG1 =
		cmdq_dev_alloc_reference_VA_by_name("mdp_fg1");
	gCmdqMdpModuleBaseVA.VENC =
		cmdq_dev_alloc_reference_VA_by_name("venc");
	gCmdqMdpModuleBaseVA.MM_MUTEX =
		cmdq_dev_alloc_reference_VA_by_name("mdp_mutex");
}

void cmdq_mdp_deinit_module_base_VA(void)
{
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_RDMA0());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_RDMA1());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_RDMA2());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_RDMA3());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_RSZ0());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_RSZ1());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_RSZ2());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_RSZ3());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_WROT0());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_WROT1());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_WROT2());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_WROT3());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_TDSHP0());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_TDSHP1());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_TDSHP2());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_TDSHP3());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_COLOR0());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_COLOR1());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_HDR0());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_HDR1());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_FG0());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_FG1());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_AAL0());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_AAL1());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_AAL2());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_AAL3());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_TCC0());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_TCC1());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_TCC2());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_MDP_TCC3());
	cmdq_dev_free_module_base_VA(cmdq_mdp_get_module_base_VA_VENC());

	memset(&gCmdqMdpModuleBaseVA, 0, sizeof(struct CmdqMdpModuleBaseVA));
}

bool cmdq_mdp_clock_is_on(enum CMDQ_ENG_ENUM engine)
{
	switch (engine) {
	case CMDQ_ENG_MDP_CAMIN:
		return cmdq_mdp_clock_is_enable_CAM_MDP() &&
				cmdq_mdp_clock_is_enable_IMG_DL_RELAY() &&
				cmdq_mdp_clock_is_enable_IMG_DL_ASYNC_TOP();
	case CMDQ_ENG_MDP_RDMA0:
		return cmdq_mdp_clock_is_enable_MDP_RDMA0();
	case CMDQ_ENG_MDP_RDMA1:
		return cmdq_mdp_clock_is_enable_MDP_RDMA1();
	case CMDQ_ENG_MDP_RSZ0:
		return cmdq_mdp_clock_is_enable_MDP_RSZ0();
	case CMDQ_ENG_MDP_RSZ1:
		return cmdq_mdp_clock_is_enable_MDP_RSZ1();
	case CMDQ_ENG_MDP_RSZ2:
		return cmdq_mdp_clock_is_enable_MDP_RSZ2();
	case CMDQ_ENG_MDP_WROT0:
		return cmdq_mdp_clock_is_enable_MDP_WROT0();
	case CMDQ_ENG_MDP_WROT1:
		return cmdq_mdp_clock_is_enable_MDP_WROT1();
	case CMDQ_ENG_MDP_TDSHP0:
		return cmdq_mdp_clock_is_enable_MDP_TDSHP0();
	case CMDQ_ENG_MDP_TDSHP1:
		return cmdq_mdp_clock_is_enable_MDP_TDSHP1();
	case CMDQ_ENG_MDP_AAL0:
		return cmdq_mdp_clock_is_enable_MDP_AAL0();
	case CMDQ_ENG_MDP_AAL1:
		return cmdq_mdp_clock_is_enable_MDP_AAL1();
	case CMDQ_ENG_MDP_TCC0:
		return cmdq_mdp_clock_is_enable_MDP_TCC0();
	case CMDQ_ENG_MDP_TCC1:
		return cmdq_mdp_clock_is_enable_MDP_TCC1();
	case CMDQ_ENG_MDP_COLOR0:
		return cmdq_mdp_clock_is_enable_MDP_COLOR0();
	case CMDQ_ENG_MDP_COLOR1:
		return cmdq_mdp_clock_is_enable_MDP_COLOR1();
	case CMDQ_ENG_MDP_HDR0:
		return cmdq_mdp_clock_is_enable_MDP_HDR0();
	case CMDQ_ENG_MDP_HDR1:
		return cmdq_mdp_clock_is_enable_MDP_HDR1();
	case CMDQ_ENG_MDP_FG0:
		return cmdq_mdp_clock_is_enable_MDP_FG0();
	case CMDQ_ENG_MDP_FG1:
		return cmdq_mdp_clock_is_enable_MDP_FG1();
	default:
		CMDQ_ERR("try to query unknown mdp clock");
		return false;
	}
}

void cmdq_mdp_enable_clock(bool enable, enum CMDQ_ENG_ENUM engine)
{
	switch (engine) {
	case CMDQ_ENG_MDP_CAMIN:
		cmdq_mdp_enable_clock_CAM_MDP(enable);
		cmdq_mdp_enable_clock_IMG_DL_RELAY(enable);
		cmdq_mdp_enable_clock_IMG_DL_ASYNC_TOP(enable);
		break;
	case CMDQ_ENG_MDP_RDMA0:
		cmdq_mdp_enable_clock_MDP_RDMA0(enable);
		break;
	case CMDQ_ENG_MDP_RDMA1:
		cmdq_mdp_enable_clock_MDP_RDMA1(enable);
		break;
		break;
	case CMDQ_ENG_MDP_RSZ0:
		cmdq_mdp_enable_clock_MDP_RSZ0(enable);
		break;
	case CMDQ_ENG_MDP_RSZ1:
		cmdq_mdp_enable_clock_MDP_RSZ1(enable);
		break;
	case CMDQ_ENG_MDP_WROT0:
		cmdq_mdp_enable_clock_MDP_WROT0(enable);
		break;
	case CMDQ_ENG_MDP_WROT1:
		cmdq_mdp_enable_clock_MDP_WROT1(enable);
		break;
	case CMDQ_ENG_MDP_TDSHP0:
		cmdq_mdp_enable_clock_MDP_TDSHP0(enable);
		break;
	case CMDQ_ENG_MDP_TDSHP1:
		cmdq_mdp_enable_clock_MDP_TDSHP1(enable);
		break;
	case CMDQ_ENG_MDP_COLOR0:
		cmdq_mdp_enable_clock_MDP_COLOR0(enable);
		break;
	case CMDQ_ENG_MDP_COLOR1:
		cmdq_mdp_enable_clock_MDP_COLOR1(enable);
		break;
	case CMDQ_ENG_MDP_HDR0:
		cmdq_mdp_enable_clock_MDP_HDR0(enable);
		break;
	case CMDQ_ENG_MDP_HDR1:
		cmdq_mdp_enable_clock_MDP_HDR1(enable);
		break;
	case CMDQ_ENG_MDP_AAL0:
		cmdq_mdp_enable_clock_MDP_AAL0(enable);
		break;
	case CMDQ_ENG_MDP_AAL1:
		cmdq_mdp_enable_clock_MDP_AAL1(enable);
		break;
	case CMDQ_ENG_MDP_TCC0:
		cmdq_mdp_enable_clock_MDP_TCC0(enable);
		break;
	case CMDQ_ENG_MDP_TCC1:
		cmdq_mdp_enable_clock_MDP_TCC1(enable);
		break;
	case CMDQ_ENG_MDP_FG0:
		cmdq_mdp_enable_clock_MDP_FG0(enable);
		break;
	case CMDQ_ENG_MDP_FG1:
		cmdq_mdp_enable_clock_MDP_FG1(enable);
		break;
	default:
		CMDQ_ERR("try to enable unknown mdp clock");
		break;
	}
}

/* Common Clock Framework */
void cmdq_mdp_init_module_clk(void)
{
	cmdq_dev_get_module_clock_by_name("mmsys_config", "CAM_MDP",
		&gCmdqMdpModuleClock.clk_CAM_MDP);
	cmdq_dev_get_module_clock_by_name("mmsys_config", "IMG_DL_RELAY",
		&gCmdqMdpModuleClock.clk_IMG_DL_RELAY);
	cmdq_dev_get_module_clock_by_name("mmsys_config", "IMG_DL_ASYNC_TOP",
		&gCmdqMdpModuleClock.clk_IMG_DL_ASYNC_TOP);
	cmdq_dev_get_module_clock_by_name("mdp_rdma0", "MDP_RDMA0",
		&gCmdqMdpModuleClock.clk_MDP_RDMA0);
	cmdq_dev_get_module_clock_by_name("mdp_rdma1", "MDP_RDMA1",
		&gCmdqMdpModuleClock.clk_MDP_RDMA1);
	cmdq_dev_get_module_clock_by_name("mdp_rdma2", "MDP_RDMA2",
		&gCmdqMdpModuleClock.clk_MDP_RDMA2);
	cmdq_dev_get_module_clock_by_name("mdp_rdma3", "MDP_RDMA3",
		&gCmdqMdpModuleClock.clk_MDP_RDMA3);
	cmdq_dev_get_module_clock_by_name("mdp_rsz0", "MDP_RSZ0",
		&gCmdqMdpModuleClock.clk_MDP_RSZ0);
	cmdq_dev_get_module_clock_by_name("mdp_rsz1", "MDP_RSZ1",
		&gCmdqMdpModuleClock.clk_MDP_RSZ1);
	cmdq_dev_get_module_clock_by_name("mdp_rsz2", "MDP_RSZ2",
		&gCmdqMdpModuleClock.clk_MDP_RSZ2);
	cmdq_dev_get_module_clock_by_name("mdp_rsz3", "MDP_RSZ3",
		&gCmdqMdpModuleClock.clk_MDP_RSZ3);
	cmdq_dev_get_module_clock_by_name("mdp_wrot0", "MDP_WROT0",
		&gCmdqMdpModuleClock.clk_MDP_WROT0);
	cmdq_dev_get_module_clock_by_name("mdp_wrot1", "MDP_WROT1",
		&gCmdqMdpModuleClock.clk_MDP_WROT1);
	cmdq_dev_get_module_clock_by_name("mdp_wrot2", "MDP_WROT2",
		&gCmdqMdpModuleClock.clk_MDP_WROT2);
	cmdq_dev_get_module_clock_by_name("mdp_wrot3", "MDP_WROT3",
		&gCmdqMdpModuleClock.clk_MDP_WROT3);
	cmdq_dev_get_module_clock_by_name("mdp_tdshp0", "MDP_TDSHP0",
		&gCmdqMdpModuleClock.clk_MDP_TDSHP0);
	cmdq_dev_get_module_clock_by_name("mdp_tdshp1", "MDP_TDSHP1",
		&gCmdqMdpModuleClock.clk_MDP_TDSHP1);
	cmdq_dev_get_module_clock_by_name("mdp_tdshp2", "MDP_TDSHP2",
		&gCmdqMdpModuleClock.clk_MDP_TDSHP2);
	cmdq_dev_get_module_clock_by_name("mdp_tdshp3", "MDP_TDSHP3",
		&gCmdqMdpModuleClock.clk_MDP_TDSHP3);
	cmdq_dev_get_module_clock_by_name("mdp_color0", "MDP_COLOR0",
		&gCmdqMdpModuleClock.clk_MDP_COLOR0);
	cmdq_dev_get_module_clock_by_name("mdp_color1", "MDP_COLOR1",
		&gCmdqMdpModuleClock.clk_MDP_COLOR1);
	cmdq_dev_get_module_clock_by_name("mdp_hdr0", "MDP_HDR0",
		&gCmdqMdpModuleClock.clk_MDP_HDR0);
	cmdq_dev_get_module_clock_by_name("mdp_hdr1", "MDP_HDR1",
		&gCmdqMdpModuleClock.clk_MDP_HDR1);
	cmdq_dev_get_module_clock_by_name("mdp_fg0", "MDP_FG0",
		&gCmdqMdpModuleClock.clk_MDP_FG0);
	cmdq_dev_get_module_clock_by_name("mdp_fg1", "MDP_FG1",
		&gCmdqMdpModuleClock.clk_MDP_FG1);
}
/* MDP engine dump */
void cmdq_mdp_dump_rsz(const unsigned long base, const char *label)
{
	uint32_t value[38] = { 0 };
	uint32_t request[4] = { 0 };
	uint32_t state = 0;

	value[0] = CMDQ_REG_GET32(base + 0x000);    /*  RSZ_ENABLE*/
	value[1] = CMDQ_REG_GET32(base + 0x004);    /*  RSZ_CONTROL_1   */
	value[2] = CMDQ_REG_GET32(base + 0x008);    /*  RSZ_CONTROL_2   */
	value[3] = CMDQ_REG_GET32(base + 0x010);    /*  RSZ_INPUT_IMAGE */
	value[4] = CMDQ_REG_GET32(base + 0x014);    /*  RSZ_OUTPUT_IMAGE    */
	value[5] = CMDQ_REG_GET32(base + 0x018);/*  RSZ_HORIZONTAL_COEFF_STEP */
	value[6] = CMDQ_REG_GET32(base + 0x01C);/*  RSZ_VERTICAL_COEFF_STEP */
	CMDQ_REG_SET32(base + 0x044, 0x00000001);
	value[7] = CMDQ_REG_GET32(base + 0x048);    /*  RSZ_DEBUG_1    */
	CMDQ_REG_SET32(base + 0x044, 0x00000002);
	value[8] = CMDQ_REG_GET32(base + 0x048);    /*  RSZ_DEBUG_2 */
	CMDQ_REG_SET32(base + 0x044, 0x00000003);
	value[9] = CMDQ_REG_GET32(base + 0x048);    /*  RSZ_DEBUG_3 */
	CMDQ_REG_SET32(base + 0x044, 0x00000009);
	value[10] = CMDQ_REG_GET32(base + 0x048);   /*  RSZ_DEBUG_9  */
	CMDQ_REG_SET32(base + 0x044, 0x0000000A);
	value[11] = CMDQ_REG_GET32(base + 0x048);   /*  RSZ_DEBUG_10    */
	CMDQ_REG_SET32(base + 0x044, 0x0000000B);
	value[12] = CMDQ_REG_GET32(base + 0x048);   /*  RSZ_DEBUG_11    */
	CMDQ_REG_SET32(base + 0x044, 0x0000000D);
	value[13] = CMDQ_REG_GET32(base + 0x048);   /*  RSZ_DEBUG_13    */
	CMDQ_REG_SET32(base + 0x044, 0x0000000E);
	value[14] = CMDQ_REG_GET32(base + 0x048);   /*  RSZ_DEBUG_14    */
	value[15] = CMDQ_REG_GET32(base + 0x100);   /*  PAT1_GEN_SET    */
	value[16] = CMDQ_REG_GET32(base + 0x200);   /*  PAT2_GEN_SET    */
	value[17] = CMDQ_REG_GET32(base + 0x00C);   /*  RSZ_INT_FLAG    */
	/*  RSZ_LUMA_HORIZONTAL_INTEGER_OFFSET  */
	value[18] = CMDQ_REG_GET32(base + 0x020);
	/*  RSZ_LUMA_HORIZONTAL_SUBPIXEL_OFFSET */
	value[19] = CMDQ_REG_GET32(base + 0x024);
	/*  RSZ_LUMA_VERTICAL_INTEGER_OFFSET    */
	value[20] = CMDQ_REG_GET32(base + 0x028);
	/*  RSZ_LUMA_VERTICAL_SUBPIXEL_OFFSET   */
	value[21] = CMDQ_REG_GET32(base + 0x02C);
	/*  RSZ_CHROMA_HORIZONTAL_INTEGER_OFFSET    */
	value[22] = CMDQ_REG_GET32(base + 0x030);
	/*  RSZ_CHROMA_HORIZONTAL_SUBPIXEL_OFFSET   */
	value[23] = CMDQ_REG_GET32(base + 0x034);
	/*  RSZ_RSV */
	value[24] = CMDQ_REG_GET32(base + 0x040);
	value[25] = CMDQ_REG_GET32(base + 0x04C);   /*  RSZ_TAP_ADAPT	*/
	value[26] = CMDQ_REG_GET32(base + 0x050);   /*  RSZ_IBSE_SOFTCLIP */
	value[27] = CMDQ_REG_GET32(base + 0x22C);   /*  RSZ_ETC_CONTROL	*/
	/*  RSZ_ETC_SWITCH_MAX_MIN_1	*/
	value[28] = CMDQ_REG_GET32(base + 0x230);
	/*  RSZ_ETC_SWITCH_MAX_MIN_2	*/
	value[29] = CMDQ_REG_GET32(base + 0x234);
	/*  RSZ_ETC_RING_CONTROL	*/
	value[30] = CMDQ_REG_GET32(base + 0x238);
	/*  RSZ_ETC_RING_CONTROL_GAINCONTROL_1	*/
	value[31] = CMDQ_REG_GET32(base + 0x23C);
	/*  RSZ_ETC_RING_CONTROL_GAINCONTROL_2	*/
	value[32] = CMDQ_REG_GET32(base + 0x240);
	/*  RSZ_ETC_RING_CONTROL_GAINCONTROL_3	*/
	value[33] = CMDQ_REG_GET32(base + 0x244);
	/*  RSZ_ETC_SIMILARITY_PROTECTION_GAINCONTROL_1	*/
	value[34] = CMDQ_REG_GET32(base + 0x248);
	/*  RSZ_ETC_SIMILARITY_PROTECTION_GAINCONTROL_2	*/
	value[35] = CMDQ_REG_GET32(base + 0x24C);
	/*  RSZ_ETC_SIMILARITY_PROTECTION_GAINCONTROL_3	*/
	value[36] = CMDQ_REG_GET32(base + 0x250);
	value[37] = CMDQ_REG_GET32(base + 0x254);   /*  RSZ_ETC_BLEND	*/

	CMDQ_ERR(
		"=============== [CMDQ] %s Status ====================================\n",
		label);
	CMDQ_ERR(
		"RSZ_ENABLE: 0x%08x, RSZ_CONTROL_1: 0x%08x, RSZ_CONTROL_2: 0x%08x, RSZ_INPUT_IMAGE: 0x%08x, RSZ_OUTPUT_IMAGE: 0x%08x\n",
		value[0], value[1], value[2], value[3],  value[4]);
	CMDQ_ERR(
		"RSZ_HORIZONTAL_COEFF_STEP: 0x%08x, RSZ_VERTICAL_COEFF_STEP: 0x%08x\n",
		value[5], value[6]);
	CMDQ_ERR(
		"RSZ_DEBUG_1: 0x%08x, RSZ_DEBUG_2: 0x%08x, RSZ_DEBUG_3: 0x%08x\n",
		value[7], value[8], value[9]);
	CMDQ_ERR(
		"RSZ_DEBUG_9: 0x%08x, RSZ_DEBUG_10: 0x%08x, RSZ_DEBUG_11: 0x%08x\n",
		value[10], value[11], value[12]);
	CMDQ_ERR(
		"RSZ_DEBUG_13: 0x%08x, RSZ_DEBUG_14: 0x%08x\n",
		value[13], value[14]);
	CMDQ_ERR("PAT1_GEN_SET: 0x%08x, PAT2_GEN_SET: 0x%08x\n",
		value[15], value[16]);
	CMDQ_ERR(
		"RSZ_INT_FLAG: 0x%08x, RSZ_LUMA_HORIZONTAL_INTEGER_OFFSET: 0x%08x, RSZ_LUMA_HORIZONTAL_SUBPIXEL_OFFSET: 0x%08x\n",
		value[17], value[18], value[19]);
	CMDQ_ERR(
		"RSZ_LUMA_VERTICAL_INTEGER_OFFSET: 0x%08x, RSZ_LUMA_VERTICAL_SUBPIXEL_OFFSET: 0x%08x, RSZ_CHROMA_HORIZONTAL_INTEGER_OFFSET: 0x%08x\n",
		value[20], value[21], value[22]);
	CMDQ_ERR(
		"RSZ_CHROMA_HORIZONTAL_SUBPIXEL_OFFSET: 0x%08x, RSZ_RSV: 0x%08x, RSZ_TAP_ADAPT: 0x%08x\n",
		value[23], value[24], value[25]);
	CMDQ_ERR(
		"RSZ_IBSE_SOFTCLIP: 0x%08x, RSZ_ETC_CONTROL: 0x%08x, RSZ_ETC_SWITCH_MAX_MIN_1: 0x%08x\n",
		value[26], value[27], value[28]);
	CMDQ_ERR(
		"RSZ_ETC_SWITCH_MAX_MIN_2: 0x%08x, RSZ_ETC_RING_CONTROL: 0x%08x, RSZ_ETC_RING_CONTROL_GAINCONTROL_1: 0x%08x\n",
		value[29], value[30], value[31]);
	CMDQ_ERR(
		"RSZ_ETC_RING_CONTROL_GAINCONTROL_2: 0x%08x, RSZ_ETC_RING_CONTROL_GAINCONTROL_3: 0x%08x, RSZ_ETC_SIMILARITY_PROTECTION_GAINCONTROL_1: 0x%08x\n",
		value[32], value[33], value[34]);
	CMDQ_ERR(
		"RSZ_ETC_SIMILARITY_PROTECTION_GAINCONTROL_2: 0x%08x, RSZ_ETC_SIMILARITY_PROTECTION_GAINCONTROL_3: 0x%08x, RSZ_ETC_BLEND: 0x%08x\n",
		value[35], value[36], value[37]);
	/* parse state */
	/* .valid=1/request=1: upstream module sends data */
	/* .ready=1: downstream module receives data */
	state = value[8] & 0xF;
	request[0] = state & (0x1);	/* out valid */
	request[1] = (state & (0x1 << 1)) >> 1;	/* out ready */
	request[2] = (state & (0x1 << 2)) >> 2;	/* in valid */
	request[3] = (state & (0x1 << 3)) >> 3;	/* in ready */
	CMDQ_ERR("RSZ inRdy,inRsq,outRdy,outRsq: %d,%d,%d,%d (%s)\n",
		request[3], request[2], request[1], request[0],
		cmdq_mdp_get_rsz_state(state));
}
void cmdq_mdp_dump_tdshp(const unsigned long base, const char *label)
{
	uint32_t value[10] = { 0 };

	value[0] = CMDQ_REG_GET32(base + 0x114);
	value[1] = CMDQ_REG_GET32(base + 0x11C);
	value[2] = CMDQ_REG_GET32(base + 0x104);
	value[3] = CMDQ_REG_GET32(base + 0x108);
	value[4] = CMDQ_REG_GET32(base + 0x10C);
	value[5] = CMDQ_REG_GET32(base + 0x110);
	value[6] = CMDQ_REG_GET32(base + 0x120);
	value[7] = CMDQ_REG_GET32(base + 0x124);
	value[8] = CMDQ_REG_GET32(base + 0x128);
	value[9] = CMDQ_REG_GET32(base + 0x12C);
	CMDQ_ERR(
		"=============== [CMDQ] %s Status ====================================\n",
		label);
	CMDQ_ERR("TDSHP INPUT_CNT: 0x%08x, OUTPUT_CNT: 0x%08x\n",
		value[0], value[1]);
	CMDQ_ERR("TDSHP INTEN: 0x%08x, INTSTA: 0x%08x, STATUS: 0x%08x\n",
		value[2], value[3], value[4]);
	CMDQ_ERR("TDSHP CFG: 0x%08x, IN_SIZE: 0x%08x, OUT_SIZE: 0x%08x\n",
		value[5], value[6], value[8]);
	CMDQ_ERR("TDSHP OUTPUT_OFFSET: 0x%08x, BLANK_WIDTH: 0x%08x\n",
		value[7], value[9]);
}
void cmdq_mdp_dump_aal(const unsigned long base, const char *label)
{
	uint32_t value[9] = { 0 };

	value[0] = CMDQ_REG_GET32(base + 0x00C);    /* MDP_AAL_INTSTA       */
	value[1] = CMDQ_REG_GET32(base + 0x010);    /* MDP_AAL_STATUS       */
	value[2] = CMDQ_REG_GET32(base + 0x024);    /* MDP_AAL_INPUT_COUNT  */
	value[3] = CMDQ_REG_GET32(base + 0x028);    /* MDP_AAL_OUTPUT_COUNT */
	value[4] = CMDQ_REG_GET32(base + 0x030);    /* MDP_AAL_SIZE         */
	value[5] = CMDQ_REG_GET32(base + 0x034);    /* MDP_AAL_OUTPUT_SIZE  */
	value[6] = CMDQ_REG_GET32(base + 0x038);    /* MDP_AAL_OUTPUT_OFFSET*/
	value[7] = CMDQ_REG_GET32(base + 0x4EC);    /* MDP_AAL_TILE_00      */
	value[8] = CMDQ_REG_GET32(base + 0x4F0);    /* MDP_AAL_TILE_01      */
	CMDQ_ERR(
		"=============== [CMDQ] %s Status ====================================\n",
		label);
	CMDQ_ERR("AAL_INTSTA: 0x%08x, AAL_STATUS: 0x%08x\n",
		value[0], value[1]);
	CMDQ_ERR(
		"AAL_INPUT_COUNT: 0x%08x, AAL_OUTPUT_COUNT: 0x%08x, AAL_SIZE: 0x%08x\n",
		value[2], value[3], value[4]);
	CMDQ_ERR("AAL_OUTPUT_SIZE: 0x%08x, AAL_OUTPUT_OFFSET: 0x%08x\n",
		value[5], value[6]);
	CMDQ_ERR("AAL_TILE_00: 0x%08x, AAL_TILE_01: 0x%08x\n",
		value[7], value[8]);
}
void cmdq_mdp_dump_hdr(const unsigned long base, const char *label)
{
	uint32_t value[15] = { 0 };

	value[0] = CMDQ_REG_GET32(base + 0x000);    /* MDP_HDR_TOP            */
	value[1] = CMDQ_REG_GET32(base + 0x004);    /* MDP_HDR_RELAY          */
	value[2] = CMDQ_REG_GET32(base + 0x00C);    /* MDP_HDR_INTSTA         */
	value[3] = CMDQ_REG_GET32(base + 0x010);    /* MDP_HDR_ENGSTA         */
	value[4] = CMDQ_REG_GET32(base + 0x020);    /* MDP_HDR_HIST_CTRL_0    */
	value[5] = CMDQ_REG_GET32(base + 0x024);    /* MDP_HDR_HIST_CTRL_1    */
	value[6] = CMDQ_REG_GET32(base + 0x014);    /* MDP_HDR_SIZE_0         */
	value[7] = CMDQ_REG_GET32(base + 0x018);    /* MDP_HDR_SIZE_1         */
	value[8] = CMDQ_REG_GET32(base + 0x01C);    /* MDP_HDR_SIZE_2         */
	value[9] = CMDQ_REG_GET32(base + 0x0EC);    /* MDP_HDR_CURSOR_CTRL    */
	value[10] = CMDQ_REG_GET32(base + 0x0F0);   /* MDP_HDR_CURSOR_POS     */
	value[11] = CMDQ_REG_GET32(base + 0x0F4);   /* MDP_HDR_CURSOR_COLOR   */
	value[12] = CMDQ_REG_GET32(base + 0x0F8);   /* MDP_HDR_TILE_POS       */
	value[13] = CMDQ_REG_GET32(base + 0x0FC);   /* MDP_HDR_CURSOR_BUF0    */
	value[14] = CMDQ_REG_GET32(base + 0x100);   /* MDP_HDR_CURSOR_BUF1    */
	CMDQ_ERR(
		"=============== [CMDQ] %s Status ====================================\n",
		label);
	CMDQ_ERR("HDR_TOP: 0x%08x, HDR_RELAY: 0x%08x, HDR_INTSTA: 0x%08x\n",
		value[0], value[1], value[2]);
	CMDQ_ERR(
		"HDR_ENGSTA: 0x%08x, HDR_HIST_CTRL0: 0x%08x, HDR_HIST_CTRL1: 0x%08x\n",
		value[3], value[4], value[5]);
	CMDQ_ERR("HDR_SIZE_0: 0x%08x, HDR_SIZE_1: 0x%08x, HDR_SIZE_2: 0x%08x\n",
		value[6], value[7], value[8]);
	CMDQ_ERR(
		"HDR_CURSOR_CTRL: 0x%08x, HDR_CURSOR_POS: 0x%08x, HDR_CURSOR_COLOR: 0x%08x\n",
		value[9], value[10], value[11]);
	CMDQ_ERR(
		"HDR_TILE_POS: 0x%08x, HDR_CURSOR_BUF0: 0x%08x, HDR_CURSOR_BUF1: 0x%08x\n",
		value[12], value[13], value[14]);
}
void cmdq_mdp_dump_tcc(const unsigned long base, const char *label)
{
	uint32_t value[7] = { 0 };

	value[0] = CMDQ_REG_GET32(base + 0x00000000);	/* MDP_TCC_CTRL	      */
	value[1] = CMDQ_REG_GET32(base + 0x00000520);	/* MDP_TCC_DEBUG      */
	value[2] = CMDQ_REG_GET32(base + 0x00000524);   /* MDP_TCC_INTEN      */
	value[3] = CMDQ_REG_GET32(base + 0x00000528);	/* MDP_TCC_INTST      */
	value[4] = CMDQ_REG_GET32(base + 0x0000052C);	/* MDP_TCC_ST         */
	value[5] = CMDQ_REG_GET32(base + 0x00000530);	/* MDP_TCC_CROP_X     */
	value[6] = CMDQ_REG_GET32(base + 0x00000534);	/* MDP_TCC_CROP_Y     */

	CMDQ_ERR(
		"=============== [CMDQ] %s Status ====================================\n",
		label);
	CMDQ_ERR(
		"MDP_TCC_CTRL: 0x%08x, MDP_TCC_DEBUG: 0x%08x, MDP_TCC_INTEN: 0x%08x\n",
		value[0], value[1], value[2]);
	CMDQ_ERR("MDP_TCC_INTST: 0x%08x, MDP_TCC_ST: 0x%08x\n",
		value[3], value[4]);
	CMDQ_ERR("MDP_TCC_CROP_X: 0x%08x, MDP_TCC_CROP_Y: 0x%08x\n",
		value[5], value[6]);
}
void cmdq_mdp_dump_fg(const unsigned long base, const char *label)
{
	uint32_t value[16] = { 0 };

	value[0] = CMDQ_REG_GET32(base + 0x000);    /* MDP_FG_TRIGGER   */
	value[1] = CMDQ_REG_GET32(base + 0x004);    /* MDP_FG_STATUS    */
	value[2] = CMDQ_REG_GET32(base + 0x020);    /* MDP_FG_FG_CTRL_0 */
	value[3] = CMDQ_REG_GET32(base + 0x024);    /* MDP_FG_FG_CK_EN  */
	value[4] = CMDQ_REG_GET32(base + 0x02C);    /* MDP_FG_BACK_DOOR_0*/
	value[5] = CMDQ_REG_GET32(base + 0x400);    /* MDP_FG_PIC_INFO_0*/
	value[6] = CMDQ_REG_GET32(base + 0x404);    /* MDP_FG_PIC_INFO_1*/
	value[7] = CMDQ_REG_GET32(base + 0x418);    /* MDP_FG_TILE_INFO_0*/
	value[8] = CMDQ_REG_GET32(base + 0x41C);    /* MDP_FG_TILE_INFO_1*/
	value[9] = CMDQ_REG_GET32(base + 0x500);    /* MDP_FG_DEBUG_0   */
	value[10] = CMDQ_REG_GET32(base + 0x504);   /* MDP_FG_DEBUG_1   */
	value[11] = CMDQ_REG_GET32(base + 0x508);   /* MDP_FG_DEBUG_2   */
	value[12] = CMDQ_REG_GET32(base + 0x50C);   /* MDP_FG_DEBUG_3   */
	value[13] = CMDQ_REG_GET32(base + 0x510);   /* MDP_FG_DEBUG_4   */
	value[14] = CMDQ_REG_GET32(base + 0x514);   /* MDP_FG_DEBUG_5   */
	value[15] = CMDQ_REG_GET32(base + 0x518);   /* MDP_FG_DEBUG_6   */

	CMDQ_ERR(
		"=============== [CMDQ] %s Status ====================================\n",
		label);
	CMDQ_ERR(
		"MDP_FG_TRIGGER 0x%08x, MDP_FG_STATUS 0x%08x, MDP_FG_FG_CTRL_0 0x%08x\n",
		value[0], value[1], value[2]);
	CMDQ_ERR(
		"MDP_FG_FG_CK_EN 0x%08x, MDP_FG_BACK_DOOR_0 0x%08x, MDP_FG_PIC_INFO_0 0x%08x\n",
		value[3], value[4], value[5]);
	CMDQ_ERR(
		"MDP_FG_PIC_INFO_1 0x%08x, MDP_FG_TILE_INFO_0 0x%08x, MDP_FG_TILE_INFO_1 0x%08x\n",
		value[6], value[7], value[8]);
	CMDQ_ERR(
		"MDP_FG_DEBUG_0x%08x, MDP_FG_DEBUG_1 0x%08x, MDP_FG_DEBUG_2 0x%08x\n",
		value[9], value[10], value[11]);
	CMDQ_ERR(
		"MDP_FG_DEBUG_3 0x%08x, MDP_FG_DEBUG_4 0x%08x, MDP_FG_DEBUG_5 0x%08x\n",
		value[12], value[13], value[14]);
	CMDQ_ERR("MDP_FG_DEBUG_6 0x%08x\n",
		value[15]);
}
int32_t cmdqMdpClockOn(uint64_t engineFlag)
{
	CMDQ_MSG("Enable MDP(0x%llx) clock begin\n", engineFlag);
#ifdef CMDQ_PWR_AWARE
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_CAMIN);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_RDMA0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_RDMA1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_RSZ0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_RSZ1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_AAL0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_AAL1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_TCC0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_TCC1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_TDSHP0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_TDSHP1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_COLOR0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_COLOR1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_HDR0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_HDR1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_FG0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_FG1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_WROT0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_WROT1);
#else
	CMDQ_MSG("Force MDP clock all on\n");

	/* enable all bits in MMSYS_CG_CLR0 and MMSYS_CG_CLR1 */
	CMDQ_REG_SET32(MMSYS_CONFIG_BASE + 0x108, 0xFFFFFFFF);
	CMDQ_REG_SET32(MMSYS_CONFIG_BASE + 0x118, 0xFFFFFFFF);

#endif				/* #ifdef CMDQ_PWR_AWARE */

	CMDQ_MSG("Enable MDP(0x%llx) clock end\n", engineFlag);

	return 0;
}

struct MODULE_BASE {
	uint64_t engine;
	/* considering 64 bit kernel, use long type to store base addr */
	long base;
	const char *name;
};

#define DEFINE_MODULE(eng, base) {eng, base, #eng}

int32_t cmdqMdpDumpInfo(uint64_t engineFlag, int logLevel)
{
	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA0))
		cmdq_mdp_dump_rdma(MDP_RDMA0_BASE, "RDMA0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA1))
		cmdq_mdp_dump_rdma(MDP_RDMA1_BASE, "RDMA1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_FG0))
		cmdq_mdp_dump_fg(MDP_FG0_BASE, "FG0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_FG1))
		cmdq_mdp_dump_fg(MDP_FG1_BASE, "FG1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ0))
		cmdq_mdp_get_func()->mdpDumpRsz(MDP_RSZ0_BASE, "RSZ0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ1))
		cmdq_mdp_get_func()->mdpDumpRsz(MDP_RSZ1_BASE, "RSZ1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP0))
		cmdq_mdp_get_func()->mdpDumpTdshp(MDP_TDSHP0_BASE, "TDSHP0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP1))
		cmdq_mdp_get_func()->mdpDumpTdshp(MDP_TDSHP1_BASE, "TDSHP1");


	if (engineFlag & (1LL << CMDQ_ENG_MDP_COLOR0))
		cmdq_mdp_dump_color(MDP_COLOR0_BASE, "COLOR0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_COLOR1))
		cmdq_mdp_dump_color(MDP_COLOR1_BASE, "COLOR1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TCC0))
		cmdq_mdp_dump_tcc(MDP_TCC0_BASE, "TCC0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TCC1))
		cmdq_mdp_dump_tcc(MDP_TCC1_BASE, "TCC1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT0))
		cmdq_mdp_dump_rot(MDP_WROT0_BASE, "WROT0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT1))
		cmdq_mdp_dump_rot(MDP_WROT1_BASE, "WROT1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_HDR0))
		cmdq_mdp_dump_hdr(MDP_HDR0_BASE, "HDR0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_HDR1))
		cmdq_mdp_dump_hdr(MDP_HDR1_BASE, "HDR1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_AAL0))
		cmdq_mdp_dump_aal(MDP_AAL0_BASE, "AAL0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_AAL1))
		cmdq_mdp_dump_aal(MDP_AAL1_BASE, "AAL1");

	/* verbose case, dump entire 1KB HW register region */
	/* for each enabled HW module. */
	if (logLevel >= 1) {
		int inner = 0;

		const struct MODULE_BASE bases[] = {
			DEFINE_MODULE(CMDQ_ENG_MDP_RDMA0, MDP_RDMA0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_RDMA1, MDP_RDMA1_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_RSZ0, MDP_RSZ0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_RSZ1, MDP_RSZ1_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_TDSHP0, MDP_TDSHP0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_TDSHP1, MDP_TDSHP1_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_COLOR0, MDP_COLOR0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_COLOR1, MDP_COLOR1_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_FG0, MDP_FG0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_FG1, MDP_FG1_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_HDR0, MDP_HDR0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_HDR1, MDP_HDR1_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_WROT0, MDP_WROT0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_WROT1, MDP_WROT1_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_AAL0, MDP_AAL0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_AAL1, MDP_AAL1_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_TCC0, MDP_TCC0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_TCC1, MDP_TCC1_BASE),
		};

		for (inner = 0; inner < ARRAY_SIZE(bases); ++inner) {
			if (engineFlag & (1LL << bases[inner].engine)) {
				CMDQ_ERR(
					"========= [CMDQ] %s dump base 0x%lx ========\n",
					bases[inner].name, bases[inner].base);
				print_hex_dump(KERN_ERR, "",
					DUMP_PREFIX_ADDRESS, 32, 4,
					(void *)bases[inner].base, 1024,
					false);
			}
		}
	}

	return 0;
}

enum MOUT_BITS {
	MOUT_BITS_ISP_MDP0  =  0,  /* bit  0: ISP_MDP multiple outupt reset */
	MOUT_BITS_ISP_MDP1  =  1,  /* bit  1: ISP_MDP multiple outupt reset */
	MOUT_BITS_ISP_MDP2  =  2,  /* bit  2: ISP_MDP multiple outupt reset */
	MOUT_BITS_ISP_MDP3  =  3,  /* bit  3: ISP_MDP multiple outupt reset */
	MOUT_BITS_MDP_AAL0  =  4,  /* bit  4: MDP_PRZ0 multiple outupt reset */
	MOUT_BITS_MDP_AAL1  =  5,  /* bit  5: MDP_PRZ1 multiple outupt reset */
	MOUT_BITS_MDP_RDMA0 =  6,  /* bit  6: MDP_RDMA0 multiple outupt reset */
	MOUT_BITS_MDP_RDMA1 =  7,  /* bit  7: MDP_RDMA0 multiple outupt reset */
	MOUT_BITS_MDP_RDMA2 =  8,  /* bit  8: MDP_RDMA0 multiple outupt reset */
	MOUT_BITS_MDP_RDMA3 =  9,  /* bit  9: MDP_RDMA0 multiple outupt reset */
};

int32_t cmdqMdpResetEng(uint64_t engineFlag)
{
#ifndef CMDQ_PWR_AWARE
	return 0;
#else
	int status = 0;
	int64_t engineToResetAgain = 0LL;
	uint32_t mout_bits_old = 0L;
	uint32_t mout_bits = 0L;

	long MMSYS_MOUT_RST_REG = MMSYS_CONFIG_BASE + (0xF00);

	CMDQ_PROF_START(0, "MDP_Rst");
	CMDQ_VERBOSE("Reset MDP(0x%llx) begin\n", engineFlag);

	/* After resetting each component, */
	/* we need also reset corresponding MOUT config. */
	mout_bits_old = CMDQ_REG_GET32(MMSYS_MOUT_RST_REG);
	mout_bits = 0;

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA0)) {
		mout_bits |= (1 << MOUT_BITS_MDP_RDMA0);

		status = cmdq_mdp_loop_reset(CMDQ_ENG_MDP_RDMA0,
			MDP_RDMA0_BASE + 0x8, MDP_RDMA0_BASE + 0x408,
			0x7FF00, 0x100, false);
		if (status != 0)
			engineToResetAgain |= (1LL << CMDQ_ENG_MDP_RDMA0);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA1)) {
		mout_bits |= (1 << MOUT_BITS_MDP_RDMA1);

		status = cmdq_mdp_loop_reset(CMDQ_ENG_MDP_RDMA1,
			MDP_RDMA1_BASE + 0x8, MDP_RDMA1_BASE + 0x408,
			0x7FF00, 0x100, false);
		if (status != 0)
			engineToResetAgain |= (1LL << CMDQ_ENG_MDP_RDMA1);
	}


	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP0)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_TDSHP0)) {
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x0);
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x2);
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP1)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_TDSHP1)) {
			CMDQ_REG_SET32(MDP_TDSHP1_BASE + 0x100, 0x0);
			CMDQ_REG_SET32(MDP_TDSHP1_BASE + 0x100, 0x2);
			CMDQ_REG_SET32(MDP_TDSHP1_BASE + 0x100, 0x0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT0)) {
		status = cmdq_mdp_loop_reset(CMDQ_ENG_MDP_WROT0,
			MDP_WROT0_BASE + 0x010, MDP_WROT0_BASE + 0x014,
			0x1, 0x1, true);
		if (status != 0)
			engineToResetAgain |= (1LL << CMDQ_ENG_MDP_WROT0);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT1)) {
		status = cmdq_mdp_loop_reset(CMDQ_ENG_MDP_WROT1,
			MDP_WROT1_BASE + 0x010, MDP_WROT1_BASE + 0x014,
			0x1, 0x1, true);
		if (status != 0)
			engineToResetAgain |= (1LL << CMDQ_ENG_MDP_WROT1);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_CAMIN)) {
		/* MDP_CAMIN can only reset by mmsys, */
		/* so this is not a "error" */
		cmdq_mdp_reset_with_mmsys((1LL << CMDQ_ENG_MDP_CAMIN));
	}
// TODO:
// AAL HDR TCC FG
	/*
	 * when MDP engines fail to reset,
	 * 1. print SMI debug log
	 * 2. try resetting from MMSYS to restore system state
	 * 3. report to QA by raising AEE warning
	 * this reset will reset all registers to power on state.
	 * but DpFramework always reconfigures register values,
	 * so there is no need to backup registers.
	 */
	if (engineToResetAgain != 0) {
		/* check SMI state immediately */
		/* if (1 == is_smi_larb_busy(0)) */
		/* { */
		/* smi_hanging_debug(5); */
		/* } */

		CMDQ_ERR(
			"Reset failed MDP engines(0x%llx), reset again with MMSYS_SW0_RST_B\n",
			 engineToResetAgain);

		cmdq_mdp_reset_with_mmsys(engineToResetAgain);

		/* finally, raise AEE warning to report normal reset fail. */
		/* we hope that reset MMSYS. */
		CMDQ_AEE("MDP", "Disable 0x%llx engine failed\n",
			engineToResetAgain);

		status = -EFAULT;
	}
	/* MOUT configuration reset */
	CMDQ_REG_SET32(MMSYS_MOUT_RST_REG, (mout_bits_old & (~mout_bits)));
	CMDQ_REG_SET32(MMSYS_MOUT_RST_REG, (mout_bits_old | mout_bits));
	CMDQ_REG_SET32(MMSYS_MOUT_RST_REG, (mout_bits_old & (~mout_bits)));

	CMDQ_MSG("Reset MDP(0x%llx) end\n", engineFlag);
	CMDQ_PROF_END(0, "MDP_Rst");

	return status;

#endif				/* #ifdef CMDQ_PWR_AWARE */

}

int32_t cmdqMdpClockOff(uint64_t engineFlag)
{
#ifdef CMDQ_PWR_AWARE

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT0)) {
		cmdq_mdp_loop_off(CMDQ_ENG_MDP_WROT0,
			MDP_WROT0_BASE + 0X010, MDP_WROT0_BASE + 0X014,
			0x1, 0x1, true);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP0)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_TDSHP0)) {
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x0);
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x2);
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x0);
			CMDQ_MSG("Disable MDP_TDSHP0 clock\n");
			cmdq_mdp_get_func()->enableMdpClock(false,
				CMDQ_ENG_MDP_TDSHP0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ1)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_RSZ1)) {
			CMDQ_REG_SET32(MDP_RSZ1_BASE, 0x0);
			CMDQ_REG_SET32(MDP_RSZ1_BASE, 0x10000);
			CMDQ_REG_SET32(MDP_RSZ1_BASE, 0x0);

			CMDQ_MSG("Disable MDP_RSZ1 clock\n");

			cmdq_mdp_get_func()->enableMdpClock(false,
				CMDQ_ENG_MDP_RSZ1);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ0)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_RSZ0)) {
			CMDQ_REG_SET32(MDP_RSZ0_BASE, 0x0);
			CMDQ_REG_SET32(MDP_RSZ0_BASE, 0x10000);
			CMDQ_REG_SET32(MDP_RSZ0_BASE, 0x0);

			CMDQ_MSG("Disable MDP_RSZ0 clock\n");

			cmdq_mdp_get_func()->enableMdpClock(false,
				CMDQ_ENG_MDP_RSZ0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA0)) {
		cmdq_mdp_loop_off(CMDQ_ENG_MDP_RDMA0, MDP_RDMA0_BASE + 0x008,
			MDP_RDMA0_BASE + 0x408, 0x7FF00, 0x100, false);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_CAMIN)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_CAMIN)) {
			cmdq_mdp_reset_with_mmsys((1LL << CMDQ_ENG_MDP_CAMIN));
			CMDQ_MSG("Disable MDP_CAMIN clock\n");
			cmdq_mdp_get_func()->enableMdpClock(false,
				CMDQ_ENG_MDP_CAMIN);
		}
	}
#ifdef CMDQ_MDP_COLOR
	if (engineFlag & (1LL << CMDQ_ENG_MDP_COLOR0)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_COLOR0)) {
			CMDQ_MSG("Disable MDP_COLOR0 clock\n");
			cmdq_mdp_get_func()->enableMdpClock(false,
				CMDQ_ENG_MDP_COLOR0);
		}
	}
#endif
	CMDQ_MSG("Disable MDP(0x%llx) clock end\n", engineFlag);
#endif				/* #ifdef CMDQ_PWR_AWARE */

	return 0;
}


void cmdqMdpInitialSetting(void)
{
#ifdef COFNIG_MTK_IOMMU
	char *data = kzalloc(MDP_DISPATCH_KEY_STR_LEN, GFP_KERNEL);

	/* Register ION Translation Fault function */
	mtk_iommu_register_fault_callback(M4U_PORT_MDP_RDMA0,
		cmdq_TranslationFault_callback, (void *)data);
	mtk_iommu_register_fault_callback(M4U_PORT_MDP_WDMA0,
		cmdq_TranslationFault_callback, (void *)data);
	mtk_iommu_register_fault_callback(M4U_PORT_MDP_WROT0,
		cmdq_TranslationFault_callback, (void *)data);
#elif defined(CONFIG_MTK_M4U)
	char *data = kzalloc(MDP_DISPATCH_KEY_STR_LEN, GFP_KERNEL);

	/* Register M4U Translation Fault function */
	m4u_register_fault_callback(M4U_PORT_MDP_RDMA0,
		cmdq_TranslationFault_callback, (void *)data);
	m4u_register_fault_callback(M4U_PORT_MDP_WDMA0,
		cmdq_TranslationFault_callback, (void *)data);
	m4u_register_fault_callback(M4U_PORT_MDP_WROT0,
		cmdq_TranslationFault_callback, (void *)data);
#endif
}

uint32_t cmdq_mdp_rdma_get_reg_offset_src_addr(void)
{
	return 0xF00;
}

uint32_t cmdq_mdp_wrot_get_reg_offset_dst_addr(void)
{
	return 0xF00;
}

uint32_t cmdq_mdp_wdma_get_reg_offset_dst_addr(void)
{
	return 0xF00;
}

const char *cmdq_mdp_parse_error_module(const struct cmdqRecStruct *task)
{
	const char *module = NULL;
	const u32 ISP_ONLY[2] = {
		((1LL << CMDQ_ENG_ISP_IMGI) | (1LL << CMDQ_ENG_ISP_IMG2O)),
		((1LL << CMDQ_ENG_ISP_IMGI) | (1LL << CMDQ_ENG_ISP_IMG2O) |
		 (1LL << CMDQ_ENG_ISP_IMGO))
	};

	/* common part for both normal and secure path */
	/* for JPEG scenario, use HW flag is sufficient */
	if (task->engineFlag & (1LL << CMDQ_ENG_JPEG_ENC))
		module = "JPGENC";
	else if (task->engineFlag & (1LL << CMDQ_ENG_JPEG_DEC))
		module = "JPGDEC";
	else if ((ISP_ONLY[0] == task->engineFlag) ||
		(ISP_ONLY[1] == task->engineFlag))
		module = "DIP_ONLY";

	/* for secure path, use HW flag is sufficient */
	do {
		if (module != NULL)
			break;

		if (!task->secData.is_secure) {
			/* normal path,
			 * need parse current running instruciton
			 * for more detail
			 */
			break;
		} else if (CMDQ_ENG_MDP_GROUP_FLAG(task->engineFlag)) {
			module = "MDP";
			break;
		} else if (CMDQ_ENG_DPE_GROUP_FLAG(task->engineFlag)) {
			module = "DPE";
			break;
		} else if (CMDQ_ENG_RSC_GROUP_FLAG(task->engineFlag)) {
			module = "RSC";
			break;
		} else if (CMDQ_ENG_GEPF_GROUP_FLAG(task->engineFlag)) {
			module = "GEPF";
			break;
		} else if (CMDQ_ENG_EAF_GROUP_FLAG(task->engineFlag)) {
			module = "EAF";
			break;
		}

		module = "CMDQ";
	} while (0);

	return module;
}

u64 cmdq_mdp_get_engine_group_bits(u32 engine_group)
{
	return gCmdqEngineGroupBits[engine_group];
}

void testcase_clkmgr_mdp(void)
{
#if defined(CMDQ_PWR_AWARE)
	/* RDMA clk test with src buffer addr */
	testcase_clkmgr_impl(CMDQ_ENG_MDP_RDMA0, "CMDQ_TEST_MDP_RDMA0",
		MDP_RDMA0_BASE + cmdq_mdp_rdma_get_reg_offset_src_addr(),
		0xAACCBBDD,
		MDP_RDMA0_BASE + cmdq_mdp_rdma_get_reg_offset_src_addr(),
		true);

	testcase_clkmgr_impl(CMDQ_ENG_MDP_RDMA1, "CMDQ_TEST_MDP_RDMA1",
		MDP_RDMA1_BASE + cmdq_mdp_rdma_get_reg_offset_src_addr(),
		0xAACCBBDD,
		MDP_RDMA1_BASE + cmdq_mdp_rdma_get_reg_offset_src_addr(),
		true);

	/* WROT clk test with dst buffer addr */
	testcase_clkmgr_impl(CMDQ_ENG_MDP_WROT0, "CMDQ_TEST_MDP_WROT0",
		MDP_WROT0_BASE + cmdq_mdp_wrot_get_reg_offset_dst_addr(),
		0xAACCBBDD,
		MDP_WROT0_BASE + cmdq_mdp_wrot_get_reg_offset_dst_addr(),
		true);

	testcase_clkmgr_impl(CMDQ_ENG_MDP_WROT1, "CMDQ_TEST_MDP_WROT1",
		MDP_WROT1_BASE + cmdq_mdp_wrot_get_reg_offset_dst_addr(),
		0xAACCBBDD,
		MDP_WROT1_BASE + cmdq_mdp_wrot_get_reg_offset_dst_addr(),
		true);

	/* TDSHP clk test with input size */
	testcase_clkmgr_impl(CMDQ_ENG_MDP_TDSHP0, "CMDQ_TEST_MDP_TDSHP0",
		MDP_TDSHP0_BASE + 0x40, 0xAACCBBDD, MDP_TDSHP0_BASE + 0x40,
		true);

	testcase_clkmgr_impl(CMDQ_ENG_MDP_TDSHP1, "CMDQ_TEST_MDP_TDSHP1",
		MDP_TDSHP1_BASE + 0x40, 0xAACCBBDD, MDP_TDSHP1_BASE + 0x40,
		true);

	/* RSZ clk test with debug port */
	testcase_clkmgr_impl(CMDQ_ENG_MDP_RSZ0, "CMDQ_TEST_MDP_RSZ0",
		MDP_RSZ0_BASE + 0x040, 0x00000001, MDP_RSZ0_BASE + 0x044,
		false);

	testcase_clkmgr_impl(CMDQ_ENG_MDP_RSZ1, "CMDQ_TEST_MDP_RSZ1",
		MDP_RSZ1_BASE + 0x040, 0x00000001, MDP_RSZ1_BASE + 0x044,
		false);

#endif
}

static void cmdq_mdp_enable_common_clock(bool enable)
{
#ifdef CMDQ_PWR_AWARE
#ifdef CONFIG_MTK_SMI_EXT
	if (enable) {
		/* Use SMI clock API */
		smi_bus_prepare_enable(SMI_LARB0, "MDP");

	} else {
		/* disable, reverse the sequence */
		smi_bus_disable_unprepare(SMI_LARB0, "MDP");
	}
#endif
#endif	/* CMDQ_PWR_AWARE */
}


static void cmdq_mdp_check_hw_status(struct cmdqRecStruct *handle)
{
}


void cmdq_mdp_platform_function_setting(void)
{
	struct cmdqMDPFuncStruct *pFunc = cmdq_mdp_get_func();

	pFunc->dumpMMSYSConfig = cmdq_mdp_dump_mmsys_config;

	pFunc->vEncDumpInfo = cmdqVEncDumpInfo;

	pFunc->initModuleBaseVA = cmdq_mdp_init_module_base_VA;
	pFunc->deinitModuleBaseVA = cmdq_mdp_deinit_module_base_VA;

	pFunc->mdpClockIsOn = cmdq_mdp_clock_is_on;
	pFunc->enableMdpClock = cmdq_mdp_enable_clock;
	pFunc->initModuleCLK = cmdq_mdp_init_module_clk;
	pFunc->mdpDumpRsz = cmdq_mdp_dump_rsz;
	pFunc->mdpDumpTdshp = cmdq_mdp_dump_tdshp;
	pFunc->mdpClockOn = cmdqMdpClockOn;
	pFunc->mdpDumpInfo = cmdqMdpDumpInfo;
	pFunc->mdpResetEng = cmdqMdpResetEng;
	pFunc->mdpClockOff = cmdqMdpClockOff;

	pFunc->mdpInitialSet = cmdqMdpInitialSetting;

	pFunc->rdmaGetRegOffsetSrcAddr = cmdq_mdp_rdma_get_reg_offset_src_addr;
	pFunc->wrotGetRegOffsetDstAddr = cmdq_mdp_wrot_get_reg_offset_dst_addr;
	pFunc->wdmaGetRegOffsetDstAddr = cmdq_mdp_wdma_get_reg_offset_dst_addr;
	pFunc->parseErrModByEngFlag = cmdq_mdp_parse_error_module;
	pFunc->getEngineGroupBits = cmdq_mdp_get_engine_group_bits;
	pFunc->testcaseClkmgrMdp = testcase_clkmgr_mdp;
	pFunc->mdpEnableCommonClock = cmdq_mdp_enable_common_clock;
	pFunc->CheckHwStatus = cmdq_mdp_check_hw_status;
}
