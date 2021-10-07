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
#ifndef BUILD_LK
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <lcm_pmic.h>

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;
static int regulator_inited;
int display_bias_regulator_init(void)
{
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}
EXPORT_SYMBOL(display_bias_regulator_init);

int disp_late_bias_enable(void)
{
	int ret = 0;
	int retval = 0;
	display_bias_regulator_init();

	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n",ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n",ret);
	retval |= ret;
	return retval;
}
EXPORT_SYMBOL(disp_late_bias_enable);

int display_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5800000, 5800000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5800000, 5800000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

#if 0
	/* get voltage */
	ret = mtk_regulator_get_voltage(&disp_bias_pos);
	if (ret < 0)
		pr_err("get voltage disp_bias_pos fail\n");
	pr_debug("pos voltage = %d\n", ret);

	ret = mtk_regulator_get_voltage(&disp_bias_neg);
	if (ret < 0)
		pr_err("get voltage disp_bias_neg fail\n");
	pr_debug("neg voltage = %d\n", ret);
#endif
	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_enable);

int display_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_disable);
#elif defined(CONFIG_KTD2151_PMU_BLED)
int display_bias_regulator_init(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_regulator_init);

int disp_late_bias_enable(void)
{
	tekhw_lcm_i2c_write_bytes(0x00,0x12);
	tekhw_lcm_i2c_write_bytes(0x01,0x12);
	return 0;
}
EXPORT_SYMBOL(disp_late_bias_enable);

int display_bias_enable(void)
{
	tekhw_lcm_i2c_write_bytes(0x00,0x12);
	tekhw_lcm_i2c_write_bytes(0x01,0x12);
	return 0;
}
EXPORT_SYMBOL(display_bias_enable);

int display_bias_disable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_disable);
#else //defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
int display_bias_regulator_init(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_regulator_init);

int display_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_enable);

int disp_late_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(disp_late_bias_enable);

int display_bias_disable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_disable);
#endif

#else

#ifndef MACH_FPGA
#include <lcm_pmic.h>
#include <platform/mt_typedefs.h>

#ifdef MTK_RT5081_PMU_CHARGER_SUPPORT
#include <cust_i2c.h>
	#define I2C_PMU_CHANNEL I2C_RT5081_PMU_CHANNEL
	#define I2C_PMU_SLAVE_7_BIT_ADDR I2C_RT5081_PMU_SLAVE_7_BIT_ADDR
#else
#ifdef MTK_MT6370_PMU
#include <platform/mt_pmu.h>
	#define I2C_PMU_CHANNEL I2C_SUBPMIC_PMU_CHANNEL
	#define I2C_PMU_SLAVE_7_BIT_ADDR I2C_SUBPMIC_PMU_SLAVE_7_BIT_ADDR
#endif
#endif

#if  defined(MTK_RT5081_PMU_CHARGER_SUPPORT) || defined(MTK_MT6370_PMU)
static int PMU_read_byte (kal_uint8 addr, kal_uint8 *dataBuffer)
{
	kal_uint32 ret = I2C_OK;
	kal_uint16 len;
	struct mt_i2c_t PMU_i2c;
	*dataBuffer = addr;

	PMU_i2c.id = I2C_PMU_CHANNEL;
	PMU_i2c.addr = I2C_PMU_SLAVE_7_BIT_ADDR;
	PMU_i2c.mode = ST_MODE;
	PMU_i2c.speed = 100;
	len = 1;

	ret = i2c_write_read(&PMU_i2c, dataBuffer, len, len);
	if (I2C_OK != ret)
		dprintf(1, "[LK/LCM] %s: i2c_read  failed! ret: %d\n", __func__, ret);
	return ret;
}

static int PMU_write_byte(kal_uint8 addr, kal_uint8 value)
{
	kal_uint32 ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;
	struct mt_i2c_t PMU_i2c;

	write_data[0] = addr;
	write_data[1] = value;

	PMU_i2c.id = I2C_PMU_CHANNEL;
	PMU_i2c.addr = I2C_PMU_SLAVE_7_BIT_ADDR;
	PMU_i2c.mode = ST_MODE;
	PMU_i2c.speed = 100;
	len = 2;

	ret_code = i2c_write(&PMU_i2c, write_data, len);

	return ret_code;
}

int PMU_REG_MASK (kal_uint8 addr, kal_uint8 val, kal_uint8 mask)
{
	kal_uint8 PMU_reg = 0;
	kal_uint32 ret = 0;

	ret = PMU_read_byte(addr, &PMU_reg);

	PMU_reg &= ~mask;
	PMU_reg |= val;

	ret = PMU_write_byte(addr, PMU_reg);

	return ret;
}
#else
int PMU_REG_MASK (kal_uint8 addr, kal_uint8 val, kal_uint8 mask)
{
	return 0;
}

#endif
#endif
#endif

