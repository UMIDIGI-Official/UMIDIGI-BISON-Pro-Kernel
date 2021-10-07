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
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include "tekhw_common_lcm_gpio.h"
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#else
#include <platform/mt_gpio.h>
#endif

 #ifndef BUILD_LK
static struct pinctrl *lcmctrl = NULL;
static struct pinctrl_state *lcm_rst = NULL;
static struct pinctrl_state *lcm_rst_gpio_high = NULL;
static struct pinctrl_state *lcm_rst_gpio_low = NULL;
#if 0
static int gpio_lcd_pwr_en = -1;
static int gpio_lcd_pwr2_en = -1;
#endif
static int  lcm_probe(struct platform_device *pdev);


static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,lcm_mode",},
	{}
};

static struct platform_driver lcm_driver = {
	.probe = lcm_probe,
	.driver = {
		   .name = "teksun_lcm",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};
#endif

extern int mtk_gpio_base_teksun(void);
void lcm_set_gpio_output(int gpio, unsigned int output)
{

#ifdef BUILD_LK
	mt_set_gpio_mode((gpio | 0x80000000), GPIO_MODE_00);
	mt_set_gpio_dir((gpio | 0x80000000), GPIO_DIR_OUT);
	mt_set_gpio_out((gpio | 0x80000000), output);
#else
#if 1
	gpio += mtk_gpio_base_teksun();
	//printk("lcm_set_gpio_output mtk_gpio_chip.base=%d",gpio);
	if (gpio_is_valid(gpio)) {
		gpio_direction_output(gpio, output);
		gpio_set_value(gpio, output);
	}
#else
		if (gpio_lcd_pwr_en!=-1) {
			gpio_direction_output(gpio_lcd_pwr_en, 1);
			gpio_set_value(gpio_lcd_pwr_en, output);
		}
#endif
#endif
}

int lcm_get_gpio_value(int gpio_number)
{
#ifdef BUILD_LK
		mt_set_gpio_mode((gpio_number | 0x80000000), 0);
		mt_set_gpio_dir((gpio_number | 0x80000000),0);
		return mt_get_gpio_in((gpio_number | 0x80000000));
#else
	gpio_number += mtk_gpio_base_teksun();
	if (gpio_is_valid(gpio_number)) {
		return gpio_get_value(gpio_number);
	}
	return -1;
#endif
}


#ifndef BUILD_LK
void lcm_set_rst_output(LCM_RST_STATUS status)
{
	switch (status) 
	{
		case LCM_RST_GPIO_HIGH:
			pinctrl_select_state(lcmctrl, lcm_rst_gpio_high);
			break;
		case LCM_RST_GPIO_LOW:
			pinctrl_select_state(lcmctrl, lcm_rst_gpio_low);
			break;
		case LCM_RST_MODE:
			pinctrl_select_state(lcmctrl, lcm_rst);
			break;
	}
}

int lcm_gpio_init(struct platform_device *pdev)
{
	int ret = 0;
	lcmctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(lcmctrl)) {
		dev_err(&pdev->dev, "Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcmctrl);
	}
	
	lcm_rst = pinctrl_lookup_state(lcmctrl, "lcm_rst");
	if (IS_ERR(lcm_rst)) {
		ret = PTR_ERR(lcm_rst);
		pr_debug("%s : pinctrl err, lcm_rst\n", __func__);
	}
	
	lcm_rst_gpio_high = pinctrl_lookup_state(lcmctrl, "lcm_rst_gpio_high");
	if (IS_ERR(lcm_rst_gpio_high)) {
		ret = PTR_ERR(lcm_rst_gpio_high);
		pr_debug("%s : pinctrl err, lcm_rst_gpio_high\n", __func__);
	}
	
	lcm_rst_gpio_low = pinctrl_lookup_state(lcmctrl, "lcm_rst_gpio_low");
	if (IS_ERR(lcm_rst_gpio_low)) {
		ret = PTR_ERR(lcm_rst_gpio_low);
		pr_debug("%s : pinctrl err, lcm_rst_gpio_low\n", __func__);
	}
#if 0
	lcm_pwr_en_gpio_high = pinctrl_lookup_state(lcmctrl, "lcm_pwr_en_gpio_high");
	if (IS_ERR(lcm_pwr_en_gpio_high)) {
		ret = PTR_ERR(lcm_pwr_en_gpio_high);
		printk("%s : pinctrl err, lcm_rst_gpio_high\n", __func__);
	}
	
	lcm_pwr_en_gpio_low = pinctrl_lookup_state(lcmctrl, "lcm_pwr_en_gpio_low");
	if (IS_ERR(lcm_pwr_en_gpio_low)) {
		ret = PTR_ERR(lcm_pwr_en_gpio_low);
		printk("%s : pinctrl err, lcm_rst_gpio_low\n", __func__);
	}
#endif
#if 0
	gpio_lcd_pwr_en = of_get_named_gpio(pdev->dev.of_node, "lcm_power_gpio", 0);
	if (!gpio_is_valid(gpio_lcd_pwr_en)) {
		pr_debug("%s : gpio err, lcm_power_gpio\n", __func__);
	}
	
	gpio_lcd_pwr2_en = of_get_named_gpio(pdev->dev.of_node, "lcm_power2_gpio", 0);
	if (!gpio_is_valid(gpio_lcd_pwr_en)) {
		pr_debug("%s : gpio err, lcm_power2_gpio\n", __func__);
	}
#endif
	
	return ret;
}

static int  lcm_probe(struct platform_device *pdev)
{
	lcm_gpio_init(pdev);
	return 0;
}

static int __init lcm_init_driver(void)
{
	printk("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		printk("LCM: failed to register disp driver\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit lcm_exit_driver(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_debug("LCM: Unregister lcm driver done\n");
}

late_initcall(lcm_init_driver);
module_exit(lcm_exit_driver);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

#endif
