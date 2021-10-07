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

#ifndef __TEKHW_COMMON_LCM_GPIO_H__
#define __TEKHW_COMMON_LCM_GPIO_H__
#ifndef BUILD_LK
typedef enum {
	LCM_RST_GPIO_HIGH,
	LCM_RST_GPIO_LOW,
	LCM_RST_MODE,
} LCM_RST_STATUS;
#endif

extern void lcm_set_gpio_output(int gpio, unsigned int output);
extern int lcm_get_gpio_value(int gpio);
#ifndef BUILD_LK
extern void lcm_set_rst_output(LCM_RST_STATUS status);
#endif
#endif /*__TEKHW_COMMON_LCM_GPIO_H__*/

