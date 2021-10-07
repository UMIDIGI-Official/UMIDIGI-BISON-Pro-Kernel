#ifndef _LINUX_ELAN_KTF_H
#define _LINUX_ELAN_KTF_H

#define ELAN_X_MAX      1080
#define ELAN_Y_MAX      2340 //1600

#define L2500_ADDR			0x7bd0
#define EKTF2100_ADDR		0x7bd0
#define EKTF2200_ADDR		0x7bd0
#define EKTF3100_ADDR		0x7c16
#define FW_ADDR					L2500_ADDR
extern struct tpd_device *tpd;


#define ELAN_KTF_NAME "elan_ktf"

struct elan_ktf_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int rst_gpio;
        int mode_check_gpio;
	int (*power)(int on);
};

#endif /* _LINUX_ELAN_KTF_H */
