# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include
obj-y   +=  gyroscope.o gyro_factory.o

ifeq ($(CONFIG_MTK_GYROHUB),y)
obj-$(CONFIG_MTK_GYROHUB)      +=  gyrohub/
else
obj-$(CONFIG_MTK_BMI160GY_I2C) +=  bmi160gy-i2c/
obj-$(CONFIG_MTK_ITG1010)   +=  ITG1010/
obj-$(CONFIG_MTK_MPU6050GY)   +=  mpu6050gy/
obj-$(CONFIG_MTK_BMI160GY_NEW)   +=  bmi160_gyro/
obj-$(CONFIG_MTK_ICM426XX_GYRO)   +=  icm426xx_gyro/
endif
