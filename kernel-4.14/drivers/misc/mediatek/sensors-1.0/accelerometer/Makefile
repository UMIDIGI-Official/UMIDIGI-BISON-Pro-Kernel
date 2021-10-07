# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include

obj-y   +=  accel.o accel_factory.o

ifeq ($(CONFIG_MTK_ACCELHUB),y)
obj-$(CONFIG_MTK_ACCELHUB) += accelhub/
else
$(info  xunhu---------$(TEKSUN_HARDWARE_GSENSOR_KERNEL414)---------- )
ifeq ($(XUNHU_LPS_TEKHW_SUPPORT),yes)
ACCELEROMETER_LISTS := $(subst ",,$(TEKSUN_HARDWARE_GSENSOR_KERNEL414))
obj-y += teksunhw_accel.o
else
ACCELEROMETER_LISTS := $(subst ",,$(CUSTOM_KERNEL_ACCELEROMETER))
endif
obj-y += $(foreach ACCELEROMETER,$(ACCELEROMETER_LISTS),$(ACCELEROMETER)/)
endif
