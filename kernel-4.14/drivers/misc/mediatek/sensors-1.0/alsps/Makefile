# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
obj-y   +=  alsps.o
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o

$(info  xunhu---------$(TEKSUN_HARDWARE_ALSPS_KERNEL414)---------- )
ifneq ($(CUSTOM_KERNEL_ALSPS),tp_proximity)
ifeq ($(XUNHU_LPS_TEKHW_SUPPORT),yes)
ALSPS_LISTS := $(subst ",,$(TEKSUN_HARDWARE_ALSPS_KERNEL414))
obj-y += teksunhw_alsps.o
else
ALSPS_LISTS := $(subst ",,$(CUSTOM_KERNEL_ALSPS))
endif

obj-y += $(foreach ALSPS,$(ALSPS_LISTS),$(ALSPS)/)
endif
