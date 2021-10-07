subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_CHIPSET_8789P1_8185P3) += hxchipset_8789p1_8185p3/
ifeq ($(XUNHU_LPS_TEKHW_SUPPORT),yes)
ifeq ($(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE),y)
$(info xunhu_tp_debug-------------------$(TEKSUN_HARDWARE_TOUCHPANNAL_KERNEL414_64BIT)--------)
TP_LISTS := $(subst ",,$(TEKSUN_HARDWARE_TOUCHPANNAL_KERNEL414_64BIT))
else
$(info xunhu_tp_debug-------------------$(TEKSUN_HARDWARE_TOUCHPANNAL_KERNEL414)--------)
TP_LISTS := $(subst ",,$(TEKSUN_HARDWARE_TOUCHPANNAL_KERNEL414))
endif
obj-y += teksunhw_tp.o
else
TP_LISTS := $(subst ",,$(CUSTOM_KERNEL_TOUCHPANEL))
endif
obj-y += teksun_gesture.o

obj-y += $(foreach TP,$(TP_LISTS), $(TP)/)

