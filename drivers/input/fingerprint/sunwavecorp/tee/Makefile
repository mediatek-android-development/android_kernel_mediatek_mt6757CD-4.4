
#include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/teei/V1.0/tz_vfs
#subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/core

sunwave_fp-objs          := sf_ctl.o sf_spi.o
obj-$(CONFIG_SUNWAVE_FINGERPRINT_SW9651) = sunwave_fp-objs
