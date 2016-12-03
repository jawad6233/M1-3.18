#
# Makefile for misc devices that really don't fit anywhere else.
#
export MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
export ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))
 
MTK_PLATFORM_PATH_ROOT := $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)
MTK_PROJECT_PATH_ROOT := $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)
MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek
 
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/flashlight/inc \
				-I$(MTK_CUSTOM_PATH)/imgsensor/inc \

obj-y		+= flashlight.o
