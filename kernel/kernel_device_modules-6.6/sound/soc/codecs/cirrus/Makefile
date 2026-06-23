# make sure that the environment variables ARCH and CROSS_COMPILE
# are set for your architecture and cross compiler
#
# e.g. for Beaglebone black:
# export ARCH := arm
# export CROSS_COMPILE := arm-linux-gnueabihf-
#
# e.g. Samsung Galaxy S6:
# export ARCH := arm64
# export CROSS_COMPILE := aarch64-linux-android-

#MODULE_NAME = snd-soc-rt

# additional debugging
#TFA_DEBUG=1

# MTK include path
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/sound/soc/mediatek/audio_dsp
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/sound/soc/mediatek/common
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/audio_ipi/include
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/base

# debugging support (also enables trace_printk)
ccflags-y += -Werror
ccflags-y += $(call cc-option,-Wno-date-time,)

snd-soc-mediatek-cs35l45-objs += cs35l45.o
snd-soc-mediatek-cs35l45-objs += cs35l45-i2c.o
snd-soc-mediatek-cs35l45-objs += cs35l45-tables.o
snd-soc-mediatek-cs35l45-objs += wm_adsp.o
snd-soc-mediatek-cs35l45-objs += cs_dsp.o

obj-$(CONFIG_SND_SOC_MEDIATEK_CS35L45) += snd-soc-mediatek-cs35l45.o 