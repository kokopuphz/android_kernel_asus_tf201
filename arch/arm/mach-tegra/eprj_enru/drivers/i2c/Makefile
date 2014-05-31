#
# I2C-based devices drivers for EPRJ ENR#U
#
# Copyright (C) 2013 EternityProject Developers
# Angelo G. Del Regno <kholk11@gmail.com
#

obj-$(CONFIG_SENSORS_NFC_PN544)		+= pn544.o
obj-$(CONFIG_SENSORS_BMA250)		+= bma250.o
obj-$(CONFIG_SENSORS_PANASONIC_GYRO)	+= ewtzmu2.o
obj-$(CONFIG_SENSORS_AKM8975_PANA_GYRO)	+= akm8975_pana_gyro.o
obj-$(CONFIG_INPUT_ISL29028_CM3628)	+= cm3628.o
obj-$(CONFIG_INPUT_CAPELLA_CM3629)	+= cm3629.o
obj-$(CONFIG_TPS61310_FLASHLIGHT)	+= tps61310_flashlight.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif
