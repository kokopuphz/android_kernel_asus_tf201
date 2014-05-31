#
# EPRJ Support for HTC ENR#U Machine.
#
# Copyright (C) 2013, EternityProject Development
# Angelo G. Del Regno <kholk11@gmail.com>
#

subdir-ccflags-y := -Wno-error

obj-${CONFIG_MACH_ENDEAVORU}		+= board-endeavoru.o
obj-${CONFIG_MACH_ENDEAVORU}		+= board-endeavor-panel.o
obj-${CONFIG_MACH_ENDEAVORU}		+= board-endeavoru-sdhci.o
obj-${CONFIG_MACH_ENDEAVORU}		+= board-endeavoru-memory.o
obj-${CONFIG_MACH_ENDEAVORU}		+= board-endeavoru-power.o
obj-${CONFIG_MACH_ENDEAVORU}		+= board-endeavoru-baseband.o
obj-${CONFIG_MACH_ENDEAVORU}		+= board-endeavoru-kbc.o
obj-${CONFIG_MACH_ENDEAVORU}		+= board-endeavoru-sensors.o
obj-${CONFIG_MACH_ENDEAVORU}		+= board-endeavoru-audio.o
obj-${CONFIG_MACH_ENDEAVORU}		+= tegra_flashlight.o

obj-${CONFIG_MACH_ENDEAVORU}		+= devices_htc.o
obj-${CONFIG_MACH_ENDEAVORU}		+= htc_audio_power.o
#obj-${CONFIG_MACH_ENDEAVORU}		+= null-pokecpu.o
obj-${CONFIG_MACH_ENDEAVORU}		+= htc_simhotswap.o
obj-${CONFIG_MACH_ENDEAVORU}		+= null_pnpmgr.o
#obj-${CONFIG_MACH_ENDEAVORU}		+= htc_wifi_nvs.o

obj-$(CONFIG_MICROP_COMMON)		+= atmega_microp_common.o
obj-${CONFIG_HTC_BATT_CORE}		+= htc_battery_core.o
obj-$(CONFIG_BATTERY_TPS80032)          += htc_battery_tps80032.o
obj-$(CONFIG_HTC_HEADSET_MGR)		+= htc_headset_mgr.o
obj-$(CONFIG_HTC_HEADSET_H2W)		+= htc_headset_h2w.o
obj-$(CONFIG_HTC_HEADSET_GPIO)		+= htc_headset_gpio.o
obj-$(CONFIG_HTC_HEADSET_MICROP)	+= htc_headset_microp.o
obj-$(CONFIG_HTC_HEADSET_PMIC)		+= htc_headset_pmic.o
obj-$(CONFIG_HTC_HEADSET_MISC)		+= htc_headset_misc.o
obj-$(CONFIG_HTC_HEADSET_8X60)		+= htc_headset_8x60.o
# For ENR#U new bootloader: leave this disorder
obj-${CONFIG_MACH_ENDEAVORU}		+= nand_partitions.o

ifeq (${CONFIG_EPRJ_ENRU_BASEBAND},y)
obj-y					+= baseband-xmm-power.o
obj-m					+= baseband-xmm-power2.o
endif

obj-${CONFIG_TEGRA_NVDUMPER}		+= nvdumper.o

# obj-y					+= htc_headset_one_wire.o


# Drivers for mainboard chips
obj-y					+= drivers/
