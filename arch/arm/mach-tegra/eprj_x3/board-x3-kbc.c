/*
 * arch/arm/mach-tegra/board-x3-kbc.c
 * Keys configuration for Nvidia tegra3 x3 platform.
 *
 * Copyright (C) 2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/dma-mapping.h>
#include <linux/mfd/max77663-core.h>

#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>

#include "../gpio-names.h"
#include "../board.h"
#include <lge/board-x3.h>
#include "../devices.h"
#include "../wakeups-t3.h"

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 1,	\
	}

#define GPIO_IKEY(_id, _irq, _iswake, _deb)	\
	{					\
		.code = _id,			\
		.gpio = -1,			\
		.irq = _irq,			\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = _deb,	\
	}

#define PMC_WAKE_STATUS		0x14
#define PMC_WAKE2_STATUS	0x168

static inline u64 read_pmc_wake_status(void)
{
	static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u64 reg = 0;

	reg = __raw_readl(pmc + PMC_WAKE_STATUS);
	reg |= ((u64)readl(pmc + PMC_WAKE2_STATUS)) << 32;

	return reg;
}

static struct gpio_keys_button x3_keys[] = {
	[0] = GPIO_KEY(KEY_VOLUMEUP, PO7, 0),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PO4, 1),
	[2] = GPIO_KEY(KEY_MICMUTE, HP_HOOK, 0),
	[3] = GPIO_IKEY(KEY_POWER, MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_EN0_FALLING, 0, 100),
	[4] = GPIO_IKEY(KEY_POWER, MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_EN0_1SEC, 0, 3000),
};

static struct gpio_keys_platform_data x3_keys_platform_data = {
	.buttons	= x3_keys,
	.nbuttons	= ARRAY_SIZE(x3_keys),
};

static struct platform_device x3_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &x3_keys_platform_data,
	},
};

int __init x3_kbc_init(void)
{
	int i;

	pr_debug("EternityProject: Registering"
		"gpio-keys for tegra-kbc.\n");

	for (i = 0; i < ARRAY_SIZE(x3_keys); i++)
		tegra_gpio_enable(x3_keys[i].gpio);

	platform_device_register(&x3_keys_device);

	pr_info("EternityProject: gpio-keys for"
		"tegra-kbc (X3) registered.\n");
	return 0;
}

