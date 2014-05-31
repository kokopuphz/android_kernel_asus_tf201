/*
 * arch/arm/mach-tegra/board-dt-tegra30.c
 *
 * NVIDIA Tegra30 device tree board support
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * Derived from:
 *
 * arch/arm/mach-tegra/board-dt-tegra20.c
 *
 * Copyright (C) 2010 Secret Lab Technologies, Ltd.
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>

#include <mach/iomap.h>

#include "board.h"
#include "clock.h"
#include "common.h"

#ifdef CONFIG_USE_OF

static struct of_device_id tegra_dt_match_table[] __initdata = {
	{ .compatible = "simple-bus", },
	{}
};

struct of_dev_auxdata tegra30_auxdata_lookup[] __initdata = {
//	OF_DEV_AUXDATA("nvidia,tegra20-gpio", TEGRA_GPIO_BASE, "tegra-gpio", NULL),
	OF_DEV_AUXDATA("nvidia,tegra20-pinmux", TEGRA_APB_MISC_BASE, "tegra-pinmux-ctl", NULL),
	OF_DEV_AUXDATA("nvidia,tegra20-sdhci", 0x78000000, "sdhci-tegra.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra20-sdhci", 0x78000200, "sdhci-tegra.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra20-sdhci", 0x78000400, "sdhci-tegra.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra20-sdhci", 0x78000600, "sdhci-tegra.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra20-i2c", 0x7000C000, "tegra-i2c.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra20-i2c", 0x7000C400, "tegra-i2c.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra20-i2c", 0x7000C500, "tegra-i2c.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra20-i2c", 0x7000C700, "tegra-i2c.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra20-i2c", 0x7000D000, "tegra-i2c.4", NULL),
	{}
};

#if 0 /* EPRJ DISABLED */
static __initdata struct tegra_clk_init_table tegra_dt_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "uarta",	"pll_p",	408000000,	true },
	{ NULL,		NULL,		0,		0},
};
#endif

static void __init tegra30_dt_init(void)
{
	extern void __init enru_init_external(void);
	pr_info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
	pr_info("[TEGRA30-DT] Tegra30 DeviceTree BOARD INIT CALLED \n");
	pr_info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
//	tegra_clk_init_from_table(tegra_dt_clk_init_table);

	enru_init_external();

	of_platform_populate(NULL, tegra_dt_match_table,
				tegra30_auxdata_lookup, NULL);
}

static void __init tegra_enterprise_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	tegra_reserve(0, SZ_4M, SZ_8M);
#else
	tegra_reserve(SZ_128M, SZ_4M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
}

static const char *tegra30_dt_board_compat[] = {
	"nvidia,tegra30",
	NULL
};

DT_MACHINE_START(TEGRA30_DT, "NVIDIA Tegra30 (Flattened Device Tree)")
	.atag_offset	= 0x100,
	.soc		= &tegra_soc_desc,
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_enterprise_reserve,
	.init_early	= tegra30_init_early,
//	.init_irq	= tegra_dt_init_irq,
	.init_irq	= tegra_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &tegra_timer,
	.init_machine	= tegra30_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= tegra30_dt_board_compat,
MACHINE_END

#endif
