/*
 * arch/arm/mach-tegra/lge/board-x3-i2c-pin-define.c
 *
 * Copyright (C) 2011 LG Electronics, Inc.
 *
 * Author: dalyong.cha@lge.com
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/power/max8971-charger.h>
#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/spdif.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "../board.h"
#include "../clock.h"
#include <lge/board-x3.h>
#include <lge/board-x3-misc.h>
#include <lge/board-x3-audio.h>
#include <lge/board-x3-input.h>
#include "../devices.h"
#include "../gpio-names.h"
#include "../fuse.h"

//                                                                
#if defined(CONFIG_LGE_NFC_PN544)
//#include "board-lge-nfc.h"
#include <linux/nfc/pn544_lge.h>
#endif
//                                                                

//                                                                             
#ifdef CONFIG_I2C_GPIO
#include <linux/i2c-gpio.h>
#endif
//                                                                             

#define SII9244_MHL_ADDRESS 0x39//0x72 MHL tx
#define SII9244A_MHL_ADDRESS 0x3D// 0x7A(TPI) Register변경 하는 i2c
#define SII9244B_MHL_ADDRESS 0x49//0x92 HDMI rx
#define SII9244C_MHL_ADDRESS 0x64//0xC8(cbus)

/*                                                                        */
#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
#include <linux/regulator/lp8720.h>
static struct lp8720_platform_data lp8720_pdata = {
	.en_gpio_num         = TEGRA_GPIO_PBB4 /*220*/,
};
#endif
#if 0 //                                                       
#include <media/lm3559_flash_led.h>
static struct lm3559_flash_led_platform_data flash_led_data = {
	.gpio_hwen = TEGRA_GPIO_PBB3,
};
#endif

static struct pn544_i2c_platform_data nfc_pdata = {
	.irq_gpio  = TEGRA_GPIO_PW2,
	.ven_gpio  = TEGRA_GPIO_PX5,
	.firm_gpio = TEGRA_GPIO_PR0,
};

static struct i2c_board_info __initdata x3_i2c_bus0_board_info[] = {
	{
		I2C_BOARD_INFO("max98088", 0x10),
#ifdef CONFIG_SND_SOC_TEGRA_MAX98088
		.platform_data = &max98088_pdata,
#endif
	},
        {
                I2C_BOARD_INFO("mpu6050" , 0x68),
                .platform_data = &mpu6050_data,

        },
        {
                I2C_BOARD_INFO("ami306", 0x0E),
		.irq    = 0,
                .platform_data = &mpu_compass_data,

        },
        {
                I2C_BOARD_INFO(APDS990x_DRV_NAME, 0x39),
                .platform_data	= &x3_prox_data,
        },
#ifdef CONFIG_LGE_NFC_PN544
	{
		I2C_BOARD_INFO("pn544", 0x28),
		.type = "pn544",
		.platform_data = &nfc_pdata,
	},
#endif
};


static struct i2c_board_info __initdata x3_i2c_bus1_board_info[] = {
	{
		I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x20),
		.platform_data	= &synaptics_pdata,
	},

	{
		I2C_BOARD_INFO("muic", 0x44),
		.platform_data	= 0,
	},
	{
		I2C_BOARD_INFO("tsc2007_adc", TSC2007_ADC_SLAVE_ADDR),
	},
#define LM3533_BACKLIGHT_ADDRESS 0x36
	{
		I2C_BOARD_INFO("lm3533_bl", LM3533_BACKLIGHT_ADDRESS),
		.platform_data = &lm3533_pdata,
	},
};

static struct i2c_board_info __initdata x3_i2c_bus2_board_info[] = {
	{
		I2C_BOARD_INFO(LP8720_I2C_NAME,  LP8720_I2C_ADDR),
		.platform_data = &lp8720_pdata,
	},
};

static struct i2c_board_info __initdata x3_i2c_bus4_board_info[] = {
	{
		I2C_BOARD_INFO("max17043", MAX17043_SLAVE_ADDR),
	},
	{
		I2C_BOARD_INFO("max8971", MAX8971_SLAVE_ADDR),
		.platform_data	= &max8971_data,
	},
	{
                I2C_BOARD_INFO(TSPDRV_I2C_DEVICE_NAME, TSPDRV_I2C_SLAVE_ADDR),
                .type = TSPDRV_I2C_DEVICE_NAME,
                .platform_data = &tspdrv_i2c_pdata,
	},
};


static struct i2c_board_info __initdata x3_i2c_bus5_board_info[] = {
#if defined(CONFIG_MHL_TX_SII9244)
	{
		I2C_BOARD_INFO("SII9244", SII9244_MHL_ADDRESS),
	},
	{
		I2C_BOARD_INFO("SII9244A", SII9244A_MHL_ADDRESS),
	},
	{
		I2C_BOARD_INFO("SII9244B", SII9244B_MHL_ADDRESS),
	},
	{
		I2C_BOARD_INFO("SII9244C", SII9244C_MHL_ADDRESS),
	},
#endif
};

static void __init x3_i2c_dev_init(void)
{
	x3_prox_data.irq = gpio_to_irq(TEGRA_GPIO_PK2);
	max8971_data.irqb_pgio = gpio_to_irq(TEGRA_GPIO_PJ2);

	/* i2c0 */
	x3_i2c_bus0_board_info[1].irq = gpio_to_irq(TEGRA_GPIO_PH4);
	x3_i2c_bus0_board_info[3].irq = gpio_to_irq(TEGRA_GPIO_PK2);
#ifdef CONFIG_LGE_NFC_PN544
	x3_i2c_bus0_board_info[4].irq = gpio_to_irq(NFC_GPIO_IRQ);
#endif

	/* i2c1 */
	x3_i2c_bus1_board_info[0].irq = gpio_to_irq(TEGRA_GPIO_PQ3);
	x3_i2c_bus1_board_info[1].irq = gpio_to_irq(MUIC_GPIO);

	/* i2c4 */
	x3_i2c_bus4_board_info[0].irq = gpio_to_irq(MAX17043_GAUGE_INT);
	x3_i2c_bus4_board_info[1].irq = gpio_to_irq(MAX8971_IRQB);

	i2c_register_board_info(0, x3_i2c_bus0_board_info,
			ARRAY_SIZE(x3_i2c_bus0_board_info));
	i2c_register_board_info(1, x3_i2c_bus1_board_info,
			ARRAY_SIZE(x3_i2c_bus1_board_info));
	i2c_register_board_info(2, x3_i2c_bus2_board_info,
			ARRAY_SIZE(x3_i2c_bus2_board_info));
	i2c_register_board_info(4, x3_i2c_bus4_board_info,
			ARRAY_SIZE(x3_i2c_bus4_board_info));
//                                                                             
	i2c_register_board_info(5, x3_i2c_bus5_board_info,
			ARRAY_SIZE(x3_i2c_bus5_board_info));
//                                                                             
}

static struct tegra_i2c_platform_data x3_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },		/* fastmode */
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data x3_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 }, //hyojin.an camea camp 100K -> 400K
//	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data x3_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
//	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data x3_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
//	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data x3_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
//	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};
//                                                                             
//MHL i2c
#ifdef CONFIG_I2C_GPIO
static struct i2c_gpio_platform_data x3_gpio_i2c_platform_data = {
	.sda_pin		= TEGRA_GPIO_PQ7,
	.sda_is_open_drain	= 1,
	.scl_pin		= TEGRA_GPIO_PQ6,
	.scl_is_open_drain	= 1,
	.scl_is_output_only = 1,
	.udelay			= 5,
};
#endif
//                                                                             
void __init x3_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &x3_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &x3_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &x3_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &x3_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &x3_i2c5_platform_data;
//                                                                             
#ifdef CONFIG_I2C_GPIO
	tegra_gpio_i2c.dev.platform_data = &x3_gpio_i2c_platform_data;
#endif
//                                                                             
	x3_i2c_dev_init();
//                                                                             
#ifdef CONFIG_I2C_GPIO
	platform_device_register(&tegra_gpio_i2c);
#endif
//                                                                             
	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}

