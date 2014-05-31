/*
 * arch/arm/mach-tegra/board-x3-power.c
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
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/tps80031.h>
#include <linux/regulator/tps80031-regulator.h>
#include <linux/mfd/max77663-core.h>
#include <linux/regulator/max77663-regulator.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/platform_data/tegra_bpc_mgmt.h>
#include <linux/kmsg_dump.h>


#include <mach/edp.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <linux/tps80031-charger.h>

#include "../gpio-names.h"
#include "../board.h"
#include <lge/board-x3.h>
#include "../pm.h"
#include "../wakeups-t3.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

/************************ MAX77663 based regulator ****************/
static struct regulator_consumer_supply max77663_sd0_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply max77663_sd1_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply max77663_sd2_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_lcd", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
};

static struct regulator_consumer_supply max77663_sd3_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddio_hsic", NULL),
};

static struct regulator_consumer_supply max77663_ldo0_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
};

static struct regulator_consumer_supply max77663_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply max77663_ldo2_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
};

static struct regulator_consumer_supply max77663_ldo3_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

static struct regulator_consumer_supply max77663_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply max77663_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
};

static struct regulator_consumer_supply max77663_ldo6_supply[] = {
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
};

static struct regulator_consumer_supply max77663_ldo7_supply[] = {
	REGULATOR_SUPPLY("mhl_1v2", NULL),
};

static struct regulator_consumer_supply max77663_ldo8_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
};

static struct max77663_regulator_fps_cfg max77663_fps_cfg[] = {
	{
		.src		= FPS_SRC_0,
		.en_src		= FPS_EN_SRC_EN0,
		.time_period	= FPS_TIME_PERIOD_DEF,
	},
	{
		.src		= FPS_SRC_1,
		.en_src		= FPS_EN_SRC_EN1,
		.time_period	= FPS_TIME_PERIOD_DEF,
	},
	{
		.src		= FPS_SRC_2,
		.en_src		= FPS_EN_SRC_EN0,
		.time_period	= FPS_TIME_PERIOD_DEF,
	},
};

#define MAX77663_PDATA_INIT(_rid, _id, _min_uV, _max_uV, _supply_reg,	\
			    _always_on, _boot_on, _apply_uV,		\
			    _fps_src, _fps_pu_period, _fps_pd_period, _flags) \
	static struct regulator_init_data max77663_regulator_idata_##_id = {  \
		.supply_regulator = _supply_reg,			\
		.constraints = {					\
			.name = max77663_rails(_id),			\
			.min_uV = _min_uV,				\
			.max_uV = _max_uV,				\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					     REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					   REGULATOR_CHANGE_STATUS |	\
					   REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
			.apply_uV = _apply_uV,				\
		},							\
		.num_consumer_supplies =				\
				ARRAY_SIZE(max77663_##_id##_supply),	\
		.consumer_supplies = max77663_##_id##_supply,		\
	};								\
	static struct max77663_regulator_platform_data max77663_regulator_pdata_##_id = \
	{								\
		.reg_init_data = &max77663_regulator_idata_##_id,	\
		.id = MAX77663_REGULATOR_ID_##_rid,			\
		.fps_src = _fps_src,					\
		.fps_pu_period = _fps_pu_period,			\
		.fps_pd_period = _fps_pd_period,			\
		.fps_cfgs = max77663_fps_cfg,				\
		.flags = _flags,					\
	}

MAX77663_PDATA_INIT(SD0, sd0,  600000, 3387500, NULL, 1, 0, 0,
		    FPS_SRC_NONE, -1, -1, EN2_CTRL_SD0);

MAX77663_PDATA_INIT(SD1, sd1,  800000, 1587500, NULL, 1, 0, 0,
		    FPS_SRC_1, -1, -1, 0);

/* SD2 must be always turned on because used as pull-up signal for NRST_IO. */
MAX77663_PDATA_INIT(SD2, sd2,  600000, 3387500, NULL, 1, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(SD3, sd3,  600000, 3387500, NULL, 0, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO0, ldo0, 800000, 2350000, NULL, 0, 0, 0,
		    FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(LDO1, ldo1, 800000, 2350000, NULL, 0, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);
MAX77663_PDATA_INIT(LDO2, ldo2, 800000, 3950000, NULL, 0, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);
MAX77663_PDATA_INIT(LDO3, ldo3, 3000000, 3000000, NULL, 0, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);
/* LDO4 must be always turned on because connected with vdd_rtc. */
MAX77663_PDATA_INIT(LDO4, ldo4, 800000, 1587500, NULL, 1, 0, 0,
		    FPS_SRC_0, -1, -1, 0);

MAX77663_PDATA_INIT(LDO5, ldo5, 800000, 3950000, NULL, 0, 0, 0,
		    FPS_SRC_1, -1, -1, 0);
MAX77663_PDATA_INIT(LDO6, ldo6, 800000, 3950000, NULL, 1, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);
MAX77663_PDATA_INIT(LDO7, ldo7, 800000, 3950000, NULL, 0, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);
MAX77663_PDATA_INIT(LDO8, ldo8, 800000, 3950000, max77663_rails(sd2), 0, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);

#define MAX77663_REG(_id, _data) &max77663_regulator_pdata_##_data

static struct max77663_regulator_platform_data *max77663_reg_pdata[] = {
	MAX77663_REG(SD0, sd0),
	MAX77663_REG(SD1, sd1),
	MAX77663_REG(SD2, sd2),
	MAX77663_REG(SD3, sd3),
	MAX77663_REG(LDO0, ldo0),
	MAX77663_REG(LDO1, ldo1),
	MAX77663_REG(LDO2, ldo2),
	MAX77663_REG(LDO3, ldo3),
	MAX77663_REG(LDO4, ldo4),
	MAX77663_REG(LDO5, ldo5),
	MAX77663_REG(LDO6, ldo6),
	MAX77663_REG(LDO7, ldo7),
	MAX77663_REG(LDO8, ldo8),
};

struct max77663_gpio_config max77663_gpio_cfg[] = {
	/*
	 * 4 AP usb detect..
	 */
        {
		.gpio		= MAX77663_GPIO1,
		.dir		= GPIO_DIR_OUT,
		.out_drv	= GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate	= GPIO_ALT_DISABLE,

 	},
	/*
	 * 4 sleep clock..
	 */
	{
		.gpio		= MAX77663_GPIO4,
		.dir            = GPIO_DIR_OUT,
		.out_drv        = GPIO_OUT_DRV_PUSH_PULL,
		.alternate	= GPIO_ALT_ENABLE,
	},
};

static struct max77663_platform_data max7763_pdata = {
	.irq_base	= MAX77663_IRQ_BASE,
	.gpio_base	= MAX77663_GPIO_BASE,

	.num_gpio_cfgs	= ARRAY_SIZE(max77663_gpio_cfg),
	.gpio_cfgs	= max77663_gpio_cfg,

	.regulator_pdata = max77663_reg_pdata,
	.num_regulator_pdata = ARRAY_SIZE(max77663_reg_pdata),
	.flags			= SLP_LPM_ENABLE,

	.use_power_off	= true, /* EternityProject: Register power off natively. */
};
/************************ MAX77663 based regulator ****************/


static struct i2c_board_info __initdata x3_regulators[] = {
	{
		/* The I2C address was determined by OTP factory setting */
		I2C_BOARD_INFO("max77663", 0x1C),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &max7763_pdata,
	},
};

/************************ GPIO based switch regulator ****************/

static struct regulator_consumer_supply fixed_reg_ldo_mhl_en_supply[] = {
        REGULATOR_SUPPLY("avdd_vhdmi_vtherm", NULL),
};

static struct regulator_consumer_supply fixed_reg_ldo_sensor_3v0_en_rev_e_supply[] = {
        REGULATOR_SUPPLY("vdd_nct1008", NULL),
        REGULATOR_SUPPLY("vdd_ina230", NULL),
        REGULATOR_SUPPLY("vdd_3v_ldo", NULL),
};


static struct regulator_consumer_supply fixed_reg_ldo_sensor_1v8_en_rev_e_supply[] = {
        REGULATOR_SUPPLY("vlg_1v8_ldo", NULL),
};


// +3V3_TPS_VFUSE
static struct
regulator_consumer_supply fixed_reg_ldo_vdd_fuse_3v3_en_supply[] = {
        REGULATOR_SUPPLY("vdd_fuse", NULL),
};


/* Macro for defining gpio switch regulator sub device data */
#define GREG_INIT(_id, _name, _input_supply, _gpio_nr, _active_low, \
			_init_state, _pg, _enable, _disable)		\
	static struct fixed_reg_regulator_subdev_data gpio_pdata_##_name =  \
	{								\
		.regulator_name	= "gpio-switch-"#_name,			\
		.input_supply	= _input_supply,			\
		.id		= _id,					\
		.gpio_nr	= _gpio_nr,				\
		.pin_group	= _pg,					\
		.active_low	= _active_low,				\
		.init_state	= _init_state,				\
		.voltages	= fixed_reg_##_name##_voltages,	\
		.n_voltages	= ARRAY_SIZE(fixed_reg_##_name##_voltages), \
		.num_consumer_supplies =				\
				ARRAY_SIZE(fixed_reg_##_name##_supply), \
		.consumer_supplies = fixed_reg_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		},							\
		.enable_rail = _enable,					\
		.disable_rail = _disable,				\
	}


/* Macro for defining fixed regulator sub device data */
#define FIXED_REG(_id, _name, _input_supply, _gpio_nr, _active_high,	\
			_millivolts, _boot_state, _sdelay)		\
	static struct regulator_init_data ri_data_##_name =		\
	{								\
		.supply_regulator = _input_supply,			\
		.num_consumer_supplies =				\
			ARRAY_SIZE(fixed_reg_##_name##_supply),		\
		.consumer_supplies = fixed_reg_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		},							\
	};								\
	static struct fixed_voltage_config fixed_reg_##_name##_pdata =	\
	{								\
		.supply_name = "fixed_reg_"#_name,			\
		.microvolts = _millivolts * 1000,			\
		.gpio = _gpio_nr,					\
		.enable_high = _active_high,				\
		.enabled_at_boot = _boot_state,				\
		.init_data = &ri_data_##_name,				\
		.startup_delay = _sdelay,				\
	};								\
	static struct platform_device fixed_reg_##_name##_dev = {	\
		.name	= "reg-fixed-voltage",				\
		.id	= _id,						\
		.dev	= {						\
			.platform_data = &fixed_reg_##_name##_pdata,	\
		},							\
	}
/* "" */

#define MHL_LDO_EN      TEGRA_GPIO_PV2
#define THERM_LDO_EN	TEGRA_GPIO_PK6

#define SENSOR_LDO_EN_REV_E   TEGRA_GPIO_PX7
#define SENSOR_LDO_EN2_REV_E  TEGRA_GPIO_PD2

#define VFUSE_LDO_EN	TEGRA_GPIO_PK7

FIXED_REG(0, ldo_mhl_en, NULL, MHL_LDO_EN, true, 3300, 0, 0);
FIXED_REG(1, ldo_sensor_3v0_en_rev_e, NULL, SENSOR_LDO_EN_REV_E, true, 3000, 0, 0);
FIXED_REG(2, ldo_sensor_1v8_en_rev_e, NULL, SENSOR_LDO_EN2_REV_E, true, 1800, 0, 0);
FIXED_REG(3, ldo_vdd_fuse_3v3_en, NULL, VFUSE_LDO_EN, true, 3300, 0, 0);

#define ADD_FIXED_REG(_name)	(&fixed_reg_##_name##_dev)
static struct platform_device *eternityproject_power_devices[] = {
	ADD_FIXED_REG(ldo_mhl_en),
	ADD_FIXED_REG(ldo_sensor_3v0_en_rev_e),
	ADD_FIXED_REG(ldo_sensor_1v8_en_rev_e),
	ADD_FIXED_REG(ldo_vdd_fuse_3v3_en),
};

static int __init eternityproject_x3_init_regulators(void)
{
	int nregs;
	struct platform_device **regulator_devices;

	regulator_devices = eternityproject_power_devices;
	nregs = ARRAY_SIZE(eternityproject_power_devices);

	return platform_add_devices(regulator_devices, nregs);
}

int __init x3_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;


	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	i2c_register_board_info(4, x3_regulators,
			ARRAY_SIZE(x3_regulators));

	/* EternityProject: Retail X3's board is Revision 1.1 */
	eternityproject_x3_init_regulators();

	return 0;
}

static void x3_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void x3_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data x3_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,

	.board_suspend = x3_board_suspend,
	.board_resume = x3_board_resume,

        .cpu_lp2_min_residency = 20000,

#ifdef CONFIG_TEGRA_LP1_950
	.lp1_lowvolt_support = false,
	.i2c_base_addr = 0,
	.pmuslave_addr = 0,
	.core_reg_addr = 0,
	.lp1_core_volt_low = 0,
	.lp1_core_volt_high = 0,
#endif
};

static void x3_init_deep_sleep_mode(void)
{
	/* EternityProject 19/03/13: Why?!?!??! */
	x3_suspend_data.suspend_mode = TEGRA_SUSPEND_LP0;
}

int __init x3_suspend_init(void)
{
	x3_init_deep_sleep_mode();
	tegra_init_suspend(&x3_suspend_data);

	return 0;
}

#ifdef CONFIG_TEGRA_EDP_LIMITS

int __init x3_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA) {
		regulator_mA = 4700; /* regular AP30 */
	}
	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
	tegra_init_system_edp_limits(TEGRA_BPC_CPU_PWR_LIMIT);
	return 0;
}
#endif

static struct tegra_bpc_mgmt_platform_data bpc_mgmt_platform_data = {
	.gpio_trigger = TEGRA_BPC_TRIGGER,
	.bpc_mgmt_timeout = TEGRA_BPC_TIMEOUT,
};

static struct platform_device x3_bpc_mgmt_device = {
	.name		= "tegra-bpc-mgmt",
	.id		= -1,
	.dev		= {
		.platform_data = &bpc_mgmt_platform_data,
	},
};

void __init x3_bpc_mgmt_init(void)
{
#ifdef CONFIG_SMP
	int int_gpio = gpio_to_irq(TEGRA_BPC_TRIGGER);

	tegra_gpio_enable(TEGRA_BPC_TRIGGER);

	cpumask_setall(&(bpc_mgmt_platform_data.affinity_mask));
	irq_set_affinity_hint(int_gpio,
				&(bpc_mgmt_platform_data.affinity_mask));
	irq_set_affinity(int_gpio, &(bpc_mgmt_platform_data.affinity_mask));
#endif
	platform_device_register(&x3_bpc_mgmt_device);

	return;
}


static irqreturn_t max77663_irq_manualreset_warning(int irq, void *data)
{
	printk("%s\n", __func__);

	kmsg_dump(KMSG_DUMP_PANIC);

	return IRQ_HANDLED;
}

int __init x3_power_late_init(void)
{
	int irq;
	int ret = 0;

	if(is_tegra_bootmode())
	{
		irq = max7763_pdata.irq_base + MAX77663_IRQ_ONOFF_HRDPOWRN;
		ret = request_threaded_irq(irq , NULL, max77663_irq_manualreset_warning,
				IRQF_ONESHOT, "MAX77663_IRQ_ONOFF_HRDPOWRN",
				NULL);
		if (ret) {
			pr_err("%s: Failed to request irq %d\n", __func__, irq);
			return ret;
		}
	}

	return ret;
}
late_initcall(x3_power_late_init);
