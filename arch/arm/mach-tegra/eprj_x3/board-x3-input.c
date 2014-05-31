/*
 * arch/arm/mach-tegra/lge/board-x3-input.c
 *
 * Copyright (C) 2011 LG Electronics, Inc.
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
#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "../board.h"
#include "../clock.h"
#include <lge/board-x3.h>
#include <lge/board-x3-input.h>
#include "../devices.h"
#include "../gpio-names.h"

#include <linux/apds990x.h>

static int touch_1V8_en = TEGRA_GPIO_PX4;
static int touch_3V0_en = TEGRA_GPIO_PQ1;

int touch_power_control(int on)
{
	printk("[TOUCH]==>synaptics_touch_ic_power_on =%d\n",on);

	if (on) {
		gpio_set_value(touch_3V0_en, 1);	
		//mdelay(20);
		gpio_set_value(touch_1V8_en, 1);
		//mdelay(100);
	}
	else {
		gpio_set_value(touch_1V8_en, 0);
		mdelay(20);
		gpio_set_value(touch_3V0_en, 0);
		mdelay(20);
	}
	
	return 1;
}

int touch_power_init(int on)
{
	int ret = 0;

	touch_1V8_en = TEGRA_GPIO_PX4;
	touch_3V0_en = TEGRA_GPIO_PQ1;		

	printk("[TOUCH] %s [%d][%d]\n", __func__, touch_1V8_en, touch_3V0_en);

	ret = gpio_request(touch_3V0_en, "TOUCH_3V0_EN");
	if (ret < 0){		
		goto failed_second;
	}
	ret = gpio_direction_output(touch_3V0_en, 0);
	if (ret < 0){
		goto failed_second;
	}
	else {
		tegra_gpio_enable(touch_3V0_en);
	}
	
	ret = gpio_request(touch_1V8_en, "TOUCH_1V8_EN");
	if (ret < 0){		
		goto failed_first;
	}
	ret = gpio_direction_output(touch_1V8_en, 0);
	if (ret < 0){
		goto failed_first;
	}
	else {
		tegra_gpio_enable(touch_1V8_en);
	}
	
       	printk("[TOUCH] %s : successful\n", __func__);

	return 0;

failed_second:
	printk(KERN_ERR "%s(%d) failed\n", __func__, __LINE__);
	gpio_free(touch_3V0_en);
failed_first:
	printk(KERN_ERR "%s(%d) failed\n", __func__, __LINE__);
	gpio_free(touch_1V8_en);
	return ret;
}

struct touch_device_caps touch_caps = {
	.button_support 		= 1,
	.number_of_button 		= 3,
	.button_name 			= {KEY_BACK,KEY_HOME,KEY_MENU},
	.button_margin 			= 10,
	.is_width_supported 	= 1,
	.is_pressure_supported 	= 1,
	.is_id_supported		= 1,
	.max_width 				= 15,
	.max_pressure 			= 0xFF,
	.max_id					= 10,
	.lcd_x					= 720,
	.lcd_y					= 1280,
	.x_max					= 1440,//1100,
	.y_max					= 2780,//1900,

};

struct touch_operation_role touch_role = {
	.operation_mode 	= INTERRUPT_MODE,
	.key_type			= TOUCH_SOFT_KEY, /* rev.a : hard_key, rev.b : soft_key */
	.report_mode			= 0,
	.delta_pos_threshold 	= 0,
	.orientation 		= 0,
	.booting_delay 			= 200,
	.reset_delay		= 20,
	.report_period		= 10000000, 	/* 12.5 msec -> 10.0 msec(X3) */
	.suspend_pwr		= POWER_OFF,
	.resume_pwr		= POWER_ON,
	.jitter_filter_enable	= 1,
	.jitter_curr_ratio		= 28,
	.accuracy_filter_enable	= 1,
	.irqflags 				= IRQF_TRIGGER_FALLING,
};

struct touch_power_module touch_pwr = {
	.use_regulator	= 0,
	.power		= touch_power_control,
	.gpio_init	= touch_power_init,
};

struct touch_platform_data  synaptics_pdata = {
	.int_pin		= TEGRA_GPIO_PQ3,

	.reset_pin		= TEGRA_GPIO_PO1,

	.maker			= "Synaptics",
	.caps	= &touch_caps,
	.role	= &touch_role,
	.pwr	= &touch_pwr,
};


static int prox_ldo_en = TEGRA_GPIO_PX1;

uint32_t prox_pwr_mask = 0; //don't need but

static s32 x3_apds990x_power_init(void)
{
        s32 ret = 0;

	prox_ldo_en = TEGRA_GPIO_PX1;

	printk(KERN_INFO"%s start!![%d][%d] \n",__func__,__LINE__, prox_ldo_en);
	//PROX  LDO Enable : 2.6V & 1.8V
        tegra_gpio_enable(prox_ldo_en);
        ret = gpio_request(prox_ldo_en, "PROX_LDO_EN");
        if (ret < 0) {
                printk("PROX_Sensor[1] : Fail to request Sensor LDO enabling\n");
                return ret;
        }
        ret=gpio_direction_output(prox_ldo_en, 0);
        if (ret < 0)
        {
                printk("PROX_Sensor[2] : Fail to direct Sensor LDO enabling\n");
                gpio_free(prox_ldo_en);
                return ret;
        }       
	
	/* PORXI_LDO_EN */
	//tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_DAT7, TEGRA_PUPD_NORMAL);
	
	printk(KERN_DEBUG "PROX_Sensor[3] : Proximity LDO Enable before toggle "
		"at BOARD: %d, Line[%d]\n", 
	gpio_get_value(prox_ldo_en), __LINE__);

	return ret;
}

static int prox_common_power_set(unsigned char onoff, int sensor)
{
	gpio_set_value(prox_ldo_en,onoff);
	
	printk(" prox_common_power_set : %d, Line[%d]\n",	gpio_get_value(prox_ldo_en),__LINE__);

	if (onoff)
		prox_pwr_mask |= sensor;
	else
		prox_pwr_mask &= ~sensor;	

	return 0;	
}

int prox_power_on(int sensor)
{
    int ret = 0;
	ret = prox_common_power_set(1, sensor);
	return ret;
}
int prox_power_off(int sensor)
{
    int ret = 0;
	ret = prox_common_power_set(0, sensor);
	return ret;	
}

static s32 x3_apds990x_irq_set(void)
{
	s32 ret = 0;
	printk(KERN_INFO"[SENSOR-APDS] %s start(%d, %d)!!!!![%d] \n",__func__, TEGRA_GPIO_PK2, gpio_to_irq(TEGRA_GPIO_PK2), __LINE__);

	//PROX INT
        tegra_gpio_enable(TEGRA_GPIO_PK2);
	
        ret = gpio_request(TEGRA_GPIO_PK2, "PROX_INT");
        if (ret < 0)
        {
                printk("Sensor[1] : Fail to request Proximity INT enabling\n");
                return ret;
        }
        ret=gpio_direction_input(TEGRA_GPIO_PK2);
        if (ret < 0)
        {
                printk("Sensor[2] : Fail to request Proximity INT enabling\n");
                gpio_free(TEGRA_GPIO_PK2);
                return ret;
        }

	return ret;
}


struct lge_sensor_int_gpio{
	s16 num;
	const char *name;
	//unsigned config;
};



//static int sensors_ldo_en = TEGRA_GPIO_PD0;

static s32 x3_mpuirq_init(void)
{
	s32 ret = 0;

//SENSOR INT
	tegra_gpio_enable(TEGRA_GPIO_PH4);

	ret = gpio_request(TEGRA_GPIO_PH4, "SENSOR_INT");
	if (ret < 0) {
		printk("Sensor : Fail to request MOTION INT enabling\n");
		return ret;
	}

	ret=gpio_direction_input(TEGRA_GPIO_PH4);
	if (ret < 0) {
		printk("Sensor : Fail to request MOTION INT enabling\n");
		gpio_free(TEGRA_GPIO_PH4);
		return ret;
	}

	return ret;
}

struct apds990x_proximity_platform_data x3_prox_data = {
        .gpio		= TEGRA_GPIO_PK2,
        .irqflags	= IRQF_TRIGGER_FALLING,
        .power_on 	= prox_power_on,
	.power_off 	= prox_power_off,
};



/* EPRJ: MPU6050B1 */

#include <linux/mpu.h> 

struct mpu_platform_data mpu6050_data = {
        .int_config      = 0x10,
        .level_shifter 	 = 0,
        .orientation     = {  0,  1,  0,
                              -1,  0,  0,
                              0,  0, 1},

};

/* compass */
struct ext_slave_platform_data mpu_compass_data = {
                .address		  = 0x0E,
                .adapt_num		  = 0,
                .bus              = EXT_SLAVE_BUS_PRIMARY,
                .orientation      = { 1,  0,  0,
                                      0,  1,  0,
                                      0,  0,  1},
};

int __init x3_sensor_input_init(void)
{
	x3_mpuirq_init();
	x3_apds990x_power_init();
	x3_apds990x_irq_set();

	return 0;
}
