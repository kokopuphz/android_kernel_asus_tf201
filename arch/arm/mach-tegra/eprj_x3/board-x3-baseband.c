/*
 * arch/arm/mach-tegra/board-x3-baseband.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/wakelock.h>
#include <linux/platform_data/tegra_usb.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/pinmux.h>
#include <mach/usb_phy.h>
#include <linux/memblock.h>
#include <linux/slab.h>

#include <lge/board-x3.h>
#include "../devices.h"
#include "../gpio-names.h"
#include "../baseband-xmm-power.h"

#define XMM6260_GPIO_BB_RST			MODEM_RESET
#define XMM6260_GPIO_BB_ON			MODEM_PWR_ON
#define XMM6260_GPIO_IPC_BB_WAKE		AP2CP_ACK1_SLAVE_WAKEUP
#define XMM6260_GPIO_IPC_AP_WAKE		CP2AP_ACK2_HOST_WAKEUP
#define XMM6260_GPIO_IPC_HSIC_ACTIVE		CP2AP_ACK1_HOST_ACTIVE
#define XMM6260_GPIO_IPC_HSIC_SUS_REQ		AP2CP_ACK2_SUSPEND_REQ

static int x3_usb_hsic_postsuspend(void);
static int x3_usb_hsic_preresume(void);
static int x3_usb_hsic_phy_ready(void);
static int x3_usb_hsic_phy_off(void);


struct baseband_power_platform_data tegra_baseband_power_data = {
	.baseband_type = BASEBAND_XMM,
	.modem = {
		.xmm = {
			.bb_rst = XMM6260_GPIO_BB_RST,
			.bb_on = XMM6260_GPIO_BB_ON,
			.ipc_bb_wake = XMM6260_GPIO_IPC_BB_WAKE,
			.ipc_ap_wake = XMM6260_GPIO_IPC_AP_WAKE,
			.ipc_hsic_active = XMM6260_GPIO_IPC_HSIC_ACTIVE,
			.ipc_hsic_sus_req = XMM6260_GPIO_IPC_HSIC_SUS_REQ,
			.hsic_device = &tegra_ehci2_device,
		},
	},
};

static struct platform_device tegra_baseband_power_device = {
	.name = "baseband_xmm_power",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};

static struct tegra_usb_phy_platform_ops hsic_imc_plat_ops = {
	.post_suspend = x3_usb_hsic_postsuspend,
	.pre_resume = x3_usb_hsic_preresume,
	.port_power = x3_usb_hsic_phy_ready, // How to handle port_power api?  //ebs_wondering
	.post_phy_on = x3_usb_hsic_phy_ready,	//Nv-1030815_resume_fromLP0.patch
	.post_phy_off = x3_usb_hsic_phy_off,
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_imc_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
	},
	.ops = &hsic_imc_plat_ops,
};

static int x3_usb_hsic_postsuspend(void)
{
	pr_debug("%s\n", __func__);
	baseband_xmm_set_power_status(BBXMM_PS_L2);
}


static int x3_usb_hsic_preresume(void)
{
	pr_debug("%s\n", __func__);
	baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
}


static int x3_usb_hsic_phy_ready(void)
{
	pr_debug("%s\n", __func__);
	baseband_xmm_set_power_status(BBXMM_PS_L0);

}

static int x3_usb_hsic_phy_off(void)
{
	pr_debug("%s\n", __func__);
	baseband_xmm_set_power_status(BBXMM_PS_L2);
}

struct platform_device *tegra_usb_hsic_host_register(void)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(tegra_ehci2_device.name,
		tegra_ehci2_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci2_device.resource,
		tegra_ehci2_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  tegra_ehci2_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci2_device.dev.coherent_dma_mask;

	val = platform_device_add_data(pdev, &tegra_ehci2_hsic_imc_pdata,
			sizeof(struct tegra_usb_platform_data));
	if (val)
		goto error;

	val = platform_device_add(pdev);
	if (val)
		goto error;

	return pdev;

error:
	pr_err("%s: failed to add the host contoller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

void tegra_usb_hsic_host_unregister(struct platform_device **platdev)
{
	struct platform_device *pdev = *platdev;

	if (pdev && &pdev->dev) {
		platform_device_unregister(pdev);
		*platdev = NULL;
	} else
		pr_err("EPRJ: No platform device while unregistering XMM HSIC\n");
}


static int __init tegra_uhsic_init(void)
{
	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_hsic_imc_pdata;

	/* enable baseband gpio(s) */
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.bb_rst);
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.bb_on);
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_bb_wake);
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_ap_wake);
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_hsic_active);
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_hsic_sus_req);

	tegra_baseband_power_data.hsic_register =
					&tegra_usb_hsic_host_register;
	tegra_baseband_power_data.hsic_unregister =
					&tegra_usb_hsic_host_unregister;

	platform_device_register(&tegra_baseband_power_device);

	return 0;
}


int __init x3_baseband_init(void)
{
	int ret;

	printk("##@%s  \n", __func__ ) ;

	tegra_uhsic_init();

	return 0;
}

