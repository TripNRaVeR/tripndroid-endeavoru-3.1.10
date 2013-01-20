/*
 * arch/arm/mach-tegra/board-endeavoru-sdhci.c
 *
 * Copyright (C) 2011-2012 NVIDIA Corporation.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/wl12xx.h>

#include <asm/mach-types.h>

#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include "gpio-names.h"
#include "board.h"

#define ENTERPRISE_WLAN_PWR	TEGRA_GPIO_PV2
#define ENTERPRISE_WLAN_RST	TEGRA_GPIO_PV3
#define ENTERPRISE_WLAN_WOW	TEGRA_GPIO_PO4

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

int enterprise_wifi_status_register(void (*callback)(int , void *), void *);
int enterprise_wifi_power(int on);
int enterprise_wifi_set_carddetect(int val);

unsigned int enterprise_wifi_status(struct device *dev);
static int enterprise_wifi_cd;

static struct wl12xx_platform_data enterprise_wlan_data __initdata = {
	.irq = TEGRA_GPIO_TO_IRQ(ENTERPRISE_WLAN_WOW),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
};

static int emmc_suspend_gpiocfg(void)
{
	ENABLE_GPIO(SDMMC4_CLK, CC4, "SDMMC4_CLK", 0, 0, NORMAL);
    	return 0;
}

static void emmc_resume_gpiocfg(void)
{
	DISABLE_GPIO(SDMMC4_CLK, CC4, NORMAL);
}

static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.mmc_data = {
		.status = enterprise_wifi_status,
		.register_status_notify	= enterprise_wifi_status_register,
		.built_in = 1,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x0F,
	.mmc_data = {
		.built_in = 1,
	},
	.suspend_gpiocfg = emmc_suspend_gpiocfg,
	.resume_gpiocfg = emmc_resume_gpiocfg,
};

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

int enterprise_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;

	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

unsigned int enterprise_wifi_status(struct device *dev)
{
	return enterprise_wifi_cd;
}

int enterprise_wifi_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);
	enterprise_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(enterprise_wifi_set_carddetect);

int enterprise_wifi_power(int on)
{
	static int power_state;

	if (on == power_state)
		return 0;

	printk("%s: Powering %s wifi\n", __func__, (on ? "on" : "off"));

	power_state = on;
	if (on) {
		gpio_set_value(ENTERPRISE_WLAN_PWR, 1);
		mdelay(20);
	} else {
		gpio_set_value(ENTERPRISE_WLAN_PWR, 0);
	}

	return 0;
}
EXPORT_SYMBOL(enterprise_wifi_power);

#ifdef CONFIG_TEGRA_PREPOWER_WIFI
static int __init enterprise_wifi_prepower(void)
{
	if (!machine_is_endeavoru())
		return 0;

	enterprise_wifi_power(1);

	return 0;
}

subsys_initcall_sync(enterprise_wifi_prepower);
#endif

static int __init enterprise_wifi_init(void)
{
	int rc;

	rc = gpio_request(ENTERPRISE_WLAN_PWR, "wlan_power");
	if (rc)
		pr_err("WLAN_PWR gpio request failed:%d\n", rc);

	rc = gpio_request(ENTERPRISE_WLAN_RST, "wlan_rst");
	if (rc)
		pr_err("WLAN_RST gpio request failed:%d\n", rc);

	rc = gpio_request(ENTERPRISE_WLAN_WOW, "bcmsdh_sdmmc");
	if (rc)
		pr_err("WLAN_WOW gpio request failed:%d\n", rc);

	rc = gpio_direction_output(ENTERPRISE_WLAN_PWR, 0);
	if (rc)
		pr_err("WLAN_PWR gpio direction configuration failed:%d\n", rc);

	gpio_direction_output(ENTERPRISE_WLAN_RST, 0);
	if (rc)
		pr_err("WLAN_RST gpio direction configuration failed:%d\n", rc);

	rc = gpio_direction_input(ENTERPRISE_WLAN_WOW);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);

	if (wl12xx_set_platform_data(&enterprise_wlan_data))
		pr_err("Error setting wl12xx_data\n");

	return 0;
}

int __init enterprise_sdhci_init(void)
{
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);
	enterprise_wifi_init();
	return 0;
}
