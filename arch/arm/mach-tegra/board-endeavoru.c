/*
 * arch/arm/mach-tegra/board-endeavoru.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <linux/cable_detect.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/keyreset.h>
#include <linux/mfd/tps80031.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/tegra_uart.h>
#include <linux/fsl_devices.h>
#include <linux/gpio_keys.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/spi-tegra.h>
#include <linux/leds-lp5521.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/tegra_vibrator.h>
#include <linux/tps80032_adc.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/io_dpd.h>
#include <mach/usb_phy.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_pmic.h>
#include <mach/tegra_flashlight.h>
#include <mach/board_htc.h>
#include <mach/thermal.h>
#include <mach/mhl.h>
#include <mach/touch.h>

#include <media/rawchip/rawchip.h>

#include "board.h"
#include "clock.h"
#include "board-endeavoru.h"
#include "baseband-xmm-power.h"
#include "devices.h"
#include "gpio-names.h"
#include "wakeups-t3.h"
#include "pm.h"

#define PMC_WAKE_STATUS 0x14

static struct balanced_throttle throttle_list[] = {
	{
		.id = BALANCED_THROTTLE_ID_TJ,
		.throt_tab_size = 10,
		.throt_tab = {
			{      0, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 760000, 1000 },
			{ 760000, 1050 },
			{1000000, 1050 },
			{1000000, 1100 },
		},
	},
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_SKIN,
		.throt_tab_size = 6,
		.throt_tab = {
			{ 640000, 1200 },
			{ 640000, 1200 },
			{ 760000, 1200 },
			{ 760000, 1200 },
			{1000000, 1200 },
			{1000000, 1200 },
		},
	},
#endif
};

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.shutdown_device_id = THERMAL_DEVICE_ID_NCT_EXT,
	.temp_shutdown = 90000,
#if defined(CONFIG_TEGRA_EDP_LIMITS) || defined(CONFIG_TEGRA_THERMAL_THROTTLE)
	.throttle_edp_device_id = THERMAL_DEVICE_ID_NCT_EXT,
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	.temp_throttle = 85000,
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	.skin_device_id = THERMAL_DEVICE_ID_SKIN,
	.temp_throttle_skin = 43000,
#endif
};

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
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

static int enrkey_wakeup(void)
{ 
	unsigned long status =  
	readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS); 
	return status & TEGRA_WAKE_GPIO_PU6 ? KEY_POWER : KEY_RESERVED; 
}

static struct gpio_keys_button A_PROJECT_keys[] = {
	[0] = GPIO_KEY(KEY_VOLUMEUP, PS0, 1),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PW3, 1),
	[2] = GPIO_KEY(KEY_POWER, PU6, 1),
	[3] = GPIO_IKEY(KEY_POWER, ENT_TPS80031_IRQ_BASE + TPS80031_INT_PWRON, 1, 2000),
 };

static struct gpio_keys_platform_data A_PROJECT_keys_platform_data = {
	.buttons	= A_PROJECT_keys,
	.nbuttons	= ARRAY_SIZE(A_PROJECT_keys),
	.wakeup_key     = enrkey_wakeup,
 };

static struct platform_device A_PROJECT_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &A_PROJECT_keys_platform_data,
	},
};

int __init A_PROJECT_keys_init(void)
{
	pr_info("Registering gpio keys\n");

	platform_device_register(&A_PROJECT_keys_device);

	return 0;
}

#ifdef	CONFIG_TEGRA_HDMI_MHL

#define EDGE_GPIO_MHL_INT       TEGRA_GPIO_PC7
#define EDGE_GPIO_MHL_USB_SEL   TEGRA_GPIO_PE0
#define EDGE_GPIO_MHL_1V2       TEGRA_GPIO_PE4
#define EDGE_GPIO_MHL_RESET     TEGRA_GPIO_PE6
#define EDGE_GPIO_MHL_DDC_CLK   TEGRA_GPIO_PV4
#define EDGE_GPIO_MHL_DDC_DATA  TEGRA_GPIO_PV5
#define EDGE_GPIO_MHL_3V3       TEGRA_GPIO_PY2

static int mhl_sii_power(int on)
{
	int rc = 0;

	pr_info("[DISP]%s(%d) IN\n", __func__, __LINE__);

	switch (on) {
		case 0:
			/*Turn off MHL_3V3*/
			gpio_set_value(EDGE_GPIO_MHL_3V3, 0);
			mdelay(10);
			/*Turn off MHL_1V2*/
			gpio_set_value(EDGE_GPIO_MHL_1V2, 0);
			mdelay(5);
			break;
		case 1:
			/*Turn on MHL_3V3*/
			gpio_set_value(EDGE_GPIO_MHL_3V3, 1);
			mdelay(10);
			/*Turn on MHL_1V2*/
			gpio_set_value(EDGE_GPIO_MHL_1V2, 1);
			mdelay(5);
			break;

		default:
			pr_info("[DISP]%s(%d) got unsupport parameter %d!\n", __func__, __LINE__, on);
			break;
	}

	pr_info("[DISP]%s(%d) OUT\n", __func__, __LINE__);

	return rc;
}

static T_MHL_PLATFORM_DATA mhl_sii_device_data = {
	.gpio_intr        = EDGE_GPIO_MHL_INT,
	.gpio_usb_sel     = EDGE_GPIO_MHL_USB_SEL,
	.gpio_reset       = EDGE_GPIO_MHL_RESET,
	.gpio_ddc_clk     = EDGE_GPIO_MHL_DDC_CLK,
	.gpio_ddc_data    = EDGE_GPIO_MHL_DDC_DATA,
	.ci2ca            = 1,
	.power            = mhl_sii_power,
	.enMhlD3Guard     = true,
};

static struct i2c_board_info i2c_mhl_sii_info[] =
{
	{
		I2C_BOARD_INFO(MHL_SII9234_I2C_NAME, 0x72 >> 1),
		.platform_data = &mhl_sii_device_data,
		.irq = TEGRA_GPIO_TO_IRQ(EDGE_GPIO_MHL_INT)
	}
};
#endif

static struct vibrator_platform_data vibrator_data = {
	.pwm_data={
		.name = "vibrator",
		.bank = 0,
	},
};

static struct platform_device tegra_vibrator = {
	.name= VIBRATOR_NAME,
	.id=-1,
	.dev = {
		.platform_data=&vibrator_data,
	},
};

static void tegra_vibrator_init(void)
{
	vibrator_data.pwm_gpio = TEGRA_GPIO_PH0;
	vibrator_data.ena_gpio = TEGRA_GPIO_PF1;

	platform_device_register(&tegra_vibrator);
}

static struct led_i2c_config lp5521_led_config[] = {
	{
		.name = "amber",
		.led_cur = 95,
		.led_lux = 100,
	},
	{
		.name = "green",
		.led_cur = 95,
		.led_lux = 100,
	},
	{
		.name = "button-backlight",
		.led_cur = 95,
		.led_lux = 55,
	},
};

static struct led_i2c_platform_data led_data = {
	.num_leds	= ARRAY_SIZE(lp5521_led_config),
	.led_config	= lp5521_led_config,
	.ena_gpio 	= TEGRA_GPIO_PY0,
};

static struct i2c_board_info i2c_led_devices[] = {
	{
		I2C_BOARD_INFO(LED_I2C_NAME, 0x32),
		.platform_data = &led_data,
		.irq = -1,
	},
};

static void leds_lp5521_init(void)
{
	i2c_register_board_info(1, i2c_led_devices,
		ARRAY_SIZE(i2c_led_devices));
}

static void config_enterprise_flashlight_gpios(void)
{
	int ret;
	printk("%s: start...", __func__);

	ret = gpio_request(FL_TORCH_EN, "fl_torch_en");

	if (ret < 0)
		pr_err("[FLT] %s: gpio_request failed for gpio %s\n", __func__, "FL_TORCH_EN");

	ret = gpio_direction_output(FL_TORCH_EN, 0);

	if (ret < 0) {
		pr_err("[FLT] %s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(FL_TORCH_EN);
		return;
	}
	gpio_export(FL_TORCH_EN, false);

	ret = gpio_request(FL_FLASH_EN, "fl_flash_en");

	if (ret < 0)
		pr_err("[FLT] %s: gpio_request failed for gpio %s\n", __func__, "FL_FLASH_EN");

	ret = gpio_direction_output(FL_FLASH_EN, 0);

	if (ret < 0) {
		pr_err("[FLT] %s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(FL_FLASH_EN);
		return;
	}
	gpio_export(FL_FLASH_EN, false);

	printk("%s: end...", __func__);
}
static struct flashlight_platform_data enterprise_flashlight_data = {
	.gpio_init  = config_enterprise_flashlight_gpios,
	.torch = FL_TORCH_EN,
	.flash = FL_FLASH_EN,
	.flash_duration_ms = 600
};
static struct platform_device enterprise_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev		= {
		.platform_data	= &enterprise_flashlight_data,
	},
};

static void enterprise_flashlight_init(void)
{
	platform_device_register(&enterprise_flashlight_device);
}

/* TI 128x Bluetooth begin */
static unsigned long retry_suspend;

int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_info("plat_kim_suspend\n");
        return 0;
}
int plat_kim_resume(struct platform_device *pdev)
{
	retry_suspend = 0;
        return 0;
}

struct ti_st_plat_data wilink_pdata = {
                .nshutdown_gpio = 160,
                .dev_name = "/dev/ttyHS2",
                .flow_cntrl = 1,
                .baud_rate = 3000000,
                .suspend = plat_kim_suspend,
                .resume = plat_kim_resume,
};

static struct platform_device btwilink_device = {
        .name = "btwilink",
        .id = -1,
};

static struct platform_device wl128x_device = {
        .name           = "kim",
        .id             = -1,
        .dev.platform_data = &wilink_pdata,
};

static noinline void __init enterprise_bt_wl128x(void)
{
        platform_device_register(&wl128x_device);
	platform_device_register(&btwilink_device);
        return;
}
/* TI 128x Bluetooth end */

static __initdata struct tegra_clk_init_table enterprise_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	5120000,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s0",	"pll_a_out0",	0,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio0",	"i2s0_sync",	0,		false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio2",	"i2s2_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi",		"pll_p",	0,		false},
	{ "vi_sensor",	"pll_p",	0,		false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_i2c_platform_data enterprise_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 384000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 384000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 384000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 390000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

// tlv320aic3008
static struct tegra_spi_device_controller_data dev_cdata_audio = {
	.is_hw_based_cs = false, /* bool is_hw_based_cs */
	.cs_setup_clk_count = 4, /* int cs_setup_clk_count */
	.cs_hold_clk_count = 2, /* int cs_hold_clk_count */
};

static struct spi_board_info spi_board_info_audio[] __initdata = {
	{
		.modalias	= "aic3008",
		.mode           = SPI_MODE_1,
		.bus_num        = 1,
		.chip_select    = 0,
		.max_speed_hz   = 10800000,
		.controller_data = &dev_cdata_audio,
	},
};

static struct platform_device enterprise_audio_device = {
	.name	= "tegra-snd-aic3008",
	.id	= 0,
	.dev	= {
		.platform_data  = NULL,
	},
};
// tlv320aic3008

/* HTC Camera SPI4 */
static struct tegra_spi_device_controller_data dev_cdata_rawchip = {
       .is_hw_based_cs = true, /* bool is_hw_based_cs */
       .cs_setup_clk_count = 1, /* int cs_setup_clk_count */
       .cs_hold_clk_count = 2, /* int cs_hold_clk_count */
};
static struct spi_board_info enterprise_spi_board_info_rawchip[] __initdata = {
       {
               .modalias       = "spi_rawchip",
               .mode           = SPI_MODE_0,
               .bus_num        = 3,
               .chip_select    = 1,
               .max_speed_hz   = 25000000,
               .controller_data = &dev_cdata_rawchip,
       },
};
EXPORT_SYMBOL_GPL(enterprise_spi_board_info_rawchip);

static int eva_use_ext_1v2(void)
{
	return 1;
}

static int eva_rawchip_vreg_on(void)
{
	struct clk *csus_clk = NULL;
	struct clk *sensor_clk = NULL;

	int ret;

	pr_info("[CAM] rawchip power on ++\n");

	/* enable main clock */
	csus_clk = clk_get(NULL, "csus");
	if (IS_ERR_OR_NULL(csus_clk)) {
		pr_err("%s: couldn't get csus clock\n", __func__);
		csus_clk = NULL;
		return -1;
	}
	sensor_clk = clk_get(NULL, "vi_sensor");
	if (IS_ERR_OR_NULL(sensor_clk)) {
		pr_err("%s: couldn't get sensor clock\n", __func__);
		sensor_clk = NULL;
		return -1;
	}
	clk_enable(csus_clk);
	clk_enable(sensor_clk);
	clk_set_rate(sensor_clk, 24000000);  /* 24MHz */

	/* rawchip power on sequence */
	tegra_gpio_disable(MCAM_SPI_CLK);
	tegra_gpio_disable(MCAM_SPI_CS0);
	tegra_gpio_disable(MCAM_SPI_DI);
	tegra_gpio_disable(MCAM_SPI_DO);
	tegra_gpio_disable(CAM_MCLK);

	gpio_direction_output(RAW_RSTN, 0);

	/* RAW_1V8_EN */
	gpio_direction_output(RAW_1V8_EN, 1);
	msleep(1);

	/* core */
	gpio_direction_output(CAM_D1V2_EN, 1);
	msleep(1);

	/* CAM SEL */
	gpio_direction_output(CAM_SEL, 0);
	msleep(1);

	/* RAW_RSTN */
	ret = gpio_direction_output(RAW_RSTN, 1);
	msleep(3);

	/* SPI send command to configure RAWCHIP here! */
	yushan_spi_write(0x0008, 0x7f);
	msleep(1);

	return 0;
}

static int eva_rawchip_vreg_off(void)
{
	struct clk *csus_clk = NULL;
	struct clk *sensor_clk = NULL;

	pr_info("[CAM] rawchip power off ++\n");

	/* disable main clock */
	csus_clk = clk_get(NULL, "csus");
	if (IS_ERR_OR_NULL(csus_clk)) {
		pr_err("%s: couldn't get csus clock\n", __func__);
		csus_clk = NULL;
		return -1;
	}
	sensor_clk = clk_get(NULL, "vi_sensor");
	if (IS_ERR_OR_NULL(sensor_clk)) {
		pr_err("%s: couldn't get sensor clock\n", __func__);
		sensor_clk = NULL;
		return -1;
	}
	clk_disable(csus_clk);
	clk_disable(sensor_clk);

	/* rawchip power off sequence */
	/* RAW RSTN */
	gpio_direction_output(RAW_RSTN, 0);
	msleep(3);

	/* digital */
	gpio_direction_output(CAM_D1V2_EN, 0);
	msleep(1);

	/* RAW_1V8_EN */
	gpio_direction_output(RAW_1V8_EN, 0);
	msleep(1);

	/* set gpio output low : O(L) */
	tegra_gpio_enable(MCAM_SPI_CLK);
	tegra_gpio_enable(MCAM_SPI_CS0);
	tegra_gpio_enable(MCAM_SPI_DI);
	tegra_gpio_enable(MCAM_SPI_DO);
	tegra_gpio_enable(CAM_MCLK);

	return 0;
}

static struct tegra_camera_rawchip_info tegra_rawchip_board_info = {
	.rawchip_reset  = RAW_RSTN,
	.rawchip_intr0  = TEGRA_GPIO_PR0,
	.rawchip_intr1  = TEGRA_GPIO_PEE1,
	.rawchip_spi_freq = 25,
	.rawchip_mclk_freq = 24,
	.camera_rawchip_power_on = eva_rawchip_vreg_on,
	.camera_rawchip_power_off = eva_rawchip_vreg_off,
	.rawchip_use_ext_1v2 = eva_use_ext_1v2,
};

static struct platform_device tegra_rawchip_device = {
	.name = "rawchip",
	.id	= -1,
	.dev	= {
		.platform_data = &tegra_rawchip_board_info,
	},
};
/* HTC Camera SPI4 */

/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.eng_cfg		= HS_EDE_U,
	.hpin_gpio		= TEGRA_GPIO_PW2,
	.key_gpio		= TEGRA_GPIO_PBB6,
	.key_enable_gpio	= 0,
	.mic_select_gpio	= 0,
};
static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

/* HTC_HEADSET_PMIC Driver */
static struct htc_headset_pmic_platform_data htc_headset_pmic_data_xe = {
	.eng_cfg	= HS_EDE_U,
	.driver_flag	= DRIVER_HS_PMIC_RPC_KEY,
	.adc_mic_bias	= {HS_DEF_MIC_ADC_12_BIT_MIN, HS_DEF_MIC_ADC_12_BIT_MAX},
	.adc_remote	= {0, 164, 165, 379, 380, 830},
};

static struct platform_device htc_headset_pmic_xe = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data_xe,
	},
};

static struct headset_adc_config htc_headset_mgr_config_xe[] = {
	{
		.type = HEADSET_UNPLUG,
		.adc_max = 4095,
		.adc_min = 3601,
	},
	{
		.type = HEADSET_MIC,
		.adc_max = 3600,
		.adc_min = 2951,
	},
	{
		.type = HEADSET_BEATS,
		.adc_max = 2950,
		.adc_min = 2101,
	},
	{
		.type = HEADSET_BEATS_SOLO,
		.adc_max = 2100,
		.adc_min = 1500,
	},
	{
		.type = HEADSET_NO_MIC,
		.adc_max = 1499,
		.adc_min = 0,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices_xe[] = {
	&htc_headset_pmic_xe,
	&htc_headset_gpio,
	/* Please put the headset detection driver on the last */
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data_xe = {
	.eng_cfg		= HS_EDE_U,
	.headset_devices_num	= ARRAY_SIZE(headset_devices_xe),
	.headset_devices	= headset_devices_xe,
	.headset_config_num	= ARRAY_SIZE(htc_headset_mgr_config_xe),
	.headset_config		= htc_headset_mgr_config_xe,
};

static struct platform_device htc_headset_mgr_xe = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data_xe,
	},
};

static void enterprise_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &enterprise_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &enterprise_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &enterprise_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &enterprise_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &enterprise_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}

static struct platform_device *enterprise_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};

struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static void headset_uart_init(void)
{
	int ret = 0;

	ret = gpio_request(TEGRA_GPIO_PY4,"headset_uart_TX");
	if (ret < 0) {
		printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_request failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PY4);
	return;
	}

	ret = gpio_direction_output(TEGRA_GPIO_PY4,0);
	if (ret < 0) {
		printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PY4);
	return;
	}

	ret = gpio_request(TEGRA_GPIO_PY5,"headset_uart_RX");
	if (ret < 0) {
		printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_request failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PY5);
	return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PY5);
	if (ret < 0) {
		printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PY5);
	return;
	}

	ret = gpio_request(TEGRA_GPIO_PZ0,"headset_uart_switch");
	if (ret < 0) {
		printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_request failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PZ0);
	return;
	}

	ret = gpio_direction_output(TEGRA_GPIO_PZ0,1);
	if (ret < 0) {
		printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PZ0);
	return;
	}
}

static struct tegra_uart_platform_data enterprise_uart_pdata;
static struct tegra_uart_platform_data enterprise_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	/* UARTD is the debug port. */
	pr_info("Selecting UARTD as the debug console\n");
	enterprise_uart_devices[3] = &debug_uartd_device;
	debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartd");

	/* Clock enable for the debug channel */
	if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->uartclk;
		pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
		c = tegra_get_clock_by_name("pll_p");
		if (IS_ERR_OR_NULL(c))
			pr_err("Not getting the parent clock pll_p\n");
		else
			clk_set_parent(debug_uart_clk, c);

		clk_enable(debug_uart_clk);
		clk_set_rate(debug_uart_clk, rate);
	} else {
		pr_err("Not getting the clock %s for debug console\n",
				debug_uart_clk->name);
	}
}

static void __init enterprise_uart_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	enterprise_uart_pdata.parent_clk_list = uart_parent_clk;
	enterprise_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	enterprise_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	enterprise_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	enterprise_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartb_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartc_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartd_device.dev.platform_data = &enterprise_uart_pdata;
	/* UARTE is used for loopback test purpose */
	tegra_uarte_device.dev.platform_data = &enterprise_loopback_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(enterprise_uart_devices,
				ARRAY_SIZE(enterprise_uart_devices));
}

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data cardhu_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static void __init enterprise_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	cardhu_spi_pdata.parent_clk_list = spi_parent_clk;
	cardhu_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);

	spi_register_board_info(spi_board_info_audio, ARRAY_SIZE(spi_board_info_audio));
        spi_register_board_info(enterprise_spi_board_info_rawchip, ARRAY_SIZE(enterprise_spi_board_info_rawchip));
	platform_device_register(&tegra_spi_device2);
	tegra_spi_device4.dev.platform_data = &cardhu_spi_pdata;
        platform_device_register(&tegra_spi_device4);
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *enterprise_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) ||  defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt0_device,
	&tegra_wdt1_device,
	&tegra_wdt2_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
	&tegra_rawchip_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};

/* Touchscreen GPIO addresses   */
#define TOUCH_GPIO_IRQ TEGRA_GPIO_PV1
#define TOUCH_GPIO_RST TEGRA_GPIO_PF3

static struct synaptics_i2c_rmi_platform_data edge_ts_3k_data[] = {
	{
		.version = 0x3330,
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.notifyFinger = NULL,
		.irqflags = IRQF_TRIGGER_FALLING,
		.default_config = 1,
		.cable_support = 1,
		.source = 1,
		.customer_register = {0xF9,0x64,0x74,0x32},
		.config = {
			0x35,0x44,0x30,0x38,
			0x00,0x3F,0x03,0x1E,0x05,0xB1,
			0x08,0x0B,0x19,0x19,0x00,0x00,0x4C,0x04,0x75,0x07,
			0x02,0x14,0x1E,0x05,0x37,0xA5,0x16,0xE8,0x03,0x01,
			0x3C,0x17,0x02,0x17,0x01,0xEC,0x4D,0x71,0x51,0xF8,
			0xA7,0xC8,0xAF,0x00,0x50,0x00,0x00,0x00,0x00,0x0A,
			0x04,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x19,0x01,
			0x00,0x0A,0x70,0x32,0xA2,0x02,0x28,0x0A,0x0A,0x64,
			0x16,0x0C,0x00,0x02,0xEE,0x00,0x80,0x03,0x0E,0x1F,
			0x11,0x38,0x00,0x13,0x08,0x1B,0x00,0x08,0xFF,0x00,
			0x06,0x0C,0x0D,0x0B,0x15,0x17,0x16,0x18,0x19,0x1A,
			0x1B,0x11,0x14,0x12,0x0F,0x0E,0x09,0x0A,0x07,0x02,
			0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x04,0x05,0x02,
			0x06,0x01,0x0C,0x07,0x08,0x0E,0x10,0x0F,0x12,0xFF,
			0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC8,0xC8,
			0xC8,0x59,0x57,0x55,0x53,0x52,0x50,0x4E,0x4D,0x00,
			0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x00,0xA0,0x0F,
			0xFF,0x28,0x00,0xC8,0x00,0xB3,0xC8,0xCD,0xA0,0x0F,
			0x00,0xC0,0x80,0x00,0x10,0x00,0x10,0x00,0x10,0x00,
			0x10,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x02,0x02,0x02,0x02,0x02,0x02,
			0x02,0x02,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
			0x58,0x5B,0x5D,0x5F,0x61,0x63,0x65,0x67,0x00,0x64,
			0x00,0x10,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,
			0xFF,0xFF,0xFF,0xFF,0x51,0x51,0x51,0x51,0xCD,0x0D,
			0x04
		}
	},
	{
		.version = 0x3330,
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.notifyFinger = NULL,
		.irqflags = IRQF_TRIGGER_FALLING,
		.cable_support = 1,
		.source = 0,
		.customer_register = {0xF9,0x64,0x74,0x32},
		.config = {
			0x35,0x4A,0x30,0x39,
			0x00,0x3F,0x03,0x1E,0x05,0xB1,
			0x08,0x0B,0x19,0x19,0x00,0x00,0x4C,0x04,0x75,0x07,
			0x02,0x14,0x1E,0x05,0x3A,0x57,0x1F,0xAF,0x02,0x01,
			0x3C,0x17,0x02,0x17,0x01,0xEC,0x4D,0x71,0x51,0xF8,
			0xA7,0xC8,0xAF,0x00,0x50,0x13,0x00,0x00,0x00,0x0A,
			0x04,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x19,0x01,
			0x00,0x0A,0x70,0x32,0xA2,0x02,0x28,0x0A,0x0A,0x64,
			0x16,0x0C,0x00,0x02,0xF0,0x00,0x80,0x03,0x0E,0x1F,
			0x11,0x38,0x00,0x13,0x08,0x1B,0x00,0x08,0xFF,0x00,
			0x06,0x0C,0x0D,0x0B,0x15,0x17,0x16,0x18,0x19,0x1A,
			0x1B,0x11,0x14,0x12,0x0F,0x0E,0x09,0x0A,0x07,0x02,
			0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x04,0x05,0x02,
			0x06,0x01,0x0C,0x07,0x08,0x0E,0x10,0x0F,0x12,0xFF,
			0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC8,0xC8,
			0xC8,0x62,0x60,0x5E,0x5C,0x5A,0x58,0x57,0x55,0x00,
			0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x00,0xB8,0x0B,
			0xCD,0x28,0x00,0xC8,0x00,0x80,0xC8,0xCD,0xB8,0x0B,
			0x00,0xC0,0x80,0x00,0x10,0x00,0x10,0x00,0x10,0x00,
			0x10,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x02,0x02,0x02,0x02,0x02,0x02,
			0x02,0x02,0x30,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
			0x7E,0x57,0x59,0x5B,0x5D,0x5F,0x61,0x63,0x00,0x64,
			0x00,0x10,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,
			0xFF,0xFF,0xFF,0xFF,0x51,0x51,0x51,0x51,0xCD,0x0D,
			0x04
		}
	},
	{
		.version = 0x3230,
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.flags = SYNAPTICS_FLIP_Y,
		.irqflags = IRQF_TRIGGER_FALLING,
		.default_config = 1,
		.source = 1,
		.config = {
			0x30, 0x30, 0x30, 0x31, 
			0x84, 0x0F, 0x03, 0x1E, 0x05, 0x20, 
			0xB1, 0x00, 0x0B, 0x19, 0x19, 0x00, 0x00, 0x4C, 0x04, 0x6C, 
			0x07, 0x1E, 0x05, 0x2D, 0x08, 0x0A, 0xA9, 0x02, 0x01, 0x3C, 
			0xFE, 0x43, 0xFE, 0x08, 0x4F, 0x65, 0x52, 0x49, 0xB5, 0xC1, 
			0x9C, 0x00, 0xB8, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x04, 0xAD, 
			0x00, 0x02, 0xF2, 0x00, 0x80, 0x02, 0x0D, 0x1E, 0x00, 0x2E, 
			0x00, 0x19, 0x04, 0x1E, 0x00, 0x10, 0xFF, 0x00, 0x00, 0x01, 
			0x02, 0x07, 0x0A, 0x09, 0x0E, 0x0F, 0x12, 0x14, 0x11, 0x1B, 
			0x1A, 0x19, 0x18, 0x16, 0x17, 0x15, 0x0B, 0x0D, 0x0C, 0x06, 
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x04, 0x05, 0x02, 0x06, 0x01, 
			0x0C, 0x07, 0x08, 0x0E, 0x10, 0x0F, 0x12, 0xFF, 0xFF, 0xFF, 
			0xFF, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x4F, 
			0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x00, 0x03, 0x06, 
			0x09, 0x0C, 0x0F, 0x12, 0x15, 0x00, 0xB8, 0x0B, 0xB3, 0xE8, 
			0x03, 0xB8, 0x0B, 0x33, 0x28, 0x80, 0xE8, 0x03, 0x00, 0xC0, 
			0x80, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x02, 0x02, 0x02, 0x02, 
			0x02, 0x02, 0x02, 0x02, 0x30, 0x20, 0x20, 0x20, 0x20, 0x20, 
			0x20, 0x20, 0x7E, 0x57, 0x5A, 0x5D, 0x60, 0x63, 0x66, 0x69, 
			0x3C, 0x43, 0x00, 0x1E, 0x19, 0x05, 0xFE, 0xFE, 0x3D, 0x08
		}
	},
	{
		.version = 0x3230,
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.irqflags = IRQF_TRIGGER_FALLING,
		.source = 0,
		.config = {
			0x30, 0x30, 0x30, 0x30, 
			0x84, 0xFF, 0x03, 0x1E, 0x05, 0x20, 0xB1, 0x00, 0x0B, 0x19, 
			0x19, 0x00, 0x00, 0x4C, 0x04, 0x6C, 0x07, 0x1E, 0x05, 0x2D, 
			0x16, 0x0F, 0x70, 0x03, 0x01, 0x37, 0xFF, 0x3D, 0xFE, 0x08, 
			0x4F, 0x99, 0x51, 0x6B, 0xCC, 0x34, 0xD4, 0x00, 0x70, 0x00, 
			0x00, 0x00, 0x00, 0x0A, 0x04, 0xC0, 0x00, 0x02, 0x02, 0x01, 
			0x80, 0x03, 0x0D, 0x1F, 0x00, 0x50, 0x00, 0x19, 0x04, 0x1E, 
			0x00, 0x10, 0xFF, 0x00, 0x06, 0x0C, 0x0D, 0x0B, 0x15, 0x17, 
			0x16, 0x18, 0x19, 0x1A, 0x1B, 0x11, 0x14, 0x12, 0x0F, 0x0E, 
			0x09, 0x0A, 0x07, 0x02, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 
			0xFF, 0x04, 0x05, 0x02, 0x06, 0x01, 0x0C, 0x07, 0x08, 0x0E, 
			0x10, 0x0F, 0x12, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x88, 0x38, 0x38, 0x38, 0x38, 0x38, 
			0x38, 0x38, 0x38, 0x00, 0x04, 0x09, 0x0E, 0x14, 0x19, 0x1E, 
			0x23, 0x00, 0x68, 0x04, 0x80, 0x68, 0x04, 0xDE, 0x2F, 0xC0, 
			0x14, 0xCC, 0x0D, 0x26, 0x00, 0xC0, 0x80, 0x00, 0x10, 0x00, 
			0x10, 0x00, 0x10, 0x00, 0x10, 0x80, 0x80, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x02, 0x02, 0x04, 0x07, 0x03, 0x0B, 0x04, 0x07, 
			0x20, 0x20, 0x30, 0x50, 0x20, 0x70, 0x30, 0x50, 0x76, 0x7B, 
			0x60, 0x5F, 0x5C, 0x5B, 0x6F, 0x6E
		}
	},

};

static struct i2c_board_info __initdata synaptics_i2c_info[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_3200_NAME, 0x20),
		.platform_data = &edge_ts_3k_data,
		.irq = TEGRA_GPIO_TO_IRQ(TOUCH_GPIO_IRQ),
	},
};

struct tegra_touchscreen_init __initdata synaptics_init_data = {
	.irq_gpio = TOUCH_GPIO_IRQ, /* GPIO1 Value for IRQ */
	.rst_gpio = TOUCH_GPIO_RST, /* GPIO2 Value for RST */
	.sv_gpio1 = {1, TOUCH_GPIO_RST, 0, 1},
	.sv_gpio2 = {1, TOUCH_GPIO_RST, 1, 100}, /* Valid, GPIOx, Set value, Delay */
	.ts_boardinfo = {1, synaptics_i2c_info, 1} /* BusNum, BoardInfo, Value */
};

static int __init enterprise_touch_init(void)
{
	gpio_request(TOUCH_GPIO_IRQ, "touch-irq");
	gpio_direction_input(TOUCH_GPIO_IRQ);

	gpio_request(TOUCH_GPIO_RST, "touch-reset");
	gpio_direction_output(TOUCH_GPIO_RST, 0);
	msleep(1);
	gpio_set_value(TOUCH_GPIO_RST, 1);
	msleep(100);

	i2c_register_board_info(1, synaptics_i2c_info, 1);

	return 0;
}

static void enterprise_usb_hsic_postsupend(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2);
#endif
}

static void enterprise_usb_hsic_preresume(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
#endif
}

static void enterprise_usb_hsic_phy_ready(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L0);
#endif
}

static void enterprise_usb_hsic_phy_off(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
#ifdef CONFIG_REMOVE_HSIC_L3_STATE
	baseband_xmm_set_power_status(BBXMM_PS_L2);
#else
	baseband_xmm_set_power_status(BBXMM_PS_L3);
#endif
#endif
}

static struct tegra_usb_phy_platform_ops hsic_xmm_plat_ops = {
	.post_suspend = enterprise_usb_hsic_postsupend,
	.pre_resume = enterprise_usb_hsic_preresume,
	.port_power = enterprise_usb_hsic_phy_ready,
	.post_phy_off = enterprise_usb_hsic_phy_off,
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_xmm_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = true,
	},
	.u_cfg.hsic = {
		.sync_start_delay = 9,
		.idle_wait_delay = 17,
		.term_range_adj = 0,
		.elastic_underrun_limit = 16,
		.elastic_overrun_limit = 16,
	},
	.ops = &hsic_xmm_plat_ops,
};



static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = ENT_TPS80031_IRQ_BASE +
				TPS80031_INT_VBUS_DET,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 7,
		.xcvr_setup = 14,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.builtin_host_disabled = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = "usb_vbus",
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static struct platform_device *tegra_usb_hsic_host_register(void)
{
	struct platform_device *pdev;
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

	val = platform_device_add_data(pdev, &tegra_ehci2_hsic_xmm_pdata,
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

static void tegra_usb_hsic_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

static void enterprise_usb_init(void)
{
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
}

static struct platform_device *enterprise_audio_devices[] __initdata = {
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
	&tegra_pcm_device,
	&enterprise_audio_device,
};

static void enterprise_audio_init(void)
{
	platform_add_devices(enterprise_audio_devices,
			ARRAY_SIZE(enterprise_audio_devices));
}

static int32_t get_tegra_adc_cb(void)
{
	int32_t adc = ~0;

	tps80032_adc_select_and_read(&adc, 6);
	adc = TPS80032ADC_12BIT(adc);
	return adc;
}

static void config_tegra_usb_id_gpios(bool output)
{
	if (output) {
		if (gpio_direction_output(TEGRA_GPIO_USB_ID, 1) < 0) {
			pr_info("[CABLE] %s: TEGRA_GPIO_USB_ID dir NG\n", __func__);
			return;
		}
	}
	else {
		if (gpio_direction_input(TEGRA_GPIO_USB_ID) < 0) {
			pr_info("[CABLE] %s: TEGRA_GPIO_USB_ID dir setup failed\n", __func__);
			return;
		}
	}
}

void config_tegra_desk_aud_gpios(bool output, bool out_val)
{
	if (output) {
		if (gpio_direction_output(TEGRA_GPIO_DESK_AUD, out_val) < 0) {
			pr_info("[CABLE] %s: TEGRA_GPIO_DESK_AUD dir NG\n", __func__);
			return;
		}
	}
	else {
		if (gpio_direction_input(TEGRA_GPIO_DESK_AUD) < 0) {
			pr_info("[CABLE] %s: TEGRA_GPIO_DESK_AUD dir setup failed\n", __func__);
			return;
		}
	}
}
EXPORT_SYMBOL(config_tegra_desk_aud_gpios);

static void tegra_usb_dpdn_switch(int path)
{
	int polarity = 1; /* high = mhl */
	int mhl = (path == PATH_MHL);

	switch (path) {
		case PATH_USB:
		case PATH_MHL:
			pr_info("[CABLE] %s: Set %s path\n", __func__, mhl ? "MHL" : "USB");
			gpio_set_value(TEGRA_GPIO_MHL_USB_SEL, (mhl ^ !polarity) ? 1 : 0);
			break;
	}

#ifdef CONFIG_TEGRA_HDMI_MHL
	sii9234_change_usb_owner((path == PATH_MHL) ? 1 : 0);
#endif
}

static void cable_tegra_gpio_init(void);
static struct cable_detect_platform_data cable_detect_pdata = {
	.detect_type		= CABLE_TYPE_PMIC_ADC,
	.usb_id_pin_gpio	= TEGRA_GPIO_USB_ID,
	.mhl_reset_gpio		= TEGRA_GPIO_MHL_RST,
	.usb_dpdn_switch	= tegra_usb_dpdn_switch,
	.config_usb_id_gpios	= config_tegra_usb_id_gpios,
	.config_desk_aud_gpios  = config_tegra_desk_aud_gpios,
	.get_adc_cb		= get_tegra_adc_cb,
	.cable_gpio_init	= cable_tegra_gpio_init,
#ifdef CONFIG_TEGRA_HDMI_MHL
	.mhl_internal_3v3 	= 1,
#endif
};

static struct platform_device cable_detect_device = {
	.name	= "cable_detect",
	.id	= -1,
	.dev	= {
		.platform_data = &cable_detect_pdata,
	},
};

static void cable_tegra_gpio_init(void)
{
	int ret;
	if (cable_detect_pdata.usb_id_pin_gpio >= 0) {
		if (gpio_request(cable_detect_pdata.usb_id_pin_gpio, "USB_ID_WAKE") < 0) {
			pr_err("[CABLE:ERR] %s: cable_detect_pdata.usb_id_pin_gpio req NG\n", __func__);
			return;
		}
		if (gpio_direction_input(cable_detect_pdata.usb_id_pin_gpio) < 0) {
			pr_err("[CABLE:ERR] %s: cable_detect_pdata.usb_id_pin_gpio dir setup failed\n", __func__);
			return;
		}
	}
	ret = gpio_request(TEGRA_GPIO_PB3, "USB_HOST");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	if (gpio_direction_output(TEGRA_GPIO_PB3, 0) < 0) {
		pr_info("[CABLE] %s: TEGRA_GPIO_PB3(USB_HOST) dir NG\n", __func__);
		return;
	}
}

static void enterprise_cable_detect_init(void)
{
	platform_device_register(&cable_detect_device);
}

static struct baseband_power_platform_data tegra_baseband_power_data = {
	.baseband_type = BASEBAND_XMM,
	.modem = {
		.xmm = {
			.bb_rst = XMM6260_GPIO_BB_RST,
			.bb_on = XMM6260_GPIO_BB_ON,
			.ipc_bb_wake = XMM6260_GPIO_IPC_BB_WAKE,
			.ipc_ap_wake = XMM6260_GPIO_IPC_AP_WAKE,
			.ipc_hsic_active = XMM6260_GPIO_IPC_HSIC_ACTIVE,
			.ipc_hsic_sus_req = XMM6260_GPIO_IPC_HSIC_SUS_REQ,
            .bb_vdd_en = BB_VDD_EN,
            .bb_rst_pwrdn = AP2BB_RST_PWRDWNn,
            .bb_rst2 = BB2AP_RST2,
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

static struct platform_device tegra_baseband_power2_device = {
	.name = "baseband_xmm_power2",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};

static struct platform_device simhotswap_device = {
    .name   = "htc_simhotswap",
    .id = -1,
};

static void enterprise_modem_init(void)
{
	int ret;

		pr_info("%s: enable baseband gpio(s)\n", __func__);

		tegra_baseband_power_data.hsic_register = &tegra_usb_hsic_host_register;
		tegra_baseband_power_data.hsic_unregister = &tegra_usb_hsic_host_unregister;

		platform_device_register(&tegra_baseband_power_device);
		platform_device_register(&tegra_baseband_power2_device);
		platform_device_register(&simhotswap_device);

		printk(KERN_INFO"%s: gpio config for sim_det#.", __func__);

		ret = gpio_request(TEGRA_GPIO_PI5, "sim_det#");
		if (ret < 0)
			pr_err("[FLT] %s: gpio_request failed for gpio %s\n",
				__func__, "sin_init");
		ret = gpio_direction_input(TEGRA_GPIO_PI5);

		if (ret < 0) {
			pr_err("[FLT] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(TEGRA_GPIO_PI5);
			return;
		}
		gpio_export(TEGRA_GPIO_PI5, true);

			printk(KERN_INFO"%s: gpio config for core dump when radio fatal error.", __func__);

				ret = gpio_request(TEGRA_GPIO_PN2, "core_dump");
				if (ret < 0)
					pr_err("[FLT] %s: gpio_request failed for gpio %s\n",
						__func__, "core_dump");

				ret = gpio_direction_input(TEGRA_GPIO_PN2);
				if (ret < 0) {
					pr_err("[FLT] %s: gpio_direction_input failed %d\n", __func__, ret);
					gpio_free(TEGRA_GPIO_PN2);
					return;
				}
				gpio_export(TEGRA_GPIO_PN2, true);
}

static void enterprise_baseband_init(void)
{
	enterprise_modem_init();
}

static ssize_t Aproj_virtual_keys_show_XC(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_BACK)       ":113:1345:124:86"
			":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":360:1345:124:86"
			":" __stringify(EV_KEY) ":" __stringify(KEY_F12) ":610:1345:124:86"
			"\n");
}
static struct kobj_attribute Aproj_virtual_keys_attr_XC = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &Aproj_virtual_keys_show_XC,
};

static struct attribute *Aproj_properties_attrs_XC[] = {
	&Aproj_virtual_keys_attr_XC.attr,
	NULL
};

static struct attribute_group Aproj_properties_attr_group_XC = {
	.attrs = Aproj_properties_attrs_XC,
};

static struct keyreset_platform_data enr_reset_keys_pdata = {
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

static struct platform_device enr_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &enr_reset_keys_pdata,
};

static void __init tegra_endeavoru_init(void)
{
	int board_id = 0;
	int val = 0;
	struct kobject *properties_kobj;

	tegra_thermal_init(&thermal_data,
				throttle_list,
				ARRAY_SIZE(throttle_list));

	board_id = htc_get_pcbid_info();
	tegra_clk_init_from_table(enterprise_clk_init_table);
	endeavoru_pinmux_init();
	enterprise_i2c_init();
	enterprise_uart_init();
	enterprise_spi_init();
	enterprise_usb_init();
	tegra_ram_console_debug_init();

	platform_add_devices(enterprise_devices, ARRAY_SIZE(enterprise_devices));

	platform_device_register(&htc_headset_mgr_xe);

	enterprise_regulator_init();
	tegra_io_dpd_init();
	enterprise_sdhci_init();
	headset_uart_init();

#ifdef CONFIG_TEGRA_EDP_LIMITS
	enterprise_edp_init();
#endif
	A_PROJECT_keys_init();

	enterprise_audio_init();

#ifdef CONFIG_TEGRA_HDMI_MHL
	i2c_register_board_info(4, i2c_mhl_sii_info,
			ARRAY_SIZE(i2c_mhl_sii_info));
#endif
	enterprise_touch_init();
	enterprise_baseband_init();
	enterprise_panel_init();
	enterprise_bt_wl128x();
	enterprise_emc_init();
	enterprise_sensors_init();
	endeavoru_cam_init();

	if (platform_device_register(&enr_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);
		properties_kobj = kobject_create_and_add("board_properties", NULL);

	if (properties_kobj) {
		val = sysfs_create_group(properties_kobj, &Aproj_properties_attr_group_XC);
		if (val)
			return;
	}
	enterprise_suspend_init();
	tegra_release_bootloader_fb();
	tegra_vibrator_init();

	leds_lp5521_init();
	enterprise_flashlight_init();
#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	enterprise_cable_detect_init();
#endif
}

static void __init tegra_endeavoru_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	tegra_reserve(0, SZ_4M, SZ_8M);
#else
	tegra_reserve(SZ_128M, SZ_4M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
}

MACHINE_START(ENDEAVORU, "endeavoru")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_endeavoru_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_endeavoru_init,
MACHINE_END
