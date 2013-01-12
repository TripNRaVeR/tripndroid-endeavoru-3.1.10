/*
 * arch/arm/mach-tegra/board-endeavoru-sensors.c
 *
 * Copyright (c) 2011-2012, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mpu.h>
#include <linux/nct1008.h>
#include <linux/err.h>
#include <linux/platform_data/ina230.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/tps65200.h>
#include <linux/nfc/pn544.h>
#include <linux/akm8975_htc.h>
#include <linux/bma250.h>
#include <linux/ewtzmu2.h>
#include <linux/cm3628.h>
#include <linux/cm3629.h>

#include <media/tps61050.h>

#include <mach/gpio.h>
#include <mach/edp.h>
#include <mach/thermal.h>
#include <mach/board_htc.h>
#include <mach/htc_battery_tps80032.h>

#include "cpu-tegra.h"
#include "gpio-names.h"
#include "board-endeavoru.h"
#include "board.h"

#define SENSOR_MPU_NAME "mpu3050"

static struct cm3628_platform_data cm3628_pdata = {
	.intr = TEGRA_GPIO_PK2,
	.levels = { 12, 14, 16, 41, 83, 3561, 6082, 6625, 7168, 65535},
	.golden_adc = 0x1145,
	.power = NULL,
	.ALS_slave_address = 0xC0>>1,
	.PS_slave_address = 0xC2>>1,
	.check_interrupt_add = 0x2C>>1	,
	.is_cmd = CM3628_ALS_IT_400ms | CM3628_ALS_PERS_2,
	.ps_thd_set = 0x4,
	.ps_conf2_val = 0,
	.ps_calibration_rule = 1,
	.ps_reset_thd = 1,
	.ps_conf1_val = CM3628_PS_DR_1_320 | CM3628_PS_IT_1_3T |
			CM3628_PS_PERS_4,
	.ps_thd_no_cal = 0x10,
	.ps_thd_with_cal = 0x4,
};

static struct cm3629_platform_data cm3629_pdata = {
	.model = CAPELLA_CM36282,
	.ps_select = CM3629_PS1_ONLY,
	.intr = TEGRA_GPIO_PK2,
	.levels = { 12, 14, 16, 176, 361, 4169, 6891, 9662, 12433, 65535},
	.golden_adc = 0x13AA,
	.power = NULL,
	.cm3629_slave_address = 0xC0>>1,
	.ps_calibration_rule = 1,
	.ps1_thd_set = 0x3,
	.ps1_thd_no_cal = 0x3,
	.ps1_thd_with_cal = 0x3,
	.ps_conf1_val = CM3629_PS_DR_1_320 | CM3629_PS_IT_1_3T |
			CM3629_PS1_PERS_4,
	.ps_conf2_val = CM3629_PS_ITB_1 | CM3629_PS_ITR_1 |
			CM3629_PS2_INT_DIS | CM3629_PS1_INT_DIS,
	.ps_conf3_val = CM3629_PS2_PROL_32,
	.dark_level = 3,
};

static struct i2c_board_info i2c_CM3628_devices[] = {
	{
		I2C_BOARD_INFO("cm3628", 0xc0 >> 1),
		.platform_data = &cm3628_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
	}
};

static struct i2c_board_info i2c_CM3629_devices[] = {
	{
		I2C_BOARD_INFO(CM3629_I2C_NAME, 0xC0 >> 1),
		.platform_data = &cm3629_pdata,
			.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
	},
};

static void psensor_init(void)
{
	if(ps_type) {
		i2c_register_board_info(0,
				i2c_CM3629_devices, ARRAY_SIZE(i2c_CM3629_devices));
		pr_info("[PS][cm3629]%s ps_type = %d\n", __func__, ps_type);
	}
	else {
		i2c_register_board_info(0,
				i2c_CM3628_devices, ARRAY_SIZE(i2c_CM3628_devices));
		pr_info("[PS][cm3628]%s ps_type = %d\n", __func__, ps_type);
	}
}

static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data,
						shutdown_temp);
}

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	thermal_device->name = "nct1008";
	thermal_device->data = data;
	thermal_device->id = THERMAL_DEVICE_ID_NCT_EXT;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = nct_get_temp;
	thermal_device->get_temp_low = nct_get_temp_low;
	thermal_device->set_limits = nct_set_limits;
	thermal_device->set_alert = nct_set_alert;
	thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_device_register(thermal_device);
}

static struct nct1008_platform_data enterprise_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
	.probe_callback = nct1008_probe_callback,
};

static struct i2c_board_info enterprise_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PCC2),
		.platform_data = &enterprise_nct1008_pdata,
	}
};

static void enterprise_nct1008_init(void)
{
	int ret;

	ret = gpio_request(TEGRA_GPIO_PCC2, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PCC2);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PCC2);
		return;
	}

	i2c_register_board_info(4, enterprise_i2c4_nct1008_board_info,
				ARRAY_SIZE(enterprise_i2c4_nct1008_board_info));
}

void config_ruby_gyro_diag_gpios(bool pulldown)
{
/*
	if (pulldown) {
		config_gpio_table(gyro_DIAG_PIN_pull_down, ARRAY_SIZE(gyro_DIAG_PIN_pull_down));
		printk(KERN_INFO "%s %d pull down\n",  __func__, RUBY_GPIO_GYRO_DIAG);
	} else {
		config_gpio_table(gyro_DIAG_PIN_no_pull, ARRAY_SIZE(gyro_DIAG_PIN_no_pull));
		printk(KERN_INFO "%s %d input none pull\n",  __func__, RUBY_GPIO_GYRO_DIAG);
	}
*/
}

static struct pana_gyro_platform_data pana_gyro_pdata = {
	.acc_dir = 0x06,
	.acc_polarity = 0x07,
	.gyro_dir = 0x06,
	.gyro_polarity = 0x07,
	.mag_dir = 0x06,
	.mag_polarity = 0x07,
	.sleep_pin = TEGRA_GPIO_PR2,
	.config_gyro_diag_gpios = config_ruby_gyro_diag_gpios,
};

static struct i2c_board_info __initdata pana_gyro_GSBI12_boardinfo[] = {
	{
		I2C_BOARD_INFO("ewtzmu2", 0x69 >> 0),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI6),
		.platform_data = &pana_gyro_pdata,
	},
};

static struct bma250_platform_data gsensor_bma250_platform_data = {
	.intr = TEGRA_GPIO_PO5,
	.chip_layout = 1,
};

static struct i2c_board_info i2c_bma250_devices[] = {
	{
		I2C_BOARD_INFO(BMA250_I2C_NAME, 0x19 >> 0),
		.platform_data = &gsensor_bma250_platform_data,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO5),
	},
};

static struct akm8975_platform_data compass_platform_data_xc = {
	.layouts = RUBY_LAYOUTS_XC,
	.use_pana_gyro = 1,
};

static struct i2c_board_info i2c_akm8975_devices_xc[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x0D >> 0),
		.platform_data = &compass_platform_data_xc,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ2),
	},
};

static void config_nfc_gpios(void)
{
    int ret = 0;

    ret = gpio_direction_output(RUBY_GPIO_NFC_VEN, 0);
    if (ret < 0) {
        pr_err("TEGRA_GPIO_PM5 output failed\n");
        pr_err("[NFC] %s: gpio_direction_output failed %d\n", __func__, ret);
        gpio_free(RUBY_GPIO_NFC_VEN);
        return;
    }

    ret = gpio_direction_output(RUBY_GPIO_NFC_DL, 0);
    if (ret < 0) {
        pr_err("TEGRA_GPIO_PM6 output failed\n");
        pr_err("[NFC] %s: gpio_direction_output failed %d\n", __func__, ret);
        gpio_free(RUBY_GPIO_NFC_DL);
        return;
    }

    ret = gpio_direction_input(RUBY_GPIO_NFC_INT);
    if (ret < 0) {
        pr_err("TEGRA_GPIO_PY6 output failed\n");
        pr_err("[NFC] %s: gpio_direction_output failed %d\n", __func__, ret);
        gpio_free(RUBY_GPIO_NFC_INT);
        return;
    }

    gpio_set_value(RUBY_GPIO_NFC_VEN, 1);
    pr_info("%s\n", __func__);

}

static struct pn544_i2c_platform_data nfc_platform_data = {
    .gpio_init  = config_nfc_gpios,
    .irq_gpio = RUBY_GPIO_NFC_INT,
    .ven_gpio = RUBY_GPIO_NFC_VEN,
    .firm_gpio = RUBY_GPIO_NFC_DL,
    .ven_isinvert = 1,
};

static struct i2c_board_info pn544_i2c_boardinfo[] = {
    {
        I2C_BOARD_INFO(PN544_I2C_NAME, 0x50 >> 1),
        .platform_data = &nfc_platform_data,
        .irq = TEGRA_GPIO_TO_IRQ(RUBY_GPIO_NFC_INT),
    },
};

static void edge_nfc_init(void)
{
    i2c_register_board_info(0, pn544_i2c_boardinfo,
            ARRAY_SIZE(pn544_i2c_boardinfo));
}

static inline void ENR_msleep(u32 t)
{
	/*
	If timer value is between ( 10us - 20ms),
	ENR_usleep_range() is recommended.
	Please read Documentation/timers/timers-howto.txt.
	*/
	usleep_range(t*1000, t*1000 + 500);
}

static inline void ENR_usleep(u32 t)
{
        usleep_range(t, t + 500);
}

static void enterprise_gsensor_irq_init(void)
{
	int ret = 0;

	pr_info("[GSNR] g-sensor irq_start...\n");

	ret = gpio_request(TEGRA_GPIO_PN5, "GSNR_INT");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PN5);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PN5);
		return;
	}
		
	pr_info("[GSNR] g-sensor irq end...\n");

}

static void enterprise_gyro_diag_init(void)
{
	int ret = 0;

	pr_info("[GYRO] gyro diag_start...\n");
		ret = gpio_request(TEGRA_GPIO_PH3, "GYRO_DIAG");
		if (ret < 0) {
			pr_err("%s: gpio_request failed %d\n", __func__, ret);
			return;
		}

		ret = gpio_direction_input(TEGRA_GPIO_PH3);
		if (ret < 0) {
			pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
			gpio_free(TEGRA_GPIO_PH3);
			return;
		}

	pr_info("[GYRO] gyro diag irq end...\n");

}

static void __init enterprise_mpuirq_init(void)
{
	int ret = 0;

	ret = gpio_request(TEGRA_GPIO_PI6, SENSOR_MPU_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PI6);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PI6);
		return;
	}
}

static void enterprise_gyro_sleep_pin(void)
{

	int ret = 0;
	ret = gpio_request(TEGRA_GPIO_PR2, "sleep_pin");
	pr_info("[GYRO] mog sleep pin...\n");
	if (ret < 0) {
	pr_err("TEGRA_GPIO_PR2 request failes\n");
			pr_err("%s: gpio_request failed %d\n", __func__, ret);
			return;
	}
	ret = gpio_direction_output(TEGRA_GPIO_PR2, 1);
	if (ret < 0) {
	pr_err("TEGRA_GPIOPR2, output failed\n");
			pr_err("[sleep_pin] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(TEGRA_GPIO_PR2);
			return;
	}
}

static void enterprise_comp_irq_init(void)
{
	int ret = 0;

	ret = gpio_request(TEGRA_GPIO_PJ2, "COMP_INT");
	if (ret < 0) {
		pr_err("[COMP] %s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PJ2);
	if (ret < 0) {
		pr_err("[COMP] %s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PJ2);
		return;
	}
}

struct enterprise_battery_gpio {
	int gpio;
	const char *label;
};

#define TEGRA_BATTERY_GPIO(_gpio, _label)	\
	{					\
		.gpio = _gpio,			\
		.label = _label,		\
	}

struct enterprise_battery_gpio enterprise_battery_gpio_data[] ={
	[0] = TEGRA_BATTERY_GPIO(TEGRA_GPIO_PU4, "mbat_in"),
	[1] = TEGRA_BATTERY_GPIO(TEGRA_GPIO_PW0, "chg_stat"),
	[2] = TEGRA_BATTERY_GPIO(TEGRA_GPIO_PX5, "chg_int"),
};

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.gpio_mbat_in = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU4),
	.gpio_mbat_in_trigger_level = MBAT_IN_LOW_TRIGGER,
	.guage_driver = GUAGE_NONE,
	.charger = SWITCH_CHARGER_TPS65200,
	.vzero_clb_channel = -1,
	.volt_adc_offset = 0,
	.power_off_by_id = 1,
};

static struct platform_device htc_battery_pdev = {
	.name	= "htc_battery",
	.id	= -1,
	.dev	= {
	        .platform_data = &htc_battery_pdev_data,
	},
};

static struct tps65200_platform_data tps65200_data = {
	.gpio_chg_stat = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW0),
	.gpio_chg_int  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PX5),
};

static struct i2c_board_info tps_65200_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65200", 0xD4 >> 1),
		.platform_data = &tps65200_data,
	},
};

static void enterprise_battery_init(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(enterprise_battery_gpio_data); i++) {
		ret = gpio_request(enterprise_battery_gpio_data[i].gpio,
				   enterprise_battery_gpio_data[i].label);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto bat_fail_free_gpio;
		}

		ret = gpio_direction_input(enterprise_battery_gpio_data[i].gpio);
		if (ret < 0) {
			pr_err("%s: gpio_direction_input failed for gpio #%d\n",
				__func__, i);
			goto bat_fail_free_gpio;
		}
	}

	platform_device_register(&htc_battery_pdev);

	i2c_register_board_info(4, tps_65200_boardinfo,
					ARRAY_SIZE(tps_65200_boardinfo));

	return;

bat_fail_free_gpio:
	pr_err("%s enterprise_battery_init failed!\n", __func__);
	while (i--)
		gpio_free(enterprise_battery_gpio_data[i].gpio);
}

#define ENTERPRISE_INA230_ENABLED 0

#if ENTERPRISE_INA230_ENABLED
static struct ina230_platform_data ina230_platform = {
	.rail_name = "VDD_AC_BAT",
	.current_threshold = TEGRA_CUR_MON_THRESHOLD,
	.resistor = TEGRA_CUR_MON_RESISTOR,
	.min_cores_online = TEGRA_CUR_MON_MIN_CORES,
};

static struct i2c_board_info enterprise_i2c0_ina230_info[] = {
	{
		I2C_BOARD_INFO("ina230", 0x42),
		.platform_data = &ina230_platform,
		.irq = -1,
	},
};

static int __init enterprise_ina230_init(void)
{
	return i2c_register_board_info(0, enterprise_i2c0_ina230_info,
				       ARRAY_SIZE(enterprise_i2c0_ina230_info));
}
#endif

int __init enterprise_sensors_init(void)
{
	int ret = 0;
	psensor_init();

	enterprise_comp_irq_init();

	enterprise_gsensor_irq_init(); 
	enterprise_mpuirq_init();
	enterprise_gyro_diag_init();

	i2c_register_board_info(0, pana_gyro_GSBI12_boardinfo, ARRAY_SIZE(pana_gyro_GSBI12_boardinfo));
	i2c_register_board_info(0, i2c_bma250_devices, ARRAY_SIZE(i2c_bma250_devices));
	i2c_register_board_info(0, i2c_akm8975_devices_xc, ARRAY_SIZE(i2c_akm8975_devices_xc));
	
	enterprise_gyro_sleep_pin();

	enterprise_battery_init();
	enterprise_nct1008_init();

#if ENTERPRISE_INA230_ENABLED
	enterprise_ina230_init();
#endif
	edge_nfc_init();
	return ret;
}
