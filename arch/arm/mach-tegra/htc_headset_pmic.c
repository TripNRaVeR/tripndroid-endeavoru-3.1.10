/*
 *
 * /arch/arm/mach-msm/htc_headset_pmic.c
 *
 * HTC PMIC headset driver.
 *
 * Copyright (C) 2010 HTC, Inc.
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

#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/tps80032_adc.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_pmic.h>
#include "../../../arch/arm/mach-tegra/board.h"

#define DRIVER_NAME "HS_PMIC"

static struct workqueue_struct *detect_wq;
static void detect_35mm_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(detect_35mm_work, detect_35mm_do_work);

static struct workqueue_struct *button_wq;
static void button_pmic_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(button_pmic_work, button_pmic_work_func);

static struct htc_35mm_pmic_info *hi;

static int hs_pmic_hpin_state(void)
{
	HS_DBG();

	return gpio_get_value(hi->pdata.hpin_gpio);
}

static int hs_pmic_remote_adc(int *adc)
{
	int ret = 0;
	int32_t result;	
	int adc_channel = 20;

	HS_DBG();

	if ((hi->pdata.eng_cfg == HS_BLE))
	{
		adc_channel = 6;
	}
	else if ((hi->pdata.eng_cfg == HS_EDE_U) || (hi->pdata.eng_cfg == HS_EDE_TD))
	{
		adc_channel = 4;
	}
	else
	{
		HS_LOG("Unknown adc channel");
		return 0;
	}

	ret = tps80032_adc_select_and_read(&result, adc_channel);
	if (ret !=0) {
		pr_err("%s: adc volt read fail %d\n", __func__, ret);
	}
	*adc = result;

	HS_LOG("Remote ADC %d (0x%X)", *adc, *adc);

	return 1;
}

static int hs_pmic_mic_status(void)
{
	int adc = 0;
	int mic = HEADSET_UNKNOWN_MIC;

	HS_DBG();

	if (!hs_pmic_remote_adc(&adc))
		return HEADSET_UNKNOWN_MIC;

	if (adc >= hi->pdata.adc_mic_bias[0] &&
	    adc <= hi->pdata.adc_mic_bias[1])
		mic = HEADSET_MIC;
	else if (adc < hi->pdata.adc_mic_bias[0])
		mic = HEADSET_NO_MIC;
	else
		mic = HEADSET_UNKNOWN_MIC;

	return mic;
}

static int hs_pmic_adc_to_keycode(int adc)
{
	int key_code = HS_MGR_KEY_INVALID;

	HS_DBG();

	if (!hi->pdata.adc_remote[5])
		return HS_MGR_KEY_INVALID;

	if (adc >= hi->pdata.adc_remote[0] &&
	    adc <= hi->pdata.adc_remote[1])
		key_code = HS_MGR_KEY_PLAY;
	else if (adc >= hi->pdata.adc_remote[2] &&
		 adc <= hi->pdata.adc_remote[3])
		key_code = HS_MGR_KEY_BACKWARD;
	else if (adc >= hi->pdata.adc_remote[4] &&
		 adc <= hi->pdata.adc_remote[5])
		key_code = HS_MGR_KEY_FORWARD;
	else if (adc > hi->pdata.adc_remote[5])
		key_code = HS_MGR_KEY_NONE;

	if (key_code != HS_MGR_KEY_INVALID)
		HS_LOG("Key code %d", key_code);
	else
		HS_LOG("Unknown key code %d", key_code);

	return key_code;
}

static void hs_pmic_rpc_key(int adc)
{
	int key_code = hs_pmic_adc_to_keycode(adc);

	HS_DBG();

	if (key_code != HS_MGR_KEY_INVALID)
		hs_notify_key_event(key_code);
}

static void hs_pmic_key_enable(int enable)
{
	HS_DBG();

	if (hi->pdata.key_enable_gpio)
		gpio_set_value(hi->pdata.key_enable_gpio, enable);
}

static void detect_35mm_do_work(struct work_struct *work)
{
	int insert = 0;

	HS_DBG();

	insert = gpio_get_value(hi->pdata.hpin_gpio) ? 0 : 1;
	hs_notify_plug_event(insert);
}

static irqreturn_t detect_irq_handler(int irq, void *data)
{
	unsigned int irq_mask = IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW;

	hs_notify_hpin_irq();

	HS_DBG();

	hi->hpin_irq_type ^= irq_mask;
	irq_set_irq_type(hi->pdata.hpin_irq, hi->hpin_irq_type);

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	queue_delayed_work(detect_wq, &detect_35mm_work, hi->hpin_debounce);

	return IRQ_HANDLED;
}

static void button_pmic_work_func(struct work_struct *work)
{
	HS_DBG();
	hs_notify_key_irq();
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	unsigned int irq_mask = IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW;

	HS_DBG();

	hi->key_irq_type ^= irq_mask;
	irq_set_irq_type(hi->pdata.key_irq, hi->key_irq_type);

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	queue_delayed_work(button_wq, &button_pmic_work, HS_JIFFIES_ZERO);

	return IRQ_HANDLED;
}

static int hs_pmic_request_irq(unsigned int gpio, unsigned int *irq,
			       irq_handler_t handler, unsigned long flags,
			       const char *name, unsigned int wake)
{
	int ret = 0;

	HS_DBG();

	ret = gpio_request(gpio, name);
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		gpio_free(gpio);
		return ret;
	}

	if (!(*irq)) {
		ret = gpio_to_irq(gpio);
		if (ret < 0) {
			gpio_free(gpio);
			return ret;
		}
		*irq = (unsigned int) ret;
	}

	ret = request_any_context_irq(*irq, handler, flags, name, NULL);
	if (ret < 0) {
		gpio_free(gpio);
		return ret;
	}

	return 1;
}

static void hs_pmic_register(void)
{
	struct headset_notifier notifier;

	if (hi->pdata.hpin_gpio) {
		notifier.id = HEADSET_REG_HPIN_GPIO;
		notifier.func = hs_pmic_hpin_state;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.driver_flag & DRIVER_HS_PMIC_RPC_KEY) {
		notifier.id = HEADSET_REG_REMOTE_ADC;
		notifier.func = hs_pmic_remote_adc;
		headset_notifier_register(&notifier);

		notifier.id = HEADSET_REG_REMOTE_KEYCODE;
		notifier.func = hs_pmic_adc_to_keycode;
		headset_notifier_register(&notifier);

		notifier.id = HEADSET_REG_RPC_KEY;
		notifier.func = hs_pmic_rpc_key;
		headset_notifier_register(&notifier);

		notifier.id = HEADSET_REG_MIC_STATUS;
		notifier.func = hs_pmic_mic_status;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.key_enable_gpio) {
		notifier.id = HEADSET_REG_KEY_ENABLE;
		notifier.func = hs_pmic_key_enable;
		headset_notifier_register(&notifier);
	}
}

static int htc_headset_pmic_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct htc_headset_pmic_platform_data *pdata = pdev->dev.platform_data;

	hi = kzalloc(sizeof(struct htc_35mm_pmic_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	hi->pdata.eng_cfg = pdata->eng_cfg;
	hi->pdata.driver_flag = pdata->driver_flag;
	hi->pdata.hpin_gpio = pdata->hpin_gpio;
	hi->pdata.hpin_irq = pdata->hpin_irq;
	hi->pdata.key_gpio = pdata->key_gpio;
	hi->pdata.key_irq = pdata->key_irq;
	hi->pdata.key_enable_gpio = pdata->key_enable_gpio;
	hi->pdata.hs_controller = pdata->hs_controller;
	hi->pdata.hs_switch = pdata->hs_switch;
	hi->pdata.adc_mic = pdata->adc_mic;

	if (!hi->pdata.adc_mic)
		hi->pdata.adc_mic = HS_DEF_MIC_ADC_12_BIT_MIN;

	if (pdata->adc_mic_bias[0] && pdata->adc_mic_bias[1]) {
		memcpy(hi->pdata.adc_mic_bias, pdata->adc_mic_bias,
		       sizeof(hi->pdata.adc_mic_bias));
		hi->pdata.adc_mic = hi->pdata.adc_mic_bias[0];
	} else {
		hi->pdata.adc_mic_bias[0] = hi->pdata.adc_mic;
		hi->pdata.adc_mic_bias[1] = HS_DEF_MIC_ADC_12_BIT_MAX;
	}

	if (pdata->adc_remote[5])
		memcpy(hi->pdata.adc_remote, pdata->adc_remote,
		       sizeof(hi->pdata.adc_remote));

	if (pdata->adc_metrico[0] && pdata->adc_metrico[1])
		memcpy(hi->pdata.adc_metrico, pdata->adc_metrico,
		       sizeof(hi->pdata.adc_metrico));

	hi->hpin_irq_type = IRQF_TRIGGER_LOW;
	hi->hpin_debounce = HS_JIFFIES_ZERO;
	hi->key_irq_type = IRQF_TRIGGER_LOW;

	wake_lock_init(&hi->hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	detect_wq = create_workqueue("HS_PMIC_DETECT");
	if (detect_wq  == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create detect workqueue");
		goto err_create_detect_work_queue;
	}

	button_wq = create_workqueue("HS_PMIC_BUTTON");
	if (button_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create button workqueue");
		goto err_create_button_work_queue;
	}

	if (hi->pdata.hpin_gpio) {
		ret = hs_pmic_request_irq(hi->pdata.hpin_gpio,
				&hi->pdata.hpin_irq, detect_irq_handler,
				hi->hpin_irq_type, "HS_PMIC_DETECT", 1);
		if (ret < 0) {
			HS_ERR("Failed to request PMIC HPIN IRQ (0x%X)", ret);
			goto err_request_detect_irq;
		}
	}

	if (hi->pdata.key_gpio) {
		ret = hs_pmic_request_irq(hi->pdata.key_gpio,
				&hi->pdata.key_irq, button_irq_handler,
				hi->key_irq_type, "HS_PMIC_BUTTON", 1);
		if (ret < 0) {
			HS_ERR("Failed to request PMIC button IRQ (0x%X)", ret);
			goto err_request_button_irq;
		}
	}

	hs_pmic_register();
	hs_notify_driver_ready(DRIVER_NAME);

	return 0;

err_request_button_irq:
	if (hi->pdata.hpin_gpio) {
		free_irq(hi->pdata.hpin_irq, 0);
		gpio_free(hi->pdata.hpin_gpio);
	}

err_request_detect_irq:
	destroy_workqueue(button_wq);

err_create_button_work_queue:
	destroy_workqueue(detect_wq);

err_create_detect_work_queue:
	wake_lock_destroy(&hi->hs_wake_lock);
	kfree(hi);

	HS_ERR("Failed to register %s driver", DRIVER_NAME);

	return ret;
}

static int htc_headset_pmic_remove(struct platform_device *pdev)
{
	if (hi->pdata.key_gpio) {
		free_irq(hi->pdata.key_irq, 0);
		gpio_free(hi->pdata.key_gpio);
	}

	if (hi->pdata.hpin_gpio) {
		free_irq(hi->pdata.hpin_irq, 0);
		gpio_free(hi->pdata.hpin_gpio);
	}

	destroy_workqueue(button_wq);
	destroy_workqueue(detect_wq);
	wake_lock_destroy(&hi->hs_wake_lock);

	kfree(hi);

	return 0;
}

static struct platform_driver htc_headset_pmic_driver = {
	.probe	= htc_headset_pmic_probe,
	.remove	= htc_headset_pmic_remove,
	.driver	= {
		.name	= "HTC_HEADSET_PMIC",
		.owner	= THIS_MODULE,
	},
};

static int __init htc_headset_pmic_init(void)
{
	HS_LOG("INIT");
	return platform_driver_register(&htc_headset_pmic_driver);
}

static void __exit htc_headset_pmic_exit(void)
{
	platform_driver_unregister(&htc_headset_pmic_driver);
}

late_initcall(htc_headset_pmic_init);
module_exit(htc_headset_pmic_exit);

MODULE_DESCRIPTION("HTC PMIC headset driver");
MODULE_LICENSE("GPL");
