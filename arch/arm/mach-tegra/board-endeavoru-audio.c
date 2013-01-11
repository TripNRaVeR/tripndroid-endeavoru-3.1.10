#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/htc_header.h>
#include <linux/regulator/consumer.h>

#include <mach/board_htc.h>

#include "gpio-names.h"
#include "htc_audio_power.h"

static struct aic3008_power *aic3008_power_ctl;
extern void config_tegra_desk_aud_gpios(bool output, bool out_val);

static void aic3008_powerinit(void)
{
	power_config("AUD_MCLK_EN", TEGRA_GPIO_PX7, INIT_OUTPUT_HIGH);
	power_config("AUD_MCLK_EN", TEGRA_GPIO_PX7, GPIO_OUTPUT);
	power_config("AUD_AIC3008_RST#", TEGRA_GPIO_PW5, INIT_OUTPUT_HIGH);
	power_config("AUD_AIC3008_RST#", TEGRA_GPIO_PW5, GPIO_OUTPUT);
	power_config("v_aud_a1v8", TEGRA_GPIO_PD2, REGULATOR_METHOD);
	power_config("v_aud_3v3", TEGRA_GPIO_PB2, REGULATOR_METHOD);

	power_config("AUD_MCLK", TEGRA_GPIO_PW4, INIT_OUTPUT_LOW);
	sfio_deconfig("AUD_MCLK", TEGRA_GPIO_PW4);
	power_config("AUD_SPK_EN", TEGRA_GPIO_PP6, INIT_OUTPUT_LOW);
	sfio_deconfig("AUD_SPK_EN", TEGRA_GPIO_PP6);
	power_config("AUD_LINEOUT_EN", TEGRA_GPIO_PP7, INIT_OUTPUT_LOW);
	sfio_deconfig("AUD_LINEOUT_EN", TEGRA_GPIO_PP7);
	common_init();

	spin_lock_init(&aic3008_power_ctl->spin_lock);
	aic3008_power_ctl->isPowerOn = true;

	return;
}

static void aic3008_resume(void)
{
	spin_lock(&aic3008_power_ctl->spin_lock);
	power_config("AUD_MCLK_EN", TEGRA_GPIO_PX7, GPIO_OUTPUT);
	common_config();
	aic3008_power_ctl->isPowerOn = true;
	spin_unlock(&aic3008_power_ctl->spin_lock);
	return;
}

static void aic3008_suspend(void)
{
	spin_lock(&aic3008_power_ctl->spin_lock);
	power_deconfig("AUD_MCLK_EN", TEGRA_GPIO_PX7, GPIO_OUTPUT);
	common_deconfig();
	aic3008_power_ctl->isPowerOn = false;
	spin_unlock(&aic3008_power_ctl->spin_lock);
	return;
}

static void aic3008_mic_powerup(void)
{
	/* No Need to Mic PowerUp */
	return;
}

static void aic3008_mic_powerdown(void)
{
	/* No Need to Mic PowerDown */
	return;
}

static void aic3008_amp_powerup(int type)
{
	switch (type) {
	case HEADSET_AMP:
		break;
	case SPEAKER_AMP:
		mdelay(50);
		power_config("AUD_SPK_EN", TEGRA_GPIO_PP6, GPIO_OUTPUT);
		break;
	case DOCK_AMP:
		mdelay(50);
		power_config("AUD_LINEOUT_EN", TEGRA_GPIO_PP7, GPIO_OUTPUT);
		config_tegra_desk_aud_gpios(true, true);
		break;
	}
	return;
}

static void aic3008_amp_powerdown(int type)
{
	switch (type) {
	case HEADSET_AMP:
		break;
	case SPEAKER_AMP:
		power_deconfig("AUD_SPK_EN", TEGRA_GPIO_PP6, GPIO_OUTPUT);
		break;
	case DOCK_AMP:
		power_deconfig("AUD_LINEOUT_EN", TEGRA_GPIO_PP7, GPIO_OUTPUT);
		config_tegra_desk_aud_gpios(false, true);
		break;
	}
	return;
}

static void aic3008_modem_coredump(void)
{
	/* No Need to modem_coredump */
	return;
}

int __init enterprise_audio_codec_init(struct htc_asoc_platform_data *pdata)
{
	aic3008_power_ctl = &pdata->aic3008_power;

	aic3008_power_ctl->mic_switch = false;
	aic3008_power_ctl->amp_switch = true;
	aic3008_power_ctl->powerinit = aic3008_powerinit;
	aic3008_power_ctl->resume = aic3008_resume;
	aic3008_power_ctl->suspend = aic3008_suspend;
	aic3008_power_ctl->mic_powerup = aic3008_mic_powerup;
	aic3008_power_ctl->mic_powerdown = aic3008_mic_powerdown;
	aic3008_power_ctl->amp_powerup = aic3008_amp_powerup;
	aic3008_power_ctl->amp_powerdown = aic3008_amp_powerdown;
	aic3008_power_ctl->modem_coredump = aic3008_modem_coredump;
}
