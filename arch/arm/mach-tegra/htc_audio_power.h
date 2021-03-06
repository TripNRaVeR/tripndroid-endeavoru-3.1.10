#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c/tfa9887.h>
#include <linux/i2c/tpa6185.h>
#include <linux/regulator/consumer.h>

#include <asm/mach-types.h>

#include <mach/board_htc.h>
#include <mach/htc_asoc_pdata.h>

#include "gpio-names.h"

void power_config(const char *name, int pin, int method);
void power_deconfig(const char *name, int pin, int method);
void sfio_config(const char *name, int pin);
void sfio_deconfig(const char *name, int pin);
void common_init(void);
void common_config(void);
void common_deconfig(void);

enum GPIO_METHOD {
	REGULATOR_METHOD = 0,
	GPIO_OUTPUT,
	GPIO_INPUT,
	INIT_OUTPUT_LOW,
	INIT_OUTPUT_HIGH,
	INIT_INPUT,
};

enum AMPLIFIER_TYPE {
	HEADSET_AMP = 0,
	SPEAKER_AMP,
	DOCK_AMP,
};

enum HEADSET_GAIN_TYPE {
	BEATS_GAIN_ON = 0,
	BEATS_GAIN_OFF,
};
