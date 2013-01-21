/*
 *
 * Copyright (c) 2012, TripNDroid Mobile Engineering.
 *
 * HTC Header containing various defines/functions used on Endeavoru
 * that where added by HTC and used tree-wide in the kernel. This
 * file needs to be removed and the original stuff needs to be
 * implemented. If possible removing of HTC stuff is the way to go.
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

#ifndef __HTC_HEADER_H
#define __HTC_HEADER_H

/* panel stuff */
#ifdef CONFIG_MACH_ENDEAVORU
#define BL_SHIFT	16
#define BL_MASK		(0x7 << BL_SHIFT)

#define BL_MIPI		(4 << BL_SHIFT)
#define BL_CPU		(6 << BL_SHIFT)

#define IF_SHIFT	19
#define IF_MASK		(0x7 << IF_SHIFT)

#define IF_MIPI		(2 << IF_SHIFT)

#define DEPTH_SHIFT	22
#define DEPTH_MASK	(0x7 << DEPTH_SHIFT)

#define DEPTH_RGB565	(0 << DEPTH_SHIFT)
#define DEPTH_RGB666	(1 << DEPTH_SHIFT)
#define DEPTH_RGB888	(2 << DEPTH_SHIFT)

#define REV_SHIFT	25
#define REV_MASK	(0x7 << REV_SHIFT)

#define REV_0		(0 << REV_SHIFT)
#define REV_1		(1 << REV_SHIFT)
#define REV_2		(2 << REV_SHIFT)

#define BKL_CAB_SHIFT	28
#define BKL_CAB_MASK	(0x3 << BKL_CAB_SHIFT)

#define BKL_CAB_OFF	(0 << BKL_CAB_SHIFT)
#define BKL_CAB_LOW	(1 << BKL_CAB_SHIFT)
#define BKL_CAB_DEF	(2 << BKL_CAB_SHIFT)
#define BKL_CAB_HIGH	(3 << BKL_CAB_SHIFT)

#define PANEL_ID_NONE		(0x0)

#define	PANEL_ID_START		0x0F

#define PANEL_ID(pro, ven, ic) PANEL_ID_##pro##_##ven##_##ic

#define PANEL_SHIFT	8
#define PANEL_MASK(id)  (id & 0xFFFFFF00)
#define PROJ_MASK(id)	(id & 0xFF)

#define PROJ_ENRU       0x01

#define PANEL_ID_SHARP_HX_XA		((0x39 << PANEL_SHIFT) | BL_CPU | IF_MIPI | DEPTH_RGB888 | REV_0)
#define PANEL_ID_SHARP_HX		((0x40 << PANEL_SHIFT) | BL_MIPI | IF_MIPI | DEPTH_RGB888)
#define PANEL_ID_SHARP_NT_C1		((0x41 << PANEL_SHIFT) | BL_MIPI | IF_MIPI | DEPTH_RGB888)
#define PANEL_ID_SONY_NT_C1		((0x42 << PANEL_SHIFT) | BL_MIPI | IF_MIPI | DEPTH_RGB888)
#define PANEL_ID_SHARP_HX_C3		((0x43 << PANEL_SHIFT) | BL_MIPI | IF_MIPI | DEPTH_RGB888)
#define PANEL_ID_SHARP_NT_C2		((0x44 << PANEL_SHIFT) | BL_MIPI | IF_MIPI | DEPTH_RGB888)
#define PANEL_ID_SONY_NT_C2		((0x45 << PANEL_SHIFT) | BL_MIPI | IF_MIPI | DEPTH_RGB888)
#define PANEL_ID_SHARP_HX_C4		((0x46 << PANEL_SHIFT) | BL_MIPI | IF_MIPI | DEPTH_RGB888)
#define PANEL_ID_SHARP_NT_C2_9A 	((0x47 << PANEL_SHIFT) | BL_MIPI | IF_MIPI | DEPTH_RGB888)
#define PANEL_ID_SHARP			((0x48 << PANEL_SHIFT) | BL_MIPI | IF_MIPI | DEPTH_RGB888)
#define PANEL_ID_SONY			((0x49 << PANEL_SHIFT) | BL_MIPI | IF_MIPI | DEPTH_RGB888)

#define	PANEL_ID_END		0xFFFF
#endif

// display debug
#define REGULATOR_GET(reg, name) \
	if (reg == NULL) { \
		reg = regulator_get(NULL, name); \
		if (IS_ERR_OR_NULL(reg)) { \
			printk(KERN_ERR "[DISP][ERR] %s(%d) Could not get regulator %s\r\n", __func__, __LINE__, name); \
			reg = NULL; \
			goto failed; \
		} \
	}

#define DISP_DEBUG_LN(fmt, args...) \
	printk(KERN_DEBUG "[DISP]%s(%d) "fmt, __FUNCTION__, __LINE__, ##args);
#define DISP_INFO_LN(fmt, args...) \
	printk(KERN_INFO "[DISP]%s(%d) "fmt, __FUNCTION__, __LINE__, ##args);
#define DISP_INFO_OUT(fmt, args...) \
	printk(KERN_INFO "[DISP]%s(%d) "fmt" OUT", __FUNCTION__, __LINE__, ##args);

/* audio debug */
#define AUDIO_DEBUG_BEEP 0
#define LOG_TAG_A "AUD"

#define pr_tag_fmt(level, tag, fmt, ...) \
 		printk (level "[" tag "] " KBUILD_MODNAME ":%s():%d: " fmt, \
 				__func__, __LINE__, ##__VA_ARGS__);
#define pr_tag_err(tag, fmt, ...) \
 		pr_tag_fmt(KERN_ERR, tag, fmt, ##__VA_ARGS__);
#define pr_tag_info(tag, fmt, ...) \
 		pr_tag_fmt(KERN_INFO, tag, fmt, ##__VA_ARGS__);
#define pr_device_power_on() \
		pr_pwr_story(" Turn on") 

#define AUD_DBG(fmt, ...) do { } while (0)
#define AUD_ERR(fmt, ...) pr_tag_err(LOG_TAG_A, fmt, ##__VA_ARGS__)
#define AUD_INFO(fmt, ...) pr_tag_info(LOG_TAG_A, fmt, ##__VA_ARGS__)

#define PWR_STORY_TAG "[PWR_STORY]"
#define PWR_DEVICE_TAG "----"
#define pr_pwr_story(fmt, ...) \
		pr_info(PWR_STORY_TAG "[" PWR_DEVICE_TAG "]" pr_fmt(fmt) "\n", ##__VA_ARGS__)

#define pr_device_clk_on() \
		pr_pwr_story("+clk")
#define pr_device_clk_off() \
		pr_pwr_story("-clk")

/* Various defines */
#define I2C_WRITE_RETRY_TIMES   2	/* How many retry's for i2c */

#endif // __HTC_HEADER_H
