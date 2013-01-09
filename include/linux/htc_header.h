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

#define BIT0                            0x00000001
#define BIT1                            0x00000002
#define BIT8                            0x00000100
#define BIT10                           0x00000400
#define BIT11                           0x00000800

#define DBG_USBCHR_L4 BIT11
#define DBG_USBCHR_L3 BIT10
#define DBG_USBCHR_L1 BIT8

#define DBG_ACM1_RW BIT1
#define DBG_ACM0_RW BIT0

// used by cdc-acm
#define PRINTRTC  do {   \
	struct timespec ts;  \
	struct rtc_time tm;  \
						 \
	getnstimeofday(&ts); \
	rtc_time_to_tm(ts.tv_sec, &tm); \
	printk(" At (%d-%02d-%02d %02d:%02d:%02d.%09lu)\n", \
	tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
	tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define PRINTRTC_PRE  do {   \
    struct timespec ts;  \
    struct rtc_time tm;  \
                         \
    getnstimeofday(&ts); \
    rtc_time_to_tm(ts.tv_sec, &tm); \
    printk("[RIL] At (%d-%02d-%02d %02d:%02d:%02d.%09lu) ", \
    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
    tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

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
