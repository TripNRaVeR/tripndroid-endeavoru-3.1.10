/*
 * Copyright (C) 2011 HTC, Inc.
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

#ifndef __HTC_INFO__
#define __HTC_INFO__

#include "../cable_detect.h"

extern ssize_t otg_show_usb_phy_setting(char *buf);
extern ssize_t otg_store_usb_phy_setting(const char *buf, size_t count);

extern int usb_get_connect_type(void);

#ifdef err
#undef err
#endif
#ifdef warn
#undef warn
#endif
#ifdef info
#undef info
#endif

#define USB_ERR(fmt, args...) \
	printk(KERN_ERR "[USB:ERR] " fmt, ## args)
#define USB_WARNING(fmt, args...) \
	printk(KERN_WARNING "[USB] " fmt, ## args)
#define USB_INFO(fmt, args...) \
	printk(KERN_INFO "[USB] " fmt, ## args)
#define USB_DEBUG(fmt, args...) \
	printk(KERN_DEBUG "[USB] " fmt, ## args)

#define USBH_ERR(fmt, args...) \
	printk(KERN_ERR "[USBH:ERR] " fmt, ## args)
#define USBH_WARNING(fmt, args...) \
	printk(KERN_WARNING "[USBH] " fmt, ## args)
#define USBH_INFO(fmt, args...) \
	printk(KERN_INFO "[USBH] " fmt, ## args)
#define USBH_DEBUG(fmt, args...) \
	printk(KERN_DEBUG "[USBH] " fmt, ## args)

#endif /* __HTC_INFO__ */

