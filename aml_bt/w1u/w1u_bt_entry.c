/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/usb.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/io.h>
#include <linux/compat.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/amlogic/pm.h>
#include <linux/firmware.h>

#include "common.h"
#include "w1u_bt_entry.h"
#include "w1u_sdio_bt.h"
#include "w1u_usb_bt.h"

extern struct aml_hif_sdio_ops g_hif_sdio_ops;
extern struct aml_pm_type g_wifi_pm;

//extern unsigned char aml_wifi_detect_bt_status __attribute__((weak));

static int w1u_bt_init(void)
{
    int ret = 0;

    BTI("%s \n", __func__);

    if (INTF_TYPE_IS_SDIO(amlbt_if_type))  //sdio interface
    {
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            BTI("%s w1u sdio \n", __func__);
            ret = amlbt_w1us_init();
        }
    }
    else if (INTF_TYPE_IS_USB(amlbt_if_type))
    {
#ifdef CONFIG_USB
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            BTI("%s w1u usb \n", __func__);
            ret = amlbt_w1uu_init();
        }
#endif
    }
    else
    {
        BTE("%s amlbt_if_type invalid!! \n", __func__);
        return ret;
    }

    return ret;
}

static void w1u_bt_exit(void)
{
    BTI("%s \n", __func__);

    if (INTF_TYPE_IS_SDIO(amlbt_if_type))  //sdio interface
    {
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            BTI("%s w1u sdio \n", __func__);
            amlbt_w1us_exit();
        }
    }
    else if (INTF_TYPE_IS_USB(amlbt_if_type))
    {
#ifdef CONFIG_USB
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            BTI("%s w1u usb \n", __func__);
            amlbt_w1uu_init();
        }
#endif
    }
    else
    {
        BTE("%s amlbt_if_type invalid!! \n", __func__);
    }
}

module_param(g_dbg_level, uint, S_IRUGO|S_IWUSR);
module_param(amlbt_if_type, uint, S_IRUGO);
module_param(polling_time, uint, S_IRUGO|S_IWUSR);

module_init(w1u_bt_init);
module_exit(w1u_bt_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(AML_W1US_VERSION);
MODULE_DESCRIPTION(AML_W1UU_VERSION);
MODULE_VERSION("1.0.0");

