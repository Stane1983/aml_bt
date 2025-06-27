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
#include "w2_bt_entry.h"
#include "w2_sdio_bt.h"
#include "w2_usb_bt.h"
#include "w2_pcie_bt.h"

extern struct aml_hif_sdio_ops g_hif_sdio_ops;
extern struct aml_pm_type g_wifi_pm;

static int w2_bt_init(void)
{
    int ret = 0;

    BTI("%s \n", __func__);
    if (INTF_TYPE_IS_USB(amlbt_if_type))        //usb interface
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            BTI("%s w2 usb \n", __func__);
            ret = amlbt_w2u_init();
        }
    }
    else if (INTF_TYPE_IS_SDIO(amlbt_if_type))
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            BTI("%s w2 sdio \n", __func__);
            ret = amlbt_w2s_init();
        }
    }
    else if (INTF_TYPE_IS_PCIE(amlbt_if_type))
    {
        BTI("%s w2 pcie \n", __func__);
        ret = amlbt_w2p_init();
    }
    else
    {
        BTE("%s amlbt_if_type invalid!! \n", __func__);
        return ret;
    }

    return ret;
}

static void w2_bt_exit(void)
{
    BTI("%s \n", __func__);
    if (INTF_TYPE_IS_USB(amlbt_if_type))        //usb interface
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            BTI("%s w2 usb \n", __func__);
            amlbt_w2u_exit();
        }
    }
    else if (INTF_TYPE_IS_SDIO(amlbt_if_type))
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            BTI("%s w2 sdio \n", __func__);
            amlbt_w2s_exit();
        }
    }
    else if (INTF_TYPE_IS_PCIE(amlbt_if_type))
    {
        BTI("%s w2 pcie \n", __func__);
        amlbt_w2p_exit();
    }
    else
    {
        BTE("%s amlbt_if_type invalid!! \n", __func__);
    }
}

module_param(amlbt_if_type, uint, S_IRUGO);
module_param(polling_time, uint, S_IRUGO|S_IWUSR);
module_param(amlbt_ft_mode, uint, S_IRUGO);

module_init(w2_bt_init);
module_exit(w2_bt_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(AML_W2U_VERSION);
MODULE_DESCRIPTION(AML_W2S_VERSION);
MODULE_DESCRIPTION(AML_W2P_VERSION);
MODULE_VERSION("1.0.0");



