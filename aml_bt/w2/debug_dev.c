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
#include <linux/compat.h>
#include <linux/io.h>
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
#include <linux/pm_wakeup.h>
#include <linux/amlogic/pm.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include "w2_bt_entry.h"
#include "common.h"
#include "debug_dev.h"
#include "w2_usb_bt.h"


typedef struct
{
    long long int regvalue;
    long long int regaddr;
    unsigned char ver_inf[100];
    unsigned char manf_inf[48];
    unsigned int manf_len;
    char hci_cmd[8];
    unsigned int hci_len;
    unsigned char hci_evt[8];
    unsigned char dbg_cmd_inf[240];
    unsigned int cmd_index;
    unsigned char dbg_evt_inf[320];
    unsigned int evt_index;
} debug_cmd_t;

typedef struct
{
    struct cdev cdev;
    int         major;
    struct class *class;
    struct device *device;
    debug_cmd_t _cmd;
    struct dentry *debug_dir;
} debug_dev_t;

static debug_dev_t debug_dev = {0};

static ssize_t amlbt_debug_level_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char temp[10];
    int len = snprintf(temp, sizeof(temp), "%d\n", g_dbg_level);
    return simple_read_from_buffer(buf, count, ppos, temp, len);
}

static ssize_t amlbt_debug_level_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char temp[10];
    int ret, val;

    if (count > sizeof(temp) - 1)
        return -EINVAL;

    if (copy_from_user(temp, buf, count))
        return -EFAULT;

    temp[count] = '\0';

    ret = kstrtoint(temp, 10, &val);
    if (ret)
    {
        pr_err("Invalid input for debug_level\n");
        return ret;
    }

    g_dbg_level = val;
    pr_info("Debug level set to %d\n", g_dbg_level);

    return count;
}

static const struct file_operations debug_level_fops =
{
    .read = amlbt_debug_level_read,
    .write = amlbt_debug_level_write,
};

int amlbt_debug_level_init(void)
{
    debug_dev.debug_dir = debugfs_create_dir("aml_btz", NULL);
    if (!debug_dev.debug_dir)
        return -ENOMEM;

    debugfs_create_file("aml_btz_dbg_lvl", 0644, debug_dev.debug_dir, NULL, &debug_level_fops);
    return 0;
}

 void amlbt_debug_level_deinit(void)
{
    if (debug_dev.debug_dir) {
        debugfs_remove_recursive(debug_dev.debug_dir);
        debug_dev.debug_dir = NULL;
    }
}

