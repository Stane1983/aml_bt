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
#include <linux/version.h>

#include "common.h"
#include "rc_list.h"

#define RG_AON_A24       0xf00060

static ws_inf p_ws_inf = NULL;
static rs_inf p_rs_inf = NULL;
static wrs_sdio_inf p_ws_sdio_inf = NULL;
static wrs_sdio_inf p_rs_sdio_inf = NULL;

static rc_list_t rc_list[MAX_MAC_LIST] = {0};
static list_bt_t list_dev = {0};

static ssize_t rc_list_read(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf);
static ssize_t rc_list_write(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count);

static void amlbt_wakeup_mutex(unsigned int flag)
{
    list_bt_t *p_bt = &list_dev;
    if (flag)
    {
        // Wake up the system and prevent it from entering
        if (p_bt->amlbt_wakeup_source && (!p_bt->amlbt_wakeup_source->active))
        {
            __pm_stay_awake(p_bt->amlbt_wakeup_source);
        }
        else
        {
            BTF("amlbt_wakeup_source is not initialized or active already\n");
        }
    }
    else
    {
        if (p_bt->amlbt_wakeup_source && p_bt->amlbt_wakeup_source->active)
        {
            __pm_relax(p_bt->amlbt_wakeup_source);
        }
        else
        {
            BTF("amlbt_wakeup_source is not initialized or not active\n");
        }
    }
    p_bt->wake_mux = flag;

    BTI("system state updated: %d\n", flag);
}

void amlbt_wakeup_lock(void)
{
    int i = 0;
    list_bt_t *p_bt = &list_dev;

    if (!p_bt->wake_mux)
    {
        for (i = 0; i < MAX_MAC_LIST; i++)
        {
            if (rc_list[i].used)
            {
                amlbt_wakeup_mutex(1);
                break;
            }
        }
    }
}

void amlbt_wakeup_unlock(void)
{
    list_bt_t *p_bt = &list_dev;
    if (p_bt->wake_mux)
    {
        amlbt_wakeup_mutex(0);
    }
}

static struct device_attribute dev_attr_rc_list = {
    .attr = { .name = AML_BT_CHAR_RCLIST_NAME, .mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH },
    .show = rc_list_read,
    .store = rc_list_write,
};

static void mac_to_str(char *buf, const char *mac)
{
    sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x;",
            mac[0] & 0xFF, mac[1] & 0xFF, mac[2] & 0xFF,
            mac[3] & 0xFF, mac[4] & 0xFF, mac[5] & 0xFF);
}

static ssize_t rc_list_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int i, len = 0;
    char mac_str[18];

    for (i = 0; i < MAX_MAC_LIST; i++) {
        if (rc_list[i].used) {
            mac_to_str(mac_str, rc_list[i].mac);
            len += sprintf(buf + len, "%.18s\n", mac_str);
        }
    }

    return len;
}

static int amlbt_add_rc_list(char *mac)
{
    unsigned int i = 0;

    for (i = 0; i < MAX_MAC_LIST; i++)
    {
        if (!rc_list[i].used)
        {
            BTP("amlbt_add_rc_list %d\n", i);
            rc_list[i].used = 1;
            memcpy(rc_list[i].mac, mac, MAC_ADDR_LEN);
            BTP("amlbt_add_rc_list success\n");
            BTP("%#x\n",rc_list[i].mac[0]);
            BTP("%#x\n",rc_list[i].mac[1]);
            BTP("%#x\n",rc_list[i].mac[2]);
            BTP("%#x\n",rc_list[i].mac[3]);
            BTP("%#x\n",rc_list[i].mac[4]);
            BTP("%#x\n",rc_list[i].mac[5]);
            BTI("mac:[%#x,%#x,%#x,%#x,%#x,%#x]\n", rc_list[i].mac[0], rc_list[i].mac[1], rc_list[i].mac[2],
                rc_list[i].mac[3],rc_list[i].mac[4],rc_list[i].mac[5]);
            return 1;
        }
    }
    BTE("%s list full!!\n", __func__);
    return 0;
}

static ssize_t rc_list_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char *mac_str = NULL;
    char *mac_token = NULL;
    char *mac_byte_str = NULL;
    char mac[MAC_ADDR_LEN] = {0};
    unsigned int i = 0, j = 0;

    BTF("%s rc_list write count:%d\n", __func__,count);
    if (count > MAX_USER_BUF_LEN)
    {
        BTE("rc_list too long!:%d, %d\n", count, MAX_USER_BUF_LEN);
        amlbt_wakeup_unlock();
        return -ENOMEM;
    }
    memset(rc_list, 0, sizeof(rc_list));
    mac_str = (char *)buf;
    while ((mac_token = strsep(&mac_str, ";")) != NULL)
    {
        BTI("mac_token length: %d\n", strlen(mac_token));

        if (strlen(mac_token) == 0)
        {
            BTE("NULL str\n");
            continue;
        }

        if (strlen(mac_token) != MAC_ADDR_LEN * 2 + 5)
        {
            BTE("Invalid MAC address length: %s\n", mac_token);
            break;
        }

        for (i = 0, j = 0; i < MAC_ADDR_LEN; i++, j += 3)
        {
            mac_byte_str = &mac_token[j];
            if (mac_token[j + 2] != ':' && j != 15)
            {
                BTE("Invalid MAC address format: %s\n", mac_token);
                return -EINVAL;
            }
            mac[i] = simple_strtoul(mac_byte_str, NULL, 16);
        }
        BTF("parsed mac: [%#x,%#x,%#x,%#x,%#x,%#x]\n",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        amlbt_add_rc_list(mac);
        amlbt_wakeup_unlock();
    }

    return count;
}

/*
static int amlbt_check_mac_exist(char *mac)
{
    unsigned int i = 0;

    for (i = 0; i < MAX_MAC_LIST; i++)
    {
        if (rc_list[i].used && !memcmp(mac, rc_list[i].mac, MAC_ADDR_LEN))
        {
            BTE("%s mac exist!!\n", __func__);
            return 1;
        }
    }

    return 0;
}
*/

/*
static int amlbt_del_rc_list(char *mac)
{
    unsigned int i = 0;

    for (i = 0; i < MAX_MAC_LIST; i++)
    {
        if (rc_list[i].used && !memcmp(mac, rc_list[i].mac, MAC_ADDR_LEN))
        {
            rc_list[i].used = 0;
            memset(rc_list[i].mac, 0, MAC_ADDR_LEN);
            return 1;
        }
    }
    BTE("%s rc addr not found!!\n", __func__);
    return 0;
}
*/

int amlbt_write_rclist_to_firmware(void)
{
    unsigned int i = 0;
    int ret = 0;
    char mac[MAC_ADDR_LEN*MAX_MAC_LIST] = {0};

    BTF("%s\n", __func__);

    for (i = 0; i < MAX_MAC_LIST; i++)
    {
        if (rc_list[i].used)
        {
            memcpy(&mac[i*MAC_ADDR_LEN], rc_list[i].mac, MAC_ADDR_LEN);
            BTP("%#x\n",mac[i*MAC_ADDR_LEN]);
            BTP("%#x\n",mac[i*MAC_ADDR_LEN+1]);
            BTP("%#x\n",mac[i*MAC_ADDR_LEN+2]);
            BTP("%#x\n",mac[i*MAC_ADDR_LEN+3]);
            BTP("%#x\n",mac[i*MAC_ADDR_LEN+4]);
            BTP("%#x\n",mac[i*MAC_ADDR_LEN+5]);
            BTP("mac:[%#x,%#x,%#x,%#x,%#x,%#x]\n", mac[i*MAC_ADDR_LEN], mac[i*MAC_ADDR_LEN+1], mac[i*MAC_ADDR_LEN+2],
                mac[i*MAC_ADDR_LEN+3],mac[i*MAC_ADDR_LEN+4],mac[i*MAC_ADDR_LEN+5]);
        }
    }
    BTP("%s 111\n", __func__);
    if (INTF_TYPE_IS_SDIO(amlbt_if_type))
    {
        p_ws_sdio_inf(mac, (unsigned char *)(unsigned long)FIFO_FW_RC_LIST_ADDR, MAC_ADDR_LEN*MAX_MAC_LIST);
    }
    else if (INTF_TYPE_IS_USB(amlbt_if_type))
    {
        ret = p_ws_inf(mac, (unsigned char *)(unsigned long)FIFO_FW_RC_LIST_ADDR, MAC_ADDR_LEN*MAX_MAC_LIST, USB_EP2);
        if (ret != 0)
        {
            goto err_exit;
        }
    }
    BTP("%s 222\n", __func__);
    memset(mac, 0, sizeof(mac));
    if (INTF_TYPE_IS_SDIO(amlbt_if_type))
    {
        p_rs_sdio_inf(mac, (unsigned char *)(unsigned long)FIFO_FW_RC_LIST_ADDR, MAC_ADDR_LEN*MAX_MAC_LIST);
    }
    else if (INTF_TYPE_IS_USB(amlbt_if_type))
    {
        ret = p_rs_inf(mac, (unsigned char *)(unsigned long)FIFO_FW_RC_LIST_ADDR, MAC_ADDR_LEN*MAX_MAC_LIST, USB_EP2);
        if (ret != 0)
        {
            goto err_exit;
        }
    }
    BTP("%#x\n",mac[0]);
    BTP("%#x\n",mac[1]);
    BTP("%#x\n",mac[2]);
    BTP("%#x\n",mac[3]);
    BTP("%#x\n",mac[4]);
    BTP("%#x\n",mac[5]);
    BTI("rclist:[%#x,%#x,%#x,%#x,%#x,%#x]\n", mac[0], mac[1], mac[2],mac[3],mac[4],mac[5]);
    return ret;
err_exit:
    return ret;
}

void amlbt_clear_rclist_from_firmware(void)
{
    char mac[MAC_ADDR_LEN*MAX_MAC_LIST] = {0};

    BTF("%s\n", __func__);
    if (INTF_TYPE_IS_SDIO(amlbt_if_type))
    {
        p_ws_sdio_inf(mac, (unsigned char *)(unsigned long)FIFO_FW_RC_LIST_ADDR, MAC_ADDR_LEN*MAX_MAC_LIST);
    }
    else if (INTF_TYPE_IS_USB(amlbt_if_type))
    {
        p_ws_inf(mac, (unsigned char *)(unsigned long)FIFO_FW_RC_LIST_ADDR, MAC_ADDR_LEN*MAX_MAC_LIST, USB_EP2);
    }
    BTP("%s 222\n", __func__);
}

static void amlbt_register_wakeupsource(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
    list_dev.amlbt_wakeup_source = wakeup_source_register("amlbt_wakeup_source");
#else
    list_dev.amlbt_wakeup_source = wakeup_source_register(NULL, "amlbt_wakeup_source");
#endif

    if (!list_dev.amlbt_wakeup_source)
    {
        BTE("Failed to create wakeup source\n");
        return ;
    }
}

static void amlbt_unregister_wakeupsource(void)
{
    if (list_dev.amlbt_wakeup_source)
    {
        wakeup_source_unregister(list_dev.amlbt_wakeup_source);
        list_dev.amlbt_wakeup_source = NULL;
    }
    else
    {
        BTE("amlbt_wakeup_source is not initialized, unregistering is not required.\n");
    }
}

int amlbt_rc_list_init(struct device *dev, \
                        ws_inf p_ws_func, \
                        rs_inf p_rs_func, \
                        wrs_sdio_inf p_ws_sdio_func, \
                        wrs_sdio_inf p_rs_sdio_func)
{
    int res = 0;

    BTF("%s\n", __func__);
    res = device_create_file(dev, &dev_attr_rc_list);
    if (res)
    {
        BTE("%s:Failed to create device attribute\n", __func__);
    }
    else
    {
        p_ws_inf = p_ws_func;
        p_rs_inf = p_rs_func;
        p_ws_sdio_inf = p_ws_sdio_func;
        p_rs_sdio_inf = p_rs_sdio_func;
        amlbt_register_wakeupsource();
    }
    return res;
}

void amlbt_rc_list_deinit(struct device *dev)
{
    BTF("%s\n", __func__);
    amlbt_unregister_wakeupsource();
    device_remove_file(dev, &dev_attr_rc_list);
}


