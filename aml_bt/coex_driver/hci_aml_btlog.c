/******************************************************************************
*
*  Copyright (C) 2019-2025 Amlogic Corporation
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at:
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*
******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include "hci_aml_btlog.h"
#include <linux/skbuff.h>
#include "hci_aml.h"



extern struct aml_hif_sdio_ops g_hif_sdio_ops;

static void amlbt_sdio_write_word(unsigned int addr, unsigned int data)
{
    if (g_hif_sdio_ops.bt_hi_write_word == NULL)
    {
        BTE("amlbt_sdio_write_word NULL");
        return ;
    }
    BTI("Write [0x%X] = 0x%X (low 2 bits: 0x%X)", addr, data, data & 0x3);
    g_hif_sdio_ops.bt_hi_write_word(addr, data);
}

static unsigned int amlbt_sdio_read_word(unsigned int addr)
{
    unsigned int value = 0;
    if (g_hif_sdio_ops.bt_hi_read_word == NULL)
    {
        BTE("amlbt_sdio_read_word NULL");
        return 0;
    }
    value = g_hif_sdio_ops.bt_hi_read_word(addr);
    BTI("Read [0x%X] = 0x%X (low 2 bits: 0x%X)", addr, value, value & 0x3);
    return value;
}


static ssize_t amlbt_logctrl_read(struct device *dev,
                             struct device_attribute *attr,
                             char *buf) {
    u32 reg_val;
    reg_val = amlbt_sdio_read_word(RG_AON_A59);
    if (reg_val == 0 && g_hif_sdio_ops.bt_hi_read_word == NULL) {
        BTE("Failed to read RG_AON_A59 (SDIO interface error)\n");
        return -EFAULT;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", (reg_val & BTLOG_LOW2_MASK) ? 1 : 0);
}

static ssize_t amlbt_logctrl_write(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf,
                              size_t count) {
    u32 reg_val;
    unsigned long val;

    if (kstrtoul(buf, 0, &val) || val > 2) {
        BTE("Usage: echo 0/1/2/3 > bt_logctrl\n");
        return -EINVAL;
    }

    reg_val = val & BTLOG_LOW2_MASK;
    amlbt_sdio_write_word(RG_AON_A59, reg_val);

    BTI("BT log set to 0x%X\n", (unsigned int)val);
    return count;
}


static struct device_attribute bt_logctrl_attr = {
    .attr = {
        .name = AML_BTLOG_CTRL_NAME,
        .mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH
    },
    .show = amlbt_logctrl_read,
    .store = amlbt_logctrl_write,
};

int aml_btlog_init(struct device *dev) {

    int ret;
    ret = device_create_file(dev, &bt_logctrl_attr);
    if (ret) {
        BTE("Failed to create sysfs attribute\n");
    }

    BTI("BT log control initialized\n");
    return 0;
}

void aml_btlog_deinit(struct device *dev) {
    device_remove_file(dev, &bt_logctrl_attr);
    BTE("BT log control deinitialized\n");
}

