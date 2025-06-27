/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#ifndef __W1U_SDIO_BT_H__
#define __W1U_SDIO_BT_H__

typedef struct
{
    struct cdev dev_cdev;
    int         dev_major;
    struct class *dev_class;
    struct device *dev_device;
    struct early_suspend early_suspend;
    int irq;
    unsigned int irq_handle;
    struct work_struct wake_work;
    struct input_dev *input_dev;
    struct device_link *link;
    const unsigned char *iccm_buf;
    const unsigned char *dccm_buf;
    const unsigned char *add_buf;
    unsigned int antenna;
    unsigned int fw_mode;
    unsigned int bt_sink;
    unsigned int pin_mux;
    unsigned int br_digit_gain;
    unsigned int edr_digit_gain;
    unsigned int fw_log;
    unsigned int driver_log;
    unsigned int factory;
    unsigned int system;
    unsigned char firmware_start;
    unsigned long shutdown_value;
} w1u_sdio_bt_t;

/*
bin:| ICCM | Additional ICCM | DCCM |
*/

int amlbt_w1us_init(void);
void amlbt_w1us_exit(void);

#endif

