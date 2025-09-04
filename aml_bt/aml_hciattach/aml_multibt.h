/******************************************************************************
 *
 *  Copyright (C) 2021-2021 amlogic Corporation
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
#ifndef AML_MULTIBT_H
#define AML_MULTIBT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define LOG_TAG "aml_hciattach"

#define MAILBOX_MOD_NAME
//#define DISTINGUISH_BT_MOD_UART

#ifndef false
#define false   0
#endif

#ifndef true
#define true    (!false)
#endif

bool get_dbg_flag(void);

#ifndef ALOGI

#define ALOGI(fmt, ...) \
    do { \
        fprintf(stderr, "[%s][%s] " fmt "\n", LOG_TAG, __func__, ##__VA_ARGS__); \
        fflush(stderr); \
    } while (0)

#endif

#ifndef ALOGD

#define ALOGD(fmt, ...) \
    do { \
        if (get_dbg_flag()) { \
            fprintf(stderr, "[%s][%s] " fmt "\n", LOG_TAG, __func__, ##__VA_ARGS__); \
            fflush(stderr); \
        } \
    } while (0)

#endif

#ifndef ALOGE

#define ALOGE(fmt, ...) \
    do { \
        fprintf(stderr, "[%s][%s] " fmt "\n", LOG_TAG, __func__, ##__VA_ARGS__); \
        fflush(stderr); \
    } while (0)

#endif

typedef  enum {
    op_distinguish_bt = 0,
    op_set_power_type,
    op_mailbox_mod_name,
    op_amlbt_drv_insmod,
    op_steps_max,
}op_steps;

#define MOD_NAME_MAX_LEN      32
#define MOD_MAX_NUM           2

// Kernel 5.15 btpower_evt path
#define BT_POWER_EVT_1 "/sys/module/amlogic_wireless/parameters/btpower_evt"
// Below kernel 5.15 btpower_evt path
#define BT_POWER_EVT_2 "/sys/module/bt_device/parameters/btpower_evt"

// Kernel 5.15 btwake_evt path
#define BT_WAKE_EVT_1 "/sys/module/amlogic_wireless/parameters/btwake_evt"
// Below kernel 5.15 btwake_evt path
#define BT_WAKE_EVT_2 "/sys/module/bt_device/parameters/btwake_evt"

#ifndef UART_DEV_PORT_BT
#define UART_DEV_PORT_BT      "/dev/ttyS1"
#endif

// Manufacturer vendor info
#define BT_VID_BROADCOM 0x0F00
#define BT_VID_QUALCOMM 0x1D00
#define BT_VID_REALTECK 0x5D00
#define BT_VID_MEDIATEK 0x4600
#define BT_VID_AMLOGIC  0xFFFF
#define BT_VID_UNISOC   0xEC01

#define UPIO_BT_POWER_OFF 0
#define UPIO_BT_POWER_ON  1
#define SDIO_GET_DEV_TYPE       _IO('m',5)
#define CLR_BT_POWER_BIT        _IO('m',6)
#define GET_AML_WIFI_MODULE     _IO('m',7)

#define HCI_MAX_EVENT_SIZE     260

#ifndef N_BRCM_HCI
#define N_BRCM_HCI             25
#endif

// Baud rates
#define USERIAL_BAUD_300        0
#define USERIAL_BAUD_600        1
#define USERIAL_BAUD_1200       2
#define USERIAL_BAUD_2400       3
#define USERIAL_BAUD_9600       4
#define USERIAL_BAUD_19200      5
#define USERIAL_BAUD_57600      6
#define USERIAL_BAUD_115200     7
#define USERIAL_BAUD_230400     8
#define USERIAL_BAUD_460800     9
#define USERIAL_BAUD_921600     10
#define USERIAL_BAUD_1M         11
#define USERIAL_BAUD_1_5M       12
#define USERIAL_BAUD_2M         13
#define USERIAL_BAUD_3M         14
#define USERIAL_BAUD_4M         15
#define USERIAL_BAUD_AUTO       16

/**** Data Format ****/
/* Stop Bits */
#define USERIAL_STOPBITS_1      1
#define USERIAL_STOPBITS_1_5    (1<<1)
#define USERIAL_STOPBITS_2      (1<<2)

/* Parity Bits */
#define USERIAL_PARITY_NONE     (1<<3)
#define USERIAL_PARITY_EVEN     (1<<4)
#define USERIAL_PARITY_ODD      (1<<5)

/* Data Bits */
#define USERIAL_DATABITS_5      (1<<6)
#define USERIAL_DATABITS_6      (1<<7)
#define USERIAL_DATABITS_7      (1<<8)
#define USERIAL_DATABITS_8      (1<<9)


#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
/* These are the ioctl values used for bt_wake ioctl via UART driver. you may
 * need to redefine them on you platform!
 * Logically they need to be unique and not colide with existing uart ioctl's.
 */
#ifndef USERIAL_IOCTL_BT_WAKE_ASSERT
#define USERIAL_IOCTL_BT_WAKE_ASSERT   0x8003
#endif
#ifndef USERIAL_IOCTL_BT_WAKE_DEASSERT
#define USERIAL_IOCTL_BT_WAKE_DEASSERT 0x8004
#endif
#ifndef USERIAL_IOCTL_BT_WAKE_GET_ST
#define USERIAL_IOCTL_BT_WAKE_GET_ST   0x8005
#endif
#endif // (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)

/* Structure used to configure serial port during open */
typedef struct {
    uint16_t fmt;       // Data format
    uint8_t  baud;      // Baud rate
} uart_cfg;

typedef enum {
#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
    USERIAL_OP_ASSERT_BT_WAKE,
    USERIAL_OP_DEASSERT_BT_WAKE,
    USERIAL_OP_GET_BT_WAKE_STATE,
#endif
    USERIAL_OP_NOP,
} userial_vendor_ioctl_op_t;

void set_dbg_flag(int val);
char* get_bt_name(void);
bool distinguish_bt_module(void);
void amlbt_drv_rmmod(void);
int steps_table_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* AML_MULTIBT_H */

