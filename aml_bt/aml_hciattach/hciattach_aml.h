/*
 *
 *  Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *  Not a Contribution.
 *
 *  Copyright 2012 The Android Open Source Project
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you
 *  may not use this file except in compliance with the License. You may
 *  obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *  implied. See the License for the specific language governing
 *  permissions and limitations under the License.
 *
 */

#ifndef HCIATTCH_AML_H
#define HCIATTCH_AML_H

#ifdef __cplusplus
extern "C" {
#endif

/* HCI Packet types */
#define MSM_ENABLE_FLOW_CTRL   16
#define MSM_DISABLE_FLOW_CTRL  17

/* baud rates */
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
#define DELIM " =\n\t\r"
#define MAX_LINE_LEN 255

/* DEFAULT BT MAC */
#define NUIFYKEY_MAC "/sys/module/kernel/parameters/btmac"

#define W1U_ROM_START_CODE                      0x0cc0006f
#define BT_DEVICE_PATH   "/dev/stpbt"

#ifndef FALSE
#define FALSE  0
#endif

#ifndef TRUE
#define TRUE   (!FALSE)
#endif

/*typedef struct {
    int mod_type;
    unsigned short iccm_base;
    char *mod_type_name;
    char *name;
    char *fw_file;
} vnd_module_t;*/

/* vendor serial control block */
typedef struct
{
    int fd;                     /* fd to Bluetooth device */
    struct termios termios;     /* serial terminal of BT port */
    char port_name[256];
} vnd_userial_cb_t;

typedef int (*action_act)(const char * p_name, char * p_value);
typedef struct {
    const char *entry_name;
    action_act p_action;
} d_entry_t;

int aml_woble_configure(int fd);

#ifdef __cplusplus
}
#endif

#endif /* HCIATTCH_AML_H */
