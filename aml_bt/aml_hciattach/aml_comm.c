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

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "aml_comm.h"
#include "aml_multibt.h"
#include "aml_conf.h"

unsigned int aml_mod_idx = W1_UART;

const vnd_module_t aml_module[] ={
    {W1_UART,  256, "W1_UART",  "aml_w1",     AML_W1_BT_FW_UART_FILE },
    {W1U_UART, 256, "W1U_UART", "aml_w1u_s",  AML_W1U_BT_FW_UART_FILE},
    {W1U_USB,  256, "W1U_USB",  "aml_w1u",    AML_W1U_BT_FW_USB_FILE },
    {W2_UART,  256, "W2_UART",  "aml_w2_s",   AML_W2_BT_FW_UART_FILE },
    {W2_UART,  256, "W2_UART",  "aml_w2_p",   AML_W2_BT_FW_UART_FILE },
    {W2_USB,   256, "W2_USB",   "aml_w2_u",   AML_W2_BT_FW_USB_FILE  },
    {W2L_UART, 384, "W2L_UART", "aml_w2l_s",  AML_W2L_BT_FW_UART_FILE},
    {W2L_USB,  384, "W2L_USB",  "aml_w2l_u",  AML_W2L_BT_FW_USB_FILE },
    {0,        0,   NULL,       NULL,         NULL},
};

/*****************************************************************************
**   COMMON FUNCTIONS DEFINE
*****************************************************************************/
void amlbt_get_mod(void)
{
    int idx;
    const char *bt_name = get_bt_name();

    if (!bt_name) {
        ALOGE("bt_name is null");
        return;
    }

    for (idx = 0; aml_module[idx].name; idx++) {
        if (strcmp(bt_name, aml_module[idx].name) == 0) {
            break;
        }
    }

    aml_mod_idx = aml_module[idx].mod_type;

    ALOGI("aml_module: %d", aml_mod_idx);
}

void aml_get_fw_version(unsigned char *str)
{
    int fd, ret;
    char *fw_version = NULL;
    unsigned char *fw_ptr = NULL;

    if (!str) {
        ALOGE("parameters err");
        return;
    }

    fw_ptr = str + 7; //skip 7byte

    ret = asprintf(&fw_version,
                   "fw_version: date:%02x.%02x, number:0x%02x%02x\n",
                   *(fw_ptr + 1), *fw_ptr, *(fw_ptr + 3), *(fw_ptr + 2));
     if (ret == -1 || !fw_version) {
        ALOGE("get bt fw version fail");
        return;
    }

    ALOGI("%s", fw_version);

    fd = open(FW_VER_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0) {
        ALOGE("open:%s fail: %s", FW_VER_FILE, strerror(errno));
        free(fw_version);
        return;
    }

    ret = write(fd, fw_version, strlen(fw_version));
    if (ret == -1) {
        ALOGE("write:%s fail: %s", FW_VER_FILE, strerror(errno));
    }

    free(fw_version);
    close(fd);
}

