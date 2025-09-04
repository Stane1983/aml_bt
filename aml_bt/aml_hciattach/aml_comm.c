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

#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>
#include <sys/types.h>

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
static int mkdir_p(const char *path, mode_t mode)
{
    char tmp[256];
    char *p = NULL;
    size_t len;

    if (!path || !*path) {
        ALOGE("path err");
        return -1;
    }

    snprintf(tmp, sizeof(tmp), "%s", path);
    len = strlen(tmp);

    if (tmp[len - 1] == '/')
        tmp[len - 1] = '\0';

    for (p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(tmp, mode) && errno != EEXIST) {
                ALOGE("mkdir %s err", tmp);
                return -1;
            }
            *p = '/';
        }
    }

    if (mkdir(tmp, mode) && errno != EEXIST) {
        ALOGE("mkdir %s err", tmp);
        return -1;
    }

    return 0;
}

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

    ALOGI("aml_mod_idx:%d", aml_mod_idx);
}

void amlbt_get_fw_ver(const char* fpath, unsigned char *str)
{
    int fd = -1;
    int ret;
    char *fw_version = NULL;
    unsigned char *fw_ptr = NULL;
    char path_copy[PATH_BUF_MAX_LEN];
    char *dir;

    if (!str) {
        ALOGE("parameters err");
        return;
    }

    fw_ptr = str + 7; //skip 7byte

    ret = asprintf(&fw_version,
                   "fw_version: date:%02x.%02x, number:0x%02x%02x",
                   *(fw_ptr + 1), *fw_ptr, *(fw_ptr + 3), *(fw_ptr + 2));
     if (ret == -1 || !fw_version) {
        ALOGE("get bt fw version fail");
        goto exit;
    }

    ALOGI("%s", fw_version);

    snprintf(path_copy, sizeof(path_copy), "%s", fpath);
    dir = dirname(path_copy);
    if (mkdir_p(dir, 0755) < 0) {
        ALOGE("mkdir_p fail for %s", dir);
        goto exit;
    }

    fd = open(fpath, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0) {
        ALOGE("open:%s fail: %s", fpath, strerror(errno));
        goto exit;
    }

    ret = write(fd, fw_version, strlen(fw_version));
    if (ret == -1) {
        ALOGE("write:%s fail: %s", fpath, strerror(errno));
    }

exit:
    if (fw_version) {
        free(fw_version);
    }

    if (fd >= 0) {
        close(fd);
    }
}

int amlbt_read_addr(const char* fpath, unsigned char *addr)
{
    int fd, num;
    char buf[BT_MAC_BUF_LEN + 1] = {0};

    if (!fpath || !addr) {
        ALOGE("Invalid argument");
        return -1;
    }

    fd = open(fpath, O_RDONLY);
    if (fd < 0) {
        ALOGE("open:%s fail: %s", fpath, strerror(errno));
        return -1;
    }

    num = read(fd, buf, BT_MAC_BUF_LEN);
    close(fd);

    if (num < BT_MAC_BUF_LEN) {
        ALOGE("mac read bytes < %u, not match", BT_MAC_BUF_LEN);
        return -1;
    }

    buf[BT_MAC_BUF_LEN] ='\0';

    if (sscanf(buf, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
        &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]) != 6) {
        ALOGE("parse mac addr fail");
        return -1;
    }

    return 0;
}

int amlbt_write_file(const char* fpath, const char *str, int len)
{
    int ret = -1;
    int fd;
    char path_copy[PATH_BUF_MAX_LEN];
    char *dir;

    if (!fpath || !str || len <= 0) {
        ALOGE("invalid input: path=%p str=%p size=%d", fpath, str, len);
        return -1;
    }

    snprintf(path_copy, sizeof(path_copy), "%s", fpath);
    dir = dirname(path_copy);
    if (mkdir_p(dir, 0755) < 0) {
        ALOGE("mkdir_p fail for %s", dir);
        return -1;
    }

    fd = open(fpath, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0) {
        ALOGE("open:%s fail", fpath);
        return -1;
    }

    ret = write(fd, str, len);
    if (ret != len) {
        ALOGE("write fail: expect:%d, get:%d", len, ret);
        ret = -1;
    }

    close(fd);
    return ret;
}

int amlbt_get_random_addr(unsigned char *bdaddr, enum addr_mode mode)
{
    int fd;
    ssize_t cnt, need;
    unsigned char buffer[BT_MAC_LEN] = {0};

    fd = open(AMLBT_RANDOM_DEV, O_RDONLY);
    if (fd < 0) {
        ALOGE("open:%s fail", AMLBT_RANDOM_DEV);
        return -1;
    }

    need = sizeof(buffer);
    cnt = 0;
    while (cnt < need) {
        ssize_t n = read(fd, buffer + cnt, need - cnt);
        if (n <= 0) {
            ALOGE("read :%s fail, cnt:%zd", AMLBT_RANDOM_DEV, cnt);
            close(fd);
            return -1;
        }
        cnt += n;
    }
    close(fd);

    if (mode == ADDR_FIXED_PREFIX) {
        // Prefix 0x22:22, last 4 bytes are random
        bdaddr[0] = 0x22;
        bdaddr[1] = 0x22;
        memcpy(bdaddr + 2, buffer + 2, 4);
    } else {
        // Fully random 6 bytes
        memcpy(bdaddr, buffer, BT_MAC_LEN);

         // Ensure locally administered + unicast
        bdaddr[0] &= 0xFE;  // clear bit0 → unicast
        bdaddr[0] |= 0x02;  // set bit1 → locally administered
    }

    return 0;
}

