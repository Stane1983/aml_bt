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

// Standard C library headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <glob.h>

// POSIX/system headers
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <pthread.h>

// Project-specific headers
#include "aml_multibt.h"

/******************************************************************************
**  Constants & Macros
******************************************************************************/
#ifdef MAILBOX_MOD_NAME
#define MBOX_USER_MAX_LEN   96
#define PATH_MAX_LEN        64
#define ARMV8_TO_AOCPU      "/dev/ree2aocpu"
#define CMD_SET_MID         0xFC
#endif

#define PORT_NAME_MAX_LEN    256
#define BUF_MAX_LEN          256

#define ENUM_DIR_PCI "/sys/bus/pci/devices"
#define ENUM_DIR_SDIO "/sys/bus/sdio/devices"
#define ENUM_DIR_USB "/sys/bus/usb/devices"

#define DBG_CFG_PATH "/etc/bluetooth/main.conf"
#define RFKILL_PATH "/sys/class/rfkill"
#define WIFI_POWER_DEV "/dev/wifi_power"

#define KERNEL_DRV_PATH "/lib/modules/*/kernel/drivers/amlogic/bt"
#define AMLBT_IF_TYPE "amlbt_if_type="

#define POWER_EVENT_DEF '0'
#define POWER_EVENT_RESET '1'
#define POWER_EVENT_EN '2'

/******************************************************************************
**  Local type definitions
******************************************************************************/
/* serial control block */
typedef struct
{
    int fd;                     /* fd to Bluetooth device */
    struct termios termios;     /* serial terminal of BT port */
    char port_name[PORT_NAME_MAX_LEN];
} uart_cb;

typedef struct {
    unsigned int vid;
    unsigned int pid;
} dev_id;

typedef struct {
    dev_id mod_id;
    char dev_name[MOD_NAME_MAX_LEN];
    char power_type;
} dev_info;

typedef struct {
    unsigned int vid;
    char dev_name[MOD_NAME_MAX_LEN];
    char power_type;
} dev_info_uart;

typedef struct {
    char mod_name[MOD_NAME_MAX_LEN];
    char mod_arg[MOD_NAME_MAX_LEN];
} mod_info;

typedef struct {
    char dev_name[MOD_NAME_MAX_LEN];
    mod_info mod[MOD_MAX_NUM];
} drv_info;

typedef void (*op_cb)(void);

typedef struct {
    op_steps step;
    op_cb tag_ops;
} steps_table;

/******************************************************************************
**  Function Declaration
******************************************************************************/
bool distinguish_bt_mod(void);
static void set_power_type(void);
static void mailbox_mod_name_thread(void);
static int amlbt_drv_insmod(void);
static int amlbt_drv_modprobe(void);
static void clr_bt_power_bit(char power_type);

/******************************************************************************
**  Static variables
******************************************************************************/
static const uart_cfg uart_cfg_h5 =
{
    (USERIAL_DATABITS_8 | USERIAL_PARITY_EVEN | USERIAL_STOPBITS_1),
    USERIAL_BAUD_115200,
};

static const uart_cfg uart_cfg_h4 =
{
    (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1),
    USERIAL_BAUD_115200,
};

static const steps_table steps_table_exp[] = {
    {op_distinguish_bt,         (void *)distinguish_bt_mod},
    {op_set_power_type,         set_power_type},
#ifdef MAILBOX_MOD_NAME
    {op_mailbox_mod_name,       mailbox_mod_name_thread},
#endif
    {op_amlbt_drv_insmod,       (void *)amlbt_drv_modprobe},
};

static uart_cb bt_uart_cb;
static char *rfkill_state_path = NULL;
static bool pci_flag = false;
static bool dbg_flag = false;
static char amlbt_name[MOD_NAME_MAX_LEN + 1] = {0};

/******************************************************************************
**  init variables
******************************************************************************/
static uint8_t vendor_info[] =     {0x01, 0x01, 0x10, 0x00};
static uint8_t vendor_reset[] =    {0x01, 0x03, 0x0c, 0x00};
static uint8_t vendor_sync[] =     {0xc0, 0x00, 0x2f, 0x00, 0xd0, 0x01, 0x7e, 0xc0}; //{0x01, 0x7E}
//static uint8_t vendor_sync_rsp[] = {0xc0,0x00,0x2f,0x00,0xd0,0x02,0x7d,0xc0}; //{0x02, 0x7D}

/******************************************************************************
**  Bt device info of pci interface
******************************************************************************/
static const dev_info bt_dev_pci[] = {
    // amlogic pice modules
    {{0x1F35, 0x0602}, "aml_w2_p",                  POWER_EVENT_RESET},
    {{0x1F35, 0x0642}, "aml_w2_p",                  POWER_EVENT_RESET},
};

/******************************************************************************
**  Bt device info of sdio interface
******************************************************************************/
static const dev_info bt_dev_sdio[] = {
    // amlogic sdio modules
    {{0x8888, 0x8888}, "aml_w1",                    POWER_EVENT_EN},
    {{0x1B8E, 0x0500}, "aml_w1u_s",                 POWER_EVENT_DEF},
    {{0x1B8E, 0x0540}, "aml_w1u_s",                 POWER_EVENT_DEF},
    {{0x1B8E, 0x0600}, "aml_w2_s",                  POWER_EVENT_DEF},
    {{0x1B8E, 0x0640}, "aml_w2_s",                  POWER_EVENT_DEF},
    {{0x1B8E, 0x0800}, "aml_w2l_s",                 POWER_EVENT_DEF},
    {{0x1B8E, 0x0810}, "aml_w2l_s",                 POWER_EVENT_DEF},
    {{0x1B8E, 0x0808}, "aml_w2l_s",                 POWER_EVENT_DEF},
    {{0x1B8E, 0x0840}, "aml_w2l_s",                 POWER_EVENT_DEF},
    {{0x1B8E, 0x0848}, "aml_w2l_s",                 POWER_EVENT_DEF},
    {{0x1B8E, 0x0850}, "aml_w2l_s",                 POWER_EVENT_DEF},
};

/******************************************************************************
**  Bt device info of usb interface
******************************************************************************/
static const dev_info bt_dev_usb[] = {
    // amlogic usb modules
    {{0x1B8E, 0x4C55}, "aml_w1u",                   POWER_EVENT_EN},
    {{0x1B8E, 0x0541}, "aml_w1u",                   POWER_EVENT_EN},
    {{0x1B8E, 0x0601}, "aml_w2_u",                  POWER_EVENT_EN},
    {{0x1B8E, 0x0641}, "aml_w2_u",                  POWER_EVENT_EN},
    {{0x1B8E, 0x0801}, "aml_w2l_u",                 POWER_EVENT_EN},
    {{0x1B8E, 0x0809}, "aml_w2l_u",                 POWER_EVENT_EN},
    {{0x1B8E, 0x0811}, "aml_w2l_u",                 POWER_EVENT_EN},
    {{0x1B8E, 0x0841}, "aml_w2l_u",                 POWER_EVENT_EN},
    {{0x1B8E, 0x0849}, "aml_w2l_u",                 POWER_EVENT_EN},
    {{0x1B8E, 0x0851}, "aml_w2l_u",                 POWER_EVENT_EN},
};

/******************************************************************************
**  Bt device info of uart interface
******************************************************************************/
static const dev_info_uart bt_dev_uart[] = {
    {BT_VID_BROADCOM, "bcm_bt", POWER_EVENT_RESET},
    {BT_VID_QUALCOMM, "qca_bt", POWER_EVENT_RESET},
    {BT_VID_REALTECK, "rtl_bt", POWER_EVENT_RESET},
    {BT_VID_MEDIATEK, "mtk_bt", POWER_EVENT_RESET},
    {BT_VID_AMLOGIC,  "aml_bt", POWER_EVENT_RESET},
    {BT_VID_UNISOC,   "uwe_bt", POWER_EVENT_RESET},
};

static const drv_info aml_drv[] = {
    {"aml_w1",        {{"sdio_bt",    ""},                  {"",   ""}}},
    {"aml_w1u",       {{"w1u_bt",     AMLBT_IF_TYPE "1280"},{"",   ""}}},
    {"aml_w1u_s",     {{"w1u_bt",     AMLBT_IF_TYPE "1280"},{"",   ""}}},
    {"aml_w2_s",      {{"w2_bt",      AMLBT_IF_TYPE "1536"},{"",   ""}}},
    {"aml_w2_p",      {{"w2_bt",      AMLBT_IF_TYPE "1538"},{"",   ""}}},
    {"aml_w2_u",      {{"w2_bt",      AMLBT_IF_TYPE "1537"},{"",   ""}}},
    {"aml_w2l_s",     {{"w2l_bt",     AMLBT_IF_TYPE "2048"},{"",   ""}}},
    {"aml_w2l_u",     {{"w2l_bt",     AMLBT_IF_TYPE "2049"},{"",   ""}}},
};

void set_dbg_flag(int val)
{
    if (val == 1) {
        dbg_flag = true;
    } else {
        dbg_flag = false;
        ALOGI("dbg_flag: %s", dbg_flag ? "true" : "false");
    }
}

bool get_dbg_flag(void)
{
    return dbg_flag;
}

#ifdef MAILBOX_MOD_NAME
static void* mailbox_mod_name(void *arg)
{
    (void)arg;

    struct merge_data {
        int cmd;
        char msg[MBOX_USER_MAX_LEN];
    } merge_data;
    int fd = -1;
    int ret = -1;
    unsigned int len = 0;
    char path[PATH_MAX_LEN] = {'\0'};
    char bt_name[MOD_NAME_MAX_LEN] = {'\0'};

    sprintf(path, "%s", ARMV8_TO_AOCPU);
    ALOGI("open %s", path);

    fd = open(path, O_RDWR);
    if (fd < 0) {
        ALOGE("open %s failed: %s (%d)", ARMV8_TO_AOCPU, strerror(errno), errno);
        goto exit;
    }

    merge_data.cmd = CMD_SET_MID;
    len += sizeof(merge_data.cmd);
    strcpy(merge_data.msg, amlbt_name);
    len += strlen(amlbt_name);
    ret = write(fd, &merge_data, len);
    if (ret < 0) {
        ALOGE("write failed: %s (%d)", strerror(errno), errno);
        goto exit;
    }

    ret = read(fd, bt_name, len);
    if (ret < 0) {
        ALOGE("read failed: %s (%d)", strerror(errno), errno);
        goto exit;
    }

    ALOGI("write len:%u,buf:%s", len, bt_name);

exit:
    if (fd >= 0) {
        close(fd);
    }

    return NULL;
}

static void mailbox_mod_name_thread(void)
{
    pthread_t thread_id;
    int ret;

    ret = pthread_create(&thread_id, NULL, mailbox_mod_name, NULL);
    if (ret != 0) {
        ALOGE("pthread_create fail: %s", strerror(ret));
    } else {
        ret = pthread_detach(thread_id);
        if (ret != 0) {
            ALOGE("pthread_detach fail: %s", strerror(ret));
        }
    }
}
#endif

static bool get_pci_flag(void)
{
    return pci_flag;
}

static void set_pci_flag(bool val)
{
    pci_flag = val;
}


static void set_bt_name(const char *dev_name)
{
    if ((dev_name == NULL)) {
        ALOGE("parameters err");
        return;
    }

    strcpy(amlbt_name, dev_name);
    ALOGD("amlbt_name:%s", amlbt_name);
}

char* get_bt_name(void)
{
    ALOGD("amlbt_name:%s", amlbt_name);
    return amlbt_name;
}

static bool matching_dev_id(const dev_info *dev, unsigned int dev_size, const dev_id *mod_id)
{
    unsigned int cnt;
    bool ret =false;

    if ((dev == NULL) || (mod_id == NULL)) {
        ALOGE("parameters err");
        return ret;
    }

    for (cnt = 0; cnt < dev_size; cnt++) {
        if ((dev[cnt].mod_id.vid == mod_id->vid) && (dev[cnt].mod_id.pid == mod_id->pid)) {
            ALOGI("matched vid:%04x, pid:%04x, dev_name:%s, cnt:%u, set property",
                dev[cnt].mod_id.vid, dev[cnt].mod_id.pid, dev[cnt].dev_name, cnt);
            set_bt_name(dev[cnt].dev_name);
            ret = true;
            break;
        }
    }

    return ret;
}

static bool matching_dev_id_uart(unsigned int vid)
{
    unsigned int cnt;
    bool ret = false;

    for (cnt = 0; cnt < (sizeof(bt_dev_uart) / sizeof(dev_info_uart)); cnt++) {
        if (bt_dev_uart[cnt].vid == vid) {
            ALOGI("matched vid:%04x, dev_name:%s, cnt:%u, set property",bt_dev_uart[cnt].vid,
                bt_dev_uart[cnt].dev_name, cnt);
            strcpy(amlbt_name, bt_dev_uart[cnt].dev_name);
            ret = true;
            break;
        }
    }

    return ret;
}

static int matching_dev_name(const dev_info *dev, unsigned int dev_size, const char *val)
{
    int ret = -1;
    int cnt;

    if ((dev == NULL) || (val == NULL)) {
        ALOGE("parameters err");
        return ret;
    }

    for (cnt = 0; cnt < dev_size; cnt++) {
        if (!strcmp(dev[cnt].dev_name, val)) {
            ALOGI("matched dev_name:%s, cnt:%d", dev[cnt].dev_name, cnt);
            ret = cnt;
            break;
        }
    }

    return ret;
}

static bool amlbt_get_name(void)
{
    int fd;
    bool ret = false;
    char str[MOD_NAME_MAX_LEN + 1] = {'\0'};

    fd = open(WIFI_POWER_DEV, O_RDWR);
    if (fd < 0) {
        ALOGE("open (%s) failed: %s (%d)", WIFI_POWER_DEV, strerror(errno), errno);
        goto exit;
    }

    if (ioctl (fd, GET_AML_WIFI_MODULE, str) < 0) {
        ALOGE("ioctl GET_AML_WIFI_MODULE failed: %s (%d)", strerror(errno), errno);
        goto exit;
    }

    ALOGD("get bt module: %s", str);

    if (!strncmp(str, "aml" ,3)) {
        strncpy(amlbt_name, str, MOD_NAME_MAX_LEN);
        ALOGI("amlbt_name: %s", amlbt_name);
        ret = true;
    }

exit:
    if (fd >= 0) {
        close(fd);
    }

    return ret;
}

static bool write_power_type(const char *power_type, unsigned int len)
{
    int fd;
    bool ret = false;

    if (access(BT_POWER_EVT_1, F_OK) == 0) {
        fd = open(BT_POWER_EVT_1, O_WRONLY);
    } else {
        fd = open(BT_POWER_EVT_2, O_WRONLY);
    }

    if (fd < 0) {
        ALOGE("open btpower_evt failed: %s (%d)", strerror(errno), errno);
        goto exit;
    } else {
        if (write(fd, power_type, len) < len) {
            ALOGE("write btpower_evt failed: %s (%d)", strerror(errno), errno);
        } else {
            ret = true;
        }
    }

exit:
    if (fd >= 0) {
        close(fd);
    }

    return ret;
}

static bool get_power_type(char *power_type)
{
    int idx = -1;

    if (power_type == NULL) {
        ALOGE("parameters err");
        return false;
    }

    if (strlen(amlbt_name)) {
        idx = matching_dev_name(bt_dev_pci, (sizeof(bt_dev_pci) / sizeof(dev_info)),
            amlbt_name);
        if (idx >= 0) {
            *power_type = bt_dev_pci[idx].power_type;
            set_pci_flag(true);
            goto exit;
        }

        idx = matching_dev_name(bt_dev_sdio, (sizeof(bt_dev_sdio) / sizeof(dev_info)),
            amlbt_name);
        if (idx >= 0) {
            *power_type = bt_dev_sdio[idx].power_type;
            goto exit;
        }

        idx = matching_dev_name(bt_dev_usb, (sizeof(bt_dev_usb) / sizeof(dev_info)),
            amlbt_name);
        if (idx >= 0) {
            *power_type = bt_dev_usb[idx].power_type;
            goto exit;
        }

        for (idx = 0; idx < ( sizeof(bt_dev_uart) / sizeof(dev_info_uart)); idx++) {
            if (!strcmp(bt_dev_uart[idx].dev_name, amlbt_name)) {
                *power_type = bt_dev_uart[idx].power_type;
                goto exit;
            }
        }
    }

    return false;

exit:
    ALOGD("matched amlbt_name:%s, idx:%d, power_type:%c", amlbt_name, idx, *power_type);
    return true;
}

static void set_power_type(void)
{
    char power_type = '\0';

    if (!get_power_type(&power_type)) {
        ALOGI("get power type failed");
        return;
    }

    if (!write_power_type(&power_type, sizeof(char))) {
        ALOGI("write power type failed");
        return;
    }

    clr_bt_power_bit(power_type);
}

static bool find_target_file(const char *path, const char * file)
{
    DIR *dir;
    struct dirent *next;
    bool ret = false;

    if ((path == NULL) || (file == NULL)) {
         ALOGE("parameters err");
         return ret;
    }

    dir = opendir(path);
    if (dir == NULL) {
       ALOGE("opendir (%s) failed: %s (%d)", path, strerror(errno), errno);
       goto exit;
    }

    while ((next = readdir(dir)) != NULL) {
        if (!strncmp(next->d_name, file, strlen(file))) {
            ret = true;
            goto exit;
        }
    }

exit:
    if (dir) {
        closedir(dir);
    }

    return ret;
}

static void clr_bt_power_bit(char power_type)
{
    int fd;

    if (get_pci_flag()) {
        ALOGI("pcie needn't clr bt power bit");
        return;
    }

    if (!strcmp(amlbt_name, "aml_w2_s")) {
        ALOGI("aml w2_s needn't clr bt power bit");
        return;
    }

    if (power_type == POWER_EVENT_RESET) {
        ALOGI("power_type:(%c) on separately", power_type);
    } else {
        ALOGI("power_type:(%c) not on separately",  power_type);
        return;
    }

    fd = open(WIFI_POWER_DEV, O_RDWR);
    if (fd < 0) {
        ALOGE("open (%s) failed: %s (%d)", WIFI_POWER_DEV, strerror(errno), errno);
        return;
    }

    if (ioctl(fd, CLR_BT_POWER_BIT) < 0) {
        ALOGE("ioctl CLR_BT_POWER_BIT failed: %s (%d)", strerror(errno), errno);
    }

    if (fd >= 0) {
        close(fd);
    }

    return;
}

static bool distinguish_bt_mod_pci(void)
{
    DIR *dir;
    struct dirent *next;
    FILE *fp = NULL;
    bool ret = false;

    dir = opendir(ENUM_DIR_PCI);
    if (!dir) {
        ALOGE("opendir (%s) failed: %s (%d)", ENUM_DIR_PCI, strerror(errno), errno);
        goto exit;
    }

    while ((next = readdir(dir)) != NULL) {
        char line[BUF_MAX_LEN] = {'\0'};
        char uevent_file[BUF_MAX_LEN*2] = {'\0'};

        /* Read pci uevent file, uevent's data like selow:
         * DRIVER=w2_comm
         * PCI_CLASS=78000
         * PCI_ID=1F35:0602
         * PCI_SUBSYS_ID=1556:1111
         * PCI_SLOT_NAME=0000:01:00.0
         * MODALIAS=pci:v00001F35d00000602sv00001556sd00001111bc07sc80i00
         */
        snprintf(uevent_file, sizeof(uevent_file), "%s/%s/uevent", ENUM_DIR_PCI, next->d_name);
        fp = fopen(uevent_file, "r");
        if (fp == NULL) {
            continue;
        }

        while (fgets(line, sizeof(line), fp)) {
            dev_id mod_id = {0, 0};
            char *pos = NULL;

            pos = strstr(line, "PCI_ID=");
            if (!pos) {
                continue;
            }

            if (sscanf(pos + 7, "%x:%x", &(mod_id.vid), &(mod_id.pid)) <= 0) {
                continue;
            }

            ALOGD("list vid:0x%04x, pid:0x%04x", mod_id.vid, mod_id.pid);
            ret = matching_dev_id(bt_dev_pci, (sizeof(bt_dev_pci) / sizeof(dev_info)), &mod_id);
            if (ret) {
                goto exit;
            }
        }

        if (fp) {
            fclose(fp);
            fp = NULL;
        }
    }

exit:
    if (fp) {
        fclose(fp);
        fp = NULL;
    }

    if (dir) {
        closedir(dir);
    }

    return ret;
}

static bool distinguish_bt_mod_sdio(void)
{
    DIR *dir;
    struct dirent *next;
    FILE *fp = NULL;
    bool ret = false;

    dir = opendir(ENUM_DIR_SDIO);
    if (!dir) {
        ALOGE("opendir (%s) failed: %s (%d)", ENUM_DIR_SDIO, strerror(errno), errno);
        goto exit;
    }

    while ((next = readdir(dir)) != NULL) {
        char line[BUF_MAX_LEN] = {'\0'};
        char uevent_file[BUF_MAX_LEN*2] = {'\0'};

        /* Read sdio uevent file, uevent's data like below:
         * DRIVER=aml_sdio
         * OF_NAME=wifi
         * OF_FULLNAME=/soc/sdio@fe088000/wifi@1
         * OF_COMPATIBLE_0=brcm,bcm4329-fmac
         * OF_COMPATIBLE_N=1
         * SDIO_CLASS=07
         * SDIO_ID=1B8E:0540
         * MODALIAS=sdio:c07v1B8Ed0540
         */
        snprintf(uevent_file, sizeof(uevent_file), "%s/%s/uevent", ENUM_DIR_SDIO, next->d_name);
        fp = fopen(uevent_file, "r");
        if (fp == NULL) {
            continue;
        }

        while (fgets(line, sizeof(line), fp)) {
            dev_id mod_id = {0, 0};
            char *pos = NULL;

            pos = strstr(line, "SDIO_ID=");
            if (!pos) {
                continue;
            }

            if (sscanf(pos + 8, "%x:%x", &(mod_id.vid), &(mod_id.pid)) <= 0) {
                continue;
            }

            ALOGD("list vid:0x%04x, pid:0x%04x", mod_id.vid, mod_id.pid);
            ret = matching_dev_id(bt_dev_sdio, (sizeof(bt_dev_sdio) / sizeof(dev_info)), &mod_id);
            if (ret) {
                goto exit;
            }
        }

        if (fp) {
            fclose(fp);
            fp = NULL;
        }
    }

exit:
    if (fp) {
        fclose(fp);
        fp = NULL;
    }

    if (dir) {
        closedir(dir);
    }

    return ret;
}

static bool distinguish_bt_mod_usb(void)
{
    DIR *dir;
    struct dirent *next;
    FILE *fp = NULL;
    bool ret = false;

    dir = opendir(ENUM_DIR_USB);
    if (!dir) {
        ALOGE("opendir (%s) failed: %s (%d)", ENUM_DIR_USB, strerror(errno), errno);
        goto exit;
    }

    while ((next = readdir(dir)) != NULL) {
        char line[BUF_MAX_LEN] = {'\0'};
        char uevent_file[BUF_MAX_LEN*2] = {'\0'};

        /* Read usb uevent file, uevent's data like below:
         * MAJOR=189
         * MINOR=1
         * DEVNAME=bus/usb/001/002
         * DEVTYPE=usb_device
         * DRIVER=usb
         * PRODUCT=1b8e/601/101
         * TYPE=0/0/0
         * BUSNUM=001
         * DEVNUM=002
         */
        snprintf(uevent_file, sizeof(uevent_file), "%s/%s/uevent", ENUM_DIR_USB, next->d_name);
        fp = fopen(uevent_file, "r");
        if (fp == NULL) {
            continue;
        }

        while (fgets(line, sizeof(line), fp)) {
            unsigned int bcdev;
            dev_id mod_id = {0, 0};
            char *pos = NULL;

            pos = strstr(line, "PRODUCT=");
            if (!pos) {
                continue;
            }

            if (sscanf(pos + 8, "%x/%x/%x", &(mod_id.vid), &(mod_id.pid), &bcdev) <= 0) {
                continue;
            }

            ALOGD("list vid:0x%04x, pid:0x%04x", mod_id.vid, mod_id.pid);
            ret = matching_dev_id(bt_dev_usb, (sizeof(bt_dev_usb) / sizeof(dev_info)), &mod_id);
            if (ret) {
                goto exit;
            }
        }

        if (fp) {
            fclose(fp);
            fp = NULL;
        }
    }

exit:
    if (fp) {
        fclose(fp);
        fp = NULL;
    }

    if (dir) {
        closedir(dir);
    }

    return ret;
}

/*******************************************************************************
**
** Function        init_rfkill_aml_bt
**
** Description     Traverse rfkill file path to obtain rfkill node named bt-dev
**
** Returns         true  : success
**                 false : fail
**
*******************************************************************************/
static bool init_rfkill_aml_bt(void)
{
    glob_t globbuf;
    FILE *fp = NULL;
    bool ret = false;
    int i;

    char pattern[BUF_MAX_LEN];
    snprintf(pattern, sizeof(pattern), "%s/rfkill*/name", RFKILL_PATH);

    if (glob(pattern, 0, NULL, &globbuf) != 0) {
        ALOGE("glob failed: %s", strerror(errno));
        return false;
    }

    for (i = 0; i < globbuf.gl_pathc; ++i) {
        char line[BUF_MAX_LEN] = {0};

        fp = fopen(globbuf.gl_pathv[i], "r");
        if (!fp) {
            continue;
        }

        if (fgets(line, sizeof(line), fp)) {
            if (strncmp(line, "bt-dev", 6) == 0) {
                // Get the parent path: .../rfkill*/name -> .../rfkill*/state
                char *slash = strrchr(globbuf.gl_pathv[i], '/');
                if (slash) {
                    *slash = '\0';  // Trim "/name"
                    (void)asprintf(&rfkill_state_path, "%s/state", globbuf.gl_pathv[i]);
                    ALOGI("RFKILL_STATE_PATH: %s", rfkill_state_path);
                    ret = true;
                    fclose(fp);
                    break;
                }
            }
        }

        fclose(fp);
    }

    globfree(&globbuf);
    return ret;
}

/*******************************************************************************
**
** Function        upio_set_bluetooth_power
**
** Description     Interact with low layer driver to set Bluetooth power
**                 on/off.
**
** Returns         0  : SUCCESS or Not-Applicable
**                 <0 : ERROR
**
*******************************************************************************/
static int upio_set_bluetooth_power(int on)
{
    int fd;
    int len = -1;
    int ret = -1;

    if (!init_rfkill_aml_bt()) {
        ALOGE("init rfkill failed");
        return ret;
    }

    fd = open(rfkill_state_path, O_WRONLY);
    if (fd < 0) {
        ALOGE("open (%s) failed: %s (%d)", rfkill_state_path, strerror(errno), errno);
        return ret;
    }

    if (on == UPIO_BT_POWER_OFF) {
        len = write(fd, "0", 1);
    } else if (on == UPIO_BT_POWER_ON) {
        len = write(fd, "1", 1);
    }

    if (len < 1) {
        ALOGE("write (%s) failed: %s (%d)", rfkill_state_path, strerror(errno), errno);
    } else {
        ret = 0;
    }

    close(fd);

    return ret;
}

/*******************************************************************************
**
** Function        userial_to_tcio_baud
**
** Description     helper function converts USERIAL baud rates into TCIO
**                 conforming baud rates
**
** Returns         TRUE/FALSE
**
*******************************************************************************/
static bool userial_to_tcio_baud(uint8_t cfg_baud, uint32_t *baud)
{
    if (cfg_baud == USERIAL_BAUD_115200)
        *baud = B115200;
    else if (cfg_baud == USERIAL_BAUD_4M)
        *baud = B4000000;
    else if (cfg_baud == USERIAL_BAUD_3M)
        *baud = B3000000;
    else if (cfg_baud == USERIAL_BAUD_2M)
        *baud = B2000000;
    else if (cfg_baud == USERIAL_BAUD_1M)
        *baud = B1000000;
    else if (cfg_baud == USERIAL_BAUD_921600)
        *baud = B921600;
    else if (cfg_baud == USERIAL_BAUD_460800)
        *baud = B460800;
    else if (cfg_baud == USERIAL_BAUD_230400)
        *baud = B230400;
    else if (cfg_baud == USERIAL_BAUD_57600)
        *baud = B57600;
    else if (cfg_baud == USERIAL_BAUD_19200)
        *baud = B19200;
    else if (cfg_baud == USERIAL_BAUD_9600)
        *baud = B9600;
    else if (cfg_baud == USERIAL_BAUD_1200)
        *baud = B1200;
    else if (cfg_baud == USERIAL_BAUD_600)
        *baud = B600;
    else
    {
        ALOGE( "userial vendor open: unsupported baud idx %i", cfg_baud);
        *baud = B115200;
        return false;
    }

    return true;
}

#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
/*******************************************************************************
**
** Function        userial_ioctl_init_bt_wake
**
** Description     helper function to set the open state of the bt_wake if ioctl
**                 is used. it should not hurt in the rfkill case but it might
**                 be better to compile it out.
**
** Returns         none
**
*******************************************************************************/
static void userial_ioctl_init_bt_wake(int fd)
{
    uint32_t bt_wake_state;

#if (BT_WAKE_USERIAL_LDISC == TRUE)
    int ldisc = N_BRCM_HCI; /* brcm sleep mode support line discipline */

    /* attempt to load enable discipline driver */
    if (ioctl(bt_uart_cb.fd, TIOCSETD, &ldisc) < 0)
    {
        ALOGI("USERIAL_Open():fd %d, TIOCSETD failed: error %d for ldisc: %d",
            fd, errno, ldisc);
    }
#endif

    /* assert BT_WAKE through ioctl */
    ioctl(fd, USERIAL_IOCTL_BT_WAKE_ASSERT, NULL);
    ioctl(fd, USERIAL_IOCTL_BT_WAKE_GET_ST, &bt_wake_state);
    ALOGI("userial_ioctl_init_bt_wake read back BT_WAKE state=%i", \
        bt_wake_state);
}
#endif // (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)

/*****************************************************************************
**   Userial Vendor API Functions
*****************************************************************************/

/*******************************************************************************
**
** Function        userial_vendor_init
**
** Description     Initialize userial vendor-specific control block
**
** Returns         None
**
*******************************************************************************/
static void userial_vendor_init(void)
{
    bt_uart_cb.fd = -1;
    snprintf(bt_uart_cb.port_name, PORT_NAME_MAX_LEN, "%s", UART_DEV_PORT_BT);
}

/*******************************************************************************
**
** Function        userial_vendor_open
**
** Description     Open the serial port with the given configuration
**
** Returns         device fd
**
*******************************************************************************/
static int userial_vendor_open(uart_cfg *p_cfg)
{
    uint32_t baud;
    uint8_t data_bits;
    uint16_t parity;
    uint8_t stop_bits;

    bt_uart_cb.fd = -1;

    if (!userial_to_tcio_baud(p_cfg->baud, &baud))
    {
        return -1;
    }

    if (p_cfg->fmt & USERIAL_DATABITS_8)
        data_bits = CS8;
    else if (p_cfg->fmt & USERIAL_DATABITS_7)
        data_bits = CS7;
    else if (p_cfg->fmt & USERIAL_DATABITS_6)
        data_bits = CS6;
    else if (p_cfg->fmt & USERIAL_DATABITS_5)
        data_bits = CS5;
    else
    {
        ALOGE("userial vendor open: unsupported data bits");
        return -1;
    }

    ALOGI("data_bits:%u", data_bits);

    if (p_cfg->fmt & USERIAL_PARITY_NONE)
        parity = 0;
    else if (p_cfg->fmt & USERIAL_PARITY_EVEN)
        parity = PARENB;
    else if (p_cfg->fmt & USERIAL_PARITY_ODD)
        parity = (PARENB | PARODD);
    else
    {
        ALOGE("userial vendor open: unsupported parity bit mode");
        return -1;
    }

    if (p_cfg->fmt & USERIAL_STOPBITS_1)
        stop_bits = 0;
    else if (p_cfg->fmt & USERIAL_STOPBITS_2)
        stop_bits = CSTOPB;
    else
    {
        ALOGE("userial vendor open: unsupported stop bits");
        return -1;
    }

    ALOGI("userial vendor open: opening %s", bt_uart_cb.port_name);

    if ((bt_uart_cb.fd = open(bt_uart_cb.port_name, O_RDWR)) < 0)
    {
        ALOGE("userial vendor open: unable to open %s", bt_uart_cb.port_name);
        return -1;
    }

    ALOGI("userial vendor open success!!");

    tcflush(bt_uart_cb.fd, TCIOFLUSH);

    tcgetattr(bt_uart_cb.fd, &bt_uart_cb.termios);
    cfmakeraw(&bt_uart_cb.termios);

    /* Set UART Control Modes */
    bt_uart_cb.termios.c_cflag |= CLOCAL;
    bt_uart_cb.termios.c_cflag |= (CRTSCTS | stop_bits| parity);


    tcsetattr(bt_uart_cb.fd, TCSANOW, &bt_uart_cb.termios);

    /* set input/output baudrate */
    cfsetospeed(&bt_uart_cb.termios, baud);
    cfsetispeed(&bt_uart_cb.termios, baud);
    tcsetattr(bt_uart_cb.fd, TCSANOW, &bt_uart_cb.termios);

#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
    userial_ioctl_init_bt_wake(bt_uart_cb.fd);
#endif
    tcflush(bt_uart_cb.fd, TCIOFLUSH);

    ALOGI("device fd = %d open", bt_uart_cb.fd);

    return bt_uart_cb.fd;
}

/*******************************************************************************
**
** Function        do_write
**
** Description     write
**
** Returns         len
**
*******************************************************************************/
static int do_write(int fd, unsigned char *buf,int len)
{
    int ret = 0;
    int write_offset = 0;
    int write_len = len;
    do {
        ret = write(fd,buf+write_offset,write_len);
        if (ret < 0)
        {
            ALOGE("write failed: %s (%d)", strerror(errno), errno);
            return -1;
        } else if (ret == 0) {
            ALOGE("write failed: %s (%d)", strerror(errno), errno);
            return 0;
        } else {
            if (ret < write_len) {
                ALOGI("write pending, do write ret: %d, %s (%d)", ret, strerror(errno), errno);
                write_len = write_len - ret;
                write_offset = ret;
            } else {
                ALOGI("Write successful");
                break;
            }
        }
    } while(1);
    return len;
}

/*******************************************************************************
**
** Function        check vendor event
**
** Description     check info
**
** Returns         success return 1
**
*******************************************************************************/
static int check_event(unsigned char * rsp, int size, unsigned char *cmd)
{
    for (int i = 0; i < size; i++) {
        ALOGD("%02x", rsp[i]);
    }

    if (!(size >= 7))
        return 0;

    if (rsp[4] != cmd[1] || rsp[5] != cmd[2] || rsp[6] != 0X00)
    {
        return 0;
    }
    return 1;
}

/*******************************************************************************
**
** Function        read_vendor_event
**
** Description     read_vendor_event
**
** Returns         str
**
*******************************************************************************/
static int read_vendor_event(int fd, unsigned char* buf, int size)
{
    int remain, r;
    int count = 0;

    if (size <= 0)
        return -1;

    struct pollfd pfd;
    int poll_ret;
    pfd.fd = fd;
    pfd.events = POLLIN | POLLHUP;

    poll_ret = poll(&pfd, 1, 100);

    if (poll_ret <= 0) {
        ALOGE("receive hci event timeout! ret=%d", poll_ret);
        return -1;
    }
    ALOGD("poll ret=%d", poll_ret);

    while (1) {
        r = read(fd, buf, 1);
        if (r <= 0)
            return -1;
        if (buf[0] == 0x04)
            break;
    }
    count++;

    while (count < 3) {
        r = read(fd, buf + count, 3 - count);
        if (r <= 0)
            return -1;
        count += r;
    }

    if (buf[2] < (size - 3))
        remain = buf[2];
    else
        remain = size - 3;

    while ((count - 3) < remain) {
        r = read(fd, buf + count, remain - (count - 3));
        if (r <= 0)
            return -1;
        count += r;
    }

    return count;
}

/*******************************************************************************
**
** Function        h5_read_vendor_event
**
** Description     h5_read_vendor_event
**
** Returns         count
**
*******************************************************************************/
static int h5_read_vendor_event(int fd, unsigned char* buf, int size)
{
    int r;
    int count = 0;

    if (size <= 0)
        return -1;

    struct pollfd pfd;
    int poll_ret;
    pfd.fd = fd;
    pfd.events = POLLIN | POLLHUP;

    poll_ret = poll(&pfd, 1, 100);

    if (poll_ret <= 0) {
        ALOGE("receive hci event timeout! ret=%d", poll_ret);
        return -1;
    }
    ALOGD("poll ret=%d", poll_ret);

    while (1) {
        r = read(fd, buf, 1);
        if (r <= 0)
            return -1;
        if (buf[0] == 0xc0)
            break;
    }
    count++;

    while (count < 2) {
        r = read(fd, buf + count, 2 - count);
        if (r <= 0)
            return -1;
        count += r;
    }

    return count;
}

static int h5_send_vendor_cmd(int fd, unsigned char* cmd, int size)
{
    if (do_write(fd, cmd, size) != size)
    {
        ALOGE("cmd send is error");
        goto error;
    }
    return 0;
error:
    return 1;
}

/*******************************************************************************
**
** Function        hci_vendor_reset
**
** Description     hci_reset
**
** Returns         int
**
*******************************************************************************/
static int start_vendor_cmd(int fd, unsigned char* cmd, int size)
{
    int rsp_size = 0;

    unsigned char rsp[HCI_MAX_EVENT_SIZE];

    memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);
    if (do_write(fd, cmd, size) != size)
    {
        ALOGE("cmd send is error");
        goto error;
    }

    rsp_size = read_vendor_event(fd, rsp, HCI_MAX_EVENT_SIZE);

    if (rsp_size < 0)
    {
        ALOGE("read vendor event error");
        goto error;
    }

    if (!check_event(rsp, rsp_size, cmd))
    {
        ALOGE("rsp event is error");
        goto error;
    }
    return 0;
error:
    return 1;

}

/*******************************************************************************
**
** Function        get_vendor_info
**
** Description     get_vendor_info
**
** Returns         str
**
*******************************************************************************/
static unsigned char * get_vendor_info(int fd, unsigned char * cmd, int size, \
    unsigned char * event, int *event_size)
{
    int rsp_size = 0;
    unsigned char rsp[HCI_MAX_EVENT_SIZE];

    ALOGD();

    memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);
    if (do_write(fd, cmd, size) != size)
    {
        ALOGE("cmd send is error");
        goto error;
    }

    rsp_size = read_vendor_event(fd, rsp, HCI_MAX_EVENT_SIZE);

    if (rsp_size < 0)
    {
        ALOGE("get_vendor_info error");
        goto error;
    }

    if (!check_event(rsp, rsp_size, cmd))
    {
        ALOGE("rsp event is error");
        goto error;
    }

    *event_size = rsp_size;
    memcpy(event, rsp, rsp_size);

    return event;
error:
    return NULL;
}

static bool matching_mfrs_vid(const unsigned char *buf, int size)
{
    bool ret = false;
    unsigned int vid = 0;

    if (size >= 13) {
        vid = (((unsigned int)buf[11])<< 8) | ((unsigned int)buf[12]);
    } else {
        if ((buf[0] == 0xc0) && (buf[1] == 0x00)) {
            vid = BT_VID_REALTECK;
        }
    }

    if (matching_dev_id_uart(vid)) {
        ret = true;
    } else {
        ALOGE("vendor id don't match :0x%04x", vid);
    }

    return ret;
}

/*******************************************************************************
**
** Function        userial_vendor_close
**
** Description     Conduct vendor-specific close work
**
** Returns         None
**
*******************************************************************************/
static void userial_vendor_close(void)
{
    int result;

    if (bt_uart_cb.fd == -1)
        return;

#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
    /* de-assert bt_wake BEFORE closing port */
    ioctl(bt_uart_cb.fd, USERIAL_IOCTL_BT_WAKE_DEASSERT, NULL);
#endif

    ALOGI("device fd = %d close", bt_uart_cb.fd);
    // flush Tx before close to make sure no chars in buffer
    tcflush(bt_uart_cb.fd, TCIOFLUSH);
    if ((result = close(bt_uart_cb.fd)) < 0)
        ALOGE( "close(fd:%d) FAILED result:%d", bt_uart_cb.fd, result);

    bt_uart_cb.fd = -1;
}

static bool distinguish_bt_mod_uart_h4(void)
{
    int fd;
    int event_size = 0;
    unsigned char event[HCI_MAX_EVENT_SIZE] = {'\0'};
    bool ret = false;

    ALOGD("start uart h4 init");

    userial_vendor_init();
    fd = userial_vendor_open((uart_cfg *) &uart_cfg_h4);
    if (fd < 0) {
        ALOGE("open uart h4 fail");
        goto exit;
    }

    if (start_vendor_cmd(fd,(unsigned char *)vendor_reset, sizeof(vendor_reset))) {
        ALOGE("start_vendor_cmd err");
        goto exit;
    }

    if (get_vendor_info(fd, (unsigned char *)vendor_info, sizeof(vendor_info),  event, &event_size) == NULL) {
        ALOGE("get_vendor_info err");
        goto exit;
    }

    if (matching_mfrs_vid(event, event_size)) {
        ALOGI("uart h4 matching vid success");
        ret = true;
    } else {
        ALOGE("uart h4 matching vid fail");
    }

exit:
    userial_vendor_close();
    return ret;
}

static bool distinguish_bt_mod_uart_h5(void)
{
    int fd;
    int event_size = 0;
    unsigned char event[HCI_MAX_EVENT_SIZE] = {'\0'};
    bool ret = false;

    ALOGD("start uart h5 init");

    userial_vendor_init();
    fd = userial_vendor_open((uart_cfg *) &uart_cfg_h5);
    if (fd < 0) {
        ALOGE("open uart h5 fail");
        goto exit;
    }

    if (h5_send_vendor_cmd(fd, (unsigned char *)vendor_sync, sizeof(vendor_sync))) {
        ALOGE("h5_send_vendor_cmd err");
        goto exit;
    }

    event_size = h5_read_vendor_event(fd, event, HCI_MAX_EVENT_SIZE);
    if (event_size < 0) {
        ALOGE("h5_read_vendor_event err");
        goto exit;
    }

    if (matching_mfrs_vid(event, event_size)) {
        ALOGI("uart h5 matching vid success");
        ret = true;
        goto exit;
    } else {
        ALOGE("uart h5 matching vid fail");
    }

exit:
    userial_vendor_close();
    return ret;
}

static bool distinguish_bt_mod_uart(void)
{
    bool ret;

    ret = distinguish_bt_mod_uart_h4();
    if (!ret) {
        ret = distinguish_bt_mod_uart_h5();
    }

    return ret;
}

bool distinguish_bt_mod(void)
{
    ALOGD();

    if (amlbt_get_name()) {
        goto exit;
    }

    if (distinguish_bt_mod_pci()) {
        goto exit;
    }

    if (distinguish_bt_mod_sdio()) {
        goto exit;
    }

    if (distinguish_bt_mod_usb()) {
        goto exit;
    }

#ifdef DISTINGUISH_BT_MOD_UART
    if (distinguish_bt_mod_uart()) {
        goto exit;
    }
#endif

    return false;

exit:
    return  true;
}

static int insmod(const char *filename, const char *args)
{
    int fd, ret;

    if (!filename) {
        ALOGE("parameters err");
        return -1;
    }

    fd = TEMP_FAILURE_RETRY(open(filename, O_RDONLY | O_CLOEXEC | O_NOFOLLOW));
    if (fd < 0) {
        ALOGE("Failed to open %s", filename);
        return -1;
    }

    ret = syscall(__NR_finit_module, fd, args, 0);
    if (ret != 0) {
        ALOGE("finit_module");
    } else {
        ALOGD("insmod ok: %s %s", filename, args ? args : "");
    }

    close(fd);

    usleep(200000);

    return ret;
}

static int rmmod(const char *modname)
{
    int ret = -1;
    int maxtry = 10;

    if (modname == NULL) {
        ALOGE("parameters err");
        return ret;
    }

    while (maxtry-- > 0) {
        ret = delete_module(modname, O_NONBLOCK | O_EXCL);
        if ((ret < 0) && (errno == EAGAIN)) {
            usleep(500000);  // 500 ms
        } else {
            break;
        }
    }

    if (ret != 0) {
        int saved_errno = errno;
        ALOGE("Unable to unload driver module %s: %s",
              modname, strerror(saved_errno));
    } else {
        ALOGD("Driver module %s unloaded successfully", modname);
    }

    return ret;
}

static int amlbt_drv_insmod(void)
{
    int i, j;

    for (i = 0; i < sizeof(aml_drv)/sizeof(drv_info); i++) {
        if (strcmp(amlbt_name, aml_drv[i].dev_name) != 0) {
            continue;
        }

        ALOGD("Matched dev_name: %s", aml_drv[i].dev_name);

        for (j = 0; j < MOD_MAX_NUM; j++) {
            if (strlen(aml_drv[i].mod[j].mod_name) == 0) {
                continue;
            }

            glob_t glob_result;
            int ret = glob(KERNEL_DRV_PATH, 0, NULL, &glob_result);

            if (ret != 0) {
                ALOGE("path can't find");
                return -1;
            }

            int found = 0;

            for (size_t k = 0; k < glob_result.gl_pathc; k++) {
                DIR *dir = opendir(glob_result.gl_pathv[k]);
                if (!dir) continue;

                struct dirent *entry;
                while ((entry = readdir(dir)) != NULL) {
                    if (strcmp(entry->d_name, aml_drv[i].mod[j].mod_name) == 0) {
                        char ko_path[BUF_MAX_LEN*2];
                        snprintf(ko_path, sizeof(ko_path), "%s/%s.ko",
                                 glob_result.gl_pathv[k], entry->d_name);
                        ALOGD("find drv: %s", ko_path);
                        insmod(ko_path, aml_drv[i].mod[j].mod_arg);
                        found = 1;
                        break;
                    }
                }
                closedir(dir);

                if (found) break;
            }

            globfree(&glob_result);

            if (!found) {
                printf("can't find: %s", aml_drv[i].mod[j].mod_name);
                return -1;
            }
        }
    }

    return 0;
}

static int amlbt_drv_modprobe(void)
{
    int i, j;
    char cmd[BUF_MAX_LEN] = {0};

    for (i = 0; i < sizeof(aml_drv)/sizeof(drv_info); i++) {
        if (strcmp(amlbt_name, aml_drv[i].dev_name) != 0) {
            continue;
        }

        ALOGD("Matched dev_name: %s", aml_drv[i].dev_name);

        for (j = 0; j < MOD_MAX_NUM; j++) {
            if (strlen(aml_drv[i].mod[j].mod_name) == 0) {
                continue;
            }

            if (strlen(aml_drv[i].mod[j].mod_arg) > 0) {
                snprintf(cmd, sizeof(cmd), "modprobe %s %s", aml_drv[i].mod[j].mod_name,
                    aml_drv[i].mod[j].mod_arg);
            } else {
                snprintf(cmd, sizeof(cmd), "modprobe %s", aml_drv[i].mod[j].mod_name);
            }

            ALOGD("exec: %s", cmd);

            int sys_ret = system(cmd);
            if (sys_ret != 0) {
                ALOGE("modprobe: %s fail", cmd);
                return -1;
            }
        }

        break;
    }

    return 0;
}

void steps_table_poll(void)
{
    int i = 0;

    // todo: get steps from conf

    for (i = 0; i < sizeof(steps_table_exp)/sizeof(steps_table); i++) {
        steps_table_exp[i].tag_ops();
    }
}

