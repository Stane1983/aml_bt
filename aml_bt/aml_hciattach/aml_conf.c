/******************************************************************************
*
*  Copyright (C) 2019-2021 Amlogic Corporation
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

/******************************************************************************
*
*  Filename:      conf.c
*
*  Description:   Contains functions to conduct run-time module configuration
*                 based on entries present in the .conf file
*
******************************************************************************/

#define _GNU_SOURCE

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "aml_multibt.h"
#include "hciattach_aml_usb.h"
#include "aml_conf.h"

/******************************************************************************
**  Local type definitions
******************************************************************************/
#define CONF_COMMENT '#'
#define CONF_DELIMITERS " =\n\r\t"
#define CONF_VALUES_DELIMITERS "=\n\r\t"
#define CONF_MAX_LINE_LEN 255

typedef int (*conf_action)(char *value, unsigned int *param);

typedef struct {
    const char *conf_name;      // Configure item name
    conf_action action;         // Processing function
    unsigned int *param;        // Optional parameters
} conf_item_t;

unsigned int amlbt_poweron = 2;
unsigned int amlbt_chiptype = 2;
unsigned int amlbt_btrecovery = 0;
unsigned int amlbt_btsink = 0;
unsigned int amlbt_rftype = 0;
unsigned int amlbt_fw_mode = 0;
unsigned int amlbt_pin_mux = 0;
unsigned int amlbt_br_digit_gain = 0;
unsigned int amlbt_edr_digit_gain = 0;
unsigned int amlbt_fwlog_config = 0;
unsigned int amlbt_manf_cnt = 0;
unsigned int amlbt_factory = 0;
unsigned int amlbt_system = 0;
unsigned int amlbt_manf_para = 0;
unsigned char APCF_config_manf_data[256] = {'\0'};
unsigned char w1u_manf_data[MANF_ROW][MANF_COLUMN] = {0};

/******************************************************************************
**  Function declaration
******************************************************************************/
static int handle_uint(char *value, unsigned int *param);
static int handle_manf_data(char *value, unsigned int* param);
static int handle_w1u_manf_data(char *value, unsigned int* param);

/******************************************************************************
**  Static variables
******************************************************************************/
static conf_item_t conf_table[] = {
    {"BtPowerOn",      handle_uint,            &amlbt_poweron},
    {"BtChip",         handle_uint,            &amlbt_chiptype},
    {"BtRecovery",     handle_uint,            &amlbt_btrecovery},
    {"BtSink",         handle_uint,            &amlbt_btsink},
    {"BtAntenna",      handle_uint,            &amlbt_rftype},
    {"FirmwareMode",   handle_uint,            &amlbt_fw_mode},
    {"ChangePinMux",   handle_uint,            &amlbt_pin_mux},
    {"BrDigitGain",    handle_uint,            &amlbt_br_digit_gain},
    {"EdrDigitGain",   handle_uint,            &amlbt_edr_digit_gain},
    {"Btfwlog",        handle_uint,            &amlbt_fwlog_config},
    {"ManfData",       handle_manf_data,       NULL},
    {"W1UManfData",    handle_w1u_manf_data,   NULL},
    {"Btfactory",      handle_uint,            &amlbt_factory},
    {"Btsystem",       handle_uint,            &amlbt_system},
    {"Manfcnt",        handle_uint,            &amlbt_manf_para},
    {NULL,             NULL,                   0}
};

/*****************************************************************************
**   CONF INTERFACE FUNCTIONS
*****************************************************************************/
static char *aml_trim(char *str)
{
    while (isspace(*str))
        ++str;

    if (!*str)
        return str;

    char *end_str = str + strlen(str) - 1;
    while (end_str > str && isspace(*end_str))
        --end_str;

    end_str[1] = '\0';
    return str;
}

static int manf_data_split(char *p)
{
    int tmp = 0;
    int i=0,j=0;

    while (i < strlen(p)) {
        sscanf(p + i, "%2x", &tmp);
        //ALOGD("%#02x", tmp);
        if (tmp < 0 || tmp > 0xff) {
            ALOGD("%#02x", tmp);
            return 0;
        }
        APCF_config_manf_data[j++] = *((unsigned char*)&tmp);

        if (isspace(*(p+2+i))) {
            i += 3;
        } else {
            i += 2;
        }
    }
    return j;
}

static int w1u_manf_data_split(char *p)
{
    int tmp = 0;
    int i = 0, j = 0, index = 0;

    while (i < strlen(p)) {
        sscanf(p + i, "%2x", &tmp);
        //ALOGD("%#02x", tmp);
        if (tmp < 0 || tmp > 0xff) {
            ALOGD("%#02x", tmp);
            return 0;
        }
        w1u_manf_data[index][j++] = *((unsigned char*)&tmp);

        if (isspace(*(p+2+i))) {
            i += 3;
            index++;
            j = 0;
        } else {
            i += 2;
        }
    }
    return index;
}

static int handle_uint(char *value, unsigned int *param)
{
    char *endptr;

    if (!value || !param) {
         ALOGE("parameters err");
        return -1;
    }

    *param = strtol(value, &endptr, 0);

    ALOGD("value:%u", *param);

    return 0;
}

static int handle_manf_data(char *value, unsigned int *param)
{
    (void)param;

    if (!value || strlen(value) < 2) {
        ALOGE("manfdata err");
        return -1;
    }

    amlbt_manf_cnt = manf_data_split(value);
    if (amlbt_manf_cnt % 6 != 0)
        amlbt_manf_cnt = 0;

    ALOGD("amlbt_manf_cnt:%u", amlbt_manf_cnt);

    return 0;
}

static int handle_w1u_manf_data(char *value, unsigned int *param)
{
    (void)param;

    if (!value || strlen(value) < 2) {
        ALOGE("manfdata err");
        return -1;
    }

    ALOGD("manfdata:%s len:%zu", value, strlen(value));

    w1u_manf_data_split(value);

    return 0;
}

void amlbt_load_conf(void)
{
    FILE *fp = fopen(AML_BT_CFG_FILE, "rt");
    if (!fp) {
        ALOGE("unable to open file %s", AML_BT_CFG_FILE);
        return;
    }
    ALOGD("success to open file %s", AML_BT_CFG_FILE);

    char line[1024];
    int line_num = 0;
    while (fgets(line, sizeof(line), fp)) {
        ++line_num;
        char *line_ptr = aml_trim(line);
        if (*line_ptr == '\0' || *line_ptr == '#' || *line_ptr == '[')
            continue;

        char *split = strchr(line_ptr, '=');
        if (!split) {
            ALOGD("no key/value separator found on line %d.", line_num);
            continue;
        }

        *split = '\0';
        char *key = aml_trim(line_ptr);
        char *value = aml_trim(split + 1);

        for (int i = 0; conf_table[i].conf_name != NULL; i++) {
            if (strcmp(key, conf_table[i].conf_name) == 0) {
                ALOGD("%s", conf_table[i].conf_name);
                conf_table[i].action(value, conf_table[i].param);
                break;
            }
        }
    }

    fclose(fp);
}

