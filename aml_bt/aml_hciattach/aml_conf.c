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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string.h>
#include <ctype.h>


#include "aml_multibt.h"
#include "hciattach_aml_usb.h"
#include "aml_conf.h"

#if 0
/******************************************************************************
**  Externs
******************************************************************************/
int userial_set_port(char *p_conf_name, char *p_conf_value, int param);
int hw_set_patch_file_path(char *p_conf_name, char *p_conf_value, int param);
int hw_set_patch_file_name(char *p_conf_name, char *p_conf_value, int param);

#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
int hw_set_patch_settlement_delay(char *p_conf_name, char *p_conf_value, int param);
#endif
#endif

/******************************************************************************
**  Local type definitions
******************************************************************************/

#define CONF_COMMENT '#'
#define CONF_DELIMITERS " =\n\r\t"
#define CONF_VALUES_DELIMITERS "=\n\r\t"
#define CONF_MAX_LINE_LEN 255

typedef int (conf_action_t)(char *p_conf_name, char *p_conf_value, int param);

typedef struct {
    const char *    conf_entry;
    conf_action_t * p_action;
    int    param;
} conf_entry_t;

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
**  Static variables
******************************************************************************/

/*
 * Current supported entries and corresponding action functions
 */
static const conf_entry_t conf_table[] = {
#if 0
	{ "UartPort",		    userial_set_port,		   0 },
	{ "FwPatchFilePath",	    hw_set_patch_file_path,	   0 },
	{ "FwPatchFileName",	    hw_set_patch_file_name,	   0 },
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
	{ "FwPatchSettlementDelay", hw_set_patch_settlement_delay, 0 },
#endif
#endif
	{ (const char *)NULL,	    NULL,			   0 }
};


/*****************************************************************************
**   CONF INTERFACE FUNCTIONS
*****************************************************************************/

/*******************************************************************************
**
** Function        vnd_load_conf
**
** Description     Read conf entry from p_path file one by one and call
**                 the corresponding config function
**
** Returns         None
**
*******************************************************************************/
void vnd_load_conf(const char *p_path)
{
    FILE *p_file;
    char *p_name;
    char *p_value;
    conf_entry_t *p_entry;
    char line[CONF_MAX_LINE_LEN + 1]; /* add 1 for \0 char */

    ALOGI("Attempt to load conf from %s", p_path);

    if ((p_file = fopen(p_path, "r")) != NULL)
    {
        /* read line by line */
        while (fgets(line, CONF_MAX_LINE_LEN + 1, p_file) != NULL)
        {
            if (line[0] == CONF_COMMENT)
                continue;

            p_name = strtok(line, CONF_DELIMITERS);

            if (NULL == p_name)
            {
                continue;
            }

            p_value = strtok(NULL, CONF_DELIMITERS);

            if (NULL == p_value)
            {
                ALOGE("vnd_load_conf: missing value for name: %s", p_name);
                continue;
            }

            p_entry = (conf_entry_t *)conf_table;

            while (p_entry->conf_entry != NULL)
            {
                if (strcmp(p_entry->conf_entry, (const char *)p_name) == 0)
                {
                    p_entry->p_action(p_name, p_value, p_entry->param);
                    break;
                }

                p_entry++;
            }
        }

        fclose(p_file);
    }
    else
    {
        ALOGE("vnd_load_conf file >%s< not found", p_path);
    }
}

static char *aml_trim(char *str) {
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

    while (i < strlen(p))
    {
        sscanf(p + i, "%2x", &tmp);
        //ALOGD("%#02x", tmp);
        if (tmp < 0 || tmp > 0xff)
        {
            ALOGI("%#02x", tmp);
            return 0;
        }
        APCF_config_manf_data[j++] = *((unsigned char*)&tmp);

        if (isspace(*(p+2+i)))
        {
            i += 3;
        }
        else
        {
            i += 2;
        }
    }
    return j;
}

static int w1u_manf_data_split(char *p)
{
    int tmp = 0;
    int i = 0, j = 0, index = 0;

    while (i < strlen(p))
    {
        sscanf(p + i, "%2x", &tmp);
        //ALOGD("%#02x", tmp);
        if (tmp < 0 || tmp > 0xff)
        {
            ALOGI("%#02x", tmp);
            return 0;
        }
        w1u_manf_data[index][j++] = *((unsigned char*)&tmp);

        if (isspace(*(p+2+i)))
        {
            i += 3;
            index++;
            j = 0;
        }
        else
        {
            i += 2;
        }
    }
    return index;
}

void amlbt_load_conf()
{
    char *str;
    char *split;
    int line_num = 0;
    char line[1024] = {0};
    //char value[1024];

    FILE *fp = fopen(AML_BT_CFG_FILE, "rt");
    if (!fp) {
      ALOGE("unable to open file %s", AML_BT_CFG_FILE);
      return;
    }
    ALOGI("success to open file %s", AML_BT_CFG_FILE);

    while (fgets(line, sizeof(line), fp)) {
        char *line_ptr = aml_trim(line);
        char line_f[100] = {0};
        ++line_num;

        // Skip blank and comment lines.
        if (*line_ptr == '\0' || *line_ptr == '#' || *line_ptr == '[')
          continue;

        split = strchr(line_ptr, '=');
        if (!split) {
            ALOGI("no key/value separator found on line %d.", line_num);
            continue;
        }
        strncpy(line_f,line_ptr,strlen(line_ptr)-strlen(split));
        // *split = '\0';
        ALOGI(" %s  %s", aml_trim(line_f), aml_trim(split+1));
        char *endptr;
        if (!strcmp(aml_trim(line_f), "BtPowerOn")) {
            amlbt_poweron = strtol(aml_trim(split+1), &endptr, 0);
        ALOGI("amlbt_poweron %d", amlbt_poweron);
        }
        else if (!strcmp(aml_trim(line_f), "BtChip")) {
            amlbt_chiptype = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_chiptype %d", amlbt_chiptype);
        }
        else if (!strcmp(aml_trim(line_f), "BtRecovery")) {
            amlbt_btrecovery = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_btrecovery %d", amlbt_btrecovery);
        }
        else if (!strcmp(aml_trim(line_f), "BtSink")) {
            amlbt_btsink = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_btsink %d", amlbt_btsink);
        }
        else if (!strcmp(aml_trim(line_f), "BtAntenna")) {
            amlbt_rftype = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_rftype %d", amlbt_rftype);
        }
        else if (!strcmp(aml_trim(line_f), "FirmwareMode")) {
            amlbt_fw_mode = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_fw_mode %d", amlbt_fw_mode);
        }
        else if (!strcmp(aml_trim(line_f), "ChangePinMux")) {
            amlbt_pin_mux = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_pin_mux %d", amlbt_pin_mux);
        }
        else if (!strcmp(aml_trim(line_f), "BrDigitGain")) {
            amlbt_br_digit_gain = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_br_digit_gain %#x", amlbt_br_digit_gain);
        }
        else if (!strcmp(aml_trim(line_f), "EdrDigitGain")) {
            amlbt_edr_digit_gain = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_edr_digit_gain %#x", amlbt_edr_digit_gain);
        }
        else if (!strcmp(aml_trim(line_f), "Btfwlog")) {
            amlbt_fwlog_config = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_edr_digit_gain %#x", amlbt_fwlog_config);
        }
        else if (!strcmp(aml_trim(line_f), "ManfData")) {
            str = aml_trim(split+1);
            ALOGI("manfdata '%s' len %zu", str, strlen(str));
            if (strlen(str) < 2)
            {
                ALOGE("manfdata error");
            }
            else
            {
                amlbt_manf_cnt = manf_data_split(str);
            }
            if (amlbt_manf_cnt % 6 != 0)
            {
                amlbt_manf_cnt = 0;
            }
            ALOGI("manf cnt %d", amlbt_manf_cnt);
        }
        else if (!strcmp(aml_trim(line_f), "W1UManfData")) {
            str = aml_trim(split+1);
            ALOGI("manfdata %s len %zu", str, strlen(str));
            if (strlen(str) < 2)
            {
                ALOGE("manfdata error");
            }
            else
            {
                w1u_manf_data_split(str);
            }
        }
        else if (!strcmp(aml_trim(line_f), "Btfactory")) {
            amlbt_factory = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_factory '%#x'", amlbt_factory);
        }
        else if (!strcmp(aml_trim(line_f), "Btsystem")) {
            amlbt_system = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_system '%#x'", amlbt_system);
        }
        else if (!strcmp(aml_trim(line_f), "Manfcnt")) {
            amlbt_manf_para = strtol(aml_trim(split+1), &endptr, 0);
            ALOGI("amlbt_manf_para '%#x'", amlbt_manf_para);
        }
    }
    fclose(fp);
}

