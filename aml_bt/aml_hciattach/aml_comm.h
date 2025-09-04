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
#ifndef AML_COMM_H
#define AML_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/*
 * LINUX_PUBLIC: default yocto STB/TV
 * AUDIO_PROJECT: audio buildroot/yocto
 * ROKU_PROJECT: roku system
 */
#if !defined(ROKU_PROJECT) && !defined(AUDIO_PROJECT)
#define LINUX_PUBLIC
#endif

#define BT_MAC_LEN          6
#define BT_MAC_BUF_LEN      17

#ifdef ROKU_PROJECT
#define SAVE_MAC          "/nvram/bt_mac"
#define PCEDIT_MAC        "/tmp/bdaddr"
#define FW_VER_FILE       "/tmp/bt_fw_version"
#else
#define AMLBT_ADDR_PATH   "/etc/bluetooth/aml/bt_addr"
#define FW_VER_FILE       "/etc/bluetooth/aml/bt_fw_version"
#endif

#define AMLBT_RANDOM_DEV "/dev/urandom"

/* aml bt module index */
enum amlbt_idx{
    UNK_AML_IDX = 0,
    W1_UART,
    W1U_UART,
    W1U_USB,
    W2_UART,
    W2_USB,
    W2L_UART,
    W2L_USB,
    W3_UART
};

typedef struct {
    unsigned int mod_type;
    unsigned short iccm_base;
    char *mod_type_name;
    char *name;
    char *fw_file;
} vnd_module_t;

enum addr_mode {
    ADDR_FIXED_PREFIX,   // fixed prefix + random suffix
    ADDR_FULL_RANDOM     // fully random address
};

extern unsigned int aml_mod_idx;
extern const vnd_module_t aml_module[];

void amlbt_get_mod(void);
void amlbt_get_fw_ver(const char* fpath, unsigned char *str);
int amlbt_read_addr(const char* fpath, unsigned char *addr);
int amlbt_write_file(const char* fpath, const char *str, int len);
int amlbt_get_random_addr(unsigned char *bdaddr, enum addr_mode mode);

#ifdef __cplusplus
}
#endif

#endif /* AML_COMM_H */

