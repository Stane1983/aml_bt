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

/*
 * LINUX_PUBLIC: default yocto STB/TV
 * AUDIO_PROJECT: audio buildroot/yocto
 * ROKU_PROJECT: roku system
 */
#if !defined(ROKU_PROJECT) && !defined(AUDIO_PROJECT)
#define LINUX_PUBLIC
#endif

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

extern unsigned int aml_mod_idx;
extern const vnd_module_t aml_module[];

void amlbt_get_mod(void);
void aml_get_fw_version(unsigned char *str);

#ifdef __cplusplus
}
#endif

#endif /* AML_COMM_H */
