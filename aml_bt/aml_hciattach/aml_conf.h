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

#ifndef AML_CONF_H
#define AML_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define HCI_MAX_CMD_SIZE       260
#define HCI_MAX_EVENT_SIZE     260
/* HCI Packet types */
#define HCI_COMMAND_PKT        0x01
#define HCI_ACLDATA_PKT        0x02
#define HCI_SCODATA_PKT        0x03
#define HCI_EVENT_PKT          0x04
#define HCI_CMD_IND            (1)
#define HCI_COMMAND_HDR_SIZE   (3)
#define HCI_RESET              0x0C03

/* bt fw&config */
#define AML_BT_PATH       "/lib/firmware/aml"
#define AML_BT_CFG_FILE   AML_BT_PATH"/aml_bt.conf"

#define AML_A2DP_CFG_FILE          AML_BT_PATH"/a2dp_mode_cfg.txt"
#define AML_BT_CONFIG_RF_FILE      AML_BT_PATH"/aml_bt_rf.txt"
#define AML_W1_BT_FW_UART_FILE     AML_BT_PATH"/w1_bt_fw_uart.bin"
#define AML_W1U_BT_FW_UART_FILE    AML_BT_PATH"/w1u_bt_fw_uart.bin"
#define AML_W1U_BT_FW_USB_FILE     AML_BT_PATH"/w1u_bt_fw_usb.bin"
#define AML_W2_BT_FW_UART_FILE     AML_BT_PATH"/w2_bt_fw_uart.bin"
#define AML_W2_BT_FW_USB_FILE      AML_BT_PATH"/w2_bt_fw_usb.bin"
#define AML_W2L_BT_FW_UART_FILE    AML_BT_PATH"/w2l_bt_15p4_fw_uart.bin"
#define AML_W2L_BT_FW_USB_FILE     AML_BT_PATH"/w2l_bt_15p4_fw_usb.bin"

#define MANF_ROW                      16
#define MANF_COLUMN                   16
#define PATH_BUF_MAX_LEN 256
#define BIT(_n)                 (1 << (_n))

//get aml_bt.conf param
extern unsigned int amlbt_rftype;
extern unsigned int amlbt_btsink;
extern unsigned int amlbt_fw_mode;
extern unsigned int amlbt_pin_mux;
extern unsigned int amlbt_br_digit_gain;
extern unsigned int amlbt_edr_digit_gain;
extern unsigned int amlbt_fwlog_config;
extern unsigned char APCF_config_manf_data[256];
extern unsigned int amlbt_manf_cnt;
extern unsigned int amlbt_factory;
extern unsigned int amlbt_system;
extern unsigned int amlbt_manf_para;
extern unsigned char w1u_manf_data[MANF_ROW][MANF_COLUMN];

void amlbt_load_conf();

#ifdef __cplusplus
}
#endif

#endif /* AML_CONF_H */

