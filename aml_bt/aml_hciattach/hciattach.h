/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2003-2010  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#if 0
#include <termios.h>
#include <stdint.h>
#include <syslog.h>

#ifndef N_HCI
#define N_HCI	15
#endif

#define HCIUARTSETPROTO		_IOW('U', 200, int)
#define HCIUARTGETPROTO		_IOR('U', 201, int)
#define HCIUARTGETDEVICE	_IOR('U', 202, int)
#define HCIUARTSETFLAGS		_IOW('U', 203, int)
#define HCIUARTGETFLAGS		_IOR('U', 204, int)

#define HCI_UART_H4	0
#define HCI_UART_BCSP	1
#define HCI_UART_3WIRE	2
#define HCI_UART_H4DS	3
#define HCI_UART_LL	4
#define HCI_UART_ATH3K  5
#define HCI_UART_INTEL	6
#define HCI_UART_BCM	7
#define HCI_UART_QCA	8
#define HCI_UART_AG6XX	9
#define HCI_UART_NOKIA	10
#define HCI_UART_MRVL	11
#define HCI_UART_AML	12


#define HCI_UART_RAW_DEVICE	0
#define HCI_UART_RESET_ON_INIT	1
#define HCI_UART_CREATE_AMP	2
#define HCI_UART_INIT_PENDING	3
#define HCI_UART_EXT_CONFIG	4
#define HCI_UART_VND_DETECT	5

int read_hci_event(int fd, unsigned char *buf, int size);
int set_speed(int fd, struct termios *ti, int speed);
int uart_speed(int speed);



int aml_init(int fd, char *bdaddr);
#endif

#ifndef __HCITOOL_FOR_DEBUG_H__
#define __HCITOOL_FOR_DEBUG_H__
#define LOG_TAG "Amlogic Hcitool for usb"
#define BT_USB_DEVICE  "/dev/aml_btusb"
#define RX_EVENT_TIMOUT         1000
#define READ_BT_REG_LEN         0x4
#define WRITE_BT_REG_LEN        0x8
#define LE_TEST_END_CMD_LE      0x0

#define _SYS_GET_CHAR_8_BITS(x) ((x) & 0xFF)
#define LINK_CONTROL_OGF                0x01
#define LINK_POLICY_OGF                 0x02
#define HOST_CONTROL_OGF                0x03
#define LOCAL_INFO_OGF                  0x04
#define LOCAL_STATUS_OGF                0x05
#define TEST_COMMANDS_OGF               0x06
#define LE_CONTROLLER_OGF               0x08
#define LE_SMP_COMMANDS_OGF             0x09
#define LE_GATT_COMMANDS_OGF            0x0A
#define LE_GAP_COMMANDS_OGF             0x0B
#define LE_L2CAP_COMMANDS_OGF           0x0C
#define LE_PROFILE_COMMANDS_OGF         0x0D
#define VENDOR_SPECIFIC_DEBUG_OGF       0x3f
#define PROP_OGF                        ((0x3F << 10)>>8)
#define LC_G                            ((LINK_CONTROL_OGF << 10)>>8)
#define HC_G                            ((HOST_CONTROL_OGF << 10)>>8)
#define LE_G                            ((LE_CONTROLLER_OGF << 10)>>8)

#define HCI_pduCOMMAND                                  0x01
#define HCI_EVENT_TYPE                                  0x04

#define HCI_WRITE_SCAN_ENABLE                           0x1A
#define HCI_ACCEPT_CONNECTION_REQUEST                   0x09

#define TCI_DBG_READ_REG                                0xF0
#define TCI_DBG_WRITE_REG                               0xF1
#define HCI_LE_TEST_END                                 0x1F
#define TCI_START_NORMAL_CAPTURE                        0x53
#define TCI_STOP_NORMAL_CAPTURE                         0x54
#define TCI_AML_CAPTURE_DATA_TEST                       0x37
#define TCI_WRITE_REG								    0xfef1
#define TCI_READ_REG								    0xfef0
#define TCI_UPDATE_UART_BAUDRATE					    0xfef2
#define TCI_DOWNLOAD_BT_FW							    0xfef3
#define HCI_VSC_WRITE_BD_ADDR                           0xFC1A

/*aml fw path*/
#define AML_BT_PATH "/lib/firmware/w1u"

/**********a2dp mode cfg**********/
#define AML_A2DP_CFG_FILE AML_BT_PATH"/a2dp_mode_cfg.txt"
#define AML_BT_CONFIG_RF_FILE   AML_BT_PATH"/aml_bt_rf.txt"
#define AML_W1U_BT_FW_USB_FILE  AML_BT_PATH"/w1u_bt_fw_usb.bin"


#define TCI_WRITE_READ_SUCCESS                               0
#define TCI_WRITE_READ_FAILED                                1
#define ICCM_RAM_BASE										(0x000000)
#define DCCM_RAM_BASE										(0xd00000)
#define RW_OPERTION_SIZE									(248)

#define pr_info(fmt, args...) fprintf(stdout,"[%s] " fmt "\n", __func__, ##args)
#define pr_err(fmt, args...) fprintf(stdout,"[%s]--[ERROR] [%d] " fmt "\n", __func__, __LINE__, ##args)

#define HCI_MAX_CMD_SIZE       260
#define HCI_MAX_EVENT_SIZE     260
/*opcode*/
#define HCI_RESET              0x0C03

/* HCI Packet types */
#define HCI_COMMAND_PKT      0x01
#define HCI_ACLDATA_PKT      0x02
#define HCI_SCODATA_PKT      0x03
#define HCI_EVENT_PKT        0x04
#define HCI_CMD_IND            (1)
#define HCI_COMMAND_HDR_SIZE   (3)

/*
typedef struct {
	unsigned short    opcode;
	unsigned char     plen;
} __attribute__ ((packed))  hci_command_hdr;
*/

typedef enum _download_fw_status
{
    SUCCESS,
    FAILED

}download_fw_status;

typedef unsigned char u_int8;
typedef unsigned short u_int16;
typedef unsigned int u_int32;
typedef u_int8 *t_p_pdu;
void dump(u_int8 *out, int len);
int read_event(u_int8 *buffer);
int hci_send_cmd(u_int8 *buf, int len);
int get_usb_fd();
void handle_usr_command(int argc,char *argv[]);
u_int32 read_bt_register(u_int32 address);
void write_bt_register(u_int32 address, u_int32 value);
void aml_insert_uint32(t_p_pdu p_buffer, u_int32 value_32_bit);
u_int32 aml_get_uint32(t_p_pdu p_pdu);
int accept_connect_request(t_p_pdu bd_address);
void aml_dut_loopback_enable(int timeout);
void usr_help_mesg();
void aml_set_uint16(t_p_pdu p_buffer, u_int16 value_16_bit);
u_int16 aml_get_uint16_param(t_p_pdu p_pdu);
void start_normal_capture(u_int32 test_bus);
void stop_normal_capture(const char* result_path);
int get_capture_data(const char* result_path);
int generator_capture_command(u_int32 address, u_int32 len);
int recv_capture_data(u_int32 *data, int data_len);
void hex_dump(char *pref, int width, unsigned char *buf, int len);
int start_download_firmware(char* firmware_file_path);
int TCI_Write_Register(int opcode, u_int32 address, u_int32 value);
int TCI_Read_Register(int opcode,  unsigned int address, unsigned int *value);
int download_fw_img();
int TCI_Generate_uart_load_bt_fw_Command(int opcode, u_int8 data_len, u_int32 addr, u_int8* data);
static int get_a2dp_sink_enable(void);
static int get_antenna_number(void);
static int get_config_params(void);
#endif

