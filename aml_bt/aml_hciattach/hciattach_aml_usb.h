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

#ifndef HCIATTACH_AML_USB_H
#define HCIATTACH_AML_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#define BT_USB_DEVICE  "/dev/aml_btusb"
#define RX_EVENT_TIMEOUT         1000
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
#define TCI_WRITE_REG                                   0xfef1
#define TCI_READ_REG                                    0xfef0
#define TCI_UPDATE_UART_BAUDRATE                        0xfef2
#define TCI_DOWNLOAD_BT_FW                              0xfef3
#define HCI_VSC_WRITE_BD_ADDR                           0xFC1A
#define HCI_VSC_WAKE_WRITE_DATA                         0xFC22
#define HCI_READ_BD_ADDR                                0x1009

#define TCI_WRITE_READ_SUCCESS                               0
#define TCI_WRITE_READ_FAILED                                1
#define ICCM_RAM_BASE                                        (0x000000)
#define DCCM_RAM_BASE                                        (0xd00000)
#define RW_OPERATION_SIZE                                    (248)

#define DATA_LENGTH           6 //manf data len
typedef enum _download_fw_status
{
    SUCCESS,
    FAILED
}download_fw_status;

int get_usb_fd(const char *dev);
int get_usb_fd_with_retry(const char *dev, int max_retry, int delay_ms);
int aml_usb_init(int fd);

#ifdef __cplusplus
}
#endif

#endif /* HCIATTACH_AML_USB_H */
