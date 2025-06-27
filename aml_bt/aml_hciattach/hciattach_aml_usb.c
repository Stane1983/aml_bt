/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2000-2001  Qualcomm Incorporated
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2002-2010  Marcel Holtmann <marcel@holtmann.org>
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

#include <stdio.h>
#include <getopt.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
//#include <sys/io.h>
#include <limits.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>

#include "hciattach.h"
#include "hciattach_aml_usb.h"
#include "aml_multibt.h"
#include "aml_conf.h"

#define UINT8_TO_STREAM(p, u8)   { *(p)++ = (char)(u8); }
#define UINT16_TO_STREAM(p, u16) { *(p)++ = (char)(u16); *(p)++ = (char)((u16) >> 8); }
#define CMD_TEST_FW _IOR('A', 2, int)
#define MAC_LEN 6
#define MAC_DELAY 350
#define SAVE_MAC "/nvram/bt_mac"
#define PCEDIT_MAC "/tmp/bdaddr"
static uint8_t vendor_local_addr[MAC_LEN];
static int debug = 0;
extern int g_usb_fd;
unsigned char *BT_fwICCM = NULL;
unsigned char *BT_fwDCCM = NULL;
unsigned int ICCM_LEN = 0;
unsigned int DCCM_LEN = 0;

/* aml bt module */
#define W1U_USB    0x01
#define W2_USB     0x02
#define W2L_USB    0x03

static int AML_MODULE = 0;

extern unsigned int  amlbt_manf_para;
extern unsigned char w1u_manf_data[MANF_ROW][MANF_COLUMN];

static const vnd_module_t aml_module[] ={
    {W1U_USB,  256, "W1U_USB",  "aml_w1u",    AML_W1U_BT_FW_USB_FILE },
    {W2_USB,   256, "W2_USB",   "aml_w2_u",   AML_W2_BT_FW_USB_FILE},
    {W2L_USB,  384, "W2L_USB",  "aml_w2l_u",  AML_W2L_BT_FW_USB_FILE },
    { 0,       0,    NULL,       NULL,       NULL},
};

//#define W2L_CHIPSET
//#define W2_CHIPSET
#define W1U_CHIPSET

#if defined(W2L_CHIPSET)
    #define CHIP_OFFSET (384 * 1024)
#elif defined(W1U_CHIPSET) || defined(W2_CHIPSET)
    #define CHIP_OFFSET (256 * 1024)
#else
    #error "No chipset defined. Please define W1U_CHIPSET, W2_CHIPSET, or W2L_CHIPSET."
#endif

static void ms_delay(unsigned int timeout)
{
    struct timespec delay;
    int err;

    if (timeout == 0)
        return;

    delay.tv_sec = timeout / 1000;
    delay.tv_nsec = 1000 * 1000 * (timeout % 1000);

    /* [u]sleep can't be used because it uses SIGALRM */
    do {
        err = nanosleep(&delay, &delay);
    } while (err < 0 && errno == EINTR);
}

static void aml_insert_uint32(unsigned char *p_buffer, unsigned int value_32_bit)
{
    p_buffer[0] = (unsigned char)_SYS_GET_CHAR_8_BITS(value_32_bit);
    p_buffer[1] = (unsigned char)_SYS_GET_CHAR_8_BITS(value_32_bit >> 8);
    p_buffer[2] = (unsigned char)_SYS_GET_CHAR_8_BITS(value_32_bit >> 16);
    p_buffer[3] = (unsigned char)_SYS_GET_CHAR_8_BITS(value_32_bit >> 24);
}

static unsigned int aml_get_uint32(unsigned char *p_pdu)
{
    return (*p_pdu) + ((*(p_pdu + 1)) << 8) + ((*(p_pdu + 2)) << 16) + ((*(p_pdu + 3)) << 24);
}

static void dump(unsigned char *out, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        if (i && !(i % 16))
        {
            fprintf(stderr, "\n");
        }
        fprintf(stderr, "%02x ", out[i]);
    }
    fprintf(stderr, "\n");
}

static void amlbt_get_mod(void)
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

    AML_MODULE = aml_module[idx].mod_type;

    ALOGI("aml_module: %d", AML_MODULE);
}

int get_usb_fd_with_retry(const char *dev, int max_retry, int delay_ms)
{
    int usb_fd = -1;
    int retry = 0;

    ALOGI("Trying to open BT USB Device: %s max_retry: %d delay_ms %d", dev, max_retry, delay_ms);
    while (retry < max_retry)
    {
        usb_fd = open(dev, O_RDWR | O_NOCTTY);
        if (usb_fd >= 0)
        {
            ALOGI("BT USB Device %s opened successfully on retry %d with fd %d", dev, retry, usb_fd);
            return usb_fd;
        }
        else
        {   // error = 2 ENOENT   error = 13 EACCES
            ALOGE("Attempt %d: Failed to open %s, error: %d (%s)", retry + 1, dev, errno, strerror(errno));
            ms_delay(delay_ms);
            retry++;
        }
    }

    ALOGE("Failed to open %s after %d retries", dev, max_retry);
    return -1;
}

int get_usb_fd(const char *dev)
{
    int usb_fd = -1;

    ALOGI("The device path passed in is: %s", dev);
    usb_fd = open(dev, O_RDWR | O_NOCTTY);
    if (usb_fd == -1)
    {
        ALOGE("Failed to open BT USB Device %s, error: %d", dev, errno);
        return -1;
    }
    ALOGI("BT USB Device %s opened successfully with file descriptor %d", dev, usb_fd);
    return usb_fd;
}

static void hex_dump(char *pref, int width, unsigned char *buf, int len)
{
    register int i,n;

    for (i = 0, n = 1; i < len; i++, n++) {
        if (n == 1)
            printf("%s", pref);
        printf("%2.2X ", buf[i]);
        if (n == width) {
            printf("\n");
            n = 0;
        }
    }
    if (i && n!= 1)
        printf("\n");
}

static int read_event(unsigned char *buffer)
{
    int rx_len = 0;
    int timeout_cnt = 0;
    int event_cnt = 0;
    unsigned int event_parameter_len = 0;

    ///M @{
    fd_set read_fds;
    int evt_state = 0;
    int rval;
    struct timeval wait_time = {0, 200000};
    enum
    {
        EVT_TYPE,
        EVT_HEAD,
        EVT_PLAYLOAD,
        EVT_END
    };

    ///@}

    while (evt_state < EVT_END)
    {
        FD_ZERO(&read_fds);
        FD_SET(g_usb_fd, &read_fds);
        rval = select(g_usb_fd + 1, &read_fds, NULL, NULL, &wait_time);
        if (rval > 0)
        {
            switch (evt_state)
            {
                case EVT_TYPE:
                    rx_len = read(g_usb_fd, &buffer[event_cnt], 1);
                    if (rx_len <= 0) {
                        ALOGE("EVT_TYPE read failed: rx_len %d\n", rx_len);
                        return -1;
                    }
                    event_cnt += rx_len;
                    evt_state++;
                    break;
                case EVT_HEAD:
                    rx_len = read(g_usb_fd, &buffer[event_cnt], 2);
                    if (rx_len <= 0) {
                        ALOGE("EVT_HEAD read failed: rx_len %d\n", rx_len);
                        return -1;
                    }
                    event_cnt += rx_len;
                    event_parameter_len = buffer[2];
                    evt_state++;
                case EVT_PLAYLOAD:
                    rx_len = read(g_usb_fd, &buffer[event_cnt], event_parameter_len);
                    if (rx_len <= 0) {
                        ALOGE("EVT_PLAYLOAD read failed: rx_len %d\n", rx_len);
                        return -1;
                    }
                    event_cnt += rx_len;
                    evt_state++;
                    break;
                default:
                    break;
            }
        }
        else if (rval == 0)
        {
            timeout_cnt++;
            if (timeout_cnt > 150)
            {
                printf("Rx event parameter timeout!\n");
                return 0;
            }
        }
        wait_time.tv_sec = 0;
        wait_time.tv_usec = 200000;
        FD_ZERO(&read_fds);
        FD_SET(g_usb_fd, &read_fds);
    }
    if (debug)
        dump(buffer, event_cnt);
    return event_cnt;
}

static int read_event_with_retry(unsigned char *buffer, int max_retry, int delay_ms)
{
    int err = 0;
    int retry = 0;

    ALOGI("read event with retry max_retry: %d, delay: %d\n",max_retry, delay_ms);
    while (retry < max_retry)
    {
        ms_delay(delay_ms);
        err = read_event(buffer);
        if (err > 0)
        {
            ALOGI("read event success err %d retry: %d\n",err, retry);
            break;
        } else {
            ALOGE("read_event failed err: %d\n",err);
        }
        retry++;
    }

    if (err <= 0)
    {
        ALOGE("read_event_with_retry: failed after %d retries\n", max_retry);
    }

    return err;
}


static int hci_send_cmd(unsigned char *buf, int len)
{
    int write_bytes = 0;
    //unsigned char hcitype[] = {HCI_pduCOMMAND};
    //write_bytes = write(g_usb_fd, hcitype, 1);
    write_bytes = write(g_usb_fd, buf, len);
    if (write_bytes)
    {
        //write_bytes = write(g_usb_fd, buf + 1, len -1);
        if (debug)
        {
            fprintf(stderr, "writing\n");
            dump(buf, len);

            if (write_bytes == -1) {
                fprintf(stderr,"Failed to write bytes to uart,error code is[%d]\n",errno);
            }
        }
         //printf("Send hcicommand success! write_bytes:%d\n",write_bytes);
        //return 0;
    }
    else
    {
        printf("Send hcicommand failed!\n");
        return -1;
    }
    return 0;
}

static unsigned int read_bt_register(unsigned int address)
{
    int ret = 0;
    unsigned char pdu[8] = {0};
    unsigned int data = 0;
    unsigned char recv_data[256] = {0};

    pdu[0] = HCI_pduCOMMAND;
    pdu[1] = TCI_DBG_READ_REG;
    pdu[2] = PROP_OGF;
    pdu[3] = READ_BT_REG_LEN;
    aml_insert_uint32(&pdu[4], address);
    hci_send_cmd(pdu, sizeof(pdu));
    ret = read_event(recv_data);
    if (ret > 0)
    {
        if ((recv_data[0] == 0x04) &&
            (recv_data[4] == TCI_DBG_READ_REG) &&
            (recv_data[5] == PROP_OGF))
        {
            data = aml_get_uint32(&recv_data[7]);
            return data;
        }
    }
    ALOGE("Read BT register 0x%06x failed", address);
    return 0;
}

static void write_bt_register(unsigned int address, unsigned int value)
{
    int ret = 0;
    unsigned char pdu[12] = {0};
    unsigned char recv_data[256] = {0};

    pdu[0] = HCI_pduCOMMAND;
    pdu[1] = TCI_DBG_WRITE_REG;
    pdu[2] = PROP_OGF;
    pdu[3] = WRITE_BT_REG_LEN;
    aml_insert_uint32(&pdu[4], address);
    aml_insert_uint32(&pdu[8], value);
    hci_send_cmd(pdu, sizeof(pdu));
    ret = read_event(recv_data);
    if (ret > 0)
    {
        if ((recv_data[0] == 0x04) &&
            (recv_data[4] == TCI_DBG_WRITE_REG) &&
            (recv_data[5] == PROP_OGF))
        {
            goto WriteSuccess;
        }
    }
    ALOGE("Write BT register 0x%06x failed", address);
WriteSuccess:
    return;
}

static int aml_tci_load_bt_fw_cmd(int opcode, unsigned char data_len, unsigned int addr, unsigned char* data)
{
    unsigned char pdu[300] = { 0 };
    unsigned short length = 0;
    int i = 0;
    int rx_len = 0;
    unsigned char rx_buff[1024] = {0};
    unsigned char *temp_data = (unsigned char*)malloc(300);
    if (!temp_data)
    {
        ALOGE("New space fail");
        return -1;
    }
    pdu[0] = (unsigned char)(opcode & 0xFF);
    pdu[1] = (unsigned char)((opcode >> 8) & 0xFF);
    pdu[2] = data_len;
    length = pdu[2] + 3;
    aml_insert_uint32(pdu + 3, addr);
    for (i = 0; i < data_len; i += 1)
    {
        pdu[i + 7] = *(data + i);
    }
    //temp_data[0] = HCI_pduCOMMAND;
    //memcpy(temp_data + 1, pdu, length);
    //hci_send_cmd(temp_data, length + 1);
    memcpy(temp_data, pdu, length);
    hci_send_cmd(temp_data, length);

    //dump(temp_data, length + 1);
    rx_len = read_event(rx_buff);
    if (rx_len > 0)
    {
        free(temp_data);
        return 0;
    }
    free(temp_data);
    return -1;
}

static int aml_usb_download_fw_img()
{
    int count = 0;
    int result = 0;
    unsigned char* bufferICCM = BT_fwICCM;
    unsigned char* bufferDCCM = BT_fwDCCM;
    int len = 0, offset = 0, offset_in_bt =0;
    unsigned int cmd_len = 0;
    unsigned int data_len = 0;
    int cnt = 1;
    offset_in_bt = CHIP_OFFSET;
    offset = CHIP_OFFSET;
    len = ICCM_LEN - offset_in_bt;//ALIGN(sizeof(BT_fwICCM), 4) -256*1024;
    ALOGI("BT start iccm copy, total=0x%x ", len);

    if (len == 0)
        goto DCC_START;
    ALOGI("BT iccm base 0x%x ", ICCM_RAM_BASE);
    do
    {
        data_len = (len > RW_OPERATION_SIZE) ? RW_OPERATION_SIZE : len;
        cmd_len = data_len + 4;
        result = aml_tci_load_bt_fw_cmd(TCI_DOWNLOAD_BT_FW, cmd_len,
          ICCM_RAM_BASE + offset, bufferICCM + offset);
        if (result)
        {
            ALOGE("Download iccm data error");
            return FAILED;
        }
        cnt++;
        offset += data_len;
        len -= data_len;
        count++;
    } while (len > 0);
    ALOGI("Download iccm data success");
DCC_START:
    len = DCCM_LEN;//ALIGN(sizeof(BT_fwDCCM), 4);
    offset = 0;
    cnt = 1;
    data_len = 0;
    cmd_len = 0;
    ALOGI("start dccm copy, total=0x%x", len);
    do
    {
        data_len = (len > RW_OPERATION_SIZE) ? RW_OPERATION_SIZE : len;
        cmd_len = data_len + 4;
        result = aml_tci_load_bt_fw_cmd(TCI_DOWNLOAD_BT_FW, cmd_len, DCCM_RAM_BASE + offset, bufferDCCM + offset);
        if (result) {
            ALOGE("Download dccm data error");
            return -1;
        }
        cnt++;
        offset += data_len;
        len -= data_len;
        count++;
    } while (len > 0);
    ALOGI("Download dccm data sunccess");
    return SUCCESS;
}

static int aml_tci_write_register(int opcode, unsigned int address, unsigned int value)
{
    unsigned char pdu[30] = { 0 };
    unsigned char *data = (unsigned char*)malloc(30);
    unsigned char tmp_buff[4096] = { 0 };
    int rx_len = 0;
    unsigned short length = 0;

    if (!data) {
        ALOGE("New space fail");
        return TCI_WRITE_READ_FAILED;
    }

    pdu[0] = (unsigned char)(opcode & 0xFF);
    pdu[1] = (unsigned char)((opcode >> 8) & 0xFF);
    pdu[2] = 8;
    length = pdu[2] + 3;
    aml_insert_uint32(pdu + 3, address);
    aml_insert_uint32(pdu + 7, value);
    //data[0] = HCI_pduCOMMAND;
    //memcpy(data + 1, pdu, length);
    //hci_send_cmd(data, length + 1);
    memcpy(data, pdu, length);
    hci_send_cmd(data, length);
    rx_len = read_event(tmp_buff);
    dump(tmp_buff, rx_len);

    free(data);
    ALOGI("TCI_WRITE_READ_SUCCESS success!\n");
    return TCI_WRITE_READ_SUCCESS;
}

static int aml_get_a2dp_sink_enable(void)
{
    int size;
    int fd_a2dp_cfg = 0;
    char buffer[255] = { 0 };
    char c = '=';
    unsigned char a2dp_sink_enable = 0;

    fd_a2dp_cfg = open(AML_A2DP_CFG_FILE, O_RDONLY);
    if (fd_a2dp_cfg < 0)
    {
        ALOGE("In %s, Open failed:%s", __FUNCTION__, strerror(errno));
        return -1;
    }

    size = read(fd_a2dp_cfg, buffer, sizeof(buffer));
    if (size < 0)
    {
        ALOGE("In %s, Read failed:%s", __FUNCTION__, strerror(errno));
        close(fd_a2dp_cfg);
        return -1;
    }

    buffer[sizeof(buffer) - 1] = 0;
    char *ptr_a2dp_cfg = strchr(buffer, c);
    if (!ptr_a2dp_cfg) {
        ALOGE("In %s, wrong a2dp_sink_enable value", __FUNCTION__);
        close(fd_a2dp_cfg);
        return -1;
    }
    ptr_a2dp_cfg++;
    a2dp_sink_enable = atoi(ptr_a2dp_cfg);

    close(fd_a2dp_cfg);
    ALOGI("Setting parameters to controller: a2dp_sink_enable=%d.", a2dp_sink_enable);
    return a2dp_sink_enable;
}

static int aml_get_antenna_number(void)
{
    int size;
    char c = '=';
    unsigned char antenna_num = 0;
    int fd_ant_num = 0;
    char buffer[255] = { 0 };

    fd_ant_num = open(AML_BT_CONFIG_RF_FILE, O_RDONLY);
    if (fd_ant_num < 0)
    {
            ALOGE("In %s, Open failed:%s", __FUNCTION__, strerror(errno));
            return -1;
    }

    size = read(fd_ant_num, buffer, sizeof(buffer));
    if (size < 0)
    {
        ALOGE("In %s, Read failed:%s", __FUNCTION__, strerror(errno));
        close(fd_ant_num);
        return -1;
    }

    buffer[sizeof(buffer) - 1] = 0;
    char *ptr_a2dp_cfg = strchr(buffer, c);
    if (!ptr_a2dp_cfg) {
         ALOGE("In %s, wrong a2dp_sink_enable value", __FUNCTION__);
         close(fd_ant_num);
         return -1;
    }
    ptr_a2dp_cfg++;
    antenna_num = atoi(ptr_a2dp_cfg);

    close(fd_ant_num);
    ALOGI("Setting parameters to controller: antenna number=%d.", antenna_num);
    return antenna_num;
}

static int aml_get_config_params(void)
{
    unsigned int  reg_data = 0;
    unsigned char a2dp_sink = 0;
    unsigned char ant_num = 0;

    a2dp_sink = aml_get_a2dp_sink_enable();
    ant_num = aml_get_antenna_number();
    ALOGI("a2dp_sink_enable=%d ,antenna number=%d", a2dp_sink,ant_num);

    if (ant_num == 1)
    {
        reg_data = 0x10000000; //bit28 means RF_ANT_SINGLE
    }
    else if (ant_num == 2)
    {
        reg_data = 0x20000000; //bit29 means RF_ANT_DOUBLE
    }

    if (a2dp_sink == 1)
    {
        reg_data |= (1<<25);    // bit25 means a2dp_sink_enable.
    }

    ALOGI("reg_data=%x", reg_data);//0x12000000
    return reg_data;
}

static int aml_get_random(unsigned char *bdaddr)
{
    int randomData = open("/dev/random", O_RDONLY);
    if (randomData < 0) {
        ALOGE("Failed to open /dev/random");
        return 1;
    }

    unsigned char buffer[4];
    ssize_t bytesRead = read(randomData, buffer, sizeof(buffer));
    if (bytesRead < 0) {
        ALOGE("Failed to read from /dev/random");
        close(randomData);
        return 1;
    }

    memcpy(bdaddr + 2, buffer, 4);
    close(randomData);
    return 0;
}

static unsigned char * aml_getprop_read(const char* str)
{
    int fd, n;
    char buf[18];

    memset(buf, '\0', sizeof(buf));
    ALOGI("opening %s", str);
    fd = open(str, O_RDONLY|O_CREAT, 0666);
    if (fd < 0)
    {
        ALOGE("open read");
        goto error;
    }
    n = read(fd, buf, sizeof(buf)-1);
    if (n < sizeof(buf)-1)
    {
        ALOGE("n < sizeof(buf)");
        close(fd);
        goto error;
    }

    buf[sizeof(buf)-1] ='\0';
    close(fd);

    if (strnlen(buf, 17) != 17)
    {
        ALOGE("don't matching bt mac");
        goto error;
    }
    sscanf(buf, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
                 &vendor_local_addr[0], &vendor_local_addr[1], &vendor_local_addr[2],\
                 &vendor_local_addr[3], &vendor_local_addr[4], &vendor_local_addr[5]);
    return vendor_local_addr;

error:
    return NULL;
}

static int aml_setprop_write(const char *str, int size)
{
    int err = -1;
    int fd;

    ALOGI("opening %s\n", str);
    fd = open(SAVE_MAC, O_WRONLY|O_CREAT, 0666);
    if (fd < 0)
    {
        ALOGE("open write");
        goto error;
    }
    err = write(fd, str, size);
    if (err != size)
    {
        ALOGE("write fail");
    }
    close(fd);

error:
    return err;
}

static int aml_get_fw_version(unsigned char *str)
{
    int fd;
    int ret;
    char * fw_version = NULL;
    str = str + 7; //skip 7byte

    ret = asprintf(&fw_version, "fw_version: date = %02x.%02x, number = 0x%02x%02x\n", *(str+1),*str,*(str+3),*(str+2));
    if (ret <= 0)
    {
        ALOGI("get version value failed\n");
        goto error;
    }
    ALOGI("%s", fw_version);
    fd = open(FW_VER_FILE,  O_WRONLY|O_CREAT|O_TRUNC, 0666);
    if (fd < 0)
    {
        ALOGE("open fw_file fail");
        free(fw_version);
        goto error;
    }
    write(fd, fw_version, strlen(fw_version));
    free(fw_version);
    close(fd);

error:
    return 0;
}

static int aml_hci_reset(int fd)
{
    int size, err = 0;
    unsigned char cmd[HCI_MAX_CMD_SIZE];
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    hci_command_hdr *cmd_hdr;
    int write_bytes = 0;

    ALOGI("HCI RESET");
    memset(cmd, 0x0, HCI_MAX_CMD_SIZE);
    cmd_hdr = (void *) (cmd + 1);
    cmd[0]  = HCI_COMMAND_PKT;
    cmd_hdr->opcode = HCI_RESET;
    cmd_hdr->plen   = 0;

    /* Total length of the packet to be sent to the Controller */
    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE);

    /* Send the HCI command packet to UART for transmission */
    ALOGI("HCI CMD: 0x%x 0x%x 0x%x 0x%x", cmd[0], cmd[1], cmd[2], cmd[3]);

    unsigned char hcitype[] = {HCI_pduCOMMAND};
    write_bytes = write(fd, hcitype, 1);
    if (write_bytes)
    {
        err = write(fd, cmd +1, size-1);
        if (err != size-1) {
            ALOGE("Send failed with ret value: %d", err);
            goto error;
        }
    } else {
        goto error;
    }
    ALOGI("Received HCI-Vendor Specific Event from SOC");

    /* Wait for command complete event */
    err = read_event(rsp);
    if ( err < 0) {
        ALOGE("Failed to set patch info on Controller");
        goto error;
    }

    ALOGI("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    if (rsp[4] != cmd[1] || rsp[5] != cmd[2] || rsp[6] != 0X00) {
        ALOGE("Failed to tci_write, command failure");
        return -1;
    }
    ALOGI("reset success");
    return err;

error:
    ALOGI("hci reset error");
    return err;
}

static int aml_set_wakeup_param_roku(int fd)
{
    int size;
    int err = -1;
    char *cmd_hdr = NULL;
    int write_bytes = 0;
    char line[256];
    int found = 0;
    unsigned char cmd[HCI_MAX_CMD_SIZE];
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    unsigned char APCF_config_manf_data[9] = {0};
    unsigned char apcf_config[9] = {0};

    ALOGI("set wake roku_manf_data");
    FILE *config_file = fopen(AML_BT_CFG_FILE, "r");
    if (!config_file)
    {
        ALOGE("Failed to open aml_bt.conf");
        return -EINVAL;
    }

    while (fgets(line, sizeof(line), config_file) && !found)
    {
        if (strstr(line, "APCF_config_manf_data") != NULL)
        {
            char *value = strchr(line, '=');
            if (value)
            {
                value++;
                char *newline = strchr(value, '\n');
                if (newline) *newline = '\0';

                size_t len = strlen(value);
                if (len != 18)
                {
                    ALOGE("Invalid APCF_config_manf_data length: %zu", len);
                    fclose(config_file);
                    return -EINVAL;
                }

                for (int i = 0; i < 9; i++)
                {
                    if (sscanf(value + (i * 2), "%2hhx", &APCF_config_manf_data[i]) != 1)
                    {
                        ALOGE("Failed to parse APCF_config_manf_data at index %d", i);
                        fclose(config_file);
                        return -EINVAL;
                    }
                    apcf_config[i] = APCF_config_manf_data[i];
                }
                found = 1;
            }
        }
    }
    fclose(config_file);

    if (!found)
    {
        ALOGE("APCF_config_manf_data not found in config file");
        return -EINVAL;
    }

    ALOGI("set APCF: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", apcf_config[0],apcf_config[1], apcf_config[2], apcf_config[3], apcf_config[4], apcf_config[5], apcf_config[6], apcf_config[7], apcf_config[8]);

    memset(cmd, 0x0, HCI_MAX_CMD_SIZE);
    memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);

    cmd_hdr = (void *) (cmd + 1);
    cmd[0]    = HCI_COMMAND_PKT;
    UINT16_TO_STREAM(cmd_hdr, HCI_VSC_WAKE_WRITE_DATA);
    *cmd_hdr++ = 5;
    *cmd_hdr++ = APCF_config_manf_data[4];
    *cmd_hdr++ = APCF_config_manf_data[5];
    *cmd_hdr++ = APCF_config_manf_data[6];
    *cmd_hdr++ = APCF_config_manf_data[7];
    *cmd_hdr++ = APCF_config_manf_data[8];

    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + 5);
    /* Send the HCI command packet to UART for transmission */
    ALOGI("HCI CMD: 0x%x 0x%x 0x%x 0x%x", cmd[0], cmd[1], cmd[2], cmd[3]);

    unsigned char hcitype[] = {HCI_pduCOMMAND};
    write_bytes = write(fd, hcitype, 1);

    if (write_bytes)
    {
        err = write(fd, cmd +1, size-1);
        if (err != size-1) {
            ALOGE("Send failed with ret value: %d", err);
            goto error;
        }
    } else {
        goto error;
    }

    err = read_event_with_retry(rsp, 5, 10);
    if ( err < 0) {
        ALOGE("Failed to set apcf config on Controller");
        goto error;
    }
    ALOGI("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    ALOGI("hci set roku apcf success %d",err);
    return err;

error:
    return err;
    ALOGI("hci set apcf error");
}

static int aml_set_wakeup_param_w1u(int fd)
{
    int total_len = 0;
    int cnt_data = 0;
    int cmd_size = 0;
    int write_bytes = 0;
    int err = -1;
    int cnt = 0;
    unsigned char cmd[HCI_MAX_CMD_SIZE] = {0};
    unsigned char rsp[HCI_MAX_EVENT_SIZE] = {0};
    unsigned char *p = &cmd[1];  // cmd[0] = HCI_COMMAND_PKT
    unsigned char *total_manf_data = NULL;
    unsigned char default_manf_data[] = {0x01, 0x22, 0xFC, 0x05, 0x19, 0xff, 0x01, 0x0a, 0xb};

    ALOGI("set wake w1u_manf_data");
    if (amlbt_manf_para == 0)
    {
        ALOGI("amlbt_manf_para is 0, use default");
        ALOGI("set wake default_manf_data");
        //err = aml_hci_send_cmd(fd, (unsigned char *)default_manf_data,
            //sizeof(default_manf_data),(unsigned char *)rsp, HCI_MAX_EVENT_SIZE);
        return 0;
    }

    total_manf_data = malloc(LOCAL_BDADDR_PATH_BUFFER_LEN);
    if (!total_manf_data) {
        ALOGE("Memory alloc failed!");
        return -ENOMEM;
    }
    memset(total_manf_data, 0, LOCAL_BDADDR_PATH_BUFFER_LEN);

    while (cnt < MANF_ROW) {
        if ((BIT(cnt) & amlbt_manf_para) != 0)//15 index->0/1/2/3
        {
            int group_len = w1u_manf_data[cnt][0] + 1;//param length + payload = 1 + 9
            if ((total_len + group_len) > LOCAL_BDADDR_PATH_BUFFER_LEN)
            {
                ALOGE("APCF buffer overflow!");
                break;
            }
            else
            {
                memcpy(&total_manf_data[total_len], w1u_manf_data[cnt], group_len);
                ALOGI("%#x %#x %#x %#x %#x %#x %#x %#x", total_manf_data[total_len], total_manf_data[total_len+1],
                        total_manf_data[total_len+2], total_manf_data[total_len+3],
                        total_manf_data[total_len+4], total_manf_data[total_len+5],
                        total_manf_data[total_len+6], total_manf_data[total_len+7]);
                total_len += group_len;
                ALOGI("%#x %#x %#x %#x %#x %#x %#x %#x", w1u_manf_data[cnt][0], w1u_manf_data[cnt][1],
                        w1u_manf_data[cnt][2], w1u_manf_data[cnt][3],
                        w1u_manf_data[cnt][4], w1u_manf_data[cnt][5],
                        w1u_manf_data[cnt][6], w1u_manf_data[cnt][7]);
                cnt_data++;
            }
        }
        cnt++;
    }

    if (total_len == 0 || cnt_data == 0) {
        ALOGE("No w1uF manf data to send");
        free(total_manf_data);
        return -EINVAL;
    }

    //build HCI cmd
    cmd[0] = HCI_COMMAND_PKT;
    UINT16_TO_STREAM(p, HCI_VSC_WAKE_WRITE_DATA);  // opcode
    UINT8_TO_STREAM(p, total_len + 1);             // total = all param length + 1(cnt_data)
    UINT8_TO_STREAM(p, cnt_data);                  // cnt data
    memcpy(p, total_manf_data, total_len);
    cmd_size = HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + 1/*cnt_data*/ + total_len;

    ALOGI("HCI CMD: 0x%x 0x%x 0x%x 0x%x 0x%x", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
    unsigned char hcitype[] = {HCI_pduCOMMAND};
    write_bytes = write(fd, hcitype, 1);

    if (write_bytes)
    {
        err = write(fd, cmd + 1, cmd_size - 1);
        if (err != cmd_size - 1) {
            ALOGE("Send failed with ret value: %d", err);
            goto error;
        }
    } else {
        goto error;
    }

    err = read_event_with_retry(rsp, 5, 10);
    if (err < 0) {
        ALOGE("Failed to set w1u_manf_data config on Controller");
        goto error;
    }
    ALOGI("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5]);
    ALOGI("hci set w1u_manf_data success %d", err);

    free(total_manf_data);
    return err;

error:
    return err;
    ALOGI("hci set w1u_manf_data error");
}

static int aml_set_wakeup_param_public(int fd)
{
    int size;
    int err = -1;
    char *cmd_hdr = NULL;
    int write_bytes = 0;
    unsigned char rsp[HCI_MAX_EVENT_SIZE] = {'0'};
    unsigned char APCF_config_manf_data[] = {0x01, 0x22, 0xFC, 0x05, 0x19, 0xff, 0x01, 0x0a, 0xb};
    unsigned char cmd[HCI_MAX_CMD_SIZE];

    ALOGI("set ManfData_public: %02x:%02x:%02x:%02x:%02x:%02x",
            APCF_config_manf_data[0], APCF_config_manf_data[1], APCF_config_manf_data[2],
            APCF_config_manf_data[3], APCF_config_manf_data[4], APCF_config_manf_data[5]);

    memset(cmd, 0x0, HCI_MAX_CMD_SIZE);
    memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);

    cmd_hdr = (void *) (cmd + 1);
    cmd[0] = HCI_COMMAND_PKT;
    UINT16_TO_STREAM(cmd_hdr, HCI_VSC_WAKE_WRITE_DATA);

    *cmd_hdr++ = 5;
    *cmd_hdr++ = APCF_config_manf_data[4];
    *cmd_hdr++ = APCF_config_manf_data[5];
    *cmd_hdr++ = APCF_config_manf_data[6];
    *cmd_hdr++ = APCF_config_manf_data[7];
    *cmd_hdr++ = APCF_config_manf_data[8];

    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + 5);
    ALOGI("HCI CMD: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7], cmd[8]);

    unsigned char hcitype[] = {HCI_pduCOMMAND};
    write_bytes = write(fd, hcitype, 1);

    if (write_bytes)
    {
        err = write(fd, cmd + 1, size - 1);
        if (err != size - 1) {
            ALOGE("Send failed with ret value: %d", err);
            goto error;
        }
    } else {
        goto error;
    }

    err = read_event_with_retry(rsp, 5, 10);
    if (err < 0) {
        ALOGE("Failed to set ManfData_public config on Controller");
        goto error;
    }
    ALOGI("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5]);
    ALOGI("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", rsp[6], rsp[7], rsp[8], rsp[9], rsp[10], rsp[11]);
    ALOGI("hci set ManfData_public success %d", err);
    return err;

error:
    return err;
    ALOGI("hci set ManfData_public error");
}

#if 0
static int aml_set_apcf_config_default(int fd)
{
    int size;
    int err = -1;
    char *cmd_hdr = NULL;
    int write_bytes = 0;
    char line[256];
    int found = 0;
    unsigned char cmd[HCI_MAX_CMD_SIZE];
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    unsigned char manf_data[DATA_LENGTH] = {0};

    FILE *config_file = fopen(AML_BT_CFG_FILE, "r");
    if (!config_file)
    {
        ALOGE("Failed to open aml_bt.conf");
        return -EINVAL;
    }

    while (fgets(line, sizeof(line), config_file) && !found)
    {
        if (strstr(line, "ManfData_Default") != NULL)
        {
            char *value = strchr(line, '=');
            if (value)
            {
                value++;
                char *newline = strchr(value, '\n');
                if (newline) *newline = '\0';

                size_t len = strlen(value);
                if (len != DATA_LENGTH * 2)
                {
                    ALOGE("Invalid ManfData_Default length: %zu", len);
                    fclose(config_file);
                    return -EINVAL;
                }

                for (int i = 0; i < DATA_LENGTH; i++)
                {
                    if (sscanf(value + (i * 2), "%2hhx", &manf_data[i]) != 1)
                    {
                        ALOGE("Failed to parse ManfData_Default at index %d", i);
                        fclose(config_file);
                        return -EINVAL;
                    }
                }
                found = 1;
            }
        }
    }
    fclose(config_file);

    if (!found)
    {
        ALOGE("ManfData_Default not found in config file");
        return -EINVAL;
    }

    ALOGI("set ManfData_Default: %02x:%02x:%02x:%02x:%02x:%02x",
            manf_data[0], manf_data[1], manf_data[2],
            manf_data[3], manf_data[4], manf_data[5]);

    memset(cmd, 0x0, HCI_MAX_CMD_SIZE);
    memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);

    cmd_hdr = (void *) (cmd + 1);
    cmd[0] = HCI_COMMAND_PKT;
    UINT16_TO_STREAM(cmd_hdr, HCI_VSC_WAKE_WRITE_DATA);
    /**cmd_hdr++ = DATA_LENGTH;

    for (int i = 0; i < DATA_LENGTH; i++) {
        *cmd_hdr++ = manf_data[i];
    }*/
        *cmd_hdr++ = 5;
    *cmd_hdr++ = manf_data[1];
    *cmd_hdr++ = manf_data[2];
    *cmd_hdr++ = manf_data[3];
    *cmd_hdr++ = manf_data[4];
    *cmd_hdr++ = manf_data[5];

    //size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + DATA_LENGTH);
    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + 5);
    ALOGI("HCI CMD: 0x%x 0x%x 0x%x 0x%x", cmd[0], cmd[1], cmd[2], cmd[3]);

    unsigned char hcitype[] = {HCI_pduCOMMAND};
    write_bytes = write(fd, hcitype, 1);

    if (write_bytes)
    {
        err = write(fd, cmd + 1, size - 1);
        if (err != size - 1) {
            ALOGE("Send failed with ret value: %d", err);
            goto error;
        }
    } else {
        goto error;
    }

    ms_delay(10);
    ALOGI("delay %d", 10);

    err = read_event(rsp);
    if (err < 0) {
        ALOGE("Failed to set ManfData_Default config on Controller");
        goto error;
    }
    ALOGI("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
            rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5]);
    ALOGI("hci set ManfData_Default success %d", err);
    return err;

error:
    return err;
    ALOGI("hci set ManfData_Default error");
}
#endif

/******************************************************************************
**  check efuse bdaddr
******************************************************************************/
static int aml_check_efuse_bdaddr(int fd)
{
    int size;
    int err = -1;
    unsigned char cmd[HCI_MAX_CMD_SIZE];
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    char *cmd_hdr = NULL;

    memset(cmd, 0x0, HCI_MAX_CMD_SIZE);
    memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);

    cmd_hdr = (void *) (cmd + 1);
    cmd[0]	= HCI_COMMAND_PKT;
    UINT16_TO_STREAM(cmd_hdr, HCI_READ_BD_ADDR);
    *cmd_hdr++ = 0;
    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE);

    err = write(fd, cmd, size);
    if (err != size) {
        ALOGE("Send failed with ret value: %d", err);
        goto error;
    }

    /* Wait for command complete event */
    err = read_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE);
    if ( err < 0) {
        ALOGE("Failed to set patch info on Controllern");
        goto error;
    }

    if ((rsp[7] == 0x00)
        && (rsp[8] == 0xff)
        && (rsp[9] == 0x00)
        && (rsp[10] == 0x22)
        && (rsp[11] == 0x2d)
        && (rsp[12] == 0xae)
        || (rsp[2]  == 0x01)) {
        ALOGI("No valid mac in efuse");
        err = -1;
    }
    ALOGI("done");

error:
    return err;
}

static int aml_set_bdaddr(int fd)
{
    int size;
    int err = -1;
    unsigned char cmd[HCI_MAX_CMD_SIZE];
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    char *cmd_hdr = NULL;
    int write_bytes = 0;
    char buf[18];
    unsigned char *tempbuf;
    unsigned char local_addr[MAC_LEN];

    if ((tempbuf = aml_getprop_read(PCEDIT_MAC)) != NULL)
    {
        memcpy(local_addr, tempbuf, MAC_LEN);
        goto set_mac;
    }
    /*
    else if (aml_check_efuse_bdaddr(fd) > 0 )
    {
        ALOGI("use mac in efuse");
        memset(local_addr, 0, MAC_LEN);
        goto set_mac;
    }*/
    else if ((tempbuf = aml_getprop_read(SAVE_MAC)) != NULL)
    {
        memcpy(local_addr, tempbuf, MAC_LEN);
        goto set_mac;
    }

    memset(buf, '\0', sizeof(buf));
    memset(local_addr, '\0', MAC_LEN);

    local_addr[0] = 0x22;
    local_addr[1] = 0x22;
    /*get random number from /dev/random, if succeed save it to file*/
    if (!aml_get_random(local_addr))
    {
        sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",local_addr[0],local_addr[1], local_addr[2], local_addr[3], local_addr[4], local_addr[5]);
        err = aml_setprop_write(buf, sizeof(buf));
        if (err < 0)
        {
            ALOGI("aml_getprop_write fail");
        }
    } else {
        ALOGE("Failed to get random mac addr");
        goto error;
    }

set_mac:
    ALOGI("set bdaddr: %02x:%02x:%02x:%02x:%02x:%02x", local_addr[0],local_addr[1], local_addr[2], local_addr[3], local_addr[4], local_addr[5]);

    memset(cmd, 0x0, HCI_MAX_CMD_SIZE);
    memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);

    cmd_hdr = (void *) (cmd + 1);
    cmd[0]    = HCI_COMMAND_PKT;
    UINT16_TO_STREAM(cmd_hdr, HCI_VSC_WRITE_BD_ADDR);
    *cmd_hdr++ = MAC_LEN;
    *cmd_hdr++ = local_addr[5];
    *cmd_hdr++ = local_addr[4];
    *cmd_hdr++ = local_addr[3];
    *cmd_hdr++ = local_addr[2];
    *cmd_hdr++ = local_addr[1];
    *cmd_hdr++ = local_addr[0];

    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + MAC_LEN);
    /* Send the HCI command packet to UART for transmission */
    ALOGI("HCI CMD: 0x%x 0x%x 0x%x 0x%x", cmd[0], cmd[1], cmd[2], cmd[3]);

    unsigned char hcitype[] = {HCI_pduCOMMAND};
    write_bytes = write(fd, hcitype, 1);

    if (write_bytes)
    {
        err = write(fd, cmd +1, size-1);
        if (err != size-1) {
            ALOGE("Send failed with ret value: %d", err);
            goto error;
        }
    }
    else
    {
        goto error;
    }

    err = read_event_with_retry(rsp, 5, 10);
    if ( err < 0) {
        ALOGE("Failed to set patch info on Controllern");
        goto error;
    }

    ALOGI("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    ALOGI("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", rsp[7], rsp[8], rsp[9], rsp[10], rsp[11], rsp[12], rsp[13]);

    if (rsp[4] != cmd[1] || rsp[5] != cmd[2] || rsp[6] != 0X00) {
        ALOGE("Failed to set_bdaddr, command failure");
        return -1;
    }
    aml_get_fw_version(rsp);
    ALOGI("hci set bdaddr success %d",err);
    return err;

error:
    return err;
    ALOGI("hci set bdaddr error");
}

static int aml_download_firmware(char* firmware_file_path)
{
    int binfile_fd = -1;
    unsigned int iccm_size = 0;
    unsigned int dccm_size = 0;
    int size = 0;
    char fw_bin_file[384];
    int result = 0;
    //int revData = -1;
    unsigned int reg_data = 0;
    int err = -1;
    int success = 1;

    memset(fw_bin_file, 0, sizeof(fw_bin_file));
    if ((binfile_fd = open(firmware_file_path, O_RDONLY)) < 0)
    {
        printf("Could not open %s\n", firmware_file_path);
        return err;
    }
    printf("open firmware file success binfile_fd:%d", binfile_fd);
    size = read(binfile_fd, &iccm_size, 4);
    printf("hw_config_get_iccm_size iccm_size:%d size:%d\n", iccm_size,size);
    if (size < 0)
    {
        printf("---------1hw_config_get_iccm_size read error!---------");
        close(binfile_fd);
        return err;
    }
    printf("---------2hw_config_get_iccm_size iccm_size %d---------\n", iccm_size);
    size = read(binfile_fd, &dccm_size, 4);
    if (size < 0)
    {
        printf("---------hw_config_get_dccm_size read error!---------");
        close(binfile_fd);
        return err;
    }
    printf("---------hw_config_get_dccm_size dccm_size %d---------\n", dccm_size);
    BT_fwICCM = (unsigned char*)malloc(iccm_size);
    BT_fwDCCM = (unsigned char*)malloc(dccm_size);
    if ((!BT_fwICCM) ||(!BT_fwDCCM))
    {
        printf("Malloc failed!\n");
        return err;
    }
    size = read(binfile_fd, BT_fwICCM, iccm_size);
    if (size < 0)
    {
        printf("---------hw_config_get_iccm_data error!---------");
        close(binfile_fd);
        free(BT_fwICCM);
        free(BT_fwDCCM);
        return err;
    }
    size = read(binfile_fd, BT_fwDCCM, dccm_size);
    if (size < 0)
    {
        printf("---------hw_config_get_dccm_data error!---------");
        close(binfile_fd);
        free(BT_fwICCM);
        free(BT_fwDCCM);
        return err;
    }
    ICCM_LEN = iccm_size;
    DCCM_LEN = dccm_size;

    if (aml_usb_download_fw_img() == FAILED) return err;
    result = aml_tci_write_register(TCI_WRITE_REG, 0xa70014, 0x0000000);
    if (!result) {
        ALOGE("close fw load");;
    }
    else
    {
        ALOGE("close fw load failed");
        return err;
    }

    reg_data = aml_get_config_params();
    ALOGI("0xf03040 reg_data=%u", reg_data);
    result = aml_tci_write_register(TCI_WRITE_REG, 0xf03040, reg_data);
    if (result == TCI_WRITE_READ_FAILED)
    {
        ALOGE("BT CPU can't work");;
        return err;
    }

    result = aml_tci_write_register(TCI_WRITE_REG, 0xf03058, 0x0000000);
    if (result == TCI_WRITE_READ_FAILED)
    {
        ALOGE("reset controller failed");;
        return err;
    }

    free(BT_fwICCM);
    free(BT_fwDCCM);
    //revData = ioctl(g_usb_fd, CMD_TEST_FW, 101);
    //printf("receive data=%d\n", revData);
    close(binfile_fd);
    return success;
}

/*
  1.download fw files
  2.0xa70014  set rf calibration
  3.0xf03040  set config params REG_PMU_POWER_CFG
  4.0xf03058  cpu start 		REG_DEV_RESET
  5.0xfc1a    set bdaddr		HCI_VSC_WRITE_BD_ADDR
  6.0xfc22    set wake up param HCI_VSC_WAKE_WRITE_DATA
*/
int aml_usb_init(int fd)
{
    int err = -1;

#if 0
    err = aml_download_firmware(AML_W1U_BT_FW_USB_FILE);
    if (err < 0)
    {
        ALOGE("Download fw file failed!");
        goto error;
    }
    ALOGI("Download fw file successfully");
#endif

    ms_delay(MAC_DELAY);
    ALOGI("delay %d wait cpu start",MAC_DELAY);

    amlbt_get_mod();

    /*read conf file*/
    amlbt_load_conf();

    err = aml_set_bdaddr(fd);
    if (err < 0)
    {
        ALOGE("HCI set bdaddr failed");
        goto error;
    }
    ALOGI("HCI set bdaddr is done");

#ifdef ROKU_PROJECT
    err = aml_set_wakeup_param_roku(fd);
#elif LINUX_PUBLIC
    if (AML_MODULE == W1U_USB)
    {
        err = aml_set_wakeup_param_w1u(fd);
    }
    else
    {
        err = aml_set_wakeup_param_public(fd);
    }
#else
    #error "Please define ROKU_PROJECT or LINUX_PUBLIC to select the correct APCF config"
#endif
    if (err < 0)
    {
        ALOGE("HCI set wake manf data failed");
        goto error;
    }
    ALOGI("HCI set wake manf data is done");

    ALOGI("aml_usb_init success %d",err);
    close(fd);
    return err;

error:
    ALOGI("aml_usb_init failed");
    close(fd);
    return err;
}
