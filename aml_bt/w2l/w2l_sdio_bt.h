/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#ifndef __W2L_SDIO_BT_H__
#define __W2L_SDIO_BT_H__

#define HCI_MAX_RX_SKB_SIZE     4096
#define HCI_UART_RAW_DEVICE     0
#define HCI_UART_RESET_ON_INIT  1
#define HCI_UART_CREATE_AMP     2
#define HCI_UART_INIT_PENDING   3
#define HCI_UART_EXT_CONFIG     4
#define HCI_UART_VND_DETECT     5

/* HCI_UART proto flag bits */
#define HCI_UART_PROTO_SET      0
#define HCI_UART_REGISTERED     1
#define HCI_UART_PROTO_READY    2
#define HCI_UART_NO_SUSPEND_NOTIFIER    3

enum bt_rx_state{
    HCI_RX_TYPE,
    HCI_RX_HEADER,
    HCI_RX_PAYLOAD,
    HCI_RX_FATAL,
};

/* TX states  */
#define HCI_UART_SENDING    1
#define HCI_UART_TX_WAKEUP  2

#define HCI_EVENT_PKT       0x04
#define HCI_ACLDATA_PKT     0x02

#define HCI_UART_MAX_BUFF   4096

#define HCI_MAX_SCO_SIZE    255
#define HCI_MAX_ISO_SIZE    251
#define HCI_MAX_EVENT_SIZE  260

#define BT_SKB_RESERVE      8

#define HCI_COMMAND_HDR_SIZE 3
#define HCI_EVENT_HDR_SIZE   2
#define HCI_ACL_HDR_SIZE     4
#define HCI_SCO_HDR_SIZE     3
#define HCI_ISO_HDR_SIZE     4

enum hci_rx_state {
    HCI_STATE_RX_TYPE,
    HCI_STATE_RX_HEADER,
    HCI_STATE_RX_PAYLOAD
};

struct hci_uart_rx {
    enum hci_rx_state state;
    uint8_t type;
    uint8_t header_len;
    uint8_t header[4];
    uint8_t *payload;
    int expected_length;
    int received_length;
};


#define HCI_TYPE_EVENT  0x04
#define HCI_TYPE_ACL    0x02
#define HCI_TYPE_15P4   0x10
//#define HCI_TYPE_ZIGBEE   0xf5
#define HCI_TYPE_ZIGBEE   0xfa
#define HCI_TYPE_THREAD   0xfa


struct hci_uart {
    struct tty_struct   *tty;
    unsigned long       flags;
    unsigned long       hdev_flags;

    //struct work_struct  init_ready;
    struct work_struct  write_work;

    struct percpu_rw_semaphore proto_lock;  /* Stop work for proto close */
    void                *priv;

    struct sk_buff      *tx_skb;
    struct sk_buff      *rd_skb;
    struct sk_buff      *rx_skb;
    struct sk_buff_head tx_queue;
    struct sk_buff_head rx_queue;
    unsigned long       rd_state;
    unsigned long       rx_state;
    unsigned long       rx_len;
    unsigned long       tx_state;
    wait_queue_head_t   wait_queue;
    unsigned char uart_buf[HCI_UART_MAX_BUFF];
    unsigned char *p_ub;
    unsigned char type;
    unsigned int init_speed;
    unsigned int oper_speed;
    unsigned char       alignment;
    unsigned char       padding;
};


#define AML_W2LS_MAX_COEX_DEVICES   4

typedef struct
{
    struct cdev dev_cdev;
    int         dev_major;
    struct class *dev_class;
    struct device *dev_device;
    struct early_suspend early_suspend;
    const unsigned char *iccm_buf;
    const unsigned char *dccm_buf;
    int irq;
    unsigned int irq_handle;
    struct work_struct wake_work;
    struct input_dev *input_dev;
    unsigned int antenna;
    unsigned int fw_mode;
    unsigned int bt_sink;
    unsigned int pin_mux;
    unsigned int br_digit_gain;
    unsigned int edr_digit_gain;
    unsigned int fw_log;
    unsigned int driver_log;
    unsigned int factory;
    unsigned int system;
    unsigned int isolation;
    struct completion notify_comp;
    unsigned int notify_trig;
    unsigned char bt_start;
    unsigned char zigbee_start;
    unsigned char thread_start;
    unsigned int bt_rd_state;
    unsigned int zigbee_rd_state;
    unsigned int thread_rd_state;
    unsigned long shutdown_value;
    wait_queue_head_t bt_wait_queue;
    struct sk_buff_head bt_tx_queue;
    struct sk_buff_head bt_rx_queue;
    wait_queue_head_t zigbee_wait_queue;
    struct sk_buff_head zigbee_tx_queue;
    struct sk_buff_head zigbee_rx_queue;
    wait_queue_head_t thread_wait_queue;
    struct sk_buff_head thread_tx_queue;
    struct sk_buff_head thread_rx_queue;
    struct hci_uart *hu;
    struct work_struct exception_work;
    struct device_link *link;
} w2l_sdio_bt_t;

/* | HCI_15P4_FLAG   | MHDL |           MID        | BODY LENGTH | checksum |
*  |   0x10 1 Byts   | 0xFA | command id 1 Bytes   |    2 Bytes  |  2 Bytes |
*/

int amlbt_w2ls_init(void);
void amlbt_w2ls_exit(void);


#endif

