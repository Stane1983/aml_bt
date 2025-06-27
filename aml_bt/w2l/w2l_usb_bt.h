/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#ifndef __W2L_USB_BT_NEW_H__
#define __W2L_USB_BT_NEW_H__

typedef struct
{
    struct cdev dev_cdev;
    int         dev_major;
    struct class *dev_class;
    struct device *dev_device;
    struct platform_device *pdev;      //platform device
    struct early_suspend early_suspend;
    unsigned char firmware_start;
    unsigned char hardware_error;
    struct completion comp;
    const unsigned char *iccm_buf;
    const unsigned char *dccm_buf;
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
    unsigned int rd_state; //read state
    //rx fifo
    gdsl_fifo_t *fw_type_fifo;
    gdsl_fifo_t *fw_evt_fifo;
    gdsl_fifo_t *fw_data_fifo;
    unsigned char dr_type_fifo_buf[1024];
    gdsl_fifo_t *dr_type_fifo;
    unsigned char dr_evt_fifo_buf[8*1024];
    gdsl_fifo_t *dr_evt_fifo;
    unsigned char dr_data_fifo_buf[8*1024];
    gdsl_fifo_t *dr_data_fifo;
    //tx fifo
    gdsl_fifo_t *tx_cmd_fifo;
    gdsl_tx_q_t tx_q[8]; //USB_TX_Q_NUM
    //15.4 fifo
    gdsl_fifo_t *_15p4_dr_fifo;
    gdsl_fifo_t *_15p4_tx_fifo;
    gdsl_fifo_t *_15p4_rx_fifo;
    unsigned char dr_15p4_buf[4*1024];
    //rc manfdata
    unsigned char rc_manfdata[6*8];
    unsigned int manfdata_len;
    //mac addr
    unsigned char mac_addr[6];
    unsigned int sink_mode;
    unsigned int dr_state;
    unsigned char *usb_rx_buf;
    unsigned int usb_rx_len;
    struct urb *bt_urb;
    struct task_struct *usb_irq_task;
    struct semaphore usb_irq_sem;
    struct semaphore sr_sem;
    unsigned char usb_irq_task_quit;
    wait_queue_head_t rd_wait_queue;
    struct mutex dr_type_fifo_mutex;
    struct mutex dr_evt_fifo_mutex;
    struct mutex dr_data_fifo_mutex;
    //ioctl
    unsigned long recovery_value;
    unsigned long shutdown_value;
    unsigned int input_key;
    struct input_dev *amlbt_input_dev;
    struct device_link *link;
    struct wakeup_source *amlbt_wakeup_source;
    unsigned int wake_mux;
    //new data process
    struct sk_buff_head tx_queue;
    struct sk_buff_head bt_rx_queue;
    spinlock_t bt_rx_queue_lock;
}w2l_usb_bt_t;

int amlbt_w2lu_init(void);
void amlbt_w2lu_exit(void);

#endif

