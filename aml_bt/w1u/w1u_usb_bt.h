/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#ifndef __W1U_USB_BT_H__
#define __W1U_USB_BT_H__

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
    unsigned int can_switch_SBT;
    unsigned int rd_state; //read state

    //rx fifo for update fw point
    gdsl_fifo_t *fw_type_fifo;              //-->g_rx_type_fifo
    gdsl_fifo_t *fw_evt_fifo;               //-->g_event_fifo
    gdsl_fifo_t *fw_data_fifo;              //-->g_rx_fifo  old:fw_data_index_fifo
    //gdsl_fifo_t *fw_data_index_fifo;
    //gdsl_fifo_t *fw_data_fifo;

    //rx fifo for locad driver
    unsigned char dr_type_fifo_buf[1024];   //-->type
    gdsl_fifo_t *dr_type_fifo;              //-->g_fw_type_fifo
    unsigned char dr_evt_fifo_buf[8*1024];  //-->g_fw_evt_buf
    gdsl_fifo_t *dr_evt_fifo;               //-->g_fw_evt_fifo
    unsigned char dr_data_fifo_buf[8*1024]; //-->g_fw_data_buf
    gdsl_fifo_t *dr_data_fifo;              //-->g_fw_data_fifo
    //tx fifo
    gdsl_fifo_t *tx_cmd_fifo;                //-->g_cmd_fifo
    gdsl_tx_q_t tx_q[8]; //USB_TX_Q_NUM      //-->g_tx_q

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
    struct device_link *link;
    //input dev
    unsigned int input_key;
    struct input_dev *amlbt_input_dev;
    struct mutex dr_type_fifo_mutex;
    struct mutex dr_evt_fifo_mutex;
    struct mutex dr_data_fifo_mutex;
    //new data process
    struct sk_buff_head tx_queue;
    struct sk_buff_head bt_rx_queue;
    spinlock_t bt_rx_queue_lock;
} w1u_usb_bt_t;

int amlbt_w1uu_init(void);
void amlbt_w1uu_exit(void);

#endif


