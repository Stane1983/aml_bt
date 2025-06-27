/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#ifndef __RC_LIST_H__
#define __RC_LIST_H__

#define AML_BT_CHAR_RCLIST_NAME "aml_rclist"

#define MAC_ADDR_LEN            6
#define MAX_MAC_LIST            8
#define MAX_USER_BUF_LEN        (MAX_MAC_LIST*17+1)

#define FIFO_FW_RC_LIST_ADDR    (0x93b260 + 2364 + 4) //addr 0x93bba0 usb bulk 16 bit alignment
#define FIFO_FW_SDIO_RC_LIST_ADDR    (0xa163c0) //addr 0xa163c0 sdio bulk 16 bit alignment

int amlbt_rc_list_init(struct device *dev, \
                        ws_inf p_ws_func, \
                        rs_inf p_rs_func, \
                        sdio_ww_inf p_ww_sdio_func, \
                        sdio_rw_inf p_rw_sdio_func);
void amlbt_rc_list_deinit(struct device *dev);
int amlbt_write_rclist_to_firmware(void);
void amlbt_clear_rclist_from_firmware(void);
void amlbt_wakeup_lock(void);
void amlbt_wakeup_unlock(void);

typedef struct _rc_list_t
{
    char mac[MAC_ADDR_LEN];
    char used;
} rc_list_t;

typedef struct
{
    int wake_mux;
    struct wakeup_source *amlbt_wakeup_source;
} list_bt_t;

#endif

