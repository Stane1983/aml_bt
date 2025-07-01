/******************************************************************************
*
*  Copyright (C) 2019-2025 Amlogic Corporation
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

#ifndef __AML_COEX_H__
#define __AML_COEX_H__

#define HCI_COMMAND_PKT        0x01
#define HCI_ACLDATA_PKT        0x02
#define HCI_SCODATA_PKT        0x03
#define HCI_EVENT_PKT          0x04
#define HCI_15P4_PKT           0x10
#define HCI_15P4_HDR_SIZE      4

#define HCI_AML_ZIGBEE_TYPE    0xF5
#define HCI_AML_THREAD_TYPE    0xFA

enum
{
    LOG_LEVEL_FATAL,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_ALL,
};


struct _gdsl_fifo_t;
typedef struct _gdsl_fifo_t gdsl_fifo_t;

struct _gdsl_fifo_t
{
    //unsigned char *fifo;
    unsigned char *r;
    unsigned char *w;
    unsigned char *base_addr;
    unsigned int size;
};

#define H4_RECV_ACL \
	.type = HCI_ACLDATA_PKT, \
	.hlen = HCI_ACL_HDR_SIZE, \
	.loff = 2, \
	.lsize = 2, \
	.maxlen = HCI_MAX_FRAME_SIZE \

#define H4_RECV_SCO \
	.type = HCI_SCODATA_PKT, \
	.hlen = HCI_SCO_HDR_SIZE, \
	.loff = 2, \
	.lsize = 1, \
	.maxlen = HCI_MAX_SCO_SIZE

#define H4_RECV_EVENT \
	.type = HCI_EVENT_PKT, \
	.hlen = HCI_EVENT_HDR_SIZE, \
	.loff = 1, \
	.lsize = 1, \
	.maxlen = HCI_MAX_EVENT_SIZE

#define H4_RECV_15P4 \
	.type = HCI_15P4_PKT, \
	.hlen = HCI_15P4_HDR_SIZE, \
	.loff = 2, \
	.lsize = 3, \
	.maxlen = HCI_MAX_EVENT_SIZE\

struct aml_coex_struct {
	struct sk_buff *rx_skb;
	struct sk_buff_head txq;
	struct mutex txq_mutex;
	struct sk_buff_head rxq;      // 接收队列
	struct sk_buff_head thread_rxq;
        struct sk_buff_head zigbee_rxq;
};

#if 0
struct h4_recv_pkt {
	u8  type;	/* Packet type */
	u8  hlen;	/* Header length */
	u8  loff;	/* Data length offset in header */
	u8  lsize;	/* Data length field size */
	u16 maxlen;	/* Max overall packet length */
	int (*recv)(struct hci_dev *hdev, struct sk_buff *skb);
};
#endif

extern int g_dbg_level;

#define BTA(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_ALL) printk(KERN_INFO "BTA:" fmt, ## arg)
#define BTD(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_DEBUG) printk(KERN_INFO "BTD:" fmt, ## arg)
#define BTI(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_INFO) printk(KERN_INFO "BTI:" fmt, ## arg)
#define BTW(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_WARN) printk(KERN_INFO "BTW:" fmt, ## arg)
#define BTE(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_ERROR) printk(KERN_INFO "BTE:" fmt, ## arg)
#define BTF(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_FATAL) printk(KERN_INFO "BTF:" fmt, ## arg)

//struct sk_buff *h4_recv_buf(struct hci_dev *hdev, struct sk_buff *skb,
//                const unsigned char *buffer, int count,
//                const struct h4_recv_pkt *pkts, int pkts_count);
struct aml_coex_struct *aml_coex_struct_get(void);
struct hci_uart *aml_coex_hci_get(void);

#endif

