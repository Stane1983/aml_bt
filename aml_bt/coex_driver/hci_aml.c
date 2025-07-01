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

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/poll.h>
#include <linux/version.h>

#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/ioctl.h>
#include <linux/skbuff.h>
#include <asm/unaligned.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

#include "hci_uart.h"
#include "hci_aml.h"
#include "hci_aml_thread.h"
#include "hci_aml_zigbee.h"

#define AML_COEX_VERSION    "2025-0721,1430 LEA"

struct aml_coex_struct *aml_coex;
struct hci_uart *amlhci;

int g_dbg_level = LOG_LEVEL_INFO; //Default printing level

static struct dentry *debug_dir;

static ssize_t debug_level_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) {
    char temp[10];
    int len = snprintf(temp, sizeof(temp), "%d\n", g_dbg_level);
    return simple_read_from_buffer(buf, count, ppos, temp, len);
}

static ssize_t debug_level_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) {
    char temp[10];
    int ret, val;

    if (count > sizeof(temp) - 1)
        return -EINVAL;

    if (copy_from_user(temp, buf, count))
        return -EFAULT;

    temp[count] = '\0';

    ret = kstrtoint(temp, 10, &val);
    if (ret) {
        pr_err("Invalid input for debug_level\n");
        return ret;
    }

    g_dbg_level = val;
    pr_info("Debug level set to %d\n", g_dbg_level);

    return count;
}

static const struct file_operations debug_fops = {
    .read = debug_level_read,
    .write = debug_level_write,
};

static int __init aml_coex_debugfs_init(void) {
    debug_dir = debugfs_create_dir("aml_btz", NULL);  // 创建目录 /sys/kernel/debug/aml_btz
    if (!debug_dir)
        return -ENOMEM;

    debugfs_create_file("aml_btz_dbg_lvl", 0644, debug_dir, NULL, &debug_fops);  // 创建 /sys/kernel/debug/my_driver/aml_btz_dbg_lvl
    return 0;
}

static void aml_coex_debugfs_deinit(void)
{
    if (debug_dir != NULL)
    {
        debugfs_remove_recursive(debug_dir);
        debug_dir = NULL;
    }
}

int aml_coex_recv_frame(struct hci_dev *hdev, struct sk_buff *skb);

static const struct h4_recv_pkt aml_coex_recv_pkts[] = {
    { H4_RECV_ACL,   .recv = hci_recv_frame },
    { H4_RECV_SCO,   .recv = hci_recv_frame },
    { H4_RECV_EVENT, .recv = hci_recv_frame },
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    { H4_RECV_ISO,    .recv = hci_recv_frame },
#endif
    { H4_RECV_15P4,  .recv = aml_coex_recv_frame },
};

struct aml_coex_struct *aml_coex_struct_get(void)
{
    return aml_coex;
}

struct hci_uart *aml_coex_hci_get(void)
{
    return amlhci;
}

struct sk_buff *aml_h4_recv_buf(struct hci_dev *hdev, struct sk_buff *skb,
                const unsigned char *buffer, int count,
                const struct h4_recv_pkt *pkts, int pkts_count)
{
    struct hci_uart *hu = hci_get_drvdata(hdev);
    u8 alignment = hu->alignment ? hu->alignment : 1;

    /* Check for error from previous call */
    if (IS_ERR(skb))
    {
        BTF("%s skb err\n", __func__);
        skb = NULL;
    }
    BTD("%s:count:%d\n", __func__, count);
    BTD("%s:buffer:[%#x,%#x,%#x,%#x]\n", __func__, buffer[0], buffer[1], buffer[2], buffer[3]);
    while (count) {
        int i, len;

        /* remove padding bytes from buffer */
        for (; hu->padding && count > 0; hu->padding--) {
            count--;
            buffer++;
        }
        if (!count)
            break;

        if (!skb) {
            for (i = 0; i < pkts_count; i++) {
                if (buffer[0] != (&pkts[i])->type)
                    continue;

                skb = bt_skb_alloc((&pkts[i])->maxlen,
                           GFP_ATOMIC);
                if (!skb)
                {
                BTF("%s bt_skb_alloc failed\n", __func__);
                    return ERR_PTR(-ENOMEM);
                }
                hci_skb_pkt_type(skb) = (&pkts[i])->type;
                hci_skb_expect(skb) = (&pkts[i])->hlen;
                break;
            }
            BTD("aml_h4_recv_buf hci_skb_pkt_type(skb) %#x\n", hci_skb_pkt_type(skb));

            /* Check for invalid packet type */
            if (!skb)
            {
                return ERR_PTR(-EILSEQ);
                BTF("%s invalid packet type: %#x\n", __func__, buffer[0]);
            }
            count -= 1;
            buffer += 1;
        }

        BTD("buffer:[%#x,%#x,%#x,%#x]\n", buffer[0], buffer[1], buffer[2], buffer[3]);
        len = min_t(uint, hci_skb_expect(skb) - skb->len, count);
        skb_put_data(skb, buffer, len);


        count -= len;
        buffer += len;
        BTD("aml_h4_recv_buf len %#x skb->len %#x hci_skb_expect(skb) %#x\n", len, skb->len, hci_skb_expect(skb));

        /* Check for partial packet */
        if (skb->len < hci_skb_expect(skb))
            continue;

        for (i = 0; i < pkts_count; i++) {
            if (hci_skb_pkt_type(skb) == (&pkts[i])->type)
                break;
        }

        if (i >= pkts_count) {
        BTF("%s unknown packet type in completion: %#x\n", __func__, hci_skb_pkt_type(skb));
            kfree_skb(skb);
            return ERR_PTR(-EILSEQ);
        }

        if (skb->len == (&pkts[i])->hlen) {
            u16 dlen;

            switch ((&pkts[i])->lsize) {
            case 0:
                /* No variable data length */
                dlen = 0;
                break;
            case 1:
                /* Single octet variable length */
                dlen = skb->data[(&pkts[i])->loff];
                hci_skb_expect(skb) += dlen;
                BTD("1 dlen %#x\n", dlen);

                if (skb_tailroom(skb) < dlen) {
                BTF("%s lsize=1, tailroom insufficient\n", __func__);
                    kfree_skb(skb);
                    return ERR_PTR(-EMSGSIZE);
                }
                break;
            case 2:
                /* Double octet variable length */
                dlen = get_unaligned_le16(skb->data +
                              (&pkts[i])->loff);
                hci_skb_expect(skb) += dlen;
                BTD("2 %#x\n", dlen);

                if (skb_tailroom(skb) < dlen) {
                BTF("%s lsize=2, tailroom insufficient\n", __func__);
                    kfree_skb(skb);
                    return ERR_PTR(-EMSGSIZE);
                }
                break;
            case 3:
                /* Double octet variable length */
                dlen = get_unaligned_le16(skb->data +
                              (&pkts[i])->loff);
                dlen += 2;
                hci_skb_expect(skb) += dlen;
                BTD("3 %#x\n", dlen);

                if (skb_tailroom(skb) < dlen) {
                BTF("%s lsize=3, tailroom insufficient\n", __func__);
                    kfree_skb(skb);
                    return ERR_PTR(-EMSGSIZE);
                }
                break;
            default:
                /* Unsupported variable length */
                BTF("%s unsupported lsize: %d\n", __func__, (&pkts[i])->lsize);
                kfree_skb(skb);
                return ERR_PTR(-EILSEQ);
            }

            if (!dlen) {
                hu->padding = (skb->len + 1) % alignment;
                hu->padding = (alignment - hu->padding) % alignment;

                /* No more data, complete frame */
                (&pkts[i])->recv(hdev, skb);
                skb = NULL;
                BTD("i %#x\n", i);
            }
        } else {
            hu->padding = (skb->len + 1) % alignment;
            hu->padding = (alignment - hu->padding) % alignment;

            /* Complete frame */
            (&pkts[i])->recv(hdev, skb);
            skb = NULL;
            BTD("J %#x\n", i);
        }
    }

   return skb;
}

static int aml_coex_open(struct hci_uart *hu)
{
    BTI(" open hu %p\n", hu);
    aml_coex = kzalloc(sizeof(struct aml_coex_struct), GFP_KERNEL);
    if (!aml_coex)
        return -ENOMEM;

    skb_queue_head_init(&aml_coex->txq);
    mutex_init(&aml_coex->txq_mutex);
    skb_queue_head_init(&aml_coex->rxq);
    skb_queue_head_init(&aml_coex->thread_rxq);
    skb_queue_head_init(&aml_coex->zigbee_rxq);

    amlhci = hu;
    hu->priv = aml_coex;
    return 0;
}

/* Close protocol */
static int aml_coex_close(struct hci_uart *hu)
{

    struct aml_coex_struct *p_aml_coex = hu->priv;
    BTI("close hu %p\n", hu);

    if (!p_aml_coex) {
      BTE("aml_coex already freed!\n");
      return 0;
    }

    skb_queue_purge(&p_aml_coex->txq);
    skb_queue_purge(&p_aml_coex->rxq);
    skb_queue_purge(&p_aml_coex->thread_rxq);
    skb_queue_purge(&p_aml_coex->zigbee_rxq);
    mutex_destroy(&p_aml_coex->txq_mutex);

    if (p_aml_coex->rx_skb && !IS_ERR(p_aml_coex->rx_skb)) {
      kfree_skb(p_aml_coex->rx_skb);
    }

    hu->priv = NULL;
    kfree(p_aml_coex);

    amlhci = NULL;

    return 0;

}

/* Recv data */
static int aml_coex_recv(struct hci_uart *hu, const void *data, int count)
{
    struct aml_coex_struct *p_aml_coex = hu->priv;

    BTD("aml_coex_recv: hu=%p, hu->hdev=%p\n", hu, hu ? hu->hdev : NULL);
    if (!hu->hdev)
    {
      BTE("aml_coex_recv: hu->hdev is NULL, device not ready!\n");
      return -ENODEV;
    }
    BTD("aml_coex_recv  %p count %d\n", p_aml_coex, count);
    if (p_aml_coex == NULL)
    {
        BTE("p_aml_coex NULL!\n");
        return -EUNATCH;
    }

    if (!test_bit(HCI_UART_REGISTERED, &hu->flags))
    {
        BTE("aml_coex_recv HCI_UART_REGISTERED flag err!\n");
        return -EUNATCH;
    }
    p_aml_coex->rx_skb = aml_h4_recv_buf(hu->hdev, p_aml_coex->rx_skb, data, count,
                 aml_coex_recv_pkts, ARRAY_SIZE(aml_coex_recv_pkts));
    BTD("p_aml_coex->rx_skb  %p\n", p_aml_coex->rx_skb);

    if (IS_ERR(p_aml_coex->rx_skb)) {
        int err = PTR_ERR(p_aml_coex->rx_skb);
        BTE("aml_coex_recv p_aml_coex->rx_skb err!\n");
        bt_dev_err(hu->hdev, "Frame reassembly failed (%d)", err);
        p_aml_coex->rx_skb = NULL;
        return err;
    }

    return count;
}


int aml_coex_recv_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
    int i = 0;
    BTD("aml_coex_recv_frame skb len %d hci_skb_pkt_type(skb) %#x\n",skb->len, hci_skb_pkt_type(skb));
    for (; i < 10; i++)
    {
        BTD(" %#x|\n", skb->data[i]);
    }
    if (hci_skb_pkt_type(skb) == HCI_15P4_PKT)
    {
        if (skb->data[0] == HCI_AML_THREAD_TYPE)
        {
            aml_thread_recv_frame(hdev, skb);
        }
        else if (skb->data[0] == HCI_AML_ZIGBEE_TYPE)
        {
            aml_zigbee_recv_frame(hdev, skb);
        }
    }

    return 0;
}

/* Enqueue frame for transmittion (padding, crc, etc) */
static int aml_coex_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
    struct aml_coex_struct *p_aml_coex = hu->priv;

    BTD("p_aml_coex %p skb %p hci_skb_pkt_type(skb) %#x", p_aml_coex, skb, hci_skb_pkt_type(skb));

    /* Prepend skb with frame type */
    memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);
    mutex_lock(&p_aml_coex->txq_mutex);
    skb_queue_tail(&p_aml_coex->txq, skb);
    mutex_unlock(&p_aml_coex->txq_mutex);

    return 0;
}

static struct sk_buff *aml_coex_dequeue(struct hci_uart *hu)
{
    struct aml_coex_struct *p_aml_coex = (struct aml_coex_struct *)hu->priv;

    struct sk_buff *skb;

    mutex_lock(&p_aml_coex->txq_mutex);
    skb = skb_dequeue(&p_aml_coex->txq);
    mutex_unlock(&p_aml_coex->txq_mutex);

    return skb;

}

/* Flush protocol data */
static int aml_coex_flush(struct hci_uart *hu)
{
    struct aml_coex_struct *p_aml_coex = hu->priv;

    BTD(" flush hu %p", hu);

    mutex_lock(&p_aml_coex->txq_mutex);
    skb_queue_purge(&p_aml_coex->txq);
    mutex_unlock(&p_aml_coex->txq_mutex);
    skb_queue_purge(&p_aml_coex->rxq);
    skb_queue_purge(&p_aml_coex->thread_rxq);
    skb_queue_purge(&p_aml_coex->zigbee_rxq);

    return 0;
}


static const struct hci_uart_proto aml_coex_proto = {
    .id	    = HCI_UART_LL,
    .name	    = "AML",
    .open	    = aml_coex_open,
    .close	    = aml_coex_close,
    .recv	    = aml_coex_recv,
    .enqueue	= aml_coex_enqueue,
    .dequeue	= aml_coex_dequeue,
    .flush	    = aml_coex_flush,
};

static int aml_init(void)
{
    BTI("%s, %s", __func__, AML_COEX_VERSION);
    aml_thread_init();
    aml_zigbee_init();
    aml_coex_debugfs_init();
    return hci_uart_register_proto(&aml_coex_proto);
}

static void aml_deinit(void)
{
    BTI("%s", __func__);
    aml_thread_deinit();
    aml_zigbee_deinit();
    aml_coex_debugfs_deinit();
    hci_uart_unregister_proto(&aml_coex_proto);
}

module_init(aml_init);
module_exit(aml_deinit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Basic Character Device Driver");

