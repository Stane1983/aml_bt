/******************************************************************************
*
*  Copyright (C) 2025 Amlogic Corporation
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
//#include <linux/types.h>
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

static struct aml_coex_struct *amlth;
static struct hci_uart *amlhci;

static dev_t bt_devid; /* bt char device number */
static struct cdev bt_char_dev; /* bt character device structure */
static struct class *bt_char_class; /* device class for usb char driver */
static unsigned char *acl_buf = NULL;
static unsigned char *rx_thread_buf = NULL;
static gdsl_fifo_t *rx_thread_buf_fifo = NULL;
static struct semaphore read_rx_sem;

static int evt_state = 0;
static DECLARE_WAIT_QUEUE_HEAD(poll_amlth_queue);
static int amlth_check_fw_rx(void *data);
static struct task_struct *check_fw_rx_stask = NULL;
static struct mutex fw_thread_fifo_mutex;
static int fw_data_flag = 0;


static void amlbt_dev_release(struct device *dev)
{
	BTD("%s \n", __func__);
    return;
}

static void amlbthread_buff_init(void)
{
    BTD("%s start\n", __func__);

    if (acl_buf == NULL)
    {
        acl_buf = kzalloc(THREAD_MAX_FRAME_SIZE, GFP_DMA|GFP_ATOMIC);
        if (acl_buf == NULL)
        {
            printk("%s acl_buf kzalloc failed! \n", __func__);
            return ;
        }
    }
    if (rx_thread_buf == NULL)
    {
        rx_thread_buf = kzalloc(THREAD_FIFO_SIZE, GFP_DMA|GFP_ATOMIC);
        if (rx_thread_buf == NULL)
        {
            printk("%s rx_thread_buf kzalloc failed! \n", __func__);
            kfree(acl_buf);
            return ;
        }
    }
    rx_thread_buf_fifo = gdsl_fifo_init(THREAD_FIFO_SIZE, rx_thread_buf);

	BTA("%s start read fw thread\n", __func__);
	mutex_init(&fw_thread_fifo_mutex);
    check_fw_rx_stask = kthread_run(amlth_check_fw_rx, NULL, "check_fw_rx_thread");
	if (!check_fw_rx_stask) {
        BTA("start read fw task fail\n");
        return;
    }
    sema_init(&read_rx_sem, 0);

    BTD("%s end\n", __func__);
}

static void amlthread_buff_deinit(void)
{
    kfree(acl_buf);
    kfree(rx_thread_buf);
    rx_thread_buf = NULL;
    acl_buf       = NULL;
    gdsl_fifo_deinit(rx_thread_buf_fifo);
}

static unsigned int threadchr_poll(struct file *file, poll_table *wait)
{
    int mask = 0;

    BTD("tps %#x\n", evt_state);
    poll_wait(file, &poll_amlth_queue, wait);

    if (fw_data_flag || evt_state)
    {
        mask = POLLIN | POLLRDNORM;
    }

   // mask = POLLIN | POLLRDNORM;

    return mask;

}

static void aml_thread_char_deinit(void)
{
    BTD("%s \n", __func__);
    device_destroy(bt_char_class, bt_devid);
    class_destroy(bt_char_class);
    cdev_del(&bt_char_dev);
    unregister_chrdev_region(bt_devid, 1);
}

static int aml_thread_char_open(struct inode *inode, struct file *file)
{
    BTI("11 %s\n", __func__);
    amlbthread_buff_init();
    amlhci = aml_coex_hci_get();
    amlth = aml_coex_struct_get();
    return nonseekable_open(inode, file);
}
static int aml_thread_char_close(struct inode *inode, struct file *file)
{
    BTI("aml_thread_char_close \n");
    amlthread_buff_deinit();
    return 0;
}

static int amlth_check_fw_rx(void *data)
{
    bool fw_type;

	BTD("%s start\n", __func__);
    while (!kthread_should_stop())
    {
        mutex_lock(&fw_thread_fifo_mutex);
        /*if (!evt_state)
        {
            fw_type = gdsl_fifo_used(rx_thread_buf_fifo);
        }*/
        ///M add
        fw_type = gdsl_fifo_used(rx_thread_buf_fifo);  //
        mutex_unlock(&fw_thread_fifo_mutex);

        if (fw_type && !evt_state)
        {
            BTD(" fw_type1 %#x\n", fw_type);
            fw_data_flag = 1;
            //fw_type = 0;
            BTD(" r:%#lx w:%#lx\n", (unsigned long)rx_thread_buf_fifo->r, (unsigned long)rx_thread_buf_fifo->w);
            wake_up_interruptible(&poll_amlth_queue);
            while (down_interruptible(&read_rx_sem) != 0);
        }
        else
        {
            usleep_range(3000, 3000);//change to 5000 -> 3000
        }
    }
    BTI("%s exit read fw rx thread\n", __func__);
    return 0;
}


static ssize_t aml_thread_char_read(struct file *file_p,
                                   char __user *buf_p,
                                   size_t count,
                                   loff_t *pos_p)
{
    static unsigned char host_read_buff[1024] = {0};
    unsigned int read_len = 0;

    BTD("%s:(%d, %ld)\n", __func__, evt_state, count);
    if (gdsl_fifo_used(rx_thread_buf_fifo) == 0)
    {
        BTE("FIFO empty, return directly.\n");
        return 0;
    }

    switch (evt_state)
    {
        case 0:					//read type
            evt_state = 1;
            memset(host_read_buff, 0, 1024);
            mutex_lock(&fw_thread_fifo_mutex);
            gdsl_fifo_get_data(rx_thread_buf_fifo, host_read_buff, count);
            mutex_unlock(&fw_thread_fifo_mutex);
            BTD("tp(%#x)\n", host_read_buff[0]);

            if (copy_to_user(buf_p, &host_read_buff[0], count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                return -EFAULT;
            }
        break;
        case 1:                 // read header
            evt_state = 2;
            mutex_lock(&fw_thread_fifo_mutex);
            gdsl_fifo_get_data(rx_thread_buf_fifo, &host_read_buff[1], 4);
            mutex_unlock(&fw_thread_fifo_mutex);

            BTD("thread header:%#x|%#x|%#x|%#x|\n", host_read_buff[1],
                   host_read_buff[2], host_read_buff[3], host_read_buff[4]);
            if (copy_to_user(buf_p, &host_read_buff[1], count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                return -EFAULT;
            }
        break;
        case 2:					//read payload
            mutex_lock(&fw_thread_fifo_mutex);
            read_len = 7 + ((host_read_buff[4] << 8) | (host_read_buff[3]));
            gdsl_fifo_get_data(rx_thread_buf_fifo, &host_read_buff[5], read_len - 5);
            mutex_unlock(&fw_thread_fifo_mutex);
            BTD("thread read len %d\n", read_len);

            BTD("thread payload:[%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x]\n", host_read_buff[5], host_read_buff[6],
                   host_read_buff[7], host_read_buff[8], host_read_buff[9], host_read_buff[10], host_read_buff[11], host_read_buff[12]);

            if (copy_to_user(buf_p, &host_read_buff[5], count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                return -EFAULT;
            }
            evt_state = 0;
            up(&read_rx_sem);
            fw_data_flag = 0;
            break;
        default:
            BTF("%s, evt_state error!!\n", __func__);
            break;
    }

    return count;
}

static ssize_t aml_thread_char_write(struct file *file_p,
                                     const char __user *buf_p,
                                     size_t count,
                                     loff_t *pos_p)
{
	unsigned int i = 0;
	static unsigned int w_type;
	struct sk_buff *skb;
	skb = bt_skb_alloc(1024, GFP_ATOMIC);

    //BT_ERR("%s \n", __func__);

    if (copy_from_user(acl_buf, buf_p, count))
    {
        BTE("amlbt_usb_write_word: Failed to get data from user space\n");
        return -EFAULT;
    }
    w_type = acl_buf[0];
    BTD("%s, %#x %#lx\n", __func__, w_type, count);
    if (w_type == 0x10)
    {
        for (; i < count; i++)
        {
            BTD(KERN_CONT "%#x|\n", acl_buf[i]);
        }
    }
    BTD("%s: skb->len = %d\n", __func__, skb->len);
    skb->len = count-1;
    hci_skb_pkt_type(skb) = acl_buf[0];

    BTD("%s: hci_skb_pkt_type(skb) = %#x\n", __func__, hci_skb_pkt_type(skb));

    //skb_reserve(skb,1);
    memcpy(&skb->data[0], &acl_buf[1], count-1);
    BTD("%s: Copied data to skb->data, first byte = %#x\n", __func__, skb->data[0]);

    BTD("amlth %d", skb->len);
    BTD("hci_skb_pkt_type(skb) %#x", hci_skb_pkt_type(skb));

    /* Prepend skb with frame type */
    memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);
    BTD("%s: Prepended skb, first byte = %#x\n", __func__, skb->data[0]);
    skb_queue_tail(&amlth->txq, skb);
    BTD("%s: Queued skb to txq, skb->len = %d\n",__func__, skb->len);
    hci_uart_tx_wakeup(amlhci);

    //kfree_skb(skb);
    BTA("skb %p amlhci %p", skb, amlhci);

    return count;
}

int aml_thread_recv_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
    int actlen = 0;
    BTD("thread_recv_frame skb len %d hci_skb_pkt_type(skb) %#x\n",skb->len, hci_skb_pkt_type(skb));

    mutex_lock(&fw_thread_fifo_mutex);
    actlen = gdsl_fifo_copy_data(rx_thread_buf_fifo, &hci_skb_pkt_type(skb), 1);
    actlen = gdsl_fifo_copy_data(rx_thread_buf_fifo, &skb->data[0], skb->len);
    mutex_unlock(&fw_thread_fifo_mutex);
    if (actlen <= 0)
    {
        BTE("actual len %d\n", actlen);
        kfree_skb(skb);
        return -EINVAL;
    }
    kfree_skb(skb);

    return 0;
}


static struct file_operations aml_thread_fops =
{
    .open = aml_thread_char_open,
    .release = aml_thread_char_close,
    .read = aml_thread_char_read,
    .write = aml_thread_char_write,
    .poll = threadchr_poll,
    //.unlocked_ioctl = threadchr_ioctl,
    //.compat_ioctl = threadchr_ioctl,
};

static int aml_thread_char_init(void)
{
    int res = 0;
    struct device *dev;

    BT_ERR("%s\n", __func__);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    bt_char_class = class_create(THIS_MODULE, AML_CHAR_THREAD_DEVICE_NAME);
#else
    bt_char_class = class_create(AML_CHAR_THREAD_DEVICE_NAME);
#endif
    if (IS_ERR(bt_char_class))
    {
        BT_ERR("%s:Failed to create bt char class\n", __func__);
        return PTR_ERR(bt_char_class);
    }

    res = alloc_chrdev_region(&bt_devid, 0, 1, AML_CHAR_THREAD_DEVICE_NAME);
    if (res < 0)
    {
        BT_ERR("%s:Failed to allocate bt char device\n", __func__);
        goto err_alloc;
    }

    dev = device_create(bt_char_class, NULL, bt_devid, NULL, AML_CHAR_THREAD_DEVICE_NAME);
    if (IS_ERR(dev))
    {
        BT_ERR("%s:Failed to create bt char device\n", __func__);
        res = PTR_ERR(dev);
        goto err_create;
    }

    cdev_init(&bt_char_dev, &aml_thread_fops);
    res = cdev_add(&bt_char_dev, bt_devid, 1);
    if (res < 0)
    {
        BT_ERR("%s:Failed to add bt char device\n", __func__);
        goto err_add;
    }
    //g_cdev = dev;
    BT_ERR("%s end", __func__);
    return 0;

err_add:
    device_destroy(bt_char_class, bt_devid);
err_create:
    unregister_chrdev_region(bt_devid, 1);
err_alloc:
    class_destroy(bt_char_class);
    return res;
}

static int aml_thread_probe(struct platform_device *dev)
{
    int err = 0;
    err = aml_thread_char_init();
    if (err < 0)
    {
        /* usb register will go on, even bt char register failed */
        BT_ERR("%s:Failed to register usb char device interfaces\n", __func__);
    }
    return err;
}
static int aml_thread_remove(struct platform_device *dev)
{
    BTA("%s\n", __func__);

    aml_thread_char_deinit();


    BTA("%s end\n", __func__);
    return 0;
}

static struct platform_device aml_platform_device =
{
    .name    = "aml_thread",
    .id      = -1,
    .dev     = {
        .release = &amlbt_dev_release,
    }
};

static struct platform_driver aml_platform_driver =
{
    .probe = aml_thread_probe,
    .remove = aml_thread_remove,
    .driver = {
        .name = "aml_thread",
        .owner = THIS_MODULE,
    },
};

int aml_thread_init(void)
{
    int ret = 0;

    ret = platform_device_register(&aml_platform_device);
    if (ret)
    {
        dev_err(&aml_platform_device.dev, "platform_device_register failed!\n");
        return ret;
    }
    BTD("%s 2 \n", __func__);
    ret = platform_driver_register(&aml_platform_driver);
    if (ret)
    {
        dev_err(&aml_platform_device.dev, "platform_driver_register failed!\n");
        return ret;
    }
	return 0;
}

int aml_thread_deinit(void)
{
    platform_driver_unregister(&aml_platform_driver);
    platform_device_unregister(&aml_platform_device);
	return 0;
}

