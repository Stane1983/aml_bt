/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/usb.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/io.h>
#include <linux/compat.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/amlogic/pm.h>
#include <linux/firmware.h>
#include <linux/mmc/sdio_func.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/pm_wakeup.h>
#include <linux/pm_wakeirq.h>
#include <linux/amlogic/pm.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include "common.h"
#include "w1u_bt_entry.h"
#include "w1u_sdio_bt.h"
#include "rc_list.h"

#define AML_BT_NOTE "stpbt"
#define ICCM_SIZE   0x10000
//#define ADD_SIZE    0x3c00
#define DCCM_SIZE   0x10000
#define ICCM_ROM_SIZE 256*1024
#define DOWNLOAD_SIZE 4
#define ICCM_RAM_BASE           (0x000000)
#define DCCM_RAM_BASE           (0xd00000)
#define BT_ICCM_AHB_BASE        0x00300000
#define BT_DCCM_AHB_BASE        0x00400000
#define BT_ADD_BASE             0x00a12800

#define AML_BT_CONFIG_NAME   "aml/aml_bt.conf"
#define AML_BT_FIRMWARE_NAME "aml/w1u_bt_fw_uart.bin"
#define W1U_ROM_START_CODE      0x0cc0006f

#ifndef BIT
#define BIT(_n)  (1 << (_n))
#endif

#define REG_DEV_RESET           0xf03058
#define REG_PMU_POWER_CFG       0xf03040
#define REG_RAM_PD_SHUTDWONW_SW 0xf03050
#define REG_FW_MODE             0xf000e0
#define REG_FW_PC               0x200034
#define RG_BG_A14               0xf0103c

#define BIT_PHY                 1
#define BIT_MAC                 (1 << 1)
#define BIT_CPU                 (1 << 2)
#define BIT_RF_NUM              28
#define BT_SINK_MODE            25

#define CHIP_BT_PMU_REG_BASE               (0xf03000)
#define RG_BT_PMU_A11                             (CHIP_BT_PMU_REG_BASE + 0x2c)
#define RG_BT_PMU_A12                             (CHIP_BT_PMU_REG_BASE + 0x30)
#define RG_BT_PMU_A13                             (CHIP_BT_PMU_REG_BASE + 0x34)
#define RG_BT_PMU_A14                             (CHIP_BT_PMU_REG_BASE + 0x38)
#define RG_BT_PMU_A15                             (CHIP_BT_PMU_REG_BASE + 0x3c)
#define RG_BT_PMU_A16                             (CHIP_BT_PMU_REG_BASE + 0x40)
#define RG_BT_PMU_A17                             (CHIP_BT_PMU_REG_BASE + 0x44)
#define RG_BT_PMU_A18                             (CHIP_BT_PMU_REG_BASE + 0x48)
#define RG_BT_PMU_A20                             (CHIP_BT_PMU_REG_BASE + 0x50)
#define RG_BT_PMU_A22                             (CHIP_BT_PMU_REG_BASE + 0x58)

#define CHIP_INTF_REG_BASE               (0xf00000)
#define RG_AON_A15                                (CHIP_INTF_REG_BASE + 0x3c)
#define RG_AON_A16                                (CHIP_INTF_REG_BASE + 0x40)
#define RG_AON_A17                                (CHIP_INTF_REG_BASE + 0x44)
#define RG_AON_A24                                (CHIP_INTF_REG_BASE + 0x60)
#define RG_AON_A30                                (CHIP_INTF_REG_BASE + 0x78)
#define RG_AON_A52                                (CHIP_INTF_REG_BASE + 0xd0)
#define RG_AON_A53                                (CHIP_INTF_REG_BASE + 0xd4)

/*********amlbt_tool*********/
#define RG_AON_A55                                (CHIP_INTF_REG_BASE + 0xdc)
#define RG_AON_A56                                (CHIP_INTF_REG_BASE + 0xe0)
#define RG_AON_A57                                (CHIP_INTF_REG_BASE + 0xe4)
#define RG_AON_A58                                (CHIP_INTF_REG_BASE + 0xe8)
#define RG_AON_A59                                (CHIP_INTF_REG_BASE + 0xec)
#define RG_AON_A60                                (CHIP_INTF_REG_BASE + 0xf0)
#define RG_AON_A61                                (CHIP_INTF_REG_BASE + 0xf4)
#define RG_AON_A62                                (CHIP_INTF_REG_BASE + 0xf8)

// pmu status
#define PMU_PWR_OFF       0x0
#define PMU_PWR_XOSC      0x1
#define PMU_XOSC_WAIT     0x2
#define PMU_XOSC_DPLL     0x3
#define PMU_DPLL_WAIT     0x4
#define PMU_DPLL_ACT      0x5
#define PMU_ACT_MODE      0x6
#define PMU_ACT_SLEEP     0x7
#define PMU_SLEEP_MODE    0x8
#define PMU_SLEEP_WAKE    0x9
#define PMU_WAKE_WAIT     0xa
#define PMU_WAKE_XOSC     0xb

extern void aml_sdio_exit(void);
extern int  aml_sdio_init(void);
extern struct aml_hif_sdio_ops g_hif_sdio_ops;
extern struct aml_pm_type g_wifi_pm;
extern struct sdio_func *aml_priv_to_func(int func_n);
extern unsigned char g_sdio_driver_insmoded;
extern unsigned char g_sdio_wifi_bt_alive;

//extern unsigned char aml_wifi_detect_bt_status __attribute__((weak));

static w1u_sdio_bt_t sdio_bt = {0};

static int amlbt_sdio_probe(struct platform_device *dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int amlbt_sdio_remove(struct platform_device *dev);
#else
static void amlbt_sdio_remove(struct platform_device *dev);
#endif
static int amlbt_sdio_suspend(struct platform_device *dev, pm_message_t state);
static int amlbt_sdio_resume(struct platform_device *dev);
static void amlbt_sdio_shutdown(struct platform_device *dev);

static int amlbt_sdio_fops_open(struct inode *inode, struct file *file);
static int amlbt_sdio_fops_close(struct inode *inode, struct file *file);
static ssize_t amlbt_sdio_char_write(struct file *file_p,
                                     const char __user *buf_p,
                                     size_t count,
                                     loff_t *pos_p);
static ssize_t amlbt_sdio_char_read(struct file *file_p,
                                   char __user *buf_p,
                                   size_t count,
                                   loff_t *pos_p);
static void amlbt_wake_func(struct work_struct *work);
static long amlbt_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
#ifdef CONFIG_COMPAT
static long amlbt_compat_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
#endif
static void amlbt_show_fw_debug_info(void);

static void amlbt_dev_release(struct device *dev)
{
    return;
}

static struct platform_device amlbt_sdio_device =
{
    .name    = "sdio_bt",
    .id      = -1,
    .dev     = {
        .release = &amlbt_dev_release,
    }
};

static struct platform_driver amlbt_sdio_driver =
{
    .probe = amlbt_sdio_probe,
    .remove = amlbt_sdio_remove,
    .suspend = amlbt_sdio_suspend,
    .resume = amlbt_sdio_resume,
    .shutdown = amlbt_sdio_shutdown,

    .driver = {
        .name = "sdio_bt",
        .owner = THIS_MODULE,
    },
};

static const struct file_operations amlbt_sdio_fops =
{
    .open       = amlbt_sdio_fops_open,
    .release    = amlbt_sdio_fops_close,
    .write      = amlbt_sdio_char_write,
    .read      = amlbt_sdio_char_read,
    .unlocked_ioctl = amlbt_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_compat_ioctl,
#endif
    .poll       = NULL,
    .fasync     = NULL
};

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    if (sdio_bt.irq_handle)
    {
        BTI("irq\n");
        schedule_work(&sdio_bt.wake_work);
    }
    return IRQ_HANDLED;
}

static int amlbt_register_interrupt_gpio(w1u_sdio_bt_t *p_bt)
{
    struct device_node *node;
    int gpio_num;
    int irq;
    int ret;

    BTF("%s %d\n", __func__, p_bt->irq);

    node = of_find_node_by_name(NULL, "aml_bt");
    if (!node)
    {
        BTE("Failed to find device node\n");
        return -1;
    }
    BTF("%s find node success!\n", __func__);
    gpio_num = of_get_named_gpio(node, "btwakeup-gpios", 0);
    if (gpio_num < 0)
    {
        BTE("Failed to get GPIO\n");
        return -1;
    }
    BTF("%s find gpio %d success!\n", __func__, gpio_num);
    irq = gpio_to_irq(gpio_num);
    if (irq < 0)
    {
        BTE("Failed to get IRQ for GPIO\n");
        gpio_free(gpio_num);
        return -1;
    }
    BTF("%s find irq %d success!\n", __func__, irq);

    ret = request_irq(irq, gpio_irq_handler, IRQF_TRIGGER_FALLING, "usb_gpio_irq", p_bt->dev_device);
    if (ret)
    {
        pr_err("Failed to request IRQ %d: %d\n", irq, ret);
        gpio_free(gpio_num);
        return -1;
    }
    BTF("%s request irq %d success!\n", __func__, irq);
    p_bt->irq = irq;
    INIT_WORK(&sdio_bt.wake_work, amlbt_wake_func);
    return 0;
}

static void amlbt_unregister_interrupt_gpio(w1u_sdio_bt_t *p_bt)
{
    BTF("%s %d\n", __func__, p_bt->irq);

    if (p_bt->irq > 0)
    {
        cancel_work_sync(&sdio_bt.wake_work);
        free_irq(p_bt->irq, p_bt->dev_device);
        p_bt->irq = -1;
    }
}

static int amlbt_input_device_init(struct platform_device *pdev)
{
    int err;
    w1u_sdio_bt_t *p_sdio = &sdio_bt;
    p_sdio->input_dev = input_allocate_device();

    BTF("%s %d\n", __func__);
    if (!p_sdio->input_dev)
    {
        BTF("input_allocate_device failed:");
        return -EINVAL;
    }
    set_bit(EV_KEY,  p_sdio->input_dev->evbit);
    set_bit(KEY_POWER, p_sdio->input_dev->keybit);
    set_bit(KEY_NETFLIX, p_sdio->input_dev->keybit);

    p_sdio->input_dev->name = INPUT_NAME;
    p_sdio->input_dev->phys = INPUT_PHYS;
    p_sdio->input_dev->dev.parent = &pdev->dev;
    p_sdio->input_dev->id.bustype = BUS_ISA;
    p_sdio->input_dev->id.vendor = 0x0001;
    p_sdio->input_dev->id.product = 0x0001;
    p_sdio->input_dev->id.version = 0x0100;
    p_sdio->input_dev->rep[REP_DELAY] = 0xffffffff;
    p_sdio->input_dev->rep[REP_PERIOD] = 0xffffffff;
    p_sdio->input_dev->keycodesize = sizeof(unsigned short);
    p_sdio->input_dev->keycodemax = 0x1ff;
    err = input_register_device(p_sdio->input_dev);
    if (err < 0)
    {
        pr_err("input_register_device failed: %d\n", err);
        input_free_device(p_sdio->input_dev);
        return -EINVAL;
    }

    return err;
}

static void amlbt_input_device_deinit(void)
{
    w1u_sdio_bt_t *p_sdio = &sdio_bt;
    if (p_sdio->input_dev != NULL)
    {
        input_unregister_device(p_sdio->input_dev);
        p_sdio->input_dev = NULL;
    }
}

static void amlbt_sdio_res_deinit(w1u_sdio_bt_t *p_sdio)
{
    BTF("%s \n", __func__);
    p_sdio->irq_handle = 0;
}

static int amlbt_sdio_res_init(w1u_sdio_bt_t *p_sdio)
{
    BTF("%s \n", __func__);
    p_sdio->irq_handle = 0;
    p_sdio->system = 0;
    p_sdio->shutdown_value = 0;

    return 0;
}

static void amlbt_sdio_write_word(unsigned int addr, unsigned int data)
{
    if (g_hif_sdio_ops.bt_hi_write_word == NULL)
    {
        BTE("amlbt_sdio_write_word NULL");
        return ;
    }
    g_hif_sdio_ops.bt_hi_write_word(addr, data);
}

static unsigned int amlbt_sdio_read_word(unsigned int addr)
{
    unsigned int value = 0;
    if (g_hif_sdio_ops.bt_hi_read_word == NULL)
    {
        BTE("amlbt_sdio_read_word NULL");
        return 0;
    }
    value = g_hif_sdio_ops.bt_hi_read_word(addr);
    return value;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
static int amlbt_bind_bus(void)
{
    w1u_sdio_bt_t *p_sdio = &sdio_bt;

    if (aml_priv_to_func(7) == NULL)
    {
        BTE("aml_priv_to_func(7) is NULL");
        return -ENOMEM;
    }
    else
    {
        if (p_sdio->link == NULL)
        {
            p_sdio->link = device_link_add(&amlbt_sdio_device.dev, &aml_priv_to_func(7)->dev, DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
            if (p_sdio->link == NULL)
            {
                BTE("Failed to create device link");
                return -ENOMEM;
            }
            else
            {
                BTF("Success to create device link");
            }
        }
        else
        {
            BTF("p_sdio->link is ready %#x \n", p_sdio->link);
        }
    }
    return 0;
}

static struct device_link *find_device_link(struct device *consumer, struct device *supplier)
{
    struct device_link *link;

    list_for_each_entry(link, &consumer->links.suppliers, c_node)
    {
        if (link->supplier == supplier)
        {
            return link;
        }
    }

    return NULL;
}

static void amlbt_unbind_bus(void)
{
    struct device_link *link = NULL;

    w1u_sdio_bt_t *p_sdio = &sdio_bt;

    if (p_sdio->link != NULL)
    {
        if (aml_priv_to_func(7) && device_is_registered(&aml_priv_to_func(7)->dev))
        {
            link = find_device_link(&amlbt_sdio_device.dev, &aml_priv_to_func(7)->dev);
            BTF("find_device_link : %#x", (unsigned long)link);
        }
        if (link != NULL && link == p_sdio->link)
        {
            device_link_del(p_sdio->link);
            BTF("Success to del device link");
        }
        p_sdio->link = NULL;
    }
}
#endif

static void amlbt_wake_func(struct work_struct *work)
{
    w1u_sdio_bt_t *p_sdio = &sdio_bt;
    unsigned int key = amlbt_sdio_read_word(RG_AON_A17);

    BTF("key %#x\n", key);
    if (key & BIT(5))   //bit 5 power key, bit 6 netflix
    {
        key &= ~BIT(5);
        amlbt_sdio_write_word(RG_AON_A17, key);
        input_event(p_sdio->input_dev, EV_KEY, KEY_POWER, 1);
        input_sync(p_sdio->input_dev);
        input_event(p_sdio->input_dev, EV_KEY, KEY_POWER, 0);
        input_sync(p_sdio->input_dev);
        p_sdio->irq_handle = 0;
        BTF("%s input power key\n", __func__);
    }
    else if (key & BIT(6))
    {
        key &= ~BIT(6);
        amlbt_sdio_write_word(RG_AON_A17, key);
        input_event(p_sdio->input_dev, EV_KEY, KEY_NETFLIX, 1);
        input_sync(p_sdio->input_dev);
        input_event(p_sdio->input_dev, EV_KEY, KEY_NETFLIX, 0);
        input_sync(p_sdio->input_dev);
        p_sdio->irq_handle = 0;
        BTF("%s input Netflix key\n", __func__);
    }
}

static void amlbt_aon_addr_bit_set(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;

    reg_value = amlbt_sdio_read_word(addr);
    BTI("%#x: %#x\n", addr, reg_value);
    reg_value |= BIT(bit);
    amlbt_sdio_write_word(addr, reg_value);
    BTI("%#x: %#x", addr, amlbt_sdio_read_word(addr));
}

static void amlbt_aon_addr_bit_clr(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;

    reg_value = amlbt_sdio_read_word(addr);
    BTI("%#x: %#x\n", addr, reg_value);
    reg_value &= ~BIT(bit);
    amlbt_sdio_write_word(addr, reg_value);
    BTI("%#x: %#x", addr, amlbt_sdio_read_word(addr));
}

/*
static unsigned int amlbt_aon_addr_bit_get(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;
    unsigned int bit_value = 0;

    reg_value = amlbt_sdio_read_word(addr);

    bit_value = (reg_value >> bit) & 0x1;
    BTI("get %#x bit%#d: %#x\n", addr, bit, bit_value);

    return bit_value;
}
*/

static int amlbt_sdio_create_device(w1u_sdio_bt_t *p_sdio)
{
    int ret = 0;
    int cdevErr = 0;
    dev_t dev = 0;

    BTF("%s \n", __func__);

    ret = alloc_chrdev_region(&dev, 0, 1, AML_BT_NOTE);
    if (ret)
    {
        BTE("fail to allocate chrdev\n");
        return ret;
    }

    p_sdio->dev_major = MAJOR(dev);
    BTI("major number:%d\n", p_sdio->dev_major);
    cdev_init(&p_sdio->dev_cdev, &amlbt_sdio_fops);
    p_sdio->dev_cdev.owner = THIS_MODULE;

    cdevErr = cdev_add(&p_sdio->dev_cdev, dev, 1);
    if (cdevErr)
    {
        goto error;
    }

    BTI("driver(major %d) installed.\n", p_sdio->dev_major);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    p_sdio->dev_class = class_create(THIS_MODULE, AML_BT_NOTE);
#else
    p_sdio->dev_class = class_create(AML_BT_NOTE);
#endif

    if (IS_ERR(p_sdio->dev_class))
    {
        BTE("class create fail, error code(%ld)\n", PTR_ERR(p_sdio->dev_class));
        goto err1;
    }

    p_sdio->dev_device = device_create(p_sdio->dev_class, NULL, dev, NULL, AML_BT_NOTE);
    if (IS_ERR(p_sdio->dev_device))
    {
        BTE("device create fail, error code(%ld)\n", PTR_ERR(p_sdio->dev_device));
        goto err2;
    }

    BTI("%s: BT_major %d\n", __func__, p_sdio->dev_major);
    BTI("%s: dev id %d\n", __func__, dev);

    return 0;

err2:
    if (p_sdio->dev_class)
    {
        class_destroy(p_sdio->dev_class);
        p_sdio->dev_class = NULL;
    }

err1:

error:
    if (cdevErr == 0)
        cdev_del(&p_sdio->dev_cdev);

    if (ret == 0)
        unregister_chrdev_region(dev, 1);

    return -1;
}

static int amlbt_sdio_destroy_device(w1u_sdio_bt_t *p_sdio)
{
    dev_t dev = MKDEV(p_sdio->dev_major, 0);

    BTF("%s dev id %d\n", __func__, dev);

    if (p_sdio->dev_device)
    {
        device_destroy(p_sdio->dev_class, dev);
        p_sdio->dev_device = NULL;
    }
    if (p_sdio->dev_class)
    {
        class_destroy(p_sdio->dev_class);
        p_sdio->dev_class = NULL;
    }
    cdev_del(&p_sdio->dev_cdev);

    unregister_chrdev_region(dev, 1);

    BTI("%s driver removed.\n", AML_BT_NOTE);
    return 0;
}

static void bt_earlysuspend(struct early_suspend *h)
{
    BTF("%s \n", __func__);
}

static void bt_lateresume(struct early_suspend *h)
{
    BTF("%s \n", __func__);

    sdio_bt.irq_handle = 0;
    //clear suspend bit
    amlbt_aon_addr_bit_clr(RG_AON_A52, 26);
    BTF("%#x\n", RG_AON_A52);
}

static void amlbt_register_early_suspend(struct platform_device *dev)
{
    BTF("%s \n", __func__);

    sdio_bt.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    sdio_bt.early_suspend.suspend = bt_earlysuspend;
    sdio_bt.early_suspend.resume = bt_lateresume;
    sdio_bt.early_suspend.param = dev;
    register_early_suspend(&sdio_bt.early_suspend);
}

static void amlbt_unregister_early_suspend(struct platform_device *dev)
{
    BTF("%s \n", __func__);

    unregister_early_suspend(&sdio_bt.early_suspend);
}

static void amlbt_sdio_register(void)
{
    unsigned int alive = g_sdio_wifi_bt_alive & BIT(1);

    if (alive)
    {
        BTF("wifi alive\n");
    }
    else if (!g_sdio_driver_insmoded)
    {
        BTF("wifi not alive\n");
        aml_sdio_init();
        usleep_range(150000, 150000);
    }
}

static void amlbt_sdio_unregister(void)
{
    unsigned int alive = g_sdio_wifi_bt_alive & BIT(1);

    if (alive)
    {
        BTF("wifi alive\n");
    }
    else if (g_sdio_driver_insmoded)
    {
        BTF("wifi not alive\n");
        //BTI("aml_bus_state_detect_deinit\n");
        //aml_bus_state_detect_deinit();
        BTF("remove wifi sdio device\n");
        aml_sdio_exit();
    }
}

static int amlbt_sdio_probe(struct platform_device *dev)
{
    BTF("%s \n", __func__);
    return 0;
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int amlbt_sdio_remove(struct platform_device *dev)
#else
static void amlbt_sdio_remove(struct platform_device *dev)
#endif
{
    BTF("%s \n", __func__);
    amlbt_show_fw_debug_info();
    amlbt_unregister_interrupt_gpio(&sdio_bt);
    amlbt_sdio_res_deinit(&sdio_bt);
    amlbt_sdio_write_word(RG_AON_A15, 0);
    amlbt_sdio_unregister();
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
    return 0;
#endif
}

static int amlbt_sdio_suspend(struct platform_device *dev, pm_message_t state)
{
    w1u_sdio_bt_t *p_sdio = &sdio_bt;

    BTF("%s \n", __func__);
    p_sdio->irq_handle = 0;
    amlbt_write_rclist_to_firmware();
    amlbt_aon_addr_bit_set(RG_AON_A52, 26);
    return 0;
}

static int amlbt_sdio_resume(struct platform_device *dev)
{
    w1u_sdio_bt_t *p_sdio = &sdio_bt;

    BTF("%s\n", __func__);
#ifdef  CONFIG_AMLOGIC_GX_SUSPEND
    BTF("get_resume_method %d, %d\n", get_resume_method(), BT_WAKEUP);
    if (get_resume_method() != BT_WAKEUP)
    {
        p_sdio->irq_handle = 1;
    }
#endif

    return 0;
}

static void amlbt_sdio_shutdown(struct platform_device *dev)
{
    BTF("%s \n", __func__);
    //if (sdio_bt.shutdown_value) check if shutdown or close
    //{
        amlbt_write_rclist_to_firmware();
    //}
    //amlbt_sdio_write_word(RG_BT_PMU_A16, 0);
}

static int parse_int_value(char *start, const char *key, int *value)
{
    size_t key_len = strlen(key);
    if (strncmp(start, key, key_len) == 0 && start[key_len] == '=')
    {
        *value = simple_strtol(start + key_len + 1, NULL, 10);
        return 1;
    }
    return 0;
}

static int amlbt_load_conf(w1u_sdio_bt_t *p_bt)
{
    int ret = 0;
    const struct firmware *fw_entry = NULL;
    char *data;
    size_t len, pos = 0;

    BTF("Firmware load:%s\n", AML_BT_CONFIG_NAME);
    ret = request_firmware(&fw_entry, AML_BT_CONFIG_NAME, p_bt->dev_device);
    if (ret)
    {
        BTE("%s:%d Failed to load config file: %d\n", __func__, __LINE__, ret);
        return -EINVAL;
    }

    if (!fw_entry || !fw_entry->data)
    {
        BTE("Failed to load conf or data is empty\n");
        release_firmware(fw_entry);
        return -EINVAL;
    }

    data = (char *)fw_entry->data;
    len = fw_entry->size;

    // Manual parsing loop
    while (pos < len) {
        char *line_start = data + pos;
        char *line_end = strchr(line_start, '\n');  // Find end of line
        if (!line_end) {
            line_end = data + len;  // If no newline, this is the last line
        }

        *line_end = '\0';  // Null-terminate the current line

        // Parse known keys
        if (parse_int_value(line_start, "BtAntenna", &p_bt->antenna)) {
            BTI("Parsed BtAntenna: %d\n", p_bt->antenna);
        } else if (parse_int_value(line_start, "FirmwareMode", &p_bt->fw_mode)) {
            BTI("Parsed FirmwareMode: %d\n", p_bt->fw_mode);
        } else if (parse_int_value(line_start, "BtSink", &p_bt->bt_sink)) {
            BTI("Parsed BtSink: %d\n", p_bt->bt_sink);
        } else if (parse_int_value(line_start, "ChangePinMux", &p_bt->pin_mux)) {
            BTI("Parsed ChangePinMux: %d\n", p_bt->pin_mux);
        } else if (parse_int_value(line_start, "BrDigitGain", &p_bt->br_digit_gain)) {
            BTI("Parsed BrDigitGain: %d\n", p_bt->br_digit_gain);
        } else if (parse_int_value(line_start, "EdrDigitGain", &p_bt->edr_digit_gain)) {
            BTI("Parsed EdrDigitGain: %d\n", p_bt->edr_digit_gain);
        } else if (parse_int_value(line_start, "Btfwlog", &p_bt->fw_log)) {
            BTI("Parsed Btfwlog: %d\n", p_bt->fw_log);
        } else if (parse_int_value(line_start, "Btlog", &p_bt->driver_log)) {
            BTI("Parsed Btlog: %d\n", p_bt->driver_log);
        } else if (parse_int_value(line_start, "Btfactory", &p_bt->factory)) {
            BTI("Parsed Btfactory: %d\n", p_bt->factory);
        } else if (parse_int_value(line_start, "Btsystem", &p_bt->system)) {
            BTI("Parsed Btsystem: %d\n", p_bt->system);
        }

        // Move to the next line
        pos = (line_end - data) + 1;
    }

    release_firmware(fw_entry);
    return 0;
}

static int amlbt_sdio_download_firmware(w1u_sdio_bt_t *p_sdio, unsigned int add_size)
{
    int ret = 0;
    unsigned int offset = 0;
    unsigned int remain_len = 0;
    //unsigned int iccm_base_addr = BT_ICCM_AHB_BASE + ICCM_ROM_SIZE;
    //unsigned int dccm_base_addr = BT_DCCM_AHB_BASE;
    unsigned int add_base_addr  = BT_ADD_BASE;
    //unsigned int check_buf = 0;
    //unsigned int write_buf = 0;
    unsigned char *p_add_buf = NULL;
    unsigned char *add_buf = NULL;
#if 0
    remain_len = ICCM_SIZE;

    //to do download bt fw
    BTI("amlbt_sdio_download_firmware:iccm size %#x, remain_len %#x\n", ICCM_SIZE, remain_len);

    BTI("iccm start [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        p_sdio->iccm_buf[0], p_sdio->iccm_buf[1], p_sdio->iccm_buf[2], p_sdio->iccm_buf[3],
        p_sdio->iccm_buf[4], p_sdio->iccm_buf[5], p_sdio->iccm_buf[6], p_sdio->iccm_buf[7]);
    BTI("add start [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        p_sdio->add_buf[0], p_sdio->add_buf[1], p_sdio->add_buf[2], p_sdio->add_buf[3],
        p_sdio->add_buf[4], p_sdio->add_buf[5], p_sdio->add_buf[6], p_sdio->add_buf[7]);
    BTI("dccm start [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        p_sdio->dccm_buf[0], p_sdio->dccm_buf[1], p_sdio->dccm_buf[2], p_sdio->dccm_buf[3],
        p_sdio->dccm_buf[4], p_sdio->dccm_buf[5], p_sdio->dccm_buf[6], p_sdio->dccm_buf[7]);

    while (offset < ICCM_SIZE)
    {
        write_buf = *(unsigned int *)(&p_sdio->iccm_buf[offset]);
        BTD("amlbt_sdio_download_firmware iccm2 offset %#x, write_len %#x, addr %#x\n", offset, DOWNLOAD_SIZE, iccm_base_addr);
        amlbt_sdio_write_word(iccm_base_addr, write_buf);
        check_buf = amlbt_sdio_read_word(iccm_base_addr);
        if (check_buf != write_buf)
        {
            BTE("Firmware iccm check error! offset %#x, [%#x,%#x]\n", offset, check_buf, write_buf);
            ret = -1;
            goto error;
        }
        offset += DOWNLOAD_SIZE;
        remain_len -= DOWNLOAD_SIZE;
        iccm_base_addr += DOWNLOAD_SIZE;
        BTD("amlbt_sdio_download_firmware iccm remain_len %#x\n", remain_len);
    }

    BTI("Firmware iccm check pass, offset %#x\n", offset);

    offset = 0;
    remain_len = DCCM_SIZE;
    //to do download bt fw
    BTI("amlbt_sdio_download_firmware:dccm size %#x, remain_len %#x\n", DCCM_SIZE, remain_len);
    while (offset < DCCM_SIZE)
    {
        write_buf = *(unsigned int *)(&p_sdio->dccm_buf[offset]);
        BTD("amlbt_sdio_download_firmware dccm2 offset %#x, write_len %#x, addr%#x\n", offset, DOWNLOAD_SIZE, dccm_base_addr);
        amlbt_sdio_write_word(dccm_base_addr, write_buf);
        check_buf = amlbt_sdio_read_word(dccm_base_addr);
        if (check_buf != write_buf)
        {
            BTE("Firmware dccm check error! offset %#x\n", offset);
            ret = -1;
            goto error;
        }
        offset += DOWNLOAD_SIZE;
        remain_len -= DOWNLOAD_SIZE;
        dccm_base_addr += DOWNLOAD_SIZE;
        BTD("amlbt_sdio_download_firmware dccm remain_len %#x \n", remain_len);
    }
    BTI("Firmware dccm check pass, offset %#x\n", offset);
#endif
    offset = 0;
    remain_len = add_size;
    //to do download bt fw
    BTF("amlbt_sdio_download_firmware:add size %#x, remain_len %#x\n", add_size, remain_len);

    if (g_hif_sdio_ops.bt_hi_write_sram != NULL && g_hif_sdio_ops.hi_self_define_domain_write32 != NULL
        && g_hif_sdio_ops.bt_hi_read_sram != NULL)
    {
        p_add_buf = (unsigned char *)p_sdio->add_buf;
        BTF("Firmware additional [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
            p_add_buf[0], p_add_buf[1], p_add_buf[2], p_add_buf[3],
            p_add_buf[4], p_add_buf[5], p_add_buf[6], p_add_buf[7],
            p_add_buf[8], p_add_buf[9], p_add_buf[10], p_add_buf[11]);
        g_hif_sdio_ops.hi_self_define_domain_write32(0x8150, add_base_addr & 0xfffe0000);
        g_hif_sdio_ops.bt_hi_write_sram(p_add_buf, (unsigned char *)(unsigned long)(add_base_addr & 0x1ffff), add_size);
        add_buf = kzalloc(add_size, GFP_KERNEL);
        if (add_buf)
        {
            g_hif_sdio_ops.bt_hi_read_sram(add_buf, (unsigned char *)(unsigned long)(add_base_addr & 0x1ffff), add_size);
            if (memcmp(add_buf, p_add_buf, add_size))
            {
                BTE("Firmware add check error!\n");
                goto error;
            }
            else
            {
                BTF("Firmware add check pass!\n");
            }
        }
        else
        {
            BTE("add_buf alloc error!\n");
            goto error;
        }
    }
    else
    {
        BTE("interface error![%#x,%#x,%#x]\n", (unsigned long)g_hif_sdio_ops.bt_hi_write_sram,
            (unsigned long)g_hif_sdio_ops.hi_self_define_domain_write32, (unsigned long)g_hif_sdio_ops.bt_hi_read_sram);
    }
error:
    if (add_buf)
    {
        kfree(add_buf);
        add_buf = NULL;
    }
    return ret;
}


static int amlbt_load_firmware(w1u_sdio_bt_t *p_bt)
{
    int ret = 0;
    unsigned int reg = 0;
    const struct firmware *fw_entry = NULL;
    unsigned int iccm_size;
    unsigned int dccm_size;
    unsigned int add_size;

    BTF("Firmware load:%s\n", AML_BT_FIRMWARE_NAME);
    ret = request_firmware(&fw_entry, AML_BT_FIRMWARE_NAME, p_bt->dev_device);
    if (ret)
    {
        BTE("%s:%d Failed to load firmware: %d\n", __func__, __LINE__, ret);
        return ret;
    }
    BTI("Firmware [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        fw_entry->data[0], fw_entry->data[1], fw_entry->data[2], fw_entry->data[3],
        fw_entry->data[4], fw_entry->data[5], fw_entry->data[6], fw_entry->data[7],
        fw_entry->data[8], fw_entry->data[9], fw_entry->data[10], fw_entry->data[11]);

    iccm_size = ((fw_entry->data[3]<<24)|(fw_entry->data[2]<<16)|(fw_entry->data[1]<<8)|(fw_entry->data[0]));
    add_size  = ((fw_entry->data[7]<<24)|(fw_entry->data[6]<<16)|(fw_entry->data[5]<<8)|(fw_entry->data[4]));
    dccm_size = ((fw_entry->data[11]<<24)|(fw_entry->data[10]<<16)|(fw_entry->data[9]<<8)|(fw_entry->data[8]));

    BTF("Firmware loaded successfully, iccm_size: %#x, dccm_size:%#x, add_size:%#x\n",
        iccm_size - ICCM_ROM_SIZE, dccm_size, add_size);

    if (dccm_size == W1U_ROM_START_CODE)
    {
        BTF("w1u sram code size 0!\n");
        release_firmware(fw_entry);
        p_bt->firmware_start = 1;
        p_bt->iccm_buf = NULL;
        p_bt->add_buf  = NULL;
        p_bt->dccm_buf = NULL;
        return 0;
    }

    p_bt->iccm_buf = &fw_entry->data[ICCM_ROM_SIZE + 12];
    p_bt->add_buf = &fw_entry->data[iccm_size + 12];
    p_bt->dccm_buf = &fw_entry->data[iccm_size + add_size + 12];

    ret = amlbt_sdio_download_firmware(p_bt, add_size);
    release_firmware(fw_entry);
    if (ret != 0)
    {
        BTE("Download firmware failed!!\n");
        return ret;
    }

    reg = amlbt_sdio_read_word(RG_AON_A53);
    reg &= 0xff7fffff;
    reg |= (p_bt->system << 23);
    amlbt_sdio_write_word(RG_AON_A53, reg);

    //reg = amlbt_sdio_read_word(REG_PMU_POWER_CFG);
    //reg |= ((p_bt->antenna << BIT_RF_NUM)|(p_bt->bt_sink << BT_SINK_MODE));
    //amlbt_sdio_write_word(REG_PMU_POWER_CFG, reg);
    //amlbt_sdio_write_word(REG_DEV_RESET, 0);
    p_bt->firmware_start = 1;
    p_bt->iccm_buf = NULL;
    p_bt->add_buf  = NULL;
    p_bt->dccm_buf = NULL;
    return 0;
}

static void amlbt_show_fw_debug_info(void)
{
    unsigned int value = 0;
    BTF("PMU 0x00f03040:%#x \n", amlbt_sdio_read_word(REG_PMU_POWER_CFG));
    usleep_range(10000, 10000);
    value = amlbt_sdio_read_word(REG_FW_PC);
    value = (value >> 6);
    BTF("pc1 0x200034:%#x\n", value);
    usleep_range(10000, 10000);
    value = amlbt_sdio_read_word(REG_FW_PC);
    value = (value >> 6);
    BTF("pc2 0x200034:%#x\n", value);
    usleep_range(10000, 10000);
    value = amlbt_sdio_read_word(REG_FW_PC);
    value = (value >> 6);
    BTF("pc3 0x200034:%#x\n", value);
}

static int amlbt_sdio_fops_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    BTF("%s, version:%s \n", __func__, AML_W1US_VERSION);
    file->private_data = &sdio_bt;
    amlbt_sdio_register();

    amlbt_aon_addr_bit_clr(RG_AON_A52, 26); //clear suspend bit

    if (amlbt_sdio_res_init(&sdio_bt) != 0)
    {
        return -ENOMEM;
    }
    amlbt_load_conf(&sdio_bt);
    ret = amlbt_load_firmware(&sdio_bt);
    if (ret != 0)
    {
        BTE("amlbt_load_firmware failed!\n");
        amlbt_sdio_res_deinit(&sdio_bt);
        goto exit;
    }
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
    amlbt_bind_bus();
#endif
    amlbt_register_interrupt_gpio(&sdio_bt);
exit:
    return nonseekable_open(inode, file);
}

static int amlbt_sdio_fops_close(struct inode *inode, struct file *file)
{
    BTF("%s BT closed\n", __func__);

    /*amlbt_show_fw_debug_info();
    amlbt_unregister_interrupt_gpio(&sdio_bt);
    amlbt_sdio_res_deinit(&sdio_bt);
    amlbt_sdio_write_word(RG_AON_A15, 0);
    if (!sdio_bt.shutdown_value)
    {
        amlbt_sdio_unregister();
    }*/

    return 0;
}

static ssize_t amlbt_sdio_char_write(struct file *file_p,
                                     const char __user *buf_p,
                                     size_t count,
                                     loff_t *pos_p)
{
    BTI("%s \n", __func__);
    return count;
}

static ssize_t amlbt_sdio_char_read(struct file *file_p,
                                   char __user *buf_p,
                                   size_t count,
                                   loff_t *pos_p)
{
    BTI("%s \n", __func__);
    return 0;
}

int amlbt_w1us_init(void)
{
    int ret = 0;
    struct platform_device *p_device = &amlbt_sdio_device;
    struct platform_driver *p_driver = &amlbt_sdio_driver;

    BTF("%s, version:%s", __func__, AML_W1US_VERSION);

    ret = platform_driver_register(p_driver);
    if (ret)
    {
        dev_err(&p_device->dev, "platform_driver_register failed!\n");
        return ret;
    }

    ret = platform_device_register(p_device);
    if (ret)
    {
        dev_err(&p_device->dev, "platform_device_register failed!\n");
        return ret;
    }

    amlbt_sdio_create_device(&sdio_bt);
    amlbt_register_early_suspend(p_device);
    amlbt_input_device_init(p_device);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
        amlbt_bind_bus();
#endif
    amlbt_rc_list_init(sdio_bt.dev_device, NULL, NULL, amlbt_sdio_write_word, amlbt_sdio_read_word);

    return ret;
}

static long amlbt_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    switch (cmd)
    {
        case IOCTL_SET_BT_SHUTDOWN:
        {
            if (copy_from_user(&sdio_bt.shutdown_value, (unsigned char __user *)arg, sizeof(unsigned long)) != 0)
            {
                BTE("IOCTL_SET_BT_SHUTDOWN copy error\n");
                return -EFAULT;
            }
            BTI("IOCTL_SET_BT_SHUTDOWN %#x\n", sdio_bt.shutdown_value);
        }
        break;
    }
    return 0;
}

#ifdef CONFIG_COMPAT
static long amlbt_compat_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    ret = amlbt_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
    return ret;
}
#endif

void amlbt_w1us_exit(void)
{
    struct platform_device *p_device = &amlbt_sdio_device;
    struct platform_driver *p_driver = &amlbt_sdio_driver;

    BTF("%s, log level:%d \n", __func__, g_dbg_level);

    amlbt_rc_list_deinit(sdio_bt.dev_device);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
        amlbt_unbind_bus();
#endif
    amlbt_input_device_deinit();
    amlbt_unregister_early_suspend(p_device);
    amlbt_sdio_destroy_device(&sdio_bt);

    platform_device_unregister(p_device);
    platform_driver_unregister(p_driver);
}
