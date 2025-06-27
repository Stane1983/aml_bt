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
#include <linux/interrupt.h>
#include <linux/pm_wakeup.h>
#include <linux/pm_wakeirq.h>
#include <linux/amlogic/pm.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/version.h>
#include <linux/pci.h>

#include "common.h"
#include "w2_bt_entry.h"
#include "w2_pcie_bt.h"
#include "rc_list.h"

#define AML_BT_NOTE "stpbt"
#define AML_BT_FIRMWARE_NAME "w2_bt_fw_uart.bin"
#define AML_BT_CONFIG_NAME   "aml_bt.conf"

#define ICCM_SIZE   0x40000
#define DCCM_SIZE   0x20000
#define ICCM_ROM_SIZE 256*1024
#define DOWNLOAD_SIZE 4
#define ICCM_RAM_BASE           (0x000000)
#define DCCM_RAM_BASE           (0xd00000)
#define BT_ICCM_AHB_BASE        0x00300000
#define BT_DCCM_AHB_BASE        0x00400000

#ifndef BIT
#define BIT(_n)  (1 << (_n))
#endif

#define REG_DEV_RESET           0xf03058
#define REG_PMU_POWER_CFG       0xf03040
#define REG_RAM_PD_SHUTDWONW_SW 0xf03050
#define REG_FW_MODE             0xf000e0
#define REG_FW_PC               0x200034
#define REG_DF_A194             0xf06308

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

#define AML_ADDR_CPU      0
#define AML_ADDR_AON      1
#define AML_ADDR_MAC      2

struct aml_plat_pci {
    struct usb_device *usb_dev;
    struct auc_hif_ops *hif_ops;

    struct device *dev;
    struct aml_hif_sdio_ops *hif_sdio_ops;

    struct pci_dev *pci_dev;

    bool enabled;

    int (*enable)(void *aml_hw);
    int (*disable)(void *aml_hw);
    void (*deinit)(struct aml_plat_pci *aml_plat_pci);
    u8* (*get_address)(struct aml_plat_pci *aml_plat_pci, int addr_name,
                       unsigned int offset);
    void (*ack_irq)(struct aml_plat_pci *aml_plat_pci);
    int (*get_config_reg)(struct aml_plat_pci *aml_plat_pci, const u32 **list);

    u8 priv[0] __aligned(sizeof(void *));
};

extern struct aml_plat_pci *g_aml_plat_pci;
extern struct aml_pm_type g_wifi_pm;
extern uint32_t aml_pci_read_for_bt(int base, u32 offset);
extern void aml_pci_write_for_bt(u32 val, int base, u32 offset);
extern int aml_pci_insmod(void);
extern void aml_pci_rmmod(void);
extern unsigned char g_pci_driver_insmoded;
#ifdef CONFIG_AMLOGIC_GX_SUSPEND
extern unsigned int get_resume_method(void);
#endif

//extern unsigned char aml_wifi_detect_bt_status __attribute__((weak));

static w2_pcie_bt_t pcie_bt = {0};
//static void amlbt_shutdown_func(void);

static int amlbt_pcie_probe(struct platform_device *dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int amlbt_pcie_remove(struct platform_device *dev);
#else
static void amlbt_pcie_remove(struct platform_device *dev);
#endif
static int amlbt_pcie_suspend(struct platform_device *dev, pm_message_t state);
static int amlbt_pcie_resume(struct platform_device *dev);
static void amlbt_pcie_shutdown(struct platform_device *dev);

static int amlbt_pcie_fops_open(struct inode *inode, struct file *file);
static int amlbt_pcie_fops_close(struct inode *inode, struct file *file);
static long amlbt_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
#ifdef CONFIG_COMPAT
static long amlbt_compat_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
#endif
static void amlbt_w2p_resume_work(struct work_struct *work);

static void amlbt_wake_func(struct work_struct *work)
{
    w2_pcie_bt_t *p_pcie = &pcie_bt;
    unsigned int key = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A17);

    BTF("key %#x\n", key);
    if (key & BIT(5))   //bit 5 power key, bit 6 netfix
    {
        key &= ~BIT(5);
        aml_pci_write_for_bt(key, AML_ADDR_AON, RG_AON_A17);
        input_event(p_pcie->input_dev, EV_KEY, KEY_POWER, 1);
        input_sync(p_pcie->input_dev);
        input_event(p_pcie->input_dev, EV_KEY, KEY_POWER, 0);
        input_sync(p_pcie->input_dev);
        p_pcie->irq_handle = 0;
        BTF("%s input power key %#x\n", __func__, aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A17));
    }
    else if (key & BIT(6))
    {
        key &= ~BIT(6);
        aml_pci_write_for_bt(key, AML_ADDR_AON, RG_AON_A17);
        input_event(p_pcie->input_dev, EV_KEY, KEY_NETFLIX, 1);
        input_sync(p_pcie->input_dev);
        input_event(p_pcie->input_dev, EV_KEY, KEY_NETFLIX, 0);
        input_sync(p_pcie->input_dev);
        p_pcie->irq_handle = 0;
        BTF("%s input Netflix key %#x \n", __func__, aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A17));
    }
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    if (pcie_bt.irq_handle)
    {
        BTF("irq\n");
        schedule_work(&pcie_bt.wake_work);
    }
    return IRQ_HANDLED;
}

static int amlbt_register_interrupt_gpio(w2_pcie_bt_t *p_bt)
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
    INIT_WORK(&pcie_bt.wake_work, amlbt_wake_func);

    return 0;
}

static void amlbt_unregister_interrupt_gpio(w2_pcie_bt_t *p_bt)
{
    BTF("%s %d\n", __func__, p_bt->irq);

    if (p_bt->irq > 0)
    {
        cancel_work_sync(&pcie_bt.wake_work);
        free_irq(p_bt->irq, p_bt->dev_device);
        p_bt->irq = -1;
    }
}

static int amlbt_input_device_init(struct platform_device *pdev)
{
    int err;
    w2_pcie_bt_t *p_pcie = &pcie_bt;
    p_pcie->input_dev = input_allocate_device();

    BTF("%s \n", __func__);
    if (!p_pcie->input_dev)
    {
        BTF("input_allocate_device failed:");
        return -EINVAL;
    }
    set_bit(EV_KEY,  p_pcie->input_dev->evbit);
    set_bit(KEY_POWER, p_pcie->input_dev->keybit);
    set_bit(KEY_NETFLIX, p_pcie->input_dev->keybit);

    p_pcie->input_dev->name = INPUT_NAME;
    p_pcie->input_dev->phys = INPUT_PHYS;
    p_pcie->input_dev->dev.parent = &pdev->dev;
    p_pcie->input_dev->id.bustype = BUS_ISA;
    p_pcie->input_dev->id.vendor = 0x0001;
    p_pcie->input_dev->id.product = 0x0001;
    p_pcie->input_dev->id.version = 0x0100;
    p_pcie->input_dev->rep[REP_DELAY] = 0xffffffff;
    p_pcie->input_dev->rep[REP_PERIOD] = 0xffffffff;
    p_pcie->input_dev->keycodesize = sizeof(unsigned short);
    p_pcie->input_dev->keycodemax = 0x1ff;
    err = input_register_device(p_pcie->input_dev);
    if (err < 0)
    {
        pr_err("input_register_device failed: %d\n", err);
        input_free_device(p_pcie->input_dev);
        return -EINVAL;
    }

    return err;
}

static void amlbt_input_device_deinit(void)
{
    w2_pcie_bt_t *p_sdio = &pcie_bt;
    input_unregister_device(p_sdio->input_dev);
    p_sdio->input_dev = NULL;
}


static void amlbt_dev_release(struct device *dev)
{
    return;
}

static struct platform_device amlbt_pcie_device =
{
    .name    = "pcie_bt",
    .id      = -1,
    .dev     = {
        .release = &amlbt_dev_release,
    }
};

static struct platform_driver amlbt_pcie_driver =
{
    .probe = amlbt_pcie_probe,
    .remove = amlbt_pcie_remove,
    .suspend = amlbt_pcie_suspend,
    .resume = amlbt_pcie_resume,
    .shutdown = amlbt_pcie_shutdown,

    .driver = {
        .name = "pcie_bt",
        .owner = THIS_MODULE,
    },
};

static const struct file_operations amlbt_sdio_fops =
{
    .open       = amlbt_pcie_fops_open,
    .release    = amlbt_pcie_fops_close,
    .write      = NULL,
    .read      = NULL,
    .unlocked_ioctl = amlbt_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_compat_ioctl,
#endif
    .poll       = NULL,
    .fasync     = NULL
};


static void amlbt_aon_addr_bit_set(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;

    reg_value = aml_pci_read_for_bt(AML_ADDR_AON, addr);
    BTI("%#x: %#x\n", addr, reg_value);
    reg_value |= BIT(bit);
    aml_pci_write_for_bt(reg_value, AML_ADDR_AON, addr);
    BTI("%#x: %#x", addr, aml_pci_read_for_bt(AML_ADDR_AON, addr));
}

static void amlbt_aon_addr_bit_clr(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;

    reg_value = aml_pci_read_for_bt(AML_ADDR_AON, addr);
    BTI("%#x: %#x\n", addr, reg_value);
    reg_value &= ~BIT(bit);
    aml_pci_write_for_bt(reg_value, AML_ADDR_AON, addr);
    BTI("%#x: %#x", addr, aml_pci_read_for_bt(AML_ADDR_AON, addr));
}

static unsigned int amlbt_aon_addr_bit_get(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;
    unsigned int bit_value = 0;

    reg_value = aml_pci_read_for_bt(AML_ADDR_AON, addr);

    bit_value = (reg_value >> bit) & 0x1;
    BTI("get %#x bit%#d: %#x\n", addr, bit, bit_value);

    return bit_value;
}

static unsigned int amlbt_fw_pmu_sleep_get(void)
{
    unsigned int reg_value = 0;

    reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_BT_PMU_A15);
    BTI("%s PMU FSM %#x\n", __func__, (reg_value & 0xF));

    if (((reg_value & 0xF) == PMU_SLEEP_MODE) || ((reg_value & 0xF) == PMU_ACT_SLEEP))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

static int amlbt_wake_fw(void)
{
    unsigned int reg_value = 0;

    reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_BT_PMU_A16);
    reg_value &= ~BIT(0);
    reg_value |= BIT(1);
    aml_pci_write_for_bt(reg_value, AML_ADDR_AON, RG_BT_PMU_A16);
    reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_BT_PMU_A16);

    BTI("%s RG_BT_PMU_A16 %#x\n", __func__, reg_value);

    return 0;
}

static int amlbt_powersave_clear(void)
{
    // set bt open flag
    amlbt_aon_addr_bit_set(RG_AON_A24, 24);

    // clear shutdown bit
    amlbt_aon_addr_bit_clr(RG_AON_A24, 27);

    // clear suspend bit
    amlbt_aon_addr_bit_clr(RG_AON_A24, 26);

    return 0;
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

static int amlbt_load_conf(w2_pcie_bt_t *p_bt)
{
    int ret = 0;
    const struct firmware *fw_entry = NULL;
    char *data;
    size_t len, pos = 0;

    BTI("Firmware load:%s\n", AML_BT_CONFIG_NAME);
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
        }else if (parse_int_value(line_start, "BtIsolation", &p_bt->isolation)) {
            BTI("Parsed isolatvalue: %d\n", p_bt->isolation);
        }
        /* else {
            BTI("unknown key in configuration file: %s\n", line_start);
        }*/

        // Move to the next line
        pos = (line_end - data) + 1;
    }

    release_firmware(fw_entry);
    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
static int amlbt_bind_bus(void)
{
    w2_pcie_bt_t *p_pcie = &pcie_bt;

    if (g_aml_plat_pci->pci_dev == NULL)
    {
        BTE("aml_priv_to_func(7) is NULL");
        return -ENOMEM;
    }
    else
    {
        if (p_pcie->link == NULL)
        {
            p_pcie->link = device_link_add(&amlbt_pcie_device.dev, &g_aml_plat_pci->pci_dev->dev, DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
            if (p_pcie->link == NULL)
            {
                BTE("Failed to create device link");
                return -ENOMEM;
            }
            else
            {
                BTI("Success to create device link");
            }
        }
        else
        {
            BTI("p_pcie->link is ready %#x \n", p_pcie->link);
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

    w2_pcie_bt_t *p_pcie = &pcie_bt;

    if (p_pcie->link != NULL)
    {
        if (g_aml_plat_pci->pci_dev && device_is_registered(&g_aml_plat_pci->pci_dev->dev))
        {
            link = find_device_link(&amlbt_pcie_device.dev, &g_aml_plat_pci->pci_dev->dev);
            BTI("find_device_link : %#x", (unsigned long)link);
        }
        if (link != NULL && link == p_pcie->link)
        {
            device_link_del(p_pcie->link);
            BTI("Success to del device link");
        }
        p_pcie->link = NULL;
    }
}
#endif

static void amlbt_pcie_res_deinit(w2_pcie_bt_t *p_pcie)
{
    p_pcie->irq_handle = 0;
    p_pcie->firmware_start = 0;
    if (p_pcie->resume_wq != NULL)
    {
        flush_workqueue(p_pcie->resume_wq);
        destroy_workqueue(p_pcie->resume_wq);
        p_pcie->resume_wq = NULL;
    }
    BTF("%s finished \n", __func__);
}

static int amlbt_pcie_res_init(w2_pcie_bt_t *p_pcie)
{
    BTF("%s \n", __func__);

    p_pcie->irq_handle = 0;
    p_pcie->shutdown_value = 0;

    INIT_WORK(&p_pcie->resume_work, amlbt_w2p_resume_work);
    p_pcie->resume_wq = create_singlethread_workqueue("resume_wq");
    if (p_pcie->resume_wq == NULL)
    {
        BTE("%s create_resume_workqueue failed! \n", __func__);
    }
    return 0;
}

static void amlbt_w2p_resume_work(struct work_struct *work)
{
    int wait_cnt = 0;
    unsigned int reg = 0;
    int retry_cnt = 0;
    w2_pcie_bt_t *p_pcie = &pcie_bt;

    while (atomic_read(&g_wifi_pm.bus_suspend_cnt) != 0)
    {
        usleep_range(10000, 10000);
        wait_cnt++;
        if (wait_cnt > 100)
        {
            BTF("wifi resume failed!!!!, %#x\n", atomic_read(&g_wifi_pm.bus_suspend_cnt));
            return ;
        }
    }

    //forbid fw sleep
    amlbt_aon_addr_bit_set(RG_AON_A24, 25);
    usleep_range(1000, 1000);
    // wake bt fw
wake_retry:
    if (amlbt_fw_pmu_sleep_get() == TRUE)
    {
        usleep_range(1000, 1000);
        amlbt_wake_fw();
    }
    wait_cnt = 0;
    // wait bt fw wake done
     //fw will clear bit after wake done
    do
    {
        reg = amlbt_aon_addr_bit_get(RG_AON_A17, 29);
        usleep_range(10000, 10000);
        if (wait_cnt++ > 5)//wait 50ms
        {
            BTE("%s wake fw failed\n", __func__);
            if (retry_cnt++ < 3)
                goto wake_retry;
            break;
        }
    } while (reg);
#ifdef CONFIG_AMLOGIC_GX_SUSPEND
    BTI("get_resume_method %d, %d\n", get_resume_method(), BT_WAKEUP);
    if (get_resume_method() != BT_WAKEUP &&
        get_resume_method() != REMOTE_CUS_WAKEUP &&
            get_resume_method() != REMOTE_WAKEUP)
    {
        p_pcie->irq_handle = 1;
    }
#endif
}

static int amlbt_pcie_create_device(w2_pcie_bt_t *p_pcie)
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

    p_pcie->dev_major = MAJOR(dev);
    BTI("major number:%d\n", p_pcie->dev_major);
    cdev_init(&p_pcie->dev_cdev, &amlbt_sdio_fops);
    p_pcie->dev_cdev.owner = THIS_MODULE;

    cdevErr = cdev_add(&p_pcie->dev_cdev, dev, 1);
    if (cdevErr)
    {
        goto error;
    }

    BTI("driver(major %d) installed.\n", p_pcie->dev_major);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    p_pcie->dev_class = class_create(THIS_MODULE, AML_BT_NOTE);
#else
    p_pcie->dev_class = class_create(AML_BT_NOTE);
#endif

    if (IS_ERR(p_pcie->dev_class))
    {
        BTE("class create fail, error code(%ld)\n", PTR_ERR(p_pcie->dev_class));
        goto err1;
    }

    p_pcie->dev_device = device_create(p_pcie->dev_class, NULL, dev, NULL, AML_BT_NOTE);
    if (IS_ERR(p_pcie->dev_device))
    {
        BTE("device create fail, error code(%ld)\n", PTR_ERR(p_pcie->dev_device));
        goto err2;
    }

    BTI("%s: BT_major %d\n", __func__, p_pcie->dev_major);
    BTI("%s: dev id %d\n", __func__, dev);

    return 0;

err2:
    if (p_pcie->dev_class)
    {
        class_destroy(p_pcie->dev_class);
        p_pcie->dev_class = NULL;
    }

err1:

error:
    if (cdevErr == 0)
        cdev_del(&p_pcie->dev_cdev);

    if (ret == 0)
        unregister_chrdev_region(dev, 1);

    return -1;
}

static int amlbt_pcie_destroy_device(w2_pcie_bt_t *p_pcie)
{
    dev_t dev = MKDEV(p_pcie->dev_major, 0);

    BTF("%s dev id %d\n", __func__, dev);

    if (p_pcie->dev_device)
    {
        device_destroy(p_pcie->dev_class, dev);
        p_pcie->dev_device = NULL;
    }
    if (p_pcie->dev_class)
    {
        class_destroy(p_pcie->dev_class);
        p_pcie->dev_class = NULL;
    }
    cdev_del(&p_pcie->dev_cdev);

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
    w2_pcie_bt_t *p_pcie = &pcie_bt;

    //clear suspend bit
    //if (p_pcie->firmware_start)
    //{
        amlbt_aon_addr_bit_clr(RG_AON_A24, 26);
    //}

    BTF("%s \n", __func__);
}

static void amlbt_register_early_suspend(struct platform_device *dev)
{
    BTF("%s \n", __func__);

    pcie_bt.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    pcie_bt.early_suspend.suspend = bt_earlysuspend;
    pcie_bt.early_suspend.resume = bt_lateresume;
    pcie_bt.early_suspend.param = dev;
    register_early_suspend(&pcie_bt.early_suspend);
}

static void amlbt_unregister_early_suspend(struct platform_device *dev)
{
    BTF("%s \n", __func__);

    unregister_early_suspend(&pcie_bt.early_suspend);
}

static void amlbt_pcie_register(void)
{
    unsigned int alive = atomic_read(&g_wifi_pm.wifi_enable);

    if (alive)
    {
        BTF("wifi alive %#x\n", alive);
    }
    else if (!g_pci_driver_insmoded)
    {
        BTF("wifi not alive %#x\n", g_pci_driver_insmoded);
        aml_pci_insmod();
        usleep_range(100000, 100000);
    }
}

static void amlbt_pcie_unregister(void)
{
    unsigned int alive = atomic_read(&g_wifi_pm.wifi_enable);

    if (alive)
    {
        BTF("wifi alive %#x\n", alive);
    }
    else if (g_pci_driver_insmoded)
    {
        BTF("wifi not alive %#x\n", g_pci_driver_insmoded);
        aml_pci_rmmod();
    }
}

static int amlbt_pcie_probe(struct platform_device *dev)
{
    BTF("%s \n", __func__);
    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int amlbt_pcie_remove(struct platform_device *dev)
#else
static void amlbt_pcie_remove(struct platform_device *dev)
#endif
{
    BTF("%s \n", __func__);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
    return 0;
#endif
}

static int amlbt_pcie_suspend(struct platform_device *dev, pm_message_t state)
{
    w2_pcie_bt_t *p_pcie = &pcie_bt;

    BTF("%s \n", __func__);
    //if (p_pcie->firmware_start)
    //{
        if (atomic_read(&g_wifi_pm.bus_suspend_cnt) == 0)
        {
            amlbt_write_rclist_to_firmware();
            amlbt_aon_addr_bit_set(RG_AON_A24, 26);//set suspend bit
            amlbt_aon_addr_bit_clr(RG_AON_A24, 25);//allow fw sleep
        }
        else
        {
            BTF("%s failed bus_suspend_cnt %#x\n", __func__, atomic_read(&g_wifi_pm.bus_suspend_cnt));
        }
        pcie_bt.irq_handle = 0;
    //}
    BTI("%s \n", __func__);
    return 0;
}

static int amlbt_pcie_resume(struct platform_device *dev)
{
    int wait_cnt = 0;
    unsigned reg = 0;
    int retry_cnt = 0;
    w2_pcie_bt_t *p_pcie = &pcie_bt;

    BTF("%s \n", __func__);
    //if (p_pcie->firmware_start)
    //{
        //wait usb bus ready
        BTF("g_wifi_pm.bus_suspend_cnt:%#x\n", g_wifi_pm.bus_suspend_cnt);
        if (atomic_read(&g_wifi_pm.bus_suspend_cnt) != 0)
        {
            if (p_pcie->resume_wq == NULL)
            {
                BTE("%s create_resume_workqueue failed! \n", __func__);
            }
            else
            {
                //queue_work(p_pcie->resume_wq, &p_pcie->resume_work);
            }
        }
        else
        {
            //forbid fw sleep
            amlbt_aon_addr_bit_set(RG_AON_A24, 25);
            usleep_range(1000, 1000);
            // wake bt fw
wake_retry:
            if (amlbt_fw_pmu_sleep_get() == TRUE)
            {
                usleep_range(1000, 1000);
                amlbt_wake_fw();
            }
            wait_cnt = 0;
            // wait bt fw wake done
             //fw will clear bit after wake done
            do
            {
                reg = amlbt_aon_addr_bit_get(RG_AON_A17, 29);
                usleep_range(10000, 10000);
                if (wait_cnt++ > 5)//wait 50ms
                {
                    BTE("%s wake fw failed\n", __func__);
                    if (retry_cnt++ < 3)
                        goto wake_retry;
                    break;
                }
            } while (reg);
#ifdef CONFIG_AMLOGIC_GX_SUSPEND
            BTF("get_resume_method %d, %d\n", get_resume_method(), BT_WAKEUP);
            if (get_resume_method() != BT_WAKEUP &&
                    get_resume_method() != REMOTE_CUS_WAKEUP &&
                        get_resume_method() != REMOTE_WAKEUP)
            {
                p_pcie->irq_handle = 1;
            }
#endif
        }
    //}
    BTF("%s end\n", __func__);
    return 0;
}

static void amlbt_pcie_shutdown(struct platform_device *dev)
{
    BTF("%s \n", __func__);
    //if (pcie_bt.shutdown_value)
    //{
        amlbt_write_rclist_to_firmware();
        amlbt_aon_addr_bit_set(RG_AON_A24, 27);
    //}
}

static void amlbt_show_fw_debug_info(void)
{
    unsigned int value = 0;
    BTI("PMU 0x00f03040:%#x \n", aml_pci_read_for_bt(AML_ADDR_AON, REG_PMU_POWER_CFG));
    usleep_range(10000, 10000);
    value = aml_pci_read_for_bt(AML_ADDR_CPU, REG_FW_PC);
    value = (value >> 6);
    BTI("pc1 0x200034:%#x\n", value);
    usleep_range(10000, 10000);
    value = aml_pci_read_for_bt(AML_ADDR_CPU, REG_FW_PC);
    value = (value >> 6);
    BTI("pc2 0x200034:%#x\n", value);
    usleep_range(10000, 10000);
    value = aml_pci_read_for_bt(AML_ADDR_CPU, REG_FW_PC);
    value = (value >> 6);
    BTI("pc3 0x200034:%#x\n", value);
}

static int amlbt_pcie_download_firmware(w2_pcie_bt_t *p_bt)
{
    int ret = 0;
    unsigned int offset = 0;
    unsigned int remain_len = 0;
    unsigned int iccm_base_addr = BT_ICCM_AHB_BASE + ICCM_ROM_SIZE;
    unsigned int dccm_base_addr = BT_DCCM_AHB_BASE;
    unsigned int check_buf = 0;
    unsigned int val = 0;

    remain_len = ICCM_SIZE;

    //to do download bt fw
    BTI("amlbt_sdio_download_firmware:iccm size %#x, remain_len %#x\n", ICCM_SIZE, remain_len);

    BTI("iccm start [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        p_bt->iccm_buf[0], p_bt->iccm_buf[1], p_bt->iccm_buf[2], p_bt->iccm_buf[3],
        p_bt->iccm_buf[4], p_bt->iccm_buf[5], p_bt->iccm_buf[6], p_bt->iccm_buf[7]);
    BTI("dccm start [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        p_bt->dccm_buf[0], p_bt->dccm_buf[1], p_bt->dccm_buf[2], p_bt->dccm_buf[3],
        p_bt->dccm_buf[4], p_bt->dccm_buf[5], p_bt->dccm_buf[6], p_bt->dccm_buf[7]);

    while (offset < ICCM_SIZE)
    {
        val = ((p_bt->iccm_buf[offset+3]<<24)|(p_bt->iccm_buf[offset+2]<<16)|(p_bt->iccm_buf[offset+1]<<8)|p_bt->iccm_buf[offset]);
        aml_pci_write_for_bt(val, AML_ADDR_CPU, iccm_base_addr);
        check_buf = aml_pci_read_for_bt(AML_ADDR_CPU, iccm_base_addr);
        if (check_buf != val)
        {
            BTE("Firmware iccm check error! val:%#x, check_buf:%#x, offset %#x\n", val, check_buf, offset);
            ret = -1;
            goto error;
        }
        offset += DOWNLOAD_SIZE;
        remain_len -= DOWNLOAD_SIZE;
        iccm_base_addr += DOWNLOAD_SIZE;
    }

    BTI("Firmware iccm check pass, offset %#x\n", offset);
    offset = 0;
    remain_len = DCCM_SIZE;
    //to do download bt fw
    BTI("amlbt_sdio_download_firmware:dccm size %#x, remain_len %#x\n", DCCM_SIZE, remain_len);
    while (offset < DCCM_SIZE)
    {
        val = ((p_bt->dccm_buf[offset+3]<<24)|(p_bt->dccm_buf[offset+2]<<16)|(p_bt->dccm_buf[offset+1]<<8)|p_bt->dccm_buf[offset]);
        aml_pci_write_for_bt(val, AML_ADDR_CPU, dccm_base_addr);
        check_buf = aml_pci_read_for_bt(AML_ADDR_CPU, dccm_base_addr);
        if (check_buf != val)
        {
            BTE("Firmware dccm check error! val:%#x, check_buf:%#x, offset %#x\n", val, check_buf, offset);
            ret = -1;
            goto error;
        }
        offset += DOWNLOAD_SIZE;
        remain_len -= DOWNLOAD_SIZE;
        dccm_base_addr += DOWNLOAD_SIZE;
    }
    BTI("Firmware dccm check pass, offset %#x\n", offset);
error:
    return ret;
}


static int amlbt_load_firmware(w2_pcie_bt_t *p_bt)
{
    int ret = 0;
    unsigned int reg = 0;
    const struct firmware *fw_entry = NULL;
    unsigned int iccm_size;
    unsigned int dccm_size;

    BTI("Firmware load:%s\n", AML_BT_FIRMWARE_NAME);
    ret = request_firmware(&fw_entry, AML_BT_FIRMWARE_NAME, p_bt->dev_device);
    if (ret)
    {
        BTE("%s:%d Failed to load firmware: %d\n", __func__, __LINE__, ret);
        return ret;
    }
    BTI("Firmware [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        fw_entry->data[0], fw_entry->data[1], fw_entry->data[2], fw_entry->data[3],
        fw_entry->data[4], fw_entry->data[5], fw_entry->data[6], fw_entry->data[7]);
    iccm_size = ((fw_entry->data[3]<<24)|(fw_entry->data[2]<<16)|(fw_entry->data[1]<<8)|(fw_entry->data[0]));
    dccm_size = ((fw_entry->data[7]<<24)|(fw_entry->data[6]<<16)|(fw_entry->data[5]<<8)|(fw_entry->data[4]));

    BTI("Firmware loaded successfully, iccm_size: %#x, dccm_size:%#x\n", iccm_size - ICCM_ROM_SIZE, dccm_size);

    p_bt->iccm_buf = &fw_entry->data[ICCM_ROM_SIZE + 8];
    p_bt->dccm_buf = &fw_entry->data[iccm_size + 8];

    BTI("pmu %#x\n",aml_pci_read_for_bt(AML_ADDR_AON, 0xf02078));
    aml_pci_write_for_bt(((BIT_PHY|BIT_MAC|BIT_CPU)<<16)|(BIT_PHY|BIT_MAC|BIT_CPU), AML_ADDR_AON, REG_DEV_RESET);
    usleep_range(1000, 1000);
    aml_pci_write_for_bt((BIT_CPU<<16)|BIT_CPU, AML_ADDR_AON, REG_DEV_RESET);
    BTI("hold bt cpu\n");
    ret = amlbt_pcie_download_firmware(p_bt);
    release_firmware(fw_entry);
    if (ret != 0)
    {
        BTE("Download firmware failed!!\n");
        return ret;
    }
    reg = aml_pci_read_for_bt(AML_ADDR_AON, REG_DF_A194);
    reg &= 0xfffffffc;
    reg |= (p_bt->isolation & 0x3);
    aml_pci_write_for_bt(reg, AML_ADDR_AON, REG_DF_A194);

    reg = aml_pci_read_for_bt(AML_ADDR_AON, REG_FW_MODE);
    reg &= 0xfffffffc;
    reg |= (p_bt->fw_mode & 0x3);
    aml_pci_write_for_bt(reg, AML_ADDR_AON, REG_FW_MODE);

    reg = aml_pci_read_for_bt(AML_ADDR_AON, REG_PMU_POWER_CFG);
    reg &= 0xedffffff;
    reg |= ((p_bt->antenna << BIT_RF_NUM)|(p_bt->bt_sink << BT_SINK_MODE));
    aml_pci_write_for_bt(reg, AML_ADDR_AON, REG_PMU_POWER_CFG);

    reg = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A53);
    reg &= 0xffcf0000;
    reg |= ((p_bt->pin_mux << 20) | (p_bt->factory << 21));
    reg |= (((p_bt->edr_digit_gain & 0xff) << 8) | (p_bt->br_digit_gain & 0xff));
    aml_pci_write_for_bt(reg, AML_ADDR_AON, RG_AON_A53);

    reg = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A59);
    reg &= 0xfffffffc;
    reg |= (p_bt->fw_log & 0x3);
    aml_pci_write_for_bt(reg, AML_ADDR_AON, RG_AON_A59);
    aml_pci_write_for_bt(reg, AML_ADDR_AON, REG_DEV_RESET);
    BTI("start bt cpu ok!\n");
    usleep_range(50000, 50000);
    BTI("pc1:%#x\n", aml_pci_read_for_bt(AML_ADDR_CPU, REG_FW_PC));
    usleep_range(10000, 10000);
    BTI("pc2:%#x\n", aml_pci_read_for_bt(AML_ADDR_CPU, REG_FW_PC));
    usleep_range(10000, 10000);
    BTI("pc3:%#x\n", aml_pci_read_for_bt(AML_ADDR_CPU, REG_FW_PC));
    p_bt->iccm_buf = NULL;
    p_bt->dccm_buf = NULL;
    return 0;
}


static int amlbt_pcie_fops_open(struct inode *inode, struct file *file)
{
    int ret = 0;
    unsigned int reg = 0;
    w2_pcie_bt_t *p_pcie = &pcie_bt;

    BTI("%s, version:%s \n", __func__, AML_W2P_VERSION);

    file->private_data = &pcie_bt;
    amlbt_pcie_register();
    amlbt_load_conf(&pcie_bt);
    amlbt_pcie_res_init(p_pcie);
    //clear fw lowpower
    amlbt_powersave_clear();
    ret = amlbt_load_firmware(&pcie_bt);
    if (ret != 0)
    {
        BTI("amlbt_load_firmware failed!\n");
        amlbt_pcie_res_deinit(&pcie_bt);
        amlbt_pcie_unregister();
        return -EIO;
    }
    ret = aml_pci_read_for_bt(AML_ADDR_AON,REG_DF_A194);
    reg &= 0xfffffffc;
    reg |= (p_pcie->isolation & 0x3);
    aml_pci_write_for_bt(reg,AML_ADDR_AON,REG_DF_A194);

    pcie_bt.firmware_start = 1;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
    amlbt_bind_bus();
#endif
    //amlbt_register_interrupt_gpio(&pcie_bt);
    return nonseekable_open(inode, file);
}

static int amlbt_pcie_fops_close(struct inode *inode, struct file *file)
{
    w2_pcie_bt_t *p_pcie = &pcie_bt;
    BTI("%s, version:%s \n", __func__, AML_W2P_VERSION);

    amlbt_show_fw_debug_info();
    amlbt_aon_addr_bit_clr(RG_AON_A24, 24); //firmware currently not used

    amlbt_pcie_res_deinit(p_pcie);
    aml_pci_write_for_bt(0, AML_ADDR_AON, RG_AON_A15);
    //amlbt_unregister_interrupt_gpio(&pcie_bt);
    if (!p_pcie->shutdown_value)
    {
        amlbt_pcie_unregister();
    }

    return 0;
}

static long amlbt_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    switch (cmd)
    {
        case IOCTL_SET_BT_SHUTDOWN:
        {
            if (copy_from_user(&pcie_bt.shutdown_value, (unsigned char __user *)arg, sizeof(unsigned long)) != 0)
            {
                BTE("IOCTL_SET_BT_SHUTDOWN copy error\n");
                return -EFAULT;
            }
            BTI("IOCTL_SET_BT_SHUTDOWN %#x\n", pcie_bt.shutdown_value);
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

int amlbt_w2p_init(void)
{
    int ret = 0;
    struct platform_device *p_device = &amlbt_pcie_device;
    struct platform_driver *p_driver = &amlbt_pcie_driver;

    BTF("%s, version:%s", __func__, AML_W2P_VERSION);

    ret = platform_device_register(p_device);
    if (ret)
    {
        dev_err(&p_device->dev, "platform_device_register failed!\n");
        return ret;
    }

    ret = platform_driver_register(p_driver);
    if (ret)
    {
        dev_err(&p_device->dev, "platform_driver_register failed!\n");
        return ret;
    }
    amlbt_pcie_create_device(&pcie_bt);
    amlbt_register_early_suspend(p_device);
    amlbt_input_device_init(p_device);
    amlbt_rc_list_init(pcie_bt.dev_device, NULL, NULL, NULL, NULL);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
    amlbt_bind_bus();
#endif
    amlbt_register_interrupt_gpio(&pcie_bt);

    return ret;
}

void amlbt_w2p_exit(void)
{
    struct platform_device *p_device = &amlbt_pcie_device;
    struct platform_driver *p_driver = &amlbt_pcie_driver;

    BTF("%s, log level:%d \n", __func__, g_dbg_level);

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
    amlbt_unbind_bus();
#endif
    amlbt_rc_list_deinit(pcie_bt.dev_device);
    amlbt_input_device_deinit();
    amlbt_unregister_early_suspend(p_device);
    amlbt_pcie_destroy_device(&pcie_bt);
    amlbt_unregister_interrupt_gpio(&pcie_bt);

    platform_device_unregister(p_device);
    platform_driver_unregister(p_driver);
}

