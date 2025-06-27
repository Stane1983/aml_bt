/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#ifndef __COMMON_H__
#define __COMMON_H__

struct _gdsl_fifo_t;
typedef struct _gdsl_fifo_t gdsl_fifo_t;

typedef enum
{
    GDSL_ERR_SUCCESS,
    GDSL_ERR_NULL_POINTER,
    GDSL_ERR_NOT_FULL,
    GDSL_ERR_FULL,
    GDSL_ERR_NOT_EMPTY,
    GDSL_ERR_EMPTY,
    GDSL_ERR_SPACE_INVALID,
    GDSL_ERR_SPACE_VALID,
} gdsl_err_t;

struct _gdsl_fifo_t
{
    unsigned char *r;
    unsigned char *w;
    unsigned char *base_addr;
    unsigned int size;
};

typedef struct
{
    unsigned char *tx_q_addr;
    unsigned int *tx_q_status_addr;
    unsigned int *tx_q_prio_addr;
    unsigned int *tx_q_dev_index_addr;

    unsigned int tx_q_status;
    unsigned int tx_q_prio;
    unsigned int tx_q_dev_index;
} gdsl_tx_q_t;

enum
{
    LOG_LEVEL_FATAL,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_POINT,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_ALL,
};

enum SDIO_STD_FUNNUM {
    SDIO_FUNC0=0,
    SDIO_FUNC1,
    SDIO_FUNC2,
    SDIO_FUNC3,
    SDIO_FUNC4,
    SDIO_FUNC5,
    SDIO_FUNC6,
    SDIO_FUNC7,
};

#define FUNCNUM_SDIO_LAST SDIO_FUNC7
#define SDIO_FUNCNUM_MAX (FUNCNUM_SDIO_LAST+1)
#define OS_LOCK spinlock_t

typedef unsigned long SYS_TYPE;
/* max page num while process a sdio write operation*/
#define SG_PAGE_MAX 80
#define MAXSG_SIZE        (SG_PAGE_MAX * 2)
#define MAX_SG_ENTRIES    (MAXSG_SIZE+2)
#define SG_NUM_MAX 16
#define SG_WRITE 1 //MMC_DATA_WRITE
#define SG_FRAME_MAX (1 * SG_NUM_MAX)
#ifndef FALSE
#define FALSE  0
#endif

#ifndef TRUE
#define TRUE   (!FALSE)
#endif

//bt should synchronous editing
struct aml_bus_state_detect {
  unsigned char bus_err;
  unsigned char usb_disconnect;
  unsigned char is_drv_load_finished;
  unsigned char bus_reset_ongoing;
  unsigned char is_load_by_timer;
  unsigned char is_recy_ongoing;
  struct timer_list timer;
  struct work_struct detect_work;
  int (*insmod_drv)(void);
  unsigned char usb_suspend;
};

struct tx_trb_info_ex
{
    /* The number of pages needed for a single transfer */
    unsigned int packet_num;
    /* Actual size used for each page */
    unsigned short buffer_size[128];
};

struct amlw_hif_scatter_item {
    struct sk_buff *skbbuf;
    int len;
    int page_num;
    void *packet;
};

struct amlw_hif_scatter_req {
    /* address for the read/write operation */
    unsigned int addr;
    /* request flags */
    unsigned int req;
    /* total length of entire transfer */
    unsigned int len;

    void (*complete) (struct sk_buff *);

    bool free;
    int result;
    int scat_count;

    struct scatterlist sgentries[MAX_SG_ENTRIES];
    struct amlw_hif_scatter_item scat_list[MAX_SG_ENTRIES];
    struct tx_trb_info_ex page;
};

struct aml_hwif_sdio {
    struct sdio_func * sdio_func_if[SDIO_FUNCNUM_MAX];
    bool scatter_enabled;

    /* protects access to scat_req */
    OS_LOCK scat_lock;

    /* scatter request list head */
    struct amlw_hif_scatter_req *scat_req;
};

struct aml_hif_sdio_ops {
    //sdio func0 for self define domain, cmd52
    int (*hi_self_define_domain_func0_write8)(int addr, unsigned char data);
    unsigned char (*hi_self_define_domain_func0_read8)(int addr);
    //sdio func1 for self define domain, cmd52
    int (*hi_self_define_domain_write8)(int addr, unsigned char data);
    unsigned char (*hi_self_define_domain_read8)(int addr);
    int (*hi_self_define_domain_write32)(unsigned long sram_addr, unsigned long sramdata);
    unsigned long (*hi_self_define_domain_read32)(unsigned long sram_addr);

    //sdio func2 for random ram
    void (*hi_random_word_write)(unsigned int addr, unsigned int data);
    unsigned int (*hi_random_word_read)(unsigned int addr);
    void (*hi_random_ram_write)(unsigned char *buf, unsigned char *addr, size_t len);
    void (*hi_random_ram_read)(unsigned char *buf, unsigned char *addr, size_t len);

    //sdio func3 for sram
    void (*hi_sram_word_write)(unsigned int addr, unsigned int data);
    unsigned int (*hi_sram_word_read)(unsigned int addr);
    void (*hi_sram_write)(unsigned char *buf, unsigned char *addr, size_t len);
    void (*hi_sram_read)(unsigned char *buf, unsigned char *addr, size_t len);

    //sdio func4 for tx buffer
    void (*hi_tx_buffer_write)(unsigned char *buf, unsigned char *addr, size_t len);
    void (*hi_tx_buffer_read)(unsigned char *buf, unsigned char *addr, size_t len);

    //sdio func5 for rxdesc
    void (*hi_desc_read)(unsigned char *buf, unsigned char *addr, size_t len);

    //sdio func6 for rx buffer
    void (*hi_rx_buffer_read)(unsigned char* buf, unsigned char* addr, size_t len, unsigned char scat_use);

    //scatter list operation
    int (*hi_enable_scat)(struct aml_hwif_sdio *hif_sdio);
    void (*hi_cleanup_scat)(struct aml_hwif_sdio *hif_sdio);
    struct amlw_hif_scatter_req * (*hi_get_scatreq)(struct aml_hwif_sdio *hif_sdio);
    int (*hi_scat_rw)(struct scatterlist *sg_list, unsigned int sg_num, unsigned int blkcnt,
        unsigned char func_num, unsigned int addr, unsigned char write);
    int (*hi_send_frame)(struct amlw_hif_scatter_req *scat_req);
    int (*hi_recv_frame)(struct amlw_hif_scatter_req *scat_req);

    //sdio func7 for bt
    void (*bt_hi_write_sram)(unsigned char* buf, unsigned char* addr, SYS_TYPE len);
    void (*bt_hi_read_sram)(unsigned char* buf, unsigned char* addr, SYS_TYPE len);
    void (*bt_hi_write_word)(unsigned int addr,unsigned int data);
    unsigned int (*bt_hi_read_word)(unsigned int addr);

    //suspend & resume
    int (*hif_suspend)(unsigned int suspend_enable);
};

struct auc_hif_ops {
    int (*hi_send_cmd)(unsigned int addr, unsigned int len);
    void (*hi_write_word)(unsigned int addr,unsigned int data, unsigned int ep);
    unsigned int (*hi_read_word)(unsigned int addr, unsigned int ep);
    void (*hi_write_sram)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);
    void (*hi_read_sram)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);

    void (*hi_rx_buffer_read)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);

    /*bt use*/
    void (*hi_write_word_for_bt)(unsigned int addr,unsigned int data, unsigned int ep);
    unsigned int (*hi_read_word_for_bt)(unsigned int addr, unsigned int ep);
    void (*hi_write_sram_for_bt)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);
    void (*hi_read_sram_for_bt)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);

    int (*hi_enable_scat)(void);
    void (*hi_cleanup_scat)(void);
    struct amlw_usb_hif_scatter_req * (*hi_get_scatreq)(void);
    int (*hi_scat_rw)(struct scatterlist *sg_list, unsigned int sg_num, unsigned int blkcnt,
        unsigned char func_num, unsigned int addr, unsigned char write);

    int (*hi_send_frame)(struct amlw_usb_hif_scatter_req *scat_req);
    void (*hi_rcv_frame)(unsigned char* buf, unsigned char* addr, unsigned long len);
};

struct amlw1_hif_ops {
	int				(*hi_bottom_write8)(unsigned char func_num, int addr, unsigned char data);
	unsigned char			(*hi_bottom_read8)(unsigned char func_num, int addr);
	int				(*hi_bottom_read)(unsigned char func_num, int addr, void *buf, size_t len, int incr_addr);
	int				(*hi_bottom_write)(unsigned char func_num, int addr, void *buf, size_t len, int incr_addr);

	unsigned char			(*hi_read8_func0)(unsigned long sram_addr);
	void				(*hi_write8_func0)(unsigned long sram_addr, unsigned char sramdata);

	unsigned long			(*hi_read_reg8)(unsigned long sram_addr);
	void				(*hi_write_reg8)(unsigned long sram_addr, unsigned long sramdata);
	unsigned long			(*hi_read_reg32)(unsigned long sram_addr);
	int				(*hi_write_reg32)(unsigned long sram_addr, unsigned long sramdata);

	void				(*hi_write_cmd)(unsigned long sram_addr, unsigned long sramdata);
	void				(*hi_write_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
	void				(*hi_read_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
	void				(*hi_write_word)(unsigned int addr, unsigned int data);
	unsigned int			(*hi_read_word)(unsigned int addr);

	void				(*hi_rcv_frame)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);

	int				(*hi_enable_scat)(void);
	void				(*hi_cleanup_scat)(void);
	struct amlw_hif_scatter_req *	(*hi_get_scatreq)(void);
	int				(*hi_scat_rw)(struct scatterlist *sg_list, unsigned int sg_num, unsigned int blkcnt, unsigned char func_num, unsigned int addr, unsigned char write);
	int				(*hi_send_frame)(struct amlw_hif_scatter_req *scat_req);

	/*bt use*/
	void				(*bt_hi_write_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
	void				(*bt_hi_read_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
	void				(*bt_hi_write_word)(unsigned int addr, unsigned int data);
	unsigned int			(*bt_hi_read_word)(unsigned int addr);

    void				(*hif_get_sts)(unsigned int op_code, unsigned int ctrl_code);
	void				(*hif_pt_rx_start)(unsigned int qos);
	void				(*hif_pt_rx_stop)(void);

	int				(*hif_suspend)(unsigned int suspend_enable);
};

/*struct aml_bus_state_detect {
  unsigned char bus_err;
  unsigned char is_drv_load_finished;
  unsigned char bus_reset_ongoing;
  unsigned char is_load_by_timer;
  unsigned char is_recy_ongoing;
  struct timer_list timer;
  struct work_struct detect_work;
  int (*insmod_drv)(void);
};*/

struct aml_pm_type {
    atomic_t bus_suspend_cnt;
    atomic_t drv_suspend_cnt;
    atomic_t is_suht_down;
    atomic_t wifi_enable;
};

typedef void (*bt_shutdown_func)(void);

enum usb_endpoint_num{
    USB_EP0 = 0x0,
    USB_EP1,
    USB_EP2,
    USB_EP3,
    USB_EP4,
    USB_EP5,
    USB_EP6,
    USB_EP7,
};

#define HCI_MAX_EVENT_SIZE    260
#define HCI_MAX_DATA_SIZE     1028
//#define HCI_MAX_ACL_SIZE      1021
//#define HCI_MAX_FRAME_SIZE    (HCI_MAX_ACL_SIZE + 7)
#define HCI_COMMAND_PKT       0x01
#define HCI_ACLDATA_PKT       0x02
#define HCI_SCODATA_PKT       0x03
#define HCI_EVENT_PKT         0x04
#define HCI_15P4_PKT          0x10
#define HCI_NO_TYPE           0xfe
#define HCI_VENDOR_PKT        0xff

gdsl_fifo_t *gdsl_fifo_init(unsigned int len, unsigned char *base_addr);
void gdsl_fifo_deinit(gdsl_fifo_t *p_fifo);
unsigned int gdsl_fifo_used(gdsl_fifo_t *p_fifo);
unsigned int gdsl_fifo_used_size(gdsl_fifo_t *p_fifo);
unsigned int gdsl_fifo_remain(gdsl_fifo_t *p_fifo);
unsigned int gdsl_fifo_copy_data(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len);
unsigned int gdsl_fifo_get_data(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len);
unsigned int gdsl_fifo_calc_r(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len);
unsigned int gdsl_fifo_get_data(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len);
//unsigned int gdsl_read_data(gdsl_fifo_t *p_fifo, unsigned char *data, unsigned int len, unsigned int ep);
//unsigned int gdsl_write_data_by_ep(gdsl_fifo_t *p_fifo, unsigned char *data, unsigned int len, unsigned int ep);
unsigned int gdsl_fifo_update_r(gdsl_fifo_t *p_fifo, unsigned int len);

extern unsigned int g_dbg_level;
extern unsigned int amlbt_if_type;
extern unsigned int polling_time;
extern unsigned int amlbt_ft_mode;
extern bt_shutdown_func g_bt_shutdown_func;

typedef int (*ws_inf)(unsigned char *buf, unsigned char *sram_addr, unsigned int len, unsigned int ep);
typedef int (*rs_inf)(unsigned char *buf, unsigned char *sram_addr, unsigned int len, unsigned int ep);
typedef int (*ww_inf)(unsigned int addr,unsigned int data, unsigned int ep);
typedef int (*rw_inf)(unsigned int addr, unsigned int ep, unsigned int *value);
typedef void (*wrs_sdio_inf)(unsigned char* buf, unsigned char* addr, unsigned int len);

#define BTUSB_IOC_MAGIC 'x'

#define IOCTL_GET_BT_RECOVERY       _IOR(BTUSB_IOC_MAGIC, 0, int)
#define IOCTL_GET_DEVICE_PID        _IOR(BTUSB_IOC_MAGIC, 1, int)
#define IOCTL_GET_BT_REG            _IOR(BTUSB_IOC_MAGIC, 2, int)
#define IOCTL_SET_BT_REG            _IOW(BTUSB_IOC_MAGIC, 3, int)
#define IOCTL_SET_HCI_CMD           _IOW(BTUSB_IOC_MAGIC, 4, int)
#define IOCTL_GET_BT_BUF            _IOR(BTUSB_IOC_MAGIC, 5, int)
#define IOCTL_GET_BT_VERSION        _IOR(BTUSB_IOC_MAGIC, 6, int)
#define IOCTL_SET_BT_SHUTDOWN       _IOW(BTUSB_IOC_MAGIC, 7, int)
#define IOCTL_GET_COEX_STATUS       _IOR(BTUSB_IOC_MAGIC, 8, int)
#define IOCTL_REGISTER_SDIO         _IOW(BTUSB_IOC_MAGIC, 9, int)

#define FAMILY_TYPE_IS_W1(x)        ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W1)
#define FAMILY_TYPE_IS_W1U(x)       ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W1U)
#define FAMILY_TYPE_IS_W2(x)        ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W2)
#define FAMILY_TYPE_IS_W2L(x)       ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W2L)

#define INTF_TYPE_IS_SDIO(x)        ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_SDIO)
#define INTF_TYPE_IS_USB(x)         ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_USB)
#define INTF_TYPE_IS_PCIE(x)        ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_PCIE)

#define AMLBT_PD_ID_INTF            0x7
#define AMLBT_PD_ID_FAMILY_VER      (0x7<<6)
#define AMLBT_PD_ID_FAMILY          (0x1f<<9)

#define AMLBT_INTF_SDIO             0x0
#define AMLBT_INTF_USB              0x01
#define AMLBT_INTF_PCIE             0x02

#define AMLBT_FAMILY_W1             (0x01<<9)
#define AMLBT_FAMILY_W1U            (0x02<<9)
#define AMLBT_FAMILY_W2             (0x03<<9)
#define AMLBT_FAMILY_W2L            (0x04<<9)
#define AMLBT_TRANS_UNKNOWN          0x00

#define AMLBT_TRANS_W1_UART         0x01
#define AMLBT_TRANS_W1U_UART        0x02
#define AMLBT_TRANS_W2_UART         0x03
#define AMLBT_TRANS_W1U_USB         0x04
#define AMLBT_TRANS_W2_USB          0x05
#define AMLBT_TRANS_W2L_USB         0x06

#define BTA(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_ALL) printk(KERN_INFO "BTA:" fmt, ## arg)
#define BTD(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_DEBUG) printk(KERN_INFO "BTD:" fmt, ## arg)
#define BTI(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_INFO) printk(KERN_INFO "BTI:" fmt, ## arg)
#define BTP(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_POINT) printk(KERN_INFO "BTP:" fmt, ## arg)
#define BTW(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_WARN) printk(KERN_ERR "BTW:" fmt, ## arg)
#define BTE(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_ERROR) printk(KERN_ERR "BTE:" fmt, ## arg)
#define BTF(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_FATAL) printk(KERN_ERR "BTF:" fmt, ## arg)

//input device
#define INPUT_NAME                "input_btdrv"
#define INPUT_PHYS                "input_btdrv/input0"
//#define KEY_NETFLIX               133 //android use
#define KEY_NETFLIX               468 //linux use

//wake source
#define REMOTE_WAKEUP           2 //infrared
#define BT_WAKEUP               4 //bt powerkey
#define REMOTE_CUS_WAKEUP       9 //bt netflixkey

#endif

