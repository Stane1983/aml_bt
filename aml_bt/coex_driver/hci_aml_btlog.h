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

#ifndef __AML_BTLOG_H__
#define __AML_BTLOG_H__

#include <linux/device.h>

#define AML_BTLOG_CTRL_NAME "bt_logctrl"

#define CHIP_INTF_REG_BASE 0xf00000
#define RG_AON_A59 (CHIP_INTF_REG_BASE + 0xEC)
#define BTLOG_LOW2_MASK   0x3

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


int aml_btlog_init(struct device *dev);
void aml_btlog_deinit(struct device *dev);

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


#endif

