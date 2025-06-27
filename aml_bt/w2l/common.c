/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#include <linux/skbuff.h>
#include <linux/slab.h>

#include "common.h"
#include "w2l_bt_entry.h"

unsigned int g_dbg_level = LOG_LEVEL_ERROR;
unsigned int amlbt_if_type = AMLBT_TRANS_UNKNOWN;
unsigned int polling_time = 8000;
unsigned int amlbt_ft_mode = 0;
bt_shutdown_func g_bt_shutdown_func;

gdsl_fifo_t *gdsl_fifo_init(unsigned int len, unsigned char *base_addr)
{
    gdsl_fifo_t *p_fifo = (gdsl_fifo_t *)kzalloc(sizeof(gdsl_fifo_t), GFP_DMA|GFP_ATOMIC);
    BTA("%s \n", __func__);
    if (p_fifo == NULL)
    {
        BTE("gdsl_fifo_init alloc error!\n");
        return NULL;
    }
    else
    {
        memset(p_fifo, 0, sizeof(gdsl_fifo_t));
        p_fifo->w = 0;
        p_fifo->r = 0;
        p_fifo->base_addr = base_addr;
        p_fifo->size = len;
    }
    return p_fifo;
}

void gdsl_fifo_deinit(gdsl_fifo_t *p_fifo)
{
    if (p_fifo == NULL)
    {
        return ;
    }

    kfree(p_fifo);
}

unsigned int gdsl_fifo_used(gdsl_fifo_t *p_fifo)
{
    if (p_fifo->r <= p_fifo->w)
        return (p_fifo->w - p_fifo->r);

    return (p_fifo->size + p_fifo->w - p_fifo->r);
}

unsigned int gdsl_fifo_used_size(gdsl_fifo_t *p_fifo)
{
    if (p_fifo->w == p_fifo->r)
    {
        return 0;
    }

    if (p_fifo->w > p_fifo->r)
    {
        return (p_fifo->w - p_fifo->r);
    }

    if (p_fifo->w < p_fifo->r)
    {
        return (p_fifo->size - (p_fifo->r - p_fifo->w));
    }

    return 0;
}

unsigned int gdsl_fifo_remain(gdsl_fifo_t *p_fifo)
{
    unsigned int used = gdsl_fifo_used(p_fifo);

    return p_fifo->size - used - 4;
}
#if 0
unsigned int gdsl_fifo_copy_data(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len)
{
    unsigned int i = 0;
    BTA("copy d %d, %#x, %#x\n", len, p_fifo->base_addr, p_fifo->w);

    if (gdsl_fifo_remain(p_fifo) < len)
    {
        BTE("gdsl_fifo_copy_data no space!!\n");
        BTE("fifo->base_addr %#x, fifo->size %#x\n", (unsigned long)p_fifo->base_addr, p_fifo->size);
        BTE("fifo->w %#x, fifo->r %#x\n", (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
        BTE("remain %#x, len %#x\n", gdsl_fifo_remain(p_fifo), len);
        return 0;
    }

    while (i < len)
    {
        *(unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr) = buff[i];
        p_fifo->w = (unsigned char *)(((unsigned long)p_fifo->w + 1) % p_fifo->size);
        i++;
    }

    BTA("actual len %#x \n", __func__, i);

    return i;
}
#else
unsigned int gdsl_fifo_copy_data(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len)
{
    unsigned int i = 0;
    unsigned long offset = (unsigned long)p_fifo->w;

    BTA("copy d %d, %#x, %#x\n", len, p_fifo->base_addr, p_fifo->w);

    if (gdsl_fifo_remain(p_fifo) < len)
    {
        BTE("gdsl_fifo_copy_data no space!!\n");
        BTE("fifo->base_addr %#x, fifo->size %#x\n", (unsigned long)p_fifo->base_addr, p_fifo->size);
        BTE("fifo->w %#x, fifo->r %#x\n", (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
        BTE("remain %#x, len %#x\n", gdsl_fifo_remain(p_fifo), len);
        return -1;
    }

    if (len < (p_fifo->size - offset))
    {
        memcpy((unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr), buff, len);
        p_fifo->w = (unsigned char *)(((unsigned long)p_fifo->w + len) % p_fifo->size);
    }
    else
    {
        memcpy((unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr), buff, p_fifo->size - offset);
        memcpy((unsigned char *)((unsigned long)p_fifo->base_addr), &buff[p_fifo->size - offset], len - (p_fifo->size - offset));
        p_fifo->w = (unsigned char *)((len - (p_fifo->size - offset)) % p_fifo->size);
    }

    BTA("actual len %#x \n", i);

    return len;
}
#endif
#if 0
unsigned int gdsl_fifo_calc_r(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len)
{
    unsigned int used = gdsl_fifo_used(p_fifo);
    unsigned int i = 0;
    unsigned int get_len = (len >= used ? used : len);

    BTA("get d %d, %#x, %#x\n", get_len, p_fifo->base_addr, p_fifo->w);
    if (used == 0)
    {
        return 0;
    }

    while (i < get_len)
    {
        p_fifo->r = (unsigned char *)(((unsigned long)p_fifo->r + 1) % p_fifo->size);
        i++;
    }

    BTA("actual len %#x \n", __func__, i);

    return i;
}
#else
unsigned int gdsl_fifo_calc_r(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len)
{
    unsigned int used = gdsl_fifo_used(p_fifo);
    unsigned int i = 0;
    unsigned int get_len = (len >= used ? used : len);
    unsigned long offset = (unsigned long)p_fifo->r;

    BTA("get d %d, %#x, %#x\n", get_len, p_fifo->base_addr, p_fifo->w);
    if (used == 0)
    {
        return 0;
    }

    if (get_len < (p_fifo->size - offset))
    {
        p_fifo->r = (unsigned char *)(((unsigned long)p_fifo->r + get_len) % p_fifo->size);
    }
    else
    {
        p_fifo->r = (unsigned char *)((get_len - (p_fifo->size - offset)) % p_fifo->size);
    }

    BTA("actual len %#x \n", i);

    return get_len;
}

#endif


unsigned int gdsl_fifo_update_r(gdsl_fifo_t *p_fifo, unsigned int len)
{
    unsigned int offset = 0;
    unsigned int read_len = 0;
    unsigned char *p_end = 0;

    //BTA("%s p_fifo->w %#x, p_fifo->r %#x\n", __func__, (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
    //BTA("%s len %d\n", __func__, len);

    if (p_fifo->w == p_fifo->r)
    {
        BTE("%s no data!!!\n", __func__);
        return 0;
    }

    if (p_fifo->w > p_fifo->r)
    {
        read_len = (unsigned int)(p_fifo->w - p_fifo->r);
        if (len <= read_len)
        {
            read_len = len;
        }
        //BTA("%s read len A %d\n", __func__, read_len);
        p_fifo->r += read_len;
    }
    else
    {
        p_end = (p_fifo->base_addr + p_fifo->size);
        BTA("%s w %#x, r %#x\n", __func__, (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
        BTA("%s read p_end %#x\n", __func__, (unsigned long)p_end);
        offset = (unsigned int)(p_end - p_fifo->r);
        read_len = offset;
        if (len < offset)
        {
            p_fifo->r += len;
            read_len = len;
            BTA("%s 111 len %#x \n", __func__, len);
        }
        else
        {
            p_fifo->r = p_fifo->base_addr;
            read_len += (len - offset);
            p_fifo->r += (len - offset);
            BTA("%s 222 len %#x \n", __func__, len);
        }
        //BTA("%s read len B %#x \n", __func__, read_len);
    }

    //BTA("%s actual len %#x \n", __func__, read_len);

    return read_len;
}
#if 0
unsigned int gdsl_fifo_get_data(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len)
{
    unsigned int used = gdsl_fifo_used(p_fifo);
    unsigned int i = 0;
    unsigned int get_len = (len >= used ? used : len);

    BTD("get d %d, %#x, %#x %#x\n", get_len, p_fifo->base_addr, p_fifo->w, p_fifo->r);
    if (used == 0)
    {
        return 0;
    }

    while (i < get_len)
    {
        buff[i] = *(unsigned char *)((unsigned long)p_fifo->r + (unsigned long)p_fifo->base_addr);
        p_fifo->r = (unsigned char *)(((unsigned long)p_fifo->r + 1) % p_fifo->size);
        i++;
    }

    BTA("actual len %s %#x \n", __func__, i);

    return i;
}
#else
unsigned int gdsl_fifo_get_data(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len)
{
    unsigned int used = gdsl_fifo_used(p_fifo);
    unsigned int i = 0;
    unsigned int get_len = (len >= used ? used : len);
    unsigned long offset = (unsigned long)p_fifo->r;

    BTD("get d %d, %#x, %#x %#x\n", get_len, p_fifo->base_addr, p_fifo->w, p_fifo->r);
    if (used == 0)
    {
        return 0;
    }

    if (get_len < (p_fifo->size - offset))
    {
        memcpy(buff, (unsigned char *)((unsigned long)p_fifo->r + (unsigned long)p_fifo->base_addr), get_len);
        p_fifo->r = (unsigned char *)(((unsigned long)p_fifo->r + get_len) % p_fifo->size);
    }
    else
    {
        memcpy(buff, (unsigned char *)((unsigned long)p_fifo->r + (unsigned long)p_fifo->base_addr), (p_fifo->size - offset));
        memcpy(&buff[p_fifo->size - offset], (unsigned char *)((unsigned long)p_fifo->base_addr), (get_len - (p_fifo->size - offset)));
        p_fifo->r = (unsigned char *)((get_len - (p_fifo->size - offset)) % p_fifo->size);
    }

    BTA("actual len %s %#x \n", __func__, i);

    return get_len;
}
#endif
