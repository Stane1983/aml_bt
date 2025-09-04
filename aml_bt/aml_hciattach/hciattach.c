/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2000-2001  Qualcomm Incorporated
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2002-2010  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <poll.h>
#include <sys/time.h>
#include <sys/param.h>
#include <termios.h>
#include <semaphore.h>
#include <pthread.h>

#include "hciattach.h"
#include "hciattach_aml.h"
#include "hciattach_aml_usb.h"
#include "aml_multibt.h"

struct uart_t {
    char *type;
    int  m_id;
    int  p_id;
    int  proto;
    int  init_speed;
    int  speed;
    int  flags;
    int  pm;
    char *dev;
    char *bdaddr;
    int  (*init) (int fd, struct uart_t *u, struct termios *ti);
    int  (*post) (int fd, struct uart_t *u, struct termios *ti);
};

typedef struct {
    char *dev_name;
    char *type;
} iface_info;

#define FLOW_CTL    0x0001
#define AMP_DEV     0x0002
#define ENABLE_PM   1
#define DISABLE_PM  0
static struct uart_t *u = NULL;
int g_usb_fd = -1;
static bool usb_mod_flag = false;
static bool stop_flag = false;

static volatile sig_atomic_t __io_canceled = 0;

static void sig_hup(int sig)
{
}

static void sig_term(int sig)
{
    fprintf(stderr, "sig_term\n");
    __io_canceled = 1;
}

static void sig_alarm(int sig)
{
    fprintf(stderr, "Initialization timed out.\n");
    exit(1);
}

static void amlbt_sig_usr(int sig)
{
    ALOGD("SIGUSR1");
    stop_flag = true;
    __io_canceled = 1;
}

static int tty_get_speed(int s)
{
    switch (s) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
#ifdef B2500000
    case 2500000:
        return B2500000;
#endif
#ifdef B3000000
    case 3000000:
        return B3000000;
#endif
#ifdef B3500000
    case 3500000:
        return B3500000;
#endif
#ifdef B4000000
    case 4000000:
        return B4000000;
#endif
    default:
        return B57600;
    }
}

int set_speed(int fd, struct termios *ti, int speed)
{
    if (cfsetospeed(ti, tty_get_speed(speed)) < 0)
        return -errno;

    if (cfsetispeed(ti, tty_get_speed(speed)) < 0)
        return -errno;

    if (tcsetattr(fd, TCSANOW, ti) < 0)
        return -errno;

    return 0;
}

/*
 * Read an HCI event from the given file descriptor.
 */
int read_hci_event(int fd, unsigned char* buf, int size)
{
    int remain, r;
    int count = 0;

    if (size <= 0)
        return -1;

    /* The first byte identifies the packet type. For HCI event packets, it
     * should be 0x04, so we read until we get to the 0x04. */
    while (1) {
        r = read(fd, buf, 1);
        if (r <= 0)
            return -1;
        if (buf[0] == 0x04)
            break;
    }
    count++;

    /* The next two bytes are the event code and parameter total length. */
    while (count < 3) {
        r = read(fd, buf + count, 3 - count);
        if (r <= 0)
            return -1;
        count += r;
    }

    /* Now we read the parameters. */
    if (buf[2] < (size - 3))
        remain = buf[2];
    else
        remain = size - 3;

    while ((count - 3) < remain) {
        r = read(fd, buf + count, remain - (count - 3));
        if (r <= 0)
            return -1;
        count += r;
    }

    return count;
}

static int aml(int fd, struct uart_t *u, struct termios *ti)
{
	ALOGI("amlbt uart init");
	return aml_init(fd, u->bdaddr);
}

struct uart_t uart[] = {
    /* AML ROME */
    { "aml",    0x0000, 0x0000, HCI_UART_H4, 115200, 4000000,
            FLOW_CTL, DISABLE_PM, UART_DEV_PORT_BT, NULL, aml, NULL },
#ifdef AUDIO_PROJECT
    // aml_w2l_s support bt/zibgee/thread
    { "aml_15p4",	 0x0000, 0x0000, HCI_UART_LL, 115200, 4000000,
            FLOW_CTL, DISABLE_PM, UART_DEV_PORT_BT, NULL, aml, NULL },
#else
    { "aml_15p4",	 0x0000, 0x0000, HCI_UART_H4, 115200, 4000000,
            FLOW_CTL, DISABLE_PM, UART_DEV_PORT_BT, NULL, aml, NULL },
#endif
    { NULL, 0 }
};

const iface_info amlbt_iface[] = {
    { "aml_w1",     "aml" },
    { "aml_w1u",    "aml" },
    { "aml_w1u_s",  "aml" },
    { "aml_w2_p",   "aml" },
    { "aml_w2_s",   "aml"},
    { "aml_w2_u",   "aml_usb"},
    { "aml_w2l_s",  "aml_15p4"},
    { "aml_w2l_u",  "aml_usb"},
    { NULL, 0 }
};

static int amlbt_get_init_cfg(struct uart_t **dev)
{
    int i, j;
    const char *bt_name = get_bt_name();

    if (!bt_name) {
        ALOGE("NULL bt_name");
        return -1;
    }

    for (i = 0; amlbt_iface[i].dev_name; i++) {
        if (strcmp(bt_name, amlbt_iface[i].dev_name) != 0)
            continue;

        if (strcmp(amlbt_iface[i].type, "aml_usb") == 0) {
            usb_mod_flag = true;
            return 0;
        }

        for (j = 0; uart[j].type; j++) {
            if (strcmp(amlbt_iface[i].type, uart[j].type) == 0) {
                *dev = &uart[j];
                return 0;
            }
        }
    }

    ALOGE("fail");
    return -1;
}

static struct uart_t * get_by_id(int m_id, int p_id)
{
    int i;
    for (i = 0; uart[i].type; i++) {
        if (uart[i].m_id == m_id && uart[i].p_id == p_id)
            return &uart[i];
    }
    return NULL;
}

static struct uart_t * get_by_type(char *type)
{
    int i;
    for (i = 0; uart[i].type; i++) {
        if (!strcmp(uart[i].type, type))
            return &uart[i];
    }
    return NULL;
}

/* Initialize UART driver */
static int init_uart(char *dev, struct uart_t *u, int send_break, int raw)
{
    struct termios ti;
    int fd, i;
    unsigned long flags = 0;

    if (raw)
        flags |= 1 << HCI_UART_RAW_DEVICE;

    if (u->flags & AMP_DEV)
        flags |= 1 << HCI_UART_CREATE_AMP;

    fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Can't open serial port");
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    if (tcgetattr(fd, &ti) < 0) {
        perror("Can't get port settings");
        goto fail;
    }

    cfmakeraw(&ti);

    ti.c_cflag |= CLOCAL;
    if (u->flags & FLOW_CTL)
        ti.c_cflag |= CRTSCTS;
    else
        ti.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &ti) < 0) {
        perror("Can't set port settings");
        goto fail;
    }

    /* Set initial baudrate */
    if (set_speed(fd, &ti, u->init_speed) < 0) {
        perror("Can't set initial baud rate");
        goto fail;
    }

    tcflush(fd, TCIOFLUSH);

    if (send_break) {
        tcsendbreak(fd, 0);
        usleep(500000);
    }

    if (u->init && u->init(fd, u, &ti) < 0)
        goto fail;

    tcflush(fd, TCIOFLUSH);

    /* Set actual baudrate */
    if (set_speed(fd, &ti, u->speed) < 0) {
        perror("Can't set baud rate");
        goto fail;
    }

    /* Set TTY to N_HCI line discipline */
    i = N_HCI;
    if (ioctl(fd, TIOCSETD, &i) < 0) {
        perror("Can't set line discipline");
        goto fail;
    }

    if (flags && ioctl(fd, HCIUARTSETFLAGS, flags) < 0) {
        perror("Can't set UART flags");
        goto fail;
    }

    if (ioctl(fd, HCIUARTSETPROTO, u->proto) < 0) {
        perror("Can't set device");
        goto fail;
    }

    if (u->post && u->post(fd, u, &ti) < 0)
        goto fail;

    return fd;

fail:
    close(fd);
    return -1;
}

static void usage(void)
{
    printf("hciattach - HCI UART driver initialization utility\n");
    printf("Usage:\n");
    printf("\thciattach [-n] [-p] [-b] [-r] [-t timeout] [-s initial_speed][-d debug]"
            " <tty> <type | id> [speed] [flow|noflow]"
            " [sleep|nosleep] [bdaddr]\n");
    printf("\thciattach -l\n");
}

int main(int argc, char *argv[])
{
    int detach, printpid, raw, opt, i, n, ld;
    int to = 10;
    int init_speed = 0;
    int send_break = 0;
    int err = -1, val = -1;
    pid_t pid;
    struct sigaction sa;
    struct pollfd p;
    sigset_t sigs;
    char dev[PATH_MAX];
    detach = 1;
    printpid = 0;
    raw = 0;

    while ((opt=getopt(argc, argv, "d:bnpt:s:lr")) != EOF) {
        switch (opt) {
        case 'd':
            val = atoi(optarg);
            set_dbg_flag(val);
            break;

        case 'b':
            send_break = 1;
            break;

        case 'n':
            detach = 0;
            break;

        case 'p':
            printpid = 1;
            break;

        case 't':
            to = atoi(optarg);
            break;

        case 's':
            init_speed = atoi(optarg);
            break;

        case 'l':
            for (i = 0; uart[i].type; i++) {
                printf("%-10s0x%04x,0x%04x\n", uart[i].type,
                            uart[i].m_id, uart[i].p_id);
            }
            exit(0);

        case 'r':
            raw = 1;
            break;

        default:
            usage();
            exit(1);
        }
    }

    for (n = 0; optind < argc; n++, optind++) {
        char *opt;

        opt = argv[optind];

        switch (n) {
        case 0:
            dev[0] = 0;
            if (!strchr(opt, '/'))
                strcpy(dev, "/dev/");

            if (strlen(opt) > PATH_MAX - (strlen(dev) + 1)) {
                fprintf(stderr, "Invalid serial device\n");
                exit(1);
            }

            strcat(dev, opt);
            break;

        case 1:
            if (strchr(argv[optind], ',')) {
                int m_id, p_id;
                sscanf(argv[optind], "%x,%x", &m_id, &p_id);
                u = get_by_id(m_id, p_id);
            } else {
                u = get_by_type(opt);
            }

            if (!u) {
                fprintf(stderr, "Unknown device type or id\n");
                exit(1);
            }

            break;

        case 2:
            u->speed = atoi(argv[optind]);
            break;

        case 3:
            if (!strcmp("flow", argv[optind]))
                u->flags |=  FLOW_CTL;
            else
                u->flags &= ~FLOW_CTL;
            break;

        case 4:
            if (!strcmp("sleep", argv[optind]))
                u->pm = ENABLE_PM;
            else
                u->pm = DISABLE_PM;
            break;

        case 5:
            u->bdaddr = argv[optind];
            break;
        }
    }

    if (steps_table_poll() != 0)
         exit(1);

    if (amlbt_get_init_cfg(&u) != 0)
         exit(1);

    if (usb_mod_flag) {
        if (strcmp(dev, "/dev/aml_btusb")!= 0) {
            strcpy(dev, "/dev/aml_btusb");
            //printf("usb mode dev is: %s\n", dev);
        }

        //get amlbt usb usb device file descriptor
        g_usb_fd = get_usb_fd(dev);
        if (g_usb_fd == -1) {
            fprintf(stderr, "open usb device failed!\n");
            exit(1);
        }

        //init usb bt device
        err = aml_usb_init(g_usb_fd);
        if (err < 0) {
            fprintf(stderr, "aml usb device init failed!\n");
            exit(1);
        }
    } else {
        if (!u) {
            ALOGE("Unknown amlbt uart dev type or id");
            exit(1);
        }

        /* If user specified a initial speed, use that instead of
           the hardware's default */
        if (init_speed)
            u->init_speed = init_speed;

        memset(&sa, 0, sizeof(sa));
        sa.sa_flags   = SA_NOCLDSTOP;
        sa.sa_handler = sig_alarm;
        sigaction(SIGALRM, &sa, NULL);

        /* 10 seconds should be enough for initialization */
        alarm(to);

        n = init_uart(u->dev, u, send_break, raw);
        if (n < 0) {
            perror("Can't initialize device");
            exit(1);
        }

        ALOGD("amlbt uart dev setup complete");

        alarm(0);

        memset(&sa, 0, sizeof(sa));
        sa.sa_flags   = SA_NOCLDSTOP;
        sa.sa_handler = SIG_IGN;
        sigaction(SIGCHLD, &sa, NULL);
        sigaction(SIGPIPE, &sa, NULL);

        sa.sa_handler = sig_term;
        sigaction(SIGTERM, &sa, NULL);
        sigaction(SIGINT,  &sa, NULL);

        sa.sa_handler = sig_hup;
        sigaction(SIGHUP, &sa, NULL);

        sa.sa_handler = amlbt_sig_usr;
        sigaction(SIGUSR1, &sa, NULL);

        if (detach) {
            if ((pid = fork())) {
                ALOGD("fork child process pid:%d", pid);
                if (printpid)
                    printf("%d\n", pid);
                return 0;
            }
#if 0
            for (i = 0; i < 20; i++)
                if (i != n)
                    close(i);
#endif
        }

        p.fd = n;
        p.events = POLLERR | POLLHUP;

        sigfillset(&sigs);
        sigdelset(&sigs, SIGCHLD);
        sigdelset(&sigs, SIGPIPE);
        sigdelset(&sigs, SIGTERM);
        sigdelset(&sigs, SIGINT);
        sigdelset(&sigs, SIGHUP);
        sigdelset(&sigs, SIGUSR1);

        while (!__io_canceled) {
            p.revents = 0;
            err = ppoll(&p, 1, NULL, &sigs);
            if (err < 0 && errno == EINTR)
                continue;
            if (err)
                break;
        }

        /* Restore TTY line discipline */
        ld = N_TTY;
        if (ioctl(n, TIOCSETD, &ld) < 0) {
            ALOGE("Can't restore line discipline");
            perror("Can't restore line discipline");
            close(n);
            exit(1);
        }

        if (!stop_flag) {
            if (u->type && !strcmp(u->type, "aml"))
                amlbt_set_woble_cfg(n);
        } else {
            amlbt_drv_rmmod();
        }

        close(n);
    }

    ALOGD("quit");
    return 0;
}

