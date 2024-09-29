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

#if 0
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <syslog.h>
#include <termios.h>
#include <time.h>
#include <poll.h>
#include <sys/time.h>
#include <sys/param.h>
#include <sys/ioctl.h>
#include "pthread.h"
//#include "lib/bluetooth.h"
//#include "lib/hci.h"
//#include "lib/hci_lib.h"
#include <semaphore.h>

//#include "src/shared/tty.h"

#include "hciattach.h"

struct uart_t {
	char *type;
	int  m_id;
	int  p_id;
	int  proto;
	int  init_speed;
	int  speed;
	int  flags;
	int  pm;
	char *bdaddr;
	int  (*init) (int fd, struct uart_t *u, struct termios *ti);
	int  (*post) (int fd, struct uart_t *u, struct termios *ti);
};

#define FLOW_CTL	0x0001
#define AMP_DEV		0x0002
#define ENABLE_PM	1
#define DISABLE_PM	0
static int serial_fd;
static struct uart_t *u = NULL;
static sem_t sem;

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
    fprintf(stdout,"aml\n");
    return aml_init(fd, u->bdaddr);
}

struct uart_t uart[] = {
	{ "any",        0x0000, 0x0000, HCI_UART_H4,   115200, 115200,
				FLOW_CTL, DISABLE_PM, NULL, NULL     },

	/* AML ROME */
	{ "aml",	0x0000, 0x0000, HCI_UART_H4, 115200, 4000000,
			FLOW_CTL, DISABLE_PM, NULL, aml, NULL },


	{ NULL, 0 }
};

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
	printf("\thciattach [-n] [-p] [-b] [-r] [-t timeout] [-s initial_speed]"
			" <tty> <type | id> [speed] [flow|noflow]"
			" [sleep|nosleep] [bdaddr]\n");
	printf("\thciattach -l\n");
}

int main(int argc, char *argv[])
{
	int detach, printpid, raw, opt, i, n, ld, err;
	int to = 10;
	int init_speed = 0;
	int send_break = 0;
	pid_t pid;
	struct sigaction sa;
	struct pollfd p;
	sigset_t sigs;
	char dev[PATH_MAX];

	detach = 1;
	printpid = 0;
	raw = 0;

	while ((opt=getopt(argc, argv, "bnpt:s:lr")) != EOF) {
		switch (opt) {
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

	n = argc - optind;
	if (n < 2) {
		usage();
		exit(1);
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

	if (!u) {
		fprintf(stderr, "Unknown device type or id\n");
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

	n = init_uart(dev, u, send_break, raw);
	serial_fd = n;
	if (n < 0) {
		perror("Can't initialize device");
		exit(1);
	}

	printf("Device setup complete\n");

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

	printf("Device setup complete\n");
	if (detach) {
		if ((pid = fork())) {
			if (printpid)
				printf("%d\n", pid);
			return 0;
		}

		for (i = 0; i < 20; i++)
			if (i != n)
				close(i);
	}

	p.fd = n;
	p.events = POLLERR | POLLHUP;

	sigfillset(&sigs);
	sigdelset(&sigs, SIGCHLD);
	sigdelset(&sigs, SIGPIPE);
	sigdelset(&sigs, SIGTERM);
	sigdelset(&sigs, SIGINT);
	sigdelset(&sigs, SIGHUP);

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
		perror("Can't restore line discipline");
		exit(1);
	}



	printf("hciattatch quit.\n");
	return 0;
}
#endif

#include <stdio.h>
#include <getopt.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
//#include <sys/io.h>
#include <limits.h>

#include <string.h>
#include <signal.h>
#include <unistd.h>
//#include "hcitool_for_usb.h"
#include "hciattach.h"
#define CMD_TEST_FW _IOR('A', 2, int)
#define MAC_LEN 6
#define SAVE_MAC "/nvram/bt_mac"
u_int8 vendor_local_addr[MAC_LEN];


typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

int debug = 0;
int g_usb_fd = -1;
int g_version_data = 0x117;
u_int8 *BT_fwICCM = NULL;
u_int8 *BT_fwDCCM = NULL;
u_int32 ICCM_LEN = 0;
u_int32 DCCM_LEN = 0;

#define UINT8_TO_STREAM(p, u8)   { *(p)++ = (uint8_t)(u8); }
#define UINT16_TO_STREAM(p, u16) { *(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8); }
#define set_bdaddr_delay 40
//#define W2L_CHIPSET
#define W2_CHIPSET
//#define W1U_CHIPSET

#if defined(W2L_CHIPSET)
    #define CHIP_OFFSET (384 * 1024)
#elif defined(W1U_CHIPSET) || defined(W2_CHIPSET)
    #define CHIP_OFFSET (256 * 1024)
#else
    #error "No chipset defined. Please define W1U_CHIPSET, W2_CHIPSET, or W2L_CHIPSET."
#endif

/******************************************************************************
**  delay function
******************************************************************************/
void ms_delay(uint32_t timeout)
{
	struct timespec delay;
	int err;

	if (timeout == 0)
		return;

	delay.tv_sec = timeout / 1000;
	delay.tv_nsec = 1000 * 1000 * (timeout % 1000);

	/* [u]sleep can't be used because it uses SIGALRM */
	do {
		err = nanosleep(&delay, &delay);
	} while (err < 0 && errno == EINTR);
}

void aml_insert_uint32(t_p_pdu p_buffer, u_int32 value_32_bit)
{
	p_buffer[0] = (u_int8)_SYS_GET_CHAR_8_BITS(value_32_bit);
	p_buffer[1] = (u_int8)_SYS_GET_CHAR_8_BITS(value_32_bit >> 8);
	p_buffer[2] = (u_int8)_SYS_GET_CHAR_8_BITS(value_32_bit >> 16);
	p_buffer[3] = (u_int8)_SYS_GET_CHAR_8_BITS(value_32_bit >> 24);
}

u_int32 aml_get_uint32(t_p_pdu p_pdu)
{
	return (*p_pdu) + ((*(p_pdu + 1)) << 8) + ((*(p_pdu + 2)) << 16) + ((*(p_pdu + 3)) << 24);
}

void dump(unsigned char *out, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		if (i && !(i % 16)) {
			fprintf(stderr, "\n");
		}

		fprintf(stderr, "%02x ", out[i]);
	}

	fprintf(stderr, "\n");
}

int get_usb_fd()
{
    int usb_fd = -1;
    if ((usb_fd = open(BT_USB_DEVICE, O_RDWR | O_NOCTTY)) == -1)
	{
		fprintf(stderr, "BT USB Device %s could not be opened, error %d\n",BT_USB_DEVICE, errno);
        return -1;
	}
	else
	{
		printf("BT USB Device %d can be opened\n",usb_fd);
	}
    return usb_fd;
}

void hex_dump(char *pref, int width, u_int8 *buf, int len)
{
	register int i,n;

	for (i = 0, n = 1; i < len; i++, n++) {
		if (n == 1)
			printf("%s", pref);
		printf("%2.2X ", buf[i]);
		if (n == width) {
			printf("\n");
			n = 0;
		}
	}
	if (i && n != 1)
		printf("\n");
}

int read_event(unsigned char *buffer)
{
    int rx_len = 0;
    int timeout_cnt = 0;
    int event_cnt = 0;
    unsigned int event_parameter_len = 0;

    ///M @{
    fd_set read_fds;
    int evt_state = 0;
    int rval;
    struct timeval wait_time = {0, 200000};
    enum
    {
        EVT_TYPE,
        EVT_HEAD,
        EVT_PLAYLOAD,
        EVT_END
    };

    ///@}

    while (evt_state < EVT_END)
    {
        FD_ZERO(&read_fds);
        FD_SET(g_usb_fd, &read_fds);
        rval = select(g_usb_fd + 1, &read_fds, NULL, NULL, &wait_time);
        if (rval > 0)
        {
            switch (evt_state)
            {
                case EVT_TYPE:
                    rx_len = read(g_usb_fd, &buffer[event_cnt], 1);
                    event_cnt += rx_len;
                    evt_state++;
                    break;
                case EVT_HEAD:
                    rx_len = read(g_usb_fd, &buffer[event_cnt], 2);
                    event_cnt += rx_len;
                    event_parameter_len = buffer[2];
                    evt_state++;
                case EVT_PLAYLOAD:
                    rx_len = read(g_usb_fd, &buffer[event_cnt], event_parameter_len);
                    event_cnt += rx_len;
                    evt_state++;
                    break;
                default:
                    break;
            }
        }
        else  if (rval == 0)
        {
            timeout_cnt++;
            if (timeout_cnt > 150)
            {
                printf("Rx event parameter timeout!\n");
                return 0;
            }
        }
        wait_time.tv_sec = 0;
        wait_time.tv_usec = 200000;
        FD_ZERO(&read_fds);
        FD_SET(g_usb_fd, &read_fds);
    }
    if (debug)
        dump(buffer, event_cnt);
    return event_cnt;
}


int hci_send_cmd(u_int8 *buf, int len)
{
	int write_bytes = 0;
    //u_int8 hcitype[] = {HCI_pduCOMMAND};
    //write_bytes = write(g_usb_fd, hcitype, 1);
	write_bytes = write(g_usb_fd, buf, len);
	if (write_bytes)
    {
        //write_bytes = write(g_usb_fd, buf + 1, len -1);
        if (debug)
        {
            fprintf(stderr, "writing\n");
            dump(buf, len);

            if (write_bytes == -1) {
                fprintf(stderr,"Failed to write bytes to uart,error code is[%d]\n",errno);
            }
	    }
		 //printf("Send hcicommand success! write_bytes:%d\n",write_bytes);
		//return 0;
    }

	else
    {
        printf("Send hcicommand failed!\n");
        return -1;
    }
    return 0;
}

u_int32 read_bt_register(u_int32 address)
{
    int ret = 0;
    u_int8 pdu[8] = {0};
    u_int32 data = 0;
    u_int8 recv_data[256] = {0};

    pdu[0] = HCI_pduCOMMAND;
    pdu[1] = TCI_DBG_READ_REG;
    pdu[2] = PROP_OGF;
    pdu[3] = READ_BT_REG_LEN;
    aml_insert_uint32(&pdu[4], address);
    hci_send_cmd(pdu, sizeof(pdu));
    ret = read_event(recv_data);
    if (ret > 0)
    {
        if ((recv_data[0] == 0x04) &&
            (recv_data[4] == TCI_DBG_READ_REG) &&
            (recv_data[5] == PROP_OGF))
        {
            data = aml_get_uint32(&recv_data[7]);
            return data;
        }
    }
    printf("Read BT register 0x%06x failed!\n", address);
    return 0;
}


void write_bt_register(u_int32 address, u_int32 value)
{
    int ret = 0;
    u_int8 pdu[12] = {0};
    u_int8 recv_data[256] = {0};

    pdu[0] = HCI_pduCOMMAND;
    pdu[1] = TCI_DBG_WRITE_REG;
    pdu[2] = PROP_OGF;
    pdu[3] = WRITE_BT_REG_LEN;
    aml_insert_uint32(&pdu[4], address);
    aml_insert_uint32(&pdu[8], value);
    hci_send_cmd(pdu, sizeof(pdu));
    ret = read_event(recv_data);
    if (ret > 0)
    {
        if ((recv_data[0] == 0x04) &&
            (recv_data[4] == TCI_DBG_WRITE_REG) &&
            (recv_data[5] == PROP_OGF))
        {
            goto WriteSuccess;
        }
    }

    printf("Write BT register 0x%06x failed!\n", address);
WriteSuccess:
    return;
}

int TCI_Generate_uart_load_bt_fw_Command(int opcode, u_int8 data_len, u_int32 addr, u_int8* data)
{
    u_int8 pdu[300] = { 0 };
	u_int16 length = 0;
	int i = 0;
    int rx_len = 0;
    u_int8 rx_buff[1024] = {0};
	u_int8 *temp_data = (u_int8*)malloc(300);
	if (!temp_data)
    {
		printf("New space fail!\n");
		return -1;
	}
	pdu[0] = (u_int8)(opcode & 0xFF);
	pdu[1] = (u_int8)((opcode >> 8) & 0xFF);
	pdu[2] = data_len;
	length = pdu[2] + 3;
	aml_insert_uint32(pdu + 3, addr);
	for (i = 0; i < data_len; i += 1)
	{
		pdu[i + 7] = *(data + i);
	}
	//temp_data[0] = HCI_pduCOMMAND;
	//memcpy(temp_data + 1, pdu, length);
	//hci_send_cmd(temp_data, length + 1);
	memcpy(temp_data, pdu, length);
	hci_send_cmd(temp_data, length);

    //dump(temp_data, length + 1);
    rx_len = read_event(rx_buff);
	if (rx_len > 0)
	{
		free(temp_data);
		return 0;
	}
	free(temp_data);
	return -1;
}


int download_fw_img()
{
    int count = 0;
	int result = 0;
	u_int8* bufferICCM = BT_fwICCM;
	u_int8* bufferDCCM = BT_fwDCCM;
	int len = 0, offset = 0, offset_in_bt =0;
	u_int32 cmd_len = 0;
	u_int32 data_len = 0;
	int cnt = 1;
	offset_in_bt = CHIP_OFFSET;
	offset = CHIP_OFFSET;
	len = ICCM_LEN - offset_in_bt;//ALIGN(sizeof(BT_fwICCM), 4) -256*1024;
	printf("BT start iccm copy, total=0x%x \n", len);

    if (len == 0)
		goto DCC_START;
    printf("BT iccm base 0x%x \r\n", ICCM_RAM_BASE);
    do
    {
		data_len = (len > RW_OPERTION_SIZE) ? RW_OPERTION_SIZE : len;
		cmd_len = data_len + 4;
		result = TCI_Generate_uart_load_bt_fw_Command(TCI_DOWNLOAD_BT_FW, cmd_len,
          ICCM_RAM_BASE + offset, bufferICCM + offset);
		if (result)
        {
			printf("Download iccm data error!\n");
			return FAILED;
		}
		cnt++;
		offset += data_len;
		len -= data_len;
		count++;

	}while (len > 0);
    printf("Download iccm data success!\n");
DCC_START:
    len = DCCM_LEN;//ALIGN(sizeof(BT_fwDCCM), 4);
	offset = 0;
	cnt = 1;
	data_len = 0;
	cmd_len = 0;
	printf("start dccm copy, total=0x%x\n", len);
	do
	{
		data_len = (len > RW_OPERTION_SIZE) ? RW_OPERTION_SIZE : len;
		cmd_len = data_len + 4;
		result = TCI_Generate_uart_load_bt_fw_Command(TCI_DOWNLOAD_BT_FW, cmd_len, DCCM_RAM_BASE + offset, bufferDCCM + offset);
		if (result) {
			printf("Download dccm data error!\n");
			return -1;
		}
		cnt++;
		offset += data_len;
		len -= data_len;
		count++;
	} while (len > 0);
	printf("Download dccm data sunccess!\n");
	return SUCCESS;
}

int start_download_firmware(char* firmware_file_path)
{
    int binfile_fd = -1;
    unsigned int iccm_size = 0;
    unsigned int dccm_size = 0;
    int size = 0;
    char fw_bin_file[384];
    int result = 0;
    int revData = -1;
	uint32_t reg_data = 0;
    memset(fw_bin_file, 0, strlen(fw_bin_file));
    if ((binfile_fd = open(firmware_file_path, O_RDONLY)) < 0)
    {
        printf("Could not open %s\n", firmware_file_path);
        return FAILED;
    }
	printf("open firmware file success binfile_fd:%d\n", binfile_fd);
    size = read(binfile_fd, &iccm_size, 4);
	printf("hw_config_get_iccm_size iccm_size:%d size:%d\n", iccm_size,size);
    if (size < 0)
    {
        printf("---------1hw_config_get_iccm_size read error!---------");
        close(binfile_fd);
        return FAILED;
    }
    printf("---------2hw_config_get_iccm_size iccm_size %d---------\n", iccm_size);
    size = read(binfile_fd, &dccm_size, 4);
    if (size < 0)
    {
        printf("---------hw_config_get_dccm_size read error!---------");
        close(binfile_fd);
        return FAILED;
    }
    printf("---------hw_config_get_dccm_size dccm_size %d---------\n", dccm_size);
    BT_fwICCM = (u_int8*)malloc(iccm_size);
    BT_fwDCCM = (u_int8*)malloc(dccm_size);
    if ((!BT_fwICCM) ||(!BT_fwDCCM))
    {
        printf("Malloc failed!\n");
        return FAILED;
    }
    size = read(binfile_fd, BT_fwICCM, iccm_size);
    if (size < 0)
    {
        printf("---------hw_config_get_iccm_data error!---------");
        close(binfile_fd);
        free(BT_fwICCM);
        free(BT_fwDCCM);
        return FAILED;
    }
    size = read(binfile_fd, BT_fwDCCM, dccm_size);
    if (size < 0)
    {
        printf("---------hw_config_get_dccm_data error!---------");
        close(binfile_fd);
        free(BT_fwICCM);
        free(BT_fwDCCM);
        return FAILED;
    }
    ICCM_LEN = iccm_size;
    DCCM_LEN = dccm_size;

    if (download_fw_img() == FAILED) return FAILED;
    result = TCI_Write_Register(TCI_WRITE_REG, 0xa70014, 0x0000000);
	if (!result) {
		printf("close fw load!\n");;
	}
	else
	{
		printf("close fw load failed!\n");
		return -1;
	}

	reg_data = get_config_params();
	pr_info("0xf03040 reg_data=%u\n", reg_data);
    result = TCI_Write_Register(TCI_WRITE_REG, 0xf03040, reg_data);
    if (result == TCI_WRITE_READ_FAILED)
    {
        printf("BT CPU can't work!\n");;
        return FAILED;
    }

	result = TCI_Write_Register(TCI_WRITE_REG, 0xf03058, 0x0000000);
    if (result == TCI_WRITE_READ_FAILED)
    {
        printf("reset controller fail!\n");;
        return FAILED;
    }

    free(BT_fwICCM);
    free(BT_fwDCCM);
    //revData = ioctl(g_usb_fd, CMD_TEST_FW, 101);
    //printf("receive data=%d\n", revData);
    close(binfile_fd);
    return SUCCESS;
}


int TCI_Write_Register(int opcode, u_int32 address, u_int32 value)
{
    u_int8 pdu[30] = { 0 };
	u_int8 *data = (u_int8*)malloc(30);
	u_int8 tmp_buff[4096] = { 0 };
	int rx_len = 0;
	u_int16 length = 0;
	if (!data) {
		printf("New space fail!\n");
		return TCI_WRITE_READ_FAILED;
	}

	pdu[0] = (u_int8)(opcode & 0xFF);
	pdu[1] = (u_int8)((opcode >> 8) & 0xFF);
	pdu[2] = 8;
	length = pdu[2] + 3;
	aml_insert_uint32(pdu + 3, address);
	aml_insert_uint32(pdu + 7, value);
    //data[0] = HCI_pduCOMMAND;
	//memcpy(data + 1, pdu, length);
	//hci_send_cmd(data, length + 1);
	memcpy(data, pdu, length);
	hci_send_cmd(data, length);
    rx_len = read_event(tmp_buff);
    dump(tmp_buff, rx_len);

	free(data);
	printf("TCI_WRITE_READ_SUCCESS success!\n");
	return TCI_WRITE_READ_SUCCESS;
}

static int get_a2dp_sink_enable(void)
{
	int size;
	int fd_a2dp_cfg = 0;
	char buffer[255] = { 0 };
	char c = '=';
	uint8_t a2dp_sink_enable = 0;

	fd_a2dp_cfg = open(AML_A2DP_CFG_FILE, O_RDONLY);

	if (fd_a2dp_cfg < 0)
	{
		pr_info("In %s, Open failed:%s", __FUNCTION__, strerror(errno));
		return -1;
	}

	size = read(fd_a2dp_cfg, buffer, sizeof(buffer));
	if (size < 0)
	{
		pr_info("In %s, Read failed:%s", __FUNCTION__, strerror(errno));
		close(fd_a2dp_cfg);
		return -1;
	}
	buffer[sizeof(buffer) - 1] = 0;
	char *ptr_a2dp_cfg = strchr(buffer, c);
	if (!ptr_a2dp_cfg) {
		pr_info("In %s, wrong a2dp_sink_enable value", __FUNCTION__);
		close(fd_a2dp_cfg);
		return -1;
	}
	ptr_a2dp_cfg++;
	a2dp_sink_enable = atoi(ptr_a2dp_cfg);

	close(fd_a2dp_cfg);

	pr_info("Setting parameters to controller: a2dp_sink_enable=%d.", a2dp_sink_enable);

	return a2dp_sink_enable;
}

static int get_antenna_number(void)
{
	int size;
	char c = '=';
	uint8_t antenna_num = 0;
	int fd_ant_num = 0;
	char buffer[255] = { 0 };

	fd_ant_num = open(AML_BT_CONFIG_RF_FILE, O_RDONLY);

	if (fd_ant_num < 0)
	{
			pr_info("In %s, Open failed:%s", __FUNCTION__, strerror(errno));
			return -1;
	}

	size = read(fd_ant_num, buffer, sizeof(buffer));
	if (size < 0)
	{
			pr_info("In %s, Read failed:%s", __FUNCTION__, strerror(errno));
			close(fd_ant_num);
			return -1;
	}
	buffer[sizeof(buffer) - 1] = 0;
	char *ptr_a2dp_cfg = strchr(buffer, c);
	if (!ptr_a2dp_cfg) {
			pr_info("In %s, wrong a2dp_sink_enable value", __FUNCTION__);
			close(fd_ant_num);
			return -1;
	}
	ptr_a2dp_cfg++;
	antenna_num = atoi(ptr_a2dp_cfg);

	close(fd_ant_num);

	pr_info("Setting parameters to controller: antenna number=%d.", antenna_num);

	return antenna_num;
}

static int get_config_params(void)
{
	uint32_t reg_data = 0;
	uint8_t a2dp_sink = 0;
	uint8_t ant_num = 0;

	a2dp_sink = get_a2dp_sink_enable();
	ant_num = get_antenna_number();

	pr_info("a2dp_sink_enable=%d ,antenna number=%d\n", a2dp_sink,ant_num);

	if (ant_num == 1)
	{
		reg_data = 0x10000000; //bit28 means RF_ANT_SINGLE
	}
	else if (ant_num == 2)
	{
		reg_data = 0x20000000; //bit29 means RF_ANT_DOUBLE
	}

	if (a2dp_sink == 1)
	{
		reg_data |= (1<<25);	// bit25 means a2dp_sink_enable.
	}

	pr_info("reg_data=%x\n", reg_data);//0x12000000

	return reg_data;
}

int aml_get_random(char *bdaddr)
{

	int randomData = open("/dev/random", O_RDONLY);
    if (randomData < 0) {
        pr_info("Failed to open /dev/random");
        return 1;
    }

    unsigned char buffer[4];
    ssize_t bytesRead = read(randomData, buffer, sizeof(buffer));
    if (bytesRead < 0) {
        pr_info("Failed to read from /dev/random");
        close(randomData);
        return 1;
    }

	memcpy(bdaddr + 2, buffer, 4);

    close(randomData);
    return 0;
}

/******************************************************************************
** save file
******************************************************************************/

uint8_t * aml_getprop_read(const char* str)
{
	int fd, n;
	char buf[18];
	memset(buf, '\0', sizeof(buf));
	pr_info("opening %s\n", str);
	fd = open(str, O_RDONLY|O_CREAT, 0666);
	if (fd < 0)
	{
		perror("open read");
		goto error;
	}
	n = read(fd, buf, sizeof(buf)-1);
	if (n < sizeof(buf)-1)
	{
		pr_info("n < sizeof(buf)");
		close(fd);
		goto error;
	}

	buf[sizeof(buf)-1] ='\0';
	close(fd);

	if (strnlen(buf, 17) != 17)
	{
		pr_info("don't matching bt mac");
		goto error;
	}
	sscanf(buf, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
                 &vendor_local_addr[0], &vendor_local_addr[1], &vendor_local_addr[2],\
                 &vendor_local_addr[3], &vendor_local_addr[4], &vendor_local_addr[5]);
	return vendor_local_addr;

error:
	return NULL;

}

int aml_setprop_write(const char *str, int size)
{
	int err = -1;
	int fd;
	pr_info("opening %s\n", str);
	fd = open(SAVE_MAC, O_WRONLY|O_CREAT, 0666);
	if (fd < 0)
	{
		perror("open write");
		goto error;
	}
	err = write(fd, str, size);
	if (err != size)
	{
		pr_err("write fail");
	}
	close(fd);

error:
	return err;

}

static int get_fw_version(char *str)
{
	int fd;
	int ret;
	char * fw_version = NULL;
	str = str + 7; //skip 7byte
	ret = asprintf(&fw_version, "fw_version: date = %02x.%02x, number = 0x%02x%02x\n", *(str+1),*str,*(str+3),*(str+2));
	if (ret <= 0)
	{
		pr_info("get version value failed\n");
		goto error;
	}
	fd = open(FW_VER_FILE,  O_WRONLY|O_CREAT|O_TRUNC, 0666);
	if (fd < 0)
	{
		pr_err("open fw_file fail");
		free(fw_version);
		goto error;
	}
	write(fd, fw_version, strlen(fw_version));
	free(fw_version);
	close(fd);
error:
	return 0;
}

int aml_set_APCF_config(int fd)
{
	int size;
	int err = -1;
	unsigned char cmd[HCI_MAX_CMD_SIZE];
	unsigned char rsp[HCI_MAX_EVENT_SIZE];
	char *cmd_hdr = NULL;
	int write_bytes = 0;

	unsigned char APCF_config_manf_data[] = {0x01, 0x22, 0xFC, 0x05, 0x19, 0xff, 0x01, 0x0a, 0xb};
	unsigned char apcf_config[9] = {0x01, 0x22, 0xFC, 0x05, 0x19, 0xff, 0x01, 0x0a, 0xb};


	pr_info("set APCF: %02x:%02x:%02x:%02x:%02x:%02x", apcf_config[0],apcf_config[1], apcf_config[2], apcf_config[3], apcf_config[4], apcf_config[5]);

	memset(cmd, 0x0, HCI_MAX_CMD_SIZE);
	memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);

	cmd_hdr = (void *) (cmd + 1);
	cmd[0]	= HCI_COMMAND_PKT;
	UINT16_TO_STREAM(cmd_hdr, HCI_VSC_WAKE_WRITE_DATA);
	*cmd_hdr++ = 5;
	*cmd_hdr++ = APCF_config_manf_data[4];
	*cmd_hdr++ = APCF_config_manf_data[5];
	*cmd_hdr++ = APCF_config_manf_data[6];
	*cmd_hdr++ = APCF_config_manf_data[7];
	*cmd_hdr++ = APCF_config_manf_data[8];

	size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + 5);
	/* Send the HCI command packet to UART for transmission */
	pr_info("HCI CMD: 0x%x 0x%x 0x%x 0x%x", cmd[0], cmd[1], cmd[2], cmd[3]);

	u_int8 hcitype[] = {HCI_pduCOMMAND};
	write_bytes = write(fd, hcitype, 1);

	if (write_bytes)
	{
		err = write(fd, cmd +1, size-1);
		if (err != size-1) {
			pr_err("Send failed with ret value: %d", err);
			goto error;
		}
	}
	else
	{
		goto error;
	}

	//delay 20ms wait event
	ms_delay(20);
	pr_info("delay %d",20);

	/* Wait for command complete event */
	err = read_event(rsp);
	if ( err < 0) {
		pr_err("Failed to set apcf config on Controller");
		goto error;
	}

	pr_info("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
	pr_info("hci set apcf success %d",err);
	return err;
error:
	return err;
	pr_info("hci set apcf error");
}


int aml_set_bdaddr(int fd)
{
	int size;
	int err = -1;
	unsigned char cmd[HCI_MAX_CMD_SIZE];
	unsigned char rsp[HCI_MAX_EVENT_SIZE];
	char *cmd_hdr = NULL;
	int write_bytes = 0;
	char buf[18];
	uint8_t *tempbuf;

	unsigned char local_addr[MAC_LEN];

	if ((tempbuf = aml_getprop_read(SAVE_MAC)) != NULL)
	{
		memcpy(local_addr, tempbuf, MAC_LEN);
		goto set_mac;
	}

	memset(buf, '\0', sizeof(buf));
	memset(local_addr, '\0', MAC_LEN);

	local_addr[0] = 0x22;
	local_addr[1] = 0x22;
	//local_addr[2] = (uint8_t)rand();
	//local_addr[3] = (uint8_t)rand();
	//local_addr[4] = (uint8_t)rand();
	//local_addr[5] = (uint8_t)rand();

	/*get random number from /dev/random, if succed save it to file*/
	if (!aml_get_random(local_addr))
	{
		sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",local_addr[0],local_addr[1], local_addr[2], local_addr[3], local_addr[4], local_addr[5]);
		err = aml_setprop_write(buf, sizeof(buf));
		if (err < 0)
		{
			pr_info("aml_getprop_write fail");
		}

	} else {
		pr_err("Failed to get random mac addr");
		goto error;
	}

set_mac:
	pr_info("set bdaddr: %02x:%02x:%02x:%02x:%02x:%02x", local_addr[0],local_addr[1], local_addr[2], local_addr[3], local_addr[4], local_addr[5]);

	memset(cmd, 0x0, HCI_MAX_CMD_SIZE);
	memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);

	cmd_hdr = (void *) (cmd + 1);
	cmd[0]	= HCI_COMMAND_PKT;
	UINT16_TO_STREAM(cmd_hdr, HCI_VSC_WRITE_BD_ADDR);
	*cmd_hdr++ = MAC_LEN;
	*cmd_hdr++ = local_addr[5];
	*cmd_hdr++ = local_addr[4];
	*cmd_hdr++ = local_addr[3];
	*cmd_hdr++ = local_addr[2];
	*cmd_hdr++ = local_addr[1];
	*cmd_hdr++ = local_addr[0];

	size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + MAC_LEN);
	/* Send the HCI command packet to UART for transmission */
	pr_info("HCI CMD: 0x%x 0x%x 0x%x 0x%x", cmd[0], cmd[1], cmd[2], cmd[3]);

	u_int8 hcitype[] = {HCI_pduCOMMAND};
	write_bytes = write(fd, hcitype, 1);

	if (write_bytes)
	{
		err = write(fd, cmd +1, size-1);
		if (err != size-1) {
			pr_err("Send failed with ret value: %d", err);
			goto error;
		}
	}
	else
	{
		goto error;
	}

	//delay 20ms wait event
	ms_delay(set_bdaddr_delay);
	pr_info("delay %d",set_bdaddr_delay);

	/* Wait for command complete event */
	err = read_event(rsp);
	if ( err < 0) {
		pr_err("Failed to set patch info on Controllern");
		goto error;
	}

	pr_info("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
	pr_info("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", rsp[7], rsp[8], rsp[9], rsp[10], rsp[11], rsp[12], rsp[13]);

	if (rsp[4] != cmd[1] || rsp[5] != cmd[2] || rsp[6] != 0X00) {
		pr_err("Failed to set_bdaddr, command failure");
		return -1;
	}
	get_fw_version(rsp);

	pr_info("hci set bdaddr success %d",err);
	return err;
error:
	return err;
	pr_info("hci set bdaddr error");
}


int aml_hci_reset(int fd)
{
	int size, err = 0;
	unsigned char cmd[HCI_MAX_CMD_SIZE];
	unsigned char rsp[HCI_MAX_EVENT_SIZE];
	hci_command_hdr *cmd_hdr;
	int flags;
	int write_bytes = 0;
	pr_info("HCI RESET");

	memset(cmd, 0x0, HCI_MAX_CMD_SIZE);

	cmd_hdr = (void *) (cmd + 1);
	cmd[0]  = HCI_COMMAND_PKT;
	cmd_hdr->opcode = HCI_RESET;
	cmd_hdr->plen   = 0;

	/* Total length of the packet to be sent to the Controller */
	size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE);

	/* Send the HCI command packet to UART for transmission */
	pr_info("HCI CMD: 0x%x 0x%x 0x%x 0x%x", cmd[0], cmd[1], cmd[2], cmd[3]);


	u_int8 hcitype[] = {HCI_pduCOMMAND};
	write_bytes = write(fd, hcitype, 1);

	if (write_bytes)
	{
		err = write(fd, cmd +1, size-1);
		if (err != size-1) {
			pr_err("Send failed with ret value: %d", err);
			goto error;
		}
	}
	else
	{
		goto error;
	}

	pr_info("Received HCI-Vendor Specific Event from SOC");

	/* Wait for command complete event */
	err = read_event(rsp);
	if ( err < 0) {
		pr_err("Failed to set patch info on Controller");
		goto error;
	}

	pr_info("HCI rsp: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
	if (rsp[4] != cmd[1] || rsp[5] != cmd[2] || rsp[6] != 0X00) {
		pr_err("Failed to tci_write, command failure");
		return -1;
	}
	pr_info("reset success");
	return err;

error:
	pr_info("hci reset error");
	return err;

}


void handle_usr_command(int argc,char *argv[])
{
    u_int8 buff[1024] = {0};
    u_int8 rx_buff[1024] = {0};
    int hcicmd_len = argc;
    int event_len = 0;
    u_int8 ocf;
	u_int8 ogf;
	int err = -1;

	printf("handle_usr_command\n");

    if (strcmp("d", argv[0]) == 0)
    {
        if (argc < 1)
        {
            printf("Please input the path of bt firmware file!\n");
            return;
        }
        if (start_download_firmware(argv[1]))
        {
            printf("Failed to download BT firmware\n");

        }
        else
        {
            printf("Download BT firmware success!\n");
			//TCI_Write_Register(TCI_UPDATE_UART_BAUDRATE, 0xf03058, 0x1);
			printf("sleep 1s go to set addr!\n");
			usleep(1000);
			printf("HCI start set addr!\n");
			err = aml_set_bdaddr(g_usb_fd);
			if (err < 0)
			{
				printf("HCI set bdaddr Failed\n !!!");
			}
			printf("HCI set bdaddr is done\n");

			err = aml_set_APCF_config(g_usb_fd);
			if (err < 0)
			{
				printf("HCI set APCF Failed\n !!!");
			}
			printf("HCI set APCF is done\n");
        }
    }
	else if (strcmp("reset", argv[0]) == 0)
	{
			/* Perform HCI reset here*/
		err = aml_hci_reset(g_usb_fd);
		if (err < 0)
		{
			printf("HCI Reset Failed\n !!!");
		}
		printf("HCI Reset is done\n");
	}
	else if (strcmp("setaddr", argv[0]) == 0)
	{
			/* Perform HCI set bdaddr here*/
		err = aml_set_bdaddr(g_usb_fd);
		if (err < 0)
		{
			printf("HCI set bdaddr Failed\n !!!");
		}
		printf("HCI set bdaddr is done\n");
	}
    else if (strcmp("cmd", argv[0]) == 0)
    {
        if (argc < 2)
        {
           printf("Too few parameters\n");
           return;
        }
        ogf = (u_int8)strtol(argv[1], NULL, 16);
	    ocf = (u_int8)strtol(argv[2], NULL, 16);
        hcicmd_len -= 2;
		#if 0
        buff[0] = HCI_pduCOMMAND;
        buff[1] = ocf;
        buff[2] = ((ogf<<10)>>8);
        buff[3] = hcicmd_len;

        for (int i = 0; i < hcicmd_len;++i)
        {
            buff[i + 4] = (u_int8)strtol(argv[i + 3], NULL, 16);
        }
        hci_send_cmd(buff, hcicmd_len + 4);
		#endif
		buff[0] = ocf;
	    buff[1] = ((ogf << 10) >> 8);
	    buff[2] = hcicmd_len;

	    for (int i = 0; i < hcicmd_len; ++i)
	    {
	        buff[i + 3] = (u_int8)strtol(argv[i + 3], NULL, 16);
	    }
	    hci_send_cmd(buff, hcicmd_len + 3);
        printf("< HCI Command: ogf 0x%02x, ocf 0x%04x, plen %d\n", ogf, ocf, hcicmd_len);
        hex_dump("  ", 20, buff, hcicmd_len); fflush(stdout);
        event_len = read_event(rx_buff);
        printf("> HCI Event: 0x%02x plen %d\n", rx_buff[1], event_len - 3);
	    hex_dump("Event Data: ", 20, buff, event_len - 3); fflush(stdout);
    }
    else if (strcmp("r", argv[0]) == 0)
    {
        if (argc < 1)
        {
            printf("Too few parameters\n");
            return;
        }

        printf("Register:%s Value:0x%02x\n",argv[1], read_bt_register((u_int32)atof(argv[1])));
    }
    else if (strcmp("w", argv[0]) == 0)
    {
        if (argc < 2)
        {
            printf("Too few parameters\n");
            return;
        }
        write_bt_register((u_int32)atof(argv[1]), (u_int32)atof(argv[2]));
    }
    else
    {
        printf("%s is not support!\n", argv[0]);
    }

}

void usr_help_mesg()
{
    printf("******************************************Help Information******************************************\n");
    printf("*hcitool_for_usb [command type] [parameters1] [parameters2] [...]                                  *\n");
    printf("*command type:                                                                                     *\n");
    printf("*                                                                       	                   *\n");
    printf("*\td:download BT firmware    parameter:bt firmware file path                                  *\n");
    printf("*\t\tfor example:hcitool_for_usb d /storate/emulated/0/bt_fw.bin                              *\n");
    printf("*                                                                       	                   *\n");
    printf("*\tr:read bt register  parameter:register address                                             *\n");
    printf("*                                                                       	                   *\n");
    printf("*\t\tfor example:hcitool_for_usb r 0x123546                                             *\n");
    printf("*                                                                       	                   *\n");
    printf("*\tw:write bt register  parameter1:register address parameter2:register value                 *\n");
    printf("*                                                                       	                   *\n");
    printf("*\t\tfor example:hcitool_for_usb w 0x123546 0x987654                                    *\n");
    printf("*                                                                       	                   *\n");
    printf("*\tcmd:send hcicommand parameters:hcicommand                                                  *\n");
    printf("*                                                                       	                   *\n");
    printf("*\t\tfor example:hcitool_for_usb cmd 0x01 0xf0 0xfc 0x00 0x00...                        *\n");
    printf("*                                                                       	                   *\n");
    printf("*\tloopback:enable dut enter loopback test mode parameters:timeout(s)                         *\n");
    printf("*                                                                       	                   *\n");
    printf("*\t\tfor example:hcitool_for_usb loopback 60                                            *\n");
    printf("*                                                                       	                   *\n");
    printf("*\thelp:show help information                                                                 *\n");
    printf("*                                                                       	                   *\n");
    printf("*\tv:show version information                                                                 *\n");
    printf("***********************************************End**************************************************\n");
}

int main(int argc,char *argv[])
{
	if (argc < 2)
    {
        printf("Too few parameters %s\n",argv[0]);
        return -1;
    }
    if (strcmp("help", argv[1]) == 0)
    {
        usr_help_mesg();
        return 0;
    }
    else if (strcmp("v", argv[1]) == 0)
    {
        printf("------------Amlogic hcitool for usb verison:%04x------------\n",g_version_data);
        printf("------------Author:barry.sun------------\n");
        printf("------------Email:barry.sun@amlogic.com------------\n");
		//g_usb_fd = get_usb_fd();
		//printf("------------Open USB device:%d------------\n",g_usb_fd);
        return 0;
    }
	g_usb_fd = get_usb_fd();
    if (g_usb_fd == -1)
    {
        printf("Open USB device failed!\n");
        return -1;
    }
    handle_usr_command(argc - 2, argv + 1);
    close(g_usb_fd);
	return 0;
}

