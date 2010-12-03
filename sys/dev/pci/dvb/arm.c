/*
 * OpenBSD 4.8 driver for Skystar1 DVB card rev. 1.3 and 1.5
 * Copyright (c) 2010 Dinar Talypov & Edward Garipov
 * based on code by:
 * Copyright (c) 2002 Alexander Romanov ported to OpenBSD 3.3 for Alloyant Technologies
 * based on code by:
 * FreeBSD driver for SkyStar1 DVB card (Siemens Fujitsu DVB PCI)
 * Copyright (c) 2000 Stanislav "Stephen" Golovin
 *
 * Based on linux driver code by:
 *
 * Copyright (C) 1999,2000 Ralph  Metzler & Marcus Metzler for convergence integrated media GmbH
 *
 * originally based on code by:
 *
 * Copyright (C) 1998,1999 Christian Theiss <mistert@rz.fh-augsburg.de>
 * 
 * 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 * Or, point your browser to http://www.gnu.org/copyleft/gpl.html
 * 
 * The author can be reached at stas@everest.kaluga.ru
 *
 */

#include <sys/param.h>
#include <sys/socket.h>
#include <machine/bus.h>
#include <uvm/uvm_extern.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/route.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>

#include "frontend.h"
#include "dvb.h"
#include "arm.h"
#include "skystar_dev.h"
#include "arm_dpram4k.h"
#include "arm_comcode.h"
#include "saa7146.h"
#include "ssfw.h"

/* 
 * Data Expansion Bus Interface (DEBI) 
 * TI AV711x - ARM CPU controlled by firmware.
 */

static int      debiwait_maxwait = 5000;

/* prototypes for current DVB implementation  */
static int	wait_for_debi_done(struct skystar_softc *);
static int	debiwrite(struct skystar_softc *,  u_int32_t, int, u_int32_t, int);
static int	waitdebi(struct skystar_softc *, int, int);


/* TODO: insert delays */

static int 
wait_for_debi_done(struct skystar_softc * skystar)
{
	int             i;

	/* wait for registers to be programmed */
	for (i = 0; i < 100000; i++)
		if (read_long(skystar, MC2) & 2)
			break;

	/* wait for transfer to complete */
	for (i = 0; i < 500000 && (read_long(skystar, PSR) & SPCI_DEBI_S); i++)
		read_long(skystar, MC2);
	if (i > debiwait_maxwait)
		printf("wait-for-debi-done maxwait: %d\n",
		       debiwait_maxwait = i);

	if (i >= 500000) {
		printf("DEBUG: skystar: wait_for_debi_done: timeout!\n");
		return -1;
	}
	return 0;
}

/* Initiate busmaster dma DEBI write  debi, intr will arrive when done. */

static int 
debiwrite(struct skystar_softc * skystar, u_int32_t config, int addr, u_int32_t val, int count)
{
	u_int32_t          cmd;

	if (count <= 0 || count > 32764)
		return -1;	/* out of buffer length */

	if (wait_for_debi_done(skystar) < 0)
		return -1;

	write_long(skystar, DEBI_CONFIG, config);

	if (count <= 4)		/* immediate transfer */
		write_long(skystar, DEBI_AD, val);
	else			/* block transfer */
		write_long(skystar, DEBI_AD, vtophys((vaddr_t) skystar->debi));

	write_long(skystar, DEBI_COMMAND, (cmd = (count << 17) | (addr & 0xffff)));
	write_long(skystar, MC2, (2 << 16) | 2);
	return 0;
}

/*
 * Initiaite busmaster dma DEBI read, if count > 4 debi intr will arrive when
 * done else data returns immediately in result
 */

static u_int32_t 
debiread(struct skystar_softc * skystar, u_int32_t config, int addr, int count)
{
	u_int32_t          result = 0;

	if (count > 32764 || count <= 0)
		return 0;	/* out of buffer length */

	if (wait_for_debi_done(skystar) < 0)
		return 0;

	write_long(skystar, DEBI_AD, vtophys((vaddr_t) skystar->debi));
	write_long(skystar, DEBI_COMMAND, (count << 17) | 0x10000 | (addr & 0xffff));
	write_long(skystar, DEBI_CONFIG, config);
	write_long(skystar, MC2, (2 << 16) | 2);
	if (count > 4)
		return count;

	wait_for_debi_done(skystar);
	result = read_long(skystar, DEBI_AD);
	result &= (0xffffffffUL >> ((4 - count) * 8));
#ifdef SKYSTAR_DEBUG
/*	 printf("DEBUG: skystar: debiread: result (%d) = %08x\n", count,(unsigned)result);*/
#endif
	return result;
}

/*
 * debiwrite() wrapper to use in interrupt.
 * If count > 4 - val is used as buffer address.
 * This buffer is copyed to skystar->debi, then debiwrite call
 */

void 
iwdebi(struct skystar_softc * skystar, u_int32_t config, int addr, u_int32_t val, int count)
{

	if (count > 4 && val)
		bcopy((char *) val, (char *) skystar->debi, count);

	if (debiwrite(skystar, config, addr, val, count))
		printf("skystar: DEBI write failed.\n");
}

/*
 * debiread() wrapper to use in interrupt
 * If count <= 4 - return data immediately in result
 */
u_int32_t
irdebi(struct skystar_softc * skystar, u_int32_t config, int addr, u_int32_t val, int count)
{
	u_int32_t	res;

	res = debiread(skystar, config, addr, count);
	if (count <= 4)
		bcopy((char *) &res, (char *) skystar->debi, count);
	return res;
}

/* debiwrite() wrapper to use outside interrupt, only for count<=4! */

void 
wdebi(struct skystar_softc * skystar, u_int32_t config, int addr, u_int32_t val, int count)
{
	/* unsigned long flags; */

	int	s;
	s = splvm();
	/*
	 * printf("DEBUG: wdebi (0x%08x, 0x%08x, 0x%08x, %d)\n",
	 * (unsigned)config, (unsigned)addr, (unsigned)val, count);
	 */
	debiwrite(skystar, config, addr, val, count);
	splx(s);
}

/* debiread() wrapper to use outside interrupt, only for count<=4! */

u_int32_t 
rdebi(struct skystar_softc * skystar, u_int32_t config, int addr, u_int32_t val, int count)
{
	u_int32_t          res;

	int s;
	s = splvm();
	res = debiread(skystar, config, addr, count);
	splx(s);

	return res;
}

/* wait for boot program transefr done */
static int 
waitdebi(struct skystar_softc * skystar, int adr, int state)
{
	int             k;

	for (k = 0; k < 1000; k++, DELAY(500)) {
		if (irdebi(skystar, DEBINOSWAP, adr, 0, 2) == state)
			return 0;
	}

	return -1;
}

/*
 * Load DRAM content
 */
static int 
load_dram(struct skystar_softc * skystar, u_int32_t * data, int len)
{
	int		i;
	int		blocks, rest;
	u_int32_t	base;

	blocks = len / BOOT_MAX_SIZE;
	rest = len % BOOT_MAX_SIZE;
	base = DRAM_START_CODE;

	for (i = 0; i < blocks; i++) {
		if (waitdebi(skystar, BOOT_STATE, BOOTSTATE_BUFFER_EMPTY) < 0)
			return -1;
		/* printf("Writing DRAM block %d\n",i); */
		iwdebi(skystar, DEBISWAB, BOOT_BLOCK, i * (BOOT_MAX_SIZE) + (u_int32_t) data, BOOT_MAX_SIZE);
		iwdebi(skystar, DEBISWAB, BOOT_BASE, htonl(base), 4);
		iwdebi(skystar, DEBINOSWAP, BOOT_SIZE, BOOT_MAX_SIZE, 2);
		iwdebi(skystar, DEBINOSWAP, BOOT_STATE, BOOTSTATE_BUFFER_FULL, 2);
		base += BOOT_MAX_SIZE;
	}

	if (rest > 0) {
		if (waitdebi(skystar, BOOT_STATE, BOOTSTATE_BUFFER_EMPTY) < 0)
			return -1;
		/* printf("Writing last DRAM block\n"); */
		if (rest > 4)
			iwdebi(skystar, DEBISWAB, BOOT_BLOCK, i * (BOOT_MAX_SIZE) + (u_int32_t) data, rest);
		else
			iwdebi(skystar, DEBISWAB, BOOT_BLOCK, i * (BOOT_MAX_SIZE) - 4 + (u_int32_t) data, rest + 4);

		iwdebi(skystar, DEBISWAB, BOOT_BASE, htonl(base), 4);
		iwdebi(skystar, DEBINOSWAP, BOOT_SIZE, rest, 2);
		iwdebi(skystar, DEBINOSWAP, BOOT_STATE, BOOTSTATE_BUFFER_FULL, 2);
	}
	if (waitdebi(skystar, BOOT_STATE, BOOTSTATE_BUFFER_EMPTY) < 0)
		return -1;
	iwdebi(skystar, DEBINOSWAP, BOOT_SIZE, 0, 2);
	iwdebi(skystar, DEBINOSWAP, BOOT_STATE, BOOTSTATE_BUFFER_FULL, 2);
	if (waitdebi(skystar, BOOT_STATE, BOOTSTATE_BOOT_COMPLETE) < 0)
		return -1;
	return 0;
}

/*
 * Load and boot ARM firmware
 */
int 
arm_boot(struct skystar_softc * skystar)
{
	int		result;
	u_int32_t	res;

	iwdebi(skystar, 0x6e0000, DPRAM_BASE, 0x76543210, 4);
#ifdef SKYSTAR_DEBUG
	printf("Booting ARM ...\n");
#endif	
	if ((res=irdebi(skystar, DEBINOSWAP, DPRAM_BASE, 0, 4)) != 0x10325476) {
		printf("bootarm: debi test failed, returned 0x%x!!\n", res);
		return -1;
	}
#ifdef SKYSTAR_DEBUG
        printf("BootARM: debi test OK\n");
        printf("BootARM: load boot code\n");
#endif
	i2c_setgpio(skystar, ARM_IRQ_LINE, GPIO_IRQLO);
	/* i2c_setgpio(dvb, DEBI_DONE_LINE, GPIO_INPUT); */
	/* i2c_setgpio(dvb, 3, GPIO_INPUT); */

	i2c_setgpio(skystar, RESET_LINE, GPIO_OUTLO);
	DELAY(1);

	iwdebi(skystar, DEBISWAB, DPRAM_BASE, (u_int32_t) fw_bootup, fw_bootup_len);
	iwdebi(skystar, DEBINOSWAP, BOOT_STATE, BOOTSTATE_BUFFER_FULL, 2);

	wait_for_debi_done(skystar);
	i2c_setgpio(skystar, RESET_LINE, GPIO_OUTHI);

	DELAY(1000000);
#ifdef SKYSTAR_DEBUG
        printf("BootARM: load dram code\n");
#endif
	result = load_dram(skystar, (u_int32_t *) fw_root, fw_root_len);
	if (result < 0)
		return -1;

	i2c_setgpio(skystar, RESET_LINE, GPIO_OUTLO);
	DELAY(1000);
#ifdef SKYSTAR_DEBUG
        printf("BootARM: load dpram code\n");
#endif
	iwdebi(skystar, DEBISWAB, DPRAM_BASE, (u_int32_t) fw_dpram, fw_dpram_len);
	wait_for_debi_done(skystar);

	i2c_setgpio(skystar, RESET_LINE, GPIO_OUTHI);

	DELAY(100000);
#ifdef SKYSTAR_DEBUG
        printf("DVB: ARM firmware successfully loaded.\n");
#endif
	ARM_ClearIrq(skystar);

	return 0;
}

/*
 * DEBI command polling
 */

/* TODO: process timeouts */
static int 
send_fw_cmd(struct skystar_softc * skystar, u_int16_t * buf, int length)
{
	int             i;
#ifdef SKYSTAR_DEBUG
	printf("DEBUG: send_fw_cmd %x:(%d) called\n", buf[0], length);
#endif
	/* determine correct timeout */
	for (i = 0; i < 1000; i++) {
		if (!rdebi(skystar, DEBINOSWAP, COMMAND, 0, 2))
			break;
		DELAY(1000);
	}
	if (i >= 1000) {
		printf("skystar: send_fw_cmd error: COMMAND\n");
		return -1;
	}			/* else printf("skystar: send_fw_cmd: COMMAND - %d loops\n", i); */
	for (i = 0; i < 100; i++) {
		if (!rdebi(skystar, DEBINOSWAP, HANDSHAKE_REG, 0, 2))	/* ifndef _NOHANDSHAKE */
			break;
		DELAY(1000);
	}
	if (i >= 100) {
		printf("skystar: send_fw_cmd error: HANDSHAKE\n");
		return -1;
	}			/* else printf("skystar: send_fw_cmd:
				 * HANDSHAKE - %d loops\n", i); */
	for (i = 0; i < 100; i++) {
		if (!rdebi(skystar, DEBINOSWAP, MSGSTATE, 0, 2))
			break;
		DELAY(1000);
	}
	if (i >= 100) {
		printf("skystar: send_fw_cmd error: MSGSTATE\n");
		return -1;
	}			/* else printf("skystar: send_fw_cmd: MSGSTATE
				 * - %d loops\n", i); */
	for (i = 2; i < length; i++)
		wdebi(skystar, DEBINOSWAP, COMMAND + 2 * i, buf[i], 2);

	if (length)
		wdebi(skystar, DEBINOSWAP, COMMAND + 2, buf[1], 2);
	else
		wdebi(skystar, DEBINOSWAP, COMMAND + 2, 0, 2);
	wdebi(skystar, DEBINOSWAP, COMMAND, buf[0], 2);
	return 0;
}

static int 
request_fw_cmd(struct skystar_softc * skystar, u_int16_t * Buff, int length, u_int16_t * buf, int n)
{
	u_int16_t         i;
	int             count;

	/* down_interruptible(&dvb->dcomlock); */
	send_fw_cmd(skystar, Buff, length);

	count = 0;
	while (rdebi(skystar, DEBINOSWAP, COMMAND, 0, 2)) {
		DELAY(1000);	/* wait 1 ms */
		if (++count > 100 /* 500 */ ) {
			printf("skystar: request_fw_cmd error: COMMAND\n");
			return /* DVB_ERR_TIMEOUT */ -1;
		}
	}

	count = 0;
	while (rdebi(skystar, DEBINOSWAP, HANDSHAKE_REG, 0, 2)) {	/* ifndef _NOHANDSHAKE */
		DELAY(1000);
		if (++count > 50 /* 200 */ ) {
			printf("skystar: request_fw_cmd error: HANDSHAKE\n");
			return -1;
		}
	}

	for (i = 0; i < n; i++)
		buf[i] = rdebi(skystar, DEBINOSWAP, COM_BUFF + 2 * i, 0, 2);

	/* up(&dvb->dcomlock); */
	return 0;
}


static int 
fw_query(struct skystar_softc * skystar, u_int16_t tag, u_int16_t * Buff, short length)
{
	return request_fw_cmd(skystar, &tag, 0, Buff, length);
}

static int 
send_cmd(struct skystar_softc * skystar, int type, int com, int num,...)
{
	va_list         args;
	u_int16_t         buf[num + 2];
	int             i;

	buf[0] = ((type << 8) + com);
	buf[1] = num;

	if (num) {
		va_start(args, num);
		for (i = 0; i < num; i++)
			buf[i + 2] = va_arg(args, int);
		va_end(args);
	}
	/* TODO: synchronise access */
	return send_fw_cmd(skystar, buf, num + 2);
}

u_int32_t 
get_fw_version(struct skystar_softc * skystar)
{
	u_int16_t         buf[16];
	u_int32_t         tag = ((COMTYPE_REQUEST << 8) + ReqVersion);

	fw_query(skystar, tag, buf, 16);
	
	skystar->arm.arm_fw = (buf[0] << 16) + buf[1];
	skystar->arm.arm_rtsl = (buf[2] << 16) + buf[3];
	skystar->arm.arm_vid = (buf[4] << 16) + buf[5];
	skystar->arm.arm_app = (buf[6] << 16) + buf[7];
	skystar->arm.avtype = (buf[8] << 16) + buf[9];

#ifdef SKYSTAR_DEBUG
	printf("dvb: fw ver = 0x%08x, rtsl ver = 0x%08x, vid = 0x%08x, app = 0x%08x, avtype =  0x%08x\n", skystar->arm.arm_fw, 
			skystar->arm.arm_rtsl, skystar->arm.arm_vid, skystar->arm.arm_app, skystar->arm.avtype);
#endif
	printf(" firmware 0x%08x",  skystar->arm.arm_fw);
	
	return skystar->arm.arm_fw;
}

struct dvb_filter *
hw_filter_alloc(struct skystar_softc * skystar)
{
	int             i;

	for (i = 0; i < MAXFILT - 3; i++) /* why it's -3 ? */
		if (!skystar->filter[i].state) {
			skystar->filter[i].state = 1;
			return &skystar->filter[i];
		}
	return NULL;
}

int 
hw_filter_set(struct skystar_softc * skystar, struct bitfilter * filter)
{
	u_int16_t		buf[20];
	int			ret;
	u_int16_t		handle;
	struct dvb_filter	*dvbfilter;

	if (!(dvbfilter = hw_filter_alloc(skystar))) {
		printf("skystar: SetBitFilter => unable to alloc HW filter\n");
		return -1;
	}
	bcopy(filter->data, buf + 4, 32);
	buf[0] = (COMTYPE_PID_FILTER << 8) + AddPIDFilter;
	buf[1] = 16;
	buf[2] = filter->pid;
	buf[3] = filter->mode;

	ret = request_fw_cmd(skystar, buf, 20, &handle, 1);
	if (ret < 0) {
		printf("skystar: SetBitFilter => Command failed\n");
		return ret;
	}
	filter->handle = handle;
	skystar->hfilter[handle] = dvbfilter;
	dvbfilter->pid = filter->pid;
	dvbfilter->flags = filter->flags;

	return ret;
}

int 
hw_filter_free(struct skystar_softc * skystar, struct dvb_filter * filter)
{
	if (!filter || !filter->state)
		return -1;
	filter->state = 0;
	return 0;
}

int 
hw_filter_stop(struct skystar_softc * skystar, u_int16_t handle)
{
	u_int16_t         buf[3];
	u_int16_t         answ[2];
	int             ret;

	if (hw_filter_free(skystar, skystar->hfilter[handle]) < 0) {
		return -1;
	}
	skystar->hfilter[handle] = NULL;

	buf[0] = (COMTYPE_PID_FILTER << 8) + DelPIDFilter;
	buf[1] = 1;
	buf[2] = handle;
	ret = request_fw_cmd(skystar, buf, 3, answ, 2);

	if (answ[1] != handle) {
		printf("skystar: filter shutdown error :%d\n", answ[1]);
		ret = -1;
	}
	return ret;
}

int 
hw_filters_stop(struct skystar_softc * skystar)
{
	u_int16_t         handle;

	for (handle = 0; handle < MAXFILT; handle++)
		hw_filter_stop(skystar, handle);
	return 0;
}

void 
hw_22k_set(struct skystar_softc * skystar, int state)
{
#ifdef SKYSTAR_DEBUG
	printf("skystar: hw_22k_set: %d\n", state);
#endif
	send_cmd(skystar, COMTYPE_AUDIODAC, (state ? ON22K : OFF22K), 0);
}

void 
hw_pids_set(struct skystar_softc * skystar, u_int16_t vpid, u_int16_t apid, u_int16_t ttpid)
{	
	u_int16_t	pcrpid = 0xfffe; /* PCR pid, TODO: make it changable */
	
	send_cmd(skystar, COMTYPE_PIDFILTER, MultiPID, 5, pcrpid, vpid, apid, ttpid, 0);
}

void 
hw_diseqc_set(struct skystar_softc * skystar, int input)
{
#ifdef SKYSTAR_DEBUG
	printf("skystar: hw_diseqc_set: %d\n", input);
#endif
	if (input < 0 || input > 15)
		return;
	send_cmd(skystar, COMTYPE_AUDIODAC, SendDiSEqC, 6, 4,
	       (input & 4) ? 1 : 0,
	       0xe0, 0x10, 0x38, 0xf0 + input);

	/* Send again with repeat flag (0xe1) set */
	send_cmd(skystar, COMTYPE_AUDIODAC, SendDiSEqC, 6, 4,
	       (input & 4) ? 1 : 0,
	       0xe1, 0x10, 0x38, 0xf0 + input);
	DELAY(500000);
}

#if 0
static void 
SetVoltage(struct skystar_softc * skystar, int power, int voltage)
{
	struct DVB_VOLTAGE volt;
#ifdef SKYSTAR_DEBUG
	printf("skystar: SetVoltage %d %d.\n", power, voltage);
#endif
	volt.power = power;
	volt.volt = voltage;
	dvb_command(skystar, DVB_SET_VOLT, &volt);
}


static int 
AFC(struct skystar_softc * skystar)
{
	/*
	 * TODO: implement this! (add skystar->front, set it in
	 * ioctl->SetFront and bzero in _attach)
	 */
	u_char          sync;
	int             i;
	u_int32_t          sfreq = dvb->front.curfreq;
	u_int32_t          soff = dvb->front.srate / 16;

	ddelay(10);
	decoder_command(dvb, DVB_GET_FRONTEND, &dvb->front);
	if ((dvb->front.sync & 0x1f) == 0x1f)
		return 0;

	decoder_command(dvb, DVB_RESET, 0);
	ddelay(40);
	decoder_command(dvb, DVB_GET_FRONTEND, &dvb->front);
	if ((dvb->front.sync & 0x1f) == 0x1f)
		return 0;
	decoder_command(dvb, DVB_RESET, 0);
	ddelay(40);
	decoder_command(dvb, DVB_GET_FRONTEND, &dvb->front);

	for (i = 2; i < 10; i++) {
		sync = dvb->front.sync;
		//printk("%02x \n", sync);
		if ((sync & 0x1f) == 0x1f)
			break;
		if (i & 1)
			sfreq = dvb->front.freq + soff * (i / 2);
		else
			sfreq = dvb->front.freq - soff * (i / 2);
		tuner_command(dvb, TUNER_SET_TVFREQ, &sfreq);
		mdelay(1);
		decoder_command(dvb, DVB_RESET, 0);
#ifdef SKYSTAR_DEBUG
		printf("sfreq = %d \n", sfreq);
#endif
		DELAY(1000);
		decoder_command(dvb, DVB_GET_FRONTEND, &dvb->front);
	}
	if (i == 10)
		return -1;
#ifdef SKYSTAR_DEBUG
	printf("afc = %d \n", dvb->front.afc);
#endif
	if (dvb->front.afc) {
		sfreq -= dvb->front.afc;
		tuner_command(dvb, TUNER_SET_TVFREQ, &sfreq);
		mdelay(1);
		decoder_command(dvb, DVB_RESET, 0);
#ifdef SKYSTAR_DEBUG
		printf("sfreq = %d \n", sfreq);
#endif
		decoder_command(dvb, DVB_GET_FRONTEND, &dvb->front);
#ifdef SKYSTAR_DEBUG
		printf("afc = %d \n", dvb->front.afc);
#endif		
	}
	dvb->front.curfreq = sfreq;

	return 0;
}
#endif

/* send to DAC module */
int 
hw_dac_send(struct skystar_softc * skystar, u_char addr, u_char data)
{
	return send_cmd(skystar, COMTYPE_AUDIODAC, AudioDAC, 2, addr, data);
}

int 
hw_volume_set(struct skystar_softc * skystar, u_char volleft, u_char volright)
{
	int             err;
#if 0
	if (skystar->front_type != DVBS)
		return -EOPNOTSUPP;
#endif
	if (volleft > 0x45)
		volleft = 0x45;
	if (volright > 0x45)
		volright = 0x45;
	err = hw_dac_send(skystar, 3, 0x80 + volleft);
	if (err)
		return err;
	return hw_dac_send(skystar, 4, volright);
}

/* DEBI interrupt handler part */
void
debi_intr(struct skystar_softc * skystar)
{
	int             type = skystar->debitransfer;
	int             handle = (type >> 8) & 0x1F;

	/* printf( "DEBUG: debi_intr (%d:%d)\n", handle, type&0xFF ); */

	if (type == -1) {
		printf("skystar: unexpected DEBI IRQ.\n");
		return;
	}
	skystar->debitransfer = -1;


	write_long(skystar, IER, read_long(skystar, IER) & ~MASK_19);	/* disable DEBI interrupts  */
	write_long(skystar, ISR, MASK_19);	/* clear irq */

	switch (type & 0xFF) {
	case DATA_MPEG_RECORD:
#ifdef SKYSTAR_DEBUG
		printf("DEBUG: DEBI irq => MPEG_RECORD. (unsupported)\n");
#endif
		break;
		/*
		 * { if (dvb->ps_ts_stream.type != AV_PES_STREAM)
		 * av_pes_to_pes(dvb, (u8 *)dvb->saa->debi, dvb->debilen);
		 * else dvb_buf_put(&dvb->avin, (u8 *)dvb->saa->debi,
		 * dvb->debilen); wake_up_interruptible(&dvb->avin.queue);
		 * break; }
		 */
	case DATA_COMMON_INTERFACE:
#ifdef SKYSTAR_DEBUG
		printf("DEBUG: DEBI irq => COMMON INTERFACE. (unsupported)\n");
#endif
		break;
	case DATA_TELETEXT:
#ifdef SKYSTAR_DEBUG
		printf("DEBUG: DEBI irq => TELETEXT. (unsupported)\n");
#endif
		/*
		 * fsection(dvb, handle, (void*)dvb->saa->debi,
		 * dvb->debilen);
		 */
		break;
	case DATA_FSECTION:
	case DATA_PIPING:
#ifdef SKYSTAR_DEBUG
		printf("DEBUG: DEBI irq => FSECTION/PIPING. (unsupported)\n");
#endif
		if ((handle >= MAXFILT) || !skystar->hfilter[handle]) {
			printf("DEBI IRQ => invalid handle.\n");
			break;
		}
		/*
		if (skystar->hfilter[handle]->flags&FILTER_UDP)
			net_fsection(&skystar->net, skystar->debi, skystar->debilen, 3000+handle);
			if (skystar->hfilter[handle]->flags&FILTER_MEM)
				fsection(skystar, handle, skystar->debi, skystar->debilen);*/
		break;
	case DATA_IPMPE:
		if ((handle >= MAXFILT) || !skystar->hfilter[handle]) {
			printf("skystar: invalid filter for DATA_IPMPE.\n");
			break;
		}
		if (skystar->hfilter[handle]->flags & FILTER_MEM)
			fsection(skystar, handle, (u_char *) skystar->debi, skystar->debilen);
		/* disable GPIO int ? */
		dvb_ipmpe(skystar, (u_char *) skystar->debi, skystar->debilen);
		/* enable GPIO int ? */
		break;
	case DATA_MPEG_PLAY:
#ifdef SKYSTAR_DEBUG
		printf("DEBUG: DEBI irq => MPEG_PLAY. (unsupported)\n");
#endif
		break;
	case DATA_BMP_LOAD:
#ifdef SKYSTAR_DEBUG
		printf("DEBUG: DEBI irq => BMP_LOAD. (unsupported)\n");
#endif
		break;
	default:
#ifdef SKYSTAR_DEBUG
		printf("DEBUG: DEBI irq => unknown\n");
#endif
		break;
	}
	ARM_ClearMailBox(skystar);
}

/* GPIO interrupt handler	 */
void
gpio_intr(struct skystar_softc * skystar)
{
	int             len;
	if (skystar->debitransfer != -1) {
		printf("skystar: unexpected GPIO0 irq\n");
	}
	
	/* read the type and length of the transfer */
	skystar->debitransfer = irdebi(skystar, DEBINOSWAP, IRQ_STATE, 0, 2);
	skystar->debilen = irdebi(skystar, DEBINOSWAP, IRQ_STATE_EXT, 0, 2);

	write_long(skystar, IER, read_long(skystar, IER) & ~MASK_19);
	ARM_ClearIrq(skystar);

	/* process data block */
	switch (skystar->debitransfer & 0xff) {
	case DATA_MPEG_RECORD:
#ifdef SKYSTAR_DEBUG
		printf("DEBUG: GPIO irq => MPEG_RECORD. (unsupported)\n");
#endif
		break;
	case DATA_MPEG_PLAY:
#ifdef SKYSTAR_DEBUG
		printf("DEBUG: GPIO irq => MPEG_PLAY. (unsupported)\n");
#endif
		break;
	case DATA_TELETEXT:
	case DATA_FSECTION:
	case DATA_DEBUG_MESSAGE:
	case DATA_COMMON_INTERFACE:
	case DATA_IPMPE:
	case DATA_PIPING:
		len = (skystar->debilen + 3) & (~3);
		/* printf("DEBUG: GPIO IRQ => Data (%d)\n", len); */
		if (!len || len > 6 * 1024)
			break;
		/* <-> */
		/*
		 * enable DEBI interrupt and initiate transfer from internal
		 * card buffer to skystar->debi. after transfer has been
		 * finished, interrupt will be raised
		 */
		/*
		 * write_long (skystar, IER, read_long (skystar, IER) | MASK_19
		 * );
		 */
		write_long(skystar, IER, read_long(skystar, IER) | MASK_19);
		irdebi(skystar, DEBISWAB, DATA_BUFF_BASE, 0, len);
		return;
	case DATA_BMP_LOAD:
#ifdef SKYSTAR_DEBUG
		printf("DEBUG: GPIO IRQ => BMP_LOAD (unsupported)\n");
#endif
		break;
	default:
#ifdef SKYSTAR_DEBUG
		printf("skystar: gpio_intr => unsupported transfer type.\n");
#endif
		break;
	}

	/* do not process data. throw away received packet. */
	ARM_ClearMailBox(skystar);
	skystar->debitransfer = -1;
	/* TODO: unlock DEBI */
}



/*
 * DEBI command polling
 */

/* TODO: process timeouts */

void
fsection(struct skystar_softc * skystar, int handle, u_char * data, u_int16_t len)
{
#ifdef SKYSTAR_DEBUG
	printf("DEBUG: fsection (not implemented)\n");
#endif
	return;
}

