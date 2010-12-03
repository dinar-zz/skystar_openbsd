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
#include <sys/ioctl.h>
#include <machine/bus.h>
#include <uvm/uvm_extern.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/route.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>

#include "frontend.h"
#include "dvb.h"
#include "skystar_dev.h"
#include "saa7146.h"

#define VES_I2C_ADDR		0x10 >> 1	/* interperet as 7-bit */
#define VES_I2C_FLAGS	0


/*
 * VES1893
 */

#ifdef SKYSTAR_DEBUG
#define BSRV2_DEBUG
#endif

static u_char   Init1893Tab[] = {
	0x01, 0xA4, 0x35, 0x81, 0x2A, 0x0d, 0x55, 0xC4,
	0x09, 0x69, 0x00, 0x86, 0x4c, 0x28, 0x7F, 0x00,
	0x00, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x80, 0x00, 0x31, 0xb0, 0x14, 0x00, 0xDC, 0x20,
	0x81, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x55, 0x00, 0x00, 0x7f, 0x00
};

static struct dvb_frontend_info bsrv2_info = {
	name:"Alps BSRV2",
	type:FE_QPSK,
	frequency_min:950000,
	frequency_max:2150000,
	frequency_stepsize:250,	/* kHz for QPSK frontends */
	frequency_tolerance:29500,
	symbol_rate_min:1000000,
	symbol_rate_max:45000000,
	/* symbol_rate_tolerance: ???, */
	notifier_delay:50,	/* 1/20 s */
	caps:FE_CAN_INVERSION_AUTO |
	FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
	FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
	FE_CAN_QPSK
};

static int 
VES_writereg(struct skystar_softc * skystar, int reg, int data)
{
	int             ret;
	unsigned char   msg[] = {0x00, 0x1f, 0x00};
	struct i2c_msg  m;

	msg[1] = reg;
	msg[2] = data;

	/* simulate linux i2c_master_send call */
	m.addr = VES_I2C_ADDR;
	m.flags = VES_I2C_FLAGS & I2C_M_TEN;
	m.len = 3;
	m.buf = msg;
	/* TODO: ?lock i2c */
	ret = i2c_transfer(skystar, &m, 1);
	/* TODO: ?unlock i2c */
	if (ret != 3 && ret != 1) {
		printf("%s VES1893: i2c write error: rc == %d.\n", skystar->ss_dev.dv_xname, ret);
		return -1;
	}
	return ret;
}

static u_char 
VES_readreg(struct skystar_softc * skystar, u_char reg)
{
	unsigned char   mm1[] = {0x00, 0x1e};
	unsigned char   mm2[] = {0x00};
	struct i2c_msg  msgs[2];

	msgs[0].flags = 0;
	msgs[1].flags = I2C_M_RD;
	msgs[0].addr = msgs[1].addr = VES_I2C_ADDR;
	mm1[1] = reg;
	msgs[0].len = 2;
	msgs[1].len = 1;
	msgs[0].buf = mm1;
	msgs[1].buf = mm2;
	/* TODO: ?lock i2c */
	i2c_transfer(skystar, msgs, 2);
	/* TODO: ?unlock i2c */
	/* TODO: check error */

	return mm2[0];
}

/*
	PLL write
*/
static int 
sp5659_write(struct skystar_softc * skystar, u_char data[4])
{
	struct i2c_msg  m = {addr:0x61, flags:0, buf:data, len:4};

	if (1 != i2c_transfer(skystar, &m, 1)) {
		printf("%s sp5659_write() error!\n", skystar->ss_dev.dv_xname);
		return -1;
	}
	return 0;
}

/**
 *   set up the downconverter frequency divisor for a
 *   reference clock comparision frequency of 125 kHz.
 *   freq is KHz !!!
 */
static void 
sp5659_set_tv_freq(struct skystar_softc * skystar, u_int32_t freq, u_char pwr)
{
	u_int32_t          div = (freq + 479500) / 125;	/* was: (freq + 479500)
							 * / 125;	 */
	u_char          buf[4] = {(div >> 8) & 0x7f, div & 0xff, 0x95, (pwr << 5) | 0x30};

	if (freq == skystar->front.freq)
		return;

	sp5659_write(skystar, buf);
	skystar->front.freq = freq;
	return;
}


#ifdef BSRV2_DEBUG
static int 
VES_dump(struct skystar_softc * skystar)
{
	int             i;

	printf("VES1893: DUMP\n");
	printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
	printf("===============================================\n");
	for (i = 0; i < 54; i++) {
		printf("%02x ", VES_readreg(skystar, i));
		if ((i & 0xF) == 0xF)
			printf("\n");
	}
	printf("\n");
	return 0;
}
#endif

/*
	VES1893 initialization
*/
static int 
VES_init(struct skystar_softc * skystar)
{
	int             i;

	for (i = 0; i < 54; i++)
		VES_writereg(skystar, i, Init1893Tab[i]);

	/* TDOD: initialize this values accordingly to init registers values */
	skystar->dvb.bsrv2.ctr = Init1893Tab[0x1f];
	skystar->front.freq = 0;
	skystar->front.srate = 0;
	skystar->front.fec = 9;
	skystar->front.inv = 0;
	skystar->front.power = 0;
	skystar->front.volt = 0;
#ifdef BSRV2_DEBUG
	VES_dump(skystar);
#endif
	return 0;
}

/*
	VES1893 reset
*/
static void 
VES_reset(struct skystar_softc * skystar)
{
	/* DPR(("clrbit1893\n"); */
	DELAY(20 * 1000);
	VES_writereg(skystar, 0, Init1893Tab[0] & 0xfe);
	VES_writereg(skystar, 0, Init1893Tab[0]);
}

#if 0
static int 
VES_SetControl(struct skystar_softc * skystar, u_char val)
{
	if (skystar->dvb.bsrv2.ctr == val)
		return 0;
	if (val & 0x20)
		if (!(skystar->dvb.bsrv2.ctr & 0x20)) {
			DELAY(10 * 1000);
			skystar->dvb.bsrv2.srate = 0;	/* force symbolrate
							 * reload */
		}
	skystar->dvb.bsrv2.ctr = val;
	VES_writereg(skystar, 0x1f, val);
	return 0;
}
#endif

/*
	VES1893 set FEC
*/
static int 
VES_setFEC(struct skystar_softc * skystar, u_char fec)
{
	if (fec >= 8)
		fec = 8;
	/*
	 * DEBUG: uncomment this if (skystar->front.fec==fec) return 0;
	 */
	/* TODO: move writereg before initializing skystar->dvb.bsrv2.fec */
	/* if we can't set FEC, leave structure unchanged */
	skystar->front.fec = fec;
	return VES_writereg(skystar, 0x0d, skystar->front.fec);
}

/*
	VES1893 set power
*/
static void 
VES_setPower(struct skystar_softc * skystar, u_char power, u_char volt)
{
	if (power == skystar->front.power && volt == skystar->front.volt)
		return;
#ifdef BSRV2_DEBUG
	printf("%s bsrv2: setting power:%u volt:%u\n", skystar->ss_dev.dv_xname, power, volt);
#endif
	VES_writereg(skystar, 0x1f, (power ? 0x20 : 0) | (volt ? 0x10 : 0));
	skystar->front.power = power;
	skystar->front.volt = volt;
}

/*
	VES1893 set inversion
*/
static void 
VES_setInversion(struct skystar_softc * skystar, u_char inv)
{
	if (inv == skystar->front.inv)
		return;

	VES_writereg(skystar, 0x0c, Init1893Tab[0x0c] ^ (inv ? 0x40 : 0x00));
	skystar->front.inv = inv;
	/* AR: was so: VES_ClrBit1893(skystar); */
}

/*
	VES1893 set symbol rate
*/
static void 
VES_setSymbolrate(struct skystar_softc * skystar, u_int32_t srate /* , int doclr */ )
{
	u_int32_t          BDR;
	u_int32_t          ratio;
	u_char          ADCONF, FCONF, FNR;
	u_int32_t          BDRI;
	u_int32_t          tmp;

	if (skystar->front.srate == srate) {
		/* ClrBit1893(client); */
		return;
	}
	/* DPR(("setsymbolrate %d\n", (int)srate); */

	if (srate > 90100000UL / 2)
		srate = 90100000UL / 2;
	if (srate < 500000)
		srate = 500000;
	skystar->front.srate = srate;

#define MUL (1UL<<24)
#define FIN (90106000UL>>4)
	ratio = (srate << 4) / FIN;

	tmp = ((srate << 4) % FIN) << 8;
	ratio = (ratio << 8) + tmp / FIN;

	tmp = (tmp % FIN) << 8;
	ratio = (ratio << 8) + tmp / FIN;

	FNR = 0xFF;

	if (ratio < MUL / 3)
		FNR = 0;
	if (ratio < (MUL * 11) / 50)
		FNR = 1;
	if (ratio < MUL / 6)
		FNR = 2;
	if (ratio < MUL / 9)
		FNR = 3;
	if (ratio < MUL / 12)
		FNR = 4;
	if (ratio < (MUL * 11) / 200)
		FNR = 5;
	if (ratio < MUL / 24)
		FNR = 6;
	if (ratio < (MUL * 27) / 1000)
		FNR = 7;
	if (ratio < MUL / 48)
		FNR = 8;
	if (ratio < (MUL * 137) / 10000)
		FNR = 9;

	if (FNR == 0xFF) {
		ADCONF = 0x89;	/* bypass Filter */
		FCONF = 0x80;	/* default */
		FNR = 0;
	} else {
		ADCONF = 0x81;
		FCONF = 0x88 | (FNR >> 1) | ((FNR & 0x01) << 5);	/* default | DFN | AFS */
	}

	/* (int)( ((1<<21)<<(FNR>>1)) * (float)(srate) / 90100000.0 + 0.5); */
	/* (int)(32 * 90100000.0 / (float)(srate) / (1<<(FNR>>1)) + 0.5); */


	BDR = (((ratio << (FNR >> 1)) >> 2) + 1) >> 1;
	BDRI = (((90100000UL << 4) / ((srate << (FNR >> 1)) >> 2)) + 1) >> 1;
	/*
	    DPR(("FNR= %d\n", FNR);
	    DPR(("ratio= %08x\n", (unsigned)ratio);
	    DPR(("BDR= %08x\n", (unsigned)BDR);
	    DPR(("BDRI= %02x\n", (unsigned)BDRI);
	*/
	if (BDRI > 0xFF)
		BDRI = 0xFF;

	VES_writereg(skystar, 6, 0xff & BDR);
	VES_writereg(skystar, 7, 0xff & (BDR >> 8));
	VES_writereg(skystar, 8, 0x0f & (BDR >> 16));

	VES_writereg(skystar, 9, BDRI);
	VES_writereg(skystar, 0x20, ADCONF);
	VES_writereg(skystar, 0x21, FCONF);

	if (srate < 6000000)
		VES_writereg(skystar, 5, Init1893Tab[0x05] | 0x80);
	else
		VES_writereg(skystar, 5, Init1893Tab[0x05] & 0x7f);

	VES_writereg(skystar, 0, 0);
	VES_writereg(skystar, 0, 1);
	/*
	    if (doclr)
	        VES_reset(skystar);
	   */
	return;
}

/*
	detect and initialize bsrv2 chip
*/
int 
dvb_bsrv2_detect(struct skystar_softc * skystar)
{
	if ((VES_readreg(skystar, 0x1e) & 0xf0) != 0xd0) {
#ifdef BSRV2_DEBUG
		printf("%s VES_readreg failed on initialization.\n", skystar->ss_dev.dv_xname);
#endif
		return DVB_NONE;
	}
	VES_init(skystar);
	return BSRV2;
}

/*
	bsrv2 commands
*/
int 
dvb_bsrv2_command(struct skystar_softc * skystar, unsigned int cmd, void *arg)
{
	switch (cmd) {
		/*
		case DVB_RESET:
		{
			VES_ClrBit1893(skystar);
			break;
		}
		*/
		case DVB_GET_INFO:
		{
			struct dvb_frontend_info *info = (struct dvb_frontend_info *) arg;
			*info = bsrv2_info;
			break;
		}
	case DVB_SET_FRONTEND:
		{
			/* all but 22k - this frontend can't do this */
			struct frontend *front = (struct frontend *) arg;
			VES_setPower(skystar, front->power, front->volt);
			sp5659_set_tv_freq(skystar, front->freq, 0);
			VES_setInversion(skystar, front->inv);
			VES_setFEC(skystar, front->fec);
			VES_setSymbolrate(skystar, front->srate);
			VES_reset(skystar);
			break;
		}
	case DVB_GET_FRONTEND:
		{
			u_char          tmp;
			/* TODO: frequency */
			struct frontend *front = (struct frontend *) arg;
			front->afc = ((int) ((char) (VES_readreg(skystar, 0x0a) << 1))) / 2;
			front->afc = (front->afc * (int) (skystar->front.srate / 8)) / 16;

			/* agc (signal strength)	 */
			front->agc = (VES_readreg(skystar, 0x0b) << 8);

			/* sync */
			tmp = VES_readreg(skystar, 0x0e);
			front->sync = 0;
			if (tmp & 1)
				front->sync |= DVB_SYNC_SIGNAL;
			if (tmp & 2)
				front->sync |= DVB_SYNC_CARRIER;
			if (tmp & 4)
				front->sync |= DVB_SYNC_VITERBI;
			if (tmp & 8)
				front->sync |= DVB_SYNC_FSYNC;
			if ((tmp & 0x1f) == 0x1f)
				front->sync |= DVB_SYNC_FRONT;

			/* signal/noice ratio */
			front->nest = (VES_readreg(skystar, 0x1c) << 8);

			/* viterbi eror rate */
			front->vber = VES_readreg(skystar, 0x15);
			front->vber |= (VES_readreg(skystar, 0x16) << 8);
			front->vber |= (VES_readreg(skystar, 0x17) << 16);

			/* uncorrected blocks counter */
			front->err = VES_readreg(skystar, 0x18) & 0x7f;
			if (front->err == 0x7f)
				front->err = 0xffff;	/* overflow */
			VES_writereg(skystar, 0x18, 0x00);	/* reset the counter */
			VES_writereg(skystar, 0x18, 0x80);	/* dto. */

			/* fec */
			if ((front->fec == 8) && ((front->sync & 0x1f) == 0x1f))
				front->fec = (VES_readreg(skystar, 0x0d) >> 4) & 0x07;

			break;
		}
	default:
		return -1;
	}

	return 0;
}
