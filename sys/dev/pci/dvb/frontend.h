/*
 * OpenBSD 4.8 driver for Skystar1 DVB card rev. 1.3 and 1.5
 * based on code by:
 * Copyright (c) 2002 Alexander Romanov ported to OpenBSD 3.7 for Alloyant Technologies
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

/* common demodulator data and constants */

/* frontend type */
#define FRONT_NONE			0	/* no frontend or unknown	 */
#define FRONT_TV			1	/* TV frontend			 */
#define FRONT_DVBS			2	/* DVB Satellite frontend	 */
#define FRONT_DVBC			3	/* not supported yet		 */
#define FRONT_DVBT			4	/* not supported yet		 */

/* frontend.channel_flags */
#define DVB_CHANNEL_FTA		0
#define DVB_CHANNEL_CA		1

/* frontend.sync */
#define DVB_SYNC_SIGNAL		1
#define DVB_SYNC_CARRIER	2
#define DVB_SYNC_VITERBI	4
#define DVB_SYNC_FSYNC		8
#define DVB_SYNC_FRONT		16
#define DVB_SYNC_SPEC_INV	32	/* AR */

/* frontend.flags */
#define FRONT_TP_CHANGED	1
#define FRONT_FREQ_CHANGED	2
#define FRONT_RATE_CHANGED	4

/* frontend types */
#define FE_QPSK				1
#define FE_QAM				2
#define FE_OFDM				3

/* frontend caps	 */
#define FE_IS_STUPID				0
#define FE_CAN_INVERSION_AUTO			0x1
#define FE_CAN_FEC_1_2				0x2
#define FE_CAN_FEC_2_3				0x4
#define FE_CAN_FEC_3_4				0x8
#define FE_CAN_FEC_4_5				0x10
#define FE_CAN_FEC_5_6				0x20
#define FE_CAN_FEC_6_7				0x40
#define FE_CAN_FEC_7_8				0x80
#define FE_CAN_FEC_8_9				0x100
#define FE_CAN_FEC_AUTO				0x200
#define FE_CAN_QPSK				0x400
#define FE_CAN_QAM_16				0x800
#define FE_CAN_QAM_32				0x1000
#define FE_CAN_QAM_64				0x2000
#define FE_CAN_QAM_128				0x4000
#define FE_CAN_QAM_256				0x8000
#define FE_CAN_QAM_AUTO				0x10000
#define FE_CAN_TRANSMISSION_MODE_AUTO		0x20000
#define FE_CAN_BANDWIDTH_AUTO			0x40000
#define FE_CAN_GUARD_INTERVAL_AUTO		0x80000
#define FE_CAN_HIERARCHY_AUTO			0x100000
#define FE_CAN_MUTE_TS				0x80000000

/* spectral inversion control */
#define INVERSION_OFF	0
#define INVERSION_ON	1
#define INVERSION_AUTO	2

/* FEC control */
#define FEC_AUTO		0
#define FEC_1_2			1
#define FEC_2_3			2
#define FEC_3_4			3
#define FEC_5_6			4
#define FEC_7_8			5
#define FEC_NONE		6

/* LNB voltage control	 */
#define SEC_VOLTAGE_13	0
#define SEC_VOLTAGE_18	1

/* LNB 22k control		 */
#define SEC_TONE_OFF	0
#define SEC_TONE_ON	1

/* LNB power control	 */
#define POWER_OFF	0
#define POWER_ON	1

/* Tuners i2c */
#define TSA5059_I2C_ADDR	0xC2 >> 1
#define STV0299_I2C_ADDR	0xD0 >> 1

/* frontend info	 */
struct dvb_frontend_info {
	char		name[128];
	u_int32_t	type;
	u_int32_t	frequency_min;	/* KHz */
	u_int32_t	frequency_max;	/* KHz */
	u_int32_t	frequency_stepsize;	/* KHz */
	u_int32_t	frequency_tolerance;
	u_int32_t	symbol_rate_min;	/* sym/sec */
	u_int32_t	symbol_rate_max;	/* sym/sec */
	u_int32_t	symbol_rate_tolerance;	/* ppm */
	u_int32_t	notifier_delay;	/* ms */
	u_int32_t	caps;
};


/* frontend state */
struct frontend {
	int		type;	/* type of frontend (tv tuner, dvb tuner/decoder, etc. */
	/* Sat line control */
	int		power;	/* LNB power 0=off/pass through, 1=on */
	int		volt;	/* 14/18V (V=0/H=1) */
	int		ttk;	/* 22KHz on/off	 */
	int		diseqc;	/* Diseqc input select */
	/* signal decoding, transponder info */
	u_int32_t	freq;	/* offset frequency (from local oscillator) in KHz */
	u_int32_t	srate;	/* Symbol rate char/sec */
	int		afc;	/* automatic frequency correction 1-on, 0-off */
	int		qam;	/* QAM mode for cable decoder, sat is always QPSK */
	int		inv;	/* Inversion */
	int		fec;	/* Forward Error Correction */
	/* channel info */
	u_int16_t	video_pid;
	u_int16_t	audio_pid;
	u_int32_t	ttext_pid;	/* Teletext PID */
	u_int16_t	pnr;	/* Program number = Service ID */
	int		channel_flags;
	/* status information */
	u_int		sync;	/* sync from decoder */
	long		afcOffset;	/* frequency offset in KHz */
	u_int16_t	agc;	/* gain (signal strength) */
	u_int16_t	nest;	/* noise estimation */
	u_int16_t	vber;	/* viterbi bit error rate */
	u_int16_t	err;	/* error count - from last call */
};

/* bitfilter.flags */
#define FILTER_UDP	1	/* write data to UDP socket */
#define FILTER_MEM	2	/* write data to memory buffer */

struct bitfilter {
	u_int16_t	pid;
	u_int16_t	data[16];
	u_int16_t	mode;
	u_int16_t	handle;
	u_int16_t	flags;
	struct in_addr	dest;
};



struct dvb_filter {
	int		state;
	int		flags;
	int		type;

	u_int16_t	pid;
	u_char		value[32];
	u_char		mask[32];
};

#define BASE_VIDIOCPRIVATE	195	/* 192-255 are private */
#define VIDIOCGFRONTEND		_IOR('v',  BASE_VIDIOCPRIVATE+0, struct frontend)
#define VIDIOCSFRONTEND		_IOW('v',  BASE_VIDIOCPRIVATE+1, struct frontend)
#define VIDIOCSBITFILTER	_IOWR('v', BASE_VIDIOCPRIVATE+2, struct bitfilter)
#define VIDIOCSSHUTDOWNFILTER 	_IOW('v',  BASE_VIDIOCPRIVATE+3, u_int16_t)

#define DVB_SET_FRONTEND	_IOW('v',  BASE_VIDIOCPRIVATE+0x10, struct frontend)
#define DVB_GET_FRONTEND	_IOR('v',  BASE_VIDIOCPRIVATE+0x11, struct frontend)
#define DVB_GET_INFO		_IOR('v',  BASE_VIDIOCPRIVATE+0x12, struct dvb_frontend_info *)
#define DVB_RESET               _IOR('v',  BASE_VIDIOCPRIVATE+0x13, void *)
#define DVB_WRITEREG            _IOR('v',  BASE_VIDIOCPRIVATE+0x14, u_char *)
#define DVB_READREG             _IOR('v',  BASE_VIDIOCPRIVATE+0x15, u_char *)
