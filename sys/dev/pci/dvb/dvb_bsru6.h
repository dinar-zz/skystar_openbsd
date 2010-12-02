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

/*
 *	stv0299c cpecific definitions
 */

struct DVB_BSRU6 {
	u_char          aclk;
	u_char          bclk;
};

int	dvb_bsru6_detect(struct skystar_softc *);
int	dvb_bsru6_command(struct skystar_softc *, unsigned int, void *);

/* STV0299 registers */

#define ID		0x00 /* Identification */
#define RCR		0x01 /* Reference clock */
#define MCR		0x02 /* Master clock */
#define	ACR		0x03 /* Auxiliary clock */
#define F22FR		0x04 /* F22 frequency */
#define I2CRPT		0x05 /* i2c repeater */
/* DAC registers */
#define DACR1		0x06 
#define DACR2		0x07 
/* DiSEQc registers */
#define DISEQC		0x08 /* lock control */
#define DISEQCFIFO	0x09 /* fifo */
#define DISEQCSTATUS	0x0A /* status */

#define IOCFG		0x0C /* i/o configuration */
#define AGC1C		0x0D /* AGC1 control */
#define RTC		0x0E /* Timing loop */
#define AGC1R		0x0F /* AGC1 reference */
#define AGC2O		0x10 /* AGC2 offset control */
#define TLSR		0x11 /* Timing lock sessting */
#define CFD		0x12 /* Carrier frequency detector */
#define ACLC		0x13 /* Alpha carrier and noise estimator */
#define BCLC		0x14 /* Beta carrier */
#define CLDT		0x15 /* Carrier lock detector treshold */
#define AGC1I		0x16 /* AGC1 Integrator */
#define TLIR		0x17 /* Timing lock indicator */
/* AGC2 Integrator */
#define AGC2I1		0x18
#define AGC2I2		0x19

#define RTF		0x1A /* Timing frequency */
#define VSTATUS		0x1B /* Vstatus */
#define CLDI		0x1C /* Carrier lock detector */
/* Error count */
#define ERRCNTH		0x1D
#define ERRCNTL		0x1E
/* Symbol frequency */
#define SFRH		0x1F
#define SFRM		0x20
#define SFRL		0x21
/* Carrier frequency */
#define CFRM		0x22
#define CFRL		0x23
/* Noise indicator */
#define NIRH		0x24
#define NIRL		0x25

#define VERROR		0x26 /* VERROR */
#define FECM		0x28 /* FEC mode */
/* Viterbi threshold */
#define VTH0		0x29 /* Rate 1/2 */
#define VTH1		0x2A /* Rate 2/3 */
#define VTH2		0x2B /* Rate 3/4 */
#define VTH3		0x2C /* Rate 5/6 */
#define VTH4		0x2D /* Rate 7/8 */

#define PR		0x31 /* puncture rate and synchro */
#define VSEARCH		0x32 /* viterbi and synchro search */
#define RS		0x33 /* rs control */
#define ERRCNT		0x34 /* error control */


