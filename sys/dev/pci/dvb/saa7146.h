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

/* i2c */
struct skystar_softc;

/* I2C Message - taken from the linux i2c code */
#define I2C_M_RD		0x01
#define I2C_M_TEN		0x10	/* we have a ten bit chip address	 */
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_NOSTART		0x4000


struct i2c_msg {
	u_int16_t	addr;	/* slave address			 */
	u_int16_t	flags;
	u_int32_t	len;	/* msg length				 */
	char		*buf;	/* pointer to msg data		 */
};

/*
 * prototypes for current i2c implementation 
 * TODO:  convert all
 * names to i2c_xxxx integrate it into system i2c stack
 */

u_int32_t	i2c_status_check(struct skystar_softc *);
int		i2c_busy_rise_and_fall(struct skystar_softc *, int);
int		i2c_reset(struct skystar_softc *);
int		i2c_write_out(struct skystar_softc *, u_int32_t *, int);
int		i2c_clean_up(struct i2c_msg *, int, u_int32_t *);
int		i2c_prepare(struct i2c_msg *, int, u_int32_t *);
int		i2c_transfer(struct skystar_softc *, struct i2c_msg [], int);
void		i2c_setgpio(struct skystar_softc *, int, u_int32_t);
/*			I2C 
 * time we wait after certain i2c-operations
 */
#define SAA7146_I2C_DELAY 	10000	/* was 10 for weird linux mdelay  construction */

#define	SAA7146_IICSTA		0x090	/* TODO: replace with I2C_STATUS */
#define	SAA7146_I2C_ABORT	MASK_07
#define	SAA7146_I2C_SPERR	MASK_06
#define	SAA7146_I2C_APERR	MASK_05
#define	SAA7146_I2C_DTERR	MASK_04
#define	SAA7146_I2C_DRERR	MASK_03
#define	SAA7146_I2C_AL		MASK_02
#define	SAA7146_I2C_ERR		MASK_01
#define	SAA7146_I2C_BUSY	MASK_00

#define	SAA7146_IICTRF		0x08c
#define	SAA7146_I2C_START	(0x3)
#define	SAA7146_I2C_CONT	(0x2)
#define	SAA7146_I2C_STOP	(0x1)
#define	SAA7146_I2C_NOP		(0x0)

#define SAA7146_I2C_BUS_BIT_RATE_6400	(0x5<<8)
#define SAA7146_I2C_BUS_BIT_RATE_3200	(0x1<<8)
#define SAA7146_I2C_BUS_BIT_RATE_480	(0x4<<8)
#define SAA7146_I2C_BUS_BIT_RATE_320	(0x6<<8)
#define SAA7146_I2C_BUS_BIT_RATE_240	(0x7<<8)
#define SAA7146_I2C_BUS_BIT_RATE_120	(0x0<<8)
#define SAA7146_I2C_BUS_BIT_RATE_80	(0x2<<8)
#define SAA7146_I2C_BUS_BIT_RATE_60	(0x3<<8)
