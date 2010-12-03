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
#include "skystar_dev.h"
#include "saa7146.h"


/* 
 * Philips SAA7146 Multimedia bridge, high performance Scaler and PCI circuit (SPCI) 
 * i2c interface
 */

#define SAA7146_I2C_TRANSFER_DELAY   100
#define SAA7146_I2C_RETRIES 5

/*
 *                           i2c handling implementation
 */
static u_int32_t   SAA7146_I2C_BBR = SAA7146_I2C_BUS_BIT_RATE_480;

/* this functions gets the status from the saa7146 and returns it */
u_int32_t
i2c_status_check(struct skystar_softc * skystar)
{
	return read_long(skystar, SAA7146_IICSTA);
}

/*
 * this function should be called after an i2c-command has been written. if
 * we are debugging, it checks, if the busy flags rises and falls correctly
 * and reports a timeout (-1) or the error-bits set like in described in the
 * specs, p.123, table 110
 */
int
i2c_busy_rise_and_fall(struct skystar_softc * skystar, int timeout)
{
	int             i = 0;
	u_int32_t       status = 0, mc2 = 0;

	/* do not poll for i2c-status before upload is complete */
	for (i = 5; i > 0; i--) {
		mc2 = (read_long(skystar, MC2) & 0x1);
	
	    if(mc2 != 0)
		break;
	DELAY(1000);
	}
	
	if (i == 0){
#ifdef SKYSTAR_DEBUG
	    printf("skystar: timeout error.");
#endif
	    return -1;
	}

	/* wait until busy-flag rises */
	for (i = 5; i > 0; i--) {
		status = i2c_status_check(skystar);

		/* check busy flag */
		if ((status & SAA7146_I2C_BUSY)!=0)
			break;

		/* see if anything can be done while we're waiting */
		DELAY(1000);
	}

	/*
	 * we don't check the i-value, since it does not matter if we missed
	 * the rise of the busy flag or the fall or whatever. we just have to
	 * wait some undefined time after an i2c-command has been written out
	 */
	/* wait until busy-flag is inactive or error is reported */
	for (i = timeout; i > 0; i--) {
		/*** printf("skystar: i2c_busy_rise_and_fall; fall wait %d\n",i); */
		status = i2c_status_check(skystar);

		/* check busy flag */
		if ((status & SAA7146_I2C_BUSY) == 0)
			break;

		/* check error flag */
		if ((status & SAA7146_I2C_ERR)!= 0)
			break;

		/* see if anything can be done while we're waiting  */
		DELAY(1000);
	}
	/* did a timeout occur ? */
	if (i == 0) {
		printf("skystar: i2c_busy_rise_and_fall: timeout #2\n");
		return -1;
	}
#ifdef SKYSTAR_DEBUG
	/* report every error pending */
	switch (status & 0xfc) {
	case SAA7146_I2C_SPERR:
		printf("skystar: i2c_busy_rise_and_fall: error due to invalid start/stop condition\n");
		break;

	case SAA7146_I2C_APERR:
		printf("skystar: i2c_busy_rise_and_fall: error in address phase\n");
		break;

	case SAA7146_I2C_DTERR:
		printf("skystar: i2c_busy_rise_and_fall: error in data transmission\n");
		break;

	case SAA7146_I2C_DRERR:
		printf("skystar: i2c_busy_rise_and_fall: error when receiving data\n");
		break;

	case SAA7146_I2C_AL:
		printf("skystar: i2c_busy_rise_and_fall: error because arbitration lost\n");
		break;
	}
#endif
	return status;
}

/*
 * this functions resets the saa7146 and returns 0 if everything was fine,
 * otherwise -1
 */
int
i2c_reset(struct skystar_softc * skystar)
{
	u_int32_t          status = 0;

	status = i2c_status_check(skystar);

	/* clear data-byte for sure */
	write_long(skystar, SAA7146_IICTRF, 0);

	/* check if any operation is still in progress */
	if ((status & SAA7146_I2C_BUSY)!=0) {
		/* set ABORT-OPERATION-bit */
		write_long(skystar, SAA7146_IICSTA, (SAA7146_I2C_BBR | MASK_07));
		write_long(skystar, MC2, (MASK_00 | MASK_16));
		DELAY(SAA7146_I2C_DELAY);	/* sleep 10ms */

		/*
		 * clear all error-bits pending; this is needed because
		 * p.123, note 1
		 */
		write_long(skystar, SAA7146_IICSTA, SAA7146_I2C_BBR);
		write_long(skystar, MC2, (MASK_00 | MASK_16));
		DELAY(SAA7146_I2C_DELAY);
	}
	/* check if any other error is still present */
	if ((status = i2c_status_check(skystar))!= SAA7146_I2C_BBR) {

		/* clear all error-bits pending */
		write_long(skystar, SAA7146_IICSTA, SAA7146_I2C_BBR);
		write_long(skystar, MC2, (MASK_00 | MASK_16));
		DELAY(SAA7146_I2C_DELAY);

		/*
		 * the data sheet says it might be necessary to clear the
		 * status twice after an abort
		 */
		write_long(skystar, SAA7146_IICSTA, SAA7146_I2C_BBR);
		write_long(skystar, MC2, (MASK_00 | MASK_16));
	}
	/* if any error is still present, a fatal error has occured ... */
	if (SAA7146_I2C_BBR != (status = i2c_status_check(skystar))) {
		printf("skystar: i2c_reset: fatal error, status:0x%08x\n", (unsigned) status);
		return -1;
	}
	return 0;
}

/*
 * this functions writes out the data-bytes at 'data' to the saa7146 at
 * address 'addr' regarding the 'timeout' and 'retries' values; it returns 0
 * if ok, -1 if the transfer failed, -2 if the transfer failed badly (e.g.
 * address error)
 */
int
i2c_write_out(struct skystar_softc * skystar, u_int32_t * data, int timeout)
{
	int             status = 0;

	/* write out i2c-command */
	write_long(skystar, SAA7146_IICTRF, *data);
	write_long(skystar, SAA7146_IICSTA, SAA7146_I2C_BBR);
	write_long(skystar, MC2, (MASK_00 | MASK_16));
	/*
	 * after writing out an i2c-command we have to wait for a while;
	 * because we do not know, how long we have to wait, we simply look
	 * what the busy-flag is doing, before doing something else
	 */

	/*
	 * reason: while fiddling around with the i2c-routines, I noticed
	 * that after writing out an i2c-command, one may not read out the
	 * status immediately after that. you *must* wait some time, before
	 * even the busy-flag gets set
	 */
	status = i2c_busy_rise_and_fall(skystar, timeout);
	if (status == -1) {
		printf("skystar: i2c_write_out; timeout\n");
		return -1;
	}
	/* we only handle address-errors here */
	if ((status & SAA7146_I2C_APERR)!= 0) {
		//printf("skystar: i2c_write_out; error in address phase\n");
		return -2;
	}
	/* check for some other mysterious error; we don't handle this here */
	if ((status & 0xff)!= 0) {
		printf("skystar: i2c_write_out: unknown error 0x%x\n", status);
		return -1;
	}
	/* read back data, just in case we were reading ... */
	*data = read_long(skystar, SAA7146_IICTRF);
	/*** printf("skystar: writeout: 0x%08x (after)\n",(unsigned)*data); */

	return 0;
}

int
i2c_clean_up(struct i2c_msg * m, int num, u_int32_t * op)
{
	u_int32_t          i, j;
	u_int32_t          op_count = 0;

	/* loop through all messages */
	for (i = 0; i < num; i++) {
		op_count++;
		/* loop throgh all bytes of message i */
		for (j = 0; j < m[i].len; j++) {
			/* write back all bytes that could have been read */
			m[i].buf[j] = (op[op_count / 3] >> ((3 - (op_count % 3)) * 8));
			op_count++;
		}
	}
	return 0;
}

int
i2c_prepare(struct i2c_msg * m, int num, u_int32_t * op)
{
	u_int16_t	h1, h2;
	u_int32_t	i, j, addr;
	u_int32_t	mem = 0, op_count = 0;

	/* determine size of needed memory */
	for (i = 0; i < num; i++)
		mem += m[i].len + 1;

	/*
	 * we need one u_int32_t for three bytes to be send plus one byte to
	 * address the device
	 */
	mem = 1 + ((mem - 1) / 3);

	if (mem > I2C_MEM_SIZE) {
		printf("skystar: i2c_prepare: i2c-message too big\n");
		return -1;
	}
	/* be careful: clear out the i2c-mem first */
	bzero(op, sizeof(u_int32_t) * mem);

	/* loop through all messages */
	for (i = 0; i < num; i++) {
		/*
		 * insert the address of the i2c-slave. note: we get
		 * 7-bit-i2c-addresses, so we have to perform a translation
		 */
		addr = (m[i].addr << 1) + ((0 != (m[i].flags & I2C_M_RD)) ? 1 : 0);
		h1 = op_count / 3;
		h2 = op_count % 3;
		op[h1] |= ((u_char) addr << ((3 - h2) * 8));
		op[h1] |= (SAA7146_I2C_START << ((3 - h2) * 2));
		op_count++;

		/* loop through all bytes of message i */
		for (j = 0; j < m[i].len; j++) {
			/* insert the data bytes */
			h1 = op_count / 3;
			h2 = op_count % 3;
			op[h1] |= ((u_char) m[i].buf[j] << ((3 - h2) * 8));
			op[h1] |= (SAA7146_I2C_CONT << ((3 - h2) * 2));
			op_count++;
		}

	}

	/*
	 * have a look at the last byte inserted: if it was: ...CONT change
	 * it to ...STOP
	 */
	h1 = (op_count - 1) / 3;
	h2 = (op_count - 1) % 3;
	if (SAA7146_I2C_CONT == (0x3 & (op[h1] >> ((3 - h2) * 2)))) {
		op[h1] &= ~(0x2 << ((3 - h2) * 2));
		op[h1] |= (SAA7146_I2C_STOP << ((3 - h2) * 2));
	}
	return mem;
}

/*
 * Transfer i2c message
 */
int
i2c_transfer(struct skystar_softc * skystar, struct i2c_msg msgs[], int num)
{
	int             result, count;
	int             retries = SAA7146_I2C_RETRIES;
	int             i = 0;


	/* prepare the message(s), get number of transfers */
	count = i2c_prepare(msgs, num, (u_int32_t *) skystar->i2c);
	if (count < 0) {
		printf("skystar: i2c_transfer: could not prepare i2c-message\n");
		return -1;
	}
	/* loop through number of retries ... */
	while (retries > 0) {

		retries--;

		/* reset the i2c-device if necessary */
		result = i2c_reset(skystar);
		if ( result < 0 ) {
			printf("skystar: i2c_transfer: could not reset i2c-bus\n");
			return -1;
		}
		/*
		 * see how many u32 have to be transferred; if there is only
		 * 1, we do not start the whole rps1-engine...
		 */

		for (i = 0; i < count; i++) {
			result = i2c_write_out(skystar, ((u_int32_t *) (skystar->i2c)) + i, 100);	/* TODO: set correct timeout */
			if (result!= 0) {
				/* if address-error occured, don't retry */
				if (result == -2) {
					//printf("skystar: i2c_transfer: error in address phase\n");
					return -1;
				}
				printf("skystar: i2c_transfer: error tranferring,trying again\n");
				break;
			}
		}

		/* see if an error occured & the last retry failed */
		if ((result!=0) && (retries == 0)) {
			printf("skystar: i2c_transfer: could not transfer i2c-message\n");
			return -1;
		}
		if (result == 0)
			break;
	}

	/* if any things had to be read, get the results */
	result = i2c_clean_up(msgs, num, (u_int32_t *) skystar->i2c);
	if (result < 0) {
		printf("skystar: i2c_transfer: could not cleanup\n");
		return -1;
	}
	/* return the number of delivered messages */
	return num;
}

void 
i2c_setgpio(struct skystar_softc * skystar, int port, u_int32_t data)
{
	u_int32_t	val;

	val = read_long(skystar, GPIO_CTRL);
	val &= ~(0xff << (8 * (port)));
	val |= (data) << (8 * (port));
	write_long(skystar, GPIO_CTRL, val);
}
