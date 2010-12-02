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

#include "bpfilter.h"

#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <machine/bus.h>
#include <uvm/uvm_extern.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/route.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>

#if NBPFILTER > 0
#include <net/bpf.h>
#endif

 
#include "frontend.h"
#include "dvb.h"
#include "skystar_dev.h"
#include "saa7146.h"


/* DVB chip decsription	 */
struct DVB_CHIP {
	int             chip;
	int             (*dfun) (struct skystar_softc *);
	int             (*cfun) (struct skystar_softc *, unsigned int, void *);
};

/* known DVB chips	 */
struct DVB_CHIP dvb_table[] ={
	{BSRV2, dvb_bsrv2_detect, dvb_bsrv2_command},
	{BSRU6, dvb_bsru6_detect, dvb_bsru6_command},
	{0, 0, 0}
};
static void readMAC(struct skystar_softc *, u_char *);
static void getMAC(struct skystar_softc *);

#define	DVBBAUDRATE	40000000

/*
 * Detect dvb chip, set command handler, return chip number, or 0 if not
 * detected
 */
int 
dvb_detect(struct skystar_softc * skystar)
{
	int             i;
	int             res;
	skystar->dvb_cfun = 0;
	for (i = 0; dvb_table[i].chip; i++)
		if ((res = dvb_table[i].dfun(skystar))) {
			skystar->dvb_cfun = dvb_table[i].cfun;
			return res;
		}
	return 0;
}

/* Transfer dvb command to one of chip modules	 */
int 
dvb_command(struct skystar_softc * skystar, unsigned int cmd, void *arg)
{
	if (skystar->dvb_cfun)
		return skystar->dvb_cfun(skystar, cmd, arg);
	else {
		printf("skystar: unknown demodulator chip!\n");
		return -1;
	}
}



/*
 *	network subsystem: INITIALIZE
 */
void
dvb_init(void *self)
{
	struct skystar_softc *skystar = (struct skystar_softc *) self;
	struct ifnet   *ifp = &(skystar->arpcom.ac_if);

	/* Initialize ifnet structure. */
	ifp->if_softc = skystar;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST | IFF_RUNNING;
	ifp->if_ioctl = dvb_ioctl;
	ifp->if_start = dvb_start;
	ifp->if_watchdog = dvb_watchdog;
	ifp->if_baudrate = DVBBAUDRATE;	/* 40 MBit */
	snprintf(ifp->if_xname, IFNAMSIZ, "dvb%u", skystar->ss_dev.dv_unit);
	/* Set MAC address		 */
	getMAC(skystar);
	/* Attach network interface */
	if_attach(ifp);
	ether_ifattach(ifp);
	ifp->if_output = dvb_output;	/* we can't send packets to this
					 * device, so change to dvb_output()
					 * from ether_output() */

	printf("%s at %s \"DVB network interface\" address %s\n", ifp->if_xname,
		 skystar->ss_dev.dv_xname, ether_sprintf(skystar->arpcom.ac_enaddr));
	return;
}

/*
 *	network subsystem: DONE
 */
void
dvb_done(void *self)
{
	struct skystar_softc *skystar = (struct skystar_softc *) self;
	struct ifnet   *ifp = &(skystar->arpcom.ac_if);

	/* Detach network interface */
	ether_ifdetach(ifp);
	if_detach(ifp);
	return;
}

/*
 *	network subsystem: WATCHDOG
 */
void
dvb_watchdog(struct ifnet * ifp)
{
#ifdef SKYSTAR_DEBUG
	printf("%s: watchdog called (do nothing)\n", ifp->if_xname);
#endif
	return;
};

/*
 *	device function: OUTPUT
 */
int
dvb_output(struct ifnet * ifp, struct mbuf * p2, struct sockaddr * p3, struct rtentry * p4)
{
	m_freem(p2);
#ifdef SKYSTAR_DEBUG
	printf("%s: output called (this device can't transmit packets)\n", ifp->if_xname);
#endif
	return EINVAL;
};

/*
 *	network subsystem: START
 */
void
dvb_start(struct ifnet * ifp)
{
#ifdef SKYSTAR_DEBUG
	printf("%s: start called (this device can't transmit packets)\n", ifp->if_xname);
#endif
	return;
};

/*
 *	network subsystem: IOCTL
 */
int
dvb_ioctl(struct ifnet * ifp, u_long cmd, caddr_t data)
{

	struct skystar_softc *skystar = (struct skystar_softc *) ifp->if_softc;
	/* struct ifaddr	*ifa = (struct ifaddr *)data; */
	int             s;
	int             error = 0;

	s = splnet();

	/* first, send IOCTL to ethernet subsystem */
	if ((error = ether_ioctl(ifp, &skystar->arpcom, cmd, data)) > 0){
#ifdef SKYSTAR_DEBUG
		printf( "=== ether_ioctl() error, cmd: %u\n" , cmd );
#endif
		goto fail;                                                     
        }
	/* second, handle IOCTL myself */
	switch (cmd) {
	case SIOCSIFADDR:	/* set interface address */
		ifp->if_flags |= IFF_UP;
		break;
	case SIOCSIFFLAGS:	/* set interface flags */
		/*
		 * we must handle this ok, because promiskystar mode
		 * on/off
		 */
		break;
#if 0
TODO:		SIOCADDMULTI
			SIOCDELMULTI
			SIOCSIFMEDIA
			SIOCGIFMEDIA

	case SIOCGIFADDR:	/* get interface address */
	case SIOCSIFMTU:	/* set interface MTU */
	case SIOCSIFFLAGS:	/* set interface flags */
		/*
		 * TODO: if (ifp->if_flags & IFF_UP) { rl_init(skystar); }
		 * else { if (ifp->if_flags & IFF_RUNNING) rl_stop(skystar);
		 * }
		 */
		error = 0;
		break;
	case SIOCADDMULTI:	/* add multicast address */
	case SIOCDELMULTI:	/* del multicast address */
	case SIOCSIFMEDIA:	/* set interface media status */
	case SIOCGIFMEDIA:	/* get interface media status */
	case SIOCSIFGENERIC:	/* set interface generic params */
	case SIOCGIFGENERIC:	/* get interface generic params */
#endif
	default:
		error = EINVAL;
		break;
	}
fail:
	splx(s);
	return error;
};

/*
 *	network subsystem: IPMPE
 *	make ethernet packet and give it to the system to process
 */
void
dvb_ipmpe(struct skystar_softc * dev, u_char * pkt, int pkt_len)
{
	struct ifnet   *ifp = &((struct skystar_softc *) dev)->arpcom.ac_if;
	struct mbuf    *m;
	int             datalen = pkt_len - 12;
	struct ether_header eh;
	char           *ehb = (char *) &eh;

	char            buf2[pkt_len - 12 + sizeof(struct ether_header)];

#ifdef NET_DEBUG
	char            bs[18];
	char            bd[18];
	int             i;
#endif

	if (datalen <= 0)
		return;

	/*
	 * copy packet data create ethernet frame header bzero( &eh,
	 * sizeof(ether_header) );
	 */
	ehb[0] = pkt[11];
	ehb[1] = pkt[10];
	ehb[2] = pkt[9];
	ehb[3] = pkt[8];
	ehb[4] = pkt[4];
	ehb[5] = pkt[3];
	ehb[6] = ehb[7] = ehb[8] =
		ehb[9] = ehb[10] = ehb[11] = 0;
	ehb[12] = 8;
	ehb[13] = 0;

	/* print debug info */
#ifdef NET_DEBUG
	strlcpy(bd, ether_sprintf(eh.ether_dhost), sizeof(bd));
	strlcpy(bs, ether_sprintf(eh.ether_shost), sizeof(bs));
	printf("net_ipmpe %s %s -> %s : type %u of %u bytes\n", 
	    ifp->if_xname, bs, bd, eh.ether_type, datalen);

	if (eh.ether_type == ntohs(ETHERTYPE_IP))
		printf("datatype is ip: %u.%u.%u.%u -> %u.%u.%u.%u\n",
		     pkt[12 + 12], pkt[12 + 13], pkt[12 + 14], pkt[12 + 15],
		    pkt[12 + 16], pkt[12 + 17], pkt[12 + 18], pkt[12 + 19]);

	printf("64 bytes dump foolws:\n");
	for (i = 0; i < 64; i++) {
		if ((i & 0xf) == 0)
			printf("%04X ", i);
		printf("%02x ", pkt[i + 12]);
		if ((i & 0xf) == 0xf)
			printf("\n");
	}
	printf("\n");
#endif

	/* process packet */
	memcpy(buf2, ehb, 14);
	memcpy(buf2 + 14, pkt + 12, datalen);
	m = (struct mbuf *) m_devget(buf2, datalen + 14, 0, ifp, NULL);
	if (!m) {
		printf("%s dvb: out of mbufs\n", dev->ss_dev.dv_xname);
		return;
	}
	ifp->if_ipackets++;
#if NBPFILTER > 0
	/*
	 * Check if there's a BPF listener on this interface.
	 * If so, hand off the raw packet to BPF.
	 */
	if (ifp->if_bpf)
		bpf_mtap(ifp->if_bpf, m, BPF_DIRECTION_IN);
#endif
	ether_input_mbuf(ifp, m);
	DELAY(500);		/* TODO: remove this */
}

/*
 *	Read MAC address from card
 */
static void
readMAC(struct skystar_softc * skystar, u_char * buf)
{
	unsigned char   mm1 = 0xd4;
	struct i2c_msg  msgs[2];

	msgs[0].flags = 0;
	msgs[1].flags = I2C_M_RD;
	msgs[0].addr = msgs[1].addr = 0x50;
	msgs[0].len = 1;
	msgs[1].len = 6;
	msgs[0].buf = &mm1;
	msgs[1].buf = buf;
	i2c_transfer(skystar, msgs, 2);
	return;
}

static const u_char k0[3] = {0x54, 0x7B, 0x9E};
static const u_char k1[3] = {0xD3, 0xF1, 0x23};

static void
getMAC(struct skystar_softc * skystar)
{
	u_char		data[6];
	u_char		*imac = data;
	u_int32_t  	tmp1, tmp0;
	int		i;

	readMAC(skystar, data);
	for (i = 0; i < 3; i++) {
		tmp0 = *(imac++) ^ k0[i];
		tmp1 = *(imac++) ^ k1[i];
		skystar->arpcom.ac_enaddr[5 - i] = (u_char) (((tmp1 << 8) | tmp0) >> ((tmp1 >> 6) & 0x3));
	}
	skystar->arpcom.ac_enaddr[0] = 0x00;
	skystar->arpcom.ac_enaddr[1] = 0xd0;
	skystar->arpcom.ac_enaddr[2] = 0x5c;
}

