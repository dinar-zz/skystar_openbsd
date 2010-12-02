
struct skystar_softc;

#include "dvb_bsru6.h"
#include "dvb_bsrv2.h"

#define DVB_NONE	0
#define BSRV2		1
#define BSRU6		2

union DVB {
	struct DVB_BSRV2 bsrv2;
	struct DVB_BSRU6 bsru6;
};

/*
 * network subsystem functions
 */

int	dvb_detect(struct skystar_softc *);
int	dvb_command(struct skystar_softc *, unsigned int, void *);
void	dvb_init(void *);
void	dvb_done(void *);
void	dvb_ipmpe(struct skystar_softc *, u_char *, int);
void	dvb_watchdog(struct ifnet *);
int	dvb_output(struct ifnet *, struct mbuf *, struct sockaddr *, struct rtentry *);
void	dvb_start(struct ifnet *);
int	dvb_ioctl(struct ifnet *, u_long, caddr_t);
