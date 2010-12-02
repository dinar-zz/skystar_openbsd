
/*
 *	VES1893 specific definitions
 */
struct DVB_BSRV2 {
	u_char          ctr;
	u_char          fec;
	u_char          inv;
};


int	dvb_bsrv2_detect(struct skystar_softc *);
int	dvb_bsrv2_command(struct skystar_softc *, unsigned int, void *);
