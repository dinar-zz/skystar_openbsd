/* DEBI transfer modes	 */
#define DEBINOSWAP 0x000e0000
#define DEBISWAB   0x001e0000
#define DEBISWAP   0x002e0000

void		iwdebi(struct skystar_softc *, u_int32_t, int, u_int32_t, int);
u_int32_t		irdebi(struct skystar_softc *, u_int32_t, int, u_int32_t, int);
void		wdebi(struct skystar_softc *, u_int32_t, int, u_int32_t, int);
u_int32_t		rdebi(struct skystar_softc *, u_int32_t, int, u_int32_t, int);
int		arm_boot(struct skystar_softc *);
void		debi_intr(struct skystar_softc *);
void		gpio_intr(struct skystar_softc *);
void		fsection(struct skystar_softc *, int, u_char *, u_int16_t);

u_int32_t	get_fw_version(struct skystar_softc *);

#define FW_CI_SUPPORT(arm_app)	((arm_app) & 0x80000000)
#define FW_4M_SDRAM(arm_app)	((arm_app) & 0x40000000)
#define FW_VERSION(arm_app)	((arm_app) & 0x0000FFFF)


/* handle DPRAM mailbox registers	 */
#define arm_resetMailBox( skystar ) \
{\
	int s = splnet();\
        debiread( skystar, DEBINOSWAP, IRQ_RX, 2);\
        debiwrite( skystar, DEBINOSWAP, IRQ_RX, 0, 2);\
        splx( s );\
}

#define ARM_ClearMailBox(skystar) {iwdebi(skystar, DEBINOSWAP, IRQ_RX, 0, 2);}
#define ARM_ClearIrq(skystar) {irdebi(skystar, DEBINOSWAP, IRQ_RX, 0, 2);}


/* firmware commands		 */
int	hw_filter_set(struct skystar_softc *, struct bitfilter *);
struct dvb_filter *hw_filter_alloc(struct skystar_softc *);
int	hw_filter_free(struct skystar_softc *, struct dvb_filter *);
int	hw_filter_stop(struct skystar_softc *, u_int16_t);
int	hw_filters_stop(struct skystar_softc *);
void	hw_22k_set(struct skystar_softc *, int);
void	hw_pids_set(struct skystar_softc *, u_int16_t, u_int16_t, u_int16_t);
void	hw_diseqc_set(struct skystar_softc *, int);
int	hw_dac_send(struct skystar_softc *, u_char, u_char);
int	hw_volume_set(struct skystar_softc *, u_char, u_char);
