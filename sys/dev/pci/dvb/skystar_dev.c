#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <dev/pci/pcivar.h>
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
#include "saa7146.h"

#define NET_DEBUG



/* firmware */
/* TODO: compress firmware */

extern char	*fw_bootup;
extern char	*fw_root;
extern char	*fw_dpram;
extern long	fw_bootup_len;
extern long	fw_root_len;
extern long	fw_dpram_len;

char            test_pkt[256];


/*
 * skystar driver functions
 */

int	skystar_probe(struct device *, void *, void *);
void	skystar_attach(struct device *, struct device *, void *);
int	skystar_intr(void *);
void	skystar_shutdown(void *);
void 	skystar_init (struct skystar_softc *);

/*
 * skystar device functions
 */

#define skystar_open	skystaropen
#define skystar_close	skystarclose
#define skystar_read	skystarread
#define skystar_write	skystarwrite
#define skystar_ioctl	skystarioctl


int	skystar_open(dev_t, int, int, struct proc *);
int	skystar_close(dev_t, int, int, struct proc *);
int	skystar_read(dev_t, struct uio *, int);
int	skystar_write(dev_t, struct uio *, int);
int	skystar_ioctl(dev_t, u_long, caddr_t, int, struct proc *);


/*
 *	DVB frontend: SET
 */
static void
SetFront(struct skystar_softc * skystar, struct frontend * newfront)
{
	hw_filters_stop(skystar);
	hw_diseqc_set(skystar, (newfront->diseqc * 4) | (newfront->volt ? 2 : 0) | (newfront->ttk ? 1 : 0));
	hw_22k_set(skystar, newfront->ttk);
	dvb_command(skystar, DVB_SET_FRONTEND, newfront);
	hw_pids_set(skystar, newfront->video_pid, newfront->audio_pid, newfront->ttext_pid);
	/* TODO: send CI command */
#if 0
	if (need_reset)
		dvb_command(skystar, DVB_RESET, 0);
	if ((skystar->tuner_type == DVBS) && newfront->AFC && need_reset) {
		front.freq = newfront->freq;
		AFC(skystar);
	}
#endif
}
/*
 *	DVB frontend: INITIALIZE
 */
static void
InitFront(struct skystar_softc * skystar)
{
	struct frontend front;
	//int           ttype = 0;

	/*
	 * TODO: memset(&(dvb->front), 0xff, sizeof(struct
	 * frontend));
	 */
	//bzero(&front, sizeof(struct frontend));

	//dvb_command(skystar, DVB_INIT_FRONTEND, 0);

	front.pnr = 0;
	front.channel_flags = 0;
	/*
	 * front.type=DVBS; //dvb->front.type=dvb->dvbtype; switch
	 * (front.type) { case DVBS: ttype=TUNER_SP5659;
	 *
	 * tuner_command(skystar, TUNER_SET_TYPE, &ttype);
	 */
	hw_dac_send(skystar, 2, 1);
	hw_dac_send(skystar, 2, 0);
	DELAY(100 * 1000);

	hw_dac_send(skystar, 0, 0x44);
	hw_dac_send(skystar, 1, 0x81);
	hw_dac_send(skystar, 2, 0xf0);
	hw_volume_set(skystar, 0x45, 0x45);

	write_long(skystar, MC2, (MASK_09 | MASK_25 | MASK_10 | MASK_26)); 
	front.power = 1;
	front.afc = 1;
	front.fec = 8;

	/* Astra n-tv default */
	front.channel_flags = DVB_CHANNEL_FTA;
	front.volt = 0;
	front.ttk = 1;
	front.diseqc = 0;
	//front.freq = front.curfreq = (12666000 - 10600000) * 1000;
	front.srate = 22000000;
	front.video_pid = 162;
	front.audio_pid = 96;
	front.ttext_pid = 0x1012;
	front.inv = 0;

	SetFront(skystar, &front);
}


/*
 *                             tuner control
 */
struct tunertype {
	char           *name;
	u_char          Type;

	/* all frequencies in units of Hz */
	u_int32_t          min;	/* minimum frequency */
	u_int32_t          max;	/* maximum frequency */
	u_int32_t          res;	/* frequency resolution in Hz */
	u_int32_t          step;	/* stepsize in Hz (units of setfrequency ioctl) */
	u_int32_t          IFPCoff;/* frequency offset */
	u_int32_t          thresh1;/* frequency Range for UHF,VHF-L, VHF_H */
	u_int32_t          thresh2;
	u_char          VHF_L;
	u_char          VHF_H;
	u_char          UHF;
	u_char          config;
	u_char          mode;	/* mode change value (tested PHILIPS_SECAM
				 * only) */
	/* 0x01 -> ??? no change ??? */
	/* 0x02 -> PAL BDGHI / SECAM L */
	/* 0x04 -> ??? PAL others / SECAM others ??? */
	int             capability;
};


static struct tunertype tuners[] = {
	{"SP5659", DVBS,
		100000000UL, 2700000000UL, 125000, 1, 479500000,
	0xffffffffUL, 0xffffffffUL, 0x30, 0x30, 0x30, 0x95},
	{"SPXXXX", DVBC,
		40000000UL, 870000000UL, 62500, 1, 36125000,
	174000000UL, 454000000UL, 0xa1, 0x92, 0x34, 0x8e},
};
#define TUNERS (sizeof(tuners)/sizeof(struct tunertype))

int tuner_command(struct skystar_softc *, unsigned int cmd, void *);



/*
 *	device structures ( used by KERNEL )
 */

struct cfattach skystar_ca = {
	sizeof(struct skystar_softc),
	skystar_probe,
	skystar_attach
};


struct cfdriver skystar_cd = {
	0,
	"skystar",
	DV_DULL			/* generic device, was network: DV_IFNET */
};



/*
 *	mnemory management : ALLOCATE DMA MEMORY
 */
#define BUS_DMAMEM_ALLOC	1
#define BUS_DMAMEM_MAP		2
#define BUS_DMAMAP_CREATE	3
#define BUS_DMAMAP_LOAD		4

static int
get_dma_mem(bus_dma_tag_t dma_tag, bus_dmamap_t * pmem_map, void **pmem_ptr, unsigned int size)
{
	/* returns 0 if successfull	 */
	bus_dma_segment_t seg;
	int             rseg;
	caddr_t         kva;
	int             result = 0;

	if (bus_dmamem_alloc(dma_tag, size, PAGE_SIZE, 0, &seg, 1, &rseg, BUS_DMA_NOWAIT)) {
		printf("%bus_dmamem_alloc() failed, ");
		result = BUS_DMAMEM_ALLOC;
		goto fail_alloc;
	}
	if (bus_dmamem_map(dma_tag, &seg, rseg, size, &kva, BUS_DMA_NOWAIT)) {
		printf("bus_dmamem_map() failed, ");
		result = BUS_DMAMEM_MAP;
		goto fail_map;
	}
	if (bus_dmamap_create(dma_tag, size, 1, size, 0, BUS_DMA_NOWAIT, pmem_map)) {
		printf("bus_dmamap_create() failed, ");
		result = BUS_DMAMAP_CREATE;
		goto fail_create;
	}
	if (bus_dmamap_load(dma_tag, *pmem_map, kva, size, NULL, BUS_DMA_NOWAIT)) {
		printf("%bus_dmamap_load() failed, ");
		result = BUS_DMAMAP_LOAD;
		goto fail_load;
	}
	bzero((void *) kva, size);
	*pmem_ptr = (void *) kva;
	return 0;

fail_load:
	bus_dmamap_destroy(dma_tag, *pmem_map);
fail_create:
	bus_dmamem_unmap(dma_tag, kva, size);
fail_map:
	bus_dmamem_free(dma_tag, &seg, rseg);
fail_alloc:
	return result;
}

/*
 *	memory management : FREE DMA MEMORY
 */
#if 0
static void
free_dma_mem(
	     bus_dma_tag_t dma_tag,	/* dma memory tag		 */
	     bus_dmamap_t mem_map,	/* allocated memory map tag */
	     void *mem_ptr)
{				/* allocated memory address */
	bus_dmamem_unmap(dma_tag, (caddr_t) mem_ptr, mem_map->dm_mapsize);
	bus_dmamem_free(dma_tag, mem_map->dm_segs, 1);
	bus_dmamap_destroy(dma_tag, mem_map);
}
#endif


/*
 *	driver function: PROBE
 *	probe skystar device
 */
int
skystar_probe(struct device * parent, void *match, void *aux)
{
	struct pci_attach_args *pa = (struct pci_attach_args *) aux;
	if (PCI_VENDOR(pa->pa_id) == 0x1131 &&
	    PCI_PRODUCT(pa->pa_id) == 0x7146) {
#ifdef SKYSTAR_DEBUG
		printf("\nskystar: probe DEVICE FOUND! -------\n");
#endif
		return 1;
	}
	return 0;
};


/* TODO: remove */
struct dvb_softc {
	struct device   dev;	/* base device		 */
	struct arpcom   arpcom;	/* ethernet subsystem data	 */
};

/*
 *	driver function: ATTACH
 *	attach skystar device
 */
void
skystar_attach(struct device * parent, struct device * self, void *aux)
{
	u_int32_t          fun;
	int             retval;
	int             i;

	pci_intr_handle_t ih;
	const char     *intrstr;

	struct skystar_softc *skystar = (struct skystar_softc *) self;
	struct pci_attach_args *pa = (struct pci_attach_args *) aux;

	/*
	 * map memory
	 */
	retval = pci_mapreg_map(pa, PCI_MAPREG_START, PCI_MAPREG_TYPE_MEM | PCI_MAPREG_MEM_TYPE_32BIT, 0,
	  &(skystar->memt), &(skystar->memh), NULL, &(skystar->obmemsz), 0);

	if (retval) {
		printf("%s: couldn't map memory\n", skystar->ss_dev.dv_xname);
		goto fail;
	}
	/*
	 * map interrupt
	 */
	if (pci_intr_map(pa, &ih)) {
		printf("%s: couldn't map interrupt\n", skystar->ss_dev.dv_xname);
		goto fail;
	}
	intrstr = pci_intr_string(pa->pa_pc, ih);
	skystar->ih = pci_intr_establish(pa->pa_pc, ih, IPL_NET, skystar_intr, skystar, skystar->ss_dev.dv_xname);
	if (skystar->ih == NULL) {
		printf("%s: couldn't establish interrupt at %s\n", skystar->ss_dev.dv_xname, intrstr ? intrstr : "UNKNOWN");
		goto fail;
	}
	printf(": %s", intrstr ? intrstr : "UNKNOWN");

	/*
	 * Enabled Bus Master XXX: check if all old DMA is stopped first
	 * (e.g. after warm boot)
	 */
	fun = pci_conf_read(pa->pa_pc, pa->pa_tag, PCI_COMMAND_STATUS_REG);
	pci_conf_write(pa->pa_pc, pa->pa_tag, PCI_COMMAND_STATUS_REG, fun | PCI_COMMAND_MASTER_ENABLE);

	/*
	 * allocate internal buffers
	 */

	/* allocate i2c memory */
	skystar->i2c = malloc(I2C_MEM_SIZE * sizeof(u_int32_t), M_DEVBUF, M_NOWAIT);
	if (!skystar->i2c) {
		printf("%s:could not allocate i2c memory.\n", skystar->ss_dev.dv_xname);
		goto fail;
	}
	bzero((caddr_t) (skystar->i2c), I2C_MEM_SIZE * sizeof(u_int32_t));

	/* allocate debi memory (32Kb) */
	skystar->ss_dmat = pa->pa_dmat;	/* set this !!! */
	if (get_dma_mem(skystar->ss_dmat, &(skystar->debi_dmap), &(skystar->debi), 32768)) {
		printf("%s:could not allocate DEBI memory.\n", skystar->ss_dev.dv_xname);
		goto fail;
	}

	write_long(skystar, MC1, (MASK_08 | MASK_24));

	/*
	 * disable everything on the saa7146, perform a
	 * software-reset
	 */
	write_long(skystar, MC1, 0xbfff0000);

	/* clear all registers */
	for (i = 0x0; i < 0xfc; i += 0x4)
		write_long(skystar, i, 0x0000000);
	for (i = 0x104; i < 0x1fc; i += 0x4)
		write_long(skystar, i, 0x0000000);

	/* clear out any rps-signals pending */
	write_long(skystar, MC2, 0xf8000000);

	/* TODO: enable only really needful things: i2c and DEBI */
	/* enable video-port-pins */
	/* write_long (skystar, MC1, (MASK_10 | MASK_26)); */

	/*
	 * disable all interrupt-conditions, only enable RPS
	 * interrupts
	 */
	write_long(skystar, ISR, 0xffffffff);
	write_long(skystar, IER, 0 /* (MASK_27 | MASK_28) */ );

	write_long(skystar, PCI_BT_V1, 0x0000101f);
	write_long(skystar, BCS_CTRL, 0x80400040);

	/* set dd1 stream a & b */
	/*
	write_long (skystar, DD1_STREAM_B,   0x00000000);
	write_long (skystar, DD1_INIT,       0x02000000);
	write_long (skystar, MC2, (MASK_09 | MASK_25 | MASK_10 | MASK_26));

	write_long (skystar, MC2, 0x077c077c);
	*/
	/*
	 * the Siemens DVB needs this if you want to have the I2c chips get
	 * recognized before the main driver is loaded
	 *
	 * TODO: does it need SkyStar 1
	 */
	write_long(skystar, GPIO_CTRL, 0x500000);
	
        skystar_init(skystar);
fail:
	return;
};

/*
 *	driver function: SHUTDOWN ROUTINE
 *	interrupt service routine
 */
void
skystar_shutdown(void *v)
{
#ifdef SKYSTAR_DEBUG
	printf("\n----- skystar: shutdown called -----\n");
#endif
	/* Shutdown network		 */
	dvb_done(v);
	return;
}

/*
 *	driver function: INTERRUPT HANDLER
 *	interrupt service routine
 */
int
skystar_intr(void *arg)
{
	struct skystar_softc *skystar = (struct skystar_softc *) arg;
	u_int32_t          isr = 0;/* interrupt status register */

	/* printf("DEBUG: skystar_intr: \n"); */
	/* process all interrupts */
	/* while (1) { */
	/* read out the primary status register */
	isr = read_long(skystar, ISR);

	/* clear all IRQs */
	write_long(skystar, ISR, isr);

	/* is anything to do? */
	if (0 == isr)
		return 0;

	/* DEBUG */
	/* printf("skystar: irq-call: isr:0x%08x\n", (unsigned)isr); */

	/*
	 * TODO: here we must process interrupt handlers of ALL subsystems
	 * (i2c, debi, etc.)
	 */
	if (isr & MASK_19)
		debi_intr(skystar);
	if (isr & MASK_03)
		gpio_intr(skystar);

	/* write_long (skystar, ISR, MASK_19|MASK_03); */

	/*
	 * printk(KERN_ERR "%s: unhandled interrupt: 0x%08x\n", saa->name,
	 * isr);
	 */

	return 1;
}

/*
 *	device function: OPEN
 */
int
skystar_open(dev_t dev, int flags, int fmt, struct proc * p)
{
	struct skystar_softc *skystar = (struct skystar_softc *) device_lookup(&skystar_cd, minor(dev));

	if (skystar == NULL) {
		/* the device is no longer valid/functioning */
		return (ENXIO);
	}
	if ((skystar->flags & SKYSTAR_INITIALIZED) == NULL) {
		/* the device was not properly initialized */
		return (ENXIO);
	}
	return 0;
}

/*
 *	device function: CLOSE
 */
int
skystar_close(dev_t dev, int flags, int fmt, struct proc * p)
{
	struct skystar_softc *skystar = (struct skystar_softc *) device_lookup(&skystar_cd, minor(dev));

	if (skystar == NULL) {
		/* the device is no longer valid/functioning */
		return (ENXIO);
	}
	return 0;
}

/*
 *	device function: READ
 */
int
skystar_read(dev_t dev, struct uio * uio, int ioflag)
{
	/* read() not supported!\n" ); */
	return ENXIO;
};

/*
 *	device function: WRITE
 */
int
skystar_write(dev_t dev, struct uio * uio, int ioflag)
{
	return ENXIO;
};

/*
 *	device function: IOCTL
 */
int
skystar_ioctl(dev_t dev, u_long cmd, caddr_t arg, int flags, struct proc * pr)
{
	struct skystar_softc *skystar = (struct skystar_softc *) device_lookup(&skystar_cd, minor(dev));
	int             error = 0;
	/* int s;	 */

	/* printf("skystar%08x: ioctl.\n", dev); */

	if (skystar == NULL) {
		/* the device is no longer valid/functioning */
#ifdef SKYSTAR_DEBUG
		printf("skystar%d: ioctl => device is not valid.\n", minor(dev));
#endif
		return (ENXIO);
	}
	switch (cmd) {
	case VIDIOCGFRONTEND:	/* device frontend get */
		dvb_command(skystar, DVB_GET_FRONTEND, arg);
		return 0;
	case VIDIOCSFRONTEND:	/* device frontend set */
		SetFront(skystar, (struct frontend *) arg);
		return 0;
	case VIDIOCSBITFILTER:	/* device bitfilter set */
		if (hw_filter_set(skystar, (struct bitfilter *) arg) < 0)
			error = EFAULT;
		return 0;
		/* DVB ioctls */
		/* case VIDIOCSSHUTDOWNFILTER: */
		/* case VIDIOCSDISEQCMSG: */
		/* case VIDIOCSCIMSG: */
		/* case VIDIOCSOSDCOMMAND: */
		/* case VIDIOCGVIDEOSTATE: */
		/* case VIDIOCSASPECTRATIO: */
		/* case VIDIOCSPANSCAN: */
		/* case VIDIOCSFREEZEMODE: */
		/* case VIDIOCSPLAYMODE: */
		/* case VIDIOCSWRITEMODE: */
	default:		/* unsupported ioctl */
		printf("skystar%d: unsupported ioctl %08x.\n", minor(dev), (unsigned) cmd);
		error = EINVAL;
	}
	return error;
}

/* DVB hardware initialization */
void 
skystar_init (struct skystar_softc *skystar)
{
        int	i;
#ifdef SKYSTAR_DEBUG
	printf("%s DEBUG: initializing DVB hardware...\n", skystar->ss_dev.dv_xname);
#endif

	/* initialize hardware filters */
	for (i = 0; i < MAXFILT; i++) {
		skystar->filter[i].flags = 0x00;
		skystar->filter[i].state = 0x00;
		skystar->hfilter[i] = NULL;
	}
	skystar->debitransfer = -1;	/* all DEBI transfers off */

	/* initialize DVB processor */
	/* enable DEBI */

#if 0	
	write_long (skystar, MC1, 0x08800880);
	write_long (skystar, 0x54, 0x04000000); /* WTF? 0x54 - DD1 chan B control */
	write_long (skystar, MC2, (MASK_09 | MASK_25 | MASK_10 | MASK_26));
#endif
	printf("\n MASK_07 | MASK_11 | MASK_23 | MASK_27 = 0x%x\n", MASK_07 | MASK_11 | MASK_23 | MASK_27);
	write_long(skystar, MC1, (MASK_07 | MASK_11 | MASK_23 | MASK_27));
	write_long(skystar, DD1_STREAM, MASK_26); 	/* Polarity change of the field identification signal at Port_A  to inverted = 1  */
	write_long(skystar, MC2, (MASK_09 | MASK_25 | MASK_10 | MASK_26));


	arm_boot(skystar);
	get_fw_version(skystar);

	bzero(&skystar->front, sizeof(struct frontend));
	switch (dvb_detect(skystar)) {
	case BSRV2:		/* this is release 1.3		 */
#ifdef SKYSTAR_DEBUG
		printf("skystar: VES1893 found, assume rev 1.3\n");
#else
		printf(", frontend VES1893, rev 1.3\n");
#endif
		skystar->front.type = FRONT_DVBS;
		break;
	case BSRU6:		/* this is release 1.5		 */
#ifdef SKYSTAR_DEBUG
		printf("skystar: STV0299 found, assume rev 1.5\n");
#else
		printf(", frontend STV0299, rev 1.5\n");
#endif
		skystar->front.type = FRONT_DVBS;
		break;
	default:		/* we don't know this card	 */
		printf("no DVB chip!");
		skystar->front.type = FRONT_NONE;
		return;
	}

	/* initialize i2c clients */
	skystar->tuner_addr =  TSA5059_I2C_ADDR;	/* valid only for SkyStar1 */
	skystar->tuner_flags = 0;

	/* TODO: allocate iobuffers */
	/* TODO: initialize DEBI queue */
	/* enable DEBI and GPIO ints */
	write_long(skystar, ISR, (MASK_19 | MASK_03));
	write_long(skystar, IER, read_long(skystar, IER) | MASK_03);
#ifdef SKYSTAR_DEBUG
        printf("skystar: IER status 0x%8x\n", read_long(skystar, IER));
        printf("skystar: ISR status 0x%8x\n", read_long(skystar, ISR));
#endif

	/* initialize frontend */
	InitFront(skystar);

	/* All done. */
	shutdownhook_establish(skystar_shutdown, skystar);
	skystar->flags |= SKYSTAR_INITIALIZED;


	/* Initialize network		 */
	dvb_init(skystar);
	return;
}


/*
 *                             tuner control
 */

void
set_tv_freq(struct skystar_softc * skystar, u_int32_t freq)
{
	u_char			config;
	u_int32_t			div2;
	u_int16_t			div;
	struct tunertype	*tun;
	struct i2c_msg		msg;
	unsigned char		buffer[4];
	int			rc;

	if (skystar->tuner_type == -1) {
		printf("skystar->tuner: tuner type not set\n");
		return;
	}
	tun = &tuners[skystar->tuner_type];
	freq *= tun->step;	/* turn into Hz */

	if (freq < tun->thresh1)
		config = tun->VHF_L;
	else if (freq < tun->thresh2)
		config = tun->VHF_H;
	else
		config = tun->UHF;

	config &= ~tun->mode;

	div2 = freq + tun->IFPCoff;
	div2 /= tun->res;
	div = (u_int16_t) div2;

	buffer[0] = (div >> 8) & 0x7f;
	buffer[1] = div & 0xff;
	buffer[2] = tun->config;
	buffer[3] = config;
       /* make message */
	msg.addr = skystar->tuner_addr;
	msg.flags = skystar->tuner_flags & I2C_M_TEN;
	msg.len = 4;
	msg.buf = buffer;
	/* send it */
	rc = i2c_transfer(skystar, &msg, 1);
	/* report any error */
	if (rc != 1)
	//&& rc != 4)
		printf("skystar->tuner: i2c i/o error: rc == %d\n", rc);
}

int
tuner_command(struct skystar_softc * skystar,
	      unsigned int cmd, void *arg)
{
	unsigned int   *iarg = (int *) arg;

	switch (cmd) {
		/* set frequency */
	case TUNER_SET_TVFREQ:
#ifdef SKYSTAR_DEBUG
		printf("skystar->tuner: tv freq set to %d\n", (*iarg));
#endif
		set_tv_freq(skystar, *iarg);
		skystar->tuner_freq = *iarg;
		break;
	case TUNER_SET_TYPE:
		if (skystar->tuner_type != -1)
			return 0;
		if (*iarg < 0 || *iarg >= TUNERS)
			return 0;
		skystar->tuner_type = *iarg;
#ifdef SKYSTAR_DEBUG
		printf("tuner: type set to %d (%s)\n", skystar->tuner_type, tuners[skystar->tuner_type].name);
#endif
		break;
      	default:
#ifdef SKYSTAR_DEBUG
		printf("skystar->tuner: unknown command %08x\n", cmd);
#endif
		return EINVAL;
	}
	return 0;
}
