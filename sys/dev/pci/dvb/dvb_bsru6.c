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

/*
 *                            Here we begin BSRU6 stuff
 */

#define STV0299_I2C_FLAGS	0x00

#define M_CLK (88000000UL)

#ifdef SKYSTAR_DEBUG
#define BSRU6_DEBUG
#endif

/* M=21, K=0, P=0, f_VCO = 4MHz*4*(M+1)/(K+1) = 352 MHz */

static struct dvb_frontend_info bsru6_info = {
#ifdef CONFIG_ALPS_BSRU6_IS_LG_TDQBS00X
	name:"LG TDQB-S00x",
#else
	name:"Alps BSRU6",
#endif
	type:FE_QPSK,
	frequency_min:950000,
	frequency_max:2150000,
	frequency_stepsize:125,	/* kHz for QPSK frontends */
	frequency_tolerance:M_CLK / 2000,
	symbol_rate_min:1000000,
	symbol_rate_max:45000000,
	symbol_rate_tolerance:500,	/* ppm */
	notifier_delay:0,
	caps:FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
	FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
	FE_CAN_QPSK
};

static const struct init_tab {
	u_int32_t	addr;
	u_int32_t	data;
} init_regs[] = {
	{ RCR,		0x15 },	/* M: 0x15 DIRCLK: 0 K:0	 */
	{ MCR,		0x30 },	/* P: 0  SERCLK: 0 VCO:ON  STDBY:0	 */
	{ ACR,		0x00 },
	{ F22FR,	0x7d },	/* F22FR, F22=22kHz	 */
	{ I2CRPT,	0x35 },	/* SDAT:0 SCLT:0 I2CT:1	 */
	{ DACR1,	0x00 },	/* DAC mode and MSB	 */
	{ DACR2,	0x00 },	/* DAC LSB	 */
	{ DISEQC,	0x03 },	/* DiSEqC	 */
	{ DISEQCFIFO,	0x00 },
	{ IOCFG,	0x51 },	/* QPSK reverse:1  Nyquist:0 OP0 val:1 OP0 con:1 OP1 val:1 OP1 con:1	 */
	{ AGC1C,	0x82 },
	{ RTC,		0x23 },
	{ AGC1R,	0x52 },
	{ AGC2O,	0x3d },	/* AGC2	 */
	{ TLSR,		0x84 },
	{ CFD,		0xb5 },	/* Lock detect: -64  Carrier freq detect:on	 */
	{ ACLC,		0xb6 },	/* alpha_car b:4 a:0  noise est:256ks derot:on	 */
	{ BCLC,		0x93 },	/* beat carc:0 d:0 e:0xf  phase detect algo: 1	 */
	{ CLDT, 	0xc9 },	/* lock detector threshold	 */
	{ AGC1I,	0x1d },
	{ TLIR,		0x00 },
	{ AGC2I1,	0x14 },
	{ AGC2I2,	0xf2 },
	{ RTF,		0x11 },
	{ SFRH,		0x50 },
	{ SFRM,		0x00 },
	{ SFRL,		0x00 },
	{ CFRM,		0x00 },
	{ CFRL,		0x00 },
	{ FECM,		0x00 },	/* out imp: normal  out type: parallel FEC  mode:0 */
	{ VTH0,		0x1e },	/* 1/2 threshold */
	{ VTH1,		0x14 },	/* 2/3 threshold */
	{ VTH2,		0x0f },	/* 3/4 threshold */
	{ VTH3,		0x09 },	/* 5/6 threshold */
	{ VTH4,		0x05 },	/* 7/8 threshold */
	{ PR,		0x1f },	/* test all FECs */
	{ VSEARCH,	0x19 },	/* viterbi and synchro search */
	{ RS,		0xfc },	/* rs control */
	{ ERRCNT,	0x93 },	/* error control */
};
#define INIT_TABLE_SIZE ( sizeof(init_regs) / sizeof(init_regs[0]) )

static int 
writereg(struct skystar_softc * skystar, int reg, int data)
{
	int             ret;
	unsigned char   msg[] = {0x00, 0x00};
	struct i2c_msg  m;
	m.addr = STV0299_I2C_ADDR;
	m.flags = STV0299_I2C_FLAGS & I2C_M_TEN;
	m.len = 2;
	m.buf = msg;
	msg[0] = reg;
	msg[1] = data;
	ret = i2c_transfer(skystar, &m, 1);
	if (ret != 1)		/* AR: I mean it returns number of block
				 * transferred. Isn't it ?	 */
		printf("%s ! writereg() error\n", skystar->ss_dev.dv_xname);
	return ret;
}

static u_char 
readreg(struct skystar_softc * skystar, u_char reg)
{
	unsigned char   mm1[] = {0x1e};
	unsigned char   mm2[] = {0x00};
	struct i2c_msg  msgs[2];
	u_char          ret;
	msgs[0].flags = 0;
	msgs[1].flags = I2C_M_RD;
	msgs[0].addr = msgs[1].addr = STV0299_I2C_ADDR;
	mm1[0] = reg;
	msgs[0].len = 1;
	msgs[1].len = 1;
	msgs[0].buf = mm1;
	msgs[1].buf = mm2;
	ret = i2c_transfer(skystar, msgs, 2);
	if (ret != 2)
		printf("%s readreg() error\n", skystar->ss_dev.dv_xname);
	return mm2[0];
}

static int 
tsa5059_write(struct skystar_softc * skystar, u_char data[4])
{
	u_char          rpt1[2];
	struct i2c_msg  m[2];

        /* enable i2c repeater, to get tsa5059 behind stv0299 */
        rpt1[0] = I2CRPT;
        rpt1[1] = 0xb5;
	m[0].addr = STV0299_I2C_ADDR;
	m[0].flags = I2C_M_TEN;
	m[0].buf = rpt1;
	m[0].len = 2;
       /* data to tsa5059 */
	m[1].addr = TSA5059_I2C_ADDR;
	m[1].flags = I2C_M_TEN;
	m[1].len = 4;
	m[1].buf = data;

	if (i2c_transfer(skystar, &m[0], 1)!= 1) {
		printf("%s ts5059_write(1) error !\n", skystar->ss_dev.dv_xname);
		return -1;
	}
	if (i2c_transfer(skystar, &m[1], 1)!= 1) {
		printf("%s ts5059_write(2) error !\n", skystar->ss_dev.dv_xname);
		return -1;
	}
	return 0;
}

/*
 *   set up the downconverter frequency divisor for a
 *   reference clock comparision frequency of 125 kHz.
 *   freq is KHz !!!
 */
static int 
tsa5059_set_tv_freq(struct skystar_softc * skystar, u_int32_t freq, u_char pwr)
{
	u_int32_t	div = freq / 125;
	u_char		buf[4] = {(div >> 8) & 0x7f, div & 0xff, 0x84, (pwr << 5) | 0x20};
	return tsa5059_write(skystar, buf);
}

#ifdef BSRU6_DEBUG
static void 
dump(struct skystar_softc * skystar)
{
	int             i;
	printf("--- STV0229c registers dump follows ---");
	for (i = 0; i < 0x34; i++) {
		if (!(i & 15))
			printf("\n%03x:", i);
		printf(" %02x", readreg(skystar, i));
	}
	printf("\n");
}
#endif

static int 
init(struct skystar_softc * skystar)
{
	const struct init_tab	*it = init_regs;
        int		i = 0;
#ifdef BSRU6_DEBUG
	printf("%s stv0299: init chip\n", skystar->ss_dev.dv_xname);
#endif
	
	for (i=0; i< INIT_TABLE_SIZE; i++){
		writereg(skystar, it[i].addr, it[i].data);
	}

	skystar->front.freq = 0;
	skystar->front.srate = 0;
	skystar->front.fec = 9;
	skystar->front.inv = 0;

	return 0;
}


static int 
SetInversion(struct skystar_softc * skystar, int inversion)
{
	u_char          val = 0;

	if (inversion == skystar->front.inv)
		return 0;

	skystar->front.inv = inversion;

	val = readreg(skystar, IOCFG);

	switch (inversion) {
	case INVERSION_AUTO:	/* Hmm, return EINVAL or leave it like this? */
	default:
	case INVERSION_OFF:
		return writereg(skystar, IOCFG, val | 0x01);
	case INVERSION_ON:
		return writereg(skystar, IOCFG, val & 0xfe);
	}
	return 0;
}

static int 
SetFEC(struct skystar_softc * skystar, u_char fec)
{
	if (skystar->front.fec == fec)
		return 0;
	skystar->front.fec = fec;

	switch (fec) {
	case FEC_AUTO:
		return writereg(skystar, PR, 0x1f);
	case FEC_1_2:
		return writereg(skystar, PR, 0x01);
	case FEC_2_3:
		return writereg(skystar, PR, 0x02);
	case FEC_3_4:
		return writereg(skystar, PR, 0x04);
	case FEC_5_6:
		return writereg(skystar, PR, 0x08);
	case FEC_7_8:
		return writereg(skystar, PR, 0x10);
	default:
		return -EINVAL;
	}
}

static int 
SetTone(struct skystar_softc * skystar, u_int tone)
{
	u_char          val;

	val = readreg(skystar, DISEQC);

	switch (tone) {
	case SEC_TONE_ON:
		return writereg(skystar, DISEQC, val | 0x3);
	case SEC_TONE_OFF:
		return writereg(skystar, DISEQC, (val & ~0x3) | 0x02);
	default:
		return -EINVAL;
	};
}

static int 
SetVoltage(struct skystar_softc * skystar, u_int power, u_int volt)
{
	u_char          val;

	if (power == skystar->front.power
	    && volt == skystar->front.volt)
		return 0;

#ifdef BSRU6_DEBUG
	printf("%s bsru6: setting volt:%u\n", volt, skystar->ss_dev.dv_xname);
#endif
	skystar->front.power = power;
	skystar->front.volt = volt;

	val = readreg(skystar, IOCFG);
	val &= 0x0f;
	val |= 0x40;

	switch (volt) {
	case SEC_VOLTAGE_13:
		return writereg(skystar, IOCFG, val);
	case SEC_VOLTAGE_18:
		return writereg(skystar, IOCFG, val | 0x10);
	default:
		return -EINVAL;
	};
}


static int 
SetSymbolrate(struct skystar_softc * skystar, u_int32_t srate)
{
	u_int32_t	ratio;
	u_int32_t	tmp;
	u_char		aclk = 0xb4, bclk = 0x51;

	if (srate > M_CLK)
		srate = M_CLK;
	if (srate < 500000)
		srate = 500000;
	skystar->front.srate = srate;

	if (srate < 30000000) {
		aclk = 0xb6;
		bclk = 0x53;
	}
	if (srate < 14000000) {
		aclk = 0xb7;
		bclk = 0x53;
	}
	if (srate < 7000000) {
		aclk = 0xb7;
		bclk = 0x4f;
	}
	if (srate < 3000000) {
		aclk = 0xb7;
		bclk = 0x4b;
	}
	if (srate < 1500000) {
		aclk = 0xb7;
		bclk = 0x47;
	}
#define FIN (M_CLK>>4)
	tmp = srate << 4;
	ratio = tmp / FIN;

	tmp = (tmp % FIN) << 8;
	ratio = (ratio << 8) + tmp / FIN;

	tmp = (tmp % FIN) << 8;
	ratio = (ratio << 8) + tmp / FIN;

	/* ratio&=0xfffff0;	 */

	writereg(skystar, ACLC, aclk);
	writereg(skystar, BCLC, bclk);
	writereg(skystar, SFRH, (ratio >> 16) & 0xff);
	writereg(skystar, SFRM, (ratio >> 8) & 0xff);
	writereg(skystar, SFRL, (ratio) & 0xff);

	skystar->dvb.bsru6.aclk = aclk;
	skystar->dvb.bsru6.bclk = bclk & 0x3f;

	return 1;
}

int 
dvb_bsru6_detect(struct skystar_softc * skystar)
{
	if ((readreg(skystar, ID)) != 0xa1) {
#ifdef SKYSTAR_DEBUG
		printf("%s stv_detect() not detected\n", skystar->ss_dev.dv_xname);
#endif
		return DVB_NONE;
	}
	init(skystar);
	return BSRU6;
}


int 
dvb_bsru6_command(struct skystar_softc * skystar, unsigned int cmd, void *arg)
{
	switch (cmd) {
#if 0
	case DVB_RESET:
		{
#ifdef BSRU6_DEBUG
			printf( "%s DVB_RESET\n", skystar->ss_dev.dv_xname);
#endif
			writereg(skystar, CFRM, 0x00);
			writereg(skystar, CFRL, 0x00);
			readreg(skystar, CFRL);
			writereg(skystar, CFD, 0xb9);
			break;
		}
#endif /* if 0 */

	case DVB_GET_INFO:
		{
			struct dvb_frontend_info *info = (struct dvb_frontend_info *) arg;
			*info = bsru6_info;
			break;
		}
	case DVB_SET_FRONTEND:
		{
			struct frontend *front = (struct frontend *) arg;
			tsa5059_set_tv_freq(skystar, front->freq, 3);
			
			SetInversion(skystar, front->inv);
			SetVoltage(skystar, front->power, front->volt);
			SetFEC(skystar, front->fec);
			SetTone(skystar, front->ttk);
			SetSymbolrate(skystar, front->srate);
			tsa5059_set_tv_freq(skystar, front->freq, 3);

			writereg(skystar, CFRM, 0x00);
			writereg(skystar, CFRL, 0x00);
			readreg(skystar, CFRL);
			writereg(skystar, CFD, 0xb9);
#ifdef BSRU6_DEBUG			
			 dump(skystar); 
#endif
			break;
		}
	case DVB_GET_FRONTEND:
		{
			struct frontend *front = (struct frontend *) arg;
			u_char          signal = 0xff - readreg(skystar, 0x18);
			u_char          sync = readreg(skystar, 0x1b);

			front->afc = 0;	/* ((int)((char)(readreg(client,0x0a)<
					 * <1)))/2;	 */
			front->afc = 0;	/* /(front->afc*(int)(front->srate/8))
					 * /16;	 */
			front->sync = 0;

			/* sync status */
			if (signal > 10)
				front->sync |= DVB_SYNC_SIGNAL;
			if (sync & 0x80)
				front->sync |= DVB_SYNC_CARRIER;
			if (sync & 0x10)
				front->sync |= DVB_SYNC_VITERBI;
			if (sync & 0x08)
				front->sync |= DVB_SYNC_FSYNC;
			if ((sync & 0x98) == 0x98)
				front->sync |= DVB_SYNC_FRONT;

			/* signal/noice ratio */
			front->nest = (readreg(skystar, NIRH) << 8) | readreg(skystar, NIRL);

			/* gain (signal strength) */
			front->agc = ((readreg(skystar, AGC2I1) << 8)
				      | readreg(skystar, AGC2I2));

			/* viterbi eror rate */
			front->vber = (readreg(skystar, ERRCNTH) << 8) | readreg(skystar, ERRCNTL);

#if 0
			if (front->sync) {
				writereg(skystar, BCLC, 0x80 | skystar->dvb.bsru6.bclk);
				writereg(skystar, CFD, 0x39 );
			}

#ifdef BSRU6_DEBUG
			if (vstatus&0x90)
				printf("%s vstatus=0x%02x  cldi=0x%02x  nest=0x%04x\n", skystar->ss_dev.dv_xname,
				    vstatus, readreg(skystar, CLDI), front->nest);
#endif
			
#endif
			break;
		}
	default:
		return -1;
	}
	return 0;
}
