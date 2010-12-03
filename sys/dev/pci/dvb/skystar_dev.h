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

#ifndef __h_skystar_dev
#define __h_skystar_dev
#define SKYSTAR_DEBUG 1 /* set 1 to debug */
/* Miscellaneous constants and defines */
/* Bit mask constants */
#define MASK_00		0x00000001	/* Mask value for bit 0 */
#define MASK_01		0x00000002	/* Mask value for bit 1 */
#define MASK_02		0x00000004	/* Mask value for bit 2 */
#define MASK_03		0x00000008	/* Mask value for bit 3 */
#define MASK_04		0x00000010	/* Mask value for bit 4 */
#define MASK_05		0x00000020	/* Mask value for bit 5 */
#define MASK_06		0x00000040	/* Mask value for bit 6 */
#define MASK_07		0x00000080	/* Mask value for bit 7 */
#define MASK_08		0x00000100	/* Mask value for bit 8 */
#define MASK_09		0x00000200	/* Mask value for bit 9 */
#define MASK_10		0x00000400	/* Mask value for bit 10 */
#define MASK_11		0x00000800	/* Mask value for bit 11 */
#define MASK_12		0x00001000	/* Mask value for bit 12 */
#define MASK_13		0x00002000	/* Mask value for bit 13 */
#define MASK_14		0x00004000	/* Mask value for bit 14 */
#define MASK_15		0x00008000	/* Mask value for bit 15 */
#define MASK_16		0x00010000	/* Mask value for bit 16 */
#define MASK_17		0x00020000	/* Mask value for bit 17 */
#define MASK_18		0x00040000	/* Mask value for bit 18 */
#define MASK_19		0x00080000	/* Mask value for bit 19 */
#define MASK_20		0x00100000	/* Mask value for bit 20 */
#define MASK_21		0x00200000	/* Mask value for bit 21 */
#define MASK_22		0x00400000	/* Mask value for bit 22 */
#define MASK_23		0x00800000	/* Mask value for bit 23 */
#define MASK_24		0x01000000	/* Mask value for bit 24 */
#define MASK_25		0x02000000	/* Mask value for bit 25 */
#define MASK_26		0x04000000	/* Mask value for bit 26 */
#define MASK_27		0x08000000	/* Mask value for bit 27 */
#define MASK_28		0x10000000	/* Mask value for bit 28 */
#define MASK_29		0x20000000	/* Mask value for bit 29 */
#define MASK_30		0x40000000	/* Mask value for bit 30 */
#define MASK_31		0x80000000	/* Mask value for bit 31 */

#define MASK_B0		0x000000ff	/* Mask value for byte 0 */
#define MASK_B1		0x0000ff00	/* Mask value for byte 1 */
#define MASK_B2		0x00ff0000	/* Mask value for byte 2 */
#define MASK_B3		0xff000000	/* Mask value for byte 3 */

#define MASK_W0		0x0000ffff	/* Mask value for word 0 */
#define MASK_W1		0xffff0000	/* Mask value for word 1 */

#define MASK_PA		0xfffffffc	/* Mask value for physical address */
#define MASK_PR		0xfffffffe	/* Mask value for protection register */
#define MASK_ER		0xffffffff	/* Mask value for the entire register */

#define MASK_NONE	0x00000000	/* No mask */

/* SAA7146 stuff */

/* REGISTERS */
#define BASE_ODD1		0x00	/* Video DMA 1 registers  */
#define BASE_EVEN1		0x04
#define PROT_ADDR1		0x08
#define PITCH1			0x0C
#define BASE_PAGE1		0x10	/* Video DMA 1 base page */
#define NUM_LINE_BYTE1		0x14
#define BASE_ODD2		0x18	/* Video DMA 2 registers */
#define BASE_EVEN2		0x1C
#define PROT_ADDR2		0x20
#define PITCH2			0x24
#define BASE_PAGE2		0x28	/* Video DMA 2 base page */
#define NUM_LINE_BYTE2		0x2C
#define BASE_ODD3		0x30	/* Video DMA 3 registers */
#define BASE_EVEN3		0x34
#define PROT_ADDR3		0x38
#define PITCH3			0x3C
#define BASE_PAGE3		0x40	/* Video DMA 3 base page */
#define NUM_LINE_BYTE3		0x44
#define PCI_BT_V1		0x48	/* Video/FIFO 1 */
#define PCI_BT_V2		0x49	/* Video/FIFO 2 */
#define PCI_BT_V3		0x4A	/* Video/FIFO 3 */
#define PCI_BT_DEBI		0x4B	/* DEBI */
#define PCI_BT_A		0x4C	/* Audio */
#define DD1_INIT		0x50	/* Init setting of DD1 interface */
#define DD1_STREAM		0x54	/* DD1 A and B video data stream handling  A - bits 31..16, B - 15..0 */
#define BRS_CTRL		0x58	/* BRS control register */
#define HPS_CTRL		0x5C	/* HPS control register */
#define HPS_V_SCALE		0x60	/* HPS vertical scale */
#define HPS_V_GAIN		0x64	/* HPS vertical ACL and gain */
#define HPS_H_PRESCALE		0x68	/* HPS horizontal prescale   */
#define HPS_H_SCALE		0x6C	/* HPS horizontal scale */
#define BCS_CTRL		0x70	/* BCS control */
#define CHROMA_KEY_RANGE	0x74
#define CLIP_FORMAT_CTRL	0x78	/* HPS outputs formats & clipping */
#define DEBI_CONFIG		0x7C
#define DEBI_COMMAND		0x80
#define DEBI_PAGE		0x84
#define DEBI_AD			0x88
#define I2C_TRANSFER		0x8C
#define I2C_STATUS		0x90
#define BASE_A1_IN		0x94	/* Audio 1 input DMA */
#define PROT_A1_IN		0x98
#define PAGE_A1_IN		0x9C
#define BASE_A1_OUT		0xA0	/* Audio 1 output DMA */
#define PROT_A1_OUT		0xA4
#define PAGE_A1_OUT		0xA8
#define BASE_A2_IN		0xAC	/* Audio 2 input DMA */
#define PROT_A2_IN		0xB0
#define PAGE_A2_IN		0xB4
#define BASE_A2_OUT		0xB8	/* Audio 2 output DMA */
#define PROT_A2_OUT		0xBC
#define PAGE_A2_OUT		0xC0
#define RPS_PAGE0		0xC4	/* RPS task 0 page register */
#define RPS_PAGE1		0xC8	/* RPS task 1 page register */
#define RPS_THRESH0		0xCC	/* HBI threshold for task 0 */
#define RPS_THRESH1		0xD0	/* HBI threshold for task 1 */
#define RPS_TOV0		0xD4	/* RPS timeout for task 0 */
#define RPS_TOV1		0xD8	/* RPS timeout for task 1 */
#define IER			0xDC	/* Interrupt enable register */
#define GPIO_CTRL		0xE0	/* GPIO 0-3 register */
#define EC1SSR			0xE4	/* Event cnt set 1 source select */
#define EC2SSR			0xE8	/* Event cnt set 2 source select */
#define ECT1R			0xEC	/* Event cnt set 1 thresholds */
#define ECT2R			0xF0	/* Event cnt set 2 thresholds */
#define ACON1			0xF4
#define ACON2			0xF8
#define MC1			0xFC	/* Main control register 1 */
#define MC2			0x100	/* Main control register 2  */
#define RPS_ADDR0		0x104	/* RPS task 0 address register */
#define RPS_ADDR1		0x108	/* RPS task 1 address register */
#define ISR			0x10C	/* Interrupt status register */
#define PSR			0x110	/* Primary status register */
#define SSR			0x114	/* Secondary status register */
#define EC1R			0x118	/* Event counter set 1 register */
#define EC2R			0x11C	/* Event counter set 2 register */
#define PCI_VDP1		0x120	/* Video DMA pointer of FIFO 1 */
#define PCI_VDP2		0x124	/* Video DMA pointer of FIFO 2 */
#define PCI_VDP3		0x128	/* Video DMA pointer of FIFO 3 */
#define PCI_ADP1		0x12C	/* Audio DMA pointer of audio out 1 */
#define PCI_ADP2		0x130	/* Audio DMA pointer of audio in 1 */
#define PCI_ADP3		0x134	/* Audio DMA pointer of audio out 2 */
#define PCI_ADP4		0x138	/* Audio DMA pointer of audio in 2 */
#define PCI_DMA_DDP		0x13C	/* DEBI DMA pointer */
#define LEVEL_REP		0x140,
#define A_TIME_SLOT1		0x180,/* from 180 - 1BC */
#define A_TIME_SLOT2		0x1C0,/* from 1C0 - 1FC */

/* ISR-MASKS */
#define SPCI_PPEF	MASK_31	/* PCI parity error */
#define SPCI_PABO	MASK_30	/* PCI access error (target or master abort) */
#define SPCI_PPED	MASK_29	/* PCI parity error on 'real time data' */
#define SPCI_RPS_I1	MASK_28	/* Interrupt issued by RPS1 */
#define SPCI_RPS_I0	MASK_27	/* Interrupt issued by RPS0 */
#define SPCI_RPS_LATE1	MASK_26	/* RPS task 1 is late */
#define SPCI_RPS_LATE0	MASK_25	/* RPS task 0 is late */
#define SPCI_RPS_E1	MASK_24	/* RPS error from task 1 */
#define SPCI_RPS_E0	MASK_23	/* RPS error from task 0 */
#define SPCI_RPS_TO1	MASK_22	/* RPS timeout task 1 */
#define SPCI_RPS_TO0	MASK_21	/* RPS timeout task 0 */
#define SPCI_UPLD	MASK_20	/* RPS in upload */
#define SPCI_DEBI_S	MASK_19	/* DEBI status */
#define SPCI_DEBI_E	MASK_18	/* DEBI error */
#define SPCI_IIC_S	MASK_17	/* I2C status */
#define SPCI_IIC_E	MASK_16	/* I2C error */
#define SPCI_A2_IN	MASK_15	/* Audio 2 input DMA protection / limit */
#define SPCI_A2_OUT	MASK_14	/* Audio 2 output DMA protection / limit */
#define SPCI_A1_IN	MASK_13	/* Audio 1 input DMA protection / limit */
#define SPCI_A1_OUT	MASK_12	/* Audio 1 output DMA protection / limit */
#define SPCI_AFOU	MASK_11	/* Audio FIFO over- / underflow */
#define SPCI_V_PE	MASK_10	/* Video protection address */
#define SPCI_VFOU	MASK_09	/* Video FIFO over- / underflow */
#define SPCI_FIDA	MASK_08	/* Field ID video port A */
#define SPCI_FIDB	MASK_07	/* Field ID video port B */
#define SPCI_PIN3	MASK_06	/* GPIO pin 3 */
#define SPCI_PIN2	MASK_05	/* GPIO pin 2 */
#define SPCI_PIN1	MASK_04	/* GPIO pin 1 */
#define SPCI_PIN0	MASK_03	/* GPIO pin 0 */
#define SPCI_ECS	MASK_02	/* Event counter 1, 2, 4, 5 */
#define SPCI_EC3S	MASK_01	/* Event counter 3 */
#define SPCI_EC0S	MASK_00	/* Event counter 0 */

/*
 *                       Internal DVB-related structures
 */

/*
 * Definition der Speicherbereiche und Register des DPRAM (von PC aus
 * gesehen) und des DRAM (am AV7110)
 */
#define	DPRAM_BASE	0x4000
#define DPRAM_SIZE	0x2000
#define BOOT_MAX_SIZE	0x800
#define BOOT_STATE	(DPRAM_BASE + 0x7F8)
#define BOOT_SIZE	(DPRAM_BASE + 0x7FA)
#define BOOT_BASE	(DPRAM_BASE + 0x7FC)
#define BOOT_BLOCK	(DPRAM_BASE + 0x800)

#define IRQ_STATE	(DPRAM_BASE + 0x0F4)
#define IRQ_STATE_EXT	(DPRAM_BASE + 0x0F6)
#define MSGSTATE	(DPRAM_BASE + 0x0F8)
#define FILT_STATE	(DPRAM_BASE + 0x0FA)
#define COMMAND		(DPRAM_BASE + 0x0FC)
#define COM_BUFF	(DPRAM_BASE + 0x100)
#define COM_BUFF_SIZE	0x20
#define BUFF1_BASE	(DPRAM_BASE + 0x120)
#define BUFF1_SIZE	0xE0
#define DATA_BUFF_BASE	(DPRAM_BASE + 0x200)
#define DATA_BUFF_SIZE	0x1C00
#define Reserved	(DPRAM_BASE + 0x1E00)
#define HANDSHAKE_REG	(DPRAM_BASE + 0x1FF8)
#define COM_IF_LOCK	(DPRAM_BASE + 0x1FFA)
#define IRQ_RX		(DPRAM_BASE + 0x1FFC)
#define IRQ_TX		(DPRAM_BASE + 0x1FFE)

#define DRAM_START_CODE		0x2e000404
#define DRAM_MAX_CODE_SIZE	0x00100000

#define RESET_LINE		2
#define DEBI_DONE_LINE		1
#define ARM_IRQ_LINE		0

#define DAC_CS		0x8000
#define DAC_CDS		0x0000

/* ??? */
typedef enum BOOTSTATES {
	BOOTSTATE_BUFFER_EMPTY = 0,
	BOOTSTATE_BUFFER_FULL = 1,
	BOOTSTATE_BOOT_COMPLETE = 2
} BOOTSTATES;

typedef enum GPIO_MODE {
	GPIO_INPUT = 0x00,
	GPIO_IRQHI = 0x10,
	GPIO_IRQLO = 0x20,
	GPIO_IRQHL = 0x30,
	GPIO_OUTLO = 0x40,
	GPIO_OUTHI = 0x50
} GPIO_MODE;

/* data types */
#define DATA_TELETEXT		0x00
#define DATA_FSECTION		0x01
#define DATA_IPMPE		0x02
#define DATA_MPEG_RECORD	0x03
#define DATA_DEBUG_MESSAGE	0x04
#define DATA_COMMON_INTERFACE	0x05
#define DATA_MPEG_PLAY		0x06
#define DATA_BMP_LOAD		0x07
#define DATA_IRCOMMAND		0x08
#define DATA_PIPING		0x09
#define DATA_STREAMING		0x0a

#ifdef REMOVED
/* Command codes */
typedef enum {
	WCreate,
	WDestroy,
	WMoveD,
	WMoveA,
	WHide,
	WTop,
	DBox,
	DLine,
	DText,
	Set_Font,
	SetColor,
	SetBlend,
	SetWBlend,
	SetCBlend,
	SetNonBlend,
	LoadBmp,
	BlitBmp,
	ReleaseBmp
}               OSDCOM;

typedef enum {
	VideoPID,
	AudioPID,
	WaitVideoPlay,
	WaitUnMute,
	InitFilt,
	AddFilt8,
	AddFilt16,
	DeleteFilt,
	FiltError,
	RecSection,
	NewVersion,
	CacheError,
	SetFiltBytes,
	SetRange,
	AddSecFilt,
	AddRangeFilt,
	AddFilt12,
	MultiPID,
	AddPIDFilter,
	DelPIDFilter,
	TestSec
}               PIDCOM;

typedef enum {
	SelAudChannels
}               MPEGCOM;

typedef enum {
	AudioDAC,
	CabADAC,
	ON22K,
	OFF22K,
	MainSwitch,
	ADSwitch,
	SendDiSEqC,
	OFDM_Channel,
	OFDM_Guard,
	OFDM_MpegBer,
	SetRegister,
	SetIrControl,
	LoadTTuner
}               AUDCOM;

typedef enum {
	TTXEnable,
	TTXDisable,
	ModifyTable,
	ReqestP29_P830,
	NoPageMemory,
	NoPESMemory,
	PESPacket,
	TTXPage,
	P29_P830,
	DroppedPages
}               TTXCOM;

typedef enum {
	AudioState = 0,
	AudioBuffState = 1,
	VideoState1 = 2,
	VideoState2 = 3,
	VideoState3 = 4,
	CrashCounter = 5,
	ReqVersion = 6,
	ReqVCXO = 7,
	ReqRegister = 8
} REQCOM;

typedef enum {
	SetVidMode,
	SetTestMode,
	LoadVidCode,
	SetMonitorType,
	SetPanScanType,
	SetFreezeMode
}               ENC;

typedef enum {
	IncCrashCounter,
	dummy
}               SYSTEM;

typedef enum {
	__Record,
	__Stop,
	__Play,
	__Pause,
	__Slow,
	__FF_IP,
	__Scan_I
}               REC_PLAY;

typedef enum {
	COMTYPE_NOCOM,		/* 0 */
	COMTYPE_TUNER,
	COMTYPE_DEMODULATOR,	/* 2 */
	COMTYPE_DESCRAMBLE,
	COMTYPE_PIDFILTER,	/* 4 */
	COMTYPE_MPEGDECODER,
	COMTYPE_OSD,		/* 6 */
	COMTYPE_BMP,
	COMTYPE_ENCODER,	/* 8 */
	COMTYPE_CONFACCESS,
	COMTYPE_I2C,		/* a */
	COMTYPE_AUDIODAC,
	COMTYPE_TELETEXT,	/* c */
	COMTYPE_REQUEST,
	COMTYPE_SYSTEM,		/* e */
	COMTYPE_REC_PLAY,
	COMTYPE_COMMON_IF,	/* 10 */
	COMTYPE_PID_FILTER,
}               COMTYPE;
#endif

/*
 *                       Public DVB-related structures
 */

/* tuner */
/* IOCTLS */
#define TUNER_SET_TYPE		_IOW('t',1,int)	/* set tuner type */
#define TUNER_SET_TVFREQ	_IOW('t',2,int)	/* set tv freq */

/* tuner types */
#define TUNER_SP5659 0		/* tuner in SkyStar1 */
#define TUNER_DVBC   1		/* Cable tuner (TODO: unsupported) */

/* internal types */
#define NOTUNER		0xFF
#define DVBC		6
#define DVBS		0

struct skystar_softc;		/* forward declaration */

/*
 *                           tuner control
 */
int	tuner_command(struct skystar_softc *,unsigned int, void *);
void	set_tv_freq(struct skystar_softc *, u_int32_t);	/* TODO: remove declaration */

#define MAXFILT 32	/* limit of filters */

/*
 *                   internal driver data  (TODO: move to skystar_internal.h)
 */

struct arm_av711x {
    u_int32_t                 arm_fw;
    u_int32_t                 arm_rtsl;
    u_int32_t                 arm_vid;
    u_int32_t                 arm_app;
    u_int32_t                 avtype;
};

/* skystar_softc.flags */
#define SKYSTAR_INITIALIZED	0x00000001
#define SKYSTAR_OPENED		0x00000002	/* opened as video device */

struct skystar_softc {
	struct device		ss_dev;	/* base device			 */
	bus_dma_tag_t		ss_dmat;/* DMA tag				 */
	bus_space_tag_t		memt;	/* card memory tag			 */
	bus_space_handle_t	memh;/* card memory handle		 */
	bus_size_t		obmemsz;/* card memory size (bytes)		 */
	void			*ih;	/* interrupt handler			 */
	struct arpcom		arpcom;	/* ethernet data structure		 */
	u_int32_t		flags;	/* miscellaneous device flags	 */
	void			*i2c;
	void			*debi;
	bus_dmamap_t		debi_dmap;	/* debi DMA map			 */

	/* DVB related stuff begins here */
	struct dvb_filter	filter[MAXFILT];
	struct dvb_filter	*hfilter[MAXFILT];

	int		debitransfer;	/* type of the DEBI transfer (IP, MPEG, etc.) */
	int		debilen;/* size of the DEBI data block */

	struct frontend		front;	/* frontend data */
	int			(*dvb_cfun)(struct skystar_softc *, unsigned int, void *); /* control function */
	union DVB		dvb;

	struct arm_av711x	arm;	/* ARM CPU info */
	/* tuner-specific parameters */
	int		tuner_type;
	int		tuner_freq;
	u_char		tuner_addr;	/* tuner i2c adddress */
	u_char		tuner_flags;	/* tuner i2c flags */
};

/* some memory-sizes */
#define GRABBING_MEM_SIZE	0x240000	/* 1024 * 576 * 4 */
#define CLIPPING_MEM_SIZE	20000	/* 1024 * 625 / 32 */
#define I2C_MEM_SIZE		0x000800	/* 2048 */
#define	RPS_MEM_SIZE		0x000800	/* 2048 */

/* external grabbing states */
#define GBUFFER_UNUSED         0x000
#define GBUFFER_GRABBING       0x001
#define GBUFFER_DONE           0x002

/* macroses for accessing memory-mapped chip registers */
#define read_long(skystar,offset)		bus_space_read_4((skystar)->memt,(skystar)->memh,(offset))
#define write_long(skystar,offset,value)	bus_space_write_4((skystar)->memt,(skystar)->memh,(offset),(value))

#endif	/* __h_skystar_dev */
