
#define	GPMQFull	0x0001	/* Main Message Queue Full	 */
#define	GPMQOver	0x0002	/* Main Message Queue Overflow	 */
#define	HPQFull		0x0004	/* High Priority Msg Queue Full	 */
#define	HPQOver		0x0008
#define	OSDQFull	0x0010	/* OSD Queue Full	 */
#define	OSDQOver	0x0020

#define	SECTION_EIT	        0x01
#define	SECTION_SINGLE	        0x00
#define	SECTION_CYCLE		0x02
#define	SECTION_CONTINUOS	0x04
#define	SECTION_MODE		0x06
#define SECTION_IPMPE		0x0C
#define SECTION_HIGH_SPEED	0x1C	
#define DATA_PIPING_FLAG	0x20	/* for Data Piping Filter	 */

#define	PBUFSIZE_NONE	0x0000
#define	PBUFSIZE_1P	0x0100
#define	PBUFSIZE_2P	0x0200
#define	PBUFSIZE_1K	0x0300
#define	PBUFSIZE_2K	0x0400
#define	PBUFSIZE_4K	0x0500
#define	PBUFSIZE_8K	0x0600
#define PBUFSIZE_16K	0x0700
#define PBUFSIZE_32K	0x0800

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
	ReleaseBmp,
	SetWTrans,
	SetWNoTrans
} OSDCOM;

typedef enum {
	VideoPID,		/* 00 */
	AudioPID,
	WaitVideoPlay,
	WaitUnMute,
	InitFilt,		/* 04 */
	AddFilt8,
	AddFilt16,
	DeleteFilt,
	FiltError,		/* 08 */
	RecSection,
	NewVersion,
	CacheError,
	SetFiltBytes,		/* 0C */
	SetRange,
	AddSecFilt,
	AddRangeFilt,
	AddFilt12,		/* 10 */
	MultiPID,		/* 11 */
	AddPIDFilter,		/* 12 */
	DelPIDFilter,
	TestSec,
	Scan,
	SetDescr,
} PIDCOM;

/* typedef enum {	 */
/* SelAudChannels	 */
/* } MPEGCOM;	 */

typedef enum {
	AudioDAC,		/* 00 */
	CabADAC,
	ON22K,			/* 02 */
	OFF22K,
	MainSwitch,		/* 04 */
	ADSwitch,
	SendDiSEqC,		/* 06 */
	OFDM_Channel,
	OFDM_Guard,		/* 08 */
	OFDM_MpegBer,
	SetRegister,
	SetIrControl,
	LoadTTuner
} AUDCOM;

typedef enum {
	AudioState,
	AudioBuffState,
	VideoState1,
	VideoState2,
	VideoState3,
	CrashCounter,
	ReqVersion,
	ReqVCXO,
	ReqRegister
} REQCOM;

typedef enum {
	SetVidMode,
	SetTestMode,
	LoadVidCode,
	SetMonitorType,
	SetPanScanType,
	SetFreezeMode
} ENC;

typedef enum {
	__Record,
	__Stop,
	__Play,
	__Pause,
	__Slow,
	__FF_IP,
	__Scan_I,
	__Continue,
} REC_PLAY;

typedef enum {
	COMTYPE_NOCOM,		/* 00 */
	COMTYPE_TUNER,		/* 01 */
	COMTYPE_DEMODULATOR,	/* 02 */
	COMTYPE_DESCRAMBLE,	/* 03 */
	COMTYPE_PIDFILTER,	/* 04 */
	COMTYPE_MPEGDECODER,	/* 05 */
	COMTYPE_OSD,		/* 06 */
	COMTYPE_BMP,		/* 07 */
	COMTYPE_ENCODER,	/* 08 */
	COMTYPE_CONFACCESS,
	COMTYPE_I2C,
	COMTYPE_AUDIODAC,
	COMTYPE_TELETEXT,	/* 0C */
	COMTYPE_REQUEST,	/* 0D */
	COMTYPE_SYSTEM,		/* 0E */
	COMTYPE_REC_PLAY,	/* 0F */
	COMTYPE_COMMON_IF,	/* 10 */
	COMTYPE_PID_FILTER,	/* 11 */
	COMTYPE_PES,
	COMTYPE_TS,
	COMTYPE_VIDEO,
	COMTYPE_AUDIO,
} COMTYPE;

/* typedef enum {	 */
/* DVB_VIDEO_FREEZE,	 */
/* DVB_VIDEO_CONTINUE,	 */
/* DVB_	 */
/* } VIDEOCOM;	 */

/* typedef enum {	 */
/* DVB_AUDIO_PAUSE,	 */
/* } AUDIOCOM;	 */
