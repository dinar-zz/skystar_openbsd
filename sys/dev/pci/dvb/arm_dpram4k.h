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

/*
 * DPRAM register address
 */

#define	DPRAM_BASE	0x4000

/* Boot data	 */
#define BOOT_STATE	(DPRAM_BASE + 0x7F8)
#define BOOT_SIZE	(DPRAM_BASE + 0x7FA)
#define BOOT_BASE	(DPRAM_BASE + 0x7FC)
#define BOOT_BLOCK	(DPRAM_BASE + 0x800)
#define BOOT_MAX_SIZE	0x800

#define IRQ_STATE		(DPRAM_BASE + 0x0F4)
#define IRQ_STATE_EXT		(DPRAM_BASE + 0x0F6)
#define MSGSTATE		(DPRAM_BASE + 0x0F8)
#define FILT_STATE		(DPRAM_BASE + 0x0FA)
#define COMMAND			(DPRAM_BASE + 0x0FC)
#define COM_BUFF		(DPRAM_BASE + 0x100)
#define COM_BUFF_SIZE		0x20

#define BUFF1_BASE		(DPRAM_BASE + 0x120)
#define BUFF1_SIZE		0xE0

#define DATA_BUFF_BASE		(DPRAM_BASE + 0x200)
#define DATA_BUFF_SIZE		0x1C00

/* new buffers */

#define DATA_BUFF0_BASE		(DPRAM_BASE + 0x200)
#define DATA_BUFF0_SIZE		0x0800

#define DATA_BUFF1_BASE		(DATA_BUFF0_BASE+DATA_BUFF0_SIZE)
#define DATA_BUFF1_SIZE		0x0800

#define DATA_BUFF2_BASE		(DATA_BUFF1_BASE+DATA_BUFF1_SIZE)
#define DATA_BUFF2_SIZE		0x0800

#define Reserved		(DPRAM_BASE + 0x1E00)
#define Reserved_SIZE		0x1C0

#define DEBUG_WINDOW		(DPRAM_BASE + 0x1FC0)
#define	DBG_LOOP_CNT		(DEBUG_WINDOW + 0x00)
#define DBG_SEC_CNT		(DEBUG_WINDOW + 0x02)
#define DBG_AVRP_BUFF		(DEBUG_WINDOW + 0x04)
#define DBG_AVRP_PEAK		(DEBUG_WINDOW + 0x06)
#define DBG_MSG_CNT		(DEBUG_WINDOW + 0x08)
#define DBG_CODE_REG		(DEBUG_WINDOW + 0x0a)
#define DBG_TTX_Q		(DEBUG_WINDOW + 0x0c)
#define DBG_AUD_EN		(DEBUG_WINDOW + 0x0e)
#define DBG_WRONG_COM		(DEBUG_WINDOW + 0x10)
#define DBG_ARR_OVFL		(DEBUG_WINDOW + 0x12)
#define DBG_BUFF_OVFL		(DEBUG_WINDOW + 0x14)
#define DBG_OVFL_CNT		(DEBUG_WINDOW + 0x16)
#define DBG_SEC_OVFL		(DEBUG_WINDOW + 0x18)

#define STATUS_BASE		(DPRAM_BASE + 0x1FC0)
#define STATUS_SCR      	(STATUS_BASE + 0x00)
#define STATUS_MODES    	(STATUS_BASE + 0x04)
#define STATUS_LOOPS    	(STATUS_BASE + 0x08)

#define RX_TYPE         	(DPRAM_BASE + 0x1FE8)
#define RX_LEN          	(DPRAM_BASE + 0x1FEA)
#define TX_TYPE         	(DPRAM_BASE + 0x1FEC)
#define TX_LEN          	(DPRAM_BASE + 0x1FEE)

#define RX_BUFF         	(DPRAM_BASE + 0x1FF4)
#define TX_BUFF 		(DPRAM_BASE + 0x1FF6)

#define HANDSHAKE_REG		(DPRAM_BASE + 0x1FF8)
#define COM_IF_LOCK		(DPRAM_BASE + 0x1FFA)

#define IRQ_RX			(DPRAM_BASE + 0x1FFC)
#define IRQ_TX			(DPRAM_BASE + 0x1FFE)

#define DRAM_START_CODE		0x2e000404
#define DRAM_MAX_CODE_SIZE	0x00100000

#define RESET_LINE		2
#define DEBI_DONE_LINE		1
#define ARM_IRQ_LINE		0

#define DAC_CS			0x8000
#define DAC_CDS			0x0000
