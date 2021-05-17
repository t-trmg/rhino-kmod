/*
 * Rhino Equipment Corp.  Rhino R1T1 Card Driver
 *
 * Release version 06/29/15
 *
 * Written by
 *          Lee Reeves <helpdesk@rhinoequipment.com>
 *          Bob Conklin <helpdesk@rhinoequipment.com>
 *          Bryce Chidester <helpdesk@rhinoequipment.com>
 *          Matthew Gessner <helpdesk@rhinoequipment.com>
 *
 * Based on Digium's wct1xxp module
 * and Zapata Telephony's Zaptel Telephony Interface
 *
 * Copyright (C) 2011-2015, QTurbo, LLC
 * Copyright (C) 2005-2010, Rhino Equipment Corp.
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include "r1t1.h"
#include "GpakCust.h"
#include "GpakApi.h"

static int chanmap_t1[] = { 1, 2, 3,
	5, 6, 7,
	9, 10, 11,
	13, 14, 15,
	17, 18, 19,
	21, 22, 23,
	25, 26, 27,
	29, 30, 31
};

static int chanmap_e1[] = { 1, 2, 3,
	4, 5, 6, 7,
	8, 9, 10, 11,
	12, 13, 14, 15,
	16, 17, 18, 19,
	20, 21, 22, 23,
	24, 25, 26, 27,
	28, 29, 30, 31
};

#define USE_G168_DSP

#define CANARY 0xca1e
#define addr_t (__u32)(dma_addr_t)
#define DEBUG_MAIN      (1 << 0)
#define DEBUG_DSP       (1 << 7)

static int debug = 0;			/* Start out with no debugging enabled */
static int e1 = 0;				/* Defines whether or not the card is set to e1 mode */
static int no_ec = 0;
static int ec_disable = 0;		/* Mask defining where the ec should be disabled */
static int ec_sw = 0xffffffff;	/* Mask defining where the ec should be enabled */
static int nlp_type = 3;

static struct r1t1_card *cards[RH_MAX_CARDS];

static int r1t1_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
							   struct dahdi_echocanparam *p,
							   struct dahdi_echocan_state **ec);
static void r1t1_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);

static const struct dahdi_echocan_features my_ec_features = {
	.NLP_automatic = 1,
	.CED_tx_detect = 1,
	.CED_rx_detect = 1,
};

static const struct dahdi_echocan_ops my_ec_ops = {
#if DAHDI_VER < KERNEL_VERSION(2,5,0)
	.name = "R1T1_HWEC",
#endif
	.echocan_free = r1t1_echocan_free,
};

#ifndef __GENERIC_IO_H

unsigned int fastcall ioread8(void __iomem *);
{
	return *(volatile __u8 *) (__iomem);
}

unsigned int fastcall ioread16(void __iomem *);
{
	return *(volatile __u16 *) (__iomem);
}

unsigned int fastcall ioread32(void __iomem *);
{
	return *(volatile __u32 *) (__iomem);
}

void fastcall iowrite8(u8, void __iomem *);
{
	*(volatile __u8 *) (__iomem) = u8;
	return 0;
}

void fastcall iowrite16(u16, void __iomem *);
{
	*(volatile __u16 *) (__iomem) = u16;
	return 0;
}

void fastcall iowrite32(u32, void __iomem *);
{
	*(volatile __u32 *) (__iomem) = u32;
	return 0;
}

#endif

static int r1t1_open(struct dahdi_chan *chan)
{
	struct dahdi_span *span = chan->span;
	struct r1t1_card *r1t1_card = container_of(span, struct r1t1_card, span);

	if (r1t1_card->dead)
		return -ENODEV;
	r1t1_card->usecount++;
	try_module_get(THIS_MODULE);

	if (debug)
		printk(KERN_DEBUG "R1T1: use count %d\n", r1t1_card->usecount);
	return 0;
}

static int __r1t1_get_reg(struct r1t1_card *r1t1_card, int reg)
{
	unsigned char res;
	res = ioread8(r1t1_card->ioaddr + (reg << 2));
	return res;
}

static int __r1t1_set_reg(struct r1t1_card *r1t1_card, int reg, unsigned char val)
{
	iowrite8(val, r1t1_card->ioaddr + (reg << 2));
	/* an extra read to prevent back-to-back burst writes */
	ioread8(r1t1_card->ioaddr + (reg << 2));
	return 0;
}


static void __r1t1_stop_framer(struct r1t1_card *r1t1_card)
{
	__r1t1_set_reg(r1t1_card, R1T1_CONTROL / 4, 0x00);
}


static int r1t1_framer_hard_reset(struct r1t1_card *r1t1_card)
{
	int i;
	unsigned long flags;
	unsigned long endjiffies;

	spin_lock_irqsave(&r1t1_card->lock, flags);
	if (debug)
		printk(KERN_DEBUG "R1T1: ise1=%i\n", r1t1_card->ise1);

	/* Soft reset */
	__r1t1_set_reg(r1t1_card, DS2155_MSTRREG, 0x01);	/* Sets *ALL* regs to default values */
	__r1t1_set_reg(r1t1_card, DS2155_IOCR2, 0x03);	/* 2.048 Mhz Pll clock for all */
	/* 2 TCLK from TCLK pin if alive, or RCLK  6 TCLK from RCLK  4 TCLK from MCLK (Master) */
	__r1t1_set_reg(r1t1_card, DS2155_CCR1, 0x06);	/* 0 TCLK pin = RCLK from Rhino Chip */
	__r1t1_set_reg(r1t1_card, DS2155_CCR2, 0x03);	/* Enable BPCLK 8.019 Mhz */
	__r1t1_set_reg(r1t1_card, DS2155_IBOC, 0x28);	/* Set as #1 on 4 wide bus */


	if (r1t1_card->ise1) {
		__r1t1_set_reg(r1t1_card, DS2155_MSTRREG, 0x02);	/* Sets E1 Mode */
		__r1t1_set_reg(r1t1_card, DS2155_E1TCR1, 0x10);	/* International Si */
		__r1t1_set_reg(r1t1_card, DS2155_SIGCR, 0x80);	/* Sig reinsertion en */
		__r1t1_set_reg(r1t1_card, DS2155_LIC1, 0x21);	/* TPD turn off Power Down default 120 LBO */
		__r1t1_set_reg(r1t1_card, DS2155_LIC4, 0x0f);	/* 120 transmit term */
		__r1t1_set_reg(r1t1_card, DS2155_LIC2, 0xd8);	/* LIRST Line Intf Reset (takes 40ms) */
		__r1t1_set_reg(r1t1_card, DS2155_LIC2, 0x98);	/* Sets E1 mode JAMUX, Stops TA1 */
		__r1t1_set_reg(r1t1_card, DS2155_TAF, 0x1b);	/* Tx Align Frame Sa and Si Pattern */
		__r1t1_set_reg(r1t1_card, DS2155_TNAF, 0x5f);	/* Tx Non Align Frame Sa and Si Pattern */
		__r1t1_set_reg(r1t1_card, DS2155_TSR1, 0x50);	/* CH-0 clear */
		for (i = 0x51; i <= 0x5f; i++)
			__r1t1_set_reg(r1t1_card, i, 0x55);	/* TSR2 - TSR16 = 55 */
		for (i = 0x8c; i <= 0x8f; i++)
			__r1t1_set_reg(r1t1_card, i, 0xff);	/* TCBR1 - TCBR4 = ff */
	} else {
		__r1t1_set_reg(r1t1_card, DS2155_T1RCR1, 0x0C);	/* Sync Time & Criteria */
		__r1t1_set_reg(r1t1_card, DS2155_T1TCR1, 0x10);	/* SW Sig Insertion */
		__r1t1_set_reg(r1t1_card, DS2155_IBCC, 0x22);	/* Enable Loop Up and Down */
		__r1t1_set_reg(r1t1_card, DS2155_RUPCD1, 0x80);	/* Set the Code Values */
		__r1t1_set_reg(r1t1_card, DS2155_RUPCD2, 0x00);
		__r1t1_set_reg(r1t1_card, DS2155_RDNCD1, 0x80);
		__r1t1_set_reg(r1t1_card, DS2155_RDNCD2, 0x00);
		__r1t1_set_reg(r1t1_card, DS2155_LIC1, 0x01);	/* TPD(0) turn off Power Down default LBO */
		__r1t1_set_reg(r1t1_card, DS2155_LIC4, 0x05);	/* 75 transmit term */
		__r1t1_set_reg(r1t1_card, DS2155_LIC2, 0x58);	/* LIRST(6) Line Intf Reset (takes 40ms) */
		__r1t1_set_reg(r1t1_card, DS2155_LIC2, 0x18);	/* Sets T1 mode JAMUX, Stops TA1 */
	}
	/* Wait 100ms to give plenty of time for reset */
	endjiffies = jiffies + 10;
	while (endjiffies < jiffies);

	__r1t1_set_reg(r1t1_card, DS2155_ESCR, 0x55);	/* Re-align elastic stores */
	__r1t1_set_reg(r1t1_card, DS2155_ESCR, 0x11);	/* TX & RX elastic (TSYSCLK IN) */

	spin_unlock_irqrestore(&r1t1_card->lock, flags);
	return 0;
}

static int r1t1_shutdown(struct dahdi_span *span)
{
	if (span) {
		struct r1t1_card *r1t1_card = container_of(span, struct r1t1_card, span);
		span->flags &= ~DAHDI_FLAG_RUNNING;

		if (r1t1_card) {
			__r1t1_stop_framer(r1t1_card);
			r1t1_framer_hard_reset(r1t1_card);
		}
	}

	return 0;
}


static void r1t1_release(struct r1t1_card *r1t1_card)
{

	r1t1_shutdown(&r1t1_card->span);
#if DAHDI_VER >= KERNEL_VERSION(2,6,0)
	dahdi_unregister_device(r1t1_card->ddev);
#else
	dahdi_unregister(&r1t1_card->span);
#endif

	/* Free resources */
	free_irq(r1t1_card->dev->irq, r1t1_card);
	pci_free_consistent(r1t1_card->dev, DAHDI_MAX_CHUNKSIZE * 2 * 2 * 32 + 8,
						(void *) r1t1_card->writechunk, r1t1_card->writedma);
	iounmap(r1t1_card->ioaddr);
	release_mem_region(r1t1_card->pciaddr, R1T1_SIZE);
	/* recall that rc->chans[0] == chan_block from r1t1_init_one,
	 * and that rc->ec[0] == ec_block from r1t1_init_one. */
	kfree(r1t1_card->chans[0]);
	kfree(r1t1_card->ec[0]);
#if DAHDI_VER >= KERNEL_VERSION(2,6,0)
	dahdi_free_device(r1t1_card->ddev);
#endif
	kfree(r1t1_card);
	printk(KERN_INFO "Released a Rhino r1t1\n");
}

static int r1t1_close(struct dahdi_chan *chan)
{
	struct dahdi_span *span = chan->span;
	struct r1t1_card *r1t1_card = container_of(span, struct r1t1_card, span);
	r1t1_card->usecount--;
	module_put(THIS_MODULE);
	if (debug)
		printk(KERN_DEBUG "R1T1: use count %d\n", r1t1_card->usecount);
	/* If we're dead, release us now */
	if (!r1t1_card->usecount && r1t1_card->dead)
		r1t1_release(r1t1_card);
	return 0;
}

static void r1t1_enable_interrupts(struct r1t1_card *r1t1_card)
{
}

static void r1t1_start_dma(struct r1t1_card *r1t1_card)
{
	/* Reset Master and TDM */
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(1);
	__r1t1_set_reg(r1t1_card, R1T1_CONTROL / 4, 0x01);
	if (debug)
		printk(KERN_DEBUG "R1T1: Started DMA\n");
}

static void __r1t1_set_clear(struct r1t1_card *r1t1_card)
{
	/* Setup registers */
	int x, z;
	unsigned char b;

	/* No such thing under E1 */
	if (r1t1_card->ise1) {
		printk(KERN_WARNING "R1T1: Can't set clear mode on an E1!\n");
		return;
	}

	for (x = 0; x < 3; x++) {
		b = 0;
		for (z = 0; z < 8; z++) {
			/* Enable software signaling unless the channel is "clear" */
			if (!(r1t1_card->span.chans[x * 8 + z]->flags & DAHDI_FLAG_CLEAR))
				b |= (1 << z);
		}
		__r1t1_set_reg(r1t1_card, DS2155_SSIE + x, b);
	}
}

static void r1t1_t1_framer_start(struct r1t1_card *r1t1_card)
{
	char *coding, *framing;
	int alreadyrunning = r1t1_card->span.flags & DAHDI_FLAG_RUNNING;
	unsigned long flags;

	spin_lock_irqsave(&r1t1_card->lock, flags);

	if (r1t1_card->span.lineconfig & DAHDI_CONFIG_ESF) {
		/* CCR1(2) TFM = 1 */
		__r1t1_set_reg(r1t1_card, DS2155_T1CCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_T1CCR1) & 0xfb) | 0x04);
		/* RCR2(6) RFM = 1 */
		__r1t1_set_reg(r1t1_card, DS2155_T1RCR2,
					   (__r1t1_get_reg(r1t1_card, DS2155_T1RCR2) & 0xbf) | 0x40);
		/* Fs bit insertion TCR2 TSLC96 = 0 */
		__r1t1_set_reg(r1t1_card, DS2155_T1TCR2,
					   (__r1t1_get_reg(r1t1_card, DS2155_T1TCR2) & 0xbf) & ~0x40);
		__r1t1_set_reg(r1t1_card, DS2155_TFDL, 0x00);	/* FDL data */
		framing = "ESF";
	} else {
		/* CCR1(2) TFM = 0 */
		__r1t1_set_reg(r1t1_card, DS2155_T1CCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_T1CCR1) & 0xfb) & ~0x04);
		/* RCR2(6) RFM = 0 */
		__r1t1_set_reg(r1t1_card, DS2155_T1RCR2,
					   (__r1t1_get_reg(r1t1_card, DS2155_T1RCR2) & 0xbf) & ~0x40);
		/* Fs bit insertion TCR2(6) TSLC96 = 1 and TCR1(2) TFDLS = 0 (*default*) */
		__r1t1_set_reg(r1t1_card, DS2155_T1TCR2,
					   (__r1t1_get_reg(r1t1_card, DS2155_T1TCR2) & 0xbf) | 0x40);
		__r1t1_set_reg(r1t1_card, DS2155_TFDL, 0x1c);	/* FDL data */
		framing = "D4";
	}
	if (r1t1_card->span.lineconfig & DAHDI_CONFIG_B8ZS) {
		/* TCR2(7) TB8ZS = 1 */
		__r1t1_set_reg(r1t1_card, DS2155_T1TCR2,
					   (__r1t1_get_reg(r1t1_card, DS2155_T1TCR2) & 0x7f) | 0x80);
		/* RCR2(5) RB8ZS = 1 */
		__r1t1_set_reg(r1t1_card, DS2155_T1RCR2,
					   (__r1t1_get_reg(r1t1_card, DS2155_T1RCR2) & 0xdf) | 0x20);
		coding = "B8ZS";
	} else {
		/* TCR2(7) TB8ZS = 0 */
		__r1t1_set_reg(r1t1_card, DS2155_T1TCR2,
					   (__r1t1_get_reg(r1t1_card, DS2155_T1TCR2) & 0x7f) & ~0x80);
		/* RCR2(5) RB8ZS = 0 */
		__r1t1_set_reg(r1t1_card, DS2155_T1RCR2,
					   (__r1t1_get_reg(r1t1_card, DS2155_T1RCR2) & 0xdf) & ~0x20);
		coding = "AMI";

	}

	/* Force re-sync */
	__r1t1_set_reg(r1t1_card, DS2155_T1RCR1, (__r1t1_get_reg(r1t1_card, DS2155_T1RCR1) & 0xfe) | 0x01);
	__r1t1_set_reg(r1t1_card, DS2155_T1RCR1, (__r1t1_get_reg(r1t1_card, DS2155_T1RCR1) & 0xfe) & ~0x01);

	/* Set outgoing LBO */
	__r1t1_set_reg(r1t1_card, DS2155_LIC1,
				   (__r1t1_get_reg(r1t1_card, DS2155_LIC1) & 0x1f) | (r1t1_card->span.txlevel << 5));
	__r1t1_set_clear(r1t1_card);

	printk(KERN_INFO "R1T1: Using %s/%s coding/framing\n", coding, framing);
	if (!alreadyrunning) {
		r1t1_card->span.flags |= DAHDI_FLAG_RUNNING;
	}
	spin_unlock_irqrestore(&r1t1_card->lock, flags);
}

static void r1t1_e1_framer_start(struct r1t1_card *r1t1_card)
{
	char *coding, *framing;
	int alreadyrunning = r1t1_card->span.flags & DAHDI_FLAG_RUNNING;
	unsigned long flags;
	char *crcing = "";

	spin_lock_irqsave(&r1t1_card->lock, flags);
	if (debug)
		printk(KERN_DEBUG "R1T1: E1 framer start configuration\n");

	if (r1t1_card->span.lineconfig & DAHDI_CONFIG_CCS) {
		/*      SIGCR GRSRE(7) = 0 CCS */
		__r1t1_set_reg(r1t1_card, DS2155_SIGCR,
					   (__r1t1_get_reg(r1t1_card, DS2155_SIGCR) & 0x7f) & ~0x80);
		/*      SIGCR TCCS(1) RCCS(2) = 1 CCS */
		__r1t1_set_reg(r1t1_card, DS2155_SIGCR,
					   (__r1t1_get_reg(r1t1_card, DS2155_SIGCR) & 0xfb) | 0x04);
		/*      E1RCR1 RSIGM(6) = 1 CCS */
		__r1t1_set_reg(r1t1_card, DS2155_E1RCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1RCR1) & 0xbf) | 0x40);
		/*      E1TCR1 T16S(6) = 0 TS16 from SSIE and TSHCS */
		__r1t1_set_reg(r1t1_card, DS2155_E1TCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1TCR1) & 0xbf) & ~0x40);
		framing = "CCS";		/* Receive CCS */
	} else {
		/*      SIGCR GRSRE(7) = 1 CAS */
		__r1t1_set_reg(r1t1_card, DS2155_SIGCR,
					   (__r1t1_get_reg(r1t1_card, DS2155_SIGCR) & 0x7f) | 0x80);
		/*      SIGCR TCCS(1) RCCS(2) = 0 CAS */
		__r1t1_set_reg(r1t1_card, DS2155_SIGCR,
					   (__r1t1_get_reg(r1t1_card, DS2155_SIGCR) & 0xfb) & ~0x04);
		/*      E1RCR1 RSIGM(6) = 0 CAS */
		__r1t1_set_reg(r1t1_card, DS2155_E1RCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1RCR1) & 0xbf) & ~0x40);
		/*      E1TCR1 T16S(6) = 1 TS16 from TS1-TS16 */
		__r1t1_set_reg(r1t1_card, DS2155_E1TCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1TCR1) & 0xbf) | 0x40);
		framing = "CAS";
	}
	if (r1t1_card->span.lineconfig & DAHDI_CONFIG_HDB3) {
		/*      E1RCR1 RHDB3(5) = 1 HDB3 */
		__r1t1_set_reg(r1t1_card, DS2155_E1RCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1RCR1) & 0xdf) | 0x20);
		/*      E1TCR1 THDB3(2) = 1 HDB3 */
		__r1t1_set_reg(r1t1_card, DS2155_E1TCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1TCR1) & 0xfb) | 0x04);
		coding = "HDB3";
	} else {
		/*      E1RCR1 RHDB3(5) = 0 HDB3 */
		__r1t1_set_reg(r1t1_card, DS2155_E1RCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1RCR1) & 0xdf) & ~0x20);
		/*      E1TCR1 THDB3(2) = 0 HDB3 */
		__r1t1_set_reg(r1t1_card, DS2155_E1TCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1TCR1) & 0xfb) & ~0x04);
		coding = "AMI";
	}
	if (r1t1_card->span.lineconfig & DAHDI_CONFIG_CRC4) {
		/*       E1RCR1 RCRC4(3) = 1 */
		__r1t1_set_reg(r1t1_card, DS2155_E1RCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1RCR1) & 0xf7) | 0x08);
		/*       E1TCR1 TCRC4(0) = 1 */
		__r1t1_set_reg(r1t1_card, DS2155_E1TCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1TCR1) & 0xfe) | 0x01);
		/*       E1TCR2 AEBE(2) = 1 */
		__r1t1_set_reg(r1t1_card, DS2155_E1TCR2,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1TCR2) & 0xfb) | 0x04);
		crcing = " with CRC4";
	} else {
		/*       E1RCR1 RCRC4(3) = 0 */
		__r1t1_set_reg(r1t1_card, DS2155_E1RCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1RCR1) & 0xf7) & ~0x08);
		/*       E1TCR1 TCRC4(0) = 0 */
		__r1t1_set_reg(r1t1_card, DS2155_E1TCR1,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1TCR1) & 0xfe) & ~0x01);
		/*       E1TCR2 AEBE(2) = 0 */
		__r1t1_set_reg(r1t1_card, DS2155_E1TCR2,
					   (__r1t1_get_reg(r1t1_card, DS2155_E1TCR2) & 0xfb) & ~0x04);
		crcing = "";
	}

	/* Set outgoing LBO */
	__r1t1_set_reg(r1t1_card, DS2155_LIC1,
				   (__r1t1_get_reg(r1t1_card, DS2155_LIC1) & 0x1f) | (r1t1_card->span.txlevel << 5));

	/* Force re-sync  E1RCR1 RESYNC(0) = 1 - 0 */
	__r1t1_set_reg(r1t1_card, DS2155_E1RCR1, (__r1t1_get_reg(r1t1_card, DS2155_E1RCR1) & 0xfe) | 0x01);
	__r1t1_set_reg(r1t1_card, DS2155_E1RCR1, (__r1t1_get_reg(r1t1_card, DS2155_E1RCR1) & 0xfe) & ~0x01);


	printk(KERN_NOTICE "R1T1: Using %s/%s coding/signaling%s 120 Ohms\n", coding, framing, crcing);
	if (!alreadyrunning) {
		r1t1_card->span.flags |= DAHDI_FLAG_RUNNING;
	}
	spin_unlock_irqrestore(&r1t1_card->lock, flags);
}

static int r1t1_rbsbits(struct dahdi_chan *chan, int bits)
{
	struct dahdi_span *span = chan->span;
	struct r1t1_card *r1t1_card = container_of(span, struct r1t1_card, span);
	unsigned long flags;
	int b, o;
	unsigned char mask;

	/* Byte offset */
	spin_lock_irqsave(&r1t1_card->lock, flags);

	if (r1t1_card->ise1) {
		if (chan->chanpos % 2) {
			mask = (bits | (r1t1_card->chans[chan->chanpos - 1 + 1]->txsig << 4));
		} else {
			mask = ((bits << 4) | r1t1_card->chans[chan->chanpos - 1 - 1]->txsig);
		}
		__r1t1_set_reg(r1t1_card, 0x51 + (chan->chanpos - 1) / 2, mask);
		r1t1_card->chans[chan->chanpos - 1]->txsig = bits;
		if (debug)
			printk(KERN_DEBUG "R1T1: Register %x, Addr %x, mask %x\n",
				   __r1t1_get_reg(r1t1_card, 0x51 + (chan->chanpos - 1) / 2), mask,
				   (0x51 + (chan->chanpos - 1) / 2));

	} else {
		b = (chan->chanpos - 1) / 2;
		o = (chan->chanpos - 1) % 2;

		mask = o ? 0x80 : 0x08;

		if (bits & DAHDI_ABIT) {
			/* Set A-bit */
			r1t1_card->txsig[b] |= mask;
		} else {
			/* Clear A-bit */
			r1t1_card->txsig[b] &= ~mask;
		}
		if (bits & DAHDI_BBIT) {
			/* Set B-bit */
			r1t1_card->txsig[b] |= (mask >> 1);
		} else {
			r1t1_card->txsig[b] &= ~(mask >> 1);
		}
		if (bits & DAHDI_CBIT) {
			/* Set C-bit */
			r1t1_card->txsig[b] |= (mask >> 2);
		} else {
			r1t1_card->txsig[b] &= ~(mask >> 2);
		}
		if (bits & DAHDI_DBIT) {
			/* Set D-bit */
			r1t1_card->txsig[b] |= (mask >> 3);
		} else {
			r1t1_card->txsig[b] &= ~(mask >> 3);
		}
		/* Output new values */
		__r1t1_set_reg(r1t1_card, 0x50 + b, r1t1_card->txsig[b]);

	}
	spin_unlock_irqrestore(&r1t1_card->lock, flags);
	return 0;
}

static int r1t1_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
	switch (cmd) {
	default:
		return -ENOTTY;
	}
}

#if DAHDI_VER >= KERNEL_VERSION(2,5,0)
static int r1t1_startup(struct file *file, struct dahdi_span *span)
#else
static int r1t1_startup(struct dahdi_span *span)
#endif
{
	struct r1t1_card *r1t1_card = container_of(span, struct r1t1_card, span);

	int i, alreadyrunning = span->flags & DAHDI_FLAG_RUNNING;

	/* initialize the start value for the entire chunk of last ec buffer */
	for (i = 0; i < span->channels; i++) {
		memset(r1t1_card->ec_chunk1[i], DAHDI_LIN2X(0, span->chans[i]), DAHDI_CHUNKSIZE);
		memset(r1t1_card->ec_chunk2[i], DAHDI_LIN2X(0, span->chans[i]), DAHDI_CHUNKSIZE);
	}

	/* Reset framer with proper parameters and start */
	if (r1t1_card->ise1) {
		r1t1_e1_framer_start(r1t1_card);
	} else {
		r1t1_t1_framer_start(r1t1_card);
	}
	printk(KERN_NOTICE "R1T1: Calling startup (flags is %x)\n", (__u32) (span->flags));

	if (!alreadyrunning) {
		/* Only if we're not already going */
		r1t1_enable_interrupts(r1t1_card);
		r1t1_start_dma(r1t1_card);
		span->flags |= DAHDI_FLAG_RUNNING;
	}
	return 0;
}

static int r1t1_maint(struct dahdi_span *span, int cmd)
{
	struct r1t1_card *r1t1_card = container_of(span, struct r1t1_card, span);
	int res = 0;
	unsigned long flags;
	spin_lock_irqsave(&r1t1_card->lock, flags);
	if (r1t1_card->ise1) {
		switch (cmd) {
		case DAHDI_MAINT_NONE:
			__r1t1_set_reg(r1t1_card, DS2155_LBCR, 0);	/* no loop */
			break;
		case DAHDI_MAINT_LOCALLOOP:
			__r1t1_set_reg(r1t1_card, DS2155_LBCR, 0x08);	/* local loop */
			break;
		case DAHDI_MAINT_REMOTELOOP:
			__r1t1_set_reg(r1t1_card, DS2155_LBCR, 0x04);	/* remote loop */
			break;
		case DAHDI_MAINT_LOOPUP:
		case DAHDI_MAINT_LOOPDOWN:
#ifdef DAHDI_MAINT_LOOPSTOP
		case DAHDI_MAINT_LOOPSTOP:
#endif
			res = -ENOSYS;
			break;
		default:
			printk(KERN_ERR "R1T1: Unknown maint command: %d\n", cmd);
			res = -EINVAL;
			break;
		}
	} else {
		switch (cmd) {
		case DAHDI_MAINT_NONE:
			__r1t1_set_reg(r1t1_card, DS2155_LBCR, 0);	/* no loop */
			break;
		case DAHDI_MAINT_LOCALLOOP:
			__r1t1_set_reg(r1t1_card, DS2155_LBCR, 0x08);	/* local loop */
			break;
		case DAHDI_MAINT_REMOTELOOP:
			__r1t1_set_reg(r1t1_card, DS2155_LBCR, 0x04);	/* remote loop */
			break;
		case DAHDI_MAINT_LOOPUP:
			__r1t1_set_reg(r1t1_card, DS2155_T1CCR1, (__r1t1_get_reg(r1t1_card, DS2155_T1CCR1) & 0xfe) | 1);	/* send loopup code */
			__r1t1_set_reg(r1t1_card, DS2155_IBCC, 0x22);	/* send loopup code */
			__r1t1_set_reg(r1t1_card, DS2155_TCD1, 0x80);	/* send loopup code */
			__r1t1_set_reg(r1t1_card, DS2155_TCD2, 0x00);	/* send loopup code */
			break;
		case DAHDI_MAINT_LOOPDOWN:
			__r1t1_set_reg(r1t1_card, DS2155_T1CCR1, (__r1t1_get_reg(r1t1_card, DS2155_T1CCR1) & 0xfe) | 1);	/* send loopdown code */
			__r1t1_set_reg(r1t1_card, DS2155_IBCC, 0x62);	/* send loopdown code */
			__r1t1_set_reg(r1t1_card, DS2155_TCD1, 0x90);	/* send loopdown code */
			__r1t1_set_reg(r1t1_card, DS2155_TCD1, 0x00);	/* send loopdown code */
			break;
#ifdef DAHDI_MAINT_LOOPSTOP
		case DAHDI_MAINT_LOOPSTOP:
			__r1t1_set_reg(r1t1_card, DS2155_T1CCR1, (__r1t1_get_reg(r1t1_card, DS2155_T1CCR1) & 0xfe));	/* stop sending loopup code */
			break;
#endif
		default:
			printk(KERN_ERR "R1T1: Unknown maint command: %d\n", cmd);
			res = -EINVAL;
		}
	}
	spin_unlock_irqrestore(&r1t1_card->lock, flags);
	return res;
}

#if DAHDI_VER >= KERNEL_VERSION(2,5,0)
static int r1t1_chanconfig(struct file *file, struct dahdi_chan *chan, int sigtype)
#else
static int r1t1_chanconfig(struct dahdi_chan *chan, int sigtype)
#endif
{
	if (chan) {
		struct dahdi_span *span = chan->span;
		struct r1t1_card *r1t1_card = container_of(span, struct r1t1_card, span);
		unsigned long flags;
		int alreadyrunning = chan->span->flags & DAHDI_FLAG_RUNNING;

		spin_lock_irqsave(&r1t1_card->lock, flags);

		if (alreadyrunning && !r1t1_card->ise1)
			__r1t1_set_clear(r1t1_card);

		spin_unlock_irqrestore(&r1t1_card->lock, flags);
	}
	return 0;
}

#if DAHDI_VER >= KERNEL_VERSION(2,5,0)
static int r1t1_spanconfig(struct file *file, struct dahdi_span *span, struct dahdi_lineconfig *lc)
#else
static int r1t1_spanconfig(struct dahdi_span *span, struct dahdi_lineconfig *lc)
#endif
{
	if (span) {
		struct r1t1_card *r1t1_card = container_of(span, struct r1t1_card, span);
		span->lineconfig = lc->lineconfig;
		span->txlevel = lc->lbo;
		span->rxlevel = 0;
		/* Do we want to SYNC on receive or not */
		r1t1_card->sync = lc->sync;
		/* If already running, apply changes immediately */
		if (span->flags & DAHDI_FLAG_RUNNING)
#if DAHDI_VER >= KERNEL_VERSION(2,5,0)
			return r1t1_startup(file, span);
#else
			return r1t1_startup(span);
#endif
	}
	return 0;
}

static int r1t1_software_init(struct r1t1_card *r1t1_card)
{
#if DAHDI_VER >= KERNEL_VERSION(2,4,0)
#if DAHDI_VER >= KERNEL_VERSION(2,5,0)
	static const struct dahdi_span_ops ops = {
#else
	static struct dahdi_span_ops ops = {
#endif
		.spanconfig = r1t1_spanconfig,
		.chanconfig = r1t1_chanconfig,
		.startup = r1t1_startup,
		.shutdown = r1t1_shutdown,
		.rbsbits = r1t1_rbsbits,
		.maint = r1t1_maint,
		.open = r1t1_open,
		.close = r1t1_close,
		.ioctl = r1t1_ioctl,
#ifdef USE_G168_DSP
		.echocan_create = r1t1_echocan_create,
#endif
		.owner = THIS_MODULE
	};
#endif

	int x;


#if DAHDI_VER >= KERNEL_VERSION(2,4,0)
	r1t1_card->span.ops = &ops;
#else
	r1t1_card->span.spanconfig = r1t1_spanconfig;
	r1t1_card->span.chanconfig = r1t1_chanconfig;
	r1t1_card->span.startup = r1t1_startup;
	r1t1_card->span.shutdown = r1t1_shutdown;
	r1t1_card->span.rbsbits = r1t1_rbsbits;
	r1t1_card->span.maint = r1t1_maint;
	r1t1_card->span.open = r1t1_open;
	r1t1_card->span.close = r1t1_close;
	r1t1_card->span.ioctl = r1t1_ioctl;
#endif

	sprintf(r1t1_card->span.name, "R1T1/%d", r1t1_card->num);
	snprintf(r1t1_card->span.desc, sizeof(r1t1_card->span.desc) - 1, "%s Card %d", r1t1_card->variety,
			 r1t1_card->num);

#if DAHDI_VER >= KERNEL_VERSION(2,6,0)
	r1t1_card->ddev = dahdi_create_device();
	r1t1_card->ddev->devicetype = r1t1_card->variety;
	r1t1_card->ddev->location = kasprintf(GFP_KERNEL, "PCI Bus %02d Slot %02d",
								  r1t1_card->dev->bus->number,
								  PCI_SLOT(r1t1_card->dev->devfn) + 1);
	r1t1_card->ddev->manufacturer = "Rhino Equipment";
#else
	dahdi_copy_string(r1t1_card->span.devicetype, r1t1_card->variety, sizeof(r1t1_card->span.devicetype));
	snprintf(r1t1_card->span.location, sizeof(r1t1_card->span.location) - 1,
			 "PCI Bus %02d Slot %02d", r1t1_card->dev->bus->number,
			 PCI_SLOT(r1t1_card->dev->devfn) + 1);
	r1t1_card->span.manufacturer = "Rhino Equipment";
#endif

	r1t1_card->span.chans = r1t1_card->chans;
	r1t1_card->span.flags = DAHDI_FLAG_RBS;

	if (r1t1_card->ise1) {
		r1t1_card->span.channels = 31;
		r1t1_card->span.spantype = SPANTYPE_DIGITAL_E1;
		r1t1_card->span.linecompat = DAHDI_CONFIG_AMI | DAHDI_CONFIG_HDB3 |
			DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4;
		r1t1_card->span.deflaw = DAHDI_LAW_ALAW;
	} else {
		r1t1_card->span.channels = 24;
		r1t1_card->span.spantype = SPANTYPE_DIGITAL_T1;
		r1t1_card->span.linecompat = DAHDI_CONFIG_AMI | DAHDI_CONFIG_B8ZS |
			DAHDI_CONFIG_D4 | DAHDI_CONFIG_ESF;
		r1t1_card->span.deflaw = DAHDI_LAW_MULAW;
	}

#if DAHDI_VER < KERNEL_VERSION(2,5,0)
	init_waitqueue_head(&r1t1_card->span.maintq);
#endif
	for (x = 0; x < r1t1_card->span.channels; x++) {


		if (r1t1_card->ise1) {
			r1t1_card->chans[x]->writechunk = (u_char *) (r1t1_card->writechunk +
												   (chanmap_e1[x] * DAHDI_CHUNKSIZE));
			r1t1_card->chans[x]->readchunk = (u_char *) (r1t1_card->readchunk +
												  (chanmap_e1[x] * DAHDI_CHUNKSIZE));
		} else {
			r1t1_card->chans[x]->writechunk = (u_char *) (r1t1_card->writechunk +
												   (chanmap_t1[x] * DAHDI_CHUNKSIZE));
			r1t1_card->chans[x]->readchunk = (u_char *) (r1t1_card->readchunk +
												  (chanmap_t1[x] * DAHDI_CHUNKSIZE));
		}

		sprintf(r1t1_card->chans[x]->name, "R1T1/%d/%d", r1t1_card->num, x + 1);
		r1t1_card->chans[x]->sigcap = DAHDI_SIG_EM | DAHDI_SIG_CLEAR | DAHDI_SIG_EM_E1 |
			DAHDI_SIG_FXSLS | DAHDI_SIG_FXSGS |
			DAHDI_SIG_FXSKS | DAHDI_SIG_FXOLS | DAHDI_SIG_DACS_RBS |
#ifdef DAHDI_SIG_FXONS
			DAHDI_SIG_FXONS |
#endif
			DAHDI_SIG_FXOGS | DAHDI_SIG_FXOKS | DAHDI_SIG_CAS | DAHDI_SIG_SF;
		r1t1_card->chans[x]->chanpos = x + 1;
	}

#if DAHDI_VER >= KERNEL_VERSION(2,6,0)
	list_add_tail(&r1t1_card->span.device_node, &r1t1_card->ddev->spans);
	if (dahdi_register_device(r1t1_card->ddev, &r1t1_card->dev->dev)) {
			printk(KERN_ERR "R1T1: Unable to register span with DAHDI\n");
			return -1;
	}
#else
	if (dahdi_register(&r1t1_card->span, 0)) {
		printk(KERN_ERR "R1T1: Unable to register span with DAHDI\n");
		return -1;
	}
#endif
	return 0;
}

static void r1t1_transmitprep(struct r1t1_card *r1t1_card, int nextbuf)
{
	dahdi_transmit(&r1t1_card->span);
}

static void r1t1_receiveprep(struct r1t1_card *r1t1_card, int nextbuf)
{
	if (!r1t1_card->dsp_up)
		dahdi_ec_span(&r1t1_card->span);
	dahdi_receive(&r1t1_card->span);
}

static void __r1t1_check_sigbits(struct r1t1_card *r1t1_card, int x)
{
	int a, i, y, rxs;

	if (r1t1_card->ise1) {
		for (y = 0; y < 10; y += 2) {
			i = x * 10 + y;
			a = __r1t1_get_reg(r1t1_card, 0x61 + (i / 2));
			rxs = (a & 0xf);
			if (!(r1t1_card->chans[i]->sig & DAHDI_SIG_CLEAR)) {
				if (r1t1_card->chans[i]->rxsig != rxs)
					dahdi_rbsbits(r1t1_card->chans[i], rxs);
			}
			i++;
			rxs = (a >> 4) & 0xf;
			if (!(r1t1_card->chans[i]->sig & DAHDI_SIG_CLEAR)) {
				if (r1t1_card->chans[i]->rxsig != rxs)
					dahdi_rbsbits(r1t1_card->chans[i], rxs);
			}
		}
	} else {
		for (y = 0; y < 8; y += 2) {
			i = x * 8 + y;
			a = __r1t1_get_reg(r1t1_card, 0x60 + (i / 2));
			rxs = a & 0x0f;
			if (!(r1t1_card->chans[i]->sig & DAHDI_SIG_CLEAR)) {
				if (r1t1_card->chans[i]->rxsig != rxs)
					dahdi_rbsbits(r1t1_card->chans[i], rxs);
			}
			i++;
			rxs = (a >> 4) & 0x0f;
			if (!(r1t1_card->chans[i]->sig & DAHDI_SIG_CLEAR)) {
				if (r1t1_card->chans[i]->rxsig != rxs)
					dahdi_rbsbits(r1t1_card->chans[i], rxs);
			}
		}
	}
}

static void __r1t1_check_alarms(struct r1t1_card *r1t1_card)
{
	unsigned char c, led_state;
	int alarms;
	int x, j;

	/* Get RIR2 */
	__r1t1_set_reg(r1t1_card, DS2155_SR2, 0x00);
	c = __r1t1_get_reg(r1t1_card, DS2155_SR2);
	/*                    RLOSC                   FRCLC */
	r1t1_card->span.rxlevel = (((c >> 4) & 1) << 1) || ((c >> 5) & 1);

	/* Get status register s */
	__r1t1_set_reg(r1t1_card, DS2155_SR3, 0x60);
	c = __r1t1_get_reg(r1t1_card, DS2155_SR3);

	/* Assume no alarms */
	led_state = NORM_OP;
	alarms = 0;

	/* And consider only carrier alarms */
	r1t1_card->span.alarms &= (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE | DAHDI_ALARM_NOTOPEN);

#if SUPPORT_E1
	if (r1t1_card->ise1) {
		/* XXX Implement me XXX */
	} else {
#endif
		/* Detect loopup code if we're not sending one */
		if ((!r1t1_card->span.mainttimer) && (c & 0x20)) {
			/* Loop-up code detected */
			led_state = 0xd0;
			if (debug)
				printk(KERN_DEBUG "R1T1: SR3 20 LOOP UP\n");
			if ((r1t1_card->loopupcnt++ > 80) && (r1t1_card->span.maintstat != DAHDI_MAINT_REMOTELOOP)) {
				__r1t1_set_reg(r1t1_card, DS2155_LBCR, 0x04);	/* Remote Loop */
				r1t1_card->span.maintstat = DAHDI_MAINT_REMOTELOOP;
			}
		} else {
			r1t1_card->loopupcnt = 0;
		}
		/* Same for loopdown code */
		if ((!r1t1_card->span.mainttimer) && (c & 0x40)) {
			/* Loop-down code detected */
			if (debug)
				printk(KERN_DEBUG "R1T1: SR3 04 LOOP DOWN\n");
			led_state = NORM_OP;
			if ((r1t1_card->loopdowncnt++ > 80) &&
				(r1t1_card->span.maintstat == DAHDI_MAINT_REMOTELOOP)) {
				__r1t1_set_reg(r1t1_card, DS2155_LBCR, 0x0);	/* No remote Loop */
				r1t1_card->span.maintstat = DAHDI_MAINT_NONE;
			}
		} else
			r1t1_card->loopdowncnt = 0;
#if SUPPORT_E1
	}
#endif

	if (r1t1_card->span.lineconfig & DAHDI_CONFIG_NOTOPEN) {
		for (x = 0, j = 0; x < r1t1_card->span.channels; x++)
			if ((r1t1_card->chans[x]->flags & DAHDI_FLAG_OPEN) ||
#ifdef DAHDI_FLAG_NETDEV
				(r1t1_card->chans[x]->flags & DAHDI_FLAG_NETDEV)
#else
				0
#endif
				)
				j++;
		if (!j)
			alarms |= DAHDI_ALARM_NOTOPEN;
	}

	/* Check actual alarm status */
	__r1t1_set_reg(r1t1_card, DS2155_SR2, 0xff);
	c = __r1t1_get_reg(r1t1_card, DS2155_SR2);
	if (c & 0x4) {
		led_state = YEL_ALM;
		if (debug)
			printk(KERN_DEBUG "R1T1: SR2 04 BLUE ALARM\n");
		alarms |= DAHDI_ALARM_BLUE;
	}
	if (c & 0x2) {
		led_state = NO_CARR;
		if (debug)
			printk(KERN_DEBUG "R1T1: SR2 02 RED NO CARRIER\n");
		alarms |= DAHDI_ALARM_RED;
	}
	if (c & 0x1) {
		led_state = NO_SYNC;
		if (debug)
			printk(KERN_DEBUG "R1T1: SR2 01 RED NO SYNC\n");
		alarms |= DAHDI_ALARM_RED;
	}

	/* Keep track of recovering */
	if ((!alarms) && r1t1_card->span.alarms)
		r1t1_card->alarmtimer = DAHDI_ALARMSETTLE_TIME;

	/* If receiving alarms, go into Yellow alarm state */
	if (alarms && (!r1t1_card->span.alarms)) {
		if (r1t1_card->ise1)
			__r1t1_set_reg(r1t1_card, DS2155_TNAF, 0x7f);
		else
			__r1t1_set_reg(r1t1_card, DS2155_T1TCR1, 0x11);
	}

	if (r1t1_card->span.alarms != alarms) {
		if (!(alarms & (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE | DAHDI_ALARM_LOOPBACK)) &&
			r1t1_card->sync) {
			/* Use the recieve signalling */
			r1t1_card->span.syncsrc = r1t1_card->span.spanno;
		} else {
			r1t1_card->span.syncsrc = 0;
		}
	}
	if (r1t1_card->alarmtimer) {
		alarms |= DAHDI_ALARM_RECOVER;
		if (debug)
			printk(KERN_DEBUG "R1T1: alarmtimer %x alarm clearing\n", r1t1_card->alarmtimer);
		led_state = RECOVER;
	}
	if ((c & 0x8) && !(r1t1_card->ise1)) {
		alarms |= DAHDI_ALARM_YELLOW;
		if (debug)
			printk(KERN_DEBUG "R1T1: SR2 08 YELLOW ALARM\n");
		led_state = YEL_ALM;
	}

	r1t1_card->span.alarms = alarms;
	c = __r1t1_get_reg(r1t1_card, R1T1_STATE / 4);
	if (c != led_state) {
		if (debug)
			printk(KERN_DEBUG "R1T1: State was %x, Now setting to %x \n", c, led_state);
		__r1t1_set_reg(r1t1_card, R1T1_STATE / 4, led_state);
	}
	dahdi_alarm_notify(&r1t1_card->span);
}

static void __r1t1_do_counters(struct r1t1_card *r1t1_card)
{
	if (r1t1_card->alarmtimer) {
		if (!--r1t1_card->alarmtimer) {
			r1t1_card->span.alarms &= ~(DAHDI_ALARM_RECOVER);
			/* Clear yellow alarm */
			if (r1t1_card->ise1)
				__r1t1_set_reg(r1t1_card, DS2155_TNAF, 0x5f);
			else
				__r1t1_set_reg(r1t1_card, DS2155_T1TCR1, 0x10);
			dahdi_alarm_notify(&r1t1_card->span);
		}
	}
}

DAHDI_IRQ_HANDLER(r1t1_interrupt)
{
	struct r1t1_card *r1t1_card = dev_id;
	unsigned long flags;
	unsigned int x, nextbuf;

	nextbuf = ioread8(r1t1_card->ioaddr + 0x805) & 0x3;

	if (!(nextbuf & 2))
		return IRQ_NONE;

	nextbuf &= 1;

	r1t1_card->intcount++;

	--r1t1_card->clocktimeout;

	r1t1_receiveprep(r1t1_card, r1t1_card->nextbuf);
	r1t1_transmitprep(r1t1_card, r1t1_card->nextbuf);

	spin_lock_irqsave(&r1t1_card->lock, flags);

	/* Acknowledge the interrupt */
	__r1t1_set_reg(r1t1_card, R1T1_CONTROL / 4, 0x03);
	__r1t1_set_reg(r1t1_card, R1T1_CONTROL / 4, 0x01);

	/* Count down timers */
	__r1t1_do_counters(r1t1_card);

	/* Do some things that we don't have to do very often */
	x = r1t1_card->intcount & 15 /* 63 */ ;
	switch (x) {
	case 0:
	case 1:
	case 2:
		__r1t1_check_sigbits(r1t1_card, x);
		break;
	case 4:
		/* Check alarms 1/4 as frequently */
		if (!(r1t1_card->intcount & 0x30))
			__r1t1_check_alarms(r1t1_card);
		break;
	}

	r1t1_card->nextbuf = (r1t1_card->nextbuf + 1) & 0x01;

	spin_unlock_irqrestore(&r1t1_card->lock, flags);

	return IRQ_RETVAL(1);
}


#ifdef USE_G168_DSP

inline void __r1t1_card_pci_out(struct r1t1_card *r1t1_card, const unsigned int addr,
								const unsigned int value)
{
	writel(value, &r1t1_card->membase[addr]);
}

inline unsigned int __r1t1_card_pci_in(struct r1t1_card *r1t1_card, const unsigned int addr)
{
	unsigned int value;
	value = readl(&r1t1_card->membase[addr]);
	return value;
}

static void r1t1_card_reset_dsp(struct r1t1_card *r1t1_card)
{
	unsigned long flags;
	unsigned int hpi_c;

	spin_lock_irqsave(&r1t1_card->lock, flags);
	hpi_c = __r1t1_card_pci_in(r1t1_card, TARG_REGS + R1T1_HPIC);

	__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_HPIC, (hpi_c & ~DSP_RST));
	__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_HPIC, (hpi_c & ~DSP_RST));
	mdelay(100);

	r1t1_card->hpi_fast = 0;
	r1t1_card->hpi_xadd = 0;
	r1t1_card->dsp_sel = 0;

	__r1t1_card_pci_out(r1t1_card, R1T1_HCS_REG + TARG_REGS, 0);
	__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_HPIC, (hpi_c | DSP_RST));

	spin_unlock_irqrestore(&r1t1_card->lock, flags);
}


int try_select_dsp(struct r1t1_card *r1t1_card)
{
	int sel = __r1t1_card_pci_in(r1t1_card, R1T1_HCS_REG + TARG_REGS);

	if (sel == 0) {

		__r1t1_card_pci_out(r1t1_card, R1T1_HCS_REG + TARG_REGS, 1);

		r1t1_card->dsp_sel = 1;
		return (0);
	}
	return (1);
}

void r1t1_card_select_dsp(struct r1t1_card *r1t1_card)
{
	int ridiculous = 0;

	while (try_select_dsp(r1t1_card)) {
		ridiculous++;
		if (ridiculous > 100000) {
			printk(KERN_ERR "R1T1: Waited 10 seconds ... nothing happened. Quitting.\n");
			return;
		}
	}
	return;
}

void r1t1_card_unselect_dsp(struct r1t1_card *r1t1_card)
{
	__r1t1_card_pci_out(r1t1_card, R1T1_HCS_REG + TARG_REGS, 0);
	return;
}

static unsigned short int r1t1_card_dsp_ping(struct r1t1_card *r1t1_card)
{

	gpakPingDspStat_t ping_stat;
	unsigned short int dsp_ver;

	r1t1_card_select_dsp(r1t1_card);

	ping_stat = gpakPingDsp(r1t1_card, r1t1_card->num, &dsp_ver);

	if (debug & DEBUG_DSP) {
		if (ping_stat == PngSuccess)
			printk(KERN_INFO "R1T1: %d %d: G168 DSP Ping DSP Version %x\n", r1t1_card->num + 1,
				   r1t1_card->dsp_sel, dsp_ver);
		else
			printk(KERN_ERR "R1T1: %d %d: G168 DSP Ping Error %d\n", r1t1_card->num + 1, r1t1_card->dsp_sel,
				   ping_stat);
	}

	r1t1_card_unselect_dsp(r1t1_card);

	if (ping_stat == PngSuccess)
		return dsp_ver;
	else
		return 0;
}

static int __devinit r1t1_span_download_dsp(struct r1t1_card *r1t1_card)
{
	unsigned short int DspId;
	gpakDownloadStatus_t dl_res = 0;

	r1t1_card_select_dsp(r1t1_card);
	DspId = r1t1_card->num;
	if ((dl_res = gpakDownloadDsp_5510(r1t1_card, DspId, app_file)))
		printk(KERN_ERR "R1T1: %d DSP %d: G168 DSP App Loader Failed %d\n", r1t1_card->num + 1, 1,
			   dl_res);
	else
		printk(KERN_DEBUG "R1T1: %d DSP %d: G168 DSP App Loader Success %d\n", r1t1_card->num + 1, 1,
			   dl_res);

	r1t1_card_dsp_set(r1t1_card, DSP_IFBLK_ADDRESS, 0);
	r1t1_card_dsp_set(r1t1_card, DSP_IFBLK_ADDRESS + 1, 0);

	r1t1_card_unselect_dsp(r1t1_card);

	if (dl_res)
		return -1;
	else
		return 0;
}

static void __devinit r1t1_span_run_dsp(struct r1t1_card *r1t1_card)
{
	r1t1_card_select_dsp(r1t1_card);
	r1t1_card_hpic_set(r1t1_card, R1T1_BL_GO);
	r1t1_card_unselect_dsp(r1t1_card);
	return;
}

static GpakPortConfig_t Gpak_32_chan_port_config = {

	/* GpakSlotCfg_t         SlotsSelect1          port 1 Slot selection */
	SlotCfgNone,
	/* unsigned short int    FirstBlockNum1        port 1 first group Block Number */
	0x0000,
	/* unsigned short int    FirstSlotMask1        port 1 first group Slot Mask */
	0x0000,
	/* unsigned short int    SecBlockNum1          port 1 second group Block Number */
	0x0000,
	/* unsigned short int    SecSlotMask1          port 1 second group Slot Mask */
	0x0000,
	/* GpakSerWordSize_t     SerialWordSize1       port 1 serial word size */
	SerWordSize8,
	/* GpakCompandModes      CompandingMode1       port 1 companding mode */
	cmpNone,
	/* GpakSerFrameSyncPol_t TxFrameSyncPolarity1  port 1 Tx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerFrameSyncPol_t RxFrameSyncPolarity1  port 1 Rx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerClockPol_t     TxClockPolarity1      port 1 Tx Clock Polarity */
	SerClockActHigh,
	/* GpakSerClockPol_t     RxClockPolarity1      port 1 Rx Clock Polarity */
	SerClockActHigh,
	/* GpakSerDataDelay_t    TxDataDelay1          port 1 Tx data delay */
	DataDelay1,
	/* GpakSerDataDelay_t    RxDataDelay1          port 1 Rx data delay */
	DataDelay1,
	/* GpakActivation        DxDelay1              port 1 DX Delay */
	Disabled,
	/* unsigned short int    ThirdSlotMask1        port 1 3rd group Slot Mask */
	0x0000,
	/* unsigned short int    FouthSlotMask1        port 1 4th group Slot Mask */
	0x0000,
	/* unsigned short int    FifthSlotMask1        port 1 5th group Slot Mask */
	0x0000,
	/* unsigned short int    SixthSlotMask1        port 1 6th group Slot Mask */
	0x0000,
	/* unsigned short int    SevenSlotMask1        port 1 7th group Slot Mask */
	0x0000,
	/* unsigned short int    EightSlotMask1        port 1 8th group Slot Mask */
	0x0000,

	/* GpakSlotCfg_t         SlotsSelect2          port 2 Slot selection */
	SlotCfg8Groups,
	/* unsigned short int    FirstBlockNum2        port 2 first group Block Number */
	0,
	/* unsigned short int    FirstSlotMask2        port 2 first group Slot Mask */
	0x1110,
	/* unsigned short int    SecBlockNum2          port 2 second group Block Number */
	1,
	/* unsigned short int    SecSlotMask2          port 2 second group Slot Mask */
	0x1111,
	/* GpakSerWordSize_t     SerialWordSize2       port 2 serial word size */
	SerWordSize8,
	/* GpakCompandModes      CompandingMode2       port 2 companding mode */
	cmpNone,
	/* GpakSerFrameSyncPol_t TxFrameSyncPolarity2  port 2 Tx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerFrameSyncPol_t RxFrameSyncPolarity2  port 2 Rx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerClockPol_t     TxClockPolarity2      port 2 Tx Clock Polarity */
	SerClockActHigh,
	/* GpakSerClockPol_t     RxClockPolarity2      port 2 Rx Clock Polarity */
	SerClockActHigh,
	/* GpakSerDataDelay_t    TxDataDelay2          port 2 Tx data delay */
	DataDelay1,
	/* GpakSerDataDelay_t    RxDataDelay2          port 2 Rx data delay */
	DataDelay1,
	/* GpakActivation        DxDelay2              port 2 DX Delay */
	Disabled,
	/* unsigned short int    ThirdSlotMask2        port 2 3rd group Slot Mask */
	0x1111,
	/* unsigned short int    FouthSlotMask2        port 2 4th group Slot Mask */
	0x1111,
	/* unsigned short int    FifthSlotMask2        port 2 5th group Slot Mask */
	0x1111,
	/* unsigned short int    SixthSlotMask2        port 2 6th group Slot Mask */
	0x1111,
	/* unsigned short int    SevenSlotMask2        port 2 7th group Slot Mask */
	0x1111,
	/* unsigned short int    EightSlotMask2        port 2 8th group Slot Mask */
	0x1111,

	/* GpakSlotCfg_t         SlotsSelect3          port 3 Slot selection */
	SlotCfg8Groups,
	/* unsigned short int    FirstBlockNum3        port 3 first group Block Number */
	0,
	/* unsigned short int    FirstSlotMask3        port 3 first group Slot Mask */
	0x1110,
	/* unsigned short int    SecBlockNum3          port 3 second group Block Number */
	1,
	/* unsigned short int    SecSlotMask3          port 3 second group Slot Mask */
	0x1111,
	/* GpakSerWordSize_t     SerialWordSize3       port 3 serial word size */
	SerWordSize8,
	/* GpakCompandModes      CompandingMode3       port 3 companding mode */
	cmpNone,
	/* GpakSerFrameSyncPol_t TxFrameSyncPolarity3  port 3 Tx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerFrameSyncPol_t RxFrameSyncPolarity3  port 3 Rx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerClockPol_t     TxClockPolarity3      port 3 Tx Clock Polarity */
	SerClockActHigh,
	/* GpakSerClockPol_t     RxClockPolarity3      port 3 Rx Clock Polarity */
	SerClockActHigh,
	/* GpakSerDataDelay_t    TxDataDelay3          port 3 Tx data delay */
	DataDelay1,
	/* GpakSerDataDelay_t    RxDataDelay3          port 3 Rx data delay */
	DataDelay1,
	/* GpakActivation        DxDelay3              port 3 DX Delay */
	Disabled,
	/* unsigned short int    ThirdSlotMask3        port 3 3rd group Slot Mask */
	0x1111,
	/* unsigned short int    FouthSlotMask3        port 3 4th group Slot Mask */
	0x1111,
	/* unsigned short int    FifthSlotMask3        port 3 5th group Slot Mask */
	0x1111,
	/* unsigned short int    SixthSlotMask3        port 3 6th group Slot Mask */
	0x1111,
	/* unsigned short int    SevenSlotMask3        port 3 7th group Slot Mask */
	0x1111,
	/* unsigned short int    EightSlotMask3        port 3 8th group Slot Mask */
	0x1111,
};


static void r1t1_card_dsp_show_portconfig(GpakPortConfig_t PortConfig)
{
	if (debug & DEBUG_DSP) {
		printk("%x = %s\n", PortConfig.SlotsSelect1, "SlotsSelect1");
		printk("%x = %s\n", PortConfig.FirstBlockNum1, "FirstBlockNum1");
		printk("%x = %s\n", PortConfig.FirstSlotMask1, "FirstSlotMask1");
		printk("%x = %s\n", PortConfig.SecBlockNum1, "SecBlockNum1");
		printk("%x = %s\n", PortConfig.SecSlotMask1, "SecSlotMask1");
		printk("%x = %s\n", PortConfig.SerialWordSize1, "SerialWordSize1");
		printk("%x = %s\n", PortConfig.CompandingMode1, "CompandingMode1");
		printk("%x = %s\n", PortConfig.TxFrameSyncPolarity1, "TxFrameSyncPolarity1");
		printk("%x = %s\n", PortConfig.RxFrameSyncPolarity1, "RxFrameSyncPolarity1");
		printk("%x = %s\n", PortConfig.TxClockPolarity1, "TxClockPolarity1");
		printk("%x = %s\n", PortConfig.RxClockPolarity1, "RxClockPolarity1");
		printk("%x = %s\n", PortConfig.TxDataDelay1, "TxDataDelay1");
		printk("%x = %s\n", PortConfig.RxDataDelay1, "RxDataDelay1");
		printk("%x = %s\n", PortConfig.DxDelay1, "DxDelay1");

		printk("%x = %s\n", PortConfig.ThirdSlotMask1, "ThirdSlotMask1");
		printk("%x = %s\n", PortConfig.FouthSlotMask1, "FouthSlotMask1");
		printk("%x = %s\n", PortConfig.FifthSlotMask1, "FifthSlotMask1");
		printk("%x = %s\n", PortConfig.SixthSlotMask1, "SixthSlotMask1");
		printk("%x = %s\n", PortConfig.SevenSlotMask1, "SevenSlotMask1");
		printk("%x = %s\n", PortConfig.EightSlotMask1, "EightSlotMask1");

		printk("%x = %s\n", PortConfig.SlotsSelect2, "SlotsSelect2");
		printk("%x = %s\n", PortConfig.FirstBlockNum2, "FirstBlockNum2");
		printk("%x = %s\n", PortConfig.FirstSlotMask2, "FirstSlotMask2");
		printk("%x = %s\n", PortConfig.SecBlockNum2, "SecBlockNum2");
		printk("%x = %s\n", PortConfig.SecSlotMask2, "SecSlotMask2");
		printk("%x = %s\n", PortConfig.SerialWordSize2, "SerialWordSize2");
		printk("%x = %s\n", PortConfig.CompandingMode2, "CompandingMode2");
		printk("%x = %s\n", PortConfig.TxFrameSyncPolarity2, "TxFrameSyncPolarity2");
		printk("%x = %s\n", PortConfig.RxFrameSyncPolarity2, "RxFrameSyncPolarity2");
		printk("%x = %s\n", PortConfig.TxClockPolarity2, "TxClockPolarity2");
		printk("%x = %s\n", PortConfig.RxClockPolarity2, "RxClockPolarity2");
		printk("%x = %s\n", PortConfig.TxDataDelay2, "TxDataDelay2");
		printk("%x = %s\n", PortConfig.RxDataDelay2, "RxDataDelay2");
		printk("%x = %s\n", PortConfig.DxDelay2, "DxDelay2");

		printk("%x = %s\n", PortConfig.ThirdSlotMask2, "ThirdSlotMask2");
		printk("%x = %s\n", PortConfig.FouthSlotMask2, "FouthSlotMask2");
		printk("%x = %s\n", PortConfig.FifthSlotMask2, "FifthSlotMask2");
		printk("%x = %s\n", PortConfig.SixthSlotMask2, "SixthSlotMask2");
		printk("%x = %s\n", PortConfig.SevenSlotMask2, "SevenSlotMask2");
		printk("%x = %s\n", PortConfig.EightSlotMask2, "EightSlotMask2");

		printk("%x = %s\n", PortConfig.SlotsSelect3, "SlotsSelect3");
		printk("%x = %s\n", PortConfig.FirstBlockNum3, "FirstBlockNum3");
		printk("%x = %s\n", PortConfig.FirstSlotMask3, "FirstSlotMask3");
		printk("%x = %s\n", PortConfig.SecBlockNum3, "SecBlockNum3");
		printk("%x = %s\n", PortConfig.SecSlotMask3, "SecSlotMask3");
		printk("%x = %s\n", PortConfig.SerialWordSize3, "SerialWordSize3");
		printk("%x = %s\n", PortConfig.CompandingMode3, "CompandingMode3");
		printk("%x = %s\n", PortConfig.TxFrameSyncPolarity3, "TxFrameSyncPolarity3");
		printk("%x = %s\n", PortConfig.RxFrameSyncPolarity3, "RxFrameSyncPolarity3");
		printk("%x = %s\n", PortConfig.TxClockPolarity3, "TxClockPolarity3");
		printk("%x = %s\n", PortConfig.RxClockPolarity3, "RxClockPolarity3");
		printk("%x = %s\n", PortConfig.TxDataDelay3, "TxDataDelay3");
		printk("%x = %s\n", PortConfig.RxDataDelay3, "RxDataDelay3");
		printk("%x = %s\n", PortConfig.DxDelay3, "DxDelay3");

		printk("%x = %s\n", PortConfig.ThirdSlotMask3, "ThirdSlotMask3");
		printk("%x = %s\n", PortConfig.FouthSlotMask3, "FouthSlotMask3");
		printk("%x = %s\n", PortConfig.FifthSlotMask3, "FifthSlotMask3");
		printk("%x = %s\n", PortConfig.SixthSlotMask3, "SixthSlotMask3");
		printk("%x = %s\n", PortConfig.SevenSlotMask3, "SevenSlotMask3");
		printk("%x = %s\n", PortConfig.EightSlotMask3, "EightSlotMask3");

	}
	return;
}

static int __devinit r1t1_span_dsp_configureports(struct r1t1_card *r1t1_card,
												  GpakPortConfig_t PortConfig)
{
	gpakConfigPortStatus_t cp_res;
	GPAK_PortConfigStat_t cp_error;
	unsigned short int DspId;

	r1t1_card_dsp_show_portconfig(PortConfig);

	r1t1_card_select_dsp(r1t1_card);
	DspId = r1t1_card->num;

	if ((cp_res = gpakConfigurePorts(r1t1_card, DspId, &PortConfig, &cp_error)))
		printk(KERN_ERR "R1T1: %d DSP %d: G168 DSP Port Config failed res = %d error = %d\n",
			   r1t1_card->num + 1, 1, cp_res, cp_error);
	else if (debug & DEBUG_DSP) {
		printk(KERN_DEBUG "R1T1: %d DSP %d: G168 DSP Port Config success %d\n", r1t1_card->num + 1, 1,
			   cp_res);
	}

	r1t1_card_unselect_dsp(r1t1_card);

	if (cp_res)
		return -1;
	else
		return 0;
}

static GpakChannelConfig_t Gpak_chan_config = {

	/* GpakSerialPort_t    PCM Input Serial Port A Id */
	SerialPort2,
	/* unsigned short int  PCM Input Time Slot */
	0,
	/* GpakSerialPort_t    PCM Output Serial Port A Id */
	SerialPort3,
	/* unsigned short int  PCM Output Time Slot */
	0,
	/* GpakSerialPort_t    PCM Input Serial Port B Id */
	SerialPort3,
	/* unsigned short int  PCM Input Time Slot */
	0,
	/* GpakSerialPort_t    PCM Output Serial Port B Id */
	SerialPortNull,
	/* unsigned short int  PCM Output Time Slot */
	0,
	/* GpakToneTypes       ToneTypesA A side Tone Detect Types */
	Null_tone,
	/* GpakToneTypes       ToneTypesB B side Tone Detect Types */
	Null_tone,
	/* GpakActivation      Echo Cancel A Enabled */
	Disabled,
	/* GpakActivation      Echo Cancel B Enabled */
	Disabled,

	{
	 /* short int  Echo Can Num Taps (tail length) 64 = 512 32 = 256 */
	 1024,
	 /* short int  Echo Can NLP Type */
	 3,
	 /* short int  Echo Can Adapt Enable flag */
	 1,
	 /* short int  Echo Can G165 Detect Enable flag */
	 1,
	 /* short int  Echo Can Double Talk threshold */
	 4,
	 /* short int  Echo Can NLP threshold */
	 21,
	 /* short int  Dynamic NLP control, NLP limit when EC about to converged */
	 17,
	 /* short int  Dynamic NLP control, NLP limit when EC not converged yet */
	 12,
	 /* short int  suppression level for NLP_SUPP mode */
	 0,
	 /* short int  Echo Can CNG Noise threshold */
	 50,
	 /* short int  Echo Can Max Adapts per frame */
	 40,
	 /* short int  Echo Can Cross Correlation limit */
	 20,
	 /* short int  Echo Can Num FIR Segments */
	 3,
	 /* short int  Echo Can FIR Segment Length */
	 64,
	 },

	{
	 /* short int  Echo Can Num Taps (tail length) */
	 1024,
	 /* short int  Echo Can NLP Type */
	 3,
	 /* short int  Echo Can Adapt Enable flag */
	 1,
	 /* short int  Echo Can G165 Detect Enable flag */
	 1,
	 /* short int  Echo Can Double Talk threshold */
	 4,
	 /* short int  Echo Can NLP threshold */
	 21,
	 /* short int  Dynamic NLP control, NLP limit when EC about to converged */
	 17,
	 /* short int  Dynamic NLP control, NLP limit when EC not converged yet */
	 12,
	 /* short int  suppression level for NLP_SUPP mode */
	 0,
	 /* short int  Echo Can CNG Noise threshold */
	 50,
	 /* short int  Echo Can Max Adapts per frame */
	 40,
	 /* short int  Echo Can Cross Correlation limit */
	 20,
	 /* short int  Echo Can Num FIR Segments */
	 3,
	 /* short int  Echo Can FIR Segment Length */
	 64,
	 },

	/* GpakCompandModes    software companding */
	cmpNone,
	/* GpakRate_t          Gpak Frame Rate */
	rate2ms,
	Disabled,
	Disabled,
	Disabled,
	Disabled
};

static void r1t1_card_dsp_show_chanconfig(GpakChannelConfig_t ChanConfig)
{
	if (debug & DEBUG_DSP) {
		printk("%d = %s\n", ChanConfig.PcmInPortA, "PcmInPortA");
		printk("%d = %s\n", ChanConfig.PcmInSlotA, "PcmInSlotA");
		printk("%d = %s\n", ChanConfig.PcmOutPortA, "PcmOutPortA");
		printk("%d = %s\n", ChanConfig.PcmOutSlotA, "PcmOutSlotA");
		printk("%d = %s\n", ChanConfig.PcmInPortB, "PcmInPortB");
		printk("%d = %s\n", ChanConfig.PcmInSlotB, "PcmInSlotB");
		printk("%d = %s\n", ChanConfig.PcmOutPortB, "PcmOutPortB");
		printk("%d = %s\n", ChanConfig.PcmOutSlotB, "PcmOutSlotB");

		printk("%d = %s\n", ChanConfig.ToneTypesA, "ToneTypesA");
		printk("%d = %s\n", ChanConfig.ToneTypesB, "ToneTypesB");

		printk("%d = %s\n", ChanConfig.EcanEnableA, "EcanEnableA");
		printk("%d = %s\n", ChanConfig.EcanEnableB, "EcanEnableB");

		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanTapLength,
			   "EcanParametersA.EcanTapLength");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNlpType,
			   "EcanParametersA.EcanNlpType");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanAdaptEnable,
			   "EcanParametersA.EcanAdaptEnable");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanG165DetEnable,
			   "EcanParametersA.EcanG165DetEnable");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanDblTalkThresh,
			   "EcanParametersA.EcanDblTalkThresh");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNlpThreshold,
			   "EcanParametersA.EcanNlpThreshold");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNlpConv,
			   "EcanParametersA.EcanNlpConv");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNlpUnConv,
			   "EcanParametersA.EcanNlpUnConv");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNlpMaxSuppress,
			   "EcanParametersA.EcanNlpMaxSuppress");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanCngThreshold,
			   "EcanParametersA.EcanCngThreshold");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanAdaptLimit,
			   "EcanParametersA.EcanAdaptLimit");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanCrossCorrLimit,
			   "EcanParametersA.EcanCrossCorrLimit");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNumFirSegments,
			   "EcanParametersA.EcanNumFirSegments");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanFirSegmentLen,
			   "EcanParametersA.EcanFirSegmentLen");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanTapLength,
			   "EcanParametersB.EcanTapLength");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNlpType,
			   "EcanParametersB.EcanNlpType");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanAdaptEnable,
			   "EcanParametersB.EcanAdaptEnable");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanG165DetEnable,
			   "EcanParametersB.EcanG165DetEnable");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanDblTalkThresh,
			   "EcanParametersB.EcanDblTalkThresh");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNlpThreshold,
			   "EcanParametersB.EcanNlpThreshold");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNlpConv,
			   "EcanParametersB.EcanNlpConv");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNlpUnConv,
			   "EcanParametersB.EcanNlpUnConv");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNlpMaxSuppress,
			   "EcanParametersB.EcanNlpMaxSuppress");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanCngThreshold,
			   "EcanParametersB.EcanCngThreshold");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanAdaptLimit,
			   "EcanParametersB.EcanAdaptLimit");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanCrossCorrLimit,
			   "EcanParametersB.EcanCrossCorrLimit");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNumFirSegments,
			   "EcanParametersB.EcanNumFirSegments");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanFirSegmentLen,
			   "EcanParametersB.EcanFirSegmentLen");

		printk("%d = %s\n", ChanConfig.SoftwareCompand, "SoftwareCompand");

		printk("%d = %s\n", ChanConfig.FrameRate, "FrameRate");

		printk("%d = %s\n", ChanConfig.MuteToneA, "MuteToneA");
		printk("%d = %s\n", ChanConfig.MuteToneB, "MuteToneB");
		printk("%d = %s\n", ChanConfig.FaxCngDetA, "FaxCngDetA");
		printk("%d = %s\n", ChanConfig.FaxCngDetB, "FaxCngDetB");

	}
	return;
}

static int __devinit r1t1_span_dsp_configurechannel(struct r1t1_card *r1t1_card,
													GpakChannelConfig_t ChanConfig,
													int chan_num)
{
	GPAK_ChannelConfigStat_t chan_config_err;
	gpakConfigChanStatus_t chan_conf_stat;
	unsigned short int DspId;

	r1t1_card_dsp_show_chanconfig(ChanConfig);

	r1t1_card_select_dsp(r1t1_card);
	DspId = r1t1_card->num;

	if ((chan_conf_stat =
		 gpakConfigureChannel(r1t1_card, DspId, chan_num, tdmToTdm, &Gpak_chan_config,
							  &chan_config_err)))
		printk(KERN_ERR "R1T1: %d DSP %d: Chan %d G168 DSP Chan Config failed error = %d  %d\n",
			   r1t1_card->num + 1, 1, chan_num, chan_config_err, chan_conf_stat);
	else if (debug & DEBUG_DSP) {
		printk(KERN_DEBUG "R1T1: %d DSP %d: G168 DSP Chan %d Config success %d\n", r1t1_card->num + 1, 1,
			   chan_num, chan_conf_stat);
	}

	r1t1_card_unselect_dsp(r1t1_card);

	if (chan_conf_stat)
		return -1;
	else
		return 0;
}

static void r1t1_card_dsp_framestats(struct r1t1_card *r1t1_card)
{
	gpakReadFramingStatsStatus_t framing_status_status;
	unsigned short int ec1, ec2, ec3, dmaec, slips[6];
	unsigned short int DspId;

	r1t1_card_select_dsp(r1t1_card);
	DspId = r1t1_card->num;

	if (debug & DEBUG_DSP) {
		framing_status_status =
			gpakReadFramingStats(r1t1_card, DspId, &ec1, &ec2, &ec3, &dmaec, &slips[0]);
		if (framing_status_status == RfsSuccess) {
			printk(KERN_DEBUG "R1T1: %d DSP %d: G168 DSP Framing Status Success %d\n", r1t1_card->num + 1,
				   1, framing_status_status);
			if (ec1 + ec2 + ec3 + dmaec + slips[0] + slips[1] + slips[2] + slips[3] +
				slips[4] + slips[5]) {
				printk
					(KERN_DEBUG "R1T1: %d DSP %d: G168 DSP Framing Status p1 %2d p2 %2d p3 %2d stop %2d\n",
					 r1t1_card->num + 1, 1, ec1, ec2, ec3, dmaec);
				printk
					(KERN_DEBUG "R1T1: %d DSP %d: G168 DSP Framing Status slip0 %2d slip1 %2d slip2 %2d slip3 %2d slip4 %2d slip5 %2d\n",
					 1, 1, slips[0], slips[1], slips[2], slips[3], slips[4], slips[5]);
			} else
				printk(KERN_DEBUG "R1T1: %d DSP %d: G168 DSP Framing Status GOOD!!\n", r1t1_card->num + 1,
					   1);
		} else {
			printk(KERN_DEBUG "R1T1: %d DSP %d: G168 DSP Framing Status Failed %d\n", r1t1_card->num + 1, 1,
				   framing_status_status);
			printk(KERN_DEBUG "R1T1: %d DSP %d: G168 DSP Framing Status %d %d %d %d %d\n",
				   r1t1_card->num + 1, 1, ec1, ec2, ec3, dmaec, slips[0]);
		}
	}

	r1t1_card_unselect_dsp(r1t1_card);
	return;
}

static void r1t1_card_dsp_resetframestats(struct r1t1_card *r1t1_card)
{
	gpakResetFramingStatsStatus_t framing_reset_status;
	unsigned short int DspId;

	r1t1_card_select_dsp(r1t1_card);
	DspId = r1t1_card->num;

	if ((framing_reset_status = gpakResetFramingStats(r1t1_card, DspId)))
		printk(KERN_ERR "R1T1: %d DSP %d: G168 DSP Reset Framing Stats Failed %d\n", r1t1_card->num + 1,
			   1, framing_reset_status);
	else
		printk(KERN_INFO "R1T1: %d DSP %d: G168 DSP Reset Framing Stats Success %d\n", r1t1_card->num + 1,
			   1, framing_reset_status);

	r1t1_card_unselect_dsp(r1t1_card);
	return;
}

static void r1t1_card_dsp_cpustats(struct r1t1_card *r1t1_card)
{
	gpakReadCpuUsageStat_t cpu_status_status;
	unsigned short int pPeakUsage, pPrev1SecPeakUsage;
	unsigned short int DspId;

	r1t1_card_select_dsp(r1t1_card);
	DspId = r1t1_card->num;

	cpu_status_status = gpakReadCpuUsage(r1t1_card, DspId, &pPeakUsage, &pPrev1SecPeakUsage);
	if (cpu_status_status)
		printk(KERN_ERR "R1T1: %d DSP %d: G168 DSP CPU Status Failed %d\n", r1t1_card->num + 1, 1,
			   cpu_status_status);
	else
		printk(KERN_INFO "R1T1: %d DSP %d: G168 DSP CPU Status peek %2d  1 S %2d\n", r1t1_card->num + 1, 1,
			   pPeakUsage, pPrev1SecPeakUsage);


	r1t1_card_unselect_dsp(r1t1_card);
	return;
}

static void r1t1_chan_ec_enable(struct r1t1_card *r1t1_card, int chan_num)
{
	gpakAlgControlStat_t a_c_stat;
	GPAK_AlgControlStat_t a_c_err;
	unsigned short int DspId;
	unsigned int mask, slot_num;

	if (ec_disable & (1 << chan_num)) {
		if (debug & DEBUG_DSP)
			printk(KERN_DEBUG "r1t1 %d: Echo Can NOT enable DSP EC Chan %d\n", r1t1_card->num + 1,
				   chan_num);
		return;
	}

	DspId = r1t1_card->num;

	if (r1t1_card->ise1)
		slot_num = chanmap_e1[chan_num];
	else
		slot_num = chanmap_t1[chan_num];

	if (debug & DEBUG_DSP)
		printk(KERN_DEBUG "R1T1: %d: Echo Can enable DSP %d EC Chan %d\n", r1t1_card->num + 1, 1, chan_num);

	r1t1_card_select_dsp(r1t1_card);

	if ((a_c_stat = gpakAlgControl(r1t1_card, DspId, chan_num, EnableEcanB, &a_c_err))) {
		if ((a_c_stat = gpakAlgControl(r1t1_card, DspId, chan_num, EnableEcanB, &a_c_err))) {
			if ((a_c_stat = gpakAlgControl(r1t1_card, DspId, chan_num, EnableEcanB, &a_c_err))) {
				printk
					(KERN_ERR "R1T1: %d: G168 DSP Enable Alg Control failed res = %d error = %d\n",
					 r1t1_card->num + 1, a_c_stat, a_c_err);
			}
		}
	}

	mask = __r1t1_card_pci_in(r1t1_card, TARG_REGS + R1T1_ECA1);
	mask |= (1 << slot_num);

	r1t1_card_unselect_dsp(r1t1_card);

	return;
}

static void r1t1_chan_ec_disable(struct r1t1_card *r1t1_card, int chan_num)
{
	gpakAlgControlStat_t a_c_stat;
	GPAK_AlgControlStat_t a_c_err;
	unsigned short int DspId;
	unsigned int mask, slot_num;

	DspId = r1t1_card->num;

	if (r1t1_card->ise1)
		slot_num = chanmap_e1[chan_num];
	else
		slot_num = chanmap_t1[chan_num];

	if (debug & DEBUG_DSP)
		printk(KERN_DEBUG "R1T1: %d: Echo Can disable DSP %d EC Chan %d\n", r1t1_card->num + 1, 1,
			   chan_num);

	r1t1_card_select_dsp(r1t1_card);

	if ((a_c_stat = gpakAlgControl(r1t1_card, DspId, chan_num, BypassEcanB, &a_c_err))) {
		if ((a_c_stat = gpakAlgControl(r1t1_card, DspId, chan_num, BypassEcanB, &a_c_err))) {
			if ((a_c_stat = gpakAlgControl(r1t1_card, DspId, chan_num, BypassEcanB, &a_c_err))) {
				printk
					(KERN_ERR "R1T1: %d: G168 DSP Disable Alg Control failed res = %d error = %d\n",
					 r1t1_card->num + 1, a_c_stat, a_c_err);
			}
		}
	}

	mask = __r1t1_card_pci_in(r1t1_card, TARG_REGS + R1T1_ECA1);
	mask &= ~(1 << slot_num);

	r1t1_card_unselect_dsp(r1t1_card);

	return;
}


static int r1t1_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
							   struct dahdi_echocanparam *p,
							   struct dahdi_echocan_state **ec)
{
	struct dahdi_span *span = chan->span;
	struct r1t1_card *r1t1_card = container_of(span, struct r1t1_card, span);
	int chan_num;

	chan_num = chan->chanpos - 1;

	if (ecp->param_count > 0) {
		printk(KERN_WARNING
			   "R1T1 echo canceller does not support parameters; failing request\n");
		return -EINVAL;
	}

	if (debug & DEBUG_DSP) {
		printk(KERN_DEBUG "R1T1: %d Echo Can control Span %d Chan %d dahdi_chan %d\n", r1t1_card->num + 1,
			   1, chan_num, chan->channo);
		printk(KERN_DEBUG "DSP up %x\n", r1t1_card->dsp_up);
	}
	if (r1t1_card->dsp_up == 1) {
		*ec = r1t1_card->ec[chan_num];
		(*ec)->ops = &my_ec_ops;
		(*ec)->features = my_ec_features;
		r1t1_card->nextec |= (1 << chan_num);
		queue_work(r1t1_card->wq, &r1t1_card->work);
	}
	return 0;
}

static void r1t1_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
	struct dahdi_span *span = chan->span;
	struct r1t1_card *r1t1_card = container_of(span, struct r1t1_card, span);
	int chan_num;

	memset(ec, 0, sizeof(*ec));
	chan_num = chan->chanpos - 1;

	if (debug & DEBUG_DSP) {
		printk(KERN_DEBUG "R1T1: %d Echo Can control Span %d Chan %d dahdi_chan %d\n", r1t1_card->num + 1,
			   1, chan_num, chan->channo);
		printk(KERN_DEBUG "DSP up %x\n", r1t1_card->dsp_up);
	}

	if (r1t1_card->dsp_up == 1) {
		r1t1_card->nextec &= ~(1 << chan_num);
		queue_work(r1t1_card->wq, &r1t1_card->work);
	}
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void echocan_bh(void *data)
{
	struct r1t1_card *r1t1_card = data;
#else
static void echocan_bh(struct work_struct *data)
{
	struct r1t1_card *r1t1_card = container_of(data, struct r1t1_card, work);
#endif
	unsigned int todo, chan_num;

	todo = r1t1_card->nextec ^ r1t1_card->currec;
	if (debug & DEBUG_DSP) {
		printk(KERN_DEBUG "R1T1: %d Echo Can control bh change %x to %x\n", r1t1_card->num + 1, todo,
			   (r1t1_card->nextec & todo));
		printk(KERN_DEBUG "nextec %x currec %x\n", r1t1_card->nextec, r1t1_card->currec);
		printk(KERN_DEBUG "span.channels = %d\n", r1t1_card->span.channels);
	}
	for (chan_num = 0; chan_num < r1t1_card->span.channels; chan_num++) {
		if (todo & (1 << chan_num)) {
			if (r1t1_card->nextec & (1 << chan_num)) {
				r1t1_chan_ec_enable(r1t1_card, chan_num);
				r1t1_card->currec |= (1 << chan_num);
				schedule();
			} else {
				r1t1_chan_ec_disable(r1t1_card, chan_num);
				r1t1_card->currec &= ~(1 << chan_num);
				schedule();
			}
		}
	}
}


static int __devinit r1t1_card_init_dsp(struct r1t1_card *r1t1_card)
{
	int loops = 0;
	__u16 high, low;
	int chan_num, chan_count, slot_num;
	int ifb_z = 4;

	if (debug & DEBUG_DSP)
		printk(KERN_DEBUG "R1T1: Reset DSP\n");

	r1t1_card_reset_dsp(r1t1_card);

	if (debug & DEBUG_DSP)
		printk(KERN_DEBUG "R1T1: Un-Reset DSP\n");

	__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_ECB1, 0x00000000);	/* use ec b */
	__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_ECA1, 0x00000000);	/* use ec a */

	if (no_ec)
		return -1;

	__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_HPIC,
						(~EC_ON & __r1t1_card_pci_in(r1t1_card, TARG_REGS + R1T1_HPIC)));

	if (r1t1_span_download_dsp(r1t1_card)) {
		return -1;
	}

	r1t1_span_run_dsp(r1t1_card);
	if (debug & DEBUG_DSP)
		printk(KERN_DEBUG "R1T1: %d DSP %d: GO!!\n", r1t1_card->num + 1, 1);

	while (ifb_z != 0) {
		ifb_z = 0;
		msleep(1);
		r1t1_card_select_dsp(r1t1_card);
		high = r1t1_card_dsp_get(r1t1_card, DSP_IFBLK_ADDRESS);
		low = r1t1_card_dsp_get(r1t1_card, DSP_IFBLK_ADDRESS + 1);

		if (debug & DEBUG_DSP)
			printk(KERN_DEBUG "R1T1: %d DSP %d: IfBlockPntr %x\n", r1t1_card->num + 1, 1,
				   ((high << 16) + low));

		if ((high == 0) && (low == 0))
			ifb_z++;

		r1t1_card_unselect_dsp(r1t1_card);
		schedule();

		if ((loops++) > 2) {
			printk(KERN_DEBUG "R1T1: DSP not responding...is EC installed?\n");
			return -1;
		}
	}

	r1t1_card->hpi_fast = 0;

	r1t1_card_dsp_ping(r1t1_card);

	if (r1t1_span_dsp_configureports(r1t1_card, Gpak_32_chan_port_config))
		return -1;

	Gpak_chan_config.EcanParametersA.EcanNlpType = nlp_type;
	Gpak_chan_config.EcanParametersB.EcanNlpType = nlp_type;

	Gpak_chan_config.EcanEnableB = Enabled;

	Gpak_chan_config.EcanEnableA = Disabled;
	Gpak_chan_config.SoftwareCompand = cmpNone;

	r1t1_card_dsp_ping(r1t1_card);
	chan_count = 0;

	for (chan_num = 0; chan_num < r1t1_card->span.channels; chan_num++) {
		chan_count++;

		if (r1t1_card->ise1) {
			unsigned int hpi_c;
			hpi_c = __r1t1_card_pci_in(r1t1_card, TARG_REGS + R1T1_HPIC);
			__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_HPIC, (hpi_c | XLATE));
			__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_XLATE_EN, 0xffff7fff);
			slot_num = chanmap_e1[chan_num];
			if (chan_num != 15)
				Gpak_chan_config.SoftwareCompand = cmpPCMU;
			else
				Gpak_chan_config.SoftwareCompand = cmpNone;
		} else {
			slot_num = chanmap_t1[chan_num];
			Gpak_chan_config.SoftwareCompand = cmpPCMU;
		}

		Gpak_chan_config.PcmInSlotA = slot_num * 4;
		Gpak_chan_config.PcmOutSlotA = slot_num * 4;
		Gpak_chan_config.PcmInSlotB = slot_num * 4;
		Gpak_chan_config.PcmOutSlotB = slot_num * 4;

		if (r1t1_span_dsp_configurechannel(r1t1_card, Gpak_chan_config, chan_num))
			return -1;

		r1t1_chan_ec_disable(r1t1_card, chan_num);
	}

	printk(KERN_NOTICE "R1T1: %d DSP %d: %d channels configured\n", r1t1_card->num + 1, 1, chan_count);

	r1t1_card->dsp_up = 1;
#if DAHDI_VER < KERNEL_VERSION(2,4,0)
	r1t1_card->span.echocan_create = r1t1_echocan_create;
#endif

	if (debug & DEBUG_DSP) {

		r1t1_card_dsp_cpustats(r1t1_card);
		r1t1_card_dsp_framestats(r1t1_card);
		r1t1_card_dsp_resetframestats(r1t1_card);

		msleep(400);

		r1t1_card_dsp_cpustats(r1t1_card);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_cpustats(r1t1_card);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
		msleep(400);
		r1t1_card_dsp_framestats(r1t1_card);
	}

	__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_HPIC,
						(EC_ON | __r1t1_card_pci_in(r1t1_card, TARG_REGS + R1T1_HPIC)));

	__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_ECB1, 0x00000000);	/* use ec b */

	__r1t1_card_pci_out(r1t1_card, TARG_REGS + R1T1_ECA1, ec_sw);	/* use ec a */

	r1t1_card_dsp_ping(r1t1_card);

	r1t1_card->wq = create_singlethread_workqueue("r1t1_ec");

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	INIT_WORK(&r1t1_card->work, echocan_bh, r1t1_card);
#else
	INIT_WORK(&r1t1_card->work, echocan_bh);
#endif

	return (0);
}

#endif /* USE_G168_DSP */

static int r1t1_hardware_init(struct r1t1_card *r1t1_card)
{
	/* Setup DMA Addresses */
	/* Start at writedma */
	iowrite32(r1t1_card->writedma, r1t1_card->ioaddr + R1T1_TXBUFSTART);	/* Write start */
	ioread32(r1t1_card->ioaddr + R1T1_TXBUFSTART);
	iowrite32(r1t1_card->readdma, r1t1_card->ioaddr + R1T1_RXBUFSTART);	/* Read start */
	ioread32(r1t1_card->ioaddr + R1T1_RXBUFSTART);
	iowrite16((DAHDI_CHUNKSIZE * 32 * 2) >> 2, r1t1_card->ioaddr + R1T1_BUFLEN);
	ioread16(r1t1_card->ioaddr + R1T1_BUFLEN);

	if (r1t1_card->ise1) {
		r1t1_card->chanmap = chanmap_e1;
	} else
		r1t1_card->chanmap = chanmap_t1;

	r1t1_card->clocktimeout = 100;

	/* Reset the T1 and report */
	r1t1_framer_hard_reset(r1t1_card);

	return 0;

}

static int __devinit r1t1_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct r1t1_card *r1t1_card;
	unsigned int *canary;
	int x;
	struct dahdi_chan *chan_block;
	struct dahdi_echocan_state *ec_block;
	int chan_count;

	if (debug)
		printk(KERN_DEBUG "R1T1: init_one debug=%x e1=%d\n", debug, e1);

	if (pci_enable_device(pdev)) {
		printk(KERN_WARNING "R1T1: No Rhino spotted\n");
		return -EIO;
	}

	/* Find position */
	for (x = 0; x < RH_MAX_CARDS; x++) {
		if (!cards[x]) {
			break;
		}
	}

	if (x >= RH_MAX_CARDS)
		return -1;

	r1t1_card = kmalloc(sizeof *r1t1_card, GFP_KERNEL);

	if (r1t1_card == NULL) {
		printk(KERN_ERR "R1T1: No memory available for card\n");
		return -ENOMEM;
	}

	memset(r1t1_card, 0x0, sizeof *r1t1_card);
	r1t1_card->num = x;
	cards[r1t1_card->num] = r1t1_card;
	spin_lock_init(&r1t1_card->lock);
	r1t1_card->dev = pdev;
	r1t1_card->pciaddr = pci_resource_start(pdev, 0);

	if (e1 & (1 << r1t1_card->num)) {
		r1t1_card->ise1 = 1;
		r1t1_card->variety = "Rhino R1T1 E1/PRA";
		printk(KERN_NOTICE "R1T1: %x configured E1 by module_param\n", r1t1_card->num);
	} else {
		r1t1_card->ise1 = 0;
		r1t1_card->variety = "Rhino R1T1 T1/PRI";
		printk(KERN_NOTICE "R1T1: %x configured as T1\n", r1t1_card->num);
	}

	chan_count = r1t1_card->ise1 ? 31 : 24;
	chan_block = kmalloc(chan_count * sizeof *chan_block, GFP_KERNEL);
	ec_block = kmalloc(chan_count * sizeof *ec_block, GFP_KERNEL);

	if (chan_block == NULL || ec_block == NULL) {
		if (chan_block)
			kfree(chan_block);

		if (ec_block)
			kfree(ec_block);

		kfree(r1t1_card);

		printk(KERN_ERR "R1T1: No memory available for chans/ec\n");
		return -ENOMEM;
	}

	memset(chan_block, 0, chan_count * sizeof *chan_block);
	memset(ec_block, 0, chan_count * sizeof *ec_block);

	for (x = 0; x < chan_count; x++) {
		r1t1_card->chans[x] = chan_block + x;
		r1t1_card->ec[x] = ec_block + x;
	}

	printk(KERN_INFO "R1T1: pciaddr = %x \n", (__u32) r1t1_card->pciaddr);

	if (!request_mem_region(r1t1_card->pciaddr, R1T1_SIZE, "R1T1")) {
		printk(KERN_ERR "R1T1: Unable to claim R1T1 PCI space\n");
		kfree(chan_block);
		kfree(ec_block);
		cards[r1t1_card->num] = NULL;
		kfree(r1t1_card);
		return -ENOMEM;
	}

	printk(KERN_INFO "R1T1: claimed R1T1 PCI space\n");

	r1t1_card->ioaddr = ioremap_nocache((dma_addr_t) r1t1_card->pciaddr, R1T1_SIZE);
	r1t1_card->membase = r1t1_card->ioaddr;
	printk(KERN_INFO "R1T1: mapped R1T1 PCI space to %x\n", (__u32) (dma_addr_t) r1t1_card->ioaddr);

	r1t1_card->writechunk =
		/* 32 channels, Double-buffer, Read/Write */
		(unsigned char *) pci_alloc_consistent(pdev,
											   DAHDI_MAX_CHUNKSIZE * 32 * 2 * 2 +
											   8, &r1t1_card->writedma);
	if (!r1t1_card->writechunk) {
		printk(KERN_ERR "R1T1: Unable to allocate DMA-able memory\n");
		kfree(chan_block);
		kfree(ec_block);
		iounmap(r1t1_card->ioaddr);
		cards[r1t1_card->num] = NULL;
		kfree(r1t1_card);
		return -ENOMEM;
	}

	/* Read is after the whole write piece (in bytes) */
	r1t1_card->readchunk = r1t1_card->writechunk + DAHDI_CHUNKSIZE * 32 * 2;

	/* Same thing...  */
	r1t1_card->readdma = r1t1_card->writedma + DAHDI_CHUNKSIZE * 32 * 2;

	/* Initialize Write/Buffers to all blank data */
	memset((void *) r1t1_card->writechunk, 0x00, DAHDI_MAX_CHUNKSIZE * 2 * 2 * 32);
	/* Initialize canary */
	canary = (unsigned int *) (r1t1_card->readchunk + DAHDI_CHUNKSIZE * 64 - 4);
	*canary = (CANARY << 16) | (0xffff);

	r1t1_card->nextbuf = 0;

	/* Enable bus mastering */
	pci_set_master(pdev);

	/* Keep track of which device we are */
	pci_set_drvdata(pdev, r1t1_card);

	/* Disable interrupts on the board before enabling them in Linux */
	__r1t1_set_reg(r1t1_card, R1T1_CONTROL / 4, 0x00);

	if (request_irq(pdev->irq, r1t1_interrupt, IRQF_SHARED, "r1t1", r1t1_card)) {
		printk(KERN_ERR "R1T1: Unable to request IRQ %d\n", pdev->irq);
		iounmap(r1t1_card->ioaddr);
		release_mem_region(r1t1_card->pciaddr, R1T1_SIZE);
		pci_free_consistent(r1t1_card->dev, DAHDI_MAX_CHUNKSIZE * 2 * 2 * 32 + 8,
							(void *) r1t1_card->writechunk, r1t1_card->writedma);
		kfree(chan_block);
		kfree(ec_block);
		cards[r1t1_card->num] = NULL;
		kfree(r1t1_card);
		return -EIO;
	}

	r1t1_card->version = ioread16(r1t1_card->ioaddr + R1T1_VERSION);

	printk(KERN_INFO "R1T1: Version %d\n", r1t1_card->version);

	if (r1t1_card->version < 36) {
		printk(KERN_ERR "R1T1: Found HW version less than 36, need to upgrade your hardware!\n");
		iounmap(r1t1_card->ioaddr);
		release_mem_region(r1t1_card->pciaddr, R1T1_SIZE);
		pci_free_consistent(r1t1_card->dev, DAHDI_MAX_CHUNKSIZE * 2 * 2 * 32 + 8,
							(void *) r1t1_card->writechunk, r1t1_card->writedma);
		free_irq(pdev->irq, r1t1_card);
		kfree(chan_block);
		kfree(ec_block);
		cards[r1t1_card->num] = NULL;
		kfree(r1t1_card);
		return -1;
	}

	/* Initialize hardware */
	r1t1_hardware_init(r1t1_card);

	/* Misc. software stuff */
	r1t1_software_init(r1t1_card);

#ifdef USE_G168_DSP
	r1t1_card->dsp_type = DSP_5510;
	r1t1_card_init_dsp(r1t1_card);
#endif /* USE_G168_DSP */

	printk(KERN_NOTICE "R1T1: Spotted a Rhino: %s version %d. Module Version " RHINOPKGVER
		   "\n", r1t1_card->variety, r1t1_card->version);

	return 0;
}

static void __devexit r1t1_remove_one(struct pci_dev *pdev)
{
	struct r1t1_card *r1t1_card = pci_get_drvdata(pdev);
	if (r1t1_card) {
#ifdef USE_G168_DSP
		if (r1t1_card->dsp_up) {
			flush_workqueue(r1t1_card->wq);
			destroy_workqueue(r1t1_card->wq);
		}
#endif /* USE_G168_DSP */

		r1t1_shutdown(&r1t1_card->span);

		/* Release span, possibly delayed */
		if (!r1t1_card->usecount)
			r1t1_release(r1t1_card);
		else
			r1t1_card->dead = 1;
	}
}

static struct pci_device_id r1t1_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_RHINO, PCI_DEVICE_R1T1)},
	{0}
};

MODULE_DEVICE_TABLE(pci, r1t1_pci_tbl);

static struct pci_driver r1t1_driver = {
  name:"r1t1",
  probe:r1t1_init_one,
  remove:__devexit_p(r1t1_remove_one),
  suspend:NULL,
  resume:NULL,
  id_table:r1t1_pci_tbl,
};

static int __init r1t1_init(void)
{
	return pci_register_driver(&r1t1_driver);
}

static void __exit r1t1_cleanup(void)
{
	pci_unregister_driver(&r1t1_driver);
}


module_param(debug, int, 0600);
MODULE_PARM_DESC(debug, "1 for debugging messages");
module_param(e1, int, 0600);
MODULE_PARM_DESC(e1, "1 for E1 mode set from module_param");
module_param(ec_disable, int, 0600);
module_param(ec_sw, int, 0600);
module_param(no_ec, int, 0600);
module_param(nlp_type, int, 0600);
MODULE_PARM_DESC(nlp_type, "0 - off, 1 - mute, 2 - rand, 3 - hoth, 4 - supp");

MODULE_DESCRIPTION("Rhino R1T1 T1-E1-J1 Driver " RHINOPKGVER);
MODULE_AUTHOR
	("Lee Reeves <helpdesk@rhinoequipment.com>\n\tBob Conklin <helpdesk@rhinoequipment.com>\n\tBryce Chidester <helpdesk@rhinoequipment.com>\n\tMatthew Gessner <helpdesk@rhinoequipment.com");
MODULE_VERSION(RHINOPKGVER);

#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

module_init(r1t1_init);
module_exit(r1t1_cleanup);
