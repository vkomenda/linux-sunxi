/*
 *  drivers/mtd/nand.c
 *
 *  Overview:
 *   This is the generic MTD driver for NAND flash devices. It should be
 *   capable of working with almost all NAND chips currently available.
 *
 *	Additional technical information is available on
 *	http://www.linux-mtd.infradead.org/doc/nand.html
 *
 *  Copyright (C) 2000 Steven J. Hill (sjhill@realitydiluted.com)
 *		  2002-2006 Thomas Gleixner (tglx@linutronix.de)
 *
 *  Credits:
 *	David Woodhouse for adding multichip support
 *
 *	Aleph One Ltd. and Toby Churchill Ltd. for supporting the
 *	rework for 2K page size chips
 *
 *  TODO:
 *	Enable cached programming for 2k page size chips
 *	Check, if mtd->ecctype should be set to MTD_ECC_HW
 *	if we have HW ECC support.
 *	BBT table is not serialized, has to be fixed
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DBG(fmt, arg...) pr_info(pr_fmt("%s: " fmt "\n"), __FUNCTION__, ##arg)

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/nand_bch.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/leds.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>

/* Define default oob placement schemes for large and small page devices */
static struct nand_ecclayout nand_oob_8 = {
	.eccbytes = 3,
	.eccpos = {0, 1, 2},
	.oobfree = {
		{.offset = 3,
		 .length = 2},
		{.offset = 6,
		 .length = 2} }
};

static struct nand_ecclayout nand_oob_16 = {
	.eccbytes = 6,
	.eccpos = {0, 1, 2, 3, 6, 7},
	.oobfree = {
		{.offset = 8,
		 . length = 8} }
};

static struct nand_ecclayout nand_oob_64 = {
	.eccbytes = 24,
	.eccpos = {
		   40, 41, 42, 43, 44, 45, 46, 47,
		   48, 49, 50, 51, 52, 53, 54, 55,
		   56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = {
		{.offset = 2,
		 .length = 38} }
};

static struct nand_ecclayout nand_oob_128 = {
	.eccbytes = 48,
	.eccpos = {
		   80, 81, 82, 83, 84, 85, 86, 87,
		   88, 89, 90, 91, 92, 93, 94, 95,
		   96, 97, 98, 99, 100, 101, 102, 103,
		   104, 105, 106, 107, 108, 109, 110, 111,
		   112, 113, 114, 115, 116, 117, 118, 119,
		   120, 121, 122, 123, 124, 125, 126, 127},
	.oobfree = {
		{.offset = 2,
		 .length = 78} }
};

static int nand_get_device(struct mtd_info *mtd, int new_state);

static int nand_do_write_oob(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops);

/*
 * For devices which display every fart in the system on a separate LED. Is
 * compiled away when LED support is disabled.
 */
DEFINE_LED_TRIGGER(nand_led_trigger);

static int check_offs_len(struct mtd_info *mtd,
					loff_t ofs, uint64_t len)
{
	struct nand_chip *chip = mtd->priv;
	int ret = 0;

	/* Start address must align on block boundary */
	if (ofs & ((1ULL << chip->phys_erase_shift) - 1)) {
		pr_debug("%s: unaligned address\n", __func__);
		ret = -EINVAL;
	}

	/* Length must align on block boundary */
	if (len & ((1ULL << chip->phys_erase_shift) - 1)) {
		pr_debug("%s: length not block aligned\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

/**
 * nand_release_device - [GENERIC] release chip
 * @mtd: MTD device structure
 *
 * Release chip lock and wake up anyone waiting on the device.
 */
static void nand_release_device(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

	/* Release the controller and the chip */
	spin_lock(&chip->controller->lock);
	chip->controller->active = NULL;
	chip->state = FL_READY;
	wake_up(&chip->controller->wq);
	spin_unlock(&chip->controller->lock);
}

/**
 * nand_read_byte - [DEFAULT] read one byte from the chip
 * @mtd: MTD device structure
 *
 * Default read function for 8bit buswidth
 */
static uint8_t nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	return readb(chip->IO_ADDR_R);
}

/**
 * nand_read_byte16 - [DEFAULT] read one byte endianness aware from the chip
 * nand_read_byte16 - [DEFAULT] read one byte endianness aware from the chip
 * @mtd: MTD device structure
 *
 * Default read function for 16bit buswidth with endianness conversion.
 *
 */
static uint8_t nand_read_byte16(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	return (uint8_t) cpu_to_le16(readw(chip->IO_ADDR_R));
}

/**
 * nand_read_word - [DEFAULT] read one word from the chip
 * @mtd: MTD device structure
 *
 * Default read function for 16bit buswidth without endianness conversion.
 */
static u16 nand_read_word(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	return readw(chip->IO_ADDR_R);
}

/**
 * nand_select_chip - [DEFAULT] control CE line
 * @mtd: MTD device structure
 * @chipnr: chipnumber to select, -1 for deselect
 *
 * Default select function for 1 chip devices.
 */
static void nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd->priv;

	switch (chipnr) {
	case -1:
		chip->cmd_ctrl(mtd, NAND_CMD_NONE, 0 | NAND_CTRL_CHANGE);
		break;
	case 0:
		break;

	default:
		BUG();
	}
}

/**
 * nand_write_byte - [DEFAULT] write single byte to chip
 * @mtd: MTD device structure
 * @byte: value to write
 *
 * Default function to write a byte to I/O[7:0]
 */
static void nand_write_byte(struct mtd_info *mtd, uint8_t byte)
{
	struct nand_chip *chip = mtd->priv;

	chip->write_buf(mtd, &byte, 1);
}

/**
 * nand_write_byte16 - [DEFAULT] write single byte to a chip with width 16
 * @mtd: MTD device structure
 * @byte: value to write
 *
 * Default function to write a byte to I/O[7:0] on a 16-bit wide chip.
 */
static void nand_write_byte16(struct mtd_info *mtd, uint8_t byte)
{
	struct nand_chip *chip = mtd->priv;
	uint16_t word = byte;

	/*
	 * It's not entirely clear what should happen to I/O[15:8] when writing
	 * a byte. The ONFi spec (Revision 3.1; 2012-09-19, Section 2.16) reads:
	 *
	 *    When the host supports a 16-bit bus width, only data is
	 *    transferred at the 16-bit width. All address and command line
	 *    transfers shall use only the lower 8-bits of the data bus. During
	 *    command transfers, the host may place any value on the upper
	 *    8-bits of the data bus. During address transfers, the host shall
	 *    set the upper 8-bits of the data bus to 00h.
	 *
	 * One user of the write_byte callback is nand_onfi_set_features. The
	 * four parameters are specified to be written to I/O[7:0], but this is
	 * neither an address nor a command transfer. Let's assume a 0 on the
	 * upper I/O lines is OK.
	 */
	chip->write_buf(mtd, (uint8_t *)&word, 2);
}

/**
 * nand_write_buf - [DEFAULT] write buffer to chip
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 *
 * Default write function for 8bit buswidth.
 */
static void nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;

	iowrite8_rep(chip->IO_ADDR_W, buf, len);
}

/**
 * nand_read_buf - [DEFAULT] read chip data into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 *
 * Default read function for 8bit buswidth.
 */
static void nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;

	ioread8_rep(chip->IO_ADDR_R, buf, len);
}

/**
 * nand_write_buf16 - [DEFAULT] write buffer to chip
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 *
 * Default write function for 16bit buswidth.
 */
static void nand_write_buf16(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	u16 *p = (u16 *) buf;

	iowrite16_rep(chip->IO_ADDR_W, p, len >> 1);
}

/**
 * nand_read_buf16 - [DEFAULT] read chip data into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 *
 * Default read function for 16bit buswidth.
 */
static void nand_read_buf16(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	u16 *p = (u16 *) buf;

	ioread16_rep(chip->IO_ADDR_R, p, len >> 1);
}

/**
 * nand_block_bad - [DEFAULT] Read bad block marker from the chip
 * @mtd: MTD device structure
 * @ofs: offset from device start
 * @getchip: 0, if the chip is already selected
 *
 * Check, if the block is bad.
 */
static int nand_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	int page, chipnr, res = 0, i = 0;
	struct nand_chip *chip = mtd->priv;
	u16 bad;

	if (chip->bbt_options & NAND_BBT_SCANLASTPAGE)
		ofs += mtd->erasesize - mtd->writesize;

	page = (int)(ofs >> chip->page_shift) & chip->pagemask;

	if (getchip) {
		chipnr = (int)(ofs >> chip->chip_shift);

		nand_get_device(mtd, FL_READING);

		/* Select the NAND device */
		chip->select_chip(mtd, chipnr);
	}

	do {
		if (chip->options & NAND_BUSWIDTH_16) {
			chip->cmdfunc(mtd, NAND_CMD_READOOB,
					chip->badblockpos & 0xFE, page);
			bad = cpu_to_le16(chip->read_word(mtd));
			if (chip->badblockpos & 0x1)
				bad >>= 8;
			else
				bad &= 0xFF;
		} else {
			chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos,
					page);
			bad = chip->read_byte(mtd);
		}

		if (likely(chip->badblockbits == 8))
			res = bad != 0xFF;
		else
			res = hweight8(bad) < chip->badblockbits;
		ofs += mtd->writesize;
		page = (int)(ofs >> chip->page_shift) & chip->pagemask;
		i++;
	} while (!res && i < 2 && (chip->bbt_options & NAND_BBT_SCAN2NDPAGE));

	if (getchip) {
		chip->select_chip(mtd, -1);
		nand_release_device(mtd);
	}

	return res;
}

/**
 * nand_default_block_markbad - [DEFAULT] mark a block bad via bad block marker
 * @mtd: MTD device structure
 * @ofs: offset from device start
 *
 * This is the default implementation, which can be overridden by a hardware
 * specific driver. It provides the details for writing a bad block marker to a
 * block.
 */
static int nand_default_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	struct mtd_oob_ops ops;
	uint8_t buf[2] = { 0, 0 };
	int ret = 0, res, i = 0;

	ops.datbuf = NULL;
	ops.oobbuf = buf;
	ops.ooboffs = chip->badblockpos;
	if (chip->options & NAND_BUSWIDTH_16) {
		ops.ooboffs &= ~0x01;
		ops.len = ops.ooblen = 2;
	} else {
		ops.len = ops.ooblen = 1;
	}
	ops.mode = MTD_OPS_PLACE_OOB;

	/* Write to first/last page(s) if necessary */
	if (chip->bbt_options & NAND_BBT_SCANLASTPAGE)
		ofs += mtd->erasesize - mtd->writesize;
	do {
		res = nand_do_write_oob(mtd, ofs, &ops);
		if (!ret)
			ret = res;

		i++;
		ofs += mtd->writesize;
	} while ((chip->bbt_options & NAND_BBT_SCAN2NDPAGE) && i < 2);

	return ret;
}

/**
 * nand_block_markbad_lowlevel - mark a block bad
 * @mtd: MTD device structure
 * @ofs: offset from device start
 *
 * This function performs the generic NAND bad block marking steps (i.e., bad
 * block table(s) and/or marker(s)). We only allow the hardware driver to
 * specify how to write bad block markers to OOB (chip->block_markbad).
 *
 * We try operations in the following order:
 *  (1) erase the affected block, to allow OOB marker to be written cleanly
 *  (2) write bad block marker to OOB area of affected block (unless flag
 *      NAND_BBT_NO_OOB_BBM is present)
 *  (3) update the BBT
 * Note that we retain the first error encountered in (2) or (3), finish the
 * procedures, and dump the error in the end.
*/
static int nand_block_markbad_lowlevel(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	int res, ret = 0;

	if (!(chip->bbt_options & NAND_BBT_NO_OOB_BBM)) {
		struct erase_info einfo;

		/* Attempt erase before marking OOB */
		memset(&einfo, 0, sizeof(einfo));
		einfo.mtd = mtd;
		einfo.addr = ofs;
		einfo.len = 1ULL << chip->phys_erase_shift;
		nand_erase_nand(mtd, &einfo, 0);

		/* Write bad block marker to OOB */
		nand_get_device(mtd, FL_WRITING);
		ret = chip->block_markbad(mtd, ofs);
		nand_release_device(mtd);
	}

	/* Mark block bad in BBT */
	if (chip->bbt) {
		res = nand_markbad_bbt(mtd, ofs);
		if (!ret)
			ret = res;
	}

	if (!ret)
		mtd->ecc_stats.badblocks++;

	return ret;
}

/**
 * nand_check_wp - [GENERIC] check if the chip is write protected
 * @mtd: MTD device structure
 *
 * Check, if the device is write protected. The function expects, that the
 * device is already selected.
 */
static int nand_check_wp(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

	/* Broken xD cards report WP despite being writable */
	if (chip->options & NAND_BROKEN_XD)
		return 0;

	/* Check the WP bit */
	chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
	return (chip->read_byte(mtd) & NAND_STATUS_WP) ? 0 : 1;
}

/**
 * nand_block_isreserved - [GENERIC] Check if a block is marked reserved.
 * @mtd: MTD device structure
 * @ofs: offset from device start
 *
 * Check if the block is marked as reserved.
 */
static int nand_block_isreserved(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;

	if (!chip->bbt)
		return 0;
	/* Return info from the table */
	return nand_isreserved_bbt(mtd, ofs);
}

/**
 * nand_block_checkbad - [GENERIC] Check if a block is marked bad
 * @mtd: MTD device structure
 * @ofs: offset from device start
 * @getchip: 0, if the chip is already selected
 * @allowbbt: 1, if its allowed to access the bbt area
 *
 * Check, if the block is bad. Either by reading the bad block table or
 * calling of the scan function.
 */
static int nand_block_checkbad(struct mtd_info *mtd, loff_t ofs, int getchip,
			       int allowbbt)
{
	struct nand_chip *chip = mtd->priv;

	if (!chip->bbt)
		return chip->block_bad(mtd, ofs, getchip);

	/* Return info from the table */
	return nand_isbad_bbt(mtd, ofs, allowbbt);
}

/**
 * panic_nand_wait_ready - [GENERIC] Wait for the ready pin after commands.
 * @mtd: MTD device structure
 * @timeo: Timeout
 *
 * Helper function for nand_wait_ready used when needing to wait in interrupt
 * context.
 */
static void panic_nand_wait_ready(struct mtd_info *mtd, unsigned long timeo)
{
	struct nand_chip *chip = mtd->priv;
	int i;

	/* Wait for the device to get ready */
	for (i = 0; i < timeo; i++) {
		if (chip->dev_ready(mtd))
			break;
		touch_softlockup_watchdog();
		mdelay(1);
	}
}

/* Wait for the ready pin, after a command. The timeout is caught later. */
void nand_wait_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	unsigned long timeo = jiffies + msecs_to_jiffies(20);

	/* 400ms timeout */
	if (in_interrupt() || oops_in_progress)
		return panic_nand_wait_ready(mtd, 400);

	led_trigger_event(nand_led_trigger, LED_FULL);
	/* Wait until command is processed or timeout occurs */
	do {
		if (chip->dev_ready(mtd))
			break;
		touch_softlockup_watchdog();
	} while (time_before(jiffies, timeo));
	led_trigger_event(nand_led_trigger, LED_OFF);
}
EXPORT_SYMBOL_GPL(nand_wait_ready);

/**
 * nand_command - [DEFAULT] Send command to NAND device
 * @mtd: MTD device structure
 * @command: the command to be sent
 * @column: the column address for this command, -1 if none
 * @page_addr: the page address for this command, -1 if none
 *
 * Send command to NAND device. This function is used for small page devices
 * (512 Bytes per page).
 */
static void nand_command(struct mtd_info *mtd, unsigned int command,
			 int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;
	int ctrl = NAND_CTRL_CLE | NAND_CTRL_CHANGE;

	/* Write out the command to the device */
	if (command == NAND_CMD_SEQIN) {
		int readcmd;

		if (column >= mtd->writesize) {
			/* OOB area */
			column -= mtd->writesize;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}
		chip->cmd_ctrl(mtd, readcmd, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
	}
	chip->cmd_ctrl(mtd, command, ctrl);

	/* Address cycle, when necessary */
	ctrl = NAND_CTRL_ALE | NAND_CTRL_CHANGE;
	/* Serially input address */
	if (column != -1) {
		/* Adjust columns for 16 bit buswidth */
		if (chip->options & NAND_BUSWIDTH_16 &&
				!nand_opcode_8bits(command))
			column >>= 1;
		chip->cmd_ctrl(mtd, column, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
	}
	if (page_addr != -1) {
		chip->cmd_ctrl(mtd, page_addr, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
		chip->cmd_ctrl(mtd, page_addr >> 8, ctrl);
		/* One more address cycle for devices > 32MiB */
		if (chip->chipsize > (32 << 20))
			chip->cmd_ctrl(mtd, page_addr >> 16, ctrl);
	}
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * Program and erase have their own busy handlers status and sequential
	 * in needs no delay
	 */
	switch (command) {

	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_STATUS:
		return;

	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		chip->cmd_ctrl(mtd, NAND_CMD_STATUS,
			       NAND_CTRL_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd,
			       NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY))
				;
		return;

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}
	/*
	 * Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine.
	 */
	ndelay(100);

	nand_wait_ready(mtd);
}

/**
 * nand_command_lp - [DEFAULT] Send command to NAND large page device
 * @mtd: MTD device structure
 * @command: the command to be sent
 * @column: the column address for this command, -1 if none
 * @page_addr: the page address for this command, -1 if none
 *
 * Send command to NAND device. This is the version for the new large page
 * devices. We don't have the separate regions as we have in the small page
 * devices. We must emulate NAND_CMD_READOOB to keep the code compatible.
 */
static void nand_command_lp(struct mtd_info *mtd, unsigned int command,
			    int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;

	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	/* Command latch cycle */
	chip->cmd_ctrl(mtd, command, NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if (column != -1 || page_addr != -1) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			if (chip->options & NAND_BUSWIDTH_16 &&
					!nand_opcode_8bits(command))
				column >>= 1;
			chip->cmd_ctrl(mtd, column, ctrl);
			ctrl &= ~NAND_CTRL_CHANGE;
			chip->cmd_ctrl(mtd, column >> 8, ctrl);
		}
		if (page_addr != -1) {
			chip->cmd_ctrl(mtd, page_addr, ctrl);
			chip->cmd_ctrl(mtd, page_addr >> 8,
				       NAND_NCE | NAND_ALE);
			/* One more address cycle for devices > 128MiB */
			if (chip->chipsize > (128 << 20))
				chip->cmd_ctrl(mtd, page_addr >> 16,
					       NAND_NCE | NAND_ALE);
		}
	}
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * Program and erase have their own busy handlers status, sequential
	 * in and status need no delay.
	 */
	switch (command) {

	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
		return;

	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		chip->cmd_ctrl(mtd, NAND_CMD_STATUS,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY))
				;
		return;

	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		chip->cmd_ctrl(mtd, NAND_CMD_RNDOUTSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:
		chip->cmd_ctrl(mtd, NAND_CMD_READSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay.
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}

	/*
	 * Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine.
	 */
	ndelay(100);

	nand_wait_ready(mtd);
}

/**
 * panic_nand_get_device - [GENERIC] Get chip for selected access
 * @chip: the nand chip descriptor
 * @mtd: MTD device structure
 * @new_state: the state which is requested
 *
 * Used when in panic, no locks are taken.
 */
static void panic_nand_get_device(struct nand_chip *chip,
		      struct mtd_info *mtd, int new_state)
{
	/* Hardware controller shared among independent devices */
	chip->controller->active = chip;
	chip->state = new_state;
}

/**
 * nand_get_device - [GENERIC] Get chip for selected access
 * @mtd: MTD device structure
 * @new_state: the state which is requested
 *
 * Get the device and lock it for exclusive access
 */
static int
nand_get_device(struct mtd_info *mtd, int new_state)
{
	struct nand_chip *chip = mtd->priv;
	spinlock_t *lock = &chip->controller->lock;
	wait_queue_head_t *wq = &chip->controller->wq;
	DECLARE_WAITQUEUE(wait, current);
retry:
	spin_lock(lock);

	/* Hardware controller shared among independent devices */
	if (!chip->controller->active)
		chip->controller->active = chip;

	if (chip->controller->active == chip && chip->state == FL_READY) {
		chip->state = new_state;
		spin_unlock(lock);
		return 0;
	}
	if (new_state == FL_PM_SUSPENDED) {
		if (chip->controller->active->state == FL_PM_SUSPENDED) {
			chip->state = FL_PM_SUSPENDED;
			spin_unlock(lock);
			return 0;
		}
	}
	set_current_state(TASK_UNINTERRUPTIBLE);
	add_wait_queue(wq, &wait);
	spin_unlock(lock);
	schedule();
	remove_wait_queue(wq, &wait);
	goto retry;
}

/**
 * panic_nand_wait - [GENERIC] wait until the command is done
 * @mtd: MTD device structure
 * @chip: NAND chip structure
 * @timeo: timeout
 *
 * Wait for command done. This is a helper function for nand_wait used when
 * we are in interrupt context. May happen when in panic and trying to write
 * an oops through mtdoops.
 */
static void panic_nand_wait(struct mtd_info *mtd, struct nand_chip *chip,
			    unsigned long timeo)
{
	int i;
	for (i = 0; i < timeo; i++) {
		if (chip->dev_ready) {
			if (chip->dev_ready(mtd))
				break;
		} else {
			if (chip->read_byte(mtd) & NAND_STATUS_READY)
				break;
		}
		mdelay(1);
	}
}

/**
 * nand_wait - [DEFAULT] wait until the command is done
 * @mtd: MTD device structure
 * @chip: NAND chip structure
 *
 * Wait for command done. This applies to erase and program only. Erase can
 * take up to 400ms and program up to 20ms according to general NAND and
 * SmartMedia specs.
 */
static int nand_wait(struct mtd_info *mtd, struct nand_chip *chip)
{

	int status, state = chip->state;
	unsigned long timeo = (state == FL_ERASING ? 400 : 20);

	led_trigger_event(nand_led_trigger, LED_FULL);

	/*
	 * Apply this short delay always to ensure that we do wait tWB in any
	 * case on any machine.
	 */
	ndelay(100);

	chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);

	if (in_interrupt() || oops_in_progress)
		panic_nand_wait(mtd, chip, timeo);
	else {
		timeo = jiffies + msecs_to_jiffies(timeo);
		while (time_before(jiffies, timeo)) {
			if (chip->dev_ready) {
				if (chip->dev_ready(mtd))
					break;
			} else {
				if (chip->read_byte(mtd) & NAND_STATUS_READY)
					break;
			}
			cond_resched();
		}
	}
	led_trigger_event(nand_led_trigger, LED_OFF);

	status = (int)chip->read_byte(mtd);
	/* This can happen if in case of timeout or buggy dev_ready */
	WARN_ON(!(status & NAND_STATUS_READY));
	return status;
}

/**
 * __nand_unlock - [REPLACEABLE] unlocks specified locked blocks
 * @mtd: mtd info
 * @ofs: offset to start unlock from
 * @len: length to unlock
 * @invert: when = 0, unlock the range of blocks within the lower and
 *                    upper boundary address
 *          when = 1, unlock the range of blocks outside the boundaries
 *                    of the lower and upper boundary address
 *
 * Returs unlock status.
 */
static int __nand_unlock(struct mtd_info *mtd, loff_t ofs,
					uint64_t len, int invert)
{
	int ret = 0;
	int status, page;
	struct nand_chip *chip = mtd->priv;

	/* Submit address of first page to unlock */
	page = ofs >> chip->page_shift;
	chip->cmdfunc(mtd, NAND_CMD_UNLOCK1, -1, page & chip->pagemask);

	/* Submit address of last page to unlock */
	page = (ofs + len) >> chip->page_shift;
	chip->cmdfunc(mtd, NAND_CMD_UNLOCK2, -1,
				(page | invert) & chip->pagemask);

	/* Call wait ready function */
	status = chip->waitfunc(mtd, chip);
	/* See if device thinks it succeeded */
	if (status & NAND_STATUS_FAIL) {
		pr_debug("%s: error status = 0x%08x\n",
					__func__, status);
		ret = -EIO;
	}

	return ret;
}

/**
 * nand_unlock - [REPLACEABLE] unlocks specified locked blocks
 * @mtd: mtd info
 * @ofs: offset to start unlock from
 * @len: length to unlock
 *
 * Returns unlock status.
 */
int nand_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	int ret = 0;
	int chipnr;
	struct nand_chip *chip = mtd->priv;

	pr_debug("%s: start = 0x%012llx, len = %llu\n",
			__func__, (unsigned long long)ofs, len);

	if (check_offs_len(mtd, ofs, len))
		ret = -EINVAL;

	/* Align to last block address if size addresses end of the device */
	if (ofs + len == mtd->size)
		len -= mtd->erasesize;

	nand_get_device(mtd, FL_UNLOCKING);

	/* Shift to get chip number */
	chipnr = ofs >> chip->chip_shift;

	chip->select_chip(mtd, chipnr);

	/*
	 * Reset the chip.
	 * If we want to check the WP through READ STATUS and check the bit 7
	 * we must reset the chip
	 * some operation can also clear the bit 7 of status register
	 * eg. erase/program a locked block
	 */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* Check, if it is write protected */
	if (nand_check_wp(mtd)) {
		pr_debug("%s: device is write protected!\n",
					__func__);
		ret = -EIO;
		goto out;
	}

	ret = __nand_unlock(mtd, ofs, len, 0);

out:
	chip->select_chip(mtd, -1);
	nand_release_device(mtd);

	return ret;
}
EXPORT_SYMBOL(nand_unlock);

/**
 * nand_lock - [REPLACEABLE] locks all blocks present in the device
 * @mtd: mtd info
 * @ofs: offset to start unlock from
 * @len: length to unlock
 *
 * This feature is not supported in many NAND parts. 'Micron' NAND parts do
 * have this feature, but it allows only to lock all blocks, not for specified
 * range for block. Implementing 'lock' feature by making use of 'unlock', for
 * now.
 *
 * Returns lock status.
 */
int nand_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	int ret = 0;
	int chipnr, status, page;
	struct nand_chip *chip = mtd->priv;

	pr_debug("%s: start = 0x%012llx, len = %llu\n",
			__func__, (unsigned long long)ofs, len);

	if (check_offs_len(mtd, ofs, len))
		ret = -EINVAL;

	nand_get_device(mtd, FL_LOCKING);

	/* Shift to get chip number */
	chipnr = ofs >> chip->chip_shift;

	chip->select_chip(mtd, chipnr);

	/*
	 * Reset the chip.
	 * If we want to check the WP through READ STATUS and check the bit 7
	 * we must reset the chip
	 * some operation can also clear the bit 7 of status register
	 * eg. erase/program a locked block
	 */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* Check, if it is write protected */
	if (nand_check_wp(mtd)) {
		pr_debug("%s: device is write protected!\n",
					__func__);
		status = MTD_ERASE_FAILED;
		ret = -EIO;
		goto out;
	}

	/* Submit address of first page to lock */
	page = ofs >> chip->page_shift;
	chip->cmdfunc(mtd, NAND_CMD_LOCK, -1, page & chip->pagemask);

	/* Call wait ready function */
	status = chip->waitfunc(mtd, chip);
	/* See if device thinks it succeeded */
	if (status & NAND_STATUS_FAIL) {
		pr_debug("%s: error status = 0x%08x\n",
					__func__, status);
		ret = -EIO;
		goto out;
	}

	ret = __nand_unlock(mtd, ofs, len, 0x1);

out:
	chip->select_chip(mtd, -1);
	nand_release_device(mtd);

	return ret;
}
EXPORT_SYMBOL(nand_lock);

/**
 * nand_rnd_is_activ - check wether a region of a NAND page requires NAND
 *		       randomizer to be disabled
 * @mtd:	mtd info
 * @page:	NAND page
 * @column:	offset within the page
 * @len:	len of the region
 *
 * Returns 1 if the randomizer should be enabled, 0 if not, or -ERR in case of
 * error.
 *
 * In case of success len will contain the size of the region:
 *  - if the requested region fits in a NAND random region len will not change
 *  - else len will be replaced by the available length within the NAND random
 *    region
 */
int nand_rnd_is_activ(struct mtd_info *mtd, int page, int column, int *len)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_rnd_layout *layout = chip->cur_rnd->layout;
	struct nand_rndfree *range;
	int ret = 1;
	int tmp;
	int i;

	if (!len || *len < 0 || column < 0 ||
	    column + *len > mtd->writesize + mtd->oobsize)
		return -EINVAL;

	if (layout) {
		for (i = 0; i < layout->nranges; i++) {
			range = &layout->ranges[i];
			if (column + *len <= range->offset) {
				break;
			} else if (column >= range->offset + range->length) {
				continue;
			} else if (column < range->offset) {
				tmp = range->offset - column;
				if (*len > tmp)
					*len = tmp;
				break;
			} else {
				tmp = range->offset + range->length - column;
				if (*len > tmp)
					*len = tmp;
				ret = 0;
				break;
			}

		}
	}

	return ret;
}
EXPORT_SYMBOL(nand_rnd_is_activ);

/**
 * nand_page_is_empty - check wether a NAND page contains only FFs
 * @mtd:	mtd info
 * @data:	data buffer
 * @oob:	oob buffer
 *
 * Reads the data stored in the databuf buffer and check if it contains only
 * FFs.
 *
 * Return true if it does else return false.
 */
bool nand_page_is_empty(struct mtd_info *mtd, void *data, void *oob)
{
	u8 *buf;
	int length;
	u32 pattern = 0xffffffff;
	int bitflips = 0;
	int cnt;

	buf = data;
	length = mtd->writesize;
	while (length) {
		cnt = length < sizeof(pattern) ? length : sizeof(pattern);
		if (memcmp(&pattern, buf, cnt)) {
			int i;
			for (i = 0; i < cnt * BITS_PER_BYTE; i++) {
				if (!(buf[i / BITS_PER_BYTE] &
				      (1 << (i % BITS_PER_BYTE)))) {
					bitflips++;
					if (bitflips > mtd->ecc_strength)
						return false;
				}
			}
		}

		buf += sizeof(pattern);
		length -= sizeof(pattern);
	}

	buf = oob;
	length = mtd->oobsize;
	while (length) {
		cnt = length < sizeof(pattern) ? length : sizeof(pattern);
		if (memcmp(&pattern, buf, cnt)) {
			int i;
			for (i = 0; i < cnt * BITS_PER_BYTE; i++) {
				if (!(buf[i / BITS_PER_BYTE] &
				      (1 << (i % BITS_PER_BYTE)))) {
					bitflips++;
					if (bitflips > mtd->ecc_strength)
						return false;
				}
			}
		}

		buf += sizeof(pattern);
		length -= sizeof(pattern);
	}

	return true;
}
EXPORT_SYMBOL(nand_page_is_empty);

/**
 * nand_page_get_status - retrieve page status from the page status table (pst)
 * @mtd:	mtd info
 * @page:	page you want to get status on
 *
 * Return the page status.
 */
int nand_page_get_status(struct mtd_info *mtd, int page)
{
	struct nand_chip *chip = mtd->priv;
	u8 shift = (page % 4) * 2;
	uint64_t offset = page / 4;
	int ret = NAND_PAGE_STATUS_UNKNOWN;

	if (chip->pst)
		ret = (chip->pst[offset] >> shift) & 0x3;

	return ret;
}
EXPORT_SYMBOL(nand_page_get_status);

/**
 * nand_page_set_status - assign page status from in the page status table
 * @mtd:	mtd info
 * @page:	page you want to get status on
 * @status:	new status to assign
 */
void nand_page_set_status(struct mtd_info *mtd, int page,
			  enum nand_page_status status)
{
	struct nand_chip *chip = mtd->priv;
	u8 shift;
	uint64_t offset;

	if (!chip->pst)
		return;

	shift = (page % 4) * 2;
	offset = page / 4;
	chip->pst[offset] &= ~(0x3 << shift);
	chip->pst[offset] |= (status & 0x3) << shift;
}
EXPORT_SYMBOL(nand_page_set_status);

/**
 * nand_pst_create - create a page status table
 * @mtd:	mtd info
 *
 * Allocate a page status table and assign it to the mtd device.
 *
 * Returns 0 in case of success or -ERRNO in case of error.
 */
int nand_pst_create(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

	if (chip->pst)
		return 0;

	chip->pst = kzalloc(mtd->size >>
			    (chip->page_shift + mtd->subpage_sft + 2),
			    GFP_KERNEL);
	if (!chip->pst)
		return -ENOMEM;

	return 0;
}
EXPORT_SYMBOL(nand_pst_create);

/**
 * nand_read_page_raw - [INTERN] read raw page data without ecc
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: buffer to store read data
 * @oob_required: caller requires OOB data read to chip->oob_poi
 * @page: page number to read
 *
 * Not for syndrome calculating ECC controllers, which use a special oob layout.
 */
static int nand_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			      uint8_t *buf, int oob_required, int page)
{
	nand_rnd_config(mtd, page, 0, NAND_RND_READ);
	nand_rnd_read_buf(mtd, buf, mtd->writesize);
	if (oob_required) {
		nand_rnd_config(mtd, page, mtd->writesize, NAND_RND_READ);
		nand_rnd_read_buf(mtd, chip->oob_poi, mtd->oobsize);
	}
	nand_rnd_config(mtd, -1, -1, NAND_RND_READ);

	return 0;
}

/**
 * nand_read_page_raw_syndrome - [INTERN] read raw page data without ecc
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: buffer to store read data
 * @oob_required: caller requires OOB data read to chip->oob_poi
 * @page: page number to read
 *
 * We need a special oob layout and handling even when OOB isn't used.
 */
static int nand_read_page_raw_syndrome(struct mtd_info *mtd,
				       struct nand_chip *chip, uint8_t *buf,
				       int oob_required, int page)
{
	int eccsize = chip->cur_ecc->size;
	int eccbytes = chip->cur_ecc->bytes;
	uint8_t *oob = chip->oob_poi;
	int steps, size;
	int column = 0;

	for (steps = chip->cur_ecc->steps; steps > 0; steps--) {
		nand_rnd_config(mtd, page, column, NAND_RND_READ);
		nand_rnd_read_buf(mtd, buf, eccsize);
		buf += eccsize;
		column += eccsize;

		if (chip->cur_ecc->prepad) {
			nand_rnd_config(mtd, page, column, NAND_RND_READ);
			nand_rnd_read_buf(mtd, oob, chip->cur_ecc->prepad);
			oob += chip->cur_ecc->prepad;
			column += chip->cur_ecc->prepad;
		}

		nand_rnd_config(mtd, page, column, NAND_RND_READ);
		nand_rnd_read_buf(mtd, oob, eccbytes);
		oob += eccbytes;
		column += eccbytes;

		if (chip->cur_ecc->postpad) {
			nand_rnd_config(mtd, page, column, NAND_RND_READ);
			nand_rnd_read_buf(mtd, oob, chip->cur_ecc->postpad);
			oob += chip->cur_ecc->postpad;
			column += chip->cur_ecc->postpad;
		}
	}

	size = mtd->oobsize - (oob - chip->oob_poi);
	if (size) {
		nand_rnd_config(mtd, page, column, NAND_RND_READ);
		nand_rnd_read_buf(mtd, oob, size);
	}
	nand_rnd_config(mtd, -1, -1, NAND_RND_READ);

	return 0;
}

/**
 * nand_read_page_swecc - [REPLACEABLE] software ECC based page read function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: buffer to store read data
 * @oob_required: caller requires OOB data read to chip->oob_poi
 * @page: page number to read
 */
static int nand_read_page_swecc(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_required, int page)
{
	int i, eccsize = chip->cur_ecc->size;
	int eccbytes = chip->cur_ecc->bytes;
	int eccsteps = chip->cur_ecc->steps;
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->cur_ecc->layout->eccpos;
	unsigned int max_bitflips = 0;

	chip->cur_ecc->read_page_raw(mtd, chip, buf, 1, page);

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
		chip->cur_ecc->calculate(mtd, p, &ecc_calc[i]);

	for (i = 0; i < chip->cur_ecc->total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	eccsteps = chip->cur_ecc->steps;
	p = buf;

	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		stat = chip->cur_ecc->correct(mtd, p, &ecc_code[i],
					      &ecc_calc[i]);
		if (stat < 0) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += stat;
			max_bitflips = max_t(unsigned int, max_bitflips, stat);
		}
	}
	return max_bitflips;
}

/**
 * nand_read_subpage - [REPLACEABLE] ECC based sub-page read function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @data_offs: offset of requested data within the page
 * @readlen: data length
 * @bufpoi: buffer to store read data
 * @page: page number to read
 */
static int nand_read_subpage(struct mtd_info *mtd, struct nand_chip *chip,
			uint32_t data_offs, uint32_t readlen, uint8_t *bufpoi,
			int page)
{
	int start_step, end_step, num_steps;
	uint32_t *eccpos = chip->cur_ecc->layout->eccpos;
	uint8_t *p;
	int data_col_addr, i, gaps = 0;
	int datafrag_len, eccfrag_len, aligned_len, aligned_pos;
	int busw = (chip->options & NAND_BUSWIDTH_16) ? 2 : 1;
	int index;
	unsigned int max_bitflips = 0;

	/* Column address within the page aligned to ECC size (256bytes) */
	start_step = data_offs / chip->cur_ecc->size;
	end_step = (data_offs + readlen - 1) / chip->cur_ecc->size;
	num_steps = end_step - start_step + 1;
	index = start_step * chip->cur_ecc->bytes;

	/* Data size aligned to ECC ecc.size */
	datafrag_len = num_steps * chip->cur_ecc->size;
	eccfrag_len = num_steps * chip->cur_ecc->bytes;

	data_col_addr = start_step * chip->cur_ecc->size;
	/* If we read not a page aligned data */
	if (data_col_addr != 0)
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, data_col_addr, -1);

	p = bufpoi + data_col_addr;
	nand_rnd_config(mtd, -1, data_col_addr, NAND_RND_READ);
	nand_rnd_read_buf(mtd, p, datafrag_len);

	/* Calculate ECC */
	for (i = 0; i < eccfrag_len;
	     i += chip->cur_ecc->bytes, p += chip->cur_ecc->size)
		chip->cur_ecc->calculate(mtd, p, &chip->buffers->ecccalc[i]);

	/*
	 * The performance is faster if we position offsets according to
	 * ecc.pos. Let's make sure that there are no gaps in ECC positions.
	 */
	for (i = 0; i < eccfrag_len - 1; i++) {
		if (eccpos[i + index] + 1 != eccpos[i + index + 1]) {
			gaps = 1;
			break;
		}
	}
	if (gaps) {
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, mtd->writesize, -1);
		nand_rnd_config(mtd, -1, mtd->writesize, NAND_RND_READ);
		nand_rnd_read_buf(mtd, chip->oob_poi, mtd->oobsize);
	} else {
		/*
		 * Send the command to read the particular ECC bytes take care
		 * about buswidth alignment in read_buf.
		 */
		index = start_step * chip->cur_ecc->bytes;
		aligned_pos = eccpos[index] & ~(busw - 1);
		aligned_len = eccfrag_len;
		if (eccpos[index] & (busw - 1))
			aligned_len++;
		if (eccpos[index + (num_steps * chip->cur_ecc->bytes)] &
		    (busw - 1))
			aligned_len++;

		chip->cmdfunc(mtd, NAND_CMD_RNDOUT,
					mtd->writesize + aligned_pos, -1);
		nand_rnd_config(mtd, -1, mtd->writesize + aligned_pos, NAND_RND_READ);
		nand_rnd_read_buf(mtd, &chip->oob_poi[aligned_pos], aligned_len);
	}

	for (i = 0; i < eccfrag_len; i++)
		chip->buffers->ecccode[i] = chip->oob_poi[eccpos[i + index]];

	p = bufpoi + data_col_addr;
	for (i = 0; i < eccfrag_len;
	     i += chip->cur_ecc->bytes, p += chip->cur_ecc->size) {
		int stat;

		stat = chip->cur_ecc->correct(mtd, p,
					      &chip->buffers->ecccode[i],
					      &chip->buffers->ecccalc[i]);
		if (stat < 0) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += stat;
			max_bitflips = max_t(unsigned int, max_bitflips, stat);
		}
	}
	nand_rnd_config(mtd, -1, -1, NAND_RND_READ);
	return max_bitflips;
}

/**
 * nand_read_page_hwecc - [REPLACEABLE] hardware ECC based page read function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: buffer to store read data
 * @oob_required: caller requires OOB data read to chip->oob_poi
 * @page: page number to read
 *
 * Not for syndrome calculating ECC controllers which need a special oob layout.
 */
static int nand_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_required, int page)
{
	int i, eccsize = chip->cur_ecc->size;
	int eccbytes = chip->cur_ecc->bytes;
	int eccsteps = chip->cur_ecc->steps;
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->cur_ecc->layout->eccpos;
	unsigned int max_bitflips = 0;
	int column = 0;

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		chip->cur_ecc->hwctl(mtd, NAND_ECC_READ);
		nand_rnd_config(mtd, page, column, NAND_RND_READ);
		nand_rnd_read_buf(mtd, p, eccsize);
		chip->cur_ecc->calculate(mtd, p, &ecc_calc[i]);
		column += eccsize;
	}
	nand_rnd_config(mtd, page, column, NAND_RND_READ);
	nand_rnd_read_buf(mtd, chip->oob_poi, mtd->oobsize);

	for (i = 0; i < chip->cur_ecc->total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	eccsteps = chip->cur_ecc->steps;
	p = buf;

	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		stat = chip->cur_ecc->correct(mtd, p, &ecc_code[i],
					      &ecc_calc[i]);
		if (stat < 0) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += stat;
			max_bitflips = max_t(unsigned int, max_bitflips, stat);
		}
	}
	nand_rnd_config(mtd, -1, -1, NAND_RND_READ);
	return max_bitflips;
}

/**
 * nand_read_page_hwecc_oob_first - [REPLACEABLE] hw ecc, read oob first
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: buffer to store read data
 * @oob_required: caller requires OOB data read to chip->oob_poi
 * @page: page number to read
 *
 * Hardware ECC for large page chips, require OOB to be read first. For this
 * ECC mode, the write_page method is re-used from ECC_HW. These methods
 * read/write ECC from the OOB area, unlike the ECC_HW_SYNDROME support with
 * multiple ECC steps, follows the "infix ECC" scheme and reads/writes ECC from
 * the data area, by overwriting the NAND manufacturer bad block markings.
 */
static int nand_read_page_hwecc_oob_first(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	int i, eccsize = chip->cur_ecc->size;
	int eccbytes = chip->cur_ecc->bytes;
	int eccsteps = chip->cur_ecc->steps;
	uint8_t *p = buf;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->cur_ecc->layout->eccpos;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	unsigned int max_bitflips = 0;
	int column = 0;

	/* Read the OOB area first */
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	nand_rnd_config(mtd, page, mtd->writesize, NAND_RND_READ);
	nand_rnd_read_buf(mtd, chip->oob_poi, mtd->oobsize);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
	column = 0;

	for (i = 0; i < chip->cur_ecc->total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		chip->cur_ecc->hwctl(mtd, NAND_ECC_READ);
		nand_rnd_config(mtd, page, column, NAND_RND_READ);
		nand_rnd_read_buf(mtd, p, eccsize);
		chip->cur_ecc->calculate(mtd, p, &ecc_calc[i]);

		stat = chip->cur_ecc->correct(mtd, p, &ecc_code[i], NULL);
		if (stat < 0) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += stat;
			max_bitflips = max_t(unsigned int, max_bitflips, stat);
		}
	}
	nand_rnd_config(mtd, -1, -1, NAND_RND_READ);
	return max_bitflips;
}

/**
 * nand_read_page_syndrome - [REPLACEABLE] hardware ECC syndrome based page read
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: buffer to store read data
 * @oob_required: caller requires OOB data read to chip->oob_poi
 * @page: page number to read
 *
 * The hw generator calculates the error syndrome automatically. Therefore we
 * need a special oob layout and handling.
 */
static int nand_read_page_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
				   uint8_t *buf, int oob_required, int page)
{
	int i, eccsize = chip->cur_ecc->size;
	int eccbytes = chip->cur_ecc->bytes;
	int eccsteps = chip->cur_ecc->steps;
	uint8_t *p = buf;
	uint8_t *oob = chip->oob_poi;
	unsigned int max_bitflips = 0;
	int column = 0;

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		chip->cur_ecc->hwctl(mtd, NAND_ECC_READ);
		nand_rnd_config(mtd, page, column, NAND_RND_READ);
		nand_rnd_read_buf(mtd, p, eccsize);
		column += eccsize;

		if (chip->cur_ecc->prepad) {
			nand_rnd_config(mtd, page, column, NAND_RND_READ);
			nand_rnd_read_buf(mtd, oob, chip->cur_ecc->prepad);
			oob += chip->cur_ecc->prepad;
		}

		chip->cur_ecc->hwctl(mtd, NAND_ECC_READSYN);
		nand_rnd_config(mtd, page, column, NAND_RND_READ);
		nand_rnd_read_buf(mtd, oob, eccbytes);
		column += eccbytes;

		stat = chip->cur_ecc->correct(mtd, p, oob, NULL);

		if (stat < 0) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += stat;
			max_bitflips = max_t(unsigned int, max_bitflips, stat);
		}

		oob += eccbytes;

		if (chip->cur_ecc->postpad) {
			nand_rnd_config(mtd, page, column, NAND_RND_READ);
			nand_rnd_read_buf(mtd, oob, chip->cur_ecc->postpad);
			column += chip->cur_ecc->postpad;
			oob += chip->cur_ecc->postpad;
		}
	}

	/* Calculate remaining oob bytes */
	i = mtd->oobsize - (oob - chip->oob_poi);
	if (i) {
		nand_rnd_config(mtd, page, column, NAND_RND_READ);
		nand_rnd_read_buf(mtd, oob, i);
	}
	nand_rnd_config(mtd, -1, -1, NAND_RND_READ);

	return max_bitflips;
}

/**
 * nand_transfer_oob - [INTERN] Transfer oob to client buffer
 * @mtd: mtd structure
 * @oob: oob destination address
 * @ops: oob ops structure
 * @len: size of oob to transfer
 */
static uint8_t *nand_transfer_oob(struct mtd_info *mtd, uint8_t *oob,
				  struct mtd_oob_ops *ops, size_t len)
{
	struct nand_chip *chip = mtd->priv;

	switch (ops->mode) {

	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_RAW:
		memcpy(oob, chip->oob_poi + ops->ooboffs, len);
		return oob + len;

	case MTD_OPS_AUTO_OOB: {
		struct nand_oobfree *free = chip->cur_ecc->layout->oobfree;
		uint32_t boffs = 0, roffs = ops->ooboffs;
		size_t bytes = 0;

		for (; free->length && len; free++, len -= bytes) {
			/* Read request not from offset 0? */
			if (unlikely(roffs)) {
				if (roffs >= free->length) {
					roffs -= free->length;
					continue;
				}
				boffs = free->offset + roffs;
				bytes = min_t(size_t, len,
					      (free->length - roffs));
				roffs = 0;
			} else {
				bytes = min_t(size_t, len, free->length);
				boffs = free->offset;
			}
			memcpy(oob, chip->oob_poi + boffs, bytes);
			oob += bytes;
		}
		return oob;
	}
	default:
		BUG();
	}
	return NULL;
}

/**
 * nand_setup_read_retry - [INTERN] Set the READ RETRY mode
 * @mtd: MTD device structure
 * @retry_mode: the retry mode to use
 *
 * Some vendors supply a special command to shift the Vt threshold, to be used
 * when there are too many bitflips in a page (i.e., ECC error). After setting
 * a new threshold, the host should retry reading the page.
 */
static int nand_setup_read_retry(struct mtd_info *mtd, int retry_mode)
{
	struct nand_chip *chip = mtd->priv;

	pr_debug("setting READ RETRY mode %d\n", retry_mode);

	if (retry_mode >= chip->read_retries)
		return -EINVAL;

	if (!chip->setup_read_retry)
		return -EOPNOTSUPP;

	return chip->setup_read_retry(mtd, retry_mode);
}

/**
 * nand_do_read_ops - [INTERN] Read data with ECC
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob ops structure
 *
 * Internal function. Called with chip held.
 */
static int nand_do_read_ops(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops)
{
	int chipnr, page, realpage, col, bytes, aligned, oob_required;
	struct nand_chip *chip = mtd->priv;
	int ret = 0;
	uint32_t readlen = ops->len;
	uint32_t oobreadlen = ops->ooblen;
	uint32_t max_oobsize = ops->mode == MTD_OPS_AUTO_OOB ?
		mtd->oobavail : mtd->oobsize;

	uint8_t *bufpoi, *oob, *buf;
	int use_bufpoi;
	unsigned int max_bitflips = 0;
	int retry_mode = 0;
	bool ecc_fail = false;

	chipnr = (int)(from >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	realpage = (int)(from >> chip->page_shift);
	page = realpage & chip->pagemask;

	col = (int)(from & (mtd->writesize - 1));

	buf = ops->datbuf;
	oob = ops->oobbuf;
	oob_required = oob ? 1 : 0;

	while (1) {
		unsigned int ecc_failures = mtd->ecc_stats.failed;

		bytes = min(mtd->writesize - col, readlen);
		aligned = (bytes == mtd->writesize);

		if (!aligned)
			use_bufpoi = 1;
		else if (chip->options & NAND_USE_BOUNCE_BUFFER)
			use_bufpoi = !virt_addr_valid(buf);
		else
			use_bufpoi = 0;

		/* Is the current page in the buffer? */
		if (realpage != chip->pagebuf || oob) {
			bufpoi = use_bufpoi ? chip->buffers->databuf : buf;

			if (use_bufpoi && aligned)
				pr_debug("%s: using read bounce buffer for buf@%p\n",
						 __func__, buf);

read_retry:
			chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);

			/*
			 * Now read the page into the buffer.  Absent an error,
			 * the read methods return max bitflips per ecc step.
			 */
			nand_rnd_config(mtd, page, -1, NAND_RND_READ);
			if (unlikely(ops->mode == MTD_OPS_RAW))
				ret = chip->cur_ecc->read_page_raw(mtd, chip,
								bufpoi,
								oob_required,
								page);
			else if (!aligned && NAND_HAS_SUBPAGE_READ(chip) &&
				 !oob)
				ret = chip->cur_ecc->read_subpage(mtd, chip,
								  col, bytes,
								  bufpoi,
								  page);
			else
				ret = chip->cur_ecc->read_page(mtd, chip,
							       bufpoi,
							       oob_required,
							       page);
			nand_rnd_config(mtd, -1, -1, NAND_RND_READ);

			if (ret < 0) {
				if (use_bufpoi)
					/* Invalidate page cache */
					chip->pagebuf = -1;
				break;
			}

			max_bitflips = max_t(unsigned int, max_bitflips, ret);

			/* Transfer not aligned data */
			if (use_bufpoi) {
				if (!NAND_HAS_SUBPAGE_READ(chip) && !oob &&
				    !(mtd->ecc_stats.failed - ecc_failures) &&
				    (ops->mode != MTD_OPS_RAW)) {
					chip->pagebuf = realpage;
					chip->pagebuf_bitflips = ret;
				} else {
					/* Invalidate page cache */
					chip->pagebuf = -1;
				}
				memcpy(buf, chip->buffers->databuf + col, bytes);
			}

			if (unlikely(oob)) {
				int toread = min(oobreadlen, max_oobsize);

				if (toread) {
					oob = nand_transfer_oob(mtd, oob, ops,
								toread);
					oobreadlen -= toread;
				}
			}

			if (chip->options & NAND_NEED_READRDY) {
				/* Apply delay or wait for ready/busy pin */
				if (!chip->dev_ready)
					udelay(chip->chip_delay);
				else
					nand_wait_ready(mtd);
			}

			if (mtd->ecc_stats.failed - ecc_failures) {
				if (retry_mode + 1 < chip->read_retries) {
					retry_mode++;
					ret = nand_setup_read_retry(mtd,
							retry_mode);
					if (ret < 0)
						break;

					/* Reset failures; retry */
					mtd->ecc_stats.failed = ecc_failures;
					goto read_retry;
				} else {
					/* No more retry modes; real failure */
					ecc_fail = true;
				}
			}

			buf += bytes;
		} else {
			memcpy(buf, chip->buffers->databuf + col, bytes);
			buf += bytes;
			max_bitflips = max_t(unsigned int, max_bitflips,
					     chip->pagebuf_bitflips);
		}

		readlen -= bytes;

		/* Reset to retry mode 0 */
		if (retry_mode) {
			ret = nand_setup_read_retry(mtd, 0);
			if (ret < 0)
				break;
			retry_mode = 0;
		}

		if (!readlen)
			break;

		/* For subsequent reads align to page boundary */
		col = 0;
		/* Increment page address */
		realpage++;

		page = realpage & chip->pagemask;
		/* Check, if we cross a chip boundary */
		if (!page) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}
	}
	chip->select_chip(mtd, -1);

	ops->retlen = ops->len - (size_t) readlen;
	if (oob)
		ops->oobretlen = ops->ooblen - oobreadlen;

	if (ret < 0)
		return ret;

	if (ecc_fail)
		return -EBADMSG;

	return max_bitflips;
}

/**
 * nand_read - [MTD Interface] MTD compatibility function for nand_do_read_ecc
 * @mtd: MTD device structure
 * @from: offset to read from
 * @len: number of bytes to read
 * @retlen: pointer to variable to store the number of read bytes
 * @buf: the databuffer to put data
 *
 * Get hold of the chip and call nand_do_read.
 */
static int nand_read(struct mtd_info *mtd, loff_t from, size_t len,
		     size_t *retlen, uint8_t *buf)
{
	struct mtd_oob_ops ops;
	int ret;

	nand_get_device(mtd, FL_READING);
	ops.len = len;
	ops.datbuf = buf;
	ops.oobbuf = NULL;
	ops.mode = MTD_OPS_PLACE_OOB;
	ret = nand_do_read_ops(mtd, from, &ops);
	*retlen = ops.retlen;
	nand_release_device(mtd);
	return ret;
}

/**
 * nand_part_read - [MTD Interface] MTD compatibility function for nand_do_read_ecc
 * @mtd: MTD device structure
 * @from: offset to read from
 * @len: number of bytes to read
 * @retlen: pointer to variable to store the number of read bytes
 * @buf: the databuffer to put data
 *
 * Get hold of the chip and call nand_do_read.
 */
static int nand_part_read(struct mtd_info *mtd, loff_t from, size_t len,
			  size_t *retlen, uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_part *part = to_nand_part(mtd);
	struct mtd_oob_ops ops;
	int ret;

	from += part->offset;
	nand_get_device(part->master, FL_READING);
	if (part->ecc)
		chip->cur_ecc = part->ecc;
	if (part->rnd)
		chip->cur_rnd = part->rnd;
	ops.len = len;
	ops.datbuf = buf;
	ops.oobbuf = NULL;
	ops.mode = MTD_OPS_PLACE_OOB;
	ret = nand_do_read_ops(part->master, from, &ops);
	*retlen = ops.retlen;
	chip->cur_rnd = &chip->rnd;
	chip->cur_ecc = &chip->ecc;
	nand_release_device(part->master);
	return ret;
}

/**
 * nand_read_oob_std - [REPLACEABLE] the most common OOB data read function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @page: page number to read
 */
static int nand_read_oob_std(struct mtd_info *mtd, struct nand_chip *chip,
			     int page)
{
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	nand_rnd_config(mtd, page, mtd->writesize, NAND_RND_READ);
	nand_rnd_read_buf(mtd, chip->oob_poi, mtd->oobsize);
	nand_rnd_config(mtd, -1, -1, NAND_RND_READ);
	return 0;
}

/**
 * nand_read_oob_syndrome - [REPLACEABLE] OOB data read function for HW ECC
 *			    with syndromes
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @page: page number to read
 */
static int nand_read_oob_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
				  int page)
{
	uint8_t *buf = chip->oob_poi;
	int length = mtd->oobsize;
	int chunk = chip->cur_ecc->bytes + chip->cur_ecc->prepad +
		    chip->cur_ecc->postpad;
	int eccsize = chip->cur_ecc->size;
	uint8_t *bufpoi = buf;
	int i, toread, sndrnd = 0, pos = eccsize;

	chip->cmdfunc(mtd, NAND_CMD_READ0, chip->cur_ecc->size, page);
	for (i = 0; i < chip->cur_ecc->steps; i++) {
		if (sndrnd) {
			pos = eccsize + i * (eccsize + chunk);
			if (mtd->writesize > 512)
				chip->cmdfunc(mtd, NAND_CMD_RNDOUT, pos, -1);
			else
				chip->cmdfunc(mtd, NAND_CMD_READ0, pos, page);
		} else
			sndrnd = 1;
		toread = min_t(int, length, chunk);
		nand_rnd_config(mtd, page, pos, NAND_RND_READ);
		nand_rnd_read_buf(mtd, bufpoi, toread);
		bufpoi += toread;
		length -= toread;
	}
	if (length > 0) {
		pos = mtd->writesize + mtd->oobsize - length;
		nand_rnd_config(mtd, page, pos, NAND_RND_READ);
		nand_rnd_read_buf(mtd, bufpoi, length);
	}
	nand_rnd_config(mtd, -1, -1, NAND_RND_READ);

	return 0;
}

/**
 * nand_write_oob_std - [REPLACEABLE] the most common OOB data write function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @page: page number to write
 */
static int nand_write_oob_std(struct mtd_info *mtd, struct nand_chip *chip,
			      int page)
{
	int status = 0;
	const uint8_t *buf = chip->oob_poi;
	int length = mtd->oobsize;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, mtd->writesize, page);
	nand_rnd_config(mtd, page, mtd->writesize, NAND_RND_WRITE);
	nand_rnd_write_buf(mtd, buf, length);
	nand_rnd_config(mtd, -1, -1, NAND_RND_WRITE);
	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

/**
 * nand_write_oob_syndrome - [REPLACEABLE] OOB data write function for HW ECC
 *			     with syndrome - only for large page flash
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @page: page number to write
 */
static int nand_write_oob_syndrome(struct mtd_info *mtd,
				   struct nand_chip *chip, int page)
{
	int chunk = chip->cur_ecc->bytes + chip->cur_ecc->prepad +
		    chip->cur_ecc->postpad;
	int eccsize = chip->cur_ecc->size, length = mtd->oobsize;
	int i, len, pos, status = 0, sndcmd = 0, steps = chip->cur_ecc->steps;
	const uint8_t *bufpoi = chip->oob_poi;

	/*
	 * data-ecc-data-ecc ... ecc-oob
	 * or
	 * data-pad-ecc-pad-data-pad .... ecc-pad-oob
	 */
	if (!chip->cur_ecc->prepad && !chip->cur_ecc->postpad) {
		pos = steps * (eccsize + chunk);
		steps = 0;
	} else
		pos = eccsize;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, pos, page);
	for (i = 0; i < steps; i++) {
		if (sndcmd) {
			if (mtd->writesize <= 512) {
				uint32_t fill = 0xFFFFFFFF;

				len = eccsize;
				while (len > 0) {
					int num = min_t(int, len, 4);
					chip->write_buf(mtd, (uint8_t *)&fill,
							num);
					len -= num;
				}
			} else {
				pos = eccsize + i * (eccsize + chunk);
				chip->cmdfunc(mtd, NAND_CMD_RNDIN, pos, -1);
			}
		} else
			sndcmd = 1;
		len = min_t(int, length, chunk);
		nand_rnd_config(mtd, page, pos, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, bufpoi, len);
		bufpoi += len;
		length -= len;
	}
	if (length > 0) {
		pos = mtd->writesize + mtd->oobsize - length;
		nand_rnd_config(mtd, page, pos, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, bufpoi, length);
	}

	nand_rnd_config(mtd, -1, -1, NAND_RND_WRITE);

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

/**
 * nand_do_read_oob - [INTERN] NAND read out-of-band
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob operations description structure
 *
 * NAND read out-of-band data from the spare area.
 */
static int nand_do_read_oob(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops)
{
	int page, realpage, chipnr;
	struct nand_chip *chip = mtd->priv;
	struct mtd_ecc_stats stats;
	int readlen = ops->ooblen;
	int len;
	uint8_t *buf = ops->oobbuf;
	int ret = 0;

	pr_debug("%s: from = 0x%08Lx, len = %i\n",
			__func__, (unsigned long long)from, readlen);

	stats = mtd->ecc_stats;

	if (ops->mode == MTD_OPS_AUTO_OOB)
		len = chip->cur_ecc->layout->oobavail;
	else
		len = mtd->oobsize;

	if (unlikely(ops->ooboffs >= len)) {
		pr_debug("%s: attempt to start read outside oob\n",
				__func__);
		return -EINVAL;
	}

	/* Do not allow reads past end of device */
	if (unlikely(from >= mtd->size ||
		     ops->ooboffs + readlen > ((mtd->size >> chip->page_shift) -
					(from >> chip->page_shift)) * len)) {
		pr_debug("%s: attempt to read beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	chipnr = (int)(from >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	/* Shift to get page */
	realpage = (int)(from >> chip->page_shift);
	page = realpage & chip->pagemask;

	while (1) {
		if (ops->mode == MTD_OPS_RAW)
			ret = chip->cur_ecc->read_oob_raw(mtd, chip, page);
		else
			ret = chip->cur_ecc->read_oob(mtd, chip, page);

		if (ret < 0)
			break;

		len = min(len, readlen);
		buf = nand_transfer_oob(mtd, buf, ops, len);

		if (chip->options & NAND_NEED_READRDY) {
			/* Apply delay or wait for ready/busy pin */
			if (!chip->dev_ready)
				udelay(chip->chip_delay);
			else
				nand_wait_ready(mtd);
		}

		readlen -= len;
		if (!readlen)
			break;

		/* Increment page address */
		realpage++;

		page = realpage & chip->pagemask;
		/* Check, if we cross a chip boundary */
		if (!page) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}
	}
	chip->select_chip(mtd, -1);

	ops->oobretlen = ops->ooblen - readlen;

	if (ret < 0)
		return ret;

	if (mtd->ecc_stats.failed - stats.failed)
		return -EBADMSG;

	return  mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0;
}

/**
 * nand_read_oob - [MTD Interface] NAND read data and/or out-of-band
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob operation description structure
 *
 * NAND read data and/or out-of-band data.
 */
static int nand_read_oob(struct mtd_info *mtd, loff_t from,
			 struct mtd_oob_ops *ops)
{
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow reads past end of device */
	if (ops->datbuf && (from + ops->len) > mtd->size) {
		pr_debug("%s: attempt to read beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	nand_get_device(mtd, FL_READING);

	switch (ops->mode) {
	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_AUTO_OOB:
	case MTD_OPS_RAW:
		break;

	default:
		goto out;
	}

	if (!ops->datbuf)
		ret = nand_do_read_oob(mtd, from, ops);
	else
		ret = nand_do_read_ops(mtd, from, ops);

out:
	nand_release_device(mtd);
	return ret;
}

/**
 * nand_part_read_oob - [MTD Interface] NAND read data and/or out-of-band
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob operation description structure
 *
 * NAND read data and/or out-of-band data.
 */
static int nand_part_read_oob(struct mtd_info *mtd, loff_t from,
			 struct mtd_oob_ops *ops)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_part *part = to_nand_part(mtd);
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow reads past end of device */
	if (ops->datbuf && (from + ops->len) > mtd->size) {
		pr_debug("%s: attempt to read beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	from += part->offset;
	nand_get_device(part->master, FL_READING);
	if (part->ecc)
		chip->cur_ecc = part->ecc;
	if (part->rnd)
		chip->cur_rnd = part->rnd;

	switch (ops->mode) {
	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_AUTO_OOB:
	case MTD_OPS_RAW:
		break;

	default:
		goto out;
	}

	if (!ops->datbuf)
		ret = nand_do_read_oob(part->master, from, ops);
	else
		ret = nand_do_read_ops(part->master, from, ops);

out:
	chip->cur_rnd = &chip->rnd;
	chip->cur_ecc = &chip->ecc;
	nand_release_device(part->master);
	return ret;
}


/**
 * nand_write_page_raw - [INTERN] raw page write function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: data buffer
 * @oob_required: must write chip->oob_poi to OOB
 *
 * Not for syndrome calculating ECC controllers, which use a special oob layout.
 */
static int nand_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				const uint8_t *buf, int oob_required)
{
	nand_rnd_write_buf(mtd, buf, mtd->writesize);
	if (oob_required) {
		nand_rnd_config(mtd, -1, mtd->writesize, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, chip->oob_poi, mtd->oobsize);
	}

	return 0;
}

/**
 * nand_write_page_raw_syndrome - [INTERN] raw page write function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: data buffer
 * @oob_required: must write chip->oob_poi to OOB
 *
 * We need a special oob layout and handling even when ECC isn't checked.
 */
static int nand_write_page_raw_syndrome(struct mtd_info *mtd,
					struct nand_chip *chip,
					const uint8_t *buf, int oob_required)
{
	int eccsize = chip->cur_ecc->size;
	int eccbytes = chip->cur_ecc->bytes;
	uint8_t *oob = chip->oob_poi;
	int steps, size;
	int column = 0;

	for (steps = chip->cur_ecc->steps; steps > 0; steps--) {
		nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, buf, eccsize);
		buf += eccsize;
		column += eccsize;

		if (chip->cur_ecc->prepad) {
			nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
			nand_rnd_write_buf(mtd, oob, chip->cur_ecc->prepad);
			oob += chip->cur_ecc->prepad;
			column += chip->cur_ecc->prepad;
		}

		nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, oob, eccbytes);
		oob += eccbytes;
		column += eccbytes;

		if (chip->cur_ecc->postpad) {
			nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
			nand_rnd_write_buf(mtd, oob, chip->cur_ecc->postpad);
			oob += chip->cur_ecc->postpad;
			column += chip->cur_ecc->postpad;
		}
	}

	size = mtd->oobsize - (oob - chip->oob_poi);
	if (size) {
		nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, oob, size);
	}

	return 0;
}
/**
 * nand_write_page_swecc - [REPLACEABLE] software ECC based page write function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: data buffer
 * @oob_required: must write chip->oob_poi to OOB
 */
static int nand_write_page_swecc(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf, int oob_required)
{
	int i, eccsize = chip->cur_ecc->size;
	int eccbytes = chip->cur_ecc->bytes;
	int eccsteps = chip->cur_ecc->steps;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	const uint8_t *p = buf;
	uint32_t *eccpos = chip->cur_ecc->layout->eccpos;

	/* Software ECC calculation */
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
		chip->cur_ecc->calculate(mtd, p, &ecc_calc[i]);

	for (i = 0; i < chip->cur_ecc->total; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];

	return chip->cur_ecc->write_page_raw(mtd, chip, buf, 1);
}

/**
 * nand_write_page_hwecc - [REPLACEABLE] hardware ECC based page write function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: data buffer
 * @oob_required: must write chip->oob_poi to OOB
 */
static int nand_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf, int oob_required)
{
	int i, eccsize = chip->cur_ecc->size;
	int eccbytes = chip->cur_ecc->bytes;
	int eccsteps = chip->cur_ecc->steps;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	const uint8_t *p = buf;
	uint32_t *eccpos = chip->cur_ecc->layout->eccpos;
	int column = 0;

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		chip->cur_ecc->hwctl(mtd, NAND_ECC_WRITE);
		nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, p, eccsize);
		chip->cur_ecc->calculate(mtd, p, &ecc_calc[i]);
		column += eccsize;
	}

	for (i = 0; i < chip->cur_ecc->total; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];

	nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
	nand_rnd_write_buf(mtd, chip->oob_poi, mtd->oobsize);

	return 0;
}


/**
 * nand_write_subpage_hwecc - [REPLACABLE] hardware ECC based subpage write
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @offset:	column address of subpage within the page
 * @data_len:	data length
 * @buf:	data buffer
 * @oob_required: must write chip->oob_poi to OOB
 */
static int nand_write_subpage_hwecc(struct mtd_info *mtd,
				struct nand_chip *chip, uint32_t offset,
				uint32_t data_len, const uint8_t *buf,
				int oob_required)
{
	uint8_t *oob_buf  = chip->oob_poi;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	int ecc_size      = chip->cur_ecc->size;
	int ecc_bytes     = chip->cur_ecc->bytes;
	int ecc_steps     = chip->cur_ecc->steps;
	uint32_t *eccpos  = chip->cur_ecc->layout->eccpos;
	uint32_t start_step = offset / ecc_size;
	uint32_t end_step   = (offset + data_len - 1) / ecc_size;
	int oob_bytes       = mtd->oobsize / ecc_steps;
	int step, i;

	for (step = 0; step < ecc_steps; step++) {
		/* configure controller for WRITE access */
		chip->cur_ecc->hwctl(mtd, NAND_ECC_WRITE);

		/* write data (untouched subpages already masked by 0xFF) */
		nand_rnd_config(mtd, -1, offset, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, buf, ecc_size);
		offset += ecc_size;

		/* mask ECC of un-touched subpages by padding 0xFF */
		if ((step < start_step) || (step > end_step))
			memset(ecc_calc, 0xff, ecc_bytes);
		else
			chip->cur_ecc->calculate(mtd, buf, ecc_calc);

		/* mask OOB of un-touched subpages by padding 0xFF */
		/* if oob_required, preserve OOB metadata of written subpage */
		if (!oob_required || (step < start_step) || (step > end_step))
			memset(oob_buf, 0xff, oob_bytes);

		buf += ecc_size;
		ecc_calc += ecc_bytes;
		oob_buf  += oob_bytes;
	}

	/* copy calculated ECC for whole page to chip->buffer->oob */
	/* this include masked-value(0xFF) for unwritten subpages */
	ecc_calc = chip->buffers->ecccalc;
	for (i = 0; i < chip->cur_ecc->total; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];

	/* write OOB buffer to NAND device */
	nand_rnd_config(mtd, -1, offset, NAND_RND_WRITE);
	nand_rnd_write_buf(mtd, chip->oob_poi, mtd->oobsize);

	return 0;
}


/**
 * nand_write_page_syndrome - [REPLACEABLE] hardware ECC syndrome based page write
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: data buffer
 * @oob_required: must write chip->oob_poi to OOB
 *
 * The hw generator calculates the error syndrome automatically. Therefore we
 * need a special oob layout and handling.
 */
static int nand_write_page_syndrome(struct mtd_info *mtd,
				    struct nand_chip *chip,
				    const uint8_t *buf, int oob_required)
{
	int i, eccsize = chip->cur_ecc->size;
	int eccbytes = chip->cur_ecc->bytes;
	int eccsteps = chip->cur_ecc->steps;
	const uint8_t *p = buf;
	uint8_t *oob = chip->oob_poi;
	int column = 0;

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {

		chip->cur_ecc->hwctl(mtd, NAND_ECC_WRITE);
		nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, p, eccsize);
		column += eccsize;

		if (chip->cur_ecc->prepad) {
			nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
			nand_rnd_write_buf(mtd, oob, chip->cur_ecc->prepad);
			oob += chip->cur_ecc->prepad;
			column += chip->cur_ecc->prepad;
		}

		chip->cur_ecc->calculate(mtd, p, oob);
		nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, oob, eccbytes);
		oob += eccbytes;
		column += eccbytes;

		if (chip->cur_ecc->postpad) {
			nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
			nand_rnd_write_buf(mtd, oob, chip->cur_ecc->postpad);
			oob += chip->cur_ecc->postpad;
			column += chip->cur_ecc->postpad;
		}
	}

	/* Calculate remaining oob bytes */
	i = mtd->oobsize - (oob - chip->oob_poi);
	if (i) {
		nand_rnd_config(mtd, -1, column, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, oob, i);
	}

	return 0;
}

/**
 * nand_write_page - [REPLACEABLE] write one page
 * @mtd: MTD device structure
 * @chip: NAND chip descriptor
 * @offset: address offset within the page
 * @data_len: length of actual data to be written
 * @buf: the data to write
 * @oob_required: must write chip->oob_poi to OOB
 * @page: page number to write
 * @cached: cached programming
 * @raw: use _raw version of write_page
 */
static int nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
		uint32_t offset, int data_len, const uint8_t *buf,
		int oob_required, int page, int cached, int raw)
{
	int status, subpage;

	if (!(chip->options & NAND_NO_SUBPAGE_WRITE) &&
		chip->cur_ecc->write_subpage)
		subpage = offset || (data_len < mtd->writesize);
	else
		subpage = 0;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	nand_rnd_config(mtd, page, 0, NAND_RND_WRITE);
	if (unlikely(raw))
		status = chip->cur_ecc->write_page_raw(mtd, chip, buf,
						       oob_required);
	else if (subpage)
		status = chip->cur_ecc->write_subpage(mtd, chip, offset,
						      data_len, buf,
						      oob_required);
	else
		status = chip->cur_ecc->write_page(mtd, chip, buf,
						   oob_required);
	nand_rnd_config(mtd, -1, -1, NAND_RND_WRITE);

	if (status < 0)
		return status;

	/*
	 * Cached progamming disabled for now. Not sure if it's worth the
	 * trouble. The speed gain is not very impressive. (2.3->2.6Mib/s).
	 */
	cached = 0;

	if (!cached || !NAND_HAS_CACHEPROG(chip)) {

		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
		/*
		 * See if operation failed and additional status checks are
		 * available.
		 */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_WRITING, status,
					       page);

		if (status & NAND_STATUS_FAIL)
			return -EIO;
	} else {
		chip->cmdfunc(mtd, NAND_CMD_CACHEDPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
	}

	return 0;
}

/**
 * nand_fill_oob - [INTERN] Transfer client buffer to oob
 * @mtd: MTD device structure
 * @oob: oob data buffer
 * @len: oob data write length
 * @ops: oob ops structure
 */
static uint8_t *nand_fill_oob(struct mtd_info *mtd, uint8_t *oob, size_t len,
			      struct mtd_oob_ops *ops)
{
	struct nand_chip *chip = mtd->priv;

	/*
	 * Initialise to all 0xFF, to avoid the possibility of left over OOB
	 * data from a previous OOB read.
	 */
	memset(chip->oob_poi, 0xff, mtd->oobsize);

	switch (ops->mode) {

	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_RAW:
		memcpy(chip->oob_poi + ops->ooboffs, oob, len);
		return oob + len;

	case MTD_OPS_AUTO_OOB: {
		struct nand_oobfree *free = chip->cur_ecc->layout->oobfree;
		uint32_t boffs = 0, woffs = ops->ooboffs;
		size_t bytes = 0;

		for (; free->length && len; free++, len -= bytes) {
			/* Write request not from offset 0? */
			if (unlikely(woffs)) {
				if (woffs >= free->length) {
					woffs -= free->length;
					continue;
				}
				boffs = free->offset + woffs;
				bytes = min_t(size_t, len,
					      (free->length - woffs));
				woffs = 0;
			} else {
				bytes = min_t(size_t, len, free->length);
				boffs = free->offset;
			}
			memcpy(chip->oob_poi + boffs, oob, bytes);
			oob += bytes;
		}
		return oob;
	}
	default:
		BUG();
	}
	return NULL;
}

/*
 * This computation is correct for non-datasheet page sizes if the chip has no
 * subpages. For datasheet page sizes it's correctness depends on chip and mtd
 * info initialisation.
 */
#define page_aligned(x)							\
	(mtd->subpage_sft ?						\
	 (x & (chip->subpagesize - 1)) == 0 :				\
	 (x & (mtd->writesize - 1)) == 0)

/**
 * nand_do_write_ops - [INTERN] NAND write with ECC
 * @mtd: MTD device structure
 * @to: offset to write to
 * @ops: oob operations description structure
 *
 * NAND write with ECC.
 */
static int nand_do_write_ops(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops)
{
	int chipnr, realpage, page, blockmask, column;
	struct nand_chip *chip = mtd->priv;
	uint32_t writelen = ops->len;

	uint32_t oobwritelen = ops->ooblen;
	uint32_t oobmaxlen = ops->mode == MTD_OPS_AUTO_OOB ?
				mtd->oobavail : mtd->oobsize;

	uint8_t *oob = ops->oobbuf;
	uint8_t *buf = ops->datbuf;
	int ret;
	int oob_required = oob ? 1 : 0;

/*
	DBG("mtd %p, name %s, to 0x%llx, writelen %u, mtd writesize %u"
            "oobwritelen %u, subpagesize %u, subpage_sft %u",
	    mtd, mtd->name, to, writelen, mtd->writesize,
	    oobwritelen, chip->subpagesize, mtd->subpage_sft);
*/
	ops->retlen = 0;
	if (!writelen)
		return 0;

	/* Reject writes, which are not page aligned */
	if (!page_aligned(to) || !page_aligned(writelen)) {
		pr_notice("%s: attempt to write non page aligned data\n",
			  __func__);
		return -EINVAL;
	}

	column = to & (mtd->writesize - 1);

	chipnr = (int)(to >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	/* Check, if it is write protected */
	if (nand_check_wp(mtd)) {
		ret = -EIO;
		goto err_out;
	}

	/* Check if the write size equals the datasheet page size. */
	if (1 << chip->page_shift == mtd->writesize)
		realpage = (int) (to >> chip->page_shift);
	else /* correction for non-datasheet page size */
		realpage = (int) DIV_ROUND_UP_ULL(to, mtd->writesize);

	page = realpage & chip->pagemask;
	blockmask = (1 << (chip->phys_erase_shift - chip->page_shift)) - 1;
/*
	DBG("column %d, realpage %d, page_shift %u, page %u, pagemask 0x%x,"
	    " blockmask 0x%x, phys_erase_shift %u",
	    column, realpage, chip->page_shift, page, chip->pagemask,
	    blockmask, chip->phys_erase_shift);
*/
	/* Invalidate the page cache, when we write to the cached page */
	if (to <= ((loff_t)chip->pagebuf << chip->page_shift) &&
	    ((loff_t)chip->pagebuf << chip->page_shift) < (to + ops->len))
		chip->pagebuf = -1;

	/* Don't allow multipage oob writes with offset */
	if (oob && ops->ooboffs && (ops->ooboffs + ops->ooblen > oobmaxlen)) {
		ret = -EINVAL;
		goto err_out;
	}

	while (1) {
		int bytes = mtd->writesize;
		int cached = writelen > bytes && page != blockmask;
		uint8_t *wbuf = buf;
		int use_bufpoi;
		int part_pagewr = (column || writelen < (mtd->writesize - 1));
		int subpage;

		if (part_pagewr)
			use_bufpoi = 1;
		else if (chip->options & NAND_USE_BOUNCE_BUFFER)
			use_bufpoi = !virt_addr_valid(buf);
		else
			use_bufpoi = 0;

		/* Partial page write?, or need to use bounce buffer */
		if (use_bufpoi) {
			pr_debug("%s: using write bounce buffer for buf@%p\n",
					 __func__, buf);
			cached = 0;
			if (part_pagewr)
				bytes = min_t(int, bytes - column, writelen);
			chip->pagebuf = -1;
			memset(chip->buffers->databuf, 0xff, mtd->writesize);
			memcpy(&chip->buffers->databuf[column], buf, bytes);
			wbuf = chip->buffers->databuf;
		}

		if (unlikely(oob)) {
			size_t len = min(oobwritelen, oobmaxlen);
			oob = nand_fill_oob(mtd, oob, len, ops);
			oobwritelen -= len;
		} else {
			/* We still need to erase leftover OOB data */
			memset(chip->oob_poi, 0xff, mtd->oobsize);
		}
		ret = chip->write_page(mtd, chip, column, bytes, wbuf,
					oob_required, page, cached,
					(ops->mode == MTD_OPS_RAW));
		if (ret)
			break;

		for (subpage = column / chip->subpagesize;
		     subpage < (column + writelen) / chip->subpagesize;
		     subpage++)
			nand_page_set_status(mtd,
					     (page << mtd->subpage_sft) +
					     subpage,
					     NAND_PAGE_FILLED);

		writelen -= bytes;
		if (!writelen)
			break;

		column = 0;
		buf += bytes;
		realpage++;

		page = realpage & chip->pagemask;
		/* Check, if we cross a chip boundary */
		if (!page) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}
	}

	ops->retlen = ops->len - writelen;
	if (unlikely(oob))
		ops->oobretlen = ops->ooblen;

err_out:
	chip->select_chip(mtd, -1);
	return ret;
}

/**
 * panic_nand_write - [MTD Interface] NAND write with ECC
 * @mtd: MTD device structure
 * @to: offset to write to
 * @len: number of bytes to write
 * @retlen: pointer to variable to store the number of written bytes
 * @buf: the data to write
 *
 * NAND write with ECC. Used when performing writes in interrupt context, this
 * may for example be called by mtdoops when writing an oops while in panic.
 */
static int panic_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
			    size_t *retlen, const uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	struct mtd_oob_ops ops;
	int ret;

	/* Wait for the device to get ready */
	panic_nand_wait(mtd, chip, 400);

	/* Grab the device */
	panic_nand_get_device(chip, mtd, FL_WRITING);

	ops.len = len;
	ops.datbuf = (uint8_t *)buf;
	ops.oobbuf = NULL;
	ops.mode = MTD_OPS_PLACE_OOB;

	ret = nand_do_write_ops(mtd, to, &ops);

	*retlen = ops.retlen;
	return ret;
}

/**
 * panic_nand_part_write - [MTD Interface] NAND write with ECC
 * @mtd: MTD device structure
 * @to: offset to write to
 * @len: number of bytes to write
 * @retlen: pointer to variable to store the number of written bytes
 * @buf: the data to write
 *
 * NAND write with ECC. Used when performing writes in interrupt context, this
 * may for example be called by mtdoops when writing an oops while in panic.
 */
static int panic_nand_part_write(struct mtd_info *mtd, loff_t to, size_t len,
			    size_t *retlen, const uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_part *part = to_nand_part(mtd);
	struct mtd_oob_ops ops;
	int ret;

	to += part->offset;
	/* Wait for the device to get ready */
	panic_nand_wait(part->master, chip, 400);

	/* Grab the device */
	panic_nand_get_device(chip, part->master, FL_WRITING);
	if (part->ecc)
		chip->cur_ecc = part->ecc;
	if (part->rnd)
		chip->cur_rnd = part->rnd;

	ops.len = len;
	ops.datbuf = (uint8_t *)buf;
	ops.oobbuf = NULL;
	ops.mode = MTD_OPS_PLACE_OOB;

	ret = nand_do_write_ops(part->master, to, &ops);

	chip->cur_rnd = &chip->rnd;
	chip->cur_ecc = &chip->ecc;
	*retlen = ops.retlen;
	return ret;
}

/**
 * nand_write - [MTD Interface] NAND write with ECC
 * @mtd: MTD device structure
 * @to: offset to write to
 * @len: number of bytes to write
 * @retlen: pointer to variable to store the number of written bytes
 * @buf: the data to write
 *
 * NAND write with ECC.
 */
static int nand_write(struct mtd_info *mtd, loff_t to, size_t len,
			  size_t *retlen, const uint8_t *buf)
{
	struct mtd_oob_ops ops;
	int ret;

	nand_get_device(mtd, FL_WRITING);
	ops.len = len;
	ops.datbuf = (uint8_t *)buf;
	ops.oobbuf = NULL;
	ops.mode = MTD_OPS_PLACE_OOB;
	ret = nand_do_write_ops(mtd, to, &ops);
	*retlen = ops.retlen;
	nand_release_device(mtd);
	return ret;
}

/**
 * nand_part_write - [MTD Interface] NAND write with ECC
 * @mtd: MTD device structure
 * @to: offset to write to
 * @len: number of bytes to write
 * @retlen: pointer to variable to store the number of written bytes
 * @buf: the data to write
 *
 * NAND write with ECC.
 */
static int nand_part_write(struct mtd_info *mtd, loff_t to, size_t len,
			  size_t *retlen, const uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_part *part = to_nand_part(mtd);
	struct mtd_oob_ops ops;
	int ret;

	to += part->offset;
	nand_get_device(part->master, FL_WRITING);
	if (part->ecc)
		chip->cur_ecc = part->ecc;
	if (part->rnd)
		chip->cur_rnd = part->rnd;
	ops.len = len;
	ops.datbuf = (uint8_t *)buf;
	ops.oobbuf = NULL;
	ops.mode = MTD_OPS_PLACE_OOB;
	ret = nand_do_write_ops(part->master, to, &ops);
	*retlen = ops.retlen;
	chip->cur_rnd = &chip->rnd;
	chip->cur_ecc = &chip->ecc;
	nand_release_device(part->master);
	return ret;
}

/**
 * nand_do_write_oob - [MTD Interface] NAND write out-of-band
 * @mtd: MTD device structure
 * @to: offset to write to
 * @ops: oob operation description structure
 *
 * NAND write out-of-band.
 */
static int nand_do_write_oob(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops)
{
	int chipnr, page, status, len;
	struct nand_chip *chip = mtd->priv;

	pr_debug("%s: to = 0x%08x, len = %i\n",
			 __func__, (unsigned int)to, (int)ops->ooblen);

	if (ops->mode == MTD_OPS_AUTO_OOB)
		len = chip->cur_ecc->layout->oobavail;
	else
		len = mtd->oobsize;

	/* Do not allow write past end of page */
	if ((ops->ooboffs + ops->ooblen) > len) {
		pr_debug("%s: attempt to write past end of page\n",
				__func__);
		return -EINVAL;
	}

	if (unlikely(ops->ooboffs >= len)) {
		pr_debug("%s: attempt to start write outside oob\n",
				__func__);
		return -EINVAL;
	}

	/* Do not allow write past end of device */
	if (unlikely(to >= mtd->size ||
		     ops->ooboffs + ops->ooblen >
			((mtd->size >> chip->page_shift) -
			 (to >> chip->page_shift)) * len)) {
		pr_debug("%s: attempt to write beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	chipnr = (int)(to >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	/* Shift to get page */
	page = (int)(to >> chip->page_shift);

	/*
	 * Reset the chip. Some chips (like the Toshiba TC5832DC found in one
	 * of my DiskOnChip 2000 test units) will clear the whole data page too
	 * if we don't do this. I have no clue why, but I seem to have 'fixed'
	 * it in the doc2000 driver in August 1999.  dwmw2.
	 */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* Check, if it is write protected */
	if (nand_check_wp(mtd)) {
		chip->select_chip(mtd, -1);
		return -EROFS;
	}

	/* Invalidate the page cache, if we write to the cached page */
	if (page == chip->pagebuf)
		chip->pagebuf = -1;

	nand_fill_oob(mtd, ops->oobbuf, ops->ooblen, ops);

	if (ops->mode == MTD_OPS_RAW)
		status = chip->cur_ecc->write_oob_raw(mtd, chip,
						      page & chip->pagemask);
	else
		status = chip->cur_ecc->write_oob(mtd, chip,
						  page & chip->pagemask);

	chip->select_chip(mtd, -1);

	if (status)
		return status;

	ops->oobretlen = ops->ooblen;

	return 0;
}

/**
 * nand_write_oob - [MTD Interface] NAND write data and/or out-of-band
 * @mtd: MTD device structure
 * @to: offset to write to
 * @ops: oob operation description structure
 */
static int nand_write_oob(struct mtd_info *mtd, loff_t to,
			  struct mtd_oob_ops *ops)
{
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow writes past end of device */
	if (ops->datbuf && (to + ops->len) > mtd->size) {
		pr_debug("%s: attempt to write beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	nand_get_device(mtd, FL_WRITING);

	switch (ops->mode) {
	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_AUTO_OOB:
	case MTD_OPS_RAW:
		break;

	default:
		goto out;
	}

	if (!ops->datbuf)
		ret = nand_do_write_oob(mtd, to, ops);
	else
		ret = nand_do_write_ops(mtd, to, ops);

out:
	nand_release_device(mtd);
	return ret;
}

/**
 * nand_write_oob - [MTD Interface] NAND write data and/or out-of-band
 * @mtd: MTD device structure
 * @to: offset to write to
 * @ops: oob operation description structure
 */
static int nand_part_write_oob(struct mtd_info *mtd, loff_t to,
			  struct mtd_oob_ops *ops)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_part *part = to_nand_part(mtd);
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow writes past end of device */
	if (ops->datbuf && (to + ops->len) > mtd->size) {
		pr_debug("%s: attempt to write beyond end of device\n",
				__func__);
		return -EINVAL;
	}

	to += part->offset;
	nand_get_device(part->master, FL_WRITING);
	if (part->ecc)
		chip->cur_ecc = part->ecc;
	if (part->rnd)
		chip->cur_rnd = part->rnd;

	switch (ops->mode) {
	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_AUTO_OOB:
	case MTD_OPS_RAW:
		break;

	default:
		goto out;
	}

	if (!ops->datbuf)
		ret = nand_do_write_oob(part->master, to, ops);
	else
		ret = nand_do_write_ops(part->master, to, ops);

out:
	chip->cur_rnd = &chip->rnd;
	chip->cur_ecc = &chip->ecc;
	nand_release_device(part->master);
	return ret;
}

/**
 * single_erase - [GENERIC] NAND standard block erase command function
 * @mtd: MTD device structure
 * @page: the page address of the block which will be erased
 *
 * Standard erase command for NAND chips. Returns NAND status.
 */
static int single_erase(struct mtd_info *mtd, int page)
{
	struct nand_chip *chip = mtd->priv;
	/* Send commands to erase a block */
	chip->cmdfunc(mtd, NAND_CMD_ERASE1, -1, page);
	chip->cmdfunc(mtd, NAND_CMD_ERASE2, -1, -1);

	return chip->waitfunc(mtd, chip);
}

/**
 * nand_erase - [MTD Interface] erase block(s)
 * @mtd: MTD device structure
 * @instr: erase instruction
 *
 * Erase one ore more blocks.
 */
static int nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	return nand_erase_nand(mtd, instr, 0);
}

/**
 * nand_part_erase - [MTD Interface] erase partition block(s)
 * @mtd: MTD device structure
 * @instr: erase instruction
 *
 * Erase one ore more blocks.
 */
static int nand_part_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct nand_part *part = to_nand_part(mtd);
	int ret;

	instr->addr += part->offset;
	ret = nand_erase_nand(part->master, instr, 0);
	if (ret) {
		if (instr->fail_addr != MTD_FAIL_ADDR_UNKNOWN)
			instr->fail_addr -= part->offset;
		instr->addr -= part->offset;
	}

	return ret;
}

/**
 * nand_erase_nand - [INTERN] erase block(s)
 * @mtd: MTD device structure
 * @instr: erase instruction
 * @allowbbt: allow erasing the bbt area
 *
 * Erase one ore more blocks.
 */
int nand_erase_nand(struct mtd_info *mtd, struct erase_info *instr,
		    int allowbbt)
{
	int page, status, pages_per_block, ret, chipnr;
	struct nand_chip *chip = mtd->priv;
	loff_t len;
	int i;

	pr_debug("%s: start = 0x%012llx, len = %llu\n",
			__func__, (unsigned long long)instr->addr,
			(unsigned long long)instr->len);

	if (check_offs_len(mtd, instr->addr, instr->len))
		return -EINVAL;

	/* Grab the lock and see if the device is available */
	nand_get_device(mtd, FL_ERASING);

	/* Shift to get first page */
	page = (int)(instr->addr >> chip->page_shift);
	chipnr = (int)(instr->addr >> chip->chip_shift);

	/* Calculate pages in each block */
	pages_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);

	/* Select the NAND device */
	chip->select_chip(mtd, chipnr);

	/* Check, if it is write protected */
	if (nand_check_wp(mtd)) {
		pr_debug("%s: device is write protected!\n",
				__func__);
		instr->state = MTD_ERASE_FAILED;
		goto erase_exit;
	}

	/* Loop through the pages */
	len = instr->len;

	instr->state = MTD_ERASING;

	while (len) {
		/* Check if we have a bad block, we do not erase bad blocks! */
		if (nand_block_checkbad(mtd, ((loff_t) page) <<
					chip->page_shift, 0, allowbbt)) {
			pr_warn("%s: attempt to erase a bad block at page 0x%08x\n",
				    __func__, page);
			if (! (chip->options & NAND_INVALID_BBM)) {
					instr->state = MTD_ERASE_FAILED;
					goto erase_exit;
			}
		}

		/*
		 * Invalidate the page cache, if we erase the block which
		 * contains the current cached page.
		 */
		if (page <= chip->pagebuf && chip->pagebuf <
		    (page + pages_per_block))
			chip->pagebuf = -1;

		status = chip->erase(mtd, page & chip->pagemask);

		/*
		 * See if operation failed and additional status checks are
		 * available
		 */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_ERASING,
					       status, page);

		/* See if block erase succeeded */
		if (status & NAND_STATUS_FAIL) {
			pr_debug("%s: failed erase, page 0x%08x\n",
					__func__, page);
			instr->state = MTD_ERASE_FAILED;
			instr->fail_addr =
				((loff_t)page << chip->page_shift);
			goto erase_exit;
		}

		for (i = 0; i < pages_per_block; i++) {
			int subpage;
			for (subpage = 0;
			     subpage < 1 << mtd->subpage_sft;
			     subpage++) {
				nand_page_set_status(mtd,
					((page + i) << mtd->subpage_sft) +
					subpage,
					NAND_PAGE_EMPTY);
			}
		}

		/* Increment page address and decrement length */
		len -= (1ULL << chip->phys_erase_shift);
		page += pages_per_block;

		/* Check, if we cross a chip boundary */
		if (len && !(page & chip->pagemask)) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}
	}
	instr->state = MTD_ERASE_DONE;

erase_exit:

	ret = instr->state == MTD_ERASE_DONE ? 0 : -EIO;

	/* Deselect and wake up anyone waiting on the device */
	chip->select_chip(mtd, -1);
	nand_release_device(mtd);

	/* Do call back function */
	if (!ret)
		mtd_erase_callback(instr);

	/* Return more or less happy */
	return ret;
}

/**
 * nand_sync - [MTD Interface] sync
 * @mtd: MTD device structure
 *
 * Sync is actually a wait for chip ready function.
 */
static void nand_sync(struct mtd_info *mtd)
{
	pr_debug("%s: called\n", __func__);

	/* Grab the lock and see if the device is available */
	nand_get_device(mtd, FL_SYNCING);
	/* Release it and go back */
	nand_release_device(mtd);
}

/**
 * nand_block_isbad - [MTD Interface] Check if block at offset is bad
 * @mtd: MTD device structure
 * @offs: offset relative to mtd start
 */
static int nand_block_isbad(struct mtd_info *mtd, loff_t offs)
{
	return nand_block_checkbad(mtd, offs, 1, 0);
}

/**
 * nand_part_block_isbad - [MTD Interface] Check if block at offset is bad
 * @mtd: MTD device structure
 * @offs: offset relative to mtd start
 */
static int nand_part_block_isbad(struct mtd_info *mtd, loff_t offs)
{
	struct nand_part *part = to_nand_part(mtd);

	return nand_block_checkbad(part->master, part->offset + offs, 1, 0);
}

/**
 * nand_block_markbad - [MTD Interface] Mark block at the given offset as bad
 * @mtd: MTD device structure
 * @ofs: offset relative to mtd start
 */
static int nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	int ret;

	ret = nand_block_isbad(mtd, ofs);
	if (ret) {
		/* If it was bad already, return success and do nothing */
		if (ret > 0)
			return 0;
		return ret;
	}

	return nand_block_markbad_lowlevel(mtd, ofs);
}

/**
 * nand_part_block_markbad - [MTD Interface] Mark block at the given offset as
 *			     bad
 * @mtd: MTD device structure
 * @ofs: offset relative to mtd start
 */
static int nand_part_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_part *part = to_nand_part(mtd);
	int ret;

	ofs += part->offset;
	ret = nand_block_isbad(part->master, ofs);
	if (ret) {
		/* If it was bad already, return success and do nothing */
		if (ret > 0)
			return 0;
		return ret;
	}

	ret = nand_block_markbad_lowlevel(part->master, ofs);
	if (!ret)
		mtd->ecc_stats.badblocks++;

	return ret;
}

/**
 * nand_onfi_set_features- [REPLACEABLE] set features for ONFI nand
 * @mtd: MTD device structure
 * @chip: nand chip info structure
 * @addr: feature address.
 * @subfeature_param: the subfeature parameters, a four bytes array.
 */
static int nand_onfi_set_features(struct mtd_info *mtd, struct nand_chip *chip,
			int addr, uint8_t *subfeature_param)
{
	int status;
	int i;

	if (!chip->onfi_version ||
	    !(le16_to_cpu(chip->onfi_params.opt_cmd)
	      & ONFI_OPT_CMD_SET_GET_FEATURES))
		return -EINVAL;

	chip->cmdfunc(mtd, NAND_CMD_SET_FEATURES, addr, -1);
	for (i = 0; i < ONFI_SUBFEATURE_PARAM_LEN; ++i)
		chip->write_byte(mtd, subfeature_param[i]);

	status = chip->waitfunc(mtd, chip);
	if (status & NAND_STATUS_FAIL)
		return -EIO;
	return 0;
}

/**
 * nand_onfi_get_features- [REPLACEABLE] get features for ONFI nand
 * @mtd: MTD device structure
 * @chip: nand chip info structure
 * @addr: feature address.
 * @subfeature_param: the subfeature parameters, a four bytes array.
 */
static int nand_onfi_get_features(struct mtd_info *mtd, struct nand_chip *chip,
			int addr, uint8_t *subfeature_param)
{
	int i;

	if (!chip->onfi_version ||
	    !(le16_to_cpu(chip->onfi_params.opt_cmd)
	      & ONFI_OPT_CMD_SET_GET_FEATURES))
		return -EINVAL;

	/* clear the sub feature parameters */
	memset(subfeature_param, 0, ONFI_SUBFEATURE_PARAM_LEN);

	chip->cmdfunc(mtd, NAND_CMD_GET_FEATURES, addr, -1);
	for (i = 0; i < ONFI_SUBFEATURE_PARAM_LEN; ++i)
		*subfeature_param++ = chip->read_byte(mtd);
	return 0;
}

/**
 * nand_suspend - [MTD Interface] Suspend the NAND flash
 * @mtd: MTD device structure
 */
static int nand_suspend(struct mtd_info *mtd)
{
	return nand_get_device(mtd, FL_PM_SUSPENDED);
}

/**
 * nand_resume - [MTD Interface] Resume the NAND flash
 * @mtd: MTD device structure
 */
static void nand_resume(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

	if (chip->state == FL_PM_SUSPENDED)
		nand_release_device(mtd);
	else
		pr_err("%s called for a chip which is not in suspended state\n",
			__func__);
}

/* Set default functions */
static void nand_set_defaults(struct nand_chip *chip, int busw)
{
	/* check for proper chip_delay setup, set 20us if not */
	if (!chip->chip_delay)
		chip->chip_delay = 20;

	/* check, if a user supplied command function given */
	if (chip->cmdfunc == NULL)
		chip->cmdfunc = nand_command;

	/* check, if a user supplied wait function given */
	if (chip->waitfunc == NULL)
		chip->waitfunc = nand_wait;

	if (!chip->select_chip)
		chip->select_chip = nand_select_chip;

	/* set for ONFI nand */
	if (!chip->onfi_set_features)
		chip->onfi_set_features = nand_onfi_set_features;
	if (!chip->onfi_get_features)
		chip->onfi_get_features = nand_onfi_get_features;

	/* If called twice, pointers that depend on busw may need to be reset */
	if (!chip->read_byte || chip->read_byte == nand_read_byte)
		chip->read_byte = busw ? nand_read_byte16 : nand_read_byte;
	if (!chip->read_word)
		chip->read_word = nand_read_word;
	if (!chip->block_bad)
		chip->block_bad = nand_block_bad;
	if (!chip->block_markbad)
		chip->block_markbad = nand_default_block_markbad;
	if (!chip->write_buf || chip->write_buf == nand_write_buf)
		chip->write_buf = busw ? nand_write_buf16 : nand_write_buf;
	if (!chip->write_byte || chip->write_byte == nand_write_byte)
		chip->write_byte = busw ? nand_write_byte16 : nand_write_byte;
	if (!chip->read_buf || chip->read_buf == nand_read_buf)
		chip->read_buf = busw ? nand_read_buf16 : nand_read_buf;
	if (!chip->scan_bbt)
		chip->scan_bbt = nand_default_bbt;

	if (!chip->controller) {
		chip->controller = &chip->hwcontrol;
		spin_lock_init(&chip->controller->lock);
		init_waitqueue_head(&chip->controller->wq);
	}

}

/* Sanitize ONFI strings so we can safely print them */
static void sanitize_string(uint8_t *s, size_t len)
{
	ssize_t i;

	/* Null terminate */
	s[len - 1] = 0;

	/* Remove non printable chars */
	for (i = 0; i < len - 1; i++) {
		if (s[i] < ' ' || s[i] > 127)
			s[i] = '?';
	}

	/* Remove trailing spaces */
	strim(s);
}

static u16 onfi_crc16(u16 crc, u8 const *p, size_t len)
{
	int i;
	while (len--) {
		crc ^= *p++ << 8;
		for (i = 0; i < 8; i++)
			crc = (crc << 1) ^ ((crc & 0x8000) ? 0x8005 : 0);
	}

	return crc;
}

/* Parse the Extended Parameter Page. */
static int nand_flash_detect_ext_param_page(struct mtd_info *mtd,
		struct nand_chip *chip, struct nand_onfi_params *p)
{
	struct onfi_ext_param_page *ep;
	struct onfi_ext_section *s;
	struct onfi_ext_ecc_info *ecc;
	uint8_t *cursor;
	int ret = -EINVAL;
	int len;
	int i;

	len = le16_to_cpu(p->ext_param_page_length) * 16;
	ep = kmalloc(len, GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	/* Send our own NAND_CMD_PARAM. */
	chip->cmdfunc(mtd, NAND_CMD_PARAM, 0, -1);

	/* Use the Change Read Column command to skip the ONFI param pages. */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT,
			sizeof(*p) * p->num_of_param_pages , -1);

	/* Read out the Extended Parameter Page. */
	chip->read_buf(mtd, (uint8_t *)ep, len);
	if ((onfi_crc16(ONFI_CRC_BASE, ((uint8_t *)ep) + 2, len - 2)
		!= le16_to_cpu(ep->crc))) {
		pr_debug("fail in the CRC.\n");
		goto ext_out;
	}

	/*
	 * Check the signature.
	 * Do not strictly follow the ONFI spec, maybe changed in future.
	 */
	if (strncmp(ep->sig, "EPPS", 4)) {
		pr_debug("The signature is invalid.\n");
		goto ext_out;
	}

	/* find the ECC section. */
	cursor = (uint8_t *)(ep + 1);
	for (i = 0; i < ONFI_EXT_SECTION_MAX; i++) {
		s = ep->sections + i;
		if (s->type == ONFI_SECTION_TYPE_2)
			break;
		cursor += s->length * 16;
	}
	if (i == ONFI_EXT_SECTION_MAX) {
		pr_debug("We can not find the ECC section.\n");
		goto ext_out;
	}

	/* get the info we want. */
	ecc = (struct onfi_ext_ecc_info *)cursor;

	if (!ecc->codeword_size) {
		pr_debug("Invalid codeword size\n");
		goto ext_out;
	}

	chip->ecc_strength_ds = ecc->ecc_bits;
	chip->ecc_step_ds = 1 << ecc->codeword_size;
	ret = 0;

ext_out:
	kfree(ep);
	return ret;
}

static int nand_setup_read_retry_micron(struct mtd_info *mtd, int retry_mode)
{
	struct nand_chip *chip = mtd->priv;
	uint8_t feature[ONFI_SUBFEATURE_PARAM_LEN] = {retry_mode};

	return chip->onfi_set_features(mtd, chip, ONFI_FEATURE_ADDR_READ_RETRY,
			feature);
}

/*
 * Configure chip properties from Micron vendor-specific ONFI table
 */
static void nand_onfi_detect_micron(struct nand_chip *chip,
		struct nand_onfi_params *p)
{
	struct nand_onfi_vendor_micron *micron = (void *)p->vendor;

	if (le16_to_cpu(p->vendor_revision) < 1)
		return;

	chip->read_retries = micron->read_retry_options;
	chip->setup_read_retry = nand_setup_read_retry_micron;
}

/*
 * Check if the NAND chip is ONFI compliant, returns 1 if it is, 0 otherwise.
 */
static int nand_flash_detect_onfi(struct mtd_info *mtd, struct nand_chip *chip,
					int *busw)
{
	struct nand_onfi_params *p = &chip->onfi_params;
	int i, j;
	int val;

	/* Try ONFI for unknown chip or LP */
	chip->cmdfunc(mtd, NAND_CMD_READID, 0x20, -1);
	if (chip->read_byte(mtd) != 'O' || chip->read_byte(mtd) != 'N' ||
		chip->read_byte(mtd) != 'F' || chip->read_byte(mtd) != 'I')
		return 0;

	chip->cmdfunc(mtd, NAND_CMD_PARAM, 0, -1);
	for (i = 0; i < 3; i++) {
		for (j = 0; j < sizeof(*p); j++)
			((uint8_t *)p)[j] = chip->read_byte(mtd);
		if (onfi_crc16(ONFI_CRC_BASE, (uint8_t *)p, 254) ==
				le16_to_cpu(p->crc)) {
			break;
		}
	}

	if (i == 3) {
		pr_err("Could not find valid ONFI parameter page; aborting\n");
		return 0;
	}

	/* Check version */
	val = le16_to_cpu(p->revision);
	if (val & (1 << 5))
		chip->onfi_version = 23;
	else if (val & (1 << 4))
		chip->onfi_version = 22;
	else if (val & (1 << 3))
		chip->onfi_version = 21;
	else if (val & (1 << 2))
		chip->onfi_version = 20;
	else if (val & (1 << 1))
		chip->onfi_version = 10;

	if (!chip->onfi_version) {
		pr_info("unsupported ONFI version: %d\n", val);
		return 0;
	}

	sanitize_string(p->manufacturer, sizeof(p->manufacturer));
	sanitize_string(p->model, sizeof(p->model));
	if (!mtd->name)
		mtd->name = p->model;

	mtd->writesize = le32_to_cpu(p->byte_per_page);

	/*
	 * pages_per_block and blocks_per_lun may not be a power-of-2 size
	 * (don't ask me who thought of this...). MTD assumes that these
	 * dimensions will be power-of-2, so just truncate the remaining area.
	 */
	mtd->erasesize = 1 << (fls(le32_to_cpu(p->pages_per_block)) - 1);
	mtd->erasesize *= mtd->writesize;

	mtd->oobsize = le16_to_cpu(p->spare_bytes_per_page);

	/* See erasesize comment */
	chip->chipsize = 1 << (fls(le32_to_cpu(p->blocks_per_lun)) - 1);
	chip->chipsize *= (uint64_t)mtd->erasesize * p->lun_count;
	chip->bits_per_cell = p->bits_per_cell;

	if (onfi_feature(chip) & ONFI_FEATURE_16_BIT_BUS)
		*busw = NAND_BUSWIDTH_16;
	else
		*busw = 0;

	if (p->ecc_bits != 0xff) {
		chip->ecc_strength_ds = p->ecc_bits;
		chip->ecc_step_ds = 512;
	} else if (chip->onfi_version >= 21 &&
		(onfi_feature(chip) & ONFI_FEATURE_EXT_PARAM_PAGE)) {

		/*
		 * The nand_flash_detect_ext_param_page() uses the
		 * Change Read Column command which maybe not supported
		 * by the chip->cmdfunc. So try to update the chip->cmdfunc
		 * now. We do not replace user supplied command function.
		 */
		if (mtd->writesize > 512 && chip->cmdfunc == nand_command)
			chip->cmdfunc = nand_command_lp;

		/* The Extended Parameter Page is supported since ONFI 2.1. */
		if (nand_flash_detect_ext_param_page(mtd, chip, p))
			pr_warn("Failed to detect ONFI extended param page\n");
	} else {
		pr_warn("Could not retrieve ONFI ECC requirements\n");
	}

	if (p->jedec_id == NAND_MFR_MICRON)
		nand_onfi_detect_micron(chip, p);

	return 1;
}

/*
 * Check if the NAND chip is JEDEC compliant, returns 1 if it is, 0 otherwise.
 */
static int nand_flash_detect_jedec(struct mtd_info *mtd, struct nand_chip *chip,
					int *busw)
{
	struct nand_jedec_params *p = &chip->jedec_params;
	struct jedec_ecc_info *ecc;
	int val;
	int i, j;

	/* Try JEDEC for unknown chip or LP */
	chip->cmdfunc(mtd, NAND_CMD_READID, 0x40, -1);
	if (chip->read_byte(mtd) != 'J' || chip->read_byte(mtd) != 'E' ||
		chip->read_byte(mtd) != 'D' || chip->read_byte(mtd) != 'E' ||
		chip->read_byte(mtd) != 'C')
		return 0;

	chip->cmdfunc(mtd, NAND_CMD_PARAM, 0x40, -1);
	for (i = 0; i < 3; i++) {
		for (j = 0; j < sizeof(*p); j++)
			((uint8_t *)p)[j] = chip->read_byte(mtd);

		if (onfi_crc16(ONFI_CRC_BASE, (uint8_t *)p, 510) ==
				le16_to_cpu(p->crc))
			break;
	}

	if (i == 3) {
		pr_err("Could not find valid JEDEC parameter page; aborting\n");
		return 0;
	}

	/* Check version */
	val = le16_to_cpu(p->revision);
	if (val & (1 << 2))
		chip->jedec_version = 10;
	else if (val & (1 << 1))
		chip->jedec_version = 1; /* vendor specific version */

	if (!chip->jedec_version) {
		pr_info("unsupported JEDEC version: %d\n", val);
		return 0;
	}

	sanitize_string(p->manufacturer, sizeof(p->manufacturer));
	sanitize_string(p->model, sizeof(p->model));
	if (!mtd->name)
		mtd->name = p->model;

	mtd->writesize = le32_to_cpu(p->byte_per_page);

	/* Please reference to the comment for nand_flash_detect_onfi. */
	mtd->erasesize = 1 << (fls(le32_to_cpu(p->pages_per_block)) - 1);
	mtd->erasesize *= mtd->writesize;

	mtd->oobsize = le16_to_cpu(p->spare_bytes_per_page);

	/* Please reference to the comment for nand_flash_detect_onfi. */
	chip->chipsize = 1 << (fls(le32_to_cpu(p->blocks_per_lun)) - 1);
	chip->chipsize *= (uint64_t)mtd->erasesize * p->lun_count;
	chip->bits_per_cell = p->bits_per_cell;

	if (jedec_feature(chip) & JEDEC_FEATURE_16_BIT_BUS)
		*busw = NAND_BUSWIDTH_16;
	else
		*busw = 0;

	/* ECC info */
	ecc = &p->ecc_info[0];

	if (ecc->codeword_size >= 9) {
		chip->ecc_strength_ds = ecc->ecc_bits;
		chip->ecc_step_ds = 1 << ecc->codeword_size;
	} else {
		pr_warn("Invalid codeword size\n");
	}

	return 1;
}

/*
 * nand_id_has_period - Check if an ID string has a given wraparound period
 * @id_data: the ID string
 * @arrlen: the length of the @id_data array
 * @period: the period of repitition
 *
 * Check if an ID string is repeated within a given sequence of bytes at
 * specific repetition interval period (e.g., {0x20,0x01,0x7F,0x20} has a
 * period of 3). This is a helper function for nand_id_len(). Returns non-zero
 * if the repetition has a period of @period; otherwise, returns zero.
 */
static int nand_id_has_period(u8 *id_data, int arrlen, int period)
{
	int i, j;
	for (i = 0; i < period; i++)
		for (j = i + period; j < arrlen; j += period)
			if (id_data[i] != id_data[j])
				return 0;
	return 1;
}

/*
 * nand_id_len - Get the length of an ID string returned by CMD_READID
 * @id_data: the ID string
 * @arrlen: the length of the @id_data array

 * Returns the length of the ID string, according to known wraparound/trailing
 * zero patterns. If no pattern exists, returns the length of the array.
 */
static int nand_id_len(u8 *id_data, int arrlen)
{
	int last_nonzero, period;

	/* Find last non-zero byte */
	for (last_nonzero = arrlen - 1; last_nonzero >= 0; last_nonzero--)
		if (id_data[last_nonzero])
			break;

	/* All zeros */
	if (last_nonzero < 0)
		return 0;

	/* Calculate wraparound period */
	for (period = 1; period < arrlen; period++)
		if (nand_id_has_period(id_data, arrlen, period))
			break;

	/* There's a repeated pattern */
	if (period < arrlen)
		return period;

	/* There are trailing zeros */
	if (last_nonzero < arrlen - 1)
		return last_nonzero + 1;

	/* No pattern detected */
	return arrlen;
}

/* Extract the bits of per cell from the 3rd byte of the extended ID */
static int nand_get_bits_per_cell(u8 cellinfo)
{
	int bits;

	bits = cellinfo & NAND_CI_CELLTYPE_MSK;
	bits >>= NAND_CI_CELLTYPE_SHIFT;
	return bits + 1;
}

/*
 * Parse the Hynix ID size byte and calculate the relevant physical parameters.
 */
static int parse_hynix_sizes(struct mtd_info *mtd, struct nand_chip* chip,
			     u8 id_data[8])
{
	u8 density   = id_data[1];
	u8 sizes     = id_data[3];
	u8 plane_ecc = id_data[4];
	u8 oob_code, erase_code, ecc;

	mtd->writesize = 2048 << (sizes & 0x03);
	sizes >>= 2;
	oob_code = sizes & 0x03;
	sizes >>= 2;
	erase_code = sizes & 0x03;
	sizes >>= 2;
	oob_code |= (sizes & 0x01) << 2;
	sizes >>= 1;
	erase_code |= (sizes & 0x01) << 2;
	pr_info("Hynix codes: page size %d, block size %d, oob size %d\n",
		mtd->writesize, erase_code, oob_code);
	if (oob_code >= 0x4 || erase_code < 0x4)
		return -EINVAL;

	if (density == 0xde /* 8GiB */ ||
	    density == 0xd7 /* 4GiB */) {
		switch (oob_code) {
		case 0:
			mtd->oobsize = 2048; break;
		case 1:
			mtd->oobsize = 1664; break;
		case 2:
			mtd->oobsize = 1024; break;
		case 3:
		default:
			mtd->oobsize = 640;  break;
		}
	}
	else { // for older Hynix chips: 0xd5?, 0xd3?, 0xdc?
		switch (oob_code) {
		case 0:
			mtd->oobsize = 128;
			break;
		case 1:
			mtd->oobsize = 224;
			break;
		case 2:
			mtd->oobsize = 448;
			break;
		case 3:
		default:
			mtd->oobsize = 64;
			break;
		case 4:
			mtd->oobsize = 32;
			break;
		case 5:
			mtd->oobsize = 16;
			break;
		}
	}
	mtd->erasesize = 0x100000 << (erase_code & 0x3);

	ecc = (plane_ecc >> 4) & 0x7;
	switch (ecc) {
	case 0:
	default:
		chip->ecc_strength_ds = 0;
		break;
	case 1:
		chip->ecc_strength_ds = 4;
		break;
	case 2:
		chip->ecc_strength_ds = 24;
		break;
	case 3:
		chip->ecc_strength_ds = 32;
		break;
	case 4:
		chip->ecc_strength_ds = 40;
		break;
	case 5:
		chip->ecc_strength_ds = 50;
		break;
	case 6:
		chip->ecc_strength_ds = 60;
		break;
	}
	chip->ecc_step_ds  = 1024;
	chip->ecc.strength = chip->ecc_strength_ds;
	chip->ecc.size     = mtd->writesize;
	chip->ecc.steps    = DIV_ROUND_UP(mtd->writesize, chip->ecc_step_ds);

	return 0;
}

/*
 * Many new NAND share similar device ID codes, which represent the size of the
 * chip. The rest of the parameters must be decoded according to generic or
 * manufacturer-specific "extended ID" decoding patterns.
 */
static void nand_decode_ext_id(struct mtd_info *mtd, struct nand_chip *chip,
				u8 id_data[8], int *busw)
{
	int extid, id_len;
	/* The 3rd id byte holds MLC / multichip data */
	chip->bits_per_cell = nand_get_bits_per_cell(id_data[2]);
	/* The 4th id byte is the important one */
	extid = id_data[3];

	id_len = nand_id_len(id_data, 8);

	/*
	 * Field definitions are in the following datasheets:
	 * Old style (4,5 byte ID): Samsung K9GAG08U0M (p.32)
	 * New Samsung (6 byte ID): Samsung K9GAG08U0F (p.44)
	 * Hynix MLC   (6 byte ID): Hynix H27UBG8T2B (p.22)
	 *
	 * Check for ID length, non-zero 6th byte, cell type, and Hynix/Samsung
	 * ID to decide what to do.
	 */
	if (id_len == 6 && id_data[0] == NAND_MFR_SAMSUNG &&
			!nand_is_slc(chip) && id_data[5] != 0x00) {
		/* Calc pagesize */
		mtd->writesize = 2048 << (extid & 0x03);
		extid >>= 2;
		/* Calc oobsize */
		switch (((extid >> 2) & 0x04) | (extid & 0x03)) {
		case 1:
			mtd->oobsize = 128;
			break;
		case 2:
			mtd->oobsize = 218;
			break;
		case 3:
			mtd->oobsize = 400;
			break;
		case 4:
			mtd->oobsize = 436;
			break;
		case 5:
			mtd->oobsize = 512;
			break;
		case 6:
			mtd->oobsize = 640;
			break;
		case 7:
		default: /* Other cases are "reserved" (unknown) */
			mtd->oobsize = 1024;
			break;
		}
		extid >>= 2;
		/* Calc blocksize */
		mtd->erasesize = (128 * 1024) <<
			(((extid >> 1) & 0x04) | (extid & 0x03));
		*busw = 0;
	} else if (id_len == 6 && id_data[0] == NAND_MFR_HYNIX &&
			!nand_is_slc(chip)) {
		parse_hynix_sizes(mtd, chip, id_data);
		*busw = 0;
	} else {
		/* Calc pagesize */
		mtd->writesize = 1024 << (extid & 0x03);
		extid >>= 2;
		/* Calc oobsize */
		mtd->oobsize = (8 << (extid & 0x01)) *
			(mtd->writesize >> 9);
		extid >>= 2;
		/* Calc blocksize. Blocksize is multiples of 64KiB */
		mtd->erasesize = (64 * 1024) << (extid & 0x03);
		extid >>= 2;
		/* Get buswidth information */
		*busw = (extid & 0x01) ? NAND_BUSWIDTH_16 : 0;

		/*
		 * Toshiba 24nm raw SLC (i.e., not BENAND) have 32B OOB per
		 * 512B page. For Toshiba SLC, we decode the 5th/6th byte as
		 * follows:
		 * - ID byte 6, bits[2:0]: 100b -> 43nm, 101b -> 32nm,
		 *                         110b -> 24nm
		 * - ID byte 5, bit[7]:    1 -> BENAND, 0 -> raw SLC
		 */
		if (id_len >= 6 && id_data[0] == NAND_MFR_TOSHIBA &&
				nand_is_slc(chip) &&
				(id_data[5] & 0x7) == 0x6 /* 24nm */ &&
				!(id_data[4] & 0x80) /* !BENAND */) {
			mtd->oobsize = 32 * mtd->writesize >> 9;
		}

	}
}

/*
 * Old devices have chip data hardcoded in the device ID table. nand_decode_id
 * decodes a matching ID table entry and assigns the MTD size parameters for
 * the chip.
 */
static void nand_decode_id(struct mtd_info *mtd, struct nand_chip *chip,
				struct nand_flash_dev *type, u8 id_data[8],
				int *busw)
{
	int maf_id = id_data[0];

	mtd->erasesize = type->erasesize;
	mtd->writesize = type->pagesize;
	mtd->oobsize = mtd->writesize / 32;
	*busw = type->options & NAND_BUSWIDTH_16;

	/* All legacy ID NAND are small-page, SLC */
	chip->bits_per_cell = 1;

	/*
	 * Check for Spansion/AMD ID + repeating 5th, 6th byte since
	 * some Spansion chips have erasesize that conflicts with size
	 * listed in nand_ids table.
	 * Data sheet (5 byte ID): Spansion S30ML-P ORNAND (p.39)
	 */
	if (maf_id == NAND_MFR_AMD && id_data[4] != 0x00 && id_data[5] == 0x00
			&& id_data[6] == 0x00 && id_data[7] == 0x00
			&& mtd->writesize == 512) {
		mtd->erasesize = 128 * 1024;
		mtd->erasesize <<= ((id_data[3] & 0x03) << 1);
	}
}

/*
 * Set the bad block marker/indicator (BBM/BBI) patterns according to some
 * heuristic patterns using various detected parameters (e.g., manufacturer,
 * page size, cell-type information).
 */
static void nand_decode_bbm_options(struct mtd_info *mtd,
				    struct nand_chip *chip, u8 id_data[8])
{
	int maf_id = id_data[0];

	/* Set the bad block position */
	if (mtd->writesize > 512 || (chip->options & NAND_BUSWIDTH_16))
		chip->badblockpos = NAND_LARGE_BADBLOCK_POS;
	else
		chip->badblockpos = NAND_SMALL_BADBLOCK_POS;

	/*
	 * Bad block marker is stored in the last page of each block on Samsung
	 * and Hynix MLC devices; stored in first two pages of each block on
	 * Micron devices with 2KiB pages and on SLC Samsung, Hynix, Toshiba,
	 * AMD/Spansion, and Macronix.  All others scan only the first page.
	 */
	if (!nand_is_slc(chip) &&
			(maf_id == NAND_MFR_SAMSUNG ||
			 maf_id == NAND_MFR_HYNIX))
		chip->bbt_options |= NAND_BBT_SCANLASTPAGE;
	else if ((nand_is_slc(chip) &&
				(maf_id == NAND_MFR_SAMSUNG ||
				 maf_id == NAND_MFR_HYNIX ||
				 maf_id == NAND_MFR_TOSHIBA ||
				 maf_id == NAND_MFR_AMD ||
				 maf_id == NAND_MFR_MACRONIX)) ||
			(mtd->writesize == 2048 &&
			 maf_id == NAND_MFR_MICRON))
		chip->bbt_options |= NAND_BBT_SCAN2NDPAGE;
}

static inline bool is_full_id_nand(struct nand_flash_dev *type)
{
	return type->id_len;
}

static bool find_full_id_nand(struct mtd_info *mtd, struct nand_chip *chip,
		   struct nand_flash_dev *type, const u8 *id_data, int *busw)
{
	if (!strncmp(type->id, id_data, type->id_len)) {
		mtd->writesize = type->pagesize;
		mtd->erasesize = type->erasesize;
		mtd->oobsize = type->oobsize;

		chip->bits_per_cell = nand_get_bits_per_cell(id_data[2]);
		chip->chipsize = (uint64_t)type->chipsize << 20;
		chip->options |= type->options;
		chip->ecc_strength_ds = NAND_ECC_STRENGTH(type);
		chip->ecc_step_ds = NAND_ECC_STEP(type);
		chip->onfi_timing_mode_default =
					type->onfi_timing_mode_default;

		*busw = type->options & NAND_BUSWIDTH_16;

		if (!mtd->name)
			mtd->name = type->name;

		return true;
	}
	return false;
}

/*
 * Print full detail of chip ID read from chip.
 */
static void print_nand_chip_info(int maf_id, int dev_id, u8 id_data[8])
{
	u8 delim[8] = { [0 ... 7] = ',' };
	pr_info("device found, Manufacturer ID: 0x%02x, Chip ID: 0x%02x\n", maf_id, dev_id);
	delim[7] = ' ';
	delim[nand_id_len(id_data, 8) - 1] = ';';
	/* This sucks. Kernel seems to insert newline after every other printk so format in one go. */
	pr_info("chip id data: 0x%02x%c 0x%02x%c 0x%02x%c 0x%02x%c 0x%02x%c 0x%02x%c 0x%02x%c 0x%02x%c\n",
		id_data[0], delim[0], id_data[1], delim[1], id_data[2], delim[2], id_data[3], delim[3],
		id_data[4], delim[4], id_data[5], delim[5], id_data[6], delim[6], id_data[7], delim[7]);
}

/*
 * Get the flash and manufacturer id and lookup if the type is supported.
 */
static struct nand_flash_dev *nand_get_flash_type(struct mtd_info *mtd,
						  struct nand_chip *chip,
						  int *maf_id, int *dev_id,
						  struct nand_flash_dev *type)
{
	int busw;
	int i, maf_idx;
	u8 id_data[8];

	/* Select the device */
	chip->select_chip(mtd, 0);

	/*
	 * Reset the chip, required by some chips (e.g. Micron MT29FxGxxxxx)
	 * after power-up.
	 */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* Send the command for reading device ID */
	chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);

	/* Read manufacturer and device IDs */
	*maf_id = chip->read_byte(mtd);
	*dev_id = chip->read_byte(mtd);

	/*
	 * Try again to make sure, as some systems the bus-hold or other
	 * interface concerns can cause random data which looks like a
	 * possibly credible NAND flash to appear. If the two results do
	 * not match, ignore the device completely.
	 */

	chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);

	/* Read entire ID string */
	for (i = 0; i < 8; i++)
		id_data[i] = chip->read_byte(mtd);

	if (id_data[0] != *maf_id || id_data[1] != *dev_id) {
		pr_info("second ID read did not match %02x,%02x against %02x,%02x\n",
			*maf_id, *dev_id, id_data[0], id_data[1]);
		return ERR_PTR(-ENODEV);
	}

	if (!type)
		type = nand_flash_ids;

	for (; type->name != NULL; type++) {
		if (is_full_id_nand(type)) {
			if (find_full_id_nand(mtd, chip, type, id_data, &busw))
				goto ident_done;
		} else if (*dev_id == type->dev_id) {
				break;
		}
	}

	chip->onfi_version = 0;
	if (!type->name || !type->pagesize) {
		/* Check if the chip is ONFI compliant */
		if (nand_flash_detect_onfi(mtd, chip, &busw))
			goto ident_done;

		/* Check if the chip is JEDEC compliant */
		if (nand_flash_detect_jedec(mtd, chip, &busw))
			goto ident_done;
	}

	if (!type->name)
		return ERR_PTR(-ENODEV);

	if (!mtd->name)
		mtd->name = type->name;

	chip->chipsize = (uint64_t)type->chipsize << 20;

	if (!type->pagesize && chip->init_size) {
		/* Set the pagesize, oobsize, erasesize by the driver */
		busw = chip->init_size(mtd, chip, id_data);
	} else if (!type->pagesize) {
		/* Decode parameters from extended ID */
		nand_decode_ext_id(mtd, chip, id_data, &busw);
	} else {
		nand_decode_id(mtd, chip, type, id_data, &busw);
	}
	/* Get chip options */
	chip->options |= type->options;

	/*
	 * Check if chip is not a Samsung device. Do not clear the
	 * options for chips which do not have an extended id.
	 */
	if (*maf_id != NAND_MFR_SAMSUNG && !type->pagesize)
		chip->options &= ~NAND_SAMSUNG_LP_OPTIONS;
ident_done:

	/* Try to identify manufacturer */
	for (maf_idx = 0; nand_manuf_ids[maf_idx].id != 0x0; maf_idx++) {
		if (nand_manuf_ids[maf_idx].id == *maf_id)
			break;
	}

	if (chip->options & NAND_BUSWIDTH_AUTO) {
		WARN_ON(chip->options & NAND_BUSWIDTH_16);
		chip->options |= busw;
		nand_set_defaults(chip, busw);
	} else if (busw != (chip->options & NAND_BUSWIDTH_16)) {
		/*
		 * Check, if buswidth is correct. Hardware drivers should set
		 * chip correct!
		 */
		print_nand_chip_info(*maf_id, *dev_id, id_data);
		pr_info("%s %s\n", nand_manuf_ids[maf_idx].name, mtd->name);
		pr_warn("bus width %d instead %d bit\n",
			   (chip->options & NAND_BUSWIDTH_16) ? 16 : 8,
			   busw ? 16 : 8);
		return ERR_PTR(-EINVAL);
	}

	nand_decode_bbm_options(mtd, chip, id_data);

	/* Calculate the address shift from the page size */
	chip->page_shift = ffs(mtd->writesize) - 1;
	/* Convert chipsize to number of pages per chip -1 */
	chip->pagemask = (chip->chipsize >> chip->page_shift) - 1;

	chip->bbt_erase_shift = chip->phys_erase_shift =
		ffs(mtd->erasesize) - 1;
	if (chip->chipsize & 0xffffffff)
		chip->chip_shift = ffs((unsigned)chip->chipsize) - 1;
	else {
		chip->chip_shift = ffs((unsigned)(chip->chipsize >> 32));
		chip->chip_shift += 32 - 1;
	}

	chip->badblockbits = 8;
	chip->erase = single_erase;

	/* Do not replace user supplied command function! */
	if (mtd->writesize > 512 && chip->cmdfunc == nand_command)
		chip->cmdfunc = nand_command_lp;

	if (nand_manuf_ids[maf_idx].init) {
		int err;
		err = nand_manuf_ids[maf_idx].init(mtd, id_data);
		if (err)
			return ERR_PTR(err);
	}

	print_nand_chip_info(*maf_id, *dev_id, id_data);

	if (chip->onfi_version)
		pr_info("%s %s\n", nand_manuf_ids[maf_idx].name,
				chip->onfi_params.model);
	else if (chip->jedec_version)
		pr_info("%s %s\n", nand_manuf_ids[maf_idx].name,
				chip->jedec_params.model);
	else
		pr_info("%s %s\n", nand_manuf_ids[maf_idx].name,
				type->name);

	pr_info("%d MiB, %s, erase size: %d KiB, page size: %d, OOB size: %d\n",
		(int)(chip->chipsize >> 20), nand_is_slc(chip) ? "SLC" : "MLC",
		mtd->erasesize >> 10, mtd->writesize, mtd->oobsize);
	return type;
}

/**
 * nand_scan_ident - [NAND Interface] Scan for the NAND device
 * @mtd: MTD device structure
 * @maxchips: number of chips to scan for
 * @table: alternative NAND ID table
 *
 * This is the first phase of the normal nand_scan() function. It reads the
 * flash ID and sets up MTD fields accordingly.
 *
 * The mtd->owner field must be set to the module of the caller.
 */
int nand_scan_ident(struct mtd_info *mtd, int maxchips,
		    struct nand_flash_dev *table)
{
	int i, nand_maf_id, nand_dev_id;
	struct nand_chip *chip = mtd->priv;
	struct nand_flash_dev *type;

	/* Set the default functions */
	nand_set_defaults(chip, chip->options & NAND_BUSWIDTH_16);

	/* Read the flash type */
	type = nand_get_flash_type(mtd, chip, &nand_maf_id,
				   &nand_dev_id, table);

	if (IS_ERR(type)) {
		if (!(chip->options & NAND_SCAN_SILENT_NODEV))
			pr_warn("No NAND device found\n");
		chip->select_chip(mtd, -1);
		return PTR_ERR(type);
	}

	chip->select_chip(mtd, -1);

	/* Check for a chip array */
	for (i = 1; i < maxchips; i++) {
		chip->select_chip(mtd, i);
		/* See comment in nand_get_flash_type for reset */
		chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
		/* Send the command for reading device ID */
		chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);
		/* Read manufacturer and device IDs */
		if (nand_maf_id != chip->read_byte(mtd) ||
		    nand_dev_id != chip->read_byte(mtd)) {
			chip->select_chip(mtd, -1);
			break;
		}
		chip->select_chip(mtd, -1);
	}
	if (i > 1)
		pr_info("%d chips detected\n", i);

	/* Store the number of chips and calc total size for mtd */
	chip->numchips = i;
	mtd->size = i * chip->chipsize;

	return 0;
}
EXPORT_SYMBOL(nand_scan_ident);

/*
 * Check if the chip configuration meet the datasheet requirements.

 * If our configuration corrects A bits per B bytes and the minimum
 * required correction level is X bits per Y bytes, then we must ensure
 * both of the following are true:
 *
 * (1) A / B >= X / Y
 * (2) A >= X
 *
 * Requirement (1) ensures we can correct for the required bitflip density.
 * Requirement (2) ensures we can correct even when all bitflips are clumped
 * in the same sector.
 */
static bool nand_ecc_strength_good(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	int corr, ds_corr;

	if (ecc->size == 0 || chip->ecc_step_ds == 0)
		/* Not enough information */
		return true;

	/*
	 * We get the number of corrected bits per page to compare
	 * the correction density.
	 */
	corr = (mtd->writesize * ecc->strength) / ecc->size;
	ds_corr = (mtd->writesize * chip->ecc_strength_ds) / chip->ecc_step_ds;

	return corr >= ds_corr && ecc->strength >= chip->ecc_strength_ds;
}

/*
 * Initialize ECC struct:
 *  - fill ECC struct with default function/values when these ones are undefined
 *  - fill ECC infos based on MTD device
 */
static int nand_ecc_ctrl_init(struct mtd_info *mtd, struct nand_ecc_ctrl *ecc)
{
	int i;

	/*
	 * If no default placement scheme is given, select an appropriate one.
	 */
	if (!ecc->layout && (ecc->mode != NAND_ECC_SOFT_BCH)) {
		switch (mtd->oobsize) {
		case 8:
			ecc->layout = &nand_oob_8;
			break;
		case 16:
			ecc->layout = &nand_oob_16;
			break;
		case 64:
			ecc->layout = &nand_oob_64;
			break;
		case 128:
			ecc->layout = &nand_oob_128;
			break;
		default:
			pr_warn("No oob scheme defined for oobsize %d\n",
				   mtd->oobsize);
			BUG();
		}
	}

	/*
	 * Check ECC mode, default to software if 3byte/512byte hardware ECC is
	 * selected and we have 256 byte pagesize fallback to software ECC
	 */

	switch (ecc->mode) {
	case NAND_ECC_HW_OOB_FIRST:
		/* Similar to NAND_ECC_HW, but a separate read_page handle */
		if (!ecc->calculate || !ecc->correct || !ecc->hwctl) {
			pr_warn("No ECC functions supplied; hardware ECC not possible\n");
			BUG();
		}
		if (!ecc->read_page)
			ecc->read_page = nand_read_page_hwecc_oob_first;

	case NAND_ECC_HW:
		/* Use standard hwecc read page function? */
		if (!ecc->read_page)
			ecc->read_page = nand_read_page_hwecc;
		if (!ecc->write_page)
			ecc->write_page = nand_write_page_hwecc;
		if (!ecc->read_page_raw)
			ecc->read_page_raw = nand_read_page_raw;
		if (!ecc->write_page_raw)
			ecc->write_page_raw = nand_write_page_raw;
		if (!ecc->read_oob)
			ecc->read_oob = nand_read_oob_std;
		if (!ecc->write_oob)
			ecc->write_oob = nand_write_oob_std;
		if (!ecc->read_subpage)
			ecc->read_subpage = nand_read_subpage;
		if (!ecc->write_subpage)
			ecc->write_subpage = nand_write_subpage_hwecc;

	case NAND_ECC_HW_SYNDROME:
		if ((!ecc->calculate || !ecc->correct || !ecc->hwctl) &&
		    (!ecc->read_page ||
		     ecc->read_page == nand_read_page_hwecc ||
		     !ecc->write_page ||
		     ecc->write_page == nand_write_page_hwecc)) {
			pr_warn("No ECC functions supplied; hardware ECC not possible\n");
			BUG();
		}
		/* Use standard syndrome read/write page function? */
		if (!ecc->read_page)
			ecc->read_page = nand_read_page_syndrome;
		if (!ecc->write_page)
			ecc->write_page = nand_write_page_syndrome;
		if (!ecc->read_page_raw)
			ecc->read_page_raw = nand_read_page_raw_syndrome;
		if (!ecc->write_page_raw)
			ecc->write_page_raw = nand_write_page_raw_syndrome;
		if (!ecc->read_oob)
			ecc->read_oob = nand_read_oob_syndrome;
		if (!ecc->write_oob)
			ecc->write_oob = nand_write_oob_syndrome;

		if (mtd->writesize >= ecc->size) {
			if (!ecc->strength) {
				pr_warn("Driver must set ecc.strength when using hardware ECC\n");
				BUG();
			}
			break;
		}
		pr_warn("%d byte HW ECC not possible on %d byte page size, fallback to SW ECC\n",
			ecc->size, mtd->writesize);
		ecc->mode = NAND_ECC_SOFT;

	case NAND_ECC_SOFT:
		ecc->calculate = nand_calculate_ecc;
		ecc->correct = nand_correct_data;
		ecc->read_page = nand_read_page_swecc;
		ecc->read_subpage = nand_read_subpage;
		ecc->write_page = nand_write_page_swecc;
		ecc->read_page_raw = nand_read_page_raw;
		ecc->write_page_raw = nand_write_page_raw;
		ecc->read_oob = nand_read_oob_std;
		ecc->write_oob = nand_write_oob_std;
		if (!ecc->size)
			ecc->size = 256;
		ecc->bytes = 3;
		ecc->strength = 1;
		break;

	case NAND_ECC_SOFT_BCH:
		if (!mtd_nand_has_bch()) {
			pr_warn("CONFIG_MTD_NAND_ECC_BCH not enabled\n");
			BUG();
		}
		ecc->calculate = nand_bch_calculate_ecc;
		ecc->correct = nand_bch_correct_data;
		ecc->read_page = nand_read_page_swecc;
		ecc->read_subpage = nand_read_subpage;
		ecc->write_page = nand_write_page_swecc;
		ecc->read_page_raw = nand_read_page_raw;
		ecc->write_page_raw = nand_write_page_raw;
		ecc->read_oob = nand_read_oob_std;
		ecc->write_oob = nand_write_oob_std;
		/*
		 * Board driver should supply ecc.size and ecc.bytes values to
		 * select how many bits are correctable; see nand_bch_init()
		 * for details. Otherwise, default to 4 bits for large page
		 * devices.
		 */
		if (!ecc->size && (mtd->oobsize >= 64)) {
			ecc->size = 512;
			ecc->bytes = DIV_ROUND_UP(13 * ecc->strength, 8);
		}
		ecc->priv = nand_bch_init(mtd, ecc->size, ecc->bytes,
					       &ecc->layout);
		if (!ecc->priv) {
			pr_warn("BCH ECC initialization failed!\n");
			BUG();
		}
		ecc->strength = ecc->bytes * 8 / fls(8 * ecc->size);
		break;

	case NAND_ECC_NONE:
		pr_warn("NAND_ECC_NONE selected by board driver. This is not recommended!\n");
		ecc->read_page = nand_read_page_raw;
		ecc->write_page = nand_write_page_raw;
		ecc->read_oob = nand_read_oob_std;
		ecc->read_page_raw = nand_read_page_raw;
		ecc->write_page_raw = nand_write_page_raw;
		ecc->write_oob = nand_write_oob_std;
		ecc->size = mtd->writesize;
		ecc->bytes = 0;
		ecc->strength = 0;
		break;

	default:
		pr_warn("Invalid NAND_ECC_MODE %d\n", ecc->mode);
		BUG();
	}

	/* For many systems, the standard OOB write also works for raw */
	if (!ecc->read_oob_raw)
		ecc->read_oob_raw = ecc->read_oob;
	if (!ecc->write_oob_raw)
		ecc->write_oob_raw = ecc->write_oob;

	/*
	 * The number of bytes available for a client to place data into
	 * the out of band area.
	 */
	ecc->layout->oobavail = 0;
	for (i = 0; ecc->layout->oobfree[i].length
			&& i < ARRAY_SIZE(ecc->layout->oobfree); i++)
		ecc->layout->oobavail += ecc->layout->oobfree[i].length;
	mtd->oobavail = ecc->layout->oobavail;

	/* ECC sanity check: warn if it's too weak */
	if (!nand_ecc_strength_good(mtd))
		pr_warn("WARNING: %s: the ECC used on your system is too weak compared to the one required by the NAND chip\n",
			mtd->name);

	/*
	 * Set the number of read / write steps for one page depending on ECC
	 * mode.
	 */
	ecc->steps = mtd->writesize / ecc->size;
	if (ecc->steps * ecc->size != mtd->writesize) {
		pr_warn("Invalid ECC parameters\n");
		BUG();
	}
	ecc->total = ecc->steps * ecc->bytes;

	return 0;
}

/**
 * nand_add_partition - [NAND Interface] Add a NAND partition to a NAND device
 * @master: MTD device structure representing the NAND device
 * @part: NAND partition to add to the NAND device
 *
 * Adds a NAND partition to a NAND device.
 * The NAND partition cannot overlap with another existing partition.
 *
 * Returns zero in case of success and a negative error code in case of failure.
 */
int nand_add_partition(struct mtd_info *master, struct nand_part *part)
{
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct nand_ecc_ctrl *ecc;
	struct nand_part *pos;
	bool inserted = false;
	int ret;

	BUG_ON(!master || !part);

	chip = master->priv;
	mtd = &part->mtd;
	ecc = part->ecc;

	DBG("chip %p, mtd %p, ecc %p", chip, mtd, ecc);

	/* set up the MTD object for this partition */
	mtd->type = master->type;
	mtd->flags = master->flags & ~mtd->flags;
	mtd->subpage_sft = master->subpage_sft;
	if (!mtd->writesize) {
		mtd->writesize = master->writesize;
		mtd->writebufsize = master->writebufsize;
	}
	else
		mtd->writebufsize = mtd->writesize;

	DBG("writesize %u, writebufsize %u, subpage_sft %u",
	    mtd->writesize, mtd->writebufsize, mtd->subpage_sft);

	mtd->oobsize = master->oobsize;
	mtd->oobavail = master->oobavail;
	mtd->erasesize = master->erasesize;

	mtd->priv = chip;
	mtd->owner = master->owner;
	mtd->backing_dev_info = master->backing_dev_info;

	mtd->dev.parent = master->dev.parent;

	if (ecc) {
		ret = nand_ecc_ctrl_init(mtd, ecc);
		if (ret)
			return ret;
	} else {
		ecc = &chip->ecc;
	}

	mtd->_erase = nand_part_erase;
	mtd->_point = NULL;
	mtd->_unpoint = NULL;
	mtd->_read = nand_part_read;
	mtd->_write = nand_part_write;
	mtd->_panic_write = panic_nand_part_write;
	mtd->_read_oob = nand_part_read_oob;
	mtd->_write_oob = nand_part_write_oob;
	mtd->_sync = nand_sync;
	mtd->_lock = NULL;
	mtd->_unlock = NULL;
	mtd->_suspend = nand_suspend;
	mtd->_resume = nand_resume;
	mtd->_block_isbad = nand_part_block_isbad;
	mtd->_block_markbad = nand_part_block_markbad;

	/* propagate ecc info to mtd_info */
	mtd->ecclayout = ecc->layout;
	mtd->ecc_strength = ecc->strength;
	mtd->ecc_step_size = ecc->size;
	/*
	 * Initialize bitflip_threshold to its default prior scan_bbt() call.
	 * scan_bbt() might invoke mtd_read(), thus bitflip_threshold must be
	 * properly set.
	 */
	if (!mtd->bitflip_threshold)
		mtd->bitflip_threshold = mtd->ecc_strength;

	part->master = master;

	mutex_lock(&chip->part_lock);
	list_for_each_entry(pos, &chip->partitions, node) {
		DBG("pos %p, partitions %p",
		    (void*) pos, &chip->partitions);

		DBG("pos: offset %llx, size %llx",
		    pos->offset, pos->mtd.size);

		if (part->offset >= pos->offset + pos->mtd.size) {
			continue;
		} else if (part->offset + mtd->size > pos->offset) {
			ret = -EINVAL;
			goto out;
		}

		DBG("node %p, prev %p", &part->node, pos->node.prev);

		list_add(&part->node, pos->node.prev);
		inserted = true;
		break;
	}

	if (!inserted) {
		DBG("adding partition node %p", &part->node);
		list_add_tail(&part->node, &chip->partitions);
		DBG("partition added");
	}

	ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		DBG("mtd_device_register ERROR %d", ret);
		list_del(&part->node);
		goto out;
	}

	if (master->_block_isbad) {
		uint64_t offs = 0;

		DBG("Bad block check. Offset %llx, MTD size %llx, erasesize %x",
		    part->offset, mtd->size, mtd->erasesize);

		while (offs < mtd->size) {
			if (mtd_block_isreserved(master, offs + part->offset))
				mtd->ecc_stats.bbtblocks++;
			else if (mtd_block_isbad(master, offs + part->offset))
				mtd->ecc_stats.badblocks++;
			offs += mtd->erasesize;
		}
	}
	DBG("SUCCESS. bbtblocks %u, badblocks %u",
	    mtd->ecc_stats.bbtblocks,
	    mtd->ecc_stats.badblocks);

out:
	mutex_unlock(&chip->part_lock);
	return ret;
}
EXPORT_SYMBOL(nand_add_partition);

/**
 * nand_del_partition - [NAND Interface] Delete a NAND part from a NAND dev
 * @part: NAND partition to delete
 *
 * Deletes a NAND partition from a NAND device.
 */
void nand_del_partition(struct nand_part *part)
{
	struct nand_chip *chip;

	BUG_ON(!part);
	chip = part->mtd.priv;
	mutex_lock(&chip->part_lock);
	mtd_device_unregister(&part->mtd);
	list_del(&part->node);
	mutex_unlock(&chip->part_lock);

	if (part->ecc && part->ecc->mode == NAND_ECC_SOFT_BCH)
		nand_bch_free((struct nand_bch_control *)part->ecc->priv);

	if (part->release)
		part->release(part);
}
EXPORT_SYMBOL(nand_del_partition);

/*
 * NAND part release function. Used by nandpart_alloc as its release function.
 */
static void nandpart_release(struct nand_part *part)
{
	BUG_ON(!part);
	DBG("part name %s", part->mtd.name);

	kfree(part);
}

/**
 * nandpart_alloc - [NAND Interface] Allocate a NAND part struct
 *
 * Allocate a NAND partition and assign the nandpart release function.
 * This nand_part struct must be filled before passing it to the
 * nand_add_partition function.
 */
struct nand_part *nandpart_alloc(void)
{
	struct nand_part *part = kzalloc(sizeof(*part), GFP_KERNEL);
	if (!part)
		return ERR_PTR(-ENOMEM);
	part->release = nandpart_release;

	return part;
}
EXPORT_SYMBOL(nandpart_alloc);

/**
 * nand_scan_tail - [NAND Interface] Scan for the NAND device
 * @mtd: MTD device structure
 *
 * This is the second phase of the normal nand_scan() function. It fills out
 * all the uninitialized function pointers with the defaults and scans for a
 * bad block table if appropriate.
 */
int nand_scan_tail(struct mtd_info *mtd)
{
	int ret;
	struct nand_chip *chip = mtd->priv;
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	struct nand_buffers *nbuf;

	/* New bad blocks should be marked in OOB, flash-based BBT, or both */
	BUG_ON((chip->bbt_options & NAND_BBT_NO_OOB_BBM) &&
			!(chip->bbt_options & NAND_BBT_USE_FLASH));

	if (!(chip->options & NAND_OWN_BUFFERS)) {
		uint32_t sz = sizeof(*nbuf) + mtd->writesize + mtd->oobsize * 3;
		DBG("new buffer of size %u for mtd %p", sz, mtd);
		nbuf = kzalloc(sz, GFP_KERNEL);
		if (!nbuf)
			return -ENOMEM;
		nbuf->ecccalc = (uint8_t *)(nbuf + 1);
		nbuf->ecccode = nbuf->ecccalc + mtd->oobsize;
		nbuf->databuf = nbuf->ecccode + mtd->oobsize;
		chip->buffers = nbuf;
	} else {
		if (!chip->buffers)
			return -ENOMEM;
	}

	/* Set the internal oob buffer location, just after the page data */
	chip->oob_poi = chip->buffers->databuf + mtd->writesize;

	if (!chip->write_page)
		chip->write_page = nand_write_page;

	/* Initialize ECC struct */
	ret = nand_ecc_ctrl_init(mtd, ecc);
	if (ret) {
		if (!(chip->options & NAND_OWN_BUFFERS)) {
			DBG("freeing buffers of mtd %p", mtd);
			kfree(chip->buffers);
		}
		return ret;
	}

	INIT_LIST_HEAD(&chip->partitions);
	mutex_init(&chip->part_lock);

	chip->cur_ecc = &chip->ecc;
	chip->cur_rnd = &chip->rnd;

	/* Allow subpage writes up to ecc.steps. Not possible for MLC flash */
	if (!(chip->options & NAND_NO_SUBPAGE_WRITE) && nand_is_slc(chip)) {
		switch (ecc->steps) {
		case 2:
			mtd->subpage_sft = 1;
			break;
		case 4:
		case 8:
		case 16:
			mtd->subpage_sft = 2;
			break;
		}
	}
	chip->subpagesize = mtd->writesize >> mtd->subpage_sft;

	/* Initialize state */
	chip->state = FL_READY;

	/* Invalidate the pagebuffer reference */
	chip->pagebuf = -1;

	/* Large page NAND with SOFT_ECC should support subpage reads */
	switch (ecc->mode) {
	case NAND_ECC_SOFT:
	case NAND_ECC_SOFT_BCH:
		if (chip->page_shift > 9)
			chip->options |= NAND_SUBPAGE_READ;
		break;

	default:
		break;
	}

	/* Fill in remaining MTD driver data */
	mtd->type = nand_is_slc(chip) ? MTD_NANDFLASH : MTD_MLCNANDFLASH;
	mtd->flags = (chip->options & NAND_ROM) ? MTD_CAP_ROM :
						MTD_CAP_NANDFLASH;
	mtd->_erase = nand_erase;
	mtd->_point = NULL;
	mtd->_unpoint = NULL;
	mtd->_read = nand_read;
	mtd->_write = nand_write;
	mtd->_panic_write = panic_nand_write;
	mtd->_read_oob = nand_read_oob;
	mtd->_write_oob = nand_write_oob;
	mtd->_sync = nand_sync;
	mtd->_lock = NULL;
	mtd->_unlock = NULL;
	mtd->_suspend = nand_suspend;
	mtd->_resume = nand_resume;
	mtd->_block_isreserved = nand_block_isreserved;
	mtd->_block_isbad = nand_block_isbad;
	mtd->_block_markbad = nand_block_markbad;
	mtd->writebufsize = mtd->writesize;

	/* propagate ecc info to mtd_info */
	mtd->ecclayout = ecc->layout;
	mtd->ecc_strength = ecc->strength;
	mtd->ecc_step_size = ecc->size;
	/*
	 * Initialize bitflip_threshold to its default prior scan_bbt() call.
	 * scan_bbt() might invoke mtd_read(), thus bitflip_threshold must be
	 * properly set.
	 */
	if (!mtd->bitflip_threshold)
		mtd->bitflip_threshold = mtd->ecc_strength;

	/* Check, if we should skip the bad block table scan */
	if (chip->options & NAND_SKIP_BBTSCAN)
		return 0;

	/* Build bad block table */
	return chip->scan_bbt(mtd);
}
EXPORT_SYMBOL(nand_scan_tail);

/*
 * is_module_text_address() isn't exported, and it's mostly a pointless
 * test if this is a module _anyway_ -- they'd have to try _really_ hard
 * to call us from in-kernel code if the core NAND support is modular.
 */
#ifdef MODULE
#define caller_is_module() (1)
#else
#define caller_is_module() \
	is_module_text_address((unsigned long)__builtin_return_address(0))
#endif

/**
 * nand_scan - [NAND Interface] Scan for the NAND device
 * @mtd: MTD device structure
 * @maxchips: number of chips to scan for
 *
 * This fills out all the uninitialized function pointers with the defaults.
 * The flash ID is read and the mtd/chip structures are filled with the
 * appropriate values. The mtd->owner field must be set to the module of the
 * caller.
 */
int nand_scan(struct mtd_info *mtd, int maxchips)
{
	int ret;

	/* Many callers got this wrong, so check for it for a while... */
	if (!mtd->owner && caller_is_module()) {
		pr_crit("%s called with NULL mtd->owner!\n", __func__);
		BUG();
	}

	ret = nand_scan_ident(mtd, maxchips, NULL);
	if (!ret)
		ret = nand_scan_tail(mtd);
	return ret;
}
EXPORT_SYMBOL(nand_scan);

/**
 * nand_release - [NAND Interface] Free resources held by the NAND device
 * @mtd: MTD device structure
 */
void nand_release(struct mtd_info *mtd)
{
	struct nand_chip *chip;

	BUG_ON(!mtd || !mtd->priv);

	chip = mtd->priv;

	if (chip->ecc.mode == NAND_ECC_SOFT_BCH)
		nand_bch_free((struct nand_bch_control *)chip->ecc.priv);

	mtd_device_unregister(mtd);

	/* Free bad block table memory */
	if (chip->bbt) {
		DBG("freeing BBT %p", chip->bbt);
		kfree(chip->bbt);
	}

	if (chip->buffers && !(chip->options & NAND_OWN_BUFFERS)) {
		DBG("freeing buffers %p", chip->buffers);
		kfree(chip->buffers);
	}

	/* Free bad block descriptor memory */
	if (chip->badblock_pattern && chip->badblock_pattern->options
	    & NAND_BBT_DYNAMICSTRUCT) {
		DBG("freeing BB pattern %p", chip->badblock_pattern);
		kfree(chip->badblock_pattern);
	}
}
EXPORT_SYMBOL_GPL(nand_release);

static int __init nand_base_init(void)
{
	led_trigger_register_simple("nand-disk", &nand_led_trigger);
	return 0;
}

static void __exit nand_base_exit(void)
{
	led_trigger_unregister_simple(nand_led_trigger);
}

module_init(nand_base_init);
module_exit(nand_base_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steven J. Hill <sjhill@realitydiluted.com>");
MODULE_AUTHOR("Thomas Gleixner <tglx@linutronix.de>");
MODULE_DESCRIPTION("Generic NAND flash driver code");
