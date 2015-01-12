/*
 * Copyright (C) 2014 Boris BREZILLON <b.brezillon.dev@gmail.com>
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
 */

#include <linux/module.h>
#include <linux/mtd/nand.h>
#include <linux/slab.h>

#define DBG(fmt, arg...) pr_info(pr_fmt("%s: " fmt "\n"), __FUNCTION__, ##arg)

static u8 h27ucg8t2a_read_retry_regs[] = {
	0xcc, 0xbf, 0xaa, 0xab, 0xcd, 0xad, 0xae, 0xaf
};

static u8 h27ucg8t2e_read_retry_regs[] = {
	0x38, 0x39, 0x3a, 0x3b
};

struct hynix_read_retry {
	u8 regnum;      // number of registers to set on each RR step
	u8 *regs;       // array of register addresses
	u8 values[64];  // RR values to be written into the RR registers
};

struct hynix_nand {
	struct hynix_read_retry read_retry;
};

int nand_setup_read_retry_hynix(struct mtd_info *mtd, int retry_count)
{
	struct nand_chip *chip = mtd->priv;
	struct hynix_nand *hynix = chip->manuf_priv;
	int offset = retry_count * hynix->read_retry.regnum;
	int status;
	int i;

	DBG("%d", retry_count);
	chip->cmdfunc(mtd, 0x36, -1, -1);
	for (i = 0; i < hynix->read_retry.regnum; i++) {
		int column = hynix->read_retry.regs[i];
		column |= column << 8;
		chip->cmdfunc(mtd, NAND_CMD_NONE, column, -1);
		chip->write_byte(mtd, hynix->read_retry.values[offset + i]);
	}
	chip->cmdfunc(mtd, 0x16, -1, -1);

	status = chip->waitfunc(mtd, chip);
	if (status & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
}

static void h27ucg8t2a_cleanup(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	kfree(chip->manuf_priv);
}

static int h27ucg8t2a_init(struct mtd_info *mtd, const uint8_t *id)
{
	struct nand_chip *chip = mtd->priv;
	struct hynix_nand *hynix;
	u8 buf[1024];
	int i, j;
	int ret;

	chip->select_chip(mtd, 0);
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
	chip->cmdfunc(mtd, 0x36, 0xff, -1);
	chip->write_byte(mtd, 0x40);
	chip->cmdfunc(mtd, NAND_CMD_NONE, 0xcc, -1);
	chip->write_byte(mtd, 0x4d);
	chip->cmdfunc(mtd, 0x16, -1, -1);
	chip->cmdfunc(mtd, 0x17, -1, -1);
	chip->cmdfunc(mtd, 0x04, -1, -1);
	chip->cmdfunc(mtd, 0x19, -1, -1);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0x0, 0x200);

	chip->read_buf(mtd, buf, 2);
	if (buf[0] != 0x8 || buf[1] != 0x8)
		return -EINVAL;
	chip->read_buf(mtd, buf, 1024);

	ret = 0;
	for (j = 0; j < 8; j++) {
		for (i = 0; i < 64; i++) {
			u8 *tmp = buf + (128 * j);
			if ((tmp[i] | tmp[i + 64]) != 0xff) {
				ret = -EINVAL;
				break;
			}
		}

		if (ret)
			break;
	}

	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
	chip->cmdfunc(mtd, 0x38, -1, -1);
	chip->select_chip(mtd, -1);

	if (!ret) {
		hynix = kzalloc(sizeof(*hynix), GFP_KERNEL);
		if (!hynix)
			return -ENOMEM;

		hynix->read_retry.regs = h27ucg8t2a_read_retry_regs;
		memcpy(hynix->read_retry.values, buf, 64);
		chip->manuf_priv = hynix;
		chip->setup_read_retry = nand_setup_read_retry_hynix;
		chip->read_retries = 8;
		chip->manuf_cleanup = h27ucg8t2a_cleanup;
	}

	return ret;
}

static void h27ucg8t2e_cleanup(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	kfree(chip->manuf_priv);
}

#define UCG8T2E_RRT_OTP_SIZE 528

static int h27ucg8t2e_init(struct mtd_info *mtd, const uint8_t *id)
{
	struct nand_chip *chip = mtd->priv;
	struct hynix_nand *hynix;
	u8 rrtOTP[UCG8T2E_RRT_OTP_SIZE];
	int rrtReg, rrtSet, i;
	int ret;

	DBG("mtd %p, ID %.2x %.2x %.2x %.2x %.2x %.2x",
	    mtd, id[0], id[1], id[2], id[3], id[4], id[5]);

	chip->select_chip(mtd, 0);
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* copy RRT from OTP to rrtOTP */
	chip->cmdfunc(mtd, 0x36, 0x38, -1);
	chip->write_byte(mtd, 0x52);
	chip->cmdfunc(mtd, 0x16, -1, -1);
	chip->cmdfunc(mtd, 0x17, -1, -1);
	chip->cmdfunc(mtd, 0x04, -1, -1);
	chip->cmdfunc(mtd, 0x19, -1, -1);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, 0x200);

	/*
	 *  1. Read total RR count - 1 byte x 8 times - and RR register count -
	 *  also 1 byte x 8 times.
	 *
	 *  2. Read 8 RR register sets, each consisting of a 32-byte original
	 *  and its 32-byte inverse copy. Every set stores 8 lists of values for
	 *  the 4 RR registers on the flash chip.
	 */
	chip->read_buf(mtd, rrtOTP, UCG8T2E_RRT_OTP_SIZE);
	printk("RR count (8 copies), RR reg. count (8 copies):\n");
	for (i = 0; i < 16; i++) {
		printk(" %.2x", rrtOTP[i]);
	}

	if (rrtOTP[0] != 8 || rrtOTP[1] != 8) {
		DBG("wrong total RR count %u (%u)", rrtOTP[0], rrtOTP[1]);
		return -EINVAL;
	}

//	chip->read_buf(mtd, rrtOTP, 512);

	/* copy RRT from OTP, command suffix */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
	chip->cmdfunc(mtd, 0x36, 0x38, -1);
	chip->write_byte(mtd, 0);

	/* dummy read from any address */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, 0);

	chip->select_chip(mtd, -1);

	/* FIXME: common function - majority check, not "all correct" */
	ret = 0;
	printk("RRT sets in OTP: original (inverse)...\n");
	for (rrtSet = 0; rrtSet < 8; rrtSet++) {
		u8 *cur = rrtOTP + 16 + (64 * rrtSet);
		for (rrtReg = 0; rrtReg < 32; rrtReg++) {
			uint8_t original = cur[rrtReg];
			uint8_t inverse  = cur[rrtReg + 32];
			printk(" %.2x (%.2x)", original, inverse);
			if ((original | inverse) != 0xff) {
				ret = -EINVAL;
				break;
			}
		}
		printk("\n");

		if (ret)
			// read the next set
			continue;
		else
			// current set is correct
			break;
	}

	if (ret) {
		DBG("Loading of RRT from OTP failed. Using defaults.");

		hynix = kzalloc(sizeof(*hynix), GFP_KERNEL);
		if (!hynix)
			return -ENOMEM;

		hynix->read_retry.regs = h27ucg8t2e_read_retry_regs;
		hynix->read_retry.regnum = 4;

		// copy the first correct RRT set (the original half)
		memcpy(hynix->read_retry.values, &rrtOTP[rrtSet * 64], 32);

		chip->manuf_priv = hynix;
		chip->setup_read_retry = nand_setup_read_retry_hynix;
		chip->read_retries = 8;
		chip->manuf_cleanup = h27ucg8t2e_cleanup;
	}

	return ret;
}

struct hynix_nand_initializer {
	u8 id[6];
	int (*init)(struct mtd_info *mtd, const uint8_t *id);
};

struct hynix_nand_initializer initializers[] = {
	{
		.id = {NAND_MFR_HYNIX, 0xde, 0x94, 0xda, 0x74, 0xc4},
		.init = h27ucg8t2a_init,
	},
	{
		.id = {NAND_MFR_HYNIX, 0xde, 0x14, 0xa7, 0x42, 0x4a},
		.init = h27ucg8t2e_init,
	},
};

int hynix_nand_init(struct mtd_info *mtd, const uint8_t *id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(initializers); i++) {
		struct hynix_nand_initializer *initializer = &initializers[i];
		if (memcmp(id, initializer->id, sizeof(initializer->id)))
			continue;

		return initializer->init(mtd, id);
	}

	return 0;
}
EXPORT_SYMBOL(hynix_nand_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Boris BREZILLON <b.brezillon.dev@gmail.com>");
MODULE_DESCRIPTION("Hynix NAND specific code");
