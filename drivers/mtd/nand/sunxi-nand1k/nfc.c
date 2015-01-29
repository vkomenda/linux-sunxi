/*
 * nfc.c
 *
 * Copyright (C) 2013 Qiang Yu <yuq825@gmail.com>
 *               2015 Vladimir Komendantskiy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <plat/sys_config.h>

#include "defs.h"
#include "regs.h"
#include "dma.h"
#include "nand_id.h"

// do we need to consider exclusion of offset?
// it should be in high level that the nand_chip ops have been
// performed with exclusion already
// find it is:
//   nand_get_device() & nand_release_device()
//
static int read_offset = 0;
static int write_offset = 0;
static u8 *read_buffer = NULL;
static u8 *write_buffer = NULL;
static int buffer_size = 16384 + 2048;
static dma_addr_t read_buffer_dma;
static dma_addr_t write_buffer_dma;
static int dma_hdle = 0;
static struct nand_ecclayout sunxi_ecclayout;
static DECLARE_WAIT_QUEUE_HEAD(nand_rb_wait);
static int program_column = -1, program_page = -1;

// special initialisation-time mode for reading the OTP area
static uint8_t otp_mode = 0;

unsigned int hwecc_switch = 1;
module_param(hwecc_switch, uint, 0);
MODULE_PARM_DESC(hwecc_switch, "hardware ECC switch: 1=on, 0=off");

unsigned int bbt_use_flash = 1;
module_param(bbt_use_flash, uint, 0);
MODULE_PARM_DESC(bbt_use_flash, "flash bad block table placement: 1=flash, 0=RAM");

unsigned int invalid_bbm = 0;
module_param(invalid_bbm, uint, 0);
MODULE_PARM_DESC(invalid_bbm, "skip non-0 bad block markers: 1=yes, 0=no");

unsigned int random_switch = 1;
module_param(random_switch, uint, 0);
MODULE_PARM_DESC(random_switch, "random read/write switch: 1=on, 0=off");

/* This fixed seed should be used for writing the NAND SPL bootloader. */
#define AW_RND_SEED 0x4a80

unsigned int fixed_random_seed = 0;
module_param(fixed_random_seed, uint, 0);
MODULE_PARM_DESC(fixed_random_seed, "random seed: default=0 (pseudorandom modulo page)");

unsigned int debug = 0;
module_param(debug, uint, 0);
MODULE_PARM_DESC(debug, "debug output: 1=on, 0=off");

struct reg_info {
	u32    addr;
	char   name[18];
	size_t len;      /* Length of the register in 32-bit words. */
};

static struct reg_info nfc_regs[] = {
	{NFC_REG_CTL,           "NFC_CTL",           1},
	{NFC_REG_ST,            "NFC_ST",            1},
	{NFC_REG_INT,           "NFC_INT",           1},
	{NFC_REG_TIMING_CTL,    "NFC_TIMING_CTL",    1},
	{NFC_REG_TIMING_CFG,    "NFC_TIMING_CFG",    1},
	{NFC_REG_ADDR_LOW,      "NFC_ADDR_LOW",      1},
	{NFC_REG_ADDR_HIGH,     "NFC_ADDR_HIGH",     1},
	{NFC_REG_SECTOR_NUM,    "NFC_SECTOR_NUM",    1},
	{NFC_REG_CNT,           "NFC_CNT",           1},
	{NFC_REG_CMD,           "NFC_CMD",           1},
	{NFC_REG_RCMD_SET,      "NFC_READ_CMD_SET",  1},
	{NFC_REG_WCMD_SET,      "NFC_WRITE_CMD_SET", 1},
	{NFC_REG_IO_DATA,       "NFC_IO_DATA",       1},
	{NFC_REG_ECC_CTL,       "NFC_ECC_CTL",       1},
	{NFC_REG_ECC_ST,        "NFC_ECC_ST",        1},
	{NFC_REG_DEBUG,         "NFC_DEBUG",         1},
	{NFC_REG_ECC_CNT0,      "NFC_ECC_CNT0",      1},
	{NFC_REG_ECC_CNT1,      "NFC_ECC_CNT1",      1},
	{NFC_REG_ECC_CNT2,      "NFC_ECC_CNT2",      1},
	{NFC_REG_ECC_CNT3,      "NFC_ECC_CNT3",      1},
	{NFC_REG_USER_DATA(0),  "NFC_USER_DB",      16},
	{NFC_REG_SPARE_AREA,    "NFC_SPARE_AREA",    1},
	{NFC_RAM0_BASE,         "NFC_RAM0",        256},
	{NFC_RAM1_BASE,         "NFC_RAM1",        256},
	{0,                     "",                  0}
};

static void print_reg(struct reg_info* reg)
{
	int i, j;

	printk("%s =", reg->name);
	if (reg->len < 4) {
		for (j = 0; j < reg->len; j += 4)
			printk(" %.8x", readl(reg->addr + j));
		printk("\n");
	}
	else {
		printk("\n");
		for (i = 0; i < reg->len / 8 || i == 0; i++) {
			printk("%.8x:", reg->addr + i * 8);
			for (j = 0; j < 8; j++)
				printk(" %.8x", readl(reg->addr + i * 8 + j * 4));
			printk("\n");
		}
	}
}

static void print_regs(void)
{
	int i;
	printk(" === NFC register dump ===\n");
	for (i = 0; nfc_regs[i].addr != 0; i++)
		print_reg(&nfc_regs[i]);
}

//////////////////////////////////////////////////////////////////
// SUNXI platform
//

// Get clock rate from PLL5
static u32 sunxi_get_pll5_clk(void)
{
	u32 reg_val;
	u32 div_p, factor_n;
	u32 factor_k, factor_m;
	u32 clock;

	reg_val  = readl(PLL5_CFG_REG);
	div_p    = (reg_val & PLL5_OUT_EXT_DIV_P_MASK) >> PLL5_OUT_EXT_DIV_P_SHIFT;
	factor_n = (reg_val & PLL5_FACTOR_N_MASK) >> PLL5_FACTOR_N_SHIFT;
	factor_k = ((reg_val & PLL5_FACTOR_K_MASK) >> PLL5_FACTOR_K_SHIFT) + 1;
	factor_m = ((reg_val & PLL5_FACTOR_M_MASK) >> PLL5_FACTOR_M_SHIFT) + 1;

	clock = 24 * factor_n * factor_k / div_p / factor_m;
	DBG("pll5 %d MHz", clock);

	return clock;
}

static void sunxi_set_nand_clock(u32 nand_max_clock)
{
	u32 edo_clk, cmu_clk;
	u32 cfg;
	u32 nand_clk_divid_ratio;

	DBG("clock %u MHz", nand_max_clock);

	// open ahb nand clk (bus clock for CPU access)
	cfg = readl(AHB_GATING_REG0);
	cfg |= 1 << AHB_GATING_NAND_CLK_SHIFT;
	writel(cfg, AHB_GATING_REG0);

	// set nand clock (device clock for NFC running)
	edo_clk = nand_max_clock * 2;

	cmu_clk = sunxi_get_pll5_clk();
	nand_clk_divid_ratio = cmu_clk / edo_clk;
	if (cmu_clk % edo_clk)
		nand_clk_divid_ratio++;
	if (nand_clk_divid_ratio) {
		if (nand_clk_divid_ratio > 16)
			nand_clk_divid_ratio = 15;
		else
			nand_clk_divid_ratio--;
	}

	// set nand clock gate on
	cfg = readl(NAND_SCLK_CFG_REG);
	// gate on nand clock
	cfg |= 1 << SCLK_GATING_SHIFT;
	// take cmu pll as nand src block
	cfg &= ~CLK_SRC_SEL_MASK;
	cfg |= 0x2 << CLK_SRC_SEL_SHIFT;
	// set divn = 0
	cfg &= ~CLK_DIV_RATIO_N_MASK;
	// set divm
	cfg &= ~CLK_DIV_RATIO_M_MASK;
	cfg |= (nand_clk_divid_ratio << CLK_DIV_RATIO_M_SHIFT) & CLK_DIV_RATIO_M_MASK;
	writel(cfg, NAND_SCLK_CFG_REG);

	DBG("AHB_GATING_REG0   %.8x", readl(AHB_GATING_REG0));
	DBG("NAND_SCLK_CFG_REG %.8x", readl(NAND_SCLK_CFG_REG));
}

static void release_nand_clock(void)
{
	u32 cfg;

	DBG("");
	// disable bus clock
	cfg = readl(AHB_GATING_REG0);
	cfg &= ~(1 << AHB_GATING_NAND_CLK_SHIFT);
	writel(cfg, AHB_GATING_REG0);

	// disable device clock
	cfg = readl(NAND_SCLK_CFG_REG);
	cfg &= ~(1 << SCLK_GATING_SHIFT);
	writel(cfg, NAND_SCLK_CFG_REG);
}

u32 pioc_handle;

// Set PIOC pin for NAND Flash use
static void sunxi_set_nand_pio(void)
{
	pioc_handle = gpio_request_ex("nand_para", NULL);
}

static void sunxi_release_nand_pio(void)
{
	gpio_release(pioc_handle, 1);
}

/////////////////////////////////////////////////////////////////
// Utils
//

static inline void wait_cmdfifo_free(void)
{
	int timeout = 0xffff;

	while ((timeout--) && (readl(NFC_REG_ST) & NFC_CMD_FIFO_STATUS));
	if (timeout <= 0) {
		pr_err(pr_fmt("%s: timeout"), __FUNCTION__);
	}
}

static inline void wait_cmd_finish(void)
{
	int timeout = 0xffff;

	while((timeout--) && !(readl(NFC_REG_ST) & NFC_CMD_INT_FLAG));
	if (timeout <= 0) {
		pr_err(pr_fmt("%s: timeout"), __FUNCTION__);
		return;
	}
	writel(NFC_CMD_INT_FLAG, NFC_REG_ST);
}

static void select_rb(int rb)
{
	u32 ctl;
	// A10 has 2 RB pin
	ctl = readl(NFC_REG_CTL);
	ctl &= ~NFC_RB_SEL;
	ctl |= ((rb & 0x1) << 3);
	writel(ctl, NFC_REG_CTL);
}

// 1 for ready, 0 for not ready
static inline int check_rb_ready(int rb)
{
	return (readl(NFC_REG_ST) & (NFC_RB_STATE0 << (rb & 0x3))) ? 1 : 0;
}

static void enable_random_preset(void)
{
	if (random_switch) {
		u32 ctl;
		ctl = readl(NFC_REG_ECC_CTL);
		ctl |= NFC_RANDOM_EN;
		ctl &= ~NFC_RANDOM_DIRECTION;
		ctl &= ~NFC_RANDOM_SEED;
		ctl |= fixed_random_seed ?
			(fixed_random_seed & 0x7FFF) << 16 :
			AW_RND_SEED << 16;
		writel(ctl, NFC_REG_ECC_CTL);
//		DBG("+random:preset");
	}
}

static void enable_random(u32 page)
{
	static const uint16_t random_seed[128] = {
		0x2b75, 0x0bd0, 0x5ca3, 0x62d1, 0x1c93, 0x07e9, 0x2162, 0x3a72,
		0x0d67, 0x67f9, 0x1be7, 0x077d, 0x032f, 0x0dac, 0x2716, 0x2436,
		0x7922, 0x1510, 0x3860, 0x5287, 0x480f, 0x4252, 0x1789, 0x5a2d,
		0x2a49, 0x5e10, 0x437f, 0x4b4e, 0x2f45, 0x216e, 0x5cb7, 0x7130,
		0x2a3f, 0x60e4, 0x4dc9, 0x0ef0, 0x0f52, 0x1bb9, 0x6211, 0x7a56,
		0x226d, 0x4ea7, 0x6f36, 0x3692, 0x38bf, 0x0c62, 0x05eb, 0x4c55,
		0x60f4, 0x728c, 0x3b6f, 0x2037, 0x7f69, 0x0936, 0x651a, 0x4ceb,
		0x6218, 0x79f3, 0x383f, 0x18d9, 0x4f05, 0x5c82, 0x2912, 0x6f17,
		0x6856, 0x5938, 0x1007, 0x61ab, 0x3e7f, 0x57c2, 0x542f, 0x4f62,
		0x7454, 0x2eac, 0x7739, 0x42d4, 0x2f90, 0x435a, 0x2e52, 0x2064,
		0x637c, 0x66ad, 0x2c90, 0x0bad, 0x759c, 0x0029, 0x0986, 0x7126,
		0x1ca7, 0x1605, 0x386a, 0x27f5, 0x1380, 0x6d75, 0x24c3, 0x0f8e,
		0x2b7a, 0x1418, 0x1fd1, 0x7dc1, 0x2d8e, 0x43af, 0x2267, 0x7da3,
		0x4e3d, 0x1338, 0x50db, 0x454d, 0x764d, 0x40a3, 0x42e6, 0x262b,
		0x2d2e, 0x1aea, 0x2e17, 0x173d, 0x3a6e, 0x71bf, 0x25f9, 0x0a5d,
		0x7c57, 0x0fbe, 0x46ce, 0x4939, 0x6b17, 0x37bb, 0x3e91, 0x76db
	};

	if (random_switch) {
		u32 ctl;

		DBG("");
		ctl = readl(NFC_REG_ECC_CTL);
		ctl |= NFC_RANDOM_EN;
		ctl &= ~NFC_RANDOM_DIRECTION;
		ctl &= ~NFC_RANDOM_SEED;
		ctl |= ((u32)random_seed[page % 128] << 16);
		writel(ctl, NFC_REG_ECC_CTL);
	}
}

static void disable_random(void)
{
	if (random_switch) {
		u32 ctl;

		ctl = readl(NFC_REG_ECC_CTL);
		ctl &= ~NFC_RANDOM_EN;
		writel(ctl, NFC_REG_ECC_CTL);
	}
}

static void enable_ecc(int pipeline)
{
	if (hwecc_switch) {
		u32 cfg = readl(NFC_REG_ECC_CTL);
		if (pipeline)
			cfg |= NFC_ECC_PIPELINE;
		else
			cfg &= (~NFC_ECC_PIPELINE) & 0xffffffff;

		if (cfg & (1 << 9))
			// if random open, disable exception
			cfg &= ~(1 << 4);
		else
			cfg |=   1 << 4;

		cfg |= NFC_ECC_EN;
		writel(cfg, NFC_REG_ECC_CTL);
	}
}

static void set_ecc_mode(int mode)
{
	if (hwecc_switch) {
		u32 ctl;
		ctl = readl(NFC_REG_ECC_CTL);
		ctl &= ~NFC_ECC_MODE;
		ctl |= mode << NFC_ECC_MODE_SHIFT;
		writel(ctl, NFC_REG_ECC_CTL);
//		DBG("%d", mode);
	}
}

static int check_ecc(int block_cnt)
{
	int i, max_ecc_bit_cnt, cfg, corrected;
	uint8_t ecc_mode;
	uint8_t ecc_bits[] = {16, 24, 28, 32, 40, 48, 56, 60, 64};

        if (!hwecc_switch)
		return 0;

	corrected = 0;
	ecc_mode = (readl(NFC_REG_ECC_CTL) & NFC_ECC_MODE) >> NFC_ECC_MODE_SHIFT;

	if (ecc_mode < ARRAY_SIZE(ecc_bits))
		max_ecc_bit_cnt = ecc_bits[ecc_mode];
	else
		max_ecc_bit_cnt = ecc_bits[0];

	//check ecc error
	cfg = readl(NFC_REG_ECC_ST) & 0xffff;
	for (i = 0; i < block_cnt; i++) {
		if (cfg & (1<<i)) {
			DBG("ECC status %x: fatal error in sector %d"
			    " (%d blocks, ECC mode %u)\n",
			    cfg, i, block_cnt, ecc_mode);
			return -1;
		}
	}

	//check ecc limit
	for (i = 0; i < block_cnt; i += 4) {
		int j;
		cfg = readl(NFC_REG_ECC_CNT0 + i);

		for (j = 3; j >= 0; j--, cfg >>= 8) {
			int bits = cfg & 0xff;
			if (bits >= max_ecc_bit_cnt - 4) {
				DBG("ECC limit %d/%d in sector %d "
				    "(%d blocks, ECC mode %u)\n",
				    bits, max_ecc_bit_cnt, i + j,
				    block_cnt, ecc_mode);
				corrected++;
			}
		}
	}
	if (corrected)
		DBG("ecc bitflips %d", corrected);

	return corrected;
}

static void disable_ecc(void)
{
	if (hwecc_switch) {
		u32 cfg = readl(NFC_REG_ECC_CTL);
		cfg &= (~NFC_ECC_EN) & 0xffffffff;
		writel(cfg, NFC_REG_ECC_CTL);
	}
}

/////////////////////////////////////////////////////////////////
// NFC
//

static void nfc_select_chip(struct mtd_info *mtd, int chip)
{
	u32 ctl;
//	DBG("chipsel %d", chip);
	// A10 has 8 CE pin to support 8 flash chips
	ctl = readl(NFC_REG_CTL);
	ctl &= ~NFC_CE_SEL;
	ctl |= ((chip & 7) << 24);
	writel(ctl, NFC_REG_CTL);
}

static void nfc_cmdfunc(struct mtd_info *mtd, unsigned command, int column,
			int page_addr)
{
	int i;
	u32 cfg = command;
	int read_size, write_size, do_enable_ecc = 0, do_enable_random = 0;
	int addr_cycle, wait_rb_flag, byte_count, sector_count;
	addr_cycle = wait_rb_flag = byte_count = sector_count = 0;

	DBG("cmd %x col %x pg %x", command, column, page_addr);

	wait_cmdfifo_free();

	// switch to AHB; should not be done unless DMA has been initialised
	if (read_buffer)
		writel(readl(NFC_REG_CTL) & ~NFC_RAM_METHOD, NFC_REG_CTL);

	switch (command) {
	case NAND_CMD_NONE: // used to set up RR
		if (column) {
			addr_cycle = 1;
			byte_count = 1;
			wait_rb_flag = 1;
		}
		break;
	case NAND_CMD_RESET:
	case NAND_CMD_ERASE2:
		break;
	case NAND_CMD_READID:
		addr_cycle = 1;
		// read 8 byte ID
		byte_count = 8;
		break;
	case NAND_CMD_PARAM:  // called during ONFI and JEDEC config requests
		addr_cycle = 1;
		byte_count = SZ_1K;
		wait_rb_flag = 1;
		break;
	case NAND_CMD_READ0: /* Implied randomised read, see
			      * nand_base.c:do_nand_read_ops() */
		do_enable_ecc = 1;
		do_enable_random = 1;
		/* otherwise the same as a regular read */
	case NAND_CMD_READOOB:
	case NAND_CMD_READ1: /* Non-randomised read: this interpretation is
			      * specific to this driver. */
		if (likely(command != NAND_CMD_READOOB)) {
			if (likely(!otp_mode)) {
				// DMA is already set up; normal read
				sector_count = mtd->writesize / SZ_1K;
				read_size = mtd->writesize;
				//DBG_INFO("cmdfunc read %d %d\n", column, page_addr);
			}
			else {
				// Initialisation-time read from the OTP: read
				// 1K raw, don't read user data NFC registers
				read_size = SZ_1K;
			}
		}
		else {
			// sector num to read
			sector_count = 1;   /* SZ_1K / SZ_1K; */
			read_size = SZ_1K;  /* FIXME: try mtd->oobsize */
			// OOB offset
			column += mtd->writesize;
		}
		cfg = NAND_CMD_READ0; /* same underlying command for randomised
				       * page, non-randomised page and OOB
				       * reads. */

		if (read_buffer) {
			//access NFC internal RAM by DMA bus
			writel(readl(NFC_REG_CTL) | NFC_RAM_METHOD, NFC_REG_CTL);
			// if the size is smaller than NFC_REG_SECTOR_NUM, read command won't finish
			// does that means the data read out (by DMA through random data output) hasn't finish?
			dma_nand_config_start(dma_hdle, 0, (u32)read_buffer, read_size);

			// NFC_SEND_CMD1 for the command 1nd cycle enable
			// NFC_SEND_CMD2 for the command 2nd cycle enable
			// NFC_SEND_CMD3 & NFC_SEND_CMD4 for NFC_READ_CMD0 & NFC_READ_CMD1
			cfg |= NFC_SEND_CMD2 | NFC_DATA_SWAP_METHOD;
			// 3 - ?
			// 2 - page command
			// 1 - spare command?
			// 0 - normal command
			if (!otp_mode)
				cfg |= 2 << 30;
		}
		else {
			// direct access to the on-chip RAM if DMA is not initialised
			cfg |= NFC_SEND_CMD2 | NFC_ACCESS_DIR;
		}

		addr_cycle = 5;
		// RAM0 is 1K size
		byte_count = SZ_1K;
		wait_rb_flag = 1;

		if (!otp_mode && do_enable_random)
			// normal operation
			//
			// 0x30 for 2nd cycle of read page
			// 0x05+0xe0 is the random data output command
			writel(0x00e00530, NFC_REG_RCMD_SET);
		else
			// OTP read or emptiness test: no random output
			writel(0x00000030, NFC_REG_RCMD_SET);
		break;
	case NAND_CMD_ERASE1:
		addr_cycle = 3;
		//DBG_INFO("cmdfunc earse block %d\n", page_addr);
		break;
	case NAND_CMD_SEQIN:
		program_column = column;
		program_page = page_addr;
		write_offset = 0;
		return;
	case NAND_CMD_PAGEPROG:
		cfg = NAND_CMD_SEQIN;
		addr_cycle = 5;
		column = program_column;
		page_addr = program_page;
		if (likely(column == 0)) {
			do_enable_ecc = 1;
			do_enable_random = 1;
			sector_count = mtd->writesize / SZ_1K;
			write_size = mtd->writesize;
			for (i = 0; i < sector_count; i++)
				writel(*((u32*)
					 (write_buffer + mtd->writesize) + i * 4),
				       NFC_REG_USER_DATA(i));
		}
		else if (column == mtd->writesize) {
			/* OOB write: ECC and the randomiser are disabled */
			sector_count = SZ_1K / SZ_1K;
			write_size = SZ_1K;
		}
		else {
			pr_err(pr_fmt("unsupported column %x\n"), column);
			return;
		}

		//access NFC internal RAM by DMA bus
		writel(readl(NFC_REG_CTL) | NFC_RAM_METHOD, NFC_REG_CTL);
		dma_nand_config_start(dma_hdle, 1, (u32)write_buffer, write_size);
		// RAM0 is 1K size
		byte_count = SZ_1K;
		writel(0x00008510, NFC_REG_WCMD_SET);
		cfg |= NFC_SEND_CMD2 | NFC_DATA_SWAP_METHOD | NFC_ACCESS_DIR;
		cfg |= 2 << 30;
		if (column != 0) {
			DBG("col %x pg %x program %.2x %.2x %.2x %.2x...",
			    column, page_addr,
			    write_buffer[0], write_buffer[1],
			    write_buffer[2], write_buffer[3]);
		}
		break;
	case NAND_CMD_STATUS:
		byte_count = 1;
		break;
	case 0x36: /* Hynix OTP read or RRT write - start of the OTP mode */
		addr_cycle = 1;
		otp_mode = 1;
		write_offset = 0;
		break;
	/* Hynix read RRT in OTP command sequence */
	case 0x16:
	case 0x17:
	case 0x04:
	case 0x19:
		if (otp_mode) {
			if (!column)
				/* leave the OTP mode */
				otp_mode = 0;
			break;
		}
		/* else unknown command */
	default:
		pr_err(pr_fmt("%s: unknown command 0x%.2x\n"), __FUNCTION__, command);
		return;
	}

	// address cycle
	if (addr_cycle) {
		u32 low = 0;
		u32 high = 0;
		switch (addr_cycle) {
		case 1:
			low = column & 0xff;
			break;
		case 2:
			low = column & 0xffff;
			break;
		case 3:
			low = page_addr & 0xffffff;
			break;
		case 5:
			high = (page_addr >> 16) & 0xff;
		case 4:
			low = (column & 0xffff) | (page_addr << 16);
			break;
		}
		writel(low, NFC_REG_ADDR_LOW);
		writel(high, NFC_REG_ADDR_HIGH);
		cfg |= NFC_SEND_ADR;
		cfg |= ((addr_cycle - 1) << 16);
	}

	// command will wait until the RB ready to mark finish?
	if (wait_rb_flag)
		cfg |= NFC_WAIT_FLAG;

	// will fetch data
	if (byte_count) {
		cfg |= NFC_DATA_TRANS;
		writel(byte_count, NFC_REG_CNT);
	}

	// set sectors
	if (sector_count)
		writel(sector_count, NFC_REG_SECTOR_NUM);

	// enable random
	if (do_enable_random) {
		if (!fixed_random_seed)
			enable_random(page_addr);
		else
			enable_random_preset();
	}

	// enable ecc
	if (do_enable_ecc)
		enable_ecc(1);

	// send command
	cfg |= NFC_SEND_CMD1;
	writel(cfg, NFC_REG_CMD);

	switch (command) {
	case NAND_CMD_READ0:
	case NAND_CMD_READ1:
		// the OTP mode is switched off after a single read
		otp_mode = 0;
	case NAND_CMD_READOOB:
		if (read_buffer)
			dma_nand_wait_finish();
		break;
	case NAND_CMD_PAGEPROG:
		dma_nand_wait_finish();
		break;
	}

	// wait command send complete
	wait_cmdfifo_free();
	wait_cmd_finish();

	// reset will wait for RB ready
	switch (command) {
	case NAND_CMD_RESET:
		// wait rb0 ready
		select_rb(0);
		while (!check_rb_ready(0));
		// wait rb1 ready
//		select_rb(1);
//		while (!check_rb_ready(1));
		// select rb 0 back
//		select_rb(0);
		break;
	case NAND_CMD_READ0:
	case NAND_CMD_READ1:
		if (read_buffer) {
			u32* oob_start = (u32*) read_buffer + mtd->writesize;
			for (i = 0; i < sector_count; i++) {
				u32 userdata = readl(NFC_REG_USER_DATA(i));
				*(oob_start + i * 4) = userdata;

			}
			if (debug) {
				// Print the userdata register first
				print_reg(&nfc_regs[20]);
				printk("Buffered user data:\n");
				// Then print the buffered content in 2 rows
				for (i = 0; i < 2; i++) {
					u8 j;
					// 32 bytes in a row
					for (j = 0; j < 8; j++)
						printk(" %.8x", *(oob_start + i * 32 + j * 4));
					printk("\n");
				}
			}
		}
		break;
	}

	// disable ecc
	if (do_enable_ecc)
		disable_ecc();

	// disable random
	if (do_enable_random)
		disable_random();

	read_offset = 0;
}

static uint8_t nfc_read_byte(struct mtd_info *mtd)
{
//	DBG("read byte at RAM offset %d", read_offset);
	return readb(NFC_RAM0_BASE + read_offset++);
}

static void nfc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
//	DBG("mtd %p, from %p, length %d", mtd, buf, len);
	if (read_buffer) {
		if (read_offset + len > buffer_size) {
			pr_err(pr_fmt("out-of-bounds read at "
				      "offset %d, length %d, buffer size %d\n"),
			       read_offset, len, buffer_size);
			return;
		}
		memcpy(buf, read_buffer + read_offset, len);
	}
	else {
		// direct read from the on-chip RAM
		memcpy(buf, (const void*)(NFC_RAM0_BASE + read_offset),
		       len);
	}
	read_offset += len;
}

static void nfc_write_byte(struct mtd_info *mtd, const uint8_t b)
{
//	DBG("write %.2x at RAM offset %d", b, write_offset);
	writeb(b, NFC_RAM0_BASE + write_offset++);
}

static void nfc_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
//	DBG("mtd %p, from %p, length %d", mtd, buf, len);
	if (write_buffer) {
		if (write_offset + len > buffer_size) {
			pr_err(pr_fmt("out-of-bounds write at "
				      "offset %d, length %d, buffer size %d\n"),
			       write_offset, len, buffer_size);
			return;
		}
		memcpy(write_buffer + write_offset, buf, len);
		write_offset += len;
	}
	else {
		// writes other than those using DMA are undefined
		BUG();
	}
}

static int nfc_dev_ready(struct mtd_info *mtd)
{
	return check_rb_ready(0);
}

irqreturn_t nfc_interrupt_handler(int irq, void *dev_id)
{
	unsigned int st = readl(NFC_REG_ST);
	if (st & NFC_RB_B2R) {
		wake_up(&nand_rb_wait);
	}
	// clear interrupt
	writel(st, NFC_REG_ST);
	return IRQ_HANDLED;
}

static int get_chip_status(struct mtd_info *mtd)
{
	u8 status;

	nfc_cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
	status = nfc_read_byte(mtd);
	if (debug && status != 0xC0)  /* report anomalies */
		DBG("chip status %.2x", status);

	return status;
}

// For erase and program command to wait for chip ready
static int nfc_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	int err;

	// clear B2R interrupt state
	writel(NFC_RB_B2R, NFC_REG_ST);

	if (check_rb_ready(0))
		goto out;

	// enable B2R interrupt
	writel(NFC_B2R_INT_ENABLE, NFC_REG_INT);
	if ((err = wait_event_timeout(nand_rb_wait, check_rb_ready(0), 1*HZ)) < 0) {
		DBG("ERROR %d", err);
	}
	// disable interrupt
	writel(0, NFC_REG_INT);

out:
	return get_chip_status(mtd);
}

static void nfc_ecc_hwctl(struct mtd_info *mtd, int mode)
{

}

static int nfc_ecc_calculate(struct mtd_info *mtd, const uint8_t *dat, uint8_t *ecc_code)
{
	return 0;
}

static int nfc_ecc_correct(struct mtd_info *mtd, uint8_t *dat, uint8_t *read_ecc, uint8_t *calc_ecc)
{
	if (!hwecc_switch)
		return 0;

	return check_ecc(mtd->writesize / SZ_1K);
}

//////////////////////////////////////////////////////////////////////////////////
// 1K mode for SPL read/write

struct save_sizes {
	u32 ctl;
	u32 ecc_ctl;
	u32 spare_area;
};

static void set_pagesize(u32 size, struct save_sizes *save)
{
	u32 ctl;

	DBG("%d bytes", size);
	ctl = readl(NFC_REG_CTL);
	save->ctl = ctl;
	ctl &= ~NFC_PAGE_SIZE;
	writel(ctl, NFC_REG_CTL);

	ctl = readl(NFC_REG_ECC_CTL);
	save->ecc_ctl = ctl;
	if (size == SZ_1K)
		set_ecc_mode(8);

	ctl = readl(NFC_REG_SPARE_AREA);
	save->spare_area = ctl;
	writel(size, NFC_REG_SPARE_AREA);
}

static void restore_pagesize(struct save_sizes *save)
{
	DBG("");
	writel(save->ctl, NFC_REG_CTL);
	writel(save->ecc_ctl, NFC_REG_ECC_CTL);
	writel(save->spare_area, NFC_REG_SPARE_AREA);
}

void nfc_read_set_pagesize(u32 page_addr, u32 size, void *buff)
{
	struct save_sizes save;
	u32 cfg = NFC_SEND_CMD1 | NFC_DATA_TRANS | NFC_SEND_ADR |
		NFC_SEND_CMD2 | ((5 - 1) << 16) | NFC_WAIT_FLAG |
		NFC_DATA_SWAP_METHOD | (2 << 30);

	if (size == SZ_1K)
		cfg |= NFC_SEQ;

	nfc_select_chip(NULL, 0);

	wait_cmdfifo_free();

	set_pagesize(size, &save);

	writel(readl(NFC_REG_CTL) | NFC_RAM_METHOD, NFC_REG_CTL);
	dma_nand_config_start(dma_hdle, 0, (u32)buff, /* SZ_1K */ size); // FIXME: size

	writel(page_addr << 16, NFC_REG_ADDR_LOW);
	writel(page_addr >> 16, NFC_REG_ADDR_HIGH);
	writel(SZ_1K, NFC_REG_CNT);
	writel(0x00e00530, NFC_REG_RCMD_SET);
	writel(size / SZ_1K, NFC_REG_SECTOR_NUM);

	enable_random_preset();
	enable_ecc(1);

//	DBG("read: write command");
	writel(cfg, NFC_REG_CMD);
//	DBG("read: wait DMA finishes");
	dma_nand_wait_finish();
//	DBG("read: wait command FIFO frees");
	wait_cmdfifo_free();
//	DBG("read: wait command finishes");
	wait_cmd_finish();

	disable_ecc();
	check_ecc(size / SZ_1K);
	disable_random();

	restore_pagesize(&save);

	nfc_select_chip(NULL, -1);
}

void nfc_write_set_pagesize(u32 page_addr, u32 size, void *buff)
{
	struct save_sizes save;
	u32 cfg = NAND_CMD_SEQIN | NFC_SEQ | NFC_SEND_CMD1 | NFC_DATA_TRANS | NFC_SEND_ADR |
		NFC_SEND_CMD2 | ((5 - 1) << 16) | NFC_WAIT_FLAG | NFC_DATA_SWAP_METHOD | NFC_ACCESS_DIR |
		(2 << 30);

	nfc_select_chip(NULL, 0);

	wait_cmdfifo_free();

	set_pagesize(size, &save);

	writel(readl(NFC_REG_CTL) | NFC_RAM_METHOD, NFC_REG_CTL);
	dma_nand_config_start(dma_hdle, 1, (u32)buff, size);

	writel(page_addr << 16, NFC_REG_ADDR_LOW);
	writel(page_addr >> 16, NFC_REG_ADDR_HIGH);
	writel(SZ_1K, NFC_REG_CNT);
	writel(0x00008510, NFC_REG_WCMD_SET);
	writel(size / SZ_1K, NFC_REG_SECTOR_NUM);

	enable_random_preset();
	enable_ecc(1);

	writel(cfg, NFC_REG_CMD);

	dma_nand_wait_finish();
	wait_cmdfifo_free();
	wait_cmd_finish();

	disable_ecc();
	disable_random();

	restore_pagesize(&save);

	nfc_select_chip(NULL, -1);
}

/*
 * Page read with hardware ECC and randomisation that takes into account empty
 * pages and does not issue an ECC error when trying to read a correct empty
 * page.
 */
static int nfc_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
			       uint8_t *buf, int oob_required, int page)
{
	int eccstatus = 0, status = 0;

	/* The corresponding read operation has already been issued from
	 * nand_base.c:nand_do_read_ops() and the read buffer should contain
	 * read data. Check ECC and, in case of error, test for emptiness. */
	status = nand_page_get_status(mtd, page);
	if (status == NAND_PAGE_EMPTY) {
		memset(buf, 0xff, mtd->writesize);
		if (oob_required)
			memset(chip->oob_poi, 0xff, mtd->oobsize);
		/* success */
		return 0;
	}

	eccstatus = check_ecc(mtd->writesize / SZ_1K);

	if (status == NAND_PAGE_FILLED) {
		if (eccstatus >= 0) {
			/* Collect ECC statistics: corrected bitflips. */
			mtd->ecc_stats.corrected += eccstatus;

			/* Copy the output to the buffer of the main NAND driver. */
			nfc_read_buf(mtd, buf, mtd->writesize);
			if (oob_required)
				/* copy the OOB area */
				nfc_read_buf(mtd, chip->oob_poi, mtd->oobsize);
		}
		else {
			/* ECC error. The number of bitflips is inessential */
			mtd->ecc_stats.failed++;
			/* Then return the ECC error status. */
		}
	}
	else {  /* status == NAND_PAGE_STATUS_UNKNOWN */
		if (eccstatus >= 0) {
			/* Note the page as filled because it has correct ECC
			 * codes written by the controller. */
			nand_page_set_status(mtd, page, NAND_PAGE_FILLED);

			/* Collect ECC statistics: corrected bitflips. */
			mtd->ecc_stats.corrected += eccstatus;

			/* Copy the output to the buffer of the main NAND driver. */
			nfc_read_buf(mtd, buf, mtd->writesize);
			if (oob_required)
				/* copy the OOB area */
				nfc_read_buf(mtd, chip->oob_poi, mtd->oobsize);
		}
		else {
			/* The ECC check has failed. Check if the page is empty
			 * and update the read buffer if so. Otherwise report
			 * ECC failure. */
			eccstatus = 0;

			/* Re-read the page without the randomiser or ECC. */
			nfc_cmdfunc(mtd, NAND_CMD_READ1, 0, page);

			/* Copy the output to the main driver area. */
			nfc_read_buf(mtd, buf, mtd->writesize);
			if (oob_required)
				/* copy the OOB area */
				nfc_read_buf(mtd, chip->oob_poi, mtd->oobsize);

			if (nand_page_is_empty(mtd, buf, buf + mtd->writesize)) {
				status = NAND_PAGE_EMPTY;
				memset(buf, 0xff, mtd->writesize);
				if (oob_required)
					memset(chip->oob_poi, 0xff, mtd->oobsize);
				/* success */
			}
			else {
				status = NAND_PAGE_FILLED;
				/* ECC error. The number of bitflips is inessential */
				mtd->ecc_stats.failed++;
			}
			nand_page_set_status(mtd, page, status);
		}
	}

	DBG("pg stat %d, ECC stat %d, ECC failed %d",
	    nand_page_get_status(mtd, page),
	    eccstatus, mtd->ecc_stats.failed);
	return eccstatus;
}

int nfc_first_init(struct mtd_info *mtd)
{
	u32 ctl;
	struct nand_chip *nand = mtd->priv;
	int ret = 0;

	DBG("");

	// set NFC clock source
	sunxi_set_nand_clock(20);

	// set NFC pio
	sunxi_set_nand_pio();

	// reset NFC
	ctl = readl(NFC_REG_CTL);
	ctl |= NFC_RESET;
	writel(ctl, NFC_REG_CTL);
	while(readl(NFC_REG_CTL) & NFC_RESET);

	// enable NFC
	ctl = NFC_EN;
	writel(ctl, NFC_REG_CTL);

	// serial_access_mode = 1
	// this is needed by some nand chip to read ID
	ctl = (1 << 8);
	writel(ctl, NFC_REG_TIMING_CTL);

	nand->ecc.mode = NAND_ECC_HW;
	nand->ecc.hwctl = nfc_ecc_hwctl;
	nand->ecc.calculate = nfc_ecc_calculate;
	nand->ecc.correct = nfc_ecc_correct;
	nand->ecc.read_page = nfc_read_page_hwecc;
	nand->select_chip = nfc_select_chip;
	nand->dev_ready = nfc_dev_ready;
	nand->cmdfunc = nfc_cmdfunc;
	nand->read_byte = nfc_read_byte;
	nand->read_buf = nfc_read_buf;
	nand->write_byte = nfc_write_byte;
	nand->write_buf = nfc_write_buf;
	nand->waitfunc = nfc_wait;
	nand->bbt_options = NAND_BBT_SCANLASTPAGE; // | NAND_BBT_SCAN2NDPAGE
	if (bbt_use_flash)
		nand->bbt_options |= NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;
	if (invalid_bbm)
		nand->options |= NAND_INVALID_BBM;

	/* Set up a temporary DMA read buffer */
	dma_hdle = dma_nand_request(1);
	if (!dma_hdle) {
		pr_err(pr_fmt("failed to set up a temporary DMA buffer\n"));
		ret = -ENODEV;
		goto out;
	}

	buffer_size = SZ_1K;   // should fit the RR table from the OTP
	read_buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (read_buffer == NULL) {
		ret = -ENOMEM;
		goto release_dma;
	}

	read_buffer_dma = dma_map_single(NULL, read_buffer, buffer_size, DMA_FROM_DEVICE);
	if (!read_buffer_dma) {
		ret = -ENODEV;
		goto free_read_buffer;
	}

	return 0;

free_read_buffer:
	kfree(read_buffer);
release_dma:
	dma_nand_release(dma_hdle);
out:
	return ret;
}

#define PRINT_BUFFER_SIZE SZ_16K

static void print_page(struct mtd_info *mtd, int page, bool full)
{
	int i, j, saved_read_offset = read_offset;
	u8* buff;

	pr_info(" ===== PAGE %d READ =====\n", page);

	buff = kmalloc(PRINT_BUFFER_SIZE, GFP_KERNEL);
 	if (!buff)
 		return;

	read_offset = 0;
	memset(buff, 0xEE, PRINT_BUFFER_SIZE);
	nfc_cmdfunc(mtd, NAND_CMD_READ0, 0, page);
	if (full) {
		pr_info("READ:\n");
		nfc_read_buf(mtd, buff, mtd->writesize);
		for (i = 0; i < mtd->writesize / 32; i++) {
			for (j = 0; j < 32; j++)
				printk("%.2x ", buff[32 * i + j]);
			printk("\n");
		}
	}
	else {
		nfc_read_buf(mtd, buff, 6);
		pr_info("READ 6: %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x\n",
			buff[0], buff[1], buff[2], buff[3],
			buff[4], buff[5], buff[6], buff[7]);
	}

	pr_info(" ***** REGISTERS AFTER READ0 *****\n");
	print_regs();

	read_offset = mtd->writesize;
	memset(buff, 0xBB, PRINT_BUFFER_SIZE);
	nfc_cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	nfc_read_buf(mtd, buff, mtd->oobsize);
	pr_info("OOB:\n");
	for (i = 0; i < mtd->oobsize / 32; i++) {
		for (j = 0; j < 32; j++)
			printk("%.2x ", buff[32 * i + j]);
		printk("\n");
	}
	read_offset = saved_read_offset;

	pr_info(" ***** REGISTERS AFTER READOOB *****\n");
	print_regs();

	kfree(buff);
	pr_info(" ===== PAGE %d READ END =====\n", page);
}

static void print_set_pagesize(struct mtd_info *mtd, u32 size, int page)
{
	int i, j;
	u8* buff;

	pr_info(" ===== PAGE %d READ %d BYTES =====\n", page, size);

	buff = kmalloc(size, GFP_KERNEL);
 	if (!buff)
 		return;

	read_offset = 0;
	memset(buff, 0xEE, size);
	nfc_read_set_pagesize(0, size, buff);
	pr_info("READ %d BYTES:\n", size);
	for (i = 0; i < size / 32; i++) {
		for (j = 0; j < 32; j++)
			printk("%.2x ", buff[32 * i + j]);
		printk("\n");
	}
	pr_info(" ***** REGISTERS AFTER READ0 *****\n");
	print_regs();

	kfree(buff);
	pr_info(" ===== PAGE %d READ %d BYTES END =====\n", page, size);
}

static void test_nfc(struct mtd_info *mtd)
{
//	int i, j, n=0;
	struct nand_chip *nand = mtd->priv;
	int page = 1280;
//	u8 buff[PRINT_BUFFER_SIZE];
//	int blocks = 2, num_blocks = mtd->writesize / 1024;
	int saved_random_switch = random_switch;
	int saved_hwecc_switch  = hwecc_switch;

	DBG("============== TEST NFC ================\n");
//	memset(buff, 0, 2048);

	// read page
	DBG("Test: read page %d\n", page);
	print_page(mtd, page, 0);

	// erase block
	pr_info(" === Test: erase block of page %d ===\n", page);
	nfc_cmdfunc(mtd, NAND_CMD_ERASE1, 0, page);
	nfc_cmdfunc(mtd, NAND_CMD_ERASE2, -1, -1);
	nfc_wait(mtd, nand);
	print_page(mtd, page, 0);

	pr_info(" === Test: print the erased page without randomizer or ECC ===\n");
	hwecc_switch = 0;
	random_switch = 0;
	print_page(mtd, page, 0);
	random_switch = saved_random_switch;
	hwecc_switch = saved_hwecc_switch;

	/*
	// write block
	nfc_cmdfunc(mtd, NAND_CMD_SEQIN, 0, page);
	for (i = 0; i < blocks; i++) {
		for (j = 0; j < 1024; j++, n++)
			buff[j] = n % 256;
		nfc_write_buf(mtd, buff, 1024);
	}
	for ( ; i < num_blocks; i++) {
		memset(buff, 0xff, 1024);
		nfc_write_buf(mtd, buff, 1024);
	}
	for (i = 0, n = 128; i < mtd->oobsize; i++, n++)
		buff[i] = n % 256;
	nfc_write_buf(mtd, buff, 1024);
	nfc_cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	nfc_wait(mtd, nand);
	print_page(mtd, page);
	*/

/*
	// test oob write
	nfc_cmdfunc(mtd, NAND_CMD_SEQIN, mtd->writesize, page);
	for (i = 0, n = 0xff; i < 640; i++, n++)
		buff[i] = n % 256;
	nfc_write_buf(mtd, buff, 1024);
	nfc_cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	nfc_wait(mtd, nand);
	print_page(mtd, page);
*/
}

/*
// Test unit ops
static void test_ops(struct mtd_info *mtd)
{
	uint32_t page = 1280;
	uint32_t v1, v2;

	// test sequence read
	wait_cmdfifo_free();
	// NFC_DATA_TRANS = 1, NFC fetch data to RAM0
	// NFC_CMD_TYPE = 1,2,3 won't read out any thing
	v1 = NAND_CMD_READ0 | NFC_SEQ | NFC_SEND_CMD1 | NFC_DATA_TRANS | NFC_SEND_ADR | NFC_SEND_CMD2 | ((5 - 1) << 16) | NFC_WAIT_FLAG | (3 << 30);
	v2 = NAND_CMD_READSTART;
	writel(page << 16, NFC_REG_ADDR_LOW);
	writel(page >> 16, NFC_REG_ADDR_HIGH);
	//writel(2, NFC_REG_SECTOR_NUM);
	// NFC_REG_CNT = n, fetch n byte to RAM
	writel(1, NFC_REG_CNT);
	writel(v2, NFC_REG_RCMD_SET);
	writel(v1, NFC_REG_CMD);
	wait_cmdfifo_free();
	wait_cmd_finish();
	DBG_INFO("SEQ READ IO: %x %x %x %x %x %x\n",
			 readb(NFC_REG_IO_DATA),
			 readb(NFC_REG_IO_DATA),
			 readb(NFC_REG_IO_DATA),
			 readb(NFC_REG_IO_DATA),
			 readb(NFC_REG_IO_DATA),
			 readb(NFC_REG_IO_DATA));
	DBG_INFO("SEQ READ RAM: %x %x %x %x %x %x\n",
			 readb(NFC_RAM0_BASE),
			 readb(NFC_RAM0_BASE + 1),
			 readb(NFC_RAM0_BASE + 2),
			 readb(NFC_RAM0_BASE + 3),
			 readb(NFC_RAM0_BASE + 4),
			 readb(NFC_RAM0_BASE + 5));
}
*/

int nfc_second_init(struct mtd_info *mtd)
{
	int i, err, j;
	u32 ctl;
	u8 id[8];
	struct nand_chip_param *nand_chip_param, *chip_param = NULL;
	struct nand_chip *nand = mtd->priv;

	DBG("");
	// FIXME: assert the first chip in case chip select was set to -1 (all)
	nfc_select_chip(NULL, 0);

	// get nand chip id
	nfc_cmdfunc(mtd, NAND_CMD_READID, 0, -1);
	for (i = 0; i < 8; i++)
		id[i] = nfc_read_byte(mtd);
	DBG("nand chip id: %x %x %x %x %x %x %x %x",
	    id[0], id[1], id[2], id[3],
	    id[4], id[5], id[6], id[7]);

	// find chip
	nand_chip_param = sunxi_get_nand_chip_param(id[0]);
	for (i = 0; nand_chip_param[i].id_len; i++) {
		int find = 1;
		for (j = 0; j < nand_chip_param[i].id_len; j++) {
			if (id[j] != nand_chip_param[i].id[j]) {
				find = 0;
				break;
			}
		}
		if (find) {
			chip_param = &nand_chip_param[i];
			printk("found chip in Sunxi database: ");
			for (j = 0; j < nand_chip_param[i].id_len; j++) {
				printk("%x", nand_chip_param[i].id[j]);
			}
			printk(" (ECC mode %d)\n", nand_chip_param[i].ecc_mode);
			break;
		}
	}

	// not find
	if (chip_param == NULL) {
		pr_err(pr_fmt("unknown chip"));
		return -ENODEV;
	}

	// set final NFC clock freq
	if (chip_param->clock_freq > 30)
		chip_param->clock_freq = 30;
	sunxi_set_nand_clock(chip_param->clock_freq);
	DBG("set clock freq to %dMHz\n", chip_param->clock_freq);

	// disable interrupt
	writel(0, NFC_REG_INT);
	// clear interrupt
	writel(readl(NFC_REG_ST), NFC_REG_ST);

	// set ECC mode
	set_ecc_mode(chip_param->ecc_mode);

	// enable NFC
	ctl = NFC_EN;

	// Bus width
	if (nand->options & NAND_BUSWIDTH_16) {
		DBG("flash chip bus width 16");
		ctl |= (1 & 0x1) << 2;
	}
	else {
		DBG("flash chip bus width 8");
	}

	// Page size
	if (nand->page_shift > 14 || nand->page_shift < 10) {
		pr_err(pr_fmt("page shift out of range %d\n"), nand->page_shift);
		err = -EINVAL;
		goto out;
	}
	// 0 for 1K
	ctl |= ((nand->page_shift - 10) & 0xf) << 8;
	writel(ctl, NFC_REG_CTL);

	writel(0xff, NFC_REG_TIMING_CFG);
	writel((1 << nand->page_shift) + 2, NFC_REG_SPARE_AREA);

	// disable random
	disable_random();

	// setup ECC layout
	nand->ecc.layout = &sunxi_ecclayout;
	nand->ecc.bytes = 0;

	// FIXME: derive from the ID in nand_base.c:parse_hynix_sizes()
	nand->ecc.strength = 40;
	nand->ecc.size = SZ_1K;

	sunxi_ecclayout.eccbytes = 0;
	sunxi_ecclayout.oobavail = mtd->writesize / 1024 * 4 - 2;
	sunxi_ecclayout.oobfree->offset = 2;
	sunxi_ecclayout.oobfree->length = mtd->writesize / 1024 * 4 - 2;
	DBG("oobavail %d oobfree.offset %d oobfree.length %d",
	    sunxi_ecclayout.oobavail, sunxi_ecclayout.oobfree->offset,
	    sunxi_ecclayout.oobfree->length);

	// uninitialise the temporary DMA buffer
	if (read_buffer) {
		dma_unmap_single(NULL, read_buffer_dma, buffer_size, DMA_FROM_DEVICE);
		kfree(read_buffer);
	}
	if (dma_hdle)
		dma_nand_release(dma_hdle);

	// setup DMA for normal operation
	dma_hdle = dma_nand_request(1);
	if (dma_hdle == 0) {
		pr_err(pr_fmt("DMA request failed\n"));
		err = -ENODEV;
		goto out;
	}

	// alloc buffer
	buffer_size = mtd->writesize + mtd->oobsize;
	read_buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (read_buffer == NULL) {
		err = -ENOMEM;
		goto release_dma_out;
	}
	write_buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (write_buffer == NULL) {
		err = -ENOMEM;
		goto free_read_out;
	}

	// map
	read_buffer_dma = dma_map_single(NULL, read_buffer, buffer_size, DMA_FROM_DEVICE);
	write_buffer_dma = dma_map_single(NULL, write_buffer, buffer_size, DMA_TO_DEVICE);

	DBG("OOB size %u, page size %u, block size %u, total size %llu\n"
	    "DMA buffer size %u (read and write)",
	    mtd->oobsize, mtd->writesize, mtd->erasesize, mtd->size,
	    buffer_size);

	// test command
	//test_nfc(mtd);
	///test_ops(mtd);
	if (debug) {
		// start of SPL, read in full mode
		print_page(mtd, 0, 1);
		// start of SPL, read in 1 KiB mode
		print_set_pagesize(mtd, SZ_1K, 0);
		// start of U-Boot at 4 MiB with datasheet page size
		print_page(mtd, 256, 1);
	}
//	print_set_pagesize(mtd, SZ_2K, 0);
//	print_set_pagesize(mtd, SZ_4K, 0);
//	print_set_pagesize(mtd, SZ_8K, 0);
	return 0;

// free_write_out:
	kfree(write_buffer);
free_read_out:
	kfree(read_buffer);
release_dma_out:
	dma_nand_release(dma_hdle);
out:
	return err;
}

void nfc_disable(void)
{
	u32 cfg;

	DBG("");

	cfg = readl(NFC_REG_CTL);
	cfg &= ~NFC_EN;
	writel(cfg, NFC_REG_CTL);
}

void nfc_exit(struct mtd_info *mtd)
{
	nfc_disable();
	if (read_buffer) {
		dma_unmap_single(NULL, read_buffer_dma, buffer_size, DMA_FROM_DEVICE);
		kfree(read_buffer);
	}
	if (write_buffer) {
		dma_unmap_single(NULL, write_buffer_dma, buffer_size, DMA_TO_DEVICE);
		kfree(write_buffer);
	}
	if (dma_hdle)
		dma_nand_release(dma_hdle);

	sunxi_release_nand_pio();
	release_nand_clock();
}
