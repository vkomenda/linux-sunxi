/*
 * main.c
 *
 * Copyright (C) 2013 Qiang Yu <yuq825@gmail.com>
 *               2014 Vladimir Komendantskiy
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <plat/sys_config.h>

#include "defs.h"
#include "nfc.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("yuq");

#define	DRIVER_NAME "mtd-nand-sunxi"

extern irqreturn_t nfc_interrupt_handler(int irq, void *dev_id);

extern int debug;

static const char* const sunxi_mtd_part_types[] = {
	"cmdlinepart",
	NULL
};

/*
 * Default partitions that are set up if the kernel command-line "mtdparts"
 * option did not parse. Chip size is fixed to 8 GiB.
 */
static struct mtd_partition sunxi_mtd_partitions[] = {
	{
		.name   = "SPL",
		.offset = 0,
		.size   = SZ_4M,
	},
	{
		.name   = "U-Boot",
		.offset = SZ_4M,
		.size   = SZ_4M,
	},
	{
		.name   = "uEnv",
		.offset = SZ_8M,
		.size   = SZ_4M,
	},
	{
		.name   = "packimg",
		.offset = 12 * SZ_1M,
		.size   = SZ_16M,
	},
	{
		.name   = "kernel",
		.offset = 28 * SZ_1M,
		.size   = SZ_16M,
	},
	{
		.name   = "initramfs",
		.offset = 44 * SZ_1M,
		.size   = SZ_256M,
	},
	{
		.name   = "rootfs",
		.offset = 300 * SZ_1M,
		.size   = 4 * (uint64_t) SZ_1G - 300 * (uint64_t) SZ_1M,
//			  /* reserve 16MiB at the end for bad block tables */
//			  - SZ_16M,
	},
};

struct sunxi_nand_info {
	struct mtd_info mtd;
	struct nand_chip nand;
};

static int __devinit nand_probe(struct platform_device *pdev)
{
	int err;
	struct sunxi_nand_info *info;

	DBG("");

	if ((info = kzalloc(sizeof(*info), GFP_KERNEL)) == NULL) {
		DBG("no memory");
		err = -ENOMEM;
		goto out;
	}

	info->mtd.priv = &info->nand;
	info->mtd.name = dev_name(&pdev->dev);
	info->mtd.owner = THIS_MODULE;

	if ((err = nfc_first_init(&info->mtd)) < 0) {
		pr_err(pr_fmt("first init ERROR %d\n"), err);
		goto out_free_info;
	}

	// first scan to find the device and get the page size
	if ((err = nand_scan_ident(&info->mtd, 1, NULL)) < 0) {
		pr_err(pr_fmt("ID scan ERROR %d\n"), err);
		goto out_nfc_exit;
	}

	// init NFC with flash chip info got from first scan
	if ((err = nfc_second_init(&info->mtd)) < 0) {
		pr_err(pr_fmt("second init ERROR %d\n"), err);
		goto out_nfc_exit;
	}

	// register IRQ
	if ((err = request_irq(SW_INT_IRQNO_NAND, nfc_interrupt_handler,
			       IRQF_DISABLED, "NFC", &info->mtd)) < 0) {
		pr_err(pr_fmt("IRQ request ERROR %d\n"), err);
		goto out_nfc_exit;
	}

	// second phase scan
	if ((err = nand_scan_tail(&info->mtd)) < 0) {
		pr_err(pr_fmt("nand_scan_tail ERROR %d\n"), err);
		goto out_irq;
	}

	err = mtd_device_parse_register(&info->mtd, sunxi_mtd_part_types,
					NULL,
					sunxi_mtd_partitions,
					ARRAY_SIZE(sunxi_mtd_partitions));
	if (err < 0) {
		pr_err(pr_fmt("MTD register ERROR %d\n"), err);
		goto out_release_nand;
	}

	platform_set_drvdata(pdev, info);
	return 0;

out_release_nand:
	nand_release(&info->mtd);
out_irq:
	free_irq(SW_INT_IRQNO_NAND, &info->mtd);
out_nfc_exit:
	nfc_exit(&info->mtd);
out_free_info:
	kfree(info);
out:
	return err;
}

static int __devexit nand_remove(struct platform_device *pdev)
{
	struct sunxi_nand_info *info = platform_get_drvdata(pdev);

	DBG("");

	platform_set_drvdata(pdev, NULL);
	mtd_device_unregister(&info->mtd);
	nand_release(&info->mtd);
	free_irq(SW_INT_IRQNO_NAND, &info->mtd);
	nfc_exit(&info->mtd);
	kfree(info);
	return 0;
}

static void nand_shutdown(struct platform_device *pdev)
{

}

static int nand_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int nand_resume(struct platform_device *pdev)
{
	return 0;
}

static void	nfc_dev_release(struct device *dev)
{

}

static struct platform_driver plat_driver = {
	.probe      =             nand_probe,
	.remove     = __devexit_p(nand_remove),
	.shutdown   = nand_shutdown,
	.suspend    = nand_suspend,
	.resume     = nand_resume,
	.driver     = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static struct platform_device plat_device = {
	.name = DRIVER_NAME,
	.id = 0,
	.dev = {
		.release = nfc_dev_release,
	},
};

int nand1k_init(void);
void nand1k_exit(void);

static int __init nand_init(void)
{
	int err;
	int nand_used = 0;

	DBG("");

	if (script_parser_fetch("nand_para", "nand_used", &nand_used, sizeof(int)))
		pr_err(pr_fmt("FEX script parse error\n"));

	if(nand_used == 0) {
		DBG("nand driver is disabled");
		return 0;
	}

	if ((err = platform_driver_register(&plat_driver)) != 0) {
		pr_err(pr_fmt("platform driver registration failure\n"));
		return err;
	}

	// add an NFC, may be should be done by platform driver
	if ((err = platform_device_register(&plat_device)) < 0) {
		pr_err(pr_fmt("platform device registration failure\n"));
		return err;
	}

	if (nand1k_init())
		// consider nand1k a non-critical component and continue
		pr_err(pr_fmt("nand1k device registration failure\n"));

	return 0;
}

static void __exit nand_exit(void)
{
	int nand_used = 0;

	DBG("");

	if (script_parser_fetch("nand_para", "nand_used", &nand_used, sizeof(int)))
		pr_err(pr_fmt("FEX script parse error\n"));

	if(nand_used == 0) {
		DBG("nand driver is disabled");
		return;
	}

	nand1k_exit();
	platform_device_unregister(&plat_device);
	platform_driver_unregister(&plat_driver);
}

module_init(nand_init);
module_exit(nand_exit);
