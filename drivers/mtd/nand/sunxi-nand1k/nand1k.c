/*
 * nand1k.c
 *
 * Copyright (C) 2013 Qiang Yu <yuq825@gmail.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "nfc.h"
#include "defs.h"

extern int debug;

//////////////////////////////////////////////////////////////////////////////
// Linux character device interface
//////////////////////////////////////////////////////////////////////////////

#define DEV_CLASS_NAME "nand1k"
#define CHAR_DEV_NAME "nand1k"

#define RW_BUFFER_SIZE SZ_1K

static struct class *dev_class;
static int nand1k_major;
static char *rw_buff;

static int nand1k_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int nand1k_close(struct inode *inode, struct file *file)
{
    return 0;
}

static int nand1k_read(struct file *filp, char __user *buff, size_t count, loff_t *f_pos)
{
	loff_t offs = *f_pos;
	uint32_t len, ret, page, offset;
	size_t size = 0;

	DBG("offset %llx count %x", offs, count);

	if (offs > 128 * SZ_1K || offs < 0 ||
	    count < 0 || count > 128 * SZ_1K ||
	    offs + count > 128 * SZ_1K) {
		pr_err(pr_fmt("nand1k is restricted to access the first 128 1K pages\n"));
		return -EINVAL;
	}

	while (size < count) {
		page = offs / SZ_1K;
		offset = offs % SZ_1K;
		len = SZ_1K - offset;
		if (len > count - size)
			len = count - size;
		nfc_read_set_pagesize(page, SZ_1K, rw_buff);
		ret = copy_to_user(buff, rw_buff + offset, len);
		DBG("pg %x offset %x len %x", page, offset, len);
		DBG("read %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x...",
			   rw_buff[0], rw_buff[1], rw_buff[2], rw_buff[3],
			   rw_buff[4], rw_buff[5], rw_buff[6], rw_buff[7]);
		size += len - ret;
		offs += len - ret;
		buff += len - ret;
		if (ret)
			break;
	}

	*f_pos += size;
	return size;
}

static int nand1k_write(struct file *filp, const char __user *buff, size_t count, loff_t *f_pos)
{
	loff_t offs = *f_pos;
	uint32_t ret;
	size_t size = 0;

	DBG("offset %llx count %x", offs, count);

	if (offs > 128 * SZ_1K || offs < 0 ||
	    count < 0 || count > 128 * SZ_1K ||
	    offs + count > 128 * SZ_1K) {
		pr_err(pr_fmt("nand1k is restricted to access the first 128 1K pages"));
		return -EINVAL;
	}

	if ((offs & (SZ_1K - 1)) || (count & (SZ_1K - 1))) {
		pr_err(pr_fmt("nand1k can't write non-1K-aligned data"));
		return -EINVAL;
	}

	while (size < count) {
		ret = copy_from_user(rw_buff, buff, SZ_1K);
		if (ret)
			break;

		nfc_write_set_pagesize(offs / SZ_1K, SZ_1K, rw_buff);

		size += SZ_1K;
		offs += SZ_1K;
		buff += SZ_1K;
	}

	*f_pos += size;
	return size;
}

static long nand1k_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    switch (cmd) {
    default:
		return -ENOTTY;
    }

    return 0;
}

struct file_operations nand1k_fops = {
	.read = nand1k_read,
	.write = nand1k_write,
	.open = nand1k_open,
	.release = nand1k_close,
	.unlocked_ioctl = nand1k_ioctl,
};

int nand1k_init(void)
{
	int err;
	struct device *dev;

	DBG("");

	rw_buff = kzalloc(RW_BUFFER_SIZE, GFP_KERNEL);
	if (!rw_buff) {
		err = -ENOMEM;
		goto error0;
	}

	dev_class = class_create(THIS_MODULE, DEV_CLASS_NAME);
	if (IS_ERR(dev_class)) {
		err = PTR_ERR(dev_class);
		goto error1;
	}

	nand1k_major = register_chrdev(0, CHAR_DEV_NAME, &nand1k_fops);
	if (nand1k_major < 0) {
		err = nand1k_major;
		goto error2;
	}

    // Send uevents to udev, so it'll create /dev nodes
	dev = device_create(dev_class, NULL, MKDEV(nand1k_major, 0), NULL, CHAR_DEV_NAME);
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		goto error3;
	}

	return 0;

error3:
	unregister_chrdev(nand1k_major, CHAR_DEV_NAME);
error2:
	class_destroy(dev_class);
error1:
	kfree(rw_buff);
error0:
	return err;
}

void nand1k_exit(void)
{
	DBG("");

	device_destroy(dev_class, MKDEV(nand1k_major, 0));
	unregister_chrdev(nand1k_major, CHAR_DEV_NAME);
	class_destroy(dev_class);
	kfree(rw_buff);
}
