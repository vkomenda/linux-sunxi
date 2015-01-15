/*
 * NAND Flash partitions described by the OF (or flattened) device tree
 *
 * Copyright © 2014 Boris BREZILLON <b.brezillon.dev@gmail.com>
 *             2015 Vladimir Komendantskiy
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/mtd/mtd.h>
#include <linux/slab.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#define DBG(fmt, arg...) pr_info(pr_fmt("%s: " fmt "\n"), __FUNCTION__, ##arg)

static inline bool node_has_compatible(struct device_node *pp)
{
	return of_get_property(pp, "compatible", NULL);
}

int ofnandpart_parser(struct mtd_info *master,
		      struct mtd_partition **pparts,
		      struct mtd_part_parser_data *data)
{
	struct device_node *node;
	const char *partname;
	struct device_node *pp;
	struct ofnandpart_data* np;
	int nr_parts;

	DBG("");

	if (!data)
		return 0;

	node = data->of_node;
	if (!node)
		return 0;

	np = container_of(data, struct ofnandpart_data, gen);

	nr_parts = 0;
	for_each_child_of_node(node,  pp) {
		const __be32 *reg;
		int len;
		int a_cells, s_cells;
		uint64_t offset, size;
		uint32_t mask_flags = 0;
		struct nand_part *part;

		DBG("node %p, child %p", node, pp);

		if (node_has_compatible(pp))
			continue;

		reg = of_get_property(pp, "reg", &len);
		if (!reg)
			continue;

		a_cells = of_n_addr_cells(pp);
		s_cells = of_n_size_cells(pp);
		offset = of_read_number(reg, a_cells);
		size = of_read_number(reg + a_cells, s_cells);

		partname = of_get_property(pp, "label", &len);
		if (!partname)
			partname = of_get_property(pp, "name", &len);

		if (of_get_property(pp, "read-only", &len))
			mask_flags |= MTD_WRITEABLE;

		if (of_get_property(pp, "lock", &len))
			mask_flags |= MTD_POWERUP_LOCK;

		if (np->parse)
			part = np->parse(master, pp);
		else
			part = nandpart_alloc();

		DBG("a_cells %d, s_cells %d, offset %llx, size %llx, name %s,"
		    " part %p",
		    a_cells, s_cells, offset, size, partname, part);

		if (IS_ERR(part))
			continue;

		part->offset = offset;
		part->master = master;
		part->mtd.name = partname;
		part->mtd.size = size;
		part->mtd.flags = mask_flags;

		// NOTE: A better solution can be to return partition
		// information using the pparts argument, to be registered in
		// the generic mtd_device_parse_register(), rather than breaking
		// the generic MTD architecture by intermingling registration
		// with parsing. The below line breaks the architecture:
		if (nand_add_partition(master, part)) {
			if (part->release)
				part->release(part);
			continue;
		}

		nr_parts++;
	}

	if (!nr_parts) {
		of_node_put(pp);
		pr_err("No valid partition found on %s\n", node->full_name);
		return -EINVAL;
	}
	else
		DBG("found %d partitions on node %s",
		    nr_parts, node->full_name);

	// Return the number of partitions to be registered. For any partitions
	// to be registered by the callee, this function should make use of the
	// pparts argument. Per-partition devices have already been registered
	// by nand_add_partition(). So, signal the callee to register only the
	// parent device and no further partitions.
	return 0;
}

static struct mtd_part_parser ofnandpart_parser_rec = {
	.owner = THIS_MODULE,
	.parse_fn = ofnandpart_parser,
	.name = "ofnandpart",
};

static int __init ofnandpart_parser_init(void)
{
	register_mtd_parser(&ofnandpart_parser_rec);
	return 0;
}

static void __exit ofnandpart_parser_exit(void)
{
	deregister_mtd_parser(&ofnandpart_parser_rec);
}

module_init(ofnandpart_parser_init);
module_exit(ofnandpart_parser_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parser for NAND flash partitioning information in device tree");
MODULE_AUTHOR("Boris BREZILLON, Vladimir Komendantskiy");
