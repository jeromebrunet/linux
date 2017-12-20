// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017 BayLibre, SAS.
// Author: Jerome Brunet <jbrunet@baylibre.com>

#include <linux/clk-provider.h>
#include <linux/regmap.h>

static int clk_regmap_read(void *reg, unsigned int offset, unsigned int *val)
{
	struct regmap *map = (struct regmap *)reg;

	return regmap_read(map, offset, val);
}

static int clk_regmap_write(void *reg, unsigned int offset, unsigned int val)
{
	struct regmap *map = (struct regmap *)reg;

	return regmap_write(map, offset, val);
}

static int clk_regmap_update(void* reg, unsigned int offset, unsigned int mask,
			    unsigned int val)
{
	struct regmap *map = (struct regmap *)reg;

	return regmap_update_bits(map, offset, mask, val);
}

static int clk_regmap_might_sleep(void* reg)
{
	struct regmap *map = (struct regmap *)reg;

	return regmap_might_sleep(map);
}

const struct clk_reg_ops clk_regmap_ops = {
	.read = clk_regmap_read,
	.write = clk_regmap_write,
	.update = clk_regmap_update,
	.reg_might_sleep = clk_regmap_might_sleep,
};
