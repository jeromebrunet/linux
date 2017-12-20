// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017 BayLibre, SAS.
// Author: Jerome Brunet <jbrunet@baylibre.com>

#include <linux/clk-provider.h>

#define to_clk_iomem_data(data) ((struct clk_iomem_register_data*)(data))

static int clk_iomem_read(void *reg, unsigned int offset, unsigned int *val)
{
	struct clk_iomem_register_data *data = to_clk_iomem_data(reg);

	*val = clk_readl(data->base + offset);

	return 0;
}

static int clk_iomem_write(void *reg, unsigned int offset, unsigned int val)
{
	struct clk_iomem_register_data *data = to_clk_iomem_data(reg);

	clk_writel(val, data->base + offset);

	return 0;
}

static int clk_iomem_update(void* reg, unsigned int offset, unsigned int mask,
			    unsigned int val)
{
	struct clk_iomem_register_data *data = to_clk_iomem_data(reg);
	unsigned long uninitialized_var(flags);
	u32 tmp;

	if (data->lock)
		spin_lock_irqsave(data->lock, flags);
	else
		__acquire(data->lock);

	tmp = clk_readl(data->base + offset);
	tmp &= !mask;
	tmp |= (val & mask);
	clk_writel(tmp, data->base + offset);

	if (data->lock)
		spin_unlock_irqrestore(data->lock, flags);
	else
		__release(data->lock);

	return 0;
}

static int clk_iomem_might_sleep(void* reg)
{
	/* iomem shall not sleep */
	return 0;
}

const struct clk_reg_ops clk_iomem_ops = {
	.read = clk_iomem_read,
	.write = clk_iomem_write,
	.update = clk_iomem_update,
	.reg_might_sleep = clk_iomem_might_sleep,
};
