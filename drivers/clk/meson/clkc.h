/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2015 Endless Mobile, Inc.
 * Author: Carlo Caione <carlo@endlessm.com>
 */

#ifndef __CLKC_H
#define __CLKC_H

#include <linux/clk-provider.h>
#include "clk-regmap.h"
#include "parm.h"

struct meson_clk_dualdiv_param {
	unsigned int n1;
	unsigned int n2;
	unsigned int m1;
	unsigned int m2;
	unsigned int dual;
};

struct meson_clk_dualdiv_data {
	struct parm n1;
	struct parm n2;
	struct parm m1;
	struct parm m2;
	struct parm dual;
	const struct meson_clk_dualdiv_param *table;
};

/* clk_ops */
extern const struct clk_ops meson_clk_cpu_ops;
extern const struct clk_ops meson_clk_dualdiv_ops;
extern const struct clk_ops meson_clk_dualdiv_ro_ops;

struct clk_hw *meson_clk_hw_register_input(struct device *dev,
					   const char *of_name,
					   const char *clk_name,
					   unsigned long flags);

#endif /* __CLKC_H */
