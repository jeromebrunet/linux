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

struct pll_params_table {
	u16		m;
	u16		n;
};

#define PLL_PARAMS(_m, _n)						\
	{								\
		.m		= (_m),					\
		.n		= (_n),					\
	}

#define CLK_MESON_PLL_ROUND_CLOSEST	BIT(0)

struct meson_clk_pll_data {
	struct parm en;
	struct parm m;
	struct parm n;
	struct parm frac;
	struct parm l;
	struct parm rst;
	const struct reg_sequence *init_regs;
	unsigned int init_count;
	const struct pll_params_table *table;
	u8 flags;
};

#define to_meson_clk_pll(_hw) container_of(_hw, struct meson_clk_pll, hw)

struct meson_clk_mpll_data {
	struct parm sdm;
	struct parm sdm_en;
	struct parm n2;
	struct parm ssen;
	struct parm misc;
	spinlock_t *lock;
	u8 flags;
};

#define CLK_MESON_MPLL_ROUND_CLOSEST	BIT(0)

struct meson_clk_phase_data {
	struct parm ph;
};

int meson_clk_degrees_from_val(unsigned int val, unsigned int width);
unsigned int meson_clk_degrees_to_val(int degrees, unsigned int width);

struct meson_vid_pll_div_data {
	struct parm val;
	struct parm sel;
};

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
extern const struct clk_ops meson_clk_pll_ro_ops;
extern const struct clk_ops meson_clk_pll_ops;
extern const struct clk_ops meson_clk_cpu_ops;
extern const struct clk_ops meson_clk_mpll_ro_ops;
extern const struct clk_ops meson_clk_mpll_ops;
extern const struct clk_ops meson_clk_phase_ops;
extern const struct clk_ops meson_vid_pll_div_ro_ops;
extern const struct clk_ops meson_clk_dualdiv_ops;
extern const struct clk_ops meson_clk_dualdiv_ro_ops;

struct clk_hw *meson_clk_hw_register_input(struct device *dev,
					   const char *of_name,
					   const char *clk_name,
					   unsigned long flags);

#endif /* __CLKC_H */
