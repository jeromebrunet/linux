/*
 * Amlogic Meson8b and GXBB DWMAC glue layer
 *
 * Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of_net.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>

#include "stmmac_platform.h"

#define PRG_ETH0			0x0

#define PRG_ETH0_RGMII_MODE		BIT(0)

/* mux to choose between fclk_div2 (bit unset) and mpll2 (bit set) */
#define PRG_ETH0_CLK_M250_SEL_SHIFT	4
#define PRG_ETH0_CLK_M250_SEL_MASK	GENMASK(4, 4)

#define PRG_ETH0_TXDLY_SHIFT		5
#define PRG_ETH0_TXDLY_MASK		GENMASK(6, 5)

/* divider for the result of m250_sel */
#define PRG_ETH0_CLK_M250_DIV_SHIFT	7
#define PRG_ETH0_CLK_M250_DIV_WIDTH	3

/* divides the result of m25_sel by either 5 (bit unset) or 10 (bit set) */
#define PRG_ETH0_CLK_M25_DIV_SHIFT	10
#define PRG_ETH0_CLK_M25_DIV_WIDTH	1

#define PRG_ETH0_INVERTED_RMII_CLK	BIT(11)
#define PRG_ETH0_TX_AND_PHY_REF_CLK	BIT(12)

#define MUX_CLK_NUM_PARENTS		2

struct meson8b_dwmac {
	struct platform_device	*pdev;

	void __iomem		*regs;

	phy_interface_t		phy_mode;

	struct clk		*m250_mux_clk;
	struct clk		*m25_div_clk;

	u32			tx_delay_ns;
};

static void meson8b_dwmac_mask_bits(struct meson8b_dwmac *dwmac, u32 reg,
				    u32 mask, u32 value)
{
	u32 data;

	data = readl(dwmac->regs + reg);
	data &= ~mask;
	data |= (value & mask);

	writel(data, dwmac->regs + reg);
}

static const struct clk_div_table clk_25m_div_table[] = {
	{ .val = 0, .div = 5 },
	{ .val = 1, .div = 10 },
	{ /* sentinel */ },
};

static int meson8b_init_clk(struct meson8b_dwmac *dwmac)
{
	struct device *dev = &dwmac->pdev->dev;
	struct clk_init_data init;
	struct clk_init_reg_data reg_init;
	struct clk_iomem_register_data *data;
	struct clk_mux_ng *mux;
	struct clk_divider_ng *div_25, *div_250;
	struct clk *clk;
	int i, ret;
	char clk_name[32];
	const char *div_parent_names[1];
	const char *mux_parent_names[MUX_CLK_NUM_PARENTS];

	/* get the mux parents from DT */
	for (i = 0; i < MUX_CLK_NUM_PARENTS; i++) {
		char name[16];

		snprintf(name, sizeof(name), "clkin%d", i);
		clk = devm_clk_get(dev, name);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Missing clock %s\n", name);
			return ret;
		}

		mux_parent_names[i] = __clk_get_name(clk);
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->base = dwmac->regs;
	reg_init.data = data;
	reg_init.ops = &clk_iomem_ops;

	mux = devm_kzalloc(dev, sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return -ENOMEM;

	/* create the m250_mux */
	snprintf(clk_name, sizeof(clk_name), "%s#m250_sel", dev_name(dev));
	init.name = clk_name;
	init.ops = &clk_mux_ng_ops;
	init.flags = 0;
	init.parent_names = mux_parent_names;
	init.num_parents = MUX_CLK_NUM_PARENTS;

	mux->offset = PRG_ETH0;
	mux->shift = PRG_ETH0_CLK_M250_SEL_SHIFT;
	mux->mask = PRG_ETH0_CLK_M250_SEL_MASK;
	mux->hw.init = &init;
	mux->hw.reg_init = &reg_init;

	dwmac->m250_mux_clk = devm_clk_register(dev, &mux->hw);
	if (WARN_ON(IS_ERR(dwmac->m250_mux_clk)))
		return PTR_ERR(dwmac->m250_mux_clk);

	/* create the m250 divider */
	div_250 = devm_kzalloc(dev, sizeof(*div_250), GFP_KERNEL);
	if (!div_250)
		return -ENOMEM;

	snprintf(clk_name, sizeof(clk_name), "%s#m250_div", dev_name(dev));
	init.name = clk_name;
	init.ops = &clk_divider_ng_ops;
	init.flags = CLK_SET_RATE_PARENT;
	div_parent_names[0] = __clk_get_name(dwmac->m250_mux_clk);
	init.parent_names = div_parent_names;
	init.num_parents = ARRAY_SIZE(div_parent_names);

	div_250->offset = PRG_ETH0;
	div_250->shift = PRG_ETH0_CLK_M250_DIV_SHIFT;
	div_250->width = PRG_ETH0_CLK_M250_DIV_WIDTH;
	div_250->hw.init = &init;
	div_250->hw.reg_init = &reg_init;
	div_250->flags = CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO;

	clk = devm_clk_register(dev, &div_250->hw);
	if (WARN_ON(IS_ERR(clk)))
		return PTR_ERR(clk);

	/* create the m250 divider */
	div_25 = devm_kzalloc(dev, sizeof(*div_25), GFP_KERNEL);
	if (!div_25)
		return -ENOMEM;

	snprintf(clk_name, sizeof(clk_name), "%s#m25_div", dev_name(dev));
	init.name = clk_name;
	init.ops = &clk_divider_ng_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	div_parent_names[0] = __clk_get_name(clk);
	init.parent_names = div_parent_names;
	init.num_parents = ARRAY_SIZE(div_parent_names);

	div_25->offset= PRG_ETH0;
	div_25->shift = PRG_ETH0_CLK_M25_DIV_SHIFT;
	div_25->width = PRG_ETH0_CLK_M25_DIV_WIDTH;
	div_25->table = clk_25m_div_table;
	div_25->hw.init = &init;
	div_25->hw.reg_init = &reg_init;
	div_25->flags = CLK_DIVIDER_ALLOW_ZERO;

	dwmac->m25_div_clk = devm_clk_register(dev, &div_25->hw);
	if (WARN_ON(IS_ERR(dwmac->m25_div_clk)))
		return PTR_ERR(dwmac->m25_div_clk);

	return 0;
}

static int meson8b_init_prg_eth(struct meson8b_dwmac *dwmac)
{
	int ret;
	unsigned long clk_rate;
	u8 tx_dly_val = 0;

	switch (dwmac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_RXID:
		/* TX clock delay in ns = "8ns / 4 * tx_dly_val" (where
		 * 8ns are exactly one cycle of the 125MHz RGMII TX clock):
		 * 0ns = 0x0, 2ns = 0x1, 4ns = 0x2, 6ns = 0x3
		 */
		tx_dly_val = dwmac->tx_delay_ns >> 1;
		/* fall through */

	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		/* Generate a 25MHz clock for the PHY */
		clk_rate = 25 * 1000 * 1000;

		/* enable RGMII mode */
		meson8b_dwmac_mask_bits(dwmac, PRG_ETH0, PRG_ETH0_RGMII_MODE,
					PRG_ETH0_RGMII_MODE);

		/* only relevant for RMII mode -> disable in RGMII mode */
		meson8b_dwmac_mask_bits(dwmac, PRG_ETH0,
					PRG_ETH0_INVERTED_RMII_CLK, 0);

		meson8b_dwmac_mask_bits(dwmac, PRG_ETH0, PRG_ETH0_TXDLY_MASK,
					tx_dly_val << PRG_ETH0_TXDLY_SHIFT);
		break;

	case PHY_INTERFACE_MODE_RMII:
		/* Use the rate of the mux clock for the internal RMII PHY */
		clk_rate = clk_get_rate(dwmac->m250_mux_clk);

		/* disable RGMII mode -> enables RMII mode */
		meson8b_dwmac_mask_bits(dwmac, PRG_ETH0, PRG_ETH0_RGMII_MODE,
					0);

		/* invert internal clk_rmii_i to generate 25/2.5 tx_rx_clk */
		meson8b_dwmac_mask_bits(dwmac, PRG_ETH0,
					PRG_ETH0_INVERTED_RMII_CLK,
					PRG_ETH0_INVERTED_RMII_CLK);

		/* TX clock delay cannot be configured in RMII mode */
		meson8b_dwmac_mask_bits(dwmac, PRG_ETH0, PRG_ETH0_TXDLY_MASK,
					0);

		break;

	default:
		dev_err(&dwmac->pdev->dev, "unsupported phy-mode %s\n",
			phy_modes(dwmac->phy_mode));
		return -EINVAL;
	}

	ret = clk_prepare_enable(dwmac->m25_div_clk);
	if (ret) {
		dev_err(&dwmac->pdev->dev, "failed to enable the PHY clock\n");
		return ret;
	}

	ret = clk_set_rate(dwmac->m25_div_clk, clk_rate);
	if (ret) {
		clk_disable_unprepare(dwmac->m25_div_clk);

		dev_err(&dwmac->pdev->dev, "failed to set PHY clock\n");
		return ret;
	}

	/* enable TX_CLK and PHY_REF_CLK generator */
	meson8b_dwmac_mask_bits(dwmac, PRG_ETH0, PRG_ETH0_TX_AND_PHY_REF_CLK,
				PRG_ETH0_TX_AND_PHY_REF_CLK);

	return 0;
}

static int meson8b_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct resource *res;
	struct meson8b_dwmac *dwmac;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac) {
		ret = -ENOMEM;
		goto err_remove_config_dt;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	dwmac->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dwmac->regs)) {
		ret = PTR_ERR(dwmac->regs);
		goto err_remove_config_dt;
	}

	dwmac->pdev = pdev;
	dwmac->phy_mode = of_get_phy_mode(pdev->dev.of_node);
	if (dwmac->phy_mode < 0) {
		dev_err(&pdev->dev, "missing phy-mode property\n");
		ret = -EINVAL;
		goto err_remove_config_dt;
	}

	/* use 2ns as fallback since this value was previously hardcoded */
	if (of_property_read_u32(pdev->dev.of_node, "amlogic,tx-delay-ns",
				 &dwmac->tx_delay_ns))
		dwmac->tx_delay_ns = 2;

	ret = meson8b_init_clk(dwmac);
	if (ret)
		goto err_remove_config_dt;

	ret = meson8b_init_prg_eth(dwmac);
	if (ret)
		goto err_remove_config_dt;

	plat_dat->bsp_priv = dwmac;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_clk_disable;

	return 0;

err_clk_disable:
	clk_disable_unprepare(dwmac->m25_div_clk);
err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static int meson8b_dwmac_remove(struct platform_device *pdev)
{
	struct meson8b_dwmac *dwmac = get_stmmac_bsp_priv(&pdev->dev);

	clk_disable_unprepare(dwmac->m25_div_clk);

	return stmmac_pltfr_remove(pdev);
}

static const struct of_device_id meson8b_dwmac_match[] = {
	{ .compatible = "amlogic,meson8b-dwmac" },
	{ .compatible = "amlogic,meson-gxbb-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, meson8b_dwmac_match);

static struct platform_driver meson8b_dwmac_driver = {
	.probe  = meson8b_dwmac_probe,
	.remove = meson8b_dwmac_remove,
	.driver = {
		.name           = "meson8b-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = meson8b_dwmac_match,
	},
};
module_platform_driver(meson8b_dwmac_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Amlogic Meson8b and GXBB DWMAC glue layer");
MODULE_LICENSE("GPL v2");
