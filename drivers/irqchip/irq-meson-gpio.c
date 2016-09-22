/*
 * Copyright (c) 2015 Endless Mobile, Inc.
 * Author: Carlo Caione <carlo@endlessm.com>
 * Copyright (c) 2016 BayLibre, SAS.
 * Author: Jerome Brunet <jbrunet@baylibre.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/io.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define NUM_UPSTREAM_IRQ 8
#define MAX_INPUT_MUX 256

#define REG_EDGE_POL	0x00
#define REG_PIN_03_SEL	0x04
#define REG_PIN_47_SEL	0x08
#define REG_FILTER_SEL	0x0c

#define REG_EDGE_POL_MASK(x)	(BIT(x) | BIT(16 + (x)))
#define REG_EDGE_POL_EDGE(x)	BIT(x)
#define REG_EDGE_POL_LOW(x)	BIT(16 + (x))
#define REG_PIN_SEL_SHIFT(x)	(((x) % 4) * 8)
#define REG_FILTER_SEL_SHIFT(x)	((x) * 4)

struct meson_gpio_irq_controller {
	void __iomem *base;
	unsigned long upstream_irq[NUM_UPSTREAM_IRQ];
	DECLARE_BITMAP(map, NUM_UPSTREAM_IRQ);
	spinlock_t lock;
};

static void meson_gpio_irq_update_bits(struct meson_gpio_irq_controller *ctl,
				       unsigned int reg, u32 mask, u32 val)
{
	u32 tmp;
	unsigned long flags;

	spin_lock_irqsave(&ctl->lock, flags);

	tmp = readl_relaxed(ctl->base + reg);
	tmp &= ~mask;
	tmp |= val;
	writel_relaxed(tmp, ctl->base + reg);

	spin_unlock_irqrestore(&ctl->lock, flags);
}

static int
meson_gpio_irq_request_channel(struct meson_gpio_irq_controller *ctl,
			       unsigned long  hwirq,
			       unsigned long  **parent_hwirq)
{
	unsigned int reg, channel;

	/* Find a free channel */
	channel = find_first_zero_bit(ctl->map, NUM_UPSTREAM_IRQ);
	if (channel >= NUM_UPSTREAM_IRQ) {
		pr_err("No channel available\n");
		return -ENOSPC;
	}

	/* Mark the channel as used */
	set_bit(channel, ctl->map);

	/*
	 * Setup the mux of the channel to route the signal of the pad
	 * to the appropriate input of the GIC
	 */
	reg = (channel < 4) ? REG_PIN_03_SEL : REG_PIN_47_SEL;
	meson_gpio_irq_update_bits(ctl, reg,
				   0xff << REG_PIN_SEL_SHIFT(channel),
				   hwirq << REG_PIN_SEL_SHIFT(channel));

	/* Get the parent hwirq number assigned to this channel */
	*parent_hwirq = &(ctl->upstream_irq[channel]);

	pr_debug("hwirq %lu assigned to channel %d - parent %lu\n",
		 hwirq, channel, **parent_hwirq);

	return 0;
}

static unsigned int
meson_gpio_irq_get_channel(struct meson_gpio_irq_controller *ctl,
			   unsigned long *parent_hwirq)
{
	return parent_hwirq - ctl->upstream_irq;
}

static void
meson_gpio_irq_release_channel(struct meson_gpio_irq_controller *ctl,
			       unsigned int channel)
{
	clear_bit(channel, ctl->map);
}

static int meson_gpio_irq_type_setup(struct meson_gpio_irq_controller *ctl,
				     unsigned int type,
				     int channel)
{
	u32 val = 0;

	/*
	 * The controller has a filter block to operate in either LEVEL or
	 * EDGE mode, then signal is sent to the GIC. To enable LEVEL_LOW and
	 * EDGE_FALLING support (which the GIC does not support), the filter
	 * block is also able to invert the input signal it gets before
	 * providing it to the GIC.
	 */
	type &= IRQ_TYPE_SENSE_MASK;

	if (type == IRQ_TYPE_EDGE_BOTH)
		return -EINVAL;

	if (type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING))
		val |= REG_EDGE_POL_EDGE(channel);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_EDGE_FALLING))
		val |= REG_EDGE_POL_LOW(channel);

	meson_gpio_irq_update_bits(ctl, REG_EDGE_POL,
				   REG_EDGE_POL_MASK(channel), val);

	return 0;
}

static unsigned int meson_gpio_irq_type_output(unsigned int type)
{
	unsigned int sense = type & IRQ_TYPE_SENSE_MASK;

	type &= ~IRQ_TYPE_SENSE_MASK;

	/*
	 * The polarity of the signal provided to the GIC should always
	 * be high.
	 */
	if (sense & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW))
		type |= IRQ_TYPE_LEVEL_HIGH;
	else if (sense & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING))
		type |= IRQ_TYPE_EDGE_RISING;

	return type;
}

static int meson_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct meson_gpio_irq_controller *ctl = data->domain->host_data;
	unsigned long  *parent_hwirq = irq_data_get_irq_chip_data(data);
	unsigned int channel;
	int ret;

	channel = meson_gpio_irq_get_channel(ctl, parent_hwirq);

	pr_debug("set type of channel %u to %u\n", channel, type);

	ret = meson_gpio_irq_type_setup(ctl, type, channel);
	if (ret)
		return ret;

	return irq_chip_set_type_parent(data,
					meson_gpio_irq_type_output(type));
}

static struct irq_chip meson_gpio_irq_chip = {
	.name			= "meson-gpio-irqchip",
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_type		= meson_gpio_irq_set_type,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
#ifdef CONFIG_SMP
	.irq_set_affinity	= irq_chip_set_affinity_parent,
#endif
	.flags			= IRQCHIP_SET_TYPE_MASKED,
};

static int meson_gpio_irq_domain_translate(struct irq_domain *domain,
					   struct irq_fwspec *fwspec,
					   unsigned long *hwirq,
					   unsigned int *type)
{
	if (is_of_node(fwspec->fwnode) && fwspec->param_count == 2) {
		*hwirq	= fwspec->param[0];
		*type	= fwspec->param[1];
		return 0;
	}

	return -EINVAL;
}

static int meson_gpio_irq_allocate_gic_irq(struct irq_domain *domain,
					   unsigned int virq,
					   unsigned long hwirq,
					   unsigned int type)
{
	struct irq_fwspec fwspec;

	fwspec.fwnode = domain->parent->fwnode;
	fwspec.param_count = 3;
	fwspec.param[0] = 0;	/* SPI */
	fwspec.param[1] = hwirq;
	fwspec.param[2] = meson_gpio_irq_type_output(type);

	return irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
}

static int meson_gpio_irq_domain_alloc(struct irq_domain *domain,
				       unsigned int virq,
				       unsigned int nr_irqs,
				       void *data)
{
	struct irq_fwspec *fwspec = data;
	struct meson_gpio_irq_controller *ctl = domain->host_data;
	unsigned long hwirq;
	unsigned long *parent_hwirq;
	unsigned int type;
	int ret;

	if (WARN_ON(nr_irqs != 1))
		return -EINVAL;

	ret = meson_gpio_irq_domain_translate(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	pr_err("=== alloc irq %d, nr_irqs %d, hwirqs %lu\n", virq, nr_irqs, hwirq);

	ret = meson_gpio_irq_request_channel(ctl, hwirq, &parent_hwirq);
	if (ret)
		return ret;

	ret = meson_gpio_irq_allocate_gic_irq(domain, virq,
					      *parent_hwirq, type);
	if (ret < 0) {
		pr_err("failed to allocate gic irq %lu\n", *parent_hwirq);
		return ret;
	}

	irq_domain_set_hwirq_and_chip(domain, virq, hwirq,
				      &meson_gpio_irq_chip, parent_hwirq);

	return 0;
}

static void meson_gpio_irq_domain_free(struct irq_domain *domain,
				       unsigned int virq,
				       unsigned int nr_irqs)
{
	struct meson_gpio_irq_controller *ctl = domain->host_data;
	struct irq_data *irq_data;
	unsigned long *parent_hwirq;
	unsigned int channel;

	if (WARN_ON(nr_irqs != 1))
		return;

	pr_err("=== free irq %d, nr_irqs %d\n", virq, nr_irqs);

	irq_domain_free_irqs_parent(domain, virq, 1);

	irq_data = irq_domain_get_irq_data(domain, virq);
	parent_hwirq = irq_data_get_irq_chip_data(irq_data);
	channel = meson_gpio_irq_get_channel(ctl, parent_hwirq);

	meson_gpio_irq_release_channel(ctl, channel);
}

static const struct irq_domain_ops meson_gpio_irq_domain_ops = {
	.alloc		= meson_gpio_irq_domain_alloc,
	.free		= meson_gpio_irq_domain_free,
	.translate	= meson_gpio_irq_domain_translate,
};

static int __init meson_gpio_irq_parse_dt(struct device_node *node,
					  struct meson_gpio_irq_controller *ctl,
					  u32 *num_hwirq)
{
	struct property *p;
	const __be32 *c;
	const char *irq_prop = "meson,upstream-interrupts";
	u32 val;
	int channel = 0, ret;

	ret = of_property_read_u32(node, "meson,num-input-mux", num_hwirq);
	if (ret)
		return ret;

	if (*num_hwirq > MAX_INPUT_MUX) {
		pr_err("too many input for mux\n");
		return -EINVAL;
	}

        ret = of_property_count_u32_elems(node, irq_prop);
	if (ret < 0) {
		pr_err("error reading property %s\n", irq_prop);
		return ret;
	}

	if (ret != NUM_UPSTREAM_IRQ) {
		pr_err("controller should have %d upstream interrupts\n",
			NUM_UPSTREAM_IRQ);
		return -EINVAL;
	}

	of_property_for_each_u32(node, irq_prop, p, c, val) {
		ctl->upstream_irq[channel] = val;
		channel++;
	}

	return 0;
}

static int __init meson_gpio_irq_of_init(struct device_node *node,
					 struct device_node *parent)
{
	struct irq_domain *domain, *parent_domain;
	struct meson_gpio_irq_controller *ctl;
	u32 num_hwirq;
	int ret;

	if (!parent) {
		pr_err("missing parent interrupt node\n");
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("unable to obtain parent domain\n");
		return -ENXIO;
	}

	ctl = kzalloc(sizeof(*ctl), GFP_KERNEL);
	if (!ctl)
		return -ENOMEM;

	spin_lock_init(&ctl->lock);

	ctl->base = of_iomap(node, 0);
	if (!ctl->base) {
		ret = -ENOMEM;
		goto free_ctl;
	}

	bitmap_zero(ctl->map, NUM_UPSTREAM_IRQ);

	ret = meson_gpio_irq_parse_dt(node, ctl, &num_hwirq);
	if (ret)
		goto free_upstream_irq;

	domain = irq_domain_create_hierarchy(parent_domain, 0, num_hwirq,
					     of_node_to_fwnode(node),
					     &meson_gpio_irq_domain_ops,
					     ctl);
	if (!domain) {
		pr_err("failed to add domain\n");
		ret = -ENODEV;
		goto free_upstream_irq;
	}

	pr_info("%d to %d gpio interrupt mux initialized\n",
		num_hwirq, NUM_UPSTREAM_IRQ);

	return 0;

free_upstream_irq:
	kfree(ctl->upstream_irq);
	iounmap(ctl->base);
free_ctl:
	kfree(ctl);

	return ret;
}

IRQCHIP_DECLARE(meson_gpio_intc, "amlogic,meson-gpio-intc",
		meson_gpio_irq_of_init);
IRQCHIP_DECLARE(meson8_gpio_intc, "amlogic,meson8-gpio-intc",
		meson_gpio_irq_of_init);
IRQCHIP_DECLARE(meson8b_gpio_intc, "amlogic,meson8b-gpio-intc",
		meson_gpio_irq_of_init);
IRQCHIP_DECLARE(meson_gxbb_gpio_intc, "amlogic,meson-gxbb-gpio-intc",
		meson_gpio_irq_of_init);
