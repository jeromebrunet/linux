/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __DW_HDMI__
#define __DW_HDMI__

#include <drm/drmP.h>

struct dw_hdmi;

enum {
	DW_HDMI_RES_8,
	DW_HDMI_RES_10,
	DW_HDMI_RES_12,
	DW_HDMI_RES_MAX,
};

enum dw_hdmi_color_enc_format {
	DW_HDMI_ENC_FMT_RGB = 0,
	DW_HDMI_ENC_FMT_YCBCR444,
	DW_HDMI_ENC_FMT_YCBCR422_16BITS,
	DW_HDMI_ENC_FMT_YCBCR422_8BITS,
	DW_HDMI_ENC_FMT_XVYCC444,
};

enum dw_hdmi_phy_type {
	DW_HDMI_PHY_DWC_HDMI_TX_PHY = 0x00,
	DW_HDMI_PHY_DWC_MHL_PHY_HEAC = 0xb2,
	DW_HDMI_PHY_DWC_MHL_PHY = 0xc2,
	DW_HDMI_PHY_DWC_HDMI_3D_TX_PHY_HEAC = 0xe2,
	DW_HDMI_PHY_DWC_HDMI_3D_TX_PHY = 0xf2,
	DW_HDMI_PHY_DWC_HDMI20_TX_PHY = 0xf3,
	DW_HDMI_PHY_VENDOR_PHY = 0xfe,
};

struct dw_hdmi_mpll_config {
	unsigned long mpixelclock;
	struct {
		u16 cpce;
		u16 gmp;
	} res[DW_HDMI_RES_MAX];
};

struct dw_hdmi_curr_ctrl {
	unsigned long mpixelclock;
	u16 curr[DW_HDMI_RES_MAX];
};

struct dw_hdmi_phy_config {
	unsigned long mpixelclock;
	u16 sym_ctr;    /*clock symbol and transmitter control*/
	u16 term;       /*transmission termination value*/
	u16 vlev_ctr;   /* voltage level control */
};

struct dw_hdmi_phy_ops {
	int (*init)(struct dw_hdmi *hdmi, void *data,
		    struct drm_display_mode *mode);
	void (*disable)(struct dw_hdmi *hdmi, void *data);
	enum drm_connector_status (*read_hpd)(struct dw_hdmi *hdmi, void *data);
	void (*update_hpd)(struct dw_hdmi *hdmi, void *data,
			   bool force, bool disabled, bool rxsense);
	void (*setup_hpd)(struct dw_hdmi *hdmi, void *data);
	void (*configure_hpd)(struct dw_hdmi *hdmi, void *data);
	void (*clear_hpd)(struct dw_hdmi *hdmi, void *data);
	void (*unmute_hpd)(struct dw_hdmi *hdmi, void *data);
};

struct dw_hdmi_plat_data {
	struct regmap *regm;
	enum drm_mode_status (*mode_valid)(struct drm_connector *connector,
					   struct drm_display_mode *mode);
	enum dw_hdmi_color_enc_format input_fmt;

	/* Vendor PHY support */
	const struct dw_hdmi_phy_ops *phy_ops;
	const char *phy_name;
	void *phy_data;

	/* Synopsys PHY support */
	const struct dw_hdmi_mpll_config *mpll_cfg;
	const struct dw_hdmi_curr_ctrl *cur_ctr;
	const struct dw_hdmi_phy_config *phy_config;
	int (*configure_phy)(struct dw_hdmi *hdmi,
			     const struct dw_hdmi_plat_data *pdata,
			     unsigned long mpixelclock);
};

int dw_hdmi_probe(struct platform_device *pdev,
		  const struct dw_hdmi_plat_data *plat_data);
void dw_hdmi_remove(struct platform_device *pdev);
void dw_hdmi_unbind(struct device *dev);
int dw_hdmi_bind(struct platform_device *pdev, struct drm_encoder *encoder,
		 const struct dw_hdmi_plat_data *plat_data);

void dw_hdmi_setup_rx_sense(struct device *dev, bool hpd, bool rx_sense);

void dw_hdmi_set_sample_rate(struct dw_hdmi *hdmi, unsigned int rate);
void dw_hdmi_audio_enable(struct dw_hdmi *hdmi);
void dw_hdmi_audio_disable(struct dw_hdmi *hdmi);

/* PHY configuration */
void dw_hdmi_phy_i2c_write(struct dw_hdmi *hdmi, unsigned short data,
			   unsigned char addr);

#endif /* __IMX_HDMI_H__ */
