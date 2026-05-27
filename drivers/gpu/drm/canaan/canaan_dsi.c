// SPDX-License-Identifier: GPL-2.0-only
/*
 * Canaan K230 MIPI-DSI glue for the Synopsys DesignWare MIPI DSI host.
 *
 * The K230 DSI host is a stock Synopsys DW MIPI DSI controller, so the
 * generic drivers/gpu/drm/bridge/synopsys/dw-mipi-dsi.c library owns all
 * of the host programming. This glue only provides the K230-specific
 * D-PHY bring-up (an internal PHY whose test/PLL registers live in the
 * same MMIO region at a +0x400 offset) and the pixel clock handling.
 *
 * Copyright (C) 2022, Canaan Bright Sight Co., Ltd
 * Copyright (C) 2026 Sungjoon Moon <sumoon@seoulsaram.org>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>
#include <drm/drm_simple_kms_helper.h>

/* DW MIPI DSI host registers used directly by the PHY bring-up. */
#define LPCLK_CTRL			0x94
#define PHY_RSTZ			0xa0
#define PHY_TST_CTRL0			0xb4
#define PHY_TST_CTRL1			0xb8
#define PHY_STATUS			0xb0

/*
 * PHY_RSTZ field combinations mandated by the K230 bring-up sequence.
 * bit0 = shutdownz, bit1 = rstz, bit2 = enableclk, bit3 = enforcepll.
 */
#define PHY_RSTZ_ENFORCEPLL_CLK		0xc	/* enforcepll | enableclk */
#define PHY_RSTZ_ENABLECLK		0x4	/* enableclk */
#define PHY_RSTZ_ENFORCEPLL_CLK_SHDN	0xd	/* enforcepll | enableclk | shutdownz */
#define PHY_RSTZ_ENABLE_ALL		0xf	/* enforcepll | enableclk | rstz | shutdownz */

/* PHY test interface (PHY_TST_CTRL0/1). */
#define PHY_TST_CTRL0_CLK		BIT(1)
#define PHY_TST_CTRL0_CLR		BIT(0)
#define PHY_TST_CTRL1_EN		BIT(16)

/* All PHY config registers live in a dedicated block at base + 0x400. */
#define K230_PHY_BLOCK			0x400
#define TXDPHY_CFG0			0x00
#define TXDPHY_CFG1			0x04
#define TXDPHY_PLL_CFG0			0x08
#define TXDPHY_PLL_CFG1			0x10

/* TXDPHY_CFG0 fields. */
#define TXDPHY_CFG0_PHY1_BASEDIR	BIT(1)
#define TXDPHY_CFG0_PHY0_CFGCLKRANGE	GENMASK(7, 2)
#define TXDPHY_CFG0_PHY1_CFGCLKRANGE	GENMASK(13, 8)
#define TXDPHY_CFG0_CFGCLKRANGE_VAL	0x28	/* 25 MHz cfg clock */
/* PHY0 enable word: cfgclkfreqrange for both lanes plus enable bit. */
#define TXDPHY_CFG0_PHY0_ENABLE		0x28a0

/* TXDPHY_CFG1: phy select. */
#define TXDPHY_CFG1_PHY0_SEL		0x0
#define TXDPHY_CFG1_PHY1_SEL		0x400000

/* TXDPHY_PLL_CFG0 fields. */
#define PLL_CFG0_CPBIAS_CNTRL		GENMASK(6, 0)
#define PLL_CFG0_GMP_CNTRL		GENMASK(9, 8)
#define PLL_CFG0_INT_CNTRL		GENMASK(16, 11)
#define PLL_CFG0_M			GENMASK(26, 17)
#define PLL_CFG0_N			GENMASK(30, 27)

/* TXDPHY_PLL_CFG1 fields. */
#define PLL_CFG1_CLKSEL			GENMASK(1, 0)
#define PLL_CFG1_PROP_CNTRL		GENMASK(8, 3)
#define PLL_CFG1_SHADOW_CLEAR		BIT(9)
#define PLL_CFG1_UPDATEPLL		BIT(10)
#define PLL_CFG1_VCO_CNTRL		GENMASK(16, 11)

/* PHY test interface register addresses (8-bit). */
#define PHY_REG_MASTERMACRO		0x0c
#define PHY_REG_FSM_MONITOR		0x03
#define PHY_REG_PLL_TH1			0x14
#define PHY_REG_PLL_TH2			0x15
#define PHY_REG_PLL_TH3			0x16
#define PHY_REG_PLL_LOCK_SEL		0x1d
#define PHY_REG_MPLL_PROG		0x1f
#define PHY_REG_HS_CLK_LANE		0x30
#define PHY_REG_HSFREQRANGE		0x44
#define PHY_REG_PRG_ON_LANE0		0x4a
#define PHY_REG_SLEW_SR_RANGE		0xa0
#define PHY_REG_SLEW_OSC_FREQ		0xa4
#define PHY_REG_SLEW_CAL_EN		0xa3

/*
 * Magic value seen on PHY_STATUS once both lanes report stop state, and the
 * PHY1 FSM monitor read-back once the FSM has settled. Both are compared as an
 * exact match against values validated on K230 hardware. If a future SoC
 * revision toggles reserved bits, these may need masking down to the
 * meaningful PHY_LOCK/STOP_STATE bits instead of an exact compare.
 */
#define PHY_STATUS_READY		0x1fbd
#define PHY1_FSM_READY			0x580

#define PHY_TIMEOUT_US			200000
#define PHY_SLEEP_US			1000

/* HS frequency range selector for the K230 1.5 Gbps/lane operating point. */
#define K230_HSFREQRANGE		0x96

struct k230_dsi {
	struct device *dev;
	void __iomem *base;
	struct clk *pixel;
	struct dw_mipi_dsi *dmd;
	struct drm_encoder encoder;
	struct dw_mipi_dsi_plat_data pdata;

	/* PLL parameters computed in get_lane_mbps, consumed by init. */
	u32 phy_m;
	u32 phy_n;
	u8 phy_vco;
	u8 phy_hsfreq;
};

static inline void dsi_write(struct k230_dsi *dsi, u32 reg, u32 val)
{
	writel(val, dsi->base + reg);
}

static inline u32 dsi_read(struct k230_dsi *dsi, u32 reg)
{
	return readl(dsi->base + reg);
}

static inline void phy_write(struct k230_dsi *dsi, u32 reg, u32 val)
{
	writel(val, dsi->base + K230_PHY_BLOCK + reg);
}

static inline u32 phy_read(struct k230_dsi *dsi, u32 reg)
{
	return readl(dsi->base + K230_PHY_BLOCK + reg);
}

static inline void phy_update(struct k230_dsi *dsi, u32 reg, u32 mask, u32 val)
{
	phy_write(dsi, reg, (phy_read(dsi, reg) & ~mask) | val);
}

/* Write an 8-bit value through the DW PHY test interface. */
static void k230_phy_test_write(struct k230_dsi *dsi, u8 addr, u8 val)
{
	dsi_write(dsi, PHY_TST_CTRL1, PHY_TST_CTRL1_EN | addr);
	dsi_write(dsi, PHY_TST_CTRL0, PHY_TST_CTRL0_CLK);
	dsi_write(dsi, PHY_TST_CTRL0, 0);

	dsi_write(dsi, PHY_TST_CTRL1, val);
	dsi_write(dsi, PHY_TST_CTRL0, PHY_TST_CTRL0_CLK);
	dsi_write(dsi, PHY_TST_CTRL0, 0);
}

static void k230_phy_pll_config(struct k230_dsi *dsi)
{
	u32 reg;

	/* Pulse shadow_clear so the PLL latches the new programming. */
	phy_update(dsi, TXDPHY_PLL_CFG1, PLL_CFG1_SHADOW_CLEAR,
		   PLL_CFG1_SHADOW_CLEAR);
	phy_update(dsi, TXDPHY_PLL_CFG1, PLL_CFG1_SHADOW_CLEAR, 0);

	phy_update(dsi, TXDPHY_PLL_CFG1, PLL_CFG1_CLKSEL,
		   FIELD_PREP(PLL_CFG1_CLKSEL, 0x1));

	/* M/N feedback and input dividers (register encoding M=m+2, N=n+1). */
	reg = phy_read(dsi, TXDPHY_PLL_CFG0);
	reg &= ~(PLL_CFG0_M | PLL_CFG0_N);
	reg |= FIELD_PREP(PLL_CFG0_M, dsi->phy_m);
	reg |= FIELD_PREP(PLL_CFG0_N, dsi->phy_n);
	phy_write(dsi, TXDPHY_PLL_CFG0, reg);

	phy_update(dsi, TXDPHY_PLL_CFG1, PLL_CFG1_VCO_CNTRL,
		   FIELD_PREP(PLL_CFG1_VCO_CNTRL, dsi->phy_vco));

	/* Charge-pump / loop-filter constants from the K230 PHY databook. */
	phy_update(dsi, TXDPHY_PLL_CFG0, PLL_CFG0_CPBIAS_CNTRL,
		   FIELD_PREP(PLL_CFG0_CPBIAS_CNTRL, 0x10));
	phy_update(dsi, TXDPHY_PLL_CFG0, PLL_CFG0_GMP_CNTRL, 0);
	phy_update(dsi, TXDPHY_PLL_CFG0, PLL_CFG0_INT_CNTRL, 0);
	phy_update(dsi, TXDPHY_PLL_CFG1, PLL_CFG1_PROP_CNTRL,
		   FIELD_PREP(PLL_CFG1_PROP_CNTRL, 0x8));

	k230_phy_test_write(dsi, PHY_REG_PLL_TH1, 0x2);
	k230_phy_test_write(dsi, PHY_REG_PLL_TH2, 0x60);
	k230_phy_test_write(dsi, PHY_REG_PLL_TH3, 0x3);
	k230_phy_test_write(dsi, PHY_REG_PLL_LOCK_SEL, 0x1);

	/* Pulse updatepll, holding it long enough for the PLL to lock. */
	phy_update(dsi, TXDPHY_PLL_CFG1, PLL_CFG1_UPDATEPLL, PLL_CFG1_UPDATEPLL);
	usleep_range(20000, 21000);
	phy_update(dsi, TXDPHY_PLL_CFG1, PLL_CFG1_UPDATEPLL, 0);
}

static void k230_phy0_config(struct k230_dsi *dsi)
{
	u32 reg;

	/* Select PHY0 (master). */
	phy_write(dsi, TXDPHY_CFG1, TXDPHY_CFG1_PHY0_SEL);
	dsi_write(dsi, PHY_RSTZ, PHY_RSTZ_ENFORCEPLL_CLK);

	/* Reset the test interface. */
	dsi_write(dsi, PHY_TST_CTRL0, PHY_TST_CTRL0_CLR);
	dsi_write(dsi, PHY_TST_CTRL0, 0);

	/* mastermacro = 1, prototyping_env = 1 */
	k230_phy_test_write(dsi, PHY_REG_MASTERMACRO, 0x03);
	k230_phy_test_write(dsi, PHY_REG_HSFREQRANGE, dsi->phy_hsfreq);

	/* Slew-rate calibration constants for < 1.5 Gbps operation. */
	k230_phy_test_write(dsi, PHY_REG_SLEW_SR_RANGE, 0x40);
	k230_phy_test_write(dsi, PHY_REG_SLEW_OSC_FREQ, 0x11);
	k230_phy_test_write(dsi, PHY_REG_SLEW_OSC_FREQ, 0x85);
	k230_phy_test_write(dsi, PHY_REG_SLEW_CAL_EN, 0x1);

	k230_phy_test_write(dsi, PHY_REG_MPLL_PROG, 0x1);
	k230_phy_test_write(dsi, PHY_REG_PRG_ON_LANE0, 0x40);

	reg = phy_read(dsi, TXDPHY_CFG0);
	reg &= ~(TXDPHY_CFG0_PHY0_CFGCLKRANGE | TXDPHY_CFG0_PHY1_CFGCLKRANGE);
	reg |= FIELD_PREP(TXDPHY_CFG0_PHY0_CFGCLKRANGE,
			  TXDPHY_CFG0_CFGCLKRANGE_VAL);
	reg |= FIELD_PREP(TXDPHY_CFG0_PHY1_CFGCLKRANGE,
			  TXDPHY_CFG0_CFGCLKRANGE_VAL);
	phy_write(dsi, TXDPHY_CFG0, reg);

	k230_phy_pll_config(dsi);

	phy_write(dsi, TXDPHY_CFG0, TXDPHY_CFG0_PHY0_ENABLE);
	dsi_write(dsi, PHY_RSTZ, PHY_RSTZ_ENFORCEPLL_CLK);
}

/* Read the PHY1 FSM monitor; the value latches into PHY_TST_CTRL1. */
static u32 k230_phy1_fsm_read(struct k230_dsi *dsi)
{
	k230_phy_test_write(dsi, PHY_REG_FSM_MONITOR, 0x80);
	return dsi_read(dsi, PHY_TST_CTRL1);
}

static void k230_phy1_config(struct k230_dsi *dsi)
{
	u32 reg, val;
	int ret;

	/* Select PHY1 (slave). */
	phy_write(dsi, TXDPHY_CFG1, TXDPHY_CFG1_PHY1_SEL);

	dsi_write(dsi, PHY_TST_CTRL0, PHY_TST_CTRL0_CLR);
	dsi_write(dsi, PHY_TST_CTRL0, 0);

	k230_phy_test_write(dsi, PHY_REG_MASTERMACRO, 0x0);

	phy_write(dsi, TXDPHY_CFG1, TXDPHY_CFG1_PHY1_SEL);

	k230_phy_test_write(dsi, PHY_REG_HSFREQRANGE, dsi->phy_hsfreq);
	k230_phy_test_write(dsi, PHY_REG_HS_CLK_LANE, 0xff);

	k230_phy_test_write(dsi, PHY_REG_SLEW_SR_RANGE, 0x40);
	k230_phy_test_write(dsi, PHY_REG_SLEW_OSC_FREQ, 0x11);
	k230_phy_test_write(dsi, PHY_REG_SLEW_OSC_FREQ, 0x85);
	k230_phy_test_write(dsi, PHY_REG_SLEW_CAL_EN, 0x1);
	k230_phy_test_write(dsi, PHY_REG_MPLL_PROG, 0x1);
	k230_phy_test_write(dsi, PHY_REG_PRG_ON_LANE0, 0x40);

	reg = phy_read(dsi, TXDPHY_CFG0);
	reg &= ~(TXDPHY_CFG0_PHY0_CFGCLKRANGE | TXDPHY_CFG0_PHY1_CFGCLKRANGE);
	reg |= FIELD_PREP(TXDPHY_CFG0_PHY0_CFGCLKRANGE,
			  TXDPHY_CFG0_CFGCLKRANGE_VAL);
	reg |= FIELD_PREP(TXDPHY_CFG0_PHY1_CFGCLKRANGE,
			  TXDPHY_CFG0_CFGCLKRANGE_VAL);
	phy_write(dsi, TXDPHY_CFG0, reg);

	phy_update(dsi, TXDPHY_CFG0, TXDPHY_CFG0_PHY1_BASEDIR, 0);

	dsi_write(dsi, PHY_RSTZ, PHY_RSTZ_ENABLECLK);
	dsi_write(dsi, PHY_RSTZ, PHY_RSTZ_ENFORCEPLL_CLK_SHDN);
	dsi_write(dsi, PHY_RSTZ, PHY_RSTZ_ENABLE_ALL);

	/* Poll the PHY1 FSM monitor until it reports a settled state. */
	ret = read_poll_timeout(k230_phy1_fsm_read, val, val == PHY1_FSM_READY,
				PHY_SLEEP_US, PHY_TIMEOUT_US, false, dsi);
	if (ret)
		dev_warn(dsi->dev, "PHY1 FSM timeout\n");
}

static int k230_dsi_phy_init(void *priv_data)
{
	struct k230_dsi *dsi = priv_data;

	k230_phy0_config(dsi);
	k230_phy1_config(dsi);

	/* Re-arm PHY0 and release reset once both lanes are programmed. */
	phy_read(dsi, TXDPHY_CFG1);
	phy_write(dsi, TXDPHY_CFG1, TXDPHY_CFG1_PHY0_SEL);
	dsi_write(dsi, PHY_RSTZ, PHY_RSTZ_ENFORCEPLL_CLK_SHDN);
	dsi_write(dsi, PHY_RSTZ, PHY_RSTZ_ENABLE_ALL);

	return 0;
}

static void k230_dsi_phy_power_on(void *priv_data)
{
	struct k230_dsi *dsi = priv_data;
	u32 val;
	int ret;

	ret = readl_poll_timeout(dsi->base + PHY_STATUS, val,
				 val == PHY_STATUS_READY, PHY_SLEEP_US,
				 PHY_TIMEOUT_US);
	if (ret)
		dev_warn(dsi->dev, "PHY status timeout\n");

	/*
	 * Postpone the HS request: per the MIPI spec the first STOP_STATE
	 * must last at least 100 us (longer than T_INIT) before HS clocking.
	 */
	dsi_write(dsi, LPCLK_CTRL, 0x1);

	/* Wait for PHY1 power-up. */
	phy_read(dsi, TXDPHY_CFG1);
	phy_write(dsi, TXDPHY_CFG1, TXDPHY_CFG1_PHY1_SEL);

	ret = readl_poll_timeout(dsi->base + PHY_STATUS, val,
				 val == PHY_STATUS_READY, PHY_SLEEP_US,
				 PHY_TIMEOUT_US);
	if (ret)
		dev_warn(dsi->dev, "PHY status timeout\n");
}

static void k230_dsi_phy_power_off(void *priv_data)
{
	struct k230_dsi *dsi = priv_data;

	clk_disable_unprepare(dsi->pixel);
}

static int k230_dsi_get_lane_mbps(void *priv_data,
				  const struct drm_display_mode *mode,
				  unsigned long mode_flags, u32 lanes,
				  u32 format, unsigned int *lane_mbps)
{
	struct k230_dsi *dsi = priv_data;
	u32 clk_freq, phy_clk_freq, voc, m = 0, n = 0;
	u64 cmp, tmp, diff, closest = 100000000;
	int bpp, ret;
	u32 i, val;

	bpp = mipi_dsi_pixel_format_to_bpp(format);
	if (bpp < 0)
		return bpp;

	ret = clk_set_rate(dsi->pixel, mode->clock * 1000);
	if (ret)
		return ret;

	clk_freq = clk_get_rate(dsi->pixel) / 1000;
	*lane_mbps = (u32)div_u64((u64)clk_freq * bpp, lanes);

	/* PHY clock = pixel * bpp / lanes / 2 (DDR), in kHz. */
	phy_clk_freq = (u32)div_u64((u64)clk_freq * bpp, lanes * 2);
	if (phy_clk_freq > 1250000 || phy_clk_freq < 40000)
		return -EINVAL;
	else if (phy_clk_freq < 55000)
		voc = 0x3f;
	else if (phy_clk_freq < 82500)
		voc = 0x37;
	else if (phy_clk_freq < 110000)
		voc = 0x2f;
	else if (phy_clk_freq < 165000)
		voc = 0x27;
	else if (phy_clk_freq < 220000)
		voc = 0x1f;
	else if (phy_clk_freq < 330000)
		voc = 0x17;
	else if (phy_clk_freq < 440000)
		voc = 0x0f;
	else if (phy_clk_freq < 660000)
		voc = 0x09; /* databook says 0x07; K230 needs 0x09 here */
	else if (phy_clk_freq < 1149000)
		voc = 0x03;
	else
		voc = 0x01;

	/* Search PLL feedback (m) and input (n) dividers against fref/24 MHz. */
	cmp = div64_u64((u64)phy_clk_freq * (1 << (voc >> 4)) * 1000, 24);
	for (i = 1; i <= 16; i++) {
		tmp = cmp * i + 500000;
		val = div64_u64(tmp, 1000000);
		if (val > 625)
			continue;
		tmp = div64_u64((u64)val * 1000000, i);
		diff = abs_diff(cmp, tmp);
		if (closest > diff) {
			closest = diff;
			n = i;
			m = val;
			if (diff == 0)
				break;
		}
	}
	if (!n)
		return -EINVAL;

	/* Stash PLL params for k230_dsi_phy_init() (register encoding). */
	dsi->phy_m = m - 2;
	dsi->phy_n = n - 1;
	dsi->phy_vco = voc;
	dsi->phy_hsfreq = K230_HSFREQRANGE;

	ret = clk_prepare_enable(dsi->pixel);
	if (ret)
		return ret;

	return 0;
}

#define DSI_PHY_DELAY(fp, vp, mbps) DIV_ROUND_UP((fp) * (mbps) + 1000 * (vp), 8000)

static int k230_dsi_get_timing(void *priv_data, unsigned int lane_mbps,
			       struct dw_mipi_dsi_dphy_timing *timing)
{
	/*
	 * Analytic D-PHY transition timings (cycles), borrowed from the
	 * STM DW MIPI DSI glue; the vendor driver had none.
	 */
	timing->clk_hs2lp = DSI_PHY_DELAY(272, 136, lane_mbps);
	timing->clk_lp2hs = DSI_PHY_DELAY(512, 40, lane_mbps);
	timing->data_hs2lp = DSI_PHY_DELAY(192, 64, lane_mbps);
	timing->data_lp2hs = DSI_PHY_DELAY(256, 32, lane_mbps);

	return 0;
}

static enum drm_mode_status
k230_dsi_mode_valid(void *priv_data, const struct drm_display_mode *mode,
		    unsigned long mode_flags, u32 lanes, u32 format)
{
	u32 phy_clk_freq;
	int bpp;

	bpp = mipi_dsi_pixel_format_to_bpp(format);
	if (bpp < 0)
		return MODE_ERROR;

	phy_clk_freq = (u32)div_u64((u64)mode->clock * bpp, lanes * 2);
	if (phy_clk_freq > 1250000)
		return MODE_CLOCK_HIGH;
	if (phy_clk_freq < 40000)
		return MODE_CLOCK_LOW;

	return MODE_OK;
}

static const struct dw_mipi_dsi_phy_ops k230_dsi_phy_ops = {
	.init = k230_dsi_phy_init,
	.power_on = k230_dsi_phy_power_on,
	.power_off = k230_dsi_phy_power_off,
	.get_lane_mbps = k230_dsi_get_lane_mbps,
	.get_timing = k230_dsi_get_timing,
};

static int k230_dsi_bind(struct device *dev, struct device *master, void *data)
{
	struct k230_dsi *dsi = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	int ret;

	ret = drm_simple_encoder_init(drm, &dsi->encoder,
				      DRM_MODE_ENCODER_DSI);
	if (ret) {
		dev_err(dev, "Failed to init DSI encoder: %d\n", ret);
		return ret;
	}
	dsi->encoder.possible_crtcs = BIT(0);

	ret = dw_mipi_dsi_bind(dsi->dmd, &dsi->encoder);
	if (ret) {
		dev_err(dev, "Failed to bind dw-mipi-dsi: %d\n", ret);
		drm_encoder_cleanup(&dsi->encoder);
		return ret;
	}

	return 0;
}

static void k230_dsi_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct k230_dsi *dsi = dev_get_drvdata(dev);

	dw_mipi_dsi_unbind(dsi->dmd);
	drm_encoder_cleanup(&dsi->encoder);
}

static const struct component_ops k230_dsi_ops = {
	.bind = k230_dsi_bind,
	.unbind = k230_dsi_unbind,
};

static int k230_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct k230_dsi *dsi;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->dev = dev;
	dev_set_drvdata(dev, dsi);

	dsi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsi->base))
		return PTR_ERR(dsi->base);

	dsi->pixel = devm_clk_get(dev, "pixel");
	if (IS_ERR(dsi->pixel))
		return dev_err_probe(dev, PTR_ERR(dsi->pixel),
				     "Couldn't get the pixel clock\n");

	dsi->pdata.base = dsi->base;
	dsi->pdata.max_data_lanes = 4;
	dsi->pdata.phy_ops = &k230_dsi_phy_ops;
	dsi->pdata.mode_valid = k230_dsi_mode_valid;
	dsi->pdata.priv_data = dsi;

	dsi->dmd = dw_mipi_dsi_probe(pdev, &dsi->pdata);
	if (IS_ERR(dsi->dmd))
		return dev_err_probe(dev, PTR_ERR(dsi->dmd),
				     "Failed to probe dw-mipi-dsi\n");

	ret = component_add(dev, &k230_dsi_ops);
	if (ret) {
		dw_mipi_dsi_remove(dsi->dmd);
		return ret;
	}

	return 0;
}

static void k230_dsi_remove(struct platform_device *pdev)
{
	struct k230_dsi *dsi = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &k230_dsi_ops);
	dw_mipi_dsi_remove(dsi->dmd);
}

static const struct of_device_id k230_dsi_of_table[] = {
	{ .compatible = "canaan,k230-mipi-dsi" },
	{ }
};
MODULE_DEVICE_TABLE(of, k230_dsi_of_table);

struct platform_driver canaan_dsi_driver = {
	.probe	= k230_dsi_probe,
	.remove	= k230_dsi_remove,
	.driver	= {
		.name		= "canaan-mipi-dsi",
		.of_match_table	= k230_dsi_of_table,
	},
};

MODULE_AUTHOR("Sungjoon Moon <sumoon@seoulsaram.org>");
MODULE_DESCRIPTION("Canaan K230 DW MIPI DSI glue driver");
MODULE_LICENSE("GPL");
