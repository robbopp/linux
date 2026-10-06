// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AXI HDMI TX display driver
 *
 * Copyright 2012 Analog Devices Inc.
 * Copyright 2026 TALADIN S.R.L.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *  Co-developed-by: Robert Oprean <roprean@taladin.ro>
 */

#include <linux/adi-axi-common.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_mode_config.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#define AXI_HDMI_REG_RESET		0x040
#define   AXI_HDMI_RESET_ENABLE		BIT(0)
#define AXI_HDMI_REG_CNTRL1		0x044
#define AXI_HDMI_REG_CNTRL2		0x048
#define   AXI_HDMI_SOURCE_SEL_INPUT	1
#define AXI_HDMI_REG_CLIPP_MAX		0x068
#define AXI_HDMI_REG_CLIPP_MIN		0x06c
#define AXI_HDMI_REG_HTIMING1		0x400
#define AXI_HDMI_REG_HTIMING2		0x404
#define AXI_HDMI_REG_HTIMING3		0x408
#define AXI_HDMI_REG_VTIMING1		0x440
#define AXI_HDMI_REG_VTIMING2		0x444
#define AXI_HDMI_REG_VTIMING3		0x448

/*
 * The core converts RGB to YCbCr 4:2:2 for the ADV7511 and clamps the result
 * to BT.601 limited range. Byte order is Cr|Y|Cb: Y 16..235, Cb/Cr 16..240.
 */
#define AXI_HDMI_CLIPP_MIN_LIMITED	0x101010
#define AXI_HDMI_CLIPP_MAX_LIMITED	0xf0ebf0

#define AXI_HDMI_CORE_VERSION_MAJOR	4
/* The AXI-DMAC reads over a 64-bit bus: lengths and addresses are 8-byte aligned. */
#define AXI_HDMI_DMA_ALIGN		8

struct adi_axi_hdmi {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;

	void __iomem *base;
	struct clk *clk;
	bool clk_enabled;
	struct dma_chan *dma;
};

static struct adi_axi_hdmi *to_adi_axi_hdmi(struct drm_device *drm)
{
	return container_of(drm, struct adi_axi_hdmi, drm);
}

static const u32 adi_axi_hdmi_formats[] = {
	DRM_FORMAT_XRGB8888,
};

/* Scanout address of the clipped source rectangle, not the unclipped src_x/src_y. */
static dma_addr_t adi_axi_hdmi_scanout_addr(const struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;

	return drm_fb_dma_get_gem_obj(fb, 0)->dma_addr + fb->offsets[0] +
	       (state->src.y1 >> 16) * fb->pitches[0] +
	       (state->src.x1 >> 16) * fb->format->cpp[0];
}

static int adi_axi_hdmi_plane_atomic_check(struct drm_plane *plane,
					   struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_crtc_state *crtc_state;
	int ret;

	if (!new_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);
	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);
	if (ret)
		return ret;
	/*
	 * An enabled CRTC always scans out: an off-screen primary would leave
	 * the DMA on a stale fb.
	 */
	if (!new_state->visible)
		return crtc_state->enable ? -EINVAL : 0;

	/* The DMA would refuse these at commit time, when we can no longer fail. */
	if (!IS_ALIGNED(fb->pitches[0], AXI_HDMI_DMA_ALIGN) ||
	    !IS_ALIGNED(drm_rect_width(&new_state->dst) * fb->format->cpp[0],
			AXI_HDMI_DMA_ALIGN) ||
	    !IS_ALIGNED(adi_axi_hdmi_scanout_addr(new_state), AXI_HDMI_DMA_ALIGN))
		return -EINVAL;

	return 0;
}

static void adi_axi_hdmi_plane_atomic_update(struct drm_plane *plane,
					     struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct adi_axi_hdmi *hdmi = to_adi_axi_hdmi(plane->dev);
	struct drm_framebuffer *fb = new_state->fb;
	DEFINE_RAW_FLEX(struct dma_interleaved_template, xt, sgl, 1);
	struct dma_async_tx_descriptor *desc;
	size_t row;

	/* Never DMA into a core whose pixel clock failed to enable. */
	if (!new_state->visible || !hdmi->clk_enabled)
		return;

	row = drm_rect_width(&new_state->dst) * fb->format->cpp[0];

	/* One chunk per line, one line per frame of the template. */
	xt->dir = DMA_MEM_TO_DEV;
	xt->src_start = adi_axi_hdmi_scanout_addr(new_state);
	xt->src_inc = true;
	xt->src_sgl = true;
	xt->numf = drm_rect_height(&new_state->dst);
	xt->frame_size = 1;
	xt->sgl[0].size = row;
	xt->sgl[0].icg = fb->pitches[0] - row;

	/*
	 * ponytail: terminate + resubmit can glitch one frame on a flip; same as
	 * ADI's driver. axi-dmac already honours DMA_PREP_LOAD_EOT, but a
	 * tear-free flip also needs the old fb kept alive until the DMA has moved
	 * off it (e.g. wait for the descriptor's completion before cleanup_planes
	 * frees it); LOAD_EOT alone makes the DMA read freed memory.
	 */
	/* axi-dmac does not wait for in-flight reads; the old fb is freed ms later. */
	dmaengine_terminate_sync(hdmi->dma);

	/* Prep after terminate: terminate_all frees descriptors already prepped. */
	/* The AXI-DMAC driver takes DMA_CYCLIC as a flag for interleaved transfers. */
	desc = dmaengine_prep_interleaved_dma(hdmi->dma, xt, DMA_CYCLIC);
	if (!desc) {
		drm_err(plane->dev, "failed to prepare scanout DMA\n");
		return;
	}

	dmaengine_submit(desc);
	dma_async_issue_pending(hdmi->dma);
}

static const struct drm_plane_helper_funcs adi_axi_hdmi_plane_helper_funcs = {
	.atomic_check = adi_axi_hdmi_plane_atomic_check,
	.atomic_update = adi_axi_hdmi_plane_atomic_update,
};

static const struct drm_plane_funcs adi_axi_hdmi_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static enum drm_mode_status
adi_axi_hdmi_crtc_mode_valid(struct drm_crtc *crtc,
			     const struct drm_display_mode *mode)
{
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;
	/* XRGB8888 rows must be a multiple of the 8-byte DMA alignment. */
	if (mode->hdisplay % 2)
		return MODE_H_ILLEGAL;
	/* The pixel clock is set in Hz: mode->clock * 1000 must fit. */
	if (mode->clock > ULONG_MAX / 1000)
		return MODE_CLOCK_HIGH;

	/*
	 * No clk_round_rate() check: the clkgen is CLK_SET_RATE_GATE, so while
	 * scanning out it is rate-protected and rounds every rate to the
	 * current one, which would prune all but the active mode.
	 */
	return MODE_OK;
}

static int adi_axi_hdmi_crtc_atomic_check(struct drm_crtc *crtc,
					  struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (!crtc_state->enable)
		return 0;
	/* Never leave the DMA scanning out a plane that is gone. */
	return drm_atomic_helper_check_crtc_primary_plane(crtc_state);
}

static void adi_axi_hdmi_crtc_atomic_enable(struct drm_crtc *crtc,
					    struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	const struct drm_display_mode *m = &crtc_state->adjusted_mode;
	struct adi_axi_hdmi *hdmi = to_adi_axi_hdmi(crtc->dev);
	void __iomem *base = hdmi->base;
	/* Data enable starts after sync + back porch. */
	u32 h_de_min = m->htotal - m->hsync_start;
	u32 v_de_min = m->vtotal - m->vsync_start;
	unsigned long rate = (unsigned long)m->clock * 1000;
	unsigned long actual;
	int ret;

	ret = clk_set_rate(hdmi->clk, rate);
	if (ret)
		drm_warn(crtc->dev, "pixel clock %d kHz: %d\n", m->clock, ret);
	actual = clk_get_rate(hdmi->clk);
	if (abs_diff(actual, rate) > rate / 200)
		drm_warn(crtc->dev, "pixel clock %lu kHz requested, got %lu kHz\n",
			 rate / 1000, actual / 1000);

	ret = clk_prepare_enable(hdmi->clk);
	if (ret) {
		drm_err(crtc->dev, "failed to enable pixel clock: %d\n", ret);
		return;
	}
	hdmi->clk_enabled = true;

	writel((m->hdisplay << 16) | m->htotal, base + AXI_HDMI_REG_HTIMING1);
	writel(m->hsync_end - m->hsync_start, base + AXI_HDMI_REG_HTIMING2);
	writel(((h_de_min + m->hdisplay) << 16) | h_de_min,
	       base + AXI_HDMI_REG_HTIMING3);

	writel((m->vdisplay << 16) | m->vtotal, base + AXI_HDMI_REG_VTIMING1);
	writel(m->vsync_end - m->vsync_start, base + AXI_HDMI_REG_VTIMING2);
	writel(((v_de_min + m->vdisplay) << 16) | v_de_min,
	       base + AXI_HDMI_REG_VTIMING3);

	/* CSC and 4:2:2 subsampler on: the ADV7511 input is YCbCr 4:2:2. */
	writel(0, base + AXI_HDMI_REG_CNTRL1);
	writel(AXI_HDMI_CLIPP_MIN_LIMITED, base + AXI_HDMI_REG_CLIPP_MIN);
	writel(AXI_HDMI_CLIPP_MAX_LIMITED, base + AXI_HDMI_REG_CLIPP_MAX);
	writel(AXI_HDMI_SOURCE_SEL_INPUT, base + AXI_HDMI_REG_CNTRL2);

	writel(AXI_HDMI_RESET_ENABLE, base + AXI_HDMI_REG_RESET);
}

/* No drm_dev_enter() here: .remove unplugs before it shuts the pipeline down. */
static void adi_axi_hdmi_crtc_atomic_disable(struct drm_crtc *crtc,
					     struct drm_atomic_commit *state)
{
	struct adi_axi_hdmi *hdmi = to_adi_axi_hdmi(crtc->dev);

	/* Plane hooks don't run for an inactive CRTC (commit_tail_rpm). */
	/* axi-dmac does not wait for in-flight reads; the old fb is freed ms later. */
	dmaengine_terminate_sync(hdmi->dma);
	writel(0, hdmi->base + AXI_HDMI_REG_RESET);

	if (hdmi->clk_enabled) {
		clk_disable_unprepare(hdmi->clk);
		hdmi->clk_enabled = false;
	}
}

static const struct drm_crtc_helper_funcs adi_axi_hdmi_crtc_helper_funcs = {
	.mode_valid = adi_axi_hdmi_crtc_mode_valid,
	.atomic_check = adi_axi_hdmi_crtc_atomic_check,
	.atomic_enable = adi_axi_hdmi_crtc_atomic_enable,
	.atomic_disable = adi_axi_hdmi_crtc_atomic_disable,
};

static const struct drm_crtc_funcs adi_axi_hdmi_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_mode_config_funcs adi_axi_hdmi_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/*
 * The default commit tail updates planes before enabling the CRTC, which
 * would start the DMA into a core that is still in reset with its pixel
 * clock off. Enable first, then planes (active CRTCs only).
 */
static const struct drm_mode_config_helper_funcs adi_axi_hdmi_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

DEFINE_DRM_GEM_DMA_FOPS(adi_axi_hdmi_fops);

static const struct drm_driver adi_axi_hdmi_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &adi_axi_hdmi_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name = "adi-axi-hdmi-tx",
	.desc = "Analog Devices AXI HDMI TX",
	.major = 1,
	.minor = 0,
};

static void adi_axi_hdmi_release_dma(void *chan)
{
	dma_release_channel(chan);
}

static void adi_axi_hdmi_put_bridge(void *bridge)
{
	drm_bridge_put(bridge);
}

static int adi_axi_hdmi_kms_init(struct adi_axi_hdmi *hdmi,
				 struct drm_bridge *bridge)
{
	struct drm_device *drm = &hdmi->drm;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.max_width = 4096;
	drm->mode_config.max_height = 4096;
	drm->mode_config.funcs = &adi_axi_hdmi_mode_config_funcs;
	drm->mode_config.helper_private = &adi_axi_hdmi_mode_config_helpers;

	ret = drm_universal_plane_init(drm, &hdmi->plane, BIT(0),
				       &adi_axi_hdmi_plane_funcs,
				       adi_axi_hdmi_formats,
				       ARRAY_SIZE(adi_axi_hdmi_formats),
				       NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&hdmi->plane, &adi_axi_hdmi_plane_helper_funcs);

	ret = drmm_crtc_init_with_planes(drm, &hdmi->crtc, &hdmi->plane, NULL,
					 &adi_axi_hdmi_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&hdmi->crtc, &adi_axi_hdmi_crtc_helper_funcs);

	encoder = drmm_plain_encoder_alloc(drm, NULL, DRM_MODE_ENCODER_TMDS, NULL);
	if (IS_ERR(encoder))
		return PTR_ERR(encoder);
	encoder->possible_crtcs = drm_crtc_mask(&hdmi->crtc);

	ret = drm_bridge_attach(encoder, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;

	connector = drm_bridge_connector_init(drm, encoder);
	if (IS_ERR(connector))
		return PTR_ERR(connector);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	/* No HPD interrupt is wired from the ADV7511, so poll for the monitor. */
	drmm_kms_helper_poll_init(drm);

	return 0;
}

static int adi_axi_hdmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_axi_hdmi *hdmi;
	struct drm_bridge *bridge;
	u32 version;
	int ret;

	hdmi = devm_drm_dev_alloc(dev, &adi_axi_hdmi_driver,
				  struct adi_axi_hdmi, drm);
	if (IS_ERR(hdmi))
		return PTR_ERR(hdmi);
	platform_set_drvdata(pdev, &hdmi->drm);

	hdmi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hdmi->base))
		return PTR_ERR(hdmi->base);

	version = readl(hdmi->base + ADI_AXI_REG_VERSION);
	if (ADI_AXI_PCORE_VER_MAJOR(version) != AXI_HDMI_CORE_VERSION_MAJOR)
		return dev_err_probe(dev, -ENODEV,
				     "unsupported core version %u.%u.%c\n",
				     ADI_AXI_PCORE_VER_MAJOR(version),
				     ADI_AXI_PCORE_VER_MINOR(version),
				     ADI_AXI_PCORE_VER_PATCH(version));

	hdmi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(hdmi->clk))
		return dev_err_probe(dev, PTR_ERR(hdmi->clk),
				     "failed to get pixel clock\n");

	hdmi->dma = dma_request_chan(dev, "video");
	if (IS_ERR(hdmi->dma))
		return dev_err_probe(dev, PTR_ERR(hdmi->dma),
				     "failed to get video DMA channel\n");
	ret = devm_add_action_or_reset(dev, adi_axi_hdmi_release_dma, hdmi->dma);
	if (ret)
		return ret;

	if (!dma_has_cap(DMA_INTERLEAVE, hdmi->dma->device->cap_mask))
		return dev_err_probe(dev, -ENODEV,
				     "video DMA lacks interleaved transfers\n");

	/* GEM buffers must be allocated for the device that actually does the DMA. */
	drm_dev_set_dma_dev(&hdmi->drm, dmaengine_get_dma_device(hdmi->dma));

	bridge = of_drm_get_bridge_by_endpoint(dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge),
				     "failed to find HDMI bridge\n");
	ret = devm_add_action_or_reset(dev, adi_axi_hdmi_put_bridge, bridge);
	if (ret)
		return ret;

	ret = adi_axi_hdmi_kms_init(hdmi, bridge);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialise KMS\n");

	ret = drm_dev_register(&hdmi->drm, 0);
	if (ret)
		return ret;

	drm_client_setup(&hdmi->drm, NULL);

	return 0;
}

static void adi_axi_hdmi_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	/* Unplug first: the disable path must therefore not use drm_dev_enter(). */
	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void adi_axi_hdmi_shutdown(struct platform_device *pdev)
{
	drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static const struct of_device_id adi_axi_hdmi_of_match[] = {
	{ .compatible = "adi,axi-hdmi-tx-1.00.a" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_axi_hdmi_of_match);

static struct platform_driver adi_axi_hdmi_platform_driver = {
	.driver = {
		.name = "adi-axi-hdmi-tx",
		.of_match_table = adi_axi_hdmi_of_match,
	},
	.probe = adi_axi_hdmi_probe,
	.remove = adi_axi_hdmi_remove,
	.shutdown = adi_axi_hdmi_shutdown,
};
module_platform_driver(adi_axi_hdmi_platform_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_AUTHOR("Robert Oprean <roprean@taladin.ro>");
MODULE_DESCRIPTION("Analog Devices AXI HDMI TX display driver");
MODULE_LICENSE("GPL");
