// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022, Canaan Bright Sight Co., Ltd
 *
 * All enquiries to https://www.canaan-creative.com/
 *
 */

#include <linux/device.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "canaan_vo.h"
#include "canaan_crtc.h"
#include "canaan_plane.h"

static int canaan_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct canaan_crtc *canaan_crtc = to_canaan_crtc(crtc);
	struct canaan_vo *vo = canaan_crtc->vo;

	drm_dbg_kms(crtc->dev, "Enable vblank on CRTC:%d\n", crtc->base.id);
	return canaan_vo_enable_vblank(vo);
}

static void canaan_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct canaan_crtc *canaan_crtc = to_canaan_crtc(crtc);
	struct canaan_vo *vo = canaan_crtc->vo;

	drm_dbg_kms(crtc->dev, "Disable vblank on CRTC:%d\n", crtc->base.id);
	canaan_vo_disable_vblank(vo);
}

static void canaan_crtc_atomic_enable(struct drm_crtc *crtc,
				      struct drm_atomic_commit *old_crtc_state)
{
	struct canaan_crtc *canaan_crtc = to_canaan_crtc(crtc);
	struct canaan_vo *vo = canaan_crtc->vo;
	struct drm_display_mode *adjusted_mode = &crtc->state->adjusted_mode;

	drm_dbg_kms(crtc->dev, "Enable the CRTC:%d\n", crtc->base.id);

	canaan_vo_enable_crtc(vo, canaan_crtc, adjusted_mode);
	drm_crtc_vblank_on(crtc);
}

static void canaan_crtc_atomic_disable(struct drm_crtc *crtc,
				       struct drm_atomic_commit *old_crtc_state)
{
	struct canaan_crtc *canaan_crtc = to_canaan_crtc(crtc);
	struct canaan_vo *vo = canaan_crtc->vo;

	drm_dbg_kms(crtc->dev, "Disable the CRTC:%d\n", crtc->base.id);

	/* Send any pending event before disabling vblank */
	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);
		crtc->state->event = NULL;
	}

	drm_crtc_vblank_off(crtc);
	canaan_vo_disable_crtc(vo, canaan_crtc);
}

static void canaan_crtc_atomic_flush(struct drm_crtc *crtc,
				     struct drm_atomic_commit *old_crtc_state)
{
	struct canaan_crtc *canaan_crtc = to_canaan_crtc(crtc);
	struct canaan_vo *vo = canaan_crtc->vo;
	struct drm_pending_vblank_event *event = crtc->state->event;

	drm_dbg_kms(crtc->dev, "Flush the configuration\n");
	canaan_vo_flush_config(vo);

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_crtc_funcs canaan_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = canaan_crtc_enable_vblank,
	.disable_vblank = canaan_crtc_disable_vblank,
};

static const struct drm_crtc_helper_funcs canaan_crtc_helper_funcs = {
	.atomic_enable = canaan_crtc_atomic_enable,
	.atomic_disable = canaan_crtc_atomic_disable,
	.atomic_flush = canaan_crtc_atomic_flush,
};

struct canaan_crtc *canaan_crtc_create(struct drm_device *drm_dev,
				       struct drm_plane *primary,
				       struct drm_plane *cursor,
				       struct canaan_vo *vo)
{
	int ret = 0;
	struct canaan_crtc *canaan_crtc = NULL;
	struct drm_crtc *crtc = NULL;
	struct device *dev = vo->dev;

	canaan_crtc = devm_kzalloc(dev, sizeof(struct canaan_crtc), GFP_KERNEL);
	if (!canaan_crtc)
		return ERR_PTR(-ENOMEM);
	canaan_crtc->vo = vo;
	crtc = &canaan_crtc->base;

	ret = drm_crtc_init_with_planes(drm_dev, crtc, primary, cursor,
					&canaan_crtc_funcs, "canaan_crtc");
	if (ret) {
		dev_err(dev, "Failed to init CRTC\n");
		return ERR_PTR(ret);
	}

	drm_crtc_helper_add(crtc, &canaan_crtc_helper_funcs);
	drm_dbg_kms(drm_dev, "Create the CRTC:%d\n", crtc->base.id);

	return canaan_crtc;
}
