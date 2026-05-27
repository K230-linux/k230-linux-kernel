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
#include <drm/drm_plane.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_print.h>

#include "canaan_vo.h"
#include "canaan_crtc.h"
#include "canaan_plane.h"

static int canaan_plane_atomic_check(struct drm_plane *plane,
				     struct drm_atomic_state *state)
{
	struct drm_plane_state *plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct canaan_plane *canaan_plane = to_canaan_plane(plane);
	struct canaan_vo *vo = canaan_plane->vo;

	if (!plane_state->crtc || !plane_state->fb) {
		drm_dbg_kms(plane->dev, "crtc or fb NULL\n");
		return 0;
	}

	drm_dbg_kms(plane->dev, "Check plane:%d\n", plane->base.id);
	drm_dbg_kms(plane->dev, "(%d,%d)@(%d,%d) -> (%d,%d)@(%d,%d)\n",
		    plane_state->src_w >> 16, plane_state->src_h >> 16,
		    plane_state->src_x >> 16, plane_state->src_y >> 16,
		    plane_state->crtc_w, plane_state->crtc_h,
		    plane_state->crtc_x, plane_state->crtc_y);

	return canaan_vo_check_plane(vo, canaan_plane, plane_state);
}

static void canaan_plane_atomic_update(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct drm_plane_state *plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct canaan_plane *canaan_plane = to_canaan_plane(plane);
	struct canaan_vo *vo = canaan_plane->vo;

	if (!plane_state->crtc || !plane_state->fb) {
		drm_dbg_kms(plane->dev, "crtc or fb NULL\n");
		return;
	}

	drm_dbg_kms(plane->dev, "Update plane:%d\n", plane->base.id);
	canaan_vo_update_plane(vo, canaan_plane, plane_state);
}

static void canaan_plane_atomic_disable(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct canaan_plane *canaan_plane = to_canaan_plane(plane);
	struct canaan_vo *vo = canaan_plane->vo;

	drm_dbg_kms(plane->dev, "Disable plane:%d\n", plane->base.id);
	canaan_vo_disable_plane(vo, canaan_plane);
}

static const struct drm_plane_funcs canaan_plane_funcs = {
	.reset = drm_atomic_helper_plane_reset,
	.destroy = drm_plane_cleanup,
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static const struct drm_plane_helper_funcs canaan_plane_helper_funcs = {
	.atomic_check = canaan_plane_atomic_check,
	.atomic_update = canaan_plane_atomic_update,
	.atomic_disable = canaan_plane_atomic_disable,
};

struct canaan_plane *canaan_plane_create(struct drm_device *drm_dev,
					 struct canaan_plane_config *config,
					 struct canaan_vo *vo)
{
	int ret = 0;
	struct canaan_plane *canaan_plane = NULL;
	struct drm_plane *plane = NULL;
	struct device *dev = vo->dev;

	canaan_plane = devm_kzalloc(dev, sizeof(*canaan_plane), GFP_KERNEL);
	if (!canaan_plane)
		return ERR_PTR(-ENOMEM);
	plane = &canaan_plane->base;

	ret = drm_universal_plane_init(drm_dev, plane, config->possible_crtcs,
				       &canaan_plane_funcs, config->formats,
				       config->num_formats, NULL,
				       config->plane_type, config->name);
	if (ret) {
		dev_err(dev, "Failed to init Plane\n");
		return ERR_PTR(ret);
	}

	drm_plane_helper_add(plane, &canaan_plane_helper_funcs);
	canaan_plane->id = config->id;
	canaan_plane->config = config;
	canaan_plane->vo = vo;
	drm_dbg_kms(drm_dev, "Create plane:%d\n", plane->base.id);

	return canaan_plane;
}
