/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022, Canaan Bright Sight Co., Ltd
 *
 * All enquiries to https://www.canaan-creative.com/
 *
 */

#ifndef __CANAAN_PLANE_H__
#define __CANAAN_PLANE_H__

struct canaan_plane_config {
	const char *name;
	u32 id;
	u32 possible_crtcs;
	u32 num_formats;
	const u32 *formats;
	enum drm_plane_type plane_type;

	u32 plane_offset;
	u32 plane_enable_bit;
	u32 xctl_reg_offset;
	u32 yctl_reg_offset;
};

struct canaan_plane {
	struct drm_plane base;
	struct canaan_vo *vo;
	struct canaan_plane_config *config;
	u32 id;
};

static inline struct canaan_plane *to_canaan_plane(struct drm_plane *plane)
{
	return container_of(plane, struct canaan_plane, base);
}

struct canaan_plane *canaan_plane_create(struct drm_device *drm_dev,
					 struct canaan_plane_config *config,
					 struct canaan_vo *vo);
#endif /* __CANAAN_PLANE_H__ */
