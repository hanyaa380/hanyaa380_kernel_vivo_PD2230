/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
*/

#ifndef MTK_DRM_FBDEV_H
#define MTK_DRM_FBDEV_H

struct tag_videolfb {
	u64 fb_base;
	u32 islcmfound;
	u32 fps;
	u32 vram;
	u32 lcm_software_id;
	u32 lcm_especial_id;
	u32 silent_reboot;  /* add for silent reboot */
	u32 bl_hw_version;
	u32 lcm_imei_str[4] ;
	char lcmname[1]; /* this is the minimum size */
};

#ifdef CONFIG_DRM_FBDEV_EMULATION
int mtk_fbdev_init(struct drm_device *dev);
void mtk_fbdev_fini(struct drm_device *dev);
#else
int mtk_fbdev_init(struct drm_device *dev)
{
	return 0;
}

void mtk_fbdev_fini(struct drm_device *dev)
{
}
#endif /* CONFIG_DRM_FBDEV_EMULATION */

int _parse_tag_videolfb(unsigned int *vramsize, phys_addr_t *fb_base,
			unsigned int *fps);
bool mtk_drm_lcm_is_connect(void);
int try_free_fb_buf(struct drm_device *dev);
#define MTKFB_FACTORY_AUTO_TEST _IOR('O', 25, unsigned long)
int pan_display_test(int frame_num, int bpp);

#endif /* MTK_DRM_FBDEV_H */
