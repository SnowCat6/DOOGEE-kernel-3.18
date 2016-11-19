#ifndef __H_MTK_DISP_MGR__
#define __H_MTK_DISP_MGR__

extern unsigned int is_hwc_enabled;
char *disp_session_mode_spy(unsigned int session_id);
long mtk_disp_mgr_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

#endif
