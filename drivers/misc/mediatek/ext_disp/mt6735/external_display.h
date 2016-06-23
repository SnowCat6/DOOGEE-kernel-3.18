#ifndef _EXTD_DDP_H_
#define _EXTD_DDP_H_

#include "ddp_hal.h"
#include "ddp_manager.h"

#define ALIGN_TO(x, n)  \
    (((x) + ((n) - 1)) & ~((n) - 1))


typedef enum
{
	EXTD_DIRECT_LINK_MODE,
	EXTD_DECOUPLE_MODE,
	EXTD_SINGLE_LAYER_MODE,
	EXTD_RDMA_DPI_MODE
}EXT_DISP_PATH_MODE;

typedef enum
{	
   EXT_DISP_STATUS_OK = 0,

   EXT_DISP_STATUS_NOT_IMPLEMENTED,
   EXT_DISP_STATUS_ALREADY_SET,
   EXT_DISP_STATUS_ERROR
} EXT_DISP_STATUS;

typedef enum
{
	EXTD_DEINIT = 0,
	EXTD_INIT,
	EXTD_RESUME,
	EXTD_SUSPEND
}EXTD_POWER_STATE;

typedef struct
{
	unsigned int layer;
	unsigned int layer_en;
	unsigned int buff_source;
	unsigned int fmt;
	unsigned long addr;  
	unsigned long addr_sub_u;
	unsigned long addr_sub_v;
	unsigned long vaddr;
	unsigned int src_x;
	unsigned int src_y;
	unsigned int src_w;
	unsigned int src_h;
	unsigned int src_pitch;
	unsigned int dst_x;
	unsigned int dst_y;
	unsigned int dst_w;
	unsigned int dst_h;                  // clip region
	unsigned int keyEn;
	unsigned int key; 
	unsigned int aen; 
	unsigned char alpha;  
	
    unsigned int  sur_aen;
	unsigned int  src_alpha;
	unsigned int  dst_alpha;

	unsigned int isTdshp;
	unsigned int isDirty;

	unsigned int buff_idx;
	unsigned int identity;
	unsigned int connected_type;
	unsigned int security;
	unsigned int dirty;
}ext_disp_input_config;


int ext_disp_init(char *lcm_name,  unsigned int session);
int ext_disp_deinit(char *lcm_name);
int ext_disp_suspend(void);
int ext_disp_resume(void);
EXT_DISP_PATH_MODE ext_disp_path_get_mode(void);
void ext_disp_path_set_mode(EXT_DISP_PATH_MODE mode);

unsigned int ext_disp_get_sess_id(void);

int ext_disp_get_width(void);
int ext_disp_get_height(void);
int ext_disp_get_bpp(void);
int ext_disp_get_pages(void);

int ext_disp_is_alive(void);
int ext_disp_is_sleepd(void);
int ext_disp_wait_for_vsync(void *config);
int ext_disp_config_input(ext_disp_input_config* input);
int ext_disp_config_input_multiple(ext_disp_input_config* input, int idx);
int ext_disp_trigger(int blocking, void *callback, unsigned int userdata);

int ext_disp_get_info(void *info);
int ext_disp_is_video_mode(void);
CMDQ_SWITCH ext_disp_cmdq_enabled(void);
int ext_disp_switch_cmdq_cpu(CMDQ_SWITCH use_cmdq);
int ext_disp_diagnose(void);
void extd_get_curr_addr(unsigned long *input_curr_addr, int module);
int ext_disp_factory_test(int mode, void *config);
int extd_get_handle(disp_path_handle *dp_handle, cmdqRecHandle *pHandle);

#endif
