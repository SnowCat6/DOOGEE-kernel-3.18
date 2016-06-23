#ifndef __DISP_DRV_LOG_H__
#define __DISP_DRV_LOG_H__
#include "display_recorder.h"
        
extern unsigned int dprec_error_log_len;
extern unsigned int dprec_error_log_id;
extern unsigned int dprec_error_log_buflen;
		
#define DISPMSG(string, args...) printk("[DISP]"string, ##args)  // default on, important msg, not err
#define DISPDBG(string, args...) pr_debug("disp/"string, ##args);
#define DISPERR  	DISPPR_ERROR

#define DISP_FATAL_ERR(tag, string, args...)                                              \
    do {                                                                            \
        DISPERR(string,##args);                                \
        aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT | DB_OPT_MMPROFILE_BUFFER, tag, "error: "string, ##args);                           \
    } while(0)

#if 0
	do {\
		printk("[DISP][%s #%d]ERROR:"string,__func__, __LINE__, ##args); \
		if(0)dprec_error_log_len += scnprintf(dprec_error_log_buffer+dprec_error_log_len, dprec_error_log_buflen-dprec_error_log_len, "[%d][%s #%d]"string,dprec_error_log_id++, __func__, __LINE__, ##args) ; \
	}while(0)
#endif

#define DISPFUNC() printk("[DISP]func|%s\n", __func__)  //default on, err msg
#define DISPDBGFUNC() DISPDBG("[DISP]func|%s\n", __func__)  //default on, err msg

#define DISPCHECK(string, args...) printk("[DISPCHECK]"string, ##args) 

#if defined(CONFIG_MT_ENG_BUILD)
extern char dprec_error_log_buffer[];

#define DISPPR(string, args...) \
	do {\
		dprec_error_log_len = scnprintf(dprec_error_log_buffer, dprec_error_log_buflen, string, ##args); \
		dprec_logger_pr(DPREC_LOGGER_HWOP, dprec_error_log_buffer);\
	}while(0)
#else
#define DISPPR(string, args...) printk("[DISPPR]"string, ##args) 
#endif
	
#define DISPPR_HWOP(string, args...)  //dprec_logger_pr(DPREC_LOGGER_HWOP, string, ##args);
#define DISPPR_ERROR(string, args...)  do{dprec_logger_pr(DPREC_LOGGER_ERROR, string, ##args);printk("[DISP][%s #%d]ERROR:"string,__func__, __LINE__, ##args); }while(0)
#define DISPPR_FENCE(string, args...)  do{dprec_logger_pr(DPREC_LOGGER_FENCE, string, ##args);pr_debug("fence/"string, ##args); }while(0)

#endif // __DISP_DRV_LOG_H__
