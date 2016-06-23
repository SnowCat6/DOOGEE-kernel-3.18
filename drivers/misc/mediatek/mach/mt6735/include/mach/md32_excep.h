#ifndef __MD32_EXCEP_H__
#define __MD32_EXCEP_H__

typedef enum md32_excep_id
{
    EXCEP_SYSTEM = 0,
    MD32_NR_EXCEP,
} md32_excep_id;

typedef void(*md32_excep_handler_t)(void);

struct md32_excep_desc{
    md32_excep_handler_t stop_handler;
    md32_excep_handler_t reset_handler;
    md32_excep_handler_t restart_handler;
    const char *name;
};

int md32_excep_registration(md32_excep_id id, md32_excep_handler_t stop_handler, md32_excep_handler_t reset_handler, md32_excep_handler_t restart_handler, const char *name);

#endif
