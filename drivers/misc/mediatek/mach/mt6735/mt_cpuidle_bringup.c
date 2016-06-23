unsigned int *mt_save_banked_registers(unsigned int *container) { return container; }

void mt_restore_banked_registers(unsigned int *container) { }

unsigned int *mt_save_generic_timer(unsigned int *container, int sw) { return container; }

void mt_restore_generic_timer(unsigned int *container, int sw) { }

int mt_cpu_dormant(unsigned long flags) { return 0; }

int mt_cpu_dormant_init(void) { return 0; }

