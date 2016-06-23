#ifndef __SH_SVP_H__
#define __SH_SVP_H__

#if defined(CONFIG_CMA) && defined(CONFIG_MTK_SVP)
int svp_region_offline(void);
int svp_region_online(void);
#else
static inline int svp_region_offline(void) { return -ENOSYS; }
static inline int svp_region_online(void) { return -ENOSYS; }
#endif

extern struct cma *svp_contiguous_default_area;

#if defined(CONFIG_CMA) && defined(CONFIG_MTK_SVP)
void svp_contiguous_reserve(phys_addr_t addr_limit);

unsigned long get_svp_cma_basepfn(void);
unsigned long get_svp_cma_count(void);

int svp_migrate_range(unsigned long pfn);
int svp_is_in_range(unsigned long pfn);

#else
static inline void svp_contiguous_reserve(phys_addr_t limit) { }
static inline unsigned long get_svp_cma_basepfn(void) { return 0; }
static inline long get_svp_cma_count(void) { return 0; }
static inline int svp_migrate_range(unsigned long pfn) { return 0; }
static inline int svp_is_in_range(unsigned long pfn) { return 0; }
#endif

#endif
