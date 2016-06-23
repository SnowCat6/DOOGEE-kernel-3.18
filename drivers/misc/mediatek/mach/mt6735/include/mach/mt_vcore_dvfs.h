#ifndef _MT_VCORE_DVFS_
#define _MT_VCORE_DVFS_

//Denali-1
#if defined(CONFIG_ARCH_MT6735)
#include "mach/mt_vcore_dvfs_1.h"
#endif

//Denali-2
#if defined(CONFIG_ARCH_MT6735M)
#include "mach/mt_vcore_dvfs_2.h"
#endif

//Denali-3
#if defined(CONFIG_ARCH_MT6753)
#include "mach/mt_vcore_dvfs_3.h"
#endif

#endif
