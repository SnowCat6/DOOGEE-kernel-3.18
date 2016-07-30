

#if defined(CONFIG_MT_ENG_BUILD)
/* If build eng load */
#if !defined(CONFIG_DEBUG_LOCK_ALLOC)
	#error "CONFIG_DEBUG_LOCK_ALLOC was enabled, please check"
#elif defined(CONFIG_MTK_HIBERNATION)
	#error "CONFIG_MTK_HIBERNATION was enabled, please check"
#elif defined(CONFIG_QUOTA)
	#error "CONFIG_QUOTA was enabled, please check"
#elif defined(CONFIG_DEBUG_SLAB)
	#error "CONFIG_DEBUG_SLAB was enabled, please check"
#elif defined(CONFIG_MEMCG_KMEM)
	#error "CONFIG_MEMCG_KMEM was enabled, please check"
#elif !defined(CONFIG_DEBUG_MUTEXES)
	#error "CONFIG_DEBUG_MUTEXES was enabled, please check"
#elif defined(CONFIG_MUTEX_SPIN_ON_OWNERN)
	#error "CONFIG_MUTEX_SPIN_ON_OWNER was enabled, please check"
#elif defined(CONFIG_NUMA)
	#error "CONFIG_NUMA was enabled, please check"
#endif

#else
/* if build user load */
#if defined(CONFIG_DEBUG_LOCK_ALLOC)
	#error "CONFIG_DEBUG_LOCK_ALLOC was enabled, please check"
#elif defined(CONFIG_MTK_HIBERNATION)
	#error "CONFIG_MTK_HIBERNATION was enabled, please check"
#elif defined(CONFIG_QUOTA)
	#error "CONFIG_QUOTA was enabled, please check"
#elif defined(CONFIG_DEBUG_SLAB)
	#error "CONFIG_DEBUG_SLAB was enabled, please check"
#elif defined(CONFIG_MEMCG_KMEM)
	#error "CONFIG_MEMCG_KMEM was enabled, please check"
#elif defined(CONFIG_DEBUG_MUTEXES)
	#error "CONFIG_DEBUG_MUTEXES was enabled, please check"
#elif defined(CONFIG_MUTEX_SPIN_ON_OWNERN)
	#error "CONFIG_MUTEX_SPIN_ON_OWNER was enabled, please check"
#elif defined(CONFIG_NUMA)
	#error "CONFIG_NUMA was enabled, please check"
#endif

#endif /* End defined(CONFIG_MT_ENG_BUILD) */
