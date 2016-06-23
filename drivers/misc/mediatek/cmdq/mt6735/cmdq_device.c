#include "cmdq_device.h"
#include "cmdq_core.h"
#include <mach/mt_irq.h>

/* device tree */
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>

typedef struct CmdqModuleBaseVA {
	long MMSYS_CONFIG;
	long MDP_RDMA;
	long MDP_RSZ0;
	long MDP_RSZ1;
	long MDP_WDMA;
	long MDP_WROT;
	long MDP_TDSHP;
	long MM_MUTEX;
	long VENC;
} CmdqModuleBaseVA;

#ifdef CMDQ_INSTRUCTION_COUNT
extern CmdqModuleBasePA gCmdqModulePA;

void cmdq_dev_alloc_disp_module_PA_by_name(const char *name, int index, long *startPA, long *endPA)
{
	struct device_node *node = NULL;
	struct resource res;

	node = of_find_compatible_node(NULL, NULL, name);
	of_address_to_resource(node, index, &res);
	*startPA = res.start;
	*endPA = res.end;
	CMDQ_LOG("DEV: PA(%s): start = 0x%lx, end = 0x%lx\n", name, *startPA, *endPA);
}
#endif

typedef struct CmdqDeviceStruct {
	struct device *pDev;
	long regBaseVA;		/* considering 64 bit kernel, use long */
	long regBasePA;
	uint32_t irqId;
	uint32_t irqSecId;
} CmdqDeviceStruct;

static CmdqModuleBaseVA gCmdqModuleBaseVA;
static CmdqDeviceStruct gCmdqDev;

struct device *cmdq_dev_get(void)
{
	return gCmdqDev.pDev;
}

const uint32_t cmdq_dev_get_irq_id(void)
{
	return gCmdqDev.irqId;
}

const uint32_t cmdq_dev_get_irq_secure_id(void)
{
	return gCmdqDev.irqSecId;
}

const long cmdq_dev_get_module_base_VA_GCE(void)
{
	return gCmdqDev.regBaseVA;
}

const long cmdq_dev_get_module_base_PA_GCE(void)
{
	return gCmdqDev.regBasePA;
}

const long cmdq_dev_get_module_base_VA_MMSYS_CONFIG(void)
{
	return gCmdqModuleBaseVA.MMSYS_CONFIG;
}

const long cmdq_dev_get_module_base_VA_MDP_RDMA(void)
{
	return gCmdqModuleBaseVA.MDP_RDMA;
}

const long cmdq_dev_get_module_base_VA_MDP_RSZ0(void)
{
	return gCmdqModuleBaseVA.MDP_RSZ0;
}

const long cmdq_dev_get_module_base_VA_MDP_RSZ1(void)
{
	return gCmdqModuleBaseVA.MDP_RSZ1;
}

const long cmdq_dev_get_module_base_VA_MDP_WDMA(void)
{
	return gCmdqModuleBaseVA.MDP_WDMA;
}

const long cmdq_dev_get_module_base_VA_MDP_WROT(void)
{
	return gCmdqModuleBaseVA.MDP_WROT;
}

const long cmdq_dev_get_module_base_VA_MDP_TDSHP(void)
{
	return gCmdqModuleBaseVA.MDP_TDSHP;
}

const long cmdq_dev_get_module_base_VA_MM_MUTEX(void)
{
	return gCmdqModuleBaseVA.MM_MUTEX;
}

const long cmdq_dev_get_module_base_VA_VENC(void)
{
	return gCmdqModuleBaseVA.VENC;
}

const long cmdq_dev_alloc_module_base_VA_by_name(const char *name)
{
	unsigned long VA;
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, name);
	VA = (unsigned long)of_iomap(node, 0);
	CMDQ_LOG("DEV: VA(%s): 0x%lx\n", name, VA);

	return VA;
}

void cmdq_dev_free_module_base_VA(const long VA)
{
	iounmap((void*)VA);
}

void cmdq_dev_init_module_base_VA(void)
{
	memset(&gCmdqModuleBaseVA, 0, sizeof(CmdqModuleBaseVA));

#ifdef CMDQ_OF_SUPPORT
	gCmdqModuleBaseVA.MMSYS_CONFIG =
	    cmdq_dev_alloc_module_base_VA_by_name("mediatek,MMSYS_CONFIG");
	gCmdqModuleBaseVA.MDP_RDMA = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_RDMA");
	gCmdqModuleBaseVA.MDP_RSZ0 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_RSZ0");
	gCmdqModuleBaseVA.MDP_RSZ1 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_RSZ1");
	gCmdqModuleBaseVA.MDP_WDMA = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_WDMA");
	gCmdqModuleBaseVA.MDP_WROT = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_WROT");
	gCmdqModuleBaseVA.MDP_TDSHP = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_TDSHP");
	gCmdqModuleBaseVA.MM_MUTEX = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MM_MUTEX");
	gCmdqModuleBaseVA.VENC = cmdq_dev_alloc_module_base_VA_by_name("mediatek,VENC");
#ifdef CMDQ_INSTRUCTION_COUNT
	int32_t i;

	memset(&gCmdqModulePA, 0, sizeof(gCmdqModulePA));
	/* Get MM_SYS config registers range */
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,MMSYS_CONFIG", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_MMSYS_CONFIG], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_MMSYS_CONFIG]);
	/* Get MDP module registers range */
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,MDP_RDMA", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_RDMA], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_RDMA]);
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,MDP_RSZ0", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_RSZ0], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_RSZ0]);
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,MDP_RSZ1", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_RSZ1], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_RSZ1]);
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,MDP_WDMA", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_WDMA], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_WDMA]);
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,MDP_WROT", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_WROT], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_WROT]);
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,MDP_TDSHP", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_TDSHP], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_MDP_TDSHP]);
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,MM_MUTEX", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_MM_MUTEX], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_MM_MUTEX]);
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,VENC", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_VENC], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_VENC]);
	/* Get DISP module registers range */
	for (i = CMDQ_MODULE_INSTRUCTION_COUNT_DISP_OVL0; i <= CMDQ_MODULE_INSTRUCTION_COUNT_DISP_DPI0; i++) {
		cmdq_dev_alloc_disp_module_PA_by_name("mediatek,DISPSYS", (i-CMDQ_MODULE_INSTRUCTION_COUNT_DISP_OVL0),
		&gCmdqModulePA.start[i], &gCmdqModulePA.end[i]);
	}
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,DISPSYS", 31,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_DISP_OD], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_DISP_OD]);
	/* Get CAM module registers range */
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,CAM0", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_CAM0], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_CAM0]);
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,CAM1", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_CAM1], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_CAM1]);
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,CAM2", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_CAM2], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_CAM2]);
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,CAM3", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_CAM3], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_CAM3]);
	/* Get SODI registers range */
	cmdq_dev_alloc_disp_module_PA_by_name("mediatek,SLEEP", 0,
		&gCmdqModulePA.start[CMDQ_MODULE_INSTRUCTION_COUNT_SODI], &gCmdqModulePA.end[CMDQ_MODULE_INSTRUCTION_COUNT_SODI]);
#endif
#else
	gCmdqModuleBaseVA.MMSYS_CONFIG = MMSYS_CONFIG_BASE;
	gCmdqModuleBaseVA.MDP_RDMA = MDP_RDMA_BASE;
	gCmdqModuleBaseVA.MDP_RSZ0 = MDP_RSZ0_BASE;
	gCmdqModuleBaseVA.MDP_RSZ1 = MDP_RSZ1_BASE;
	gCmdqModuleBaseVA.MDP_WDMA = MDP_WDMA_BASE;
	gCmdqModuleBaseVA.MDP_WROT = MDP_WROT_BASE;
	gCmdqModuleBaseVA.MDP_TDSHP = MDP_TDSHP_BASE;
	gCmdqModuleBaseVA.MM_MUTEX = MMSYS_MUTEX_BASE;
	gCmdqModuleBaseVA.VENC = VENC_BASE;
#endif
}

void cmdq_dev_deinit_module_base_VA(void)
{
#ifdef CMDQ_OF_SUPPORT
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MMSYS_CONFIG());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_RDMA());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_RSZ0());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_RSZ1());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_WDMA());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_WROT());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_TDSHP());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MM_MUTEX());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_VENC());

	memset(&gCmdqModuleBaseVA, 0, sizeof(CmdqModuleBaseVA));
#else
	/* do nothing, registers' IOMAP will be destoryed by platform */
#endif
}

void cmdq_dev_init(struct platform_device *pDevice)
{
	struct device_node *node = pDevice->dev.of_node;

	/* init cmdq device dependent data */
	do {
		memset(&gCmdqDev, 0x0, sizeof(CmdqDeviceStruct));

		gCmdqDev.pDev = &pDevice->dev;
#ifdef CMDQ_OF_SUPPORT
		gCmdqDev.regBaseVA = (unsigned long)of_iomap(node, 0);
		gCmdqDev.regBasePA = (0L | 0x10217000);
		gCmdqDev.irqId = irq_of_parse_and_map(node, 0);
		gCmdqDev.irqSecId = irq_of_parse_and_map(node, 1);
#else
		gCmdqDev.regBaseVA = (0L | GCE_BASE);
		gCmdqDev.regBasePA = (0L | 0x10217000);
		gCmdqDev.irqId = CQ_DMA_IRQ_BIT_ID;
		gCmdqDev.irqSecId = CQ_DMA_SEC_IRQ_BIT_ID;
#endif

		CMDQ_LOG
		    ("[CMDQ] platform_dev: dev: %p, PA: %lx, VA: %lx, irqId: %d,  irqSecId:%d\n",
		     gCmdqDev.pDev, gCmdqDev.regBasePA, gCmdqDev.regBaseVA, gCmdqDev.irqId,
		     gCmdqDev.irqSecId);
	} while (0);

	/* init module VA */
	cmdq_dev_init_module_base_VA();
}

void cmdq_dev_deinit(void)
{
	cmdq_dev_deinit_module_base_VA();

	/* deinit cmdq device dependent data */
	do {
#ifdef CMDQ_OF_SUPPORT
		cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_GCE());
		gCmdqDev.regBaseVA = 0;
#else
		/* do nothing */
#endif
	} while (0);
}
