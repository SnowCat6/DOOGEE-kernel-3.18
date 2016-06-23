#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/of_fdt.h>
#include <linux/lockdep.h>

#include <mach/irqs.h>
#include <mach/mt_cirq.h>
#include <mach/mt_spm_idle.h>
#include <mach/mt_cpuidle.h>
#include <mach/wd_api.h>
#include <mach/mt_cpufreq.h>
#include <mach/mt_ccci_common.h>
#include <mach/upmu_common.h>

#include "mt_spm_internal.h"


/**************************************
 * only for internal debug
 **************************************/
//FIXME: for FPGA early porting
#define  CONFIG_MTK_LDVT

#ifdef CONFIG_MTK_LDVT
#define SPM_PWAKE_EN            0
#define SPM_BYPASS_SYSPWREQ     1
#else
#define SPM_PWAKE_EN            1
#define SPM_BYPASS_SYSPWREQ     0
#endif

#define WAKE_SRC_FOR_DPIDLE                                                                      \
    (WAKE_SRC_KP | WAKE_SRC_GPT | WAKE_SRC_EINT | WAKE_SRC_CONN_WDT| \
     WAKE_SRC_CCIF0_MD| WAKE_SRC_CCIF1_MD |WAKE_SRC_CONN2AP| WAKE_SRC_USB_CD| WAKE_SRC_USB_PDN| WAKE_SRC_SEJ|\
     WAKE_SRC_AFE | WAKE_SRC_CIRQ | WAKE_SRC_MD1_VRF18_WAKE|WAKE_SRC_SYSPWREQ | WAKE_SRC_MD_WDT | \
     WAKE_SRC_CLDMA_MD)

#define WAKE_SRC_FOR_MD32  0

#define I2C_CHANNEL 2

#define spm_is_wakesrc_invalid(wakesrc)     (!!((u32)(wakesrc) & 0xc0003803))

#ifdef CONFIG_MTK_RAM_CONSOLE
#define SPM_AEE_RR_REC 1
#else
#define SPM_AEE_RR_REC 0
#endif
#define SPM_USE_TWAM_DEBUG	0

#if SPM_AEE_RR_REC
enum spm_deepidle_step
{
	SPM_DEEPIDLE_ENTER=0,
	SPM_DEEPIDLE_ENTER_UART_SLEEP,
	SPM_DEEPIDLE_ENTER_WFI,
	SPM_DEEPIDLE_LEAVE_WFI,
	SPM_DEEPIDLE_ENTER_UART_AWAKE,
	SPM_DEEPIDLE_LEAVE
};
#endif

/**********************************************************
 * PCM code for deep idle
 **********************************************************/ 
static const u32 dpidle_binary[] = {
	0x81429801, 0xd80001e5, 0x17c07c1f, 0x18c0001f, 0x10001124, 0x1910001f,
	0x10001124, 0xa1108404, 0xe0c00004, 0x1910001f, 0x10001124, 0x1a00001f,
	0x10006604, 0xc0c035a0, 0xe2200007, 0x1a00001f, 0x100062c4, 0x1890001f,
	0x100062c4, 0xa0940402, 0x1b00001f, 0x10000001, 0x1910001f, 0x10006310,
	0x80c39001, 0xd8200443, 0xe2000002, 0x80c10001, 0x1b00001f, 0xbf7ce7ff,
	0xd8200443, 0x17c07c1f, 0x1b00001f, 0x7f7ce7ff, 0xf0000000, 0x17c07c1f,
	0x81429801, 0xd8000665, 0x17c07c1f, 0x18c0001f, 0x10001124, 0x1910001f,
	0x10001124, 0x81308404, 0xe0c00004, 0x1910001f, 0x10001124, 0x1a00001f,
	0x10006604, 0xc0c035a0, 0xe2200007, 0x1a00001f, 0x100062c4, 0x1890001f,
	0x100062c4, 0x80b40402, 0x1b00001f, 0x00801001, 0x1910001f, 0x10006310,
	0x80c39001, 0xd82008c3, 0xe2000002, 0x80c10001, 0x1b00001f, 0x6ffcf7ff,
	0xd80008c3, 0x17c07c1f, 0x1b00001f, 0xaffce7ff, 0xf0000000, 0x17c07c1f,
	0x81459801, 0xd8000a05, 0x17c07c1f, 0x1a00001f, 0x10006604, 0xe2200002,
	0xc0c035a0, 0x17c07c1f, 0x80328400, 0x80318400, 0xc28034a0, 0x1290041f,
	0x1b00001f, 0x6ffcf7ff, 0xf0000000, 0x17c07c1f, 0x1b00001f, 0x2f7ce7ff,
	0x1b80001f, 0x20000004, 0xd8000d6c, 0x17c07c1f, 0xa0128400, 0xc28034a0,
	0x1290841f, 0x81459801, 0xd8000d25, 0x17c07c1f, 0x1a00001f, 0x10006604,
	0xe2200005, 0xc0c035a0, 0x17c07c1f, 0x1b00001f, 0x2f7cefff, 0xf0000000,
	0x17c07c1f, 0x18c0001f, 0x102135cc, 0x1910001f, 0x102135cc, 0x813f8404,
	0xe0c00004, 0x1910001f, 0x102135cc, 0x81411801, 0xd8000fc5, 0x17c07c1f,
	0x18c0001f, 0x10006240, 0xe0e00016, 0xe0e0001e, 0xe0e0000e, 0xe0e0000f,
	0x803e0400, 0x1b80001f, 0x20000222, 0x80380400, 0x1b80001f, 0x20000280,
	0x803b0400, 0x1b80001f, 0x2000001a, 0x803d0400, 0x1b80001f, 0x20000208,
	0x80340400, 0x80310400, 0x1b80001f, 0x2000000a, 0x18c0001f, 0x10006240,
	0xe0e0000d, 0xd80015e5, 0x17c07c1f, 0x1b80001f, 0x20000020, 0x18c0001f,
	0x102130f0, 0x1910001f, 0x102130f0, 0xa9000004, 0x10000000, 0xe0c00004,
	0x1b80001f, 0x2000000a, 0x89000004, 0xefffffff, 0xe0c00004, 0x18c0001f,
	0x102140f4, 0x1910001f, 0x102140f4, 0xa9000004, 0x02000000, 0xe0c00004,
	0x1b80001f, 0x2000000a, 0x89000004, 0xfdffffff, 0xe0c00004, 0x1910001f,
	0x102140f4, 0x81fa0407, 0x81f08407, 0xe8208000, 0x10006354, 0x001fdaa3,
	0xa1d80407, 0xa1de8407, 0xa1df0407, 0xc28034a0, 0x1291041f, 0x1b00001f,
	0xbf7ce7ff, 0x1950001f, 0x100062c4, 0x80c41401, 0xd8001843, 0x17c07c1f,
	0x1b00001f, 0xaffce7ff, 0xf0000000, 0x17c07c1f, 0x1b80001f, 0x20000fdf,
	0x1a50001f, 0x10006608, 0x80c9a401, 0x810ba401, 0x10920c1f, 0xa0979002,
	0x80ca2401, 0xa0938c02, 0x8080080d, 0xd8201cc2, 0x17c07c1f, 0x81f08407,
	0xa1d80407, 0xa1de8407, 0xa1df0407, 0x1b00001f, 0x2f7ce7ff, 0x1b80001f,
	0x20000004, 0xd800234c, 0x17c07c1f, 0x1b00001f, 0xbf7ce7ff, 0x1950001f,
	0x100062c4, 0x80c41401, 0xd8002343, 0x17c07c1f, 0x1b00001f, 0xaffce7ff,
	0xd0002340, 0x17c07c1f, 0x81f80407, 0x81fe8407, 0x81ff0407, 0x1900001f,
	0x40000000, 0x18d0001f, 0x40000000, 0xe1000003, 0xc0c03680, 0x1080041f,
	0x1880001f, 0x10006320, 0xc0c02ce0, 0xe080000f, 0xd8001a23, 0x17c07c1f,
	0xe080001f, 0xa1da0407, 0xa0110400, 0xa0140400, 0xa0180400, 0xa01b0400,
	0xa01d0400, 0x1b80001f, 0x20000068, 0xa01e0400, 0x1b80001f, 0x20000104,
	0x18c0001f, 0x102135cc, 0x1910001f, 0x102135cc, 0xa11f8404, 0xe0c00004,
	0x81411801, 0xd80021e5, 0x17c07c1f, 0x18c0001f, 0x10006240, 0xc0c02c20,
	0x17c07c1f, 0xc28034a0, 0x1291841f, 0x1b00001f, 0x6ffcf7ff, 0x1950001f,
	0x100062c4, 0x80c41401, 0xd8202343, 0x17c07c1f, 0x1b00001f, 0x7f7ce7ff,
	0xf0000000, 0x17c07c1f, 0x81441801, 0xd82025a5, 0x17c07c1f, 0x1a00001f,
	0x10006604, 0xc0c035a0, 0xe2200004, 0xc0c03680, 0x1092041f, 0xc0c035a0,
	0xe2200003, 0xc0c03680, 0x1092041f, 0xc0c035a0, 0xe2200002, 0xc0c03680,
	0x1092041f, 0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4, 0xa0908402,
	0xe2000002, 0x1b00001f, 0x00801001, 0xf0000000, 0x17c07c1f, 0x1a00001f,
	0x100062c4, 0x1890001f, 0x100062c4, 0x80b08402, 0xe2000002, 0x81441801,
	0xd82029c5, 0x17c07c1f, 0x1a00001f, 0x10006604, 0xc0c035a0, 0xe2200003,
	0xc0c03680, 0x1092041f, 0xc0c035a0, 0xe2200004, 0xc0c03680, 0x1092041f,
	0xc0c035a0, 0xe2200005, 0xc0c03680, 0x1092041f, 0x1b00001f, 0x00000801,
	0xf0000000, 0x17c07c1f, 0xe0f07f16, 0x1380201f, 0xe0f07f1e, 0x1380201f,
	0xe0f07f0e, 0x1b80001f, 0x20000100, 0xe0f07f0c, 0xe0f07f0d, 0xe0f07e0d,
	0xe0f07c0d, 0xe0f0780d, 0xe0f0700d, 0xf0000000, 0x17c07c1f, 0xe0f07f0d,
	0xe0f07f0f, 0xe0f07f1e, 0xe0f07f12, 0xf0000000, 0x17c07c1f, 0x1112841f,
	0xa1d08407, 0xd8202da4, 0x80eab401, 0xd8002d23, 0x01200404, 0x1a00001f,
	0x10006814, 0xe2000003, 0xf0000000, 0x17c07c1f, 0xa1d00407, 0x1b80001f,
	0x20000100, 0x80ea3401, 0x1a00001f, 0x10006814, 0xe2000003, 0xf0000000,
	0x17c07c1f, 0xd800306a, 0x17c07c1f, 0xe2e00036, 0x17c07c1f, 0x17c07c1f,
	0xe2e0003e, 0x1380201f, 0xe2e0003c, 0xd82031aa, 0x17c07c1f, 0x1b80001f,
	0x20000018, 0xe2e0007c, 0x1b80001f, 0x20000003, 0xe2e0005c, 0xe2e0004c,
	0xe2e0004d, 0xf0000000, 0x17c07c1f, 0xa1d10407, 0x1b80001f, 0x20000020,
	0xf0000000, 0x17c07c1f, 0xa1d40407, 0x1391841f, 0xa1d90407, 0xf0000000,
	0x17c07c1f, 0xd80033ca, 0x17c07c1f, 0xe2e0004f, 0xe2e0006f, 0xe2e0002f,
	0xd820346a, 0x17c07c1f, 0xe2e0002e, 0xe2e0003e, 0xe2e00032, 0xf0000000,
	0x17c07c1f, 0x18c0001f, 0x10006b18, 0x1910001f, 0x10006b18, 0xa1002804,
	0xe0c00004, 0xf0000000, 0x17c07c1f, 0x18d0001f, 0x10006604, 0x10cf8c1f,
	0xd82035a3, 0x17c07c1f, 0xf0000000, 0x17c07c1f, 0x81499801, 0xd82037e5,
	0x17c07c1f, 0xd8203d02, 0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f,
	0x70000000, 0xd80036e2, 0x00a00402, 0x814a1801, 0xd8203945, 0x17c07c1f,
	0xd8203d02, 0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f, 0x80000000,
	0xd8003842, 0x00a00402, 0x814a9801, 0xd8203aa5, 0x17c07c1f, 0xd8203d02,
	0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f, 0xc0000000, 0xd80039a2,
	0x00a00402, 0x814c1801, 0xd8203c05, 0x17c07c1f, 0xd8203d02, 0x17c07c1f,
	0x18d0001f, 0x40000000, 0x18d0001f, 0xa0000000, 0xd8003b02, 0x00a00402,
	0xd8203d02, 0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f, 0x40000000,
	0xd8003c02, 0x00a00402, 0xf0000000, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x1840001f, 0x00000001, 0xa1d48407, 0x1990001f,
	0x10006b08, 0xe8208000, 0x10006b18, 0x00000000, 0x81441801, 0xd8204565,
	0x17c07c1f, 0x1910001f, 0x100062c4, 0x80849001, 0x1a00001f, 0x10006b0c,
	0x1950001f, 0x10006b0c, 0xa1508805, 0xe2000005, 0x18c0001f, 0x2f7ce7ff,
	0x80809001, 0x81600801, 0xa0d59403, 0xa0d60803, 0xd8004465, 0x17c07c1f,
	0x80841001, 0x81600801, 0xa0db9403, 0xa0de0803, 0xd8004465, 0x17c07c1f,
	0x80f60403, 0xe8208000, 0x10006310, 0x0b160c38, 0x13000c1f, 0x1b80001f,
	0xd00f0000, 0xd00045a0, 0x17c07c1f, 0x1b00001f, 0x2f7ce7ff, 0x81469801,
	0xd8204705, 0x17c07c1f, 0x1b80001f, 0xd00f0000, 0x8880000c, 0x2f7ce7ff,
	0xd8006502, 0x17c07c1f, 0xd0004740, 0x17c07c1f, 0x1b80001f, 0x500f0000,
	0xe8208000, 0x10006354, 0x001fdaa3, 0xc0c031e0, 0x81401801, 0xd8004c65,
	0x17c07c1f, 0x81f60407, 0x18c0001f, 0x10006200, 0xc0c03320, 0x12807c1f,
	0xe8208000, 0x1000625c, 0x00000001, 0x1b80001f, 0x20000080, 0xc0c03320,
	0x1280041f, 0x18c0001f, 0x10006208, 0xc0c03320, 0x12807c1f, 0xe8208000,
	0x10006248, 0x00000000, 0x1b80001f, 0x20000080, 0xc0c03320, 0x1280041f,
	0x18c0001f, 0x10006290, 0xc0c03320, 0x12807c1f, 0xc0c03320, 0x1280041f,
	0x18c0001f, 0x100062dc, 0xe0c00001, 0xc28034a0, 0x1292041f, 0x81469801,
	0xd8004d45, 0x17c07c1f, 0x8880000c, 0x2f7ce7ff, 0xd8006062, 0x17c07c1f,
	0xc0c03280, 0x17c07c1f, 0x18c0001f, 0x10006294, 0xe0f07fff, 0xe0e00fff,
	0xe0e000ff, 0x81449801, 0xd8005065, 0x17c07c1f, 0x1a00001f, 0x10006604,
	0xe2200006, 0xc0c035a0, 0x17c07c1f, 0x1a00001f, 0x10006604, 0xe2200001,
	0xc0c035a0, 0x17c07c1f, 0x18c0001f, 0x1000111c, 0x1910001f, 0x10006b14,
	0xe0c00004, 0xa1d38407, 0xa1d98407, 0xa0108400, 0xa0120400, 0xa0148400,
	0xa0150400, 0xa0158400, 0xa01b8400, 0xa01c0400, 0xa01c8400, 0xa0188400,
	0xa0190400, 0xa0198400, 0x81441801, 0xd8205465, 0x17c07c1f, 0x1910001f,
	0x10006b0c, 0x1a00001f, 0x100062c4, 0x1950001f, 0x100062c4, 0x80809001,
	0x81748405, 0xa1548805, 0xe2000005, 0x1a00001f, 0x10006b0c, 0x80c51801,
	0x81308404, 0xa1108c04, 0xe2000004, 0xe8208000, 0x10006310, 0x0b1603f8,
	0x1950001f, 0x100062c4, 0x80841401, 0x81600801, 0x18c0001f, 0xaf7ce7ff,
	0xa0de0803, 0xa0db9403, 0x13000c1f, 0x1b80001f, 0x90100000, 0x80c28001,
	0xc8c00903, 0x17c07c1f, 0x80c10001, 0xc8c00da3, 0x17c07c1f, 0x1a00001f,
	0x100062c4, 0x1890001f, 0x100062c4, 0xa0908402, 0xe2000002, 0x18d0001f,
	0x10006284, 0x80810c01, 0xd8205b02, 0x17c07c1f, 0x18d0001f, 0x100062c4,
	0x80840c01, 0xd8005b02, 0x17c07c1f, 0x18c0001f, 0x10001124, 0x1910001f,
	0x10001124, 0xa1108404, 0xe0c00004, 0x1910001f, 0x10001124, 0x1a00001f,
	0x10006604, 0xe2200007, 0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4,
	0xa0940402, 0xe2000002, 0x1b00001f, 0x2f7ce7ff, 0x18c0001f, 0x10006294,
	0xe0e001fe, 0xe0e003fc, 0xe0e007f8, 0xe0e00ff0, 0x1b80001f, 0x20000020,
	0xe0f07ff0, 0xe0f07f00, 0x81449801, 0xd8005de5, 0x17c07c1f, 0x1a00001f,
	0x10006604, 0xe2200006, 0xc0c035a0, 0x17c07c1f, 0xe2200000, 0xc0c035a0,
	0x17c07c1f, 0x80388400, 0x80390400, 0x80398400, 0x1b80001f, 0x20000300,
	0x803b8400, 0x803c0400, 0x803c8400, 0x1b80001f, 0x20000300, 0x80348400,
	0x80350400, 0x80358400, 0x1b80001f, 0x20000104, 0x10007c1f, 0x81f38407,
	0x81f98407, 0x81f90407, 0x81f40407, 0x81401801, 0xd8006505, 0x17c07c1f,
	0x18c0001f, 0x100062dc, 0xe0c0001f, 0x18c0001f, 0x10006290, 0x1212841f,
	0xc0c02f60, 0x12807c1f, 0xc0c02f60, 0x1280041f, 0x18c0001f, 0x10006208,
	0x1212841f, 0xc0c02f60, 0x12807c1f, 0xe8208000, 0x10006248, 0x00000001,
	0x1b80001f, 0x20000080, 0xc0c02f60, 0x1280041f, 0x18c0001f, 0x10006200,
	0x1212841f, 0xc0c02f60, 0x12807c1f, 0xe8208000, 0x1000625c, 0x00000000,
	0x1b80001f, 0x20000080, 0xc0c02f60, 0x1280041f, 0x19c0001f, 0x60415820,
	0xe8208000, 0x10006310, 0x0b160008, 0xc28034a0, 0x1293841f, 0x18c0001f,
	0x10006b14, 0xe0c0000c, 0x18c0001f, 0x10006b68, 0x1950001f, 0x100063c0,
	0xe0c00005, 0x81441801, 0xd8206b05, 0x10c0041f, 0x1910001f, 0x100062c4,
	0x80849001, 0x1a00001f, 0x10006b0c, 0x1910001f, 0x10006b0c, 0xa0908804,
	0xe2000002, 0x81441801, 0xd8206b05, 0x10c0041f, 0x1910001f, 0x100062c4,
	0x80809001, 0x81600801, 0xa0d59403, 0xa0d60803, 0xd8006aa5, 0x17c07c1f,
	0x80841001, 0x81600801, 0xa0db9403, 0xa0de0803, 0xd8006aa5, 0x17c07c1f,
	0x80f60403, 0xe8208000, 0x10006310, 0x0b160c38, 0x13000c1f, 0x1b80001f,
	0x900a0000, 0x88900001, 0x10006814, 0xd8206862, 0x17c07c1f, 0x18d0001f,
	0x10006b6c, 0x78a00003, 0x0000beef, 0xd8006d22, 0x17c07c1f, 0xc0c06f80,
	0x17c07c1f, 0xd0006860, 0x17c07c1f, 0x1910001f, 0x10006b0c, 0x1a00001f,
	0x100062c4, 0x1950001f, 0x100062c4, 0x80809001, 0x81748405, 0xa1548805,
	0xe2000005, 0x80841401, 0xd8006f02, 0x8204b401, 0xc8c00008, 0x17c07c1f,
	0x1ac0001f, 0x55aa55aa, 0x10007c1f, 0xf0000000, 0x1900001f, 0x10006014,
	0x1950001f, 0x10006014, 0xa1508405, 0xe1000005, 0x1900001f, 0x10006814,
	0xe100001f, 0x812ab401, 0xd80070a4, 0x17c07c1f, 0x1880001f, 0x10006284,
	0x18d0001f, 0x10006284, 0x80f20403, 0xe0800003, 0x80f08403, 0xe0800003,
	0x1900001f, 0x10006014, 0x1950001f, 0x10006014, 0x81708405, 0xe1000005,
	0x1900001f, 0x10006b6c, 0xe100001f, 0x81441801, 0xd80074e5, 0x17c07c1f,
	0x18c0001f, 0x10001124, 0x1910001f, 0x10001124, 0xa1108404, 0xe0c00004,
	0x1910001f, 0x10001124, 0x1a00001f, 0x10006604, 0xe2200007, 0xf0000000,
	0x17c07c1f
};
static struct pcm_desc dpidle_pcm = {
	.version	= "pcm_deepidle_v0.5.10_20150408",
	.base		= dpidle_binary,
	.size		= 937,
	.sess		= 2,
	.replace	= 0,
	.vec0		= EVENT_VEC(23, 1, 0, 0),	/* FUNC_MD_VRF18_WAKEUP */
	.vec1		= EVENT_VEC(28, 1, 0, 36),	/* FUNC_MD_VRF18_SLEEP */
	.vec2		= EVENT_VEC(11, 1, 0, 72),	/* FUNC_26M_WAKEUP */
	.vec3		= EVENT_VEC(12, 1, 0, 88),	/* FUNC_26M_SLEEP */
	.vec4		= EVENT_VEC(30, 1, 0, 109),	/* FUNC_APSRC_WAKEUP */
	.vec5		= EVENT_VEC(31, 1, 0, 196),	/* FUNC_APSRC_SLEEP */
	.vec6		= EVENT_VEC(11, 1, 0, 284),	/* FUNC_VCORE_HIGH */
	.vec7		= EVENT_VEC(12, 1, 0, 311),	/* FUNC_VCORE_LOW */
};



static struct pwr_ctrl dpidle_ctrl = {
	.wake_src		= WAKE_SRC_FOR_DPIDLE,
	.wake_src_md32		= WAKE_SRC_FOR_MD32,
	.r0_ctrl_en		= 1,
	.r7_ctrl_en		= 1,
	.infra_dcm_lock		= 1,
	.wfi_op			= WFI_OP_AND,
	.ca15_wfi0_en		= 1,
	.ca15_wfi1_en		= 1,
	.ca15_wfi2_en		= 1,
	.ca15_wfi3_en		= 1,
	.ca7_wfi0_en		= 1,
	.ca7_wfi1_en		= 1,
	.ca7_wfi2_en		= 1,
	.ca7_wfi3_en		= 1,
	.md2_req_mask		= 1,
	.disp_req_mask		= 1,
	.mfg_req_mask		= 1,
	.lte_mask		= 1,
	.syspwreq_mask		= 1,
};

struct spm_lp_scen __spm_dpidle = {
	.pcmdesc	= &dpidle_pcm,
	.pwrctrl	= &dpidle_ctrl,
};

extern int mt_irq_mask_all(struct mtk_irq_mask *mask);
extern int mt_irq_mask_restore(struct mtk_irq_mask *mask);
extern void mt_irq_unmask_for_sleep(unsigned int irq);
extern int request_uart_to_sleep(void);
extern int request_uart_to_wakeup(void);
extern unsigned int mt_get_clk_mem_sel(void);
extern int is_ext_buck_exist(void);


#if SPM_AEE_RR_REC
extern void aee_rr_rec_deepidle_val(u32 val);
extern u32 aee_rr_curr_deepidle_val(void);
#endif

#if 0
void __attribute__((weak)) mt_cirq_clone_gic(void){}
void __attribute__((weak)) mt_cirq_enable(void){}
void __attribute__((weak)) mt_cirq_flush(void){}
void __attribute__((weak)) mt_cirq_disable(void){}
#endif

static void spm_trigger_wfi_for_dpidle(struct pwr_ctrl *pwrctrl)
{
    if (is_cpu_pdn(pwrctrl->pcm_flags)) {
        mt_cpu_dormant(CPU_DEEPIDLE_MODE);
    } else {
        //Mp0_axi_config[4] is one by default. No need to program it before entering suspend.
        wfi_with_sync();
    }
}

/*
 * wakesrc: WAKE_SRC_XXX
 * enable : enable or disable @wakesrc
 * replace: if true, will replace the default setting
 */
int spm_set_dpidle_wakesrc(u32 wakesrc, bool enable, bool replace)
{
    unsigned long flags;

    if (spm_is_wakesrc_invalid(wakesrc))
        return -EINVAL;

    spin_lock_irqsave(&__spm_lock, flags);
    if (enable) {
        if (replace)
            __spm_dpidle.pwrctrl->wake_src = wakesrc;
        else
            __spm_dpidle.pwrctrl->wake_src |= wakesrc;
    } else {
        if (replace)
            __spm_dpidle.pwrctrl->wake_src = 0;
        else
            __spm_dpidle.pwrctrl->wake_src &= ~wakesrc;
    }
    spin_unlock_irqrestore(&__spm_lock, flags);

    return 0;
}
/*  //TODO
extern void spm_i2c_control(u32 channel, bool onoff);
U32 g_bus_ctrl = 0;
U32 g_sys_ck_sel = 0;
*/
static u32 vsram_vosel_on_lb;
static void spm_dpidle_pre_process(void)
{
/*  //TODO
    spm_i2c_control(I2C_CHANNEL, 1);//D1,D2 no ext bulk
    g_bus_ctrl=spm_read(0xF0001070);
    spm_write(0xF0001070 , g_bus_ctrl | (1 << 21)); //bus dcm disable
    g_sys_ck_sel = spm_read(0xF0001108);
    //spm_write(0xF0001108 , g_sys_ck_sel &~ (1<<1) );
    spm_write(0xF0001108 , 0x0);
    spm_write(0xF0000204 , spm_read(0xF0000204) | (1 << 0));  // BUS 26MHz enable 
*/
    /* set PMIC WRAP table for deepidle power control */
    mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_DEEPIDLE);
    
    vsram_vosel_on_lb = pmic_get_register_value(PMIC_VSRAM_VOSEL_ON_LB);
    spm_write(SPM_PCM_RESERVE3,(pmic_get_register_value(PMIC_VSRAM_VOSEL_OFFSET)<<8)|pmic_get_register_value(PMIC_VSRAM_VOSEL_DELTA));//delta = 0v
    pmic_set_register_value(PMIC_VSRAM_VOSEL_ON_LB,(vsram_vosel_on_lb&0xff80)|0x28);//0.85v
}

static void spm_dpidle_post_process(void)
{
/*   //TODO
    spm_i2c_control(I2C_CHANNEL, 0);
    spm_write(0xF0001070 , g_bus_ctrl); //26:26 enable 
*/
    pmic_set_register_value(PMIC_VSRAM_VOSEL_ON_LB,vsram_vosel_on_lb); 
    /* set PMIC WRAP table for normal power control */
    mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_NORMAL);       
}
#ifdef SPM_DEEPIDLE_PROFILE_TIME
extern unsigned int dpidle_profile[4];
#endif
wake_reason_t spm_go_to_dpidle(u32 spm_flags, u32 spm_data)
{
    struct wake_status wakesta;
    unsigned long flags;
    struct mtk_irq_mask mask;
    wake_reason_t wr = WR_NONE;
    struct pcm_desc *pcmdesc = __spm_dpidle.pcmdesc;
    struct pwr_ctrl *pwrctrl = __spm_dpidle.pwrctrl;

#if SPM_AEE_RR_REC
    aee_rr_rec_deepidle_val(1<<SPM_DEEPIDLE_ENTER);
#endif 

    set_pwrctrl_pcm_flags(pwrctrl, spm_flags);
    
    //pwrctrl->timer_val = 1 * 32768;

    spm_dpidle_before_wfi();

    lockdep_off();
    spin_lock_irqsave(&__spm_lock, flags);    
    mt_irq_mask_all(&mask);
    mt_irq_unmask_for_sleep(SPM_IRQ0_ID);
    mt_cirq_clone_gic();
    mt_cirq_enable();

#if SPM_AEE_RR_REC
    aee_rr_rec_deepidle_val(aee_rr_curr_deepidle_val()|(1<<SPM_DEEPIDLE_ENTER_UART_SLEEP));
#endif     

    if (request_uart_to_sleep()) {
        wr = WR_UART_BUSY;
        goto RESTORE_IRQ;
    }

    __spm_reset_and_init_pcm(pcmdesc);

    __spm_kick_im_to_fetch(pcmdesc);
	
    __spm_init_pcm_register();

    __spm_init_event_vector(pcmdesc);

    __spm_set_power_control(pwrctrl);

    __spm_set_wakeup_event(pwrctrl);

    spm_dpidle_pre_process();

    __spm_kick_pcm_to_run(pwrctrl);

#if SPM_AEE_RR_REC
    aee_rr_rec_deepidle_val(aee_rr_curr_deepidle_val()|(1<<SPM_DEEPIDLE_ENTER_WFI));
#endif

#ifdef SPM_DEEPIDLE_PROFILE_TIME
    gpt_get_cnt(SPM_PROFILE_APXGPT,&dpidle_profile[1]);
#endif   
    spm_trigger_wfi_for_dpidle(pwrctrl);
#ifdef SPM_DEEPIDLE_PROFILE_TIME
    gpt_get_cnt(SPM_PROFILE_APXGPT,&dpidle_profile[2]);
#endif

#if SPM_AEE_RR_REC
    aee_rr_rec_deepidle_val(aee_rr_curr_deepidle_val()|(1<<SPM_DEEPIDLE_LEAVE_WFI));
#endif 

    spm_dpidle_post_process();

    __spm_get_wakeup_status(&wakesta);

    __spm_clean_after_wakeup();

#if SPM_AEE_RR_REC
    aee_rr_rec_deepidle_val(aee_rr_curr_deepidle_val()|(1<<SPM_DEEPIDLE_ENTER_UART_AWAKE));
#endif

    request_uart_to_wakeup();

    wr = __spm_output_wake_reason(&wakesta, pcmdesc, false);

RESTORE_IRQ:
    mt_cirq_flush();
    mt_cirq_disable();
    mt_irq_mask_restore(&mask);  
    spin_unlock_irqrestore(&__spm_lock, flags);
    lockdep_on();
    spm_dpidle_after_wfi();

#if SPM_AEE_RR_REC
    aee_rr_rec_deepidle_val(0);
#endif 
    return wr;
}

/*
 * cpu_pdn:
 *    true  = CPU dormant
 *    false = CPU standby
 * pwrlevel:
 *    0 = AXI is off
 *    1 = AXI is 26M
 * pwake_time:
 *    >= 0  = specific wakeup period
 */
wake_reason_t spm_go_to_sleep_dpidle(u32 spm_flags, u32 spm_data)
{
    u32 sec = 0;
    u32 dpidle_timer_val = 0;
    u32 dpidle_wake_src = 0;
    int wd_ret;
    struct wake_status wakesta;
    unsigned long flags;
    struct mtk_irq_mask mask;
    struct wd_api *wd_api;
    static wake_reason_t last_wr = WR_NONE;
    struct pcm_desc *pcmdesc = __spm_dpidle.pcmdesc;
    struct pwr_ctrl *pwrctrl = __spm_dpidle.pwrctrl;

	/* backup original dpidle setting */
	dpidle_timer_val = pwrctrl->timer_val;
	dpidle_wake_src = pwrctrl->wake_src;

    set_pwrctrl_pcm_flags(pwrctrl, spm_flags);

#if SPM_PWAKE_EN
    sec = spm_get_wake_period(-1 /* FIXME */, last_wr);
#endif
    pwrctrl->timer_val = sec * 32768;

    pwrctrl->wake_src = spm_get_sleep_wakesrc();

    wd_ret = get_wd_api(&wd_api);
    if (!wd_ret)
        wd_api->wd_suspend_notify();

    spin_lock_irqsave(&__spm_lock, flags);

    mt_irq_mask_all(&mask);
    mt_irq_unmask_for_sleep(SPM_IRQ0_ID);
	
    mt_cirq_clone_gic();
    mt_cirq_enable();

    /* set PMIC WRAP table for deepidle power control */
    mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_DEEPIDLE);

    spm_crit2("sleep_deepidle, sec = %u, wakesrc = 0x%x [%u]\n",
              sec, pwrctrl->wake_src, is_cpu_pdn(pwrctrl->pcm_flags));

    __spm_reset_and_init_pcm(pcmdesc);

    __spm_kick_im_to_fetch(pcmdesc);

    if (request_uart_to_sleep()) {
        last_wr = WR_UART_BUSY;
        goto RESTORE_IRQ;
    }

    __spm_init_pcm_register();

    __spm_init_event_vector(pcmdesc);

    __spm_set_power_control(pwrctrl);

    __spm_set_wakeup_event(pwrctrl);

    __spm_kick_pcm_to_run(pwrctrl);

    spm_dpidle_pre_process();

    spm_trigger_wfi_for_dpidle(pwrctrl);

    spm_dpidle_post_process();    

    __spm_get_wakeup_status(&wakesta);

    __spm_clean_after_wakeup();

    request_uart_to_wakeup();

    last_wr = __spm_output_wake_reason(&wakesta, pcmdesc, true);

RESTORE_IRQ:

    /* set PMIC WRAP table for normal power control */
    mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_NORMAL);

    mt_cirq_flush();
    mt_cirq_disable();

    mt_irq_mask_restore(&mask);

    spin_unlock_irqrestore(&__spm_lock, flags);

    if (!wd_ret)
        wd_api->wd_resume_notify();

	/* restore original dpidle setting */
	pwrctrl->timer_val = dpidle_timer_val;
	pwrctrl->wake_src = dpidle_wake_src;

    return last_wr;  
}

#if SPM_USE_TWAM_DEBUG
#define SPM_TWAM_MONITOR_TICK 333333


static void twam_handler(struct twam_sig *twamsig)
{
	spm_crit("sig_high = %u%%  %u%%  %u%%  %u%%, r13 = 0x%x\n",
		 get_percent(twamsig->sig0,SPM_TWAM_MONITOR_TICK),
		 get_percent(twamsig->sig1,SPM_TWAM_MONITOR_TICK),
		 get_percent(twamsig->sig2,SPM_TWAM_MONITOR_TICK),
		 get_percent(twamsig->sig3,SPM_TWAM_MONITOR_TICK),
		 spm_read(SPM_PCM_REG13_DATA));
}
#endif

void spm_deepidle_init(void)
{
#if SPM_USE_TWAM_DEBUG
	unsigned long flags;
	struct twam_sig twamsig = {
		.sig0 = 26,	/* md1_srcclkena */
		.sig1 = 22,	/* md_apsrc_req_mux */
		.sig2 = 25,	/* md2_srcclkena */
		.sig3 = 21,	/* md2_apsrc_req_mux */		
		//.sig2 = 23,	/* conn_srcclkena */
		//.sig3 = 20,	/* conn_apsrc_req */
	};
#if 0
	spin_lock_irqsave(&__spm_lock, flags);
	spm_write(SPM_AP_STANBY_CON, spm_read(SPM_AP_STANBY_CON) | ASC_MD_DDR_EN_SEL);
	spin_unlock_irqrestore(&__spm_lock, flags);
#endif
	spm_twam_register_handler(twam_handler);
	spm_twam_enable_monitor(&twamsig, false,SPM_TWAM_MONITOR_TICK);
#endif
}


MODULE_DESCRIPTION("SPM-DPIdle Driver v0.1");
