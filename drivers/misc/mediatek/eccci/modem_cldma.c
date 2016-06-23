/*
 * this is a CLDMA modem driver.
 *
 * V0.1: Xiao Wang <xiao.wang@mediatek.com>
 */
#include <linux/list.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/platform_device.h>
#if defined (CONFIG_MTK_AEE_FEATURE)
#include <linux/aee.h>
#endif
#include <mach/mt_boot.h>
#include "ccci_core.h"
#include "ccci_bm.h"
#include "ccci_platform.h"
#include "modem_cldma.h"
#include "cldma_platform.h"
#include "cldma_reg.h"
#include "modem_reg_base.h"
#define CLDMA_TRACE
#ifdef CLDMA_TRACE
#define CREATE_TRACE_POINTS
#include "modem_cldma_events.h"
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#ifdef ENABLE_CLDMA_AP_SIDE
#include <linux/syscore_ops.h>
#endif

#if defined (ENABLE_32K_CLK_LESS)
#include <mach/mtk_rtc.h>
#endif

enum{
    CCCI_TRACE_TX_IRQ=0,
    CCCI_TRACE_RX_IRQ=1,
};
#ifdef CONFIG_MTK_SVLTE_SUPPORT
extern void c2k_reset_modem(void);
#endif
static unsigned long long trace_sample_time=200000000;
extern void mt_irq_dump_status(int irq);
// always keep this in mind: what if there are more than 1 modems using CLDMA...

extern unsigned int ccci_get_md_debug_mode(struct ccci_modem *md);
static int md_cd_ccif_send(struct ccci_modem *md, int channel_id);

extern int spm_is_md1_sleep(void);
extern u32 mt_irq_get_pending(unsigned int irq);
// CLDMA setting
/*
 * we use this as rgpd->data_allow_len, so skb length must be >= this size, check ccci_bm.c's skb pool design.
 * channel 3 is for network in normal mode, but for mdlogger_ctrl in exception mode, so choose the max packet size.
 */
static int rx_queue_buffer_size[8] = {SKB_4K, SKB_4K, SKB_4K, SKB_1_5K, SKB_1_5K, SKB_1_5K, SKB_4K, SKB_16};
#ifdef CLDMA_TIMER_LOOP
static int tx_ioc_interval[8] = {1, 1, 1, 1, 1, 1, 1, 1};
#else
static int tx_ioc_interval[8] = {1, 1, 1, 2, 16, 2, 1, 1};
#endif
static int rx_queue_buffer_number[8] = {16, 16, 16, 512, 512, 128, 16, 2};
static int tx_queue_buffer_number[8] = {16, 16, 16, 256, 256, 64, 16, 2};
static const unsigned char high_priority_queue_mask =  0x00;

#define NET_RX_QUEUE_MASK 0x38
#define NAPI_QUEUE_MASK NET_RX_QUEUE_MASK // Rx, only Rx-exclusive port can enable NAPI
#define NONSTOP_QUEUE_MASK 0xF0 // Rx, for convenience, queue 0,1,2,3 are non-stop
#define NONSTOP_QUEUE_MASK_32 0xF0F0F0F0

#define CLDMA_CG_POLL 6
#define CLDMA_ACTIVE_T 20
#define BOOT_TIMER_ON 10
#define BOOT_TIMER_HS1 (30)//(10+5)

#define TAG "mcd"

#define IS_PASS_SKB(md,qno) ((md->md_state!=EXCEPTION || md->ex_stage!=EX_INIT_DONE) &&((1<<qno) & NET_RX_QUEUE_MASK)) 
/*
 * do NOT add any static data, data should be in modem's instance
 */

static void cldma_dump_gpd_ring(int md_id, dma_addr_t start, int size)
{
    // assume TGPD and RGPD's "next" pointers use the same offset
    struct cldma_tgpd *curr = (struct cldma_tgpd *)phys_to_virt(start);
    int i, *tmp;
    printk("[CCCI%d/CLDMA] gpd starts from 0x%x\n",md_id+1, (unsigned int)start);
    /*
     * virtual address get from dma_pool_alloc is not equal to phys_to_virt.
     * e.g. dma_pool_alloca returns 0xFFDFF00, and phys_to_virt will return 0xDF364000 for the same
     * DMA address. therefore we can't compare gpd address with @start to exit loop.
     */
    for(i=0; i<size; i++) {
        tmp = (int *) curr;
        printk("[CCCI%d/CLDMA] 0x%p: %X %X %X %X\n",md_id+1, curr, *tmp, *(tmp+1), *(tmp+2), *(tmp+3));
        curr = (struct cldma_tgpd *)phys_to_virt(curr->next_gpd_ptr);
    }
}

static void cldma_dump_all_gpd(struct ccci_modem *md)
{
    int i;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    struct ccci_request *req = NULL;
    for(i=0; i<QUEUE_LEN(md_ctrl->txq); i++) {
        // use GPD's pointer to traverse
        printk("[CCCI%d/CLDMA] dump txq %d GPD\n", md->index+1, i);
        req = list_entry(md_ctrl->txq[i].tr_ring, struct ccci_request, entry);
        cldma_dump_gpd_ring(md->index, req->gpd_addr, tx_queue_buffer_number[i]);
#if 0 // UT code
        // use request's link head to traverse
        printk("[CLDMA] dump txq %d request\n", i);
        list_for_each_entry(req, md_ctrl->txq[i].tr_ring, entry) { // due to we do NOT have an extra head, this will miss the first request
            printk("[CLDMA] %p (%x->%x)\n", req->gpd,
                req->gpd_addr, ((struct cldma_tgpd *)req->gpd)->next_gpd_ptr);
        }
#endif
    }
    for(i=0; i<QUEUE_LEN(md_ctrl->rxq); i++) {
        // use GPD's pointer to traverse
        printk("[CCCI%d/CLDMA] dump rxq %d GPD\n", md->index+1, i);
        req = list_entry(md_ctrl->rxq[i].tr_ring, struct ccci_request, entry);
        cldma_dump_gpd_ring(md->index,req->gpd_addr, rx_queue_buffer_number[i]);
#if 0 // UT code
        // use request's link head to traverse
        printk("[CLDMA] dump rxq %d request\n", i);
        list_for_each_entry(req, md_ctrl->rxq[i].tr_ring, entry) {
            printk("[CLDMA] %p/%p (%x->%x)\n", req->gpd, req->skb,
                req->gpd_addr, ((struct cldma_rgpd *)req->gpd)->next_gpd_ptr);
        }
#endif
    }
}
#if TRAFFIC_MONITOR_INTERVAL
void md_cd_traffic_monitor_func(unsigned long data)
{
    struct ccci_modem *md = (struct ccci_modem *)data;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    struct ccci_port *port;
    unsigned long long port_full = 0;
    unsigned int i; // hardcode, port number should not be larger than 64
    u64 ts_nsec[CLDMA_TXQ_NUM];
    unsigned long rem_nsec[CLDMA_TXQ_NUM];
    u64 ts_nsec_start[CLDMA_TXQ_NUM];
    unsigned long rem_nsec_start[CLDMA_TXQ_NUM];
    u64 ts_nsec_end[CLDMA_TXQ_NUM];
    unsigned long rem_nsec_end[CLDMA_TXQ_NUM];

    for(i=0; i<md->port_number; i++) {
		port = md->ports + i;
		if(port->flags & PORT_F_RX_FULLED)
			port_full |= (1<<i);
		if(port->tx_busy_count!=0 || port->rx_busy_count!=0) {
			CCCI_INF_MSG(md->index, TAG, "port %s busy count %d/%d\n", port->name, 
				port->tx_busy_count, port->rx_busy_count);
			port->tx_busy_count = 0;
			port->rx_busy_count = 0;
		}
		if(port->ops->dump_info){
			port->ops->dump_info(port,0);
		}
	}
    for(i=0; i<QUEUE_LEN(md_ctrl->txq); i++) {
        if(md_ctrl->txq[i].busy_count != 0) {
            CCCI_INF_MSG(md->index, TAG, "Txq%d busy count %d\n", i, md_ctrl->txq[i].busy_count);
            md_ctrl->txq[i].busy_count = 0;
        }
    }
#ifdef ENABLE_CLDMA_TIMER
    CCCI_INF_MSG(md->index, TAG, "traffic(tx_timer): [3]%llu %llu, [4]%llu %llu, [5]%llu %llu\n",
        md_ctrl->tx_done_timer_last_start_time[3],md_ctrl->cldma_tx_timeout_timer_end[3],
        md_ctrl->tx_done_timer_last_start_time[4],md_ctrl->cldma_tx_timeout_timer_end[4],
        md_ctrl->tx_done_timer_last_start_time[5],md_ctrl->cldma_tx_timeout_timer_end[5]
    );

    md_cd_lock_cldma_clock_src(1);
    CCCI_INF_MSG(md->index, TAG, "traffic(tx_done_timer): CLDMA_AP_L2TIMR0=0x%x   [3]%d %llu, [4]%d %llu, [5]%d %llu\n",
        cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMR0),
        md_ctrl->tx_done_timer_last_count[3], md_ctrl->tx_done_timer_last_start_time[3],
        md_ctrl->tx_done_timer_last_count[4], md_ctrl->tx_done_timer_last_start_time[4],
        md_ctrl->tx_done_timer_last_count[5], md_ctrl->tx_done_timer_last_start_time[5]
    );
    md_cd_lock_cldma_clock_src(0);
#endif

    CCCI_INF_MSG(md->index, TAG, "traffic: (%d/%llx)(Tx(%x): %d,%d,%d,%d,%d,%d,%d,%d)(Rx(%x): %d,%d,%d,%d,%d,%d,%d,%d)\n", 
        md->md_state, port_full,
        md_ctrl->txq_active,
        md_ctrl->tx_traffic_monitor[0], md_ctrl->tx_traffic_monitor[1],
        md_ctrl->tx_traffic_monitor[2], md_ctrl->tx_traffic_monitor[3],
        md_ctrl->tx_traffic_monitor[4], md_ctrl->tx_traffic_monitor[5],
        md_ctrl->tx_traffic_monitor[6], md_ctrl->tx_traffic_monitor[7],
        md_ctrl->rxq_active,
        md_ctrl->rx_traffic_monitor[0], md_ctrl->rx_traffic_monitor[1],
        md_ctrl->rx_traffic_monitor[2], md_ctrl->rx_traffic_monitor[3],
        md_ctrl->rx_traffic_monitor[4], md_ctrl->rx_traffic_monitor[5],
        md_ctrl->rx_traffic_monitor[6], md_ctrl->rx_traffic_monitor[7]);
    CCCI_INF_MSG(md->index, TAG, "traffic(tx_pre):%d,%d,%d,%d,%d,%d,%d,%d\n",
        md_ctrl->tx_pre_traffic_monitor[0], md_ctrl->tx_pre_traffic_monitor[1],
        md_ctrl->tx_pre_traffic_monitor[2], md_ctrl->tx_pre_traffic_monitor[3],
        md_ctrl->tx_pre_traffic_monitor[4], md_ctrl->tx_pre_traffic_monitor[5],
        md_ctrl->tx_pre_traffic_monitor[6], md_ctrl->tx_pre_traffic_monitor[7]);

	CCCI_INF_MSG(md->index, TAG, "traffic(ch): tx:[%d]%ld, [%d]%ld, [%d]%ld rx:[%d]%ld, [%d]%ld, [%d]%ld\n", 
	    CCCI_PCM_TX,md_ctrl->logic_ch_pkt_cnt[CCCI_PCM_TX],
	    CCCI_UART2_TX, md_ctrl->logic_ch_pkt_cnt[CCCI_UART2_TX],
	    CCCI_FS_TX, md_ctrl->logic_ch_pkt_cnt[CCCI_FS_TX],
	    CCCI_PCM_RX,md_ctrl->logic_ch_pkt_cnt[CCCI_PCM_RX],
	    CCCI_UART2_RX,md_ctrl->logic_ch_pkt_cnt[CCCI_UART2_RX],
	    CCCI_FS_RX,md_ctrl->logic_ch_pkt_cnt[CCCI_FS_RX]);    
    CCCI_INF_MSG(md->index, TAG, "traffic(net): tx: [%d]%ld %ld, [%d]%ld %ld, [%d]%ld %ld, rx:[%d]%ld, [%d]%ld, [%d]%ld\n", 
	    CCCI_CCMNI1_TX,md_ctrl->logic_ch_pkt_pre_cnt[CCCI_CCMNI1_TX], md_ctrl->logic_ch_pkt_cnt[CCCI_CCMNI1_TX], 
	    CCCI_CCMNI2_TX,md_ctrl->logic_ch_pkt_pre_cnt[CCCI_CCMNI2_TX],md_ctrl->logic_ch_pkt_cnt[CCCI_CCMNI2_TX],
	    CCCI_CCMNI3_TX,md_ctrl->logic_ch_pkt_pre_cnt[CCCI_CCMNI3_TX],md_ctrl->logic_ch_pkt_cnt[CCCI_CCMNI3_TX],
	    CCCI_CCMNI1_RX,md_ctrl->logic_ch_pkt_cnt[CCCI_CCMNI1_RX], 
	    CCCI_CCMNI2_RX,md_ctrl->logic_ch_pkt_cnt[CCCI_CCMNI2_RX],
	    CCCI_CCMNI3_RX,md_ctrl->logic_ch_pkt_cnt[CCCI_CCMNI3_RX]);
	return 0;

}
#endif

static void cldma_dump_packet_history(struct ccci_modem *md)
{
    int i;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    for(i=0; i<QUEUE_LEN(md_ctrl->txq); i++) {
        printk("[CCCI%d-DUMP]Current txq%d pos: tr_done=%x, tx_xmit=%x\n",md->index+1, i,(unsigned int)md_ctrl->txq[i].tr_done->gpd_addr, (unsigned int)md_ctrl->txq[i].tx_xmit->gpd_addr);
    }
    for(i=0; i<QUEUE_LEN(md_ctrl->rxq); i++) {
        printk("[CCCI%d-DUMP]Current rxq%d pos: tr_done=%x\n",md->index+1, i,(unsigned int)md_ctrl->rxq[i].tr_done->gpd_addr);
    }
    ccci_dump_log_history(md, 1, QUEUE_LEN(md_ctrl->txq),QUEUE_LEN(md_ctrl->rxq));
}

#if CHECKSUM_SIZE
static inline void caculate_checksum(char *address, char first_byte)
{
    int i;
    char sum = first_byte;
    for (i = 2 ; i < CHECKSUM_SIZE; i++)
        sum += *(address + i);
    *(address + 1) = 0xFF - sum;
}
#else
#define caculate_checksum(address, first_byte)
#endif

static int cldma_queue_broadcast_state(struct ccci_modem *md, MD_STATE state, DIRECTION dir, int index)
{
	struct ccci_port *port;
	int i, match=0;

	for(i=0;i<md->port_number;i++) {
		port = md->ports + i;
		// consider network data/ack queue design
		if(md->md_state==EXCEPTION)
			match = dir==OUT?index==port->txq_exp_index:index==port->rxq_exp_index;
		else
			match = dir==OUT?index==port->txq_index||index==(port->txq_exp_index&0x0F):index==port->rxq_index;
		if(match && port->ops->md_state_notice) {
			port->ops->md_state_notice(port, state);
		}
	}
	return 0;
}

// this function may be called from both workqueue and softirq (NAPI)
static int cldma_rx_collect(struct md_cd_queue *queue, int budget, int blocking, int *result, unsigned long long *rxbytes)
{
    struct ccci_modem *md = queue->modem;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    struct ccci_request *req;
    struct cldma_rgpd *rgpd;
	struct ccci_request *new_req = NULL;
    struct ccci_header ccci_h;
    struct sk_buff *skb;
#ifdef CLDMA_TRACE
    unsigned long long time2=0;
    unsigned long long req_alloc_time=0;
    unsigned long long port_recv_time=0;
    unsigned long long skb_alloc_time=0;
    unsigned long long total_handle_time=0;
#endif
    int ret=0, count=0,qno=queue->index;
	unsigned long long skb_bytes = 0;
    *result = UNDER_BUDGET;
    *rxbytes =0;

    // find all done RGPD, we didn't use any lock here, be careful with md_cd_clear_queue()
    while(1) { // not hardware own
        #ifdef CLDMA_TRACE
        total_handle_time = sched_clock();
        #endif
        qno=queue->index;
        req = queue->tr_done;
        rgpd = (struct cldma_rgpd *)req->gpd;
        if(unlikely(!req->skb)) {
            // check this first, as tr_done should remain where we failed to allocate skb
            CCCI_ERR_MSG(md->index, TAG, "found a hole on q%d, try refill and move forward\n", queue->index);            
            goto fill_and_move;
        }
        if((rgpd->gpd_flags&0x1) != 0) {
            break;
        }
        if(unlikely(req->skb->len!=0)) {
            // should never happen
            CCCI_ERR_MSG(md->index, TAG, "reuse skb %p with len %d\n", req->skb, req->skb->len);
            break;
        }
        // allocate a new wrapper, do nothing if this failed, just wait someone to collect this queue again, if lucky enough
        req = queue->tr_done;
        if(!IS_PASS_SKB(md,qno)){
            #ifdef CLDMA_TRACE
            time2= sched_clock();
            #endif
            new_req = ccci_alloc_req(IN, -1, blocking, 0);
            #ifdef CLDMA_TRACE
            time2 = sched_clock()-time2;
            req_alloc_time = time2;
            #endif
            if(unlikely(!new_req)) {
                CCCI_ERR_MSG(md->index, TAG, "alloc req fail on q%d\n", queue->index);
                *result = NO_REQ;
                #ifdef CLDMA_TRACE
                trace_cldma_error(queue->index,(0xFF), NO_SKB,__LINE__);
                #endif
                break;
            }
        }

        skb_bytes =skb_data_size(req->skb);
        // update skb
        dma_unmap_single(&md->plat_dev->dev, req->data_buffer_ptr_saved, skb_bytes, DMA_FROM_DEVICE);
        skb_put(req->skb, rgpd->data_buff_len);
        ccci_h = *((struct ccci_header *)req->skb->data);
        
        if(IS_PASS_SKB(md,qno)){
            skb = req->skb;
        }else{
            skb = req->skb;
            new_req->skb = req->skb;
            INIT_LIST_HEAD(&new_req->entry); // as port will run list_del
        }
        
        if(atomic_cmpxchg(&md->wakeup_src, 1, 0) == 1)
            CCCI_INF_MSG(md->index, TAG, "CLDMA_MD wakeup source:(%d/%d)\n", queue->index, ccci_h.channel);
        CCCI_DBG_MSG(md->index, TAG, "recv Rx msg (%x %x %x %x) rxq=%d len=%d\n",
            ccci_h.data[0], ccci_h.data[1], *(((u32 *)&ccci_h)+2), ccci_h.reserved, queue->index, rgpd->data_buff_len);
		// Update RX pkt count
		if((ccci_h.channel & 0xFF) < CCCI_MAX_CH_NUM) {
			md_ctrl->logic_ch_pkt_cnt[ccci_h.channel&0xFF]++;
		}
        
        if(unlikely(md->md_state == GATED||md->md_state == INVALID)){
            CCCI_ERR_MSG(md->index, CORE, "md_state=%d,so dropped Rx msg (%x %x %x %x)\n",md->md_state,
                ccci_h.data[0], ccci_h.data[1], *(((u32 *)&ccci_h)+2), ccci_h.reserved);
            ret = -CCCI_ERR_DROP_PACKET;
            if(IS_PASS_SKB(md,qno)){
                dev_kfree_skb_any(skb);
            }else{
                list_del(&new_req->entry); 
        		new_req->policy = RECYCLE;
        		ccci_free_req(new_req);
            }
        }else{
            #ifdef CLDMA_TRACE
            time2 = sched_clock();
            #endif
            if(IS_PASS_SKB(md,qno)){
                ret = ccci_port_recv_skb(md, skb);
            }else{
                ret = ccci_port_recv_request(md, new_req);
            }
            CCCI_DBG_MSG(md->index, TAG, "Rx port recv req ret=%d\n", ret);

            #ifdef CLDMA_TRACE
            port_recv_time = (sched_clock()-time2);
            #endif
        }
                

        if(ret>=0 || ret==-CCCI_ERR_DROP_PACKET) {
            ccci_dump_log_add(md,IN,(int)queue->index, &ccci_h,(ret>=0?0:1));
            *rxbytes+=skb_bytes;
fill_and_move:
            #ifdef CLDMA_TRACE
            time2 = sched_clock();
            #endif

            // allocate a new skb and change skb pointer
            req->skb = ccci_alloc_skb(rx_queue_buffer_size[queue->index], blocking);

            #ifdef CLDMA_TRACE
            time2 = sched_clock()-time2;
            skb_alloc_time = time2;
            #endif
            
            if(likely(req->skb)) {
                req->data_buffer_ptr_saved = dma_map_single(&md->plat_dev->dev, req->skb->data, skb_data_size(req->skb), DMA_FROM_DEVICE);
                rgpd->data_buff_bd_ptr = (u32)(req->data_buffer_ptr_saved);
                // checksum of GPD
                caculate_checksum((char *)rgpd, 0x81);
                // update GPD
                cldma_write8(&rgpd->gpd_flags, 0, 0x81);
                // step forward
                req = list_entry(req->entry.next, struct ccci_request, entry);
                rgpd = (struct cldma_rgpd *)req->gpd;
                queue->tr_done = req;
#if TRAFFIC_MONITOR_INTERVAL
                md_ctrl->rx_traffic_monitor[queue->index]++;
#endif
            } else {
                /*
                 * low memory, just stop and if lucky enough, some one collect it again (most likely
                 * NAPI), the fill_and_move should work.
                 */
                CCCI_ERR_MSG(md->index, TAG, "alloc skb fail on q%d\n", queue->index);
                *result = NO_SKB;
                #ifdef CLDMA_TRACE
                trace_cldma_error(queue->index,(ccci_h.channel & 0xFF), NO_SKB,__LINE__);
                #endif
                break;
            }
        } else {
            // undo skb, as it remains in buffer and will be handled later
            if(IS_PASS_SKB(md,qno)){
                skb_reset_tail_pointer(skb);
            }else{
                new_req->skb->len = 0;
                skb_reset_tail_pointer(new_req->skb);
                // free the wrapper
                list_del(&new_req->entry);
                new_req->policy = NOOP;
                ccci_free_req(new_req);
            }
            *result = PORT_REFUSE;
            
            #ifdef CLDMA_TRACE
            trace_cldma_error(queue->index,(ccci_h.channel & 0xFF), PORT_REFUSE,__LINE__);
            #endif
            break;
        }
        // check budget
        if(count++ >= budget) {
           *result = REACH_BUDGET;
			//break;
        }

        #ifdef CLDMA_TRACE
        total_handle_time=(sched_clock()-total_handle_time);
        trace_cldma_rx(queue->index,(ccci_h.channel & 0xFF), req_alloc_time, port_recv_time,skb_alloc_time,total_handle_time,skb_bytes);
        #endif
    }
    
    /*
     * do not use if(count == RING_BUFFER_SIZE) to resume Rx queue.
     * resume Rx queue every time. we may not handle all RX ring buffer at one time due to
     * user can refuse to receive patckets. so when a queue is stopped after it consumes all
     * GPD, there is a chance that "count" never reaches ring buffer size and the queue is stopped
     * permanentely.
     *
     * resume after all RGPD handled also makes budget useless when it is less than ring buffer length.
     */
    // if result == 0, that means all skb have been handled
    CCCI_DBG_MSG(md->index, TAG, "CLDMA Rxq%d collected, result=%d, count=%d\n", queue->index, *result, count);
    return count;
}

static void cldma_rx_done(struct work_struct *work)
{
    struct md_cd_queue *queue = container_of(work, struct md_cd_queue, cldma_work);
    struct ccci_modem *md = queue->modem;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    int result;
    int count;
    unsigned long long rx_bytes=0;
    unsigned long flags;
//Trace start
#ifdef CLDMA_TRACE
    unsigned long long total_time=0;
    unsigned long long rx_interal;
    static unsigned long long last_leave_time[CLDMA_RXQ_NUM]={0}; 
    static unsigned long long sample_time[CLDMA_RXQ_NUM]={0}; 
    static unsigned long long sample_bytes[CLDMA_RXQ_NUM]={0};
    static unsigned long long sample_time_last[CLDMA_RXQ_NUM]={0}; 
    static unsigned long long sample_bytes_last[CLDMA_RXQ_NUM]={0}; 
    if(last_leave_time[queue->index] == 0){
        rx_interal=0;
    }else{
        rx_interal=sched_clock()-last_leave_time[queue->index];
    }
    total_time = sched_clock();
#endif
	count=cldma_rx_collect(queue, queue->budget, !IS_PASS_SKB(md,queue->index), &result,&rx_bytes);
//Trace end
#ifdef CLDMA_TRACE    
    if(count){
        total_time=sched_clock()-total_time;
        sample_time[queue->index] += (total_time+rx_interal);
        sample_bytes[queue->index]+= rx_bytes;
        if(sample_time[queue->index]>=trace_sample_time){
            sample_time_last[queue->index]=sample_time[queue->index];
            sample_bytes_last[queue->index]=sample_bytes[queue->index];
            sample_time[queue->index]=0;
            sample_bytes[queue->index]=0;            
        }        
        trace_cldma_rx_done(queue->index, rx_interal, total_time,count,rx_bytes,sample_time_last[queue->index],sample_bytes_last[queue->index]);
        last_leave_time[queue->index]=sched_clock();
    }else{
        trace_cldma_error(queue->index, -1,0,__LINE__);
    }
#endif

    md_cd_lock_cldma_clock_src(1);
    spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
    if(md_ctrl->rxq_active & (1<<queue->index)) {
        // resume Rx queue
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_RESUME_CMD, CLDMA_BM_ALL_QUEUE&(1<<queue->index));
        cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_RESUME_CMD); // dummy read
        // enable RX_DONE interrupt
        cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMCR0, CLDMA_BM_ALL_QUEUE&(1<<queue->index));
    }
    spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
    md_cd_lock_cldma_clock_src(0);
}

// this function may be called from both workqueue and ISR (timer)
static int cldma_tx_collect(struct md_cd_queue *queue, int budget, int blocking, int *result)
{
    struct ccci_modem *md = queue->modem;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    unsigned long flags;
    struct ccci_request *req;
    struct cldma_tgpd *tgpd;
    struct ccci_header *ccci_h;
    int count = 0;
    struct sk_buff *skb_free;
    DATA_POLICY skb_free_p;

    while(1) {
        spin_lock_irqsave(&queue->ring_lock, flags);
        req = queue->tr_done;
        tgpd = (struct cldma_tgpd *)req->gpd;
        if(!((tgpd->gpd_flags&0x1) == 0 && req->skb)) {
            spin_unlock_irqrestore(&queue->ring_lock, flags);
            break;
        }
	// restore IOC setting
        if(!IS_PASS_SKB(md,queue->index) && (req->ioc_override & 0x80)) {
                if(req->ioc_override & 0x1)
                        tgpd->gpd_flags |= 0x80;
                else
                        tgpd->gpd_flags &= 0x7F;
                CCCI_INF_MSG(md->index, TAG, "TX_collect: qno %d, req->ioc_override=0x%x,tgpd->gpd_flags=0x%x\n", queue->index,req->ioc_override,tgpd->gpd_flags);
        }

        tgpd->non_used = 2;
        // update counter
        queue->free_slot++;
        dma_unmap_single(&md->plat_dev->dev, req->data_buffer_ptr_saved, req->skb->len, DMA_TO_DEVICE);
        ccci_h = (struct ccci_header *)req->skb->data;
        CCCI_DBG_MSG(md->index, TAG, "harvest Tx msg (%x %x %x %x) txq=%d len=%d\n",
            ccci_h->data[0], ccci_h->data[1], *(((u32 *)ccci_h)+2), ccci_h->reserved, queue->index, tgpd->data_buff_len);
		// Update tx pkt count
		if((ccci_h->channel & 0xFF) < CCCI_MAX_CH_NUM) {
			md_ctrl->logic_ch_pkt_cnt[ccci_h->channel&0xFF]++;
		}
        // free skb
        skb_free = req->skb;
        skb_free_p = req->policy;
        req->skb = NULL;
        count++;
        // step forward
        req = list_entry(req->entry.next, struct ccci_request, entry);
        tgpd = (struct cldma_tgpd *)req->gpd;
        queue->tr_done = req;
		if(likely(md->capability & MODEM_CAP_TXBUSY_STOP)) 
			cldma_queue_broadcast_state(md, TX_IRQ, OUT, queue->index);
        spin_unlock_irqrestore(&queue->ring_lock, flags);
        /*
         * After enabled NAPI, when free skb, cosume_skb() will eventually called nf_nat_cleanup_conntrack(),
         * which will call spin_unlock_bh() to let softirq to run. so there is a chance a Rx softirq is triggered (cldma_rx_collect)
         * and if it's a TCP packet, it will send ACK -- another Tx is scheduled which will require queue->ring_lock,
         * cause a deadlock!
         *
         * This should not be an issue any more, after we start using dev_kfree_skb_any() instead of dev_kfree_skb().
         */
        ccci_free_skb(skb_free, skb_free_p);
#if TRAFFIC_MONITOR_INTERVAL
        md_ctrl->tx_traffic_monitor[queue->index]++;
#endif
    }
	if(count) {
        wake_up_nr(&queue->req_wq, count);
	}
	return count;
}

static enum hrtimer_restart cldma_tx_done_timer(struct hrtimer *timer)
{
	struct md_cd_queue *queue = container_of(timer, struct md_cd_queue, cldma_poll_timer);
	struct ccci_modem *md = queue->modem;
	struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
	int result, count,ret;
	unsigned long flags;
    // Trace start
#ifdef CLDMA_TRACE    
    unsigned long long total_time=0;
    unsigned long long tx_interal;
    static unsigned long long leave_time[CLDMA_TXQ_NUM]={0}; 
    leave_time[queue->index]= sched_clock();
    if(leave_time[queue->index] == 0){
        tx_interal=0;
    }else{
        tx_interal=sched_clock()-leave_time[queue->index];
    }
    total_time = sched_clock();
#endif
    md_ctrl->tx_done_timer_last_start_time[queue->index] = local_clock();
    count = cldma_tx_collect(queue, 0, 0, &result);
    md_ctrl->tx_done_timer_last_count[queue->index] = count;

    //Trace end
#ifdef CLDMA_TRACE
    total_time = sched_clock()-total_time;
    if(count){
        trace_cldma_tx_done(queue->index, tx_interal, total_time,count);
        leave_time[queue->index]=sched_clock();
    }else{
        trace_cldma_error(queue->index, -1,0,__LINE__);
    }
#endif 

    if(IS_PASS_SKB(md,queue->index)){
        CCCI_DBG_MSG(md->index, TAG, "qno %d, cldma_tx_done_timer: %d,%llu,cnt=%d\n", queue->index,count,md_ctrl->tx_done_timer_last_start_time[queue->index],count);
    }

    if(count) {        
        ret= HRTIMER_RESTART;
    } else {
    // enable TX_DONE interrupt
        md_cd_lock_cldma_clock_src(1);
        spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
        if(md_ctrl->txq_active & (1<<queue->index))
            cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMCR0, CLDMA_BM_ALL_QUEUE&(1<<queue->index));
        spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
        md_cd_lock_cldma_clock_src(0);
        ret= HRTIMER_NORESTART;
    }

    return ret;
}

static void cldma_tx_done(struct work_struct *work)
{
	struct md_cd_queue *queue = container_of(work, struct md_cd_queue, cldma_work);
	struct ccci_modem *md = queue->modem;
	struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
	int result,count;
	unsigned long flags;
// Trace start
#ifdef CLDMA_TRACE    
    unsigned long long total_time=0;
    unsigned long long tx_interal;
    static unsigned long long leave_time[CLDMA_TXQ_NUM]={0}; 
    leave_time[queue->index]= sched_clock();
    if(leave_time[queue->index] == 0){
        tx_interal=0;
    }else{
        tx_interal=sched_clock()-leave_time[queue->index];
    }
    total_time = sched_clock();
#endif
	count=cldma_tx_collect(queue, 0, 0, &result);

//Trace end
#ifdef CLDMA_TRACE
    total_time = sched_clock()-total_time;
    if(count){
        trace_cldma_tx_done(queue->index, tx_interal, total_time,count);
        leave_time[queue->index]=sched_clock();
    }else{
        trace_cldma_error(queue->index, -1,0,__LINE__);
    }
#endif

    if(IS_PASS_SKB(md,queue->index)){
        CCCI_DBG_MSG(md->index, TAG, "qno %d, cldma_tx_done: %d,%llu, count=%d\n", queue->index,count,md_ctrl->tx_done_timer_last_start_time[queue->index],count);
    }

	// enable TX_DONE interrupt
	md_cd_lock_cldma_clock_src(1);
	spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
	if(md_ctrl->txq_active & (1<<queue->index))
		cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMCR0, CLDMA_BM_ALL_QUEUE&(1<<queue->index));
	spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
	md_cd_lock_cldma_clock_src(0);
}


static void cldma_rx_queue_init(struct md_cd_queue *queue)
{
    int i;
    struct ccci_request *req;
    struct cldma_rgpd *gpd=NULL, *prev_gpd=NULL;
    struct ccci_modem *md = queue->modem;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    for(i=0; i<rx_queue_buffer_number[queue->index]; i++) {
	req = ccci_alloc_req(IN, rx_queue_buffer_size[queue->index], 1, 0);
        req->gpd = dma_pool_alloc(md_ctrl->rgpd_dmapool, GFP_KERNEL, &req->gpd_addr);
        gpd = (struct cldma_rgpd *)req->gpd;
        memset(gpd, 0, sizeof(struct cldma_rgpd));
        req->data_buffer_ptr_saved = dma_map_single(&md->plat_dev->dev, req->skb->data, skb_data_size(req->skb), DMA_FROM_DEVICE);
        gpd->data_buff_bd_ptr = (u32)(req->data_buffer_ptr_saved);
        gpd->data_allow_len = rx_queue_buffer_size[queue->index];
        gpd->gpd_flags = 0x81; // IOC|HWO
        if(i==0) {
            queue->tr_done = req;
            queue->tr_ring = &req->entry;
            INIT_LIST_HEAD(queue->tr_ring); // check ccci_request_struct_init for why we init here
        } else {
            prev_gpd->next_gpd_ptr = req->gpd_addr;
            caculate_checksum((char *)prev_gpd, 0x81);
            list_add_tail(&req->entry, queue->tr_ring);
        }
        prev_gpd = gpd;
    }
    gpd->next_gpd_ptr = queue->tr_done->gpd_addr;
    caculate_checksum((char *)gpd, 0x81);

    /*
     * we hope work item of different CLDMA queue can work concurrently, but work items of the same
     * CLDMA queue must be work sequentially as wo didn't implement any lock in rx_done or tx_done.
     */
    queue->worker = alloc_workqueue("md%d_rx%d_worker", WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1, md->index+1, queue->index);
    INIT_WORK(&queue->cldma_work, cldma_rx_done);
    CCCI_DBG_MSG(md->index, TAG, "rxq%d work=%p\n", queue->index, &queue->cldma_work);
}

static void cldma_tx_queue_init(struct md_cd_queue *queue)
{
    int i;
    struct ccci_request *req;
    struct cldma_tgpd *gpd=NULL, *prev_gpd=NULL;
    struct ccci_modem *md = queue->modem;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    for(i=0; i<tx_queue_buffer_number[queue->index]; i++) {
        req = ccci_alloc_req(OUT, -1, 1, 0);
        req->gpd = dma_pool_alloc(md_ctrl->tgpd_dmapool, GFP_KERNEL, &req->gpd_addr);
        gpd = (struct cldma_tgpd *)req->gpd;
        memset(gpd, 0, sizeof(struct cldma_tgpd));
        // network needs we free skb as soon as possible as they are tracking skb completion
		if(i%tx_ioc_interval[queue->index] == 0)
        gpd->gpd_flags = 0x80; // IOC
        if(i==0) {
            queue->tr_done = req;
            queue->tx_xmit = req;
            queue->tr_ring = &req->entry;
            INIT_LIST_HEAD(queue->tr_ring);
        } else {
            prev_gpd->next_gpd_ptr = req->gpd_addr;
            list_add_tail(&req->entry, queue->tr_ring);
        }
        prev_gpd = gpd;
    }
    gpd->next_gpd_ptr = queue->tr_done->gpd_addr;

    queue->worker = alloc_workqueue("md%d_tx%d_worker", WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1, md->index+1, queue->index);
    INIT_WORK(&queue->cldma_work, cldma_tx_done);
    CCCI_DBG_MSG(md->index, TAG, "txq%d work=%p\n", queue->index, &queue->cldma_work);
    hrtimer_init(&queue->cldma_poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    queue->cldma_poll_timer.function = cldma_tx_done_timer;
}

#ifdef ENABLE_CLDMA_TIMER
static void cldma_timeout_timer_func(unsigned long data)
{
    struct ccci_modem *md = (struct ccci_modem *)data;
    struct ccci_port *port;
    unsigned long long port_full=0, i; // hardcode, port number should not larger than 64

    if(MD_IN_DEBUG(md))
        return;
    
    for(i=0; i<md->port_number; i++) {
        port = md->ports + i;
        if(port->flags & PORT_F_RX_FULLED)
            port_full |= (1<<i);
    }
    CCCI_ERR_MSG(md->index, TAG, "CLDMA no response for %d seconds, ports=%llx\n", CLDMA_ACTIVE_T, port_full);
    md_cd_traffic_monitor_func((unsigned long)md);

    md->ops->dump_info(md, DUMP_FLAG_CLDMA, NULL, 0);
    CCCI_ERR_MSG(md->index, TAG, "CLDMA no response, force assert md by CCIF_INTERRUPT\n");
    md->ops->force_assert(md, CCIF_INTERRUPT);
}
#endif
static void cldma_irq_work_cb(struct ccci_modem *md)
{
    int i, ret;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    unsigned int L2TIMR0, L2RIMR0, L2TISAR0, L2RISAR0;
    unsigned int L3TIMR0, L3RIMR0, L3TISAR0, L3RISAR0;

    md_cd_lock_cldma_clock_src(1);
    // get L2 interrupt status
    L2TISAR0 = cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0);
    L2RISAR0 = cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2RISAR0);
    L2TIMR0 = cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMR0);
    L2RIMR0 = cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMR0);
    // get L3 interrupt status
    L3TISAR0 = cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TISAR0);
    L3RISAR0 = cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RISAR0);
    L3TIMR0 = cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TIMR0);
    L3RIMR0 = cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RIMR0);

    if(atomic_read(&md->wakeup_src)== 1)
        CCCI_INF_MSG(md->index, TAG, "wake up by CLDMA_MD L2(%x/%x) L3(%x/%x)!\n", L2TISAR0, L2RISAR0, L3TISAR0, L3RISAR0);
    else
        CCCI_DBG_MSG(md->index, TAG, "CLDMA IRQ L2(%x/%x) L3(%x/%x)!\n", L2TISAR0, L2RISAR0, L3TISAR0, L3RISAR0);

    L2TISAR0 &= (~L2TIMR0);
    L2RISAR0 &= (~L2RIMR0);

    L3TISAR0 &= (~L3TIMR0);
    L3RISAR0 &= (~L3RIMR0);

    if(L2TISAR0 & CLDMA_BM_INT_ERROR) {
        // TODO:
    }
    if(L2RISAR0 & CLDMA_BM_INT_ERROR) {
        // TODO:
    }
	if(unlikely(!(L2RISAR0&CLDMA_BM_INT_DONE) && !(L2TISAR0&CLDMA_BM_INT_DONE))) {
		CCCI_ERR_MSG(md->index, TAG, "no Tx or Rx, L2TISAR0=%X, L3TISAR0=%X, L2RISAR0=%X, L3RISAR0=%X, L2TIMR0=%X, L2RIMR0=%X, CODA_VERSION=%X\n",
				cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0),
				cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TISAR0),
				cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2RISAR0),
				cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RISAR0),
				cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMR0),
				cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMR0),
				cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_CLDMA_CODA_VERSION));
    } 
    // ack Tx interrupt
	if(L2TISAR0) {
        #ifdef CLDMA_TRACE
        trace_cldma_irq(CCCI_TRACE_TX_IRQ,(L2TISAR0 & CLDMA_BM_INT_DONE));
        #endif
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0, L2TISAR0);
        for(i=0; i<QUEUE_LEN(md_ctrl->txq); i++) {
            if(L2TISAR0 & CLDMA_BM_INT_DONE & (1<<i)) {
                #ifdef ENABLE_CLDMA_TIMER
                if(IS_PASS_SKB(md,i)){
                    md_ctrl->cldma_tx_timeout_timer_end[i]=local_clock();
                    ret=del_timer(&md_ctrl->cldma_tx_timeout_timer[i]);
                    CCCI_DBG_MSG(md->index, TAG, "qno%d del_timer %d ptr=0x%p\n", i,ret,&md_ctrl->cldma_tx_timeout_timer[i]);
                }
                #endif
                // disable TX_DONE interrupt
                cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMSR0, CLDMA_BM_ALL_QUEUE&(1<<i));
#ifdef CLDMA_TIMER_LOOP
                if(IS_PASS_SKB(md,i) && (hrtimer_active(&md_ctrl->txq[i].cldma_poll_timer)==0)){
                						
                            ret=hrtimer_start(&md_ctrl->txq[i].cldma_poll_timer, 
                                   ktime_set(0, CLDMA_TIMER_LOOP * 1000000),
                                   HRTIMER_MODE_REL);
                            CCCI_DBG_MSG(md->index, TAG, "qno%d hrtimer_start=%d\n", i,ret);
                }else{
                        ret = queue_work(md_ctrl->txq[i].worker, &md_ctrl->txq[i].cldma_work);
                        CCCI_DBG_MSG(md->index, TAG, "qno%d queue_work=%d\n", i,ret);
                }
#else
                ret = queue_work(md_ctrl->txq[i].worker, &md_ctrl->txq[i].cldma_work);
#endif
            }
        }
    }
    // ack Rx interrupt
	if(L2RISAR0) {
        #ifdef CLDMA_TRACE
        trace_cldma_irq(CCCI_TRACE_RX_IRQ,(L2RISAR0 & CLDMA_BM_INT_DONE));
        #endif
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2RISAR0, L2RISAR0);
        // clear MD2AP_PEER_WAKEUP when get RX_DONE
#ifdef MD_PEER_WAKEUP
        if(L2RISAR0 & CLDMA_BM_INT_DONE)
            cldma_write32(md_ctrl->md_peer_wakeup, 0, cldma_read32(md_ctrl->md_peer_wakeup, 0) & ~0x01);
#endif
#ifdef  ENABLE_CLDMA_AP_SIDE
        // clear IP busy register wake up cpu case
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_CLDMA_IP_BUSY, cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_CLDMA_IP_BUSY));		
#endif
        for(i=0; i<QUEUE_LEN(md_ctrl->rxq); i++) {
            if(L2RISAR0 & CLDMA_BM_INT_DONE & (1<<i)) {
                // disable RX_DONE interrupt
                cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMSR0, CLDMA_BM_ALL_QUEUE&(1<<i));
                if(md->md_state!=EXCEPTION && md_ctrl->rxq[i].napi_port) {
                    md_ctrl->rxq[i].napi_port->ops->md_state_notice(md_ctrl->rxq[i].napi_port, RX_IRQ);
                } else {
                    ret = queue_work(md_ctrl->rxq[i].worker, &md_ctrl->rxq[i].cldma_work);
                }
            }
        }
    }
    md_cd_lock_cldma_clock_src(0);
    enable_irq(md_ctrl->cldma_irq_id);

}
static irqreturn_t cldma_isr(int irq, void *data)
{
    struct ccci_modem *md = (struct ccci_modem *)data;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    CCCI_DBG_MSG(md->index, TAG, "CLDMA IRQ!\n");
    disable_irq_nosync(md_ctrl->cldma_irq_id);
#ifdef  ENABLE_CLDMA_AP_SIDE
    cldma_irq_work_cb(md);
#else
	queue_work(md_ctrl->cldma_irq_worker, &md_ctrl->cldma_irq_work);
#endif
    return IRQ_HANDLED;
}

static void cldma_irq_work(struct work_struct *work)
{
	struct md_cd_ctrl *md_ctrl = container_of(work, struct md_cd_ctrl, cldma_irq_work);
	struct ccci_modem *md = md_ctrl->modem;
    cldma_irq_work_cb(md);
}

static inline void cldma_stop(struct ccci_modem *md)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    int ret, count;
    unsigned long flags;
    int qno;
    CCCI_INF_MSG(md->index, TAG, "%s from %ps\n", __FUNCTION__, __builtin_return_address(0));
    spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
    // stop all Tx and Rx queues
    count = 0;
    md_ctrl->txq_active &= (~CLDMA_BM_ALL_QUEUE);
    do {
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_STOP_CMD, CLDMA_BM_ALL_QUEUE);
        cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_STOP_CMD); // dummy read
        ret = cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_STATUS);
		if((++count)%100000 == 0) {
            CCCI_INF_MSG(md->index, TAG, "stop Tx CLDMA, status=%x, count=%d\n", ret, count);
#if defined (CONFIG_MTK_AEE_FEATURE)
            aee_kernel_dal_show("stop Tx CLDMA failed.\n");
#endif
            CCCI_INF_MSG(md->index, KERN, "Dump MD EX log\n");
            ccci_mem_dump(md->index, md->smem_layout.ccci_exp_smem_base_vir, md->smem_layout.ccci_exp_dump_size);
            md_cd_dump_debug_register(md);
            cldma_dump_register(md);
	}
    } while(ret != 0);
    count = 0;
    md_ctrl->rxq_active &= (~CLDMA_BM_ALL_QUEUE);
    do {
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_STOP_CMD, CLDMA_BM_ALL_QUEUE);
        cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_STOP_CMD); // dummy read
        ret = cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_STATUS);
		if((++count)%100000 == 0) {
            CCCI_INF_MSG(md->index, TAG, "stop Rx CLDMA, status=%x, count=%d\n", ret, count);
#if defined (CONFIG_MTK_AEE_FEATURE)
            aee_kernel_dal_show("stop Rx CLDMA failed.\n");
#endif
            CCCI_INF_MSG(md->index, KERN, "Dump MD EX log\n");
            ccci_mem_dump(md->index, md->smem_layout.ccci_exp_smem_base_vir, md->smem_layout.ccci_exp_dump_size);
            md_cd_dump_debug_register(md);
            cldma_dump_register(md);
	}
    } while(ret != 0);
    // clear all L2 and L3 interrupts
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2RISAR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2RISAR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TISAR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TISAR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RISAR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RISAR1, CLDMA_BM_INT_ALL);
    // disable all L2 and L3 interrupts
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMSR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMSR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMSR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMSR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TIMSR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TIMSR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RIMSR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RIMSR1, CLDMA_BM_INT_ALL);
    for(qno=0;qno<CLDMA_TXQ_NUM;qno++){
    // stop timer
#ifdef ENABLE_CLDMA_TIMER
       del_timer(&md_ctrl->cldma_tx_timeout_timer[qno]);
#endif
    }
    spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
}

static inline void cldma_stop_for_ee(struct ccci_modem *md)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    int ret,count;
    unsigned long flags;

    CCCI_INF_MSG(md->index, TAG, "%s from %ps\n", __FUNCTION__, __builtin_return_address(0));
    spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
    // stop all Tx and Rx queues, but non-stop Rx ones
    count = 0;
    md_ctrl->txq_active &= (~CLDMA_BM_ALL_QUEUE);
    do {
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_STOP_CMD, CLDMA_BM_ALL_QUEUE);
        cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_STOP_CMD); // dummy read
        ret = cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_STATUS);
		if((++count)%100000 == 0) {
            CCCI_INF_MSG(md->index, TAG, "stop Tx CLDMA E, status=%x, count=%d\n", ret, count);
#if defined (CONFIG_MTK_AEE_FEATURE)
            aee_kernel_dal_show("stop Tx CLDMA failed for EE.\n");
#endif
            CCCI_INF_MSG(md->index, KERN, "Dump MD EX log\n");
            ccci_mem_dump(md->index, md->smem_layout.ccci_exp_smem_base_vir, md->smem_layout.ccci_exp_dump_size);
            md_cd_dump_debug_register(md);
            cldma_dump_register(md);
	}
    } while(ret != 0);
    count = 0;
    md_ctrl->rxq_active &= (~(CLDMA_BM_ALL_QUEUE&NONSTOP_QUEUE_MASK));
    do {
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_STOP_CMD, CLDMA_BM_ALL_QUEUE&NONSTOP_QUEUE_MASK);
        cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_STOP_CMD); // dummy read
        ret = cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_STATUS) & NONSTOP_QUEUE_MASK;
		if((++count)%100000 == 0) {
            CCCI_INF_MSG(md->index, TAG, "stop Rx CLDMA E, status=%x, count=%d\n", ret, count);
#if defined (CONFIG_MTK_AEE_FEATURE)
            aee_kernel_dal_show("stop Rx CLDMA failed for EE.\n");
#endif
            CCCI_INF_MSG(md->index, KERN, "Dump MD EX log\n");
            ccci_mem_dump(md->index, md->smem_layout.ccci_exp_smem_base_vir, md->smem_layout.ccci_exp_dump_size);
            md_cd_dump_debug_register(md);
            cldma_dump_register(md);
	}
    } while(ret != 0);
    // clear all L2 and L3 interrupts, but non-stop Rx ones
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2RISAR0, CLDMA_BM_INT_ALL&NONSTOP_QUEUE_MASK_32);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2RISAR1, CLDMA_BM_INT_ALL&NONSTOP_QUEUE_MASK_32);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TISAR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TISAR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RISAR0, CLDMA_BM_INT_ALL&NONSTOP_QUEUE_MASK_32);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RISAR1, CLDMA_BM_INT_ALL&NONSTOP_QUEUE_MASK_32);
    // disable all L2 and L3 interrupts, but non-stop Rx ones
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMSR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMSR1, CLDMA_BM_INT_ALL);
	cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMSR0, (CLDMA_BM_INT_DONE|CLDMA_BM_INT_ERROR)&NONSTOP_QUEUE_MASK_32);
	cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMSR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TIMSR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TIMSR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RIMSR0, CLDMA_BM_INT_ALL&NONSTOP_QUEUE_MASK_32);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RIMSR1, CLDMA_BM_INT_ALL&NONSTOP_QUEUE_MASK_32);
    
    spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
}

static inline void cldma_reset(struct ccci_modem *md)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
	volatile unsigned int SO_CFG;
    
    CCCI_INF_MSG(md->index, TAG, "%s from %ps\n", __FUNCTION__, __builtin_return_address(0));
    cldma_stop(md);
    // enable OUT DMA
    cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG, cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG)|0x01);
	SO_CFG = cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG);
	if (SO_CFG & 0x1 == 0)  // write function didn't work
	{
		CCCI_ERR_MSG(md->index, TAG, "Enable AP OUTCLDMA failed. Register can't be wrote. SO_CFG=0x%x\n", SO_CFG);
		cldma_dump_register(md);
		cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG, cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG)|0x01);
	}
    // wait RGPD write transaction repsonse
    cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG, cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG) | 0x4);
	SO_CFG = cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG);
	if (SO_CFG & 0x1 == 0)  // write function didn't work
	{
		CCCI_ERR_MSG(md->index, TAG, "Enable AP OUTCLDMA failed. Register can't be wrote. SO_CFG=0x%x\n", SO_CFG);
		cldma_dump_register(md);
		cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG, cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG)|0x05);
	}
    // enable SPLIT_EN
    cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_BUS_CFG, cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_BUS_CFG)|0x02);
    // set high priority queue
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_HPQR, high_priority_queue_mask);
    // TODO: traffic control value
    // set checksum
    switch (CHECKSUM_SIZE) {
    case 0:
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_CHECKSUM_CHANNEL_ENABLE, 0);
        cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CHECKSUM_CHANNEL_ENABLE, 0);
        break;
    case 12:
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_CHECKSUM_CHANNEL_ENABLE, CLDMA_BM_ALL_QUEUE);
        cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CHECKSUM_CHANNEL_ENABLE, CLDMA_BM_ALL_QUEUE);
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_CFG, cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_CFG)&~0x10);
        cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG, cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG)&~0x10);
        break;
    case 16:
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_CHECKSUM_CHANNEL_ENABLE, CLDMA_BM_ALL_QUEUE);
        cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CHECKSUM_CHANNEL_ENABLE, CLDMA_BM_ALL_QUEUE);
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_CFG, cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_CFG)|0x10);
        cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG, cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG)|0x10);
        break;
    }
    // TODO: need to select CLDMA mode in CFG?
    // TODO: enable debug ID?
}

static inline void cldma_start(struct ccci_modem *md)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    int i;
    unsigned long flags;
    
    CCCI_INF_MSG(md->index, TAG, "%s from %ps\n", __FUNCTION__, __builtin_return_address(0));
    spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
    // set start address
    for(i=0; i<QUEUE_LEN(md_ctrl->txq); i++) {
        cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_TQSAR(md_ctrl->txq[i].index), md_ctrl->txq[i].tr_done->gpd_addr);
#ifdef ENABLE_CLDMA_AP_SIDE
        cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_TQSABAK(md_ctrl->txq[i].index), md_ctrl->txq[i].tr_done->gpd_addr);
#endif
    }
    for(i=0; i<QUEUE_LEN(md_ctrl->rxq); i++) {
        cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_RQSAR(md_ctrl->rxq[i].index), md_ctrl->rxq[i].tr_done->gpd_addr);
    }
    wmb();
    // start all Tx and Rx queues
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_START_CMD, CLDMA_BM_ALL_QUEUE);
    cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_START_CMD); // dummy read
    md_ctrl->txq_active |= CLDMA_BM_ALL_QUEUE;
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_START_CMD, CLDMA_BM_ALL_QUEUE);
    cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_START_CMD); // dummy read
    md_ctrl->rxq_active |= CLDMA_BM_ALL_QUEUE;
    // enable L2 DONE and ERROR interrupts
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMCR0, CLDMA_BM_INT_DONE|CLDMA_BM_INT_ERROR);
    cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMCR0, CLDMA_BM_INT_DONE|CLDMA_BM_INT_ERROR);
    // enable all L3 interrupts
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TIMCR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TIMCR1, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RIMCR0, CLDMA_BM_INT_ALL);
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RIMCR1, CLDMA_BM_INT_ALL);
    spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
}

// only allowed when cldma is stopped
static void md_cd_clear_all_queue(struct ccci_modem *md, DIRECTION dir)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    int i;
    struct ccci_request *req = NULL;
    struct cldma_tgpd *tgpd;
    unsigned long flags;

    if(dir == OUT) {
        for(i=0; i<QUEUE_LEN(md_ctrl->txq); i++) {
            spin_lock_irqsave(&md_ctrl->txq[i].ring_lock, flags);
            req = list_entry(md_ctrl->txq[i].tr_ring, struct ccci_request, entry);
            md_ctrl->txq[i].tr_done = req;
            md_ctrl->txq[i].tx_xmit = req;
            md_ctrl->txq[i].free_slot = tx_queue_buffer_number[i];
            md_ctrl->txq[i].debug_id = 0;
#if PACKET_HISTORY_DEPTH
            md->tx_history_ptr[i] = 0;
#endif
            do {
                tgpd = (struct cldma_tgpd *)req->gpd;
                cldma_write8(&tgpd->gpd_flags, 0, cldma_read8(&tgpd->gpd_flags, 0) & ~0x1);
                cldma_write32(&tgpd->data_buff_bd_ptr, 0, 0);
                cldma_write16(&tgpd->data_buff_len, 0, 0);
                if(req->skb) {
                    ccci_free_skb(req->skb, req->policy);
                    req->skb = NULL;
                }

                req = list_entry(req->entry.next, struct ccci_request, entry);
            } while(&req->entry != md_ctrl->txq[i].tr_ring);
            spin_unlock_irqrestore(&md_ctrl->txq[i].ring_lock, flags);
        }
    } else if(dir == IN) {
        struct cldma_rgpd *rgpd;
        for(i=0; i<QUEUE_LEN(md_ctrl->rxq); i++) {
            req = list_entry(md_ctrl->rxq[i].tr_ring, struct ccci_request, entry);
            md_ctrl->rxq[i].tr_done = req;
#if PACKET_HISTORY_DEPTH
            md->rx_history_ptr[i] = 0;
#endif
            do {
                rgpd = (struct cldma_rgpd *)req->gpd;
                cldma_write8(&rgpd->gpd_flags, 0, 0x81);
                cldma_write16(&rgpd->data_buff_len, 0, 0);
                req->skb->len = 0;
                skb_reset_tail_pointer(req->skb);

                req = list_entry(req->entry.next, struct ccci_request, entry);
            } while(&req->entry != md_ctrl->rxq[i].tr_ring);
        }
    }
}

static int md_cd_stop_queue(struct ccci_modem *md, unsigned char qno, DIRECTION dir)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    int count, ret;
    unsigned long flags;

    if(dir==OUT && qno >= QUEUE_LEN(md_ctrl->txq))
        return -CCCI_ERR_INVALID_QUEUE_INDEX;
    if(dir==IN && qno >= QUEUE_LEN(md_ctrl->rxq))
        return -CCCI_ERR_INVALID_QUEUE_INDEX;

    if(dir==IN) {
        // disable RX_DONE queue and interrupt
        md_cd_lock_cldma_clock_src(1);
        spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
        cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMSR0, CLDMA_BM_ALL_QUEUE&(1<<qno));
        count = 0;
        md_ctrl->rxq_active &= (~(CLDMA_BM_ALL_QUEUE&(1<<qno)));
        do {
            cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_STOP_CMD, CLDMA_BM_ALL_QUEUE&(1<<qno));
            cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_STOP_CMD); // dummy read
            ret = cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_STATUS) & (1<<qno);
            CCCI_INF_MSG(md->index, TAG, "stop Rx CLDMA queue %d, status=%x, count=%d\n", qno, ret, count++);
        } while(ret != 0);
        spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
        md_cd_lock_cldma_clock_src(0);
    }
    return 0;
}

static int md_cd_start_queue(struct ccci_modem *md, unsigned char qno, DIRECTION dir)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    struct ccci_request *req = NULL;
    struct cldma_rgpd *rgpd;
    unsigned long flags;

    if(dir==OUT && qno >= QUEUE_LEN(md_ctrl->txq))
        return -CCCI_ERR_INVALID_QUEUE_INDEX;
    if(dir==IN && qno >= QUEUE_LEN(md_ctrl->rxq))
        return -CCCI_ERR_INVALID_QUEUE_INDEX;

    if(dir==IN) {
        // reset Rx ring buffer
        req = list_entry(md_ctrl->rxq[qno].tr_ring, struct ccci_request, entry);
        md_ctrl->rxq[qno].tr_done = req;
#if PACKET_HISTORY_DEPTH
        md->rx_history_ptr[qno] = 0;
#endif
        do {
            rgpd = (struct cldma_rgpd *)req->gpd;
            cldma_write8(&rgpd->gpd_flags, 0, 0x81);
            cldma_write16(&rgpd->data_buff_len, 0, 0);
            req->skb->len = 0;
            skb_reset_tail_pointer(req->skb);

            req = list_entry(req->entry.next, struct ccci_request, entry);
        } while(&req->entry != md_ctrl->rxq[qno].tr_ring);
        // enable queue and RX_DONE interrupt
        md_cd_lock_cldma_clock_src(1);
        spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);    
        if(md->md_state!=RESET && md->md_state!=GATED && md->md_state!=INVALID) {
            cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_RQSAR(md_ctrl->rxq[qno].index), md_ctrl->rxq[qno].tr_done->gpd_addr);
            cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_START_CMD, CLDMA_BM_ALL_QUEUE&(1<<qno));
            cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMCR0, CLDMA_BM_ALL_QUEUE&(1<<qno));
            cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_START_CMD); // dummy read
            md_ctrl->rxq_active |= (CLDMA_BM_ALL_QUEUE&(1<<qno));
        }
        spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);    
        md_cd_lock_cldma_clock_src(0);
    }
    return 0;
}

static void md_cd_wdt_work(struct work_struct *work)
{
    struct md_cd_ctrl *md_ctrl = container_of(work, struct md_cd_ctrl, wdt_work);
	struct ccci_modem *md = md_ctrl->modem;
	int ret = 0;
    // 1. dump RGU reg
	CCCI_INF_MSG(md->index, TAG, "Dump MD RGU registers\n");
    md_cd_lock_modem_clock_src(1);
	ccci_mem_dump(md->index, md_ctrl->md_rgu_base, 0x30);
    md_cd_lock_modem_clock_src(0);
	if(md->md_state == INVALID) {
		CCCI_ERR_MSG(md->index, TAG, "md_cd_wdt_work: md_state is INVALID\n");
		return;
	}
    // 2. wakelock
    wake_lock_timeout(&md_ctrl->trm_wake_lock, 10*HZ);

#if 1
    if(*((int *)(md->mem_layout.smem_region_vir+CCCI_SMEM_OFFSET_EPON)) == 0xBAEBAE10) { //hardcode
        // 3. reset
        ret = md->ops->reset(md);
        CCCI_INF_MSG(md->index, TAG, "reset MD after SW WDT %d\n", ret);
        // 4. send message, only reset MD on non-eng load
        ccci_send_virtual_md_msg(md, CCCI_MONITOR_CH, CCCI_MD_MSG_RESET, 0);
        
        #ifdef CONFIG_MTK_SVLTE_SUPPORT
        c2k_reset_modem();
        #endif
    } else {
        if(md->critical_user_active[2]== 0) //mdlogger closed
        {
            ret = md->ops->reset(md);
            CCCI_INF_MSG(md->index, TAG, "mdlogger closed,reset MD after WDT %d \n", ret);
            // 4. send message, only reset MD on non-eng load
            ccci_send_virtual_md_msg(md, CCCI_MONITOR_CH, CCCI_MD_MSG_RESET, 0);
        }
        else
        {
       	    md_cd_dump_debug_register(md);
            ccci_md_exception_notify(md, MD_WDT);
        }
    }
#endif // Mask by chao for build error
}

static irqreturn_t md_cd_wdt_isr(int irq, void *data)
{
    struct ccci_modem *md = (struct ccci_modem *)data;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    CCCI_INF_MSG(md->index, TAG, "MD WDT IRQ\n");
    #ifdef ENABLE_DSP_SMEM_SHARE_MPU_REGION
    ccci_set_exp_region_protection(md);
    #endif
    // 1. disable MD WDT
    del_timer(&md_ctrl->bus_timeout_timer);
#ifdef ENABLE_MD_WDT_DBG
    unsigned int state;
    state = cldma_read32(md_ctrl->md_rgu_base, WDT_MD_STA);
    cldma_write32(md_ctrl->md_rgu_base, WDT_MD_MODE, WDT_MD_MODE_KEY);
    CCCI_INF_MSG(md->index, TAG, "WDT IRQ disabled for debug, state=%X\n", state);
#endif
    // 2. start a work queue to do the reset, because we used flush_work which is not designed for ISR
    schedule_work(&md_ctrl->wdt_work);
    return IRQ_HANDLED;
}

void md_cd_ap2md_bus_timeout_timer_func(unsigned long data)
{
    struct ccci_modem *md = (struct ccci_modem *)data;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    CCCI_INF_MSG(md->index, TAG, "MD bus timeout but no WDT IRQ\n");
    // same as WDT ISR
    schedule_work(&md_ctrl->wdt_work);
}

#ifdef ENABLE_HS1_POLLING_TIMER
void md_cd_hs1_polling_timer_func(unsigned long data)
{
    struct ccci_modem *md = (struct ccci_modem *)data;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    CCCI_INF_MSG(md->index, TAG, "md hs1 polling status:\n");
		md->ops->dump_info(md, DUMP_FLAG_CCIF, NULL, 0);
		if(md->md_state==BOOTING)
			mod_timer(&md_ctrl->hs1_polling_timer, jiffies+HZ/2);
}
#endif

#if 0
static irqreturn_t md_cd_ap2md_bus_timeout_isr(int irq, void *data)
{
    struct ccci_modem *md = (struct ccci_modem *)data;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    CCCI_INF_MSG(md->index, TAG, "MD bus timeout IRQ\n");
    mod_timer(&md_ctrl->bus_timeout_timer, jiffies+5*HZ);
    return IRQ_HANDLED;
}
#endif
static int md_cd_ccif_send(struct ccci_modem *md, int channel_id)
{
    int busy = 0;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    busy = cldma_read32(md_ctrl->ap_ccif_base, APCCIF_BUSY);
    if(busy & (1<<channel_id)) {
        return -1;
    }
    cldma_write32(md_ctrl->ap_ccif_base, APCCIF_BUSY, 1<<channel_id);
    cldma_write32(md_ctrl->ap_ccif_base, APCCIF_TCHNUM, channel_id);
    return 0;
}

static void md_cd_exception(struct ccci_modem *md, HIF_EX_STAGE stage)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
	volatile unsigned int SO_CFG;

    CCCI_INF_MSG(md->index, TAG, "MD exception HIF %d\n", stage);
    // in exception mode, MD won't sleep, so we do not need to request MD resource first
    switch(stage) {
    case HIF_EX_INIT:
        #ifdef ENABLE_DSP_SMEM_SHARE_MPU_REGION
        ccci_set_exp_region_protection(md);
        #endif
        if(*((int *)(md->mem_layout.smem_region_vir+CCCI_SMEM_OFFSET_SEQERR)) != 0) {
            CCCI_INF_MSG(md->index, KERN, "MD found wrong sequence number\n");
            md->ops->dump_info(md, DUMP_FLAG_CLDMA, NULL, 0);
        }        
        wake_lock_timeout(&md_ctrl->trm_wake_lock, 10*HZ);
        ccci_md_exception_notify(md, EX_INIT);
        // disable CLDMA except un-stop queues
        cldma_stop_for_ee(md);
        // purge Tx queue
        md_cd_clear_all_queue(md, OUT);
        // Rx dispatch does NOT depend on queue index in port structure, so it still can find right port.
        md_cd_ccif_send(md, H2D_EXCEPTION_ACK);
        break;
    case HIF_EX_INIT_DONE:
        ccci_md_exception_notify(md, EX_DHL_DL_RDY);
        break;
    case HIF_EX_CLEARQ_DONE:
        // give DHL some time to flush data
        schedule_delayed_work(&md_ctrl->ccif_delayed_work, 2*HZ);
        break;
    case HIF_EX_ALLQ_RESET:
        // re-start CLDMA
        cldma_reset(md);
        md_cd_clear_all_queue(md, IN); // purge Rx queue
        ccci_md_exception_notify(md, EX_INIT_DONE);
        cldma_start(md);

		SO_CFG = cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG);
		if (SO_CFG & 0x1 == 0)  // write function didn't work
		{
			CCCI_ERR_MSG(md->index, TAG, "Enable AP OUTCLDMA failed. Register can't be wrote. SO_CFG=0x%x\n", SO_CFG);
			cldma_dump_register(md);
			cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG, cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_SO_CFG)|0x05);
		}
        break;
    default:
        break;
    };
}

static void md_cd_ccif_delayed_work(struct work_struct *work)
{
    struct md_cd_ctrl *md_ctrl = container_of(to_delayed_work(work), struct md_cd_ctrl, ccif_delayed_work);
	struct ccci_modem *md = md_ctrl->modem;
    int i;

#if defined (CONFIG_MTK_AEE_FEATURE)
	aee_kernel_dal_show("Modem exception dump start, please wait up to 5 minutes.\n");
#endif

    // stop CLDMA, we don't want to get CLDMA IRQ when MD is reseting CLDMA after it got cleaq_ack
    cldma_stop(md);
    // flush work
    for(i=0; i<QUEUE_LEN(md_ctrl->txq); i++) {
        flush_work(&md_ctrl->txq[i].cldma_work);
    }
    for(i=0; i<QUEUE_LEN(md_ctrl->rxq); i++) {
        flush_work(&md_ctrl->rxq[i].cldma_work);
    }
#ifdef  ENABLE_CLDMA_AP_SIDE
     md_cldma_hw_reset(md);
#endif
    // tell MD to reset CLDMA
    md_cd_ccif_send(md, H2D_EXCEPTION_CLEARQ_ACK);
    CCCI_INF_MSG(md->index, TAG, "send clearq_ack to MD\n");
}

static void md_cd_ccif_work(struct work_struct *work)
{
    struct md_cd_ctrl *md_ctrl = container_of(work, struct md_cd_ctrl, ccif_work);
	struct ccci_modem *md = md_ctrl->modem;
    // seems sometime MD send D2H_EXCEPTION_INIT_DONE and D2H_EXCEPTION_CLEARQ_DONE together
    if(md_ctrl->channel_id & (1<<D2H_EXCEPTION_INIT))
        md_cd_exception(md, HIF_EX_INIT);
    if(md_ctrl->channel_id & (1<<D2H_EXCEPTION_INIT_DONE))
        md_cd_exception(md, HIF_EX_INIT_DONE);
    if(md_ctrl->channel_id & (1<<D2H_EXCEPTION_CLEARQ_DONE))
        md_cd_exception(md, HIF_EX_CLEARQ_DONE);
    if(md_ctrl->channel_id & (1<<D2H_EXCEPTION_ALLQ_RESET))
        md_cd_exception(md, HIF_EX_ALLQ_RESET);
    if(md_ctrl->channel_id & (1<<AP_MD_PEER_WAKEUP))
        wake_lock_timeout(&md_ctrl->peer_wake_lock, HZ);
    if(md_ctrl->channel_id & (1<<AP_MD_SEQ_ERROR)) {
        CCCI_ERR_MSG(md->index, TAG, "MD check seq fail\n");
        md->ops->dump_info(md, DUMP_FLAG_CCIF, NULL, 0);
    }
}

static irqreturn_t md_cd_ccif_isr(int irq, void *data)
{
    struct ccci_modem *md = (struct ccci_modem *)data;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    // must ack first, otherwise IRQ will rush in
    md_ctrl->channel_id = cldma_read32(md_ctrl->ap_ccif_base, APCCIF_RCHNUM);
	CCCI_DBG_MSG(md->index, TAG, "MD CCIF IRQ 0x%X\n", md_ctrl->channel_id);
    cldma_write32(md_ctrl->ap_ccif_base, APCCIF_ACK, md_ctrl->channel_id);

#if 0 // workqueue is too slow
    schedule_work(&md_ctrl->ccif_work);
#else
    md_cd_ccif_work(&md_ctrl->ccif_work);
#endif
    return IRQ_HANDLED;
}

static inline int cldma_sw_init(struct ccci_modem *md)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    int ret;
    // do NOT touch CLDMA HW after power on MD
    // Copy HW info
    md_ctrl->ap_ccif_base = (void __iomem *)md_ctrl->hw_info->ap_ccif_base;
    md_ctrl->md_ccif_base = (void __iomem *)md_ctrl->hw_info->md_ccif_base;
    md_ctrl->cldma_irq_id = md_ctrl->hw_info->cldma_irq_id;
    md_ctrl->ap_ccif_irq_id = md_ctrl->hw_info->ap_ccif_irq_id;
    md_ctrl->md_wdt_irq_id = md_ctrl->hw_info->md_wdt_irq_id;
    md_ctrl->ap2md_bus_timeout_irq_id = md_ctrl->hw_info->ap2md_bus_timeout_irq_id;
 
    // do NOT touch CLDMA HW after power on MD
    // ioremap CLDMA register region
    md_cd_io_remap_md_side_register(md);
    
    // request IRQ
    ret = request_irq(md_ctrl->hw_info->cldma_irq_id, cldma_isr, md_ctrl->hw_info->cldma_irq_flags, "CLDMA_AP", md);
    if(ret) {
        CCCI_ERR_MSG(md->index, TAG, "request CLDMA_AP IRQ(%d) error %d\n", md_ctrl->hw_info->cldma_irq_id, ret);
        return ret;
    }
#ifndef FEATURE_FPGA_PORTING
    CCCI_INF_MSG(md->index, TAG, "cldma_sw_init is request_irq wdt(%d)\n",md_ctrl->hw_info->md_wdt_irq_id); 
    ret = request_irq(md_ctrl->hw_info->md_wdt_irq_id, md_cd_wdt_isr, md_ctrl->hw_info->md_wdt_irq_flags, "MD_WDT", md);
    if(ret) {
        CCCI_ERR_MSG(md->index, TAG, "request MD_WDT IRQ(%d) error %d\n", md_ctrl->hw_info->md_wdt_irq_id, ret);
        return ret;
    }
    atomic_inc(&md_ctrl->wdt_enabled); // IRQ is enabled after requested, so call enable_irq after request_irq will get a unbalance warning
    ret = request_irq(md_ctrl->hw_info->ap_ccif_irq_id, md_cd_ccif_isr, md_ctrl->hw_info->ap_ccif_irq_flags, "CCIF0_AP", md);
    if(ret) {
        CCCI_ERR_MSG(md->index, TAG, "request CCIF0_AP IRQ(%d) error %d\n", md_ctrl->hw_info->ap_ccif_irq_id, ret);
        return ret;
    }
#endif
    return 0;
}

static int md_cd_broadcast_state(struct ccci_modem *md, MD_STATE state)
{
    int i;
    struct ccci_port *port;

    // only for thoes states which are updated by port_kernel.c
    switch(state) {
    case READY:
        md_cd_bootup_cleanup(md, 1);
        // Update time to modem here, to cover case that user set time between HS1 and IPC channel ready.
        // only modem 1 need. so add here.
        notify_time_update();
        break;
    case BOOT_FAIL:
        if(md->md_state != BOOT_FAIL) // bootup timeout may comes before MD EE
            md_cd_bootup_cleanup(md, 0);
        return 0;
    case RX_IRQ:
	case TX_IRQ:
	case TX_FULL:
		CCCI_ERR_MSG(md->index, TAG, "%ps broadcast %d to ports!\n", __builtin_return_address(0), state);
        return 0;
    default:
        break;
    };

    if(md->md_state == state) // must have, due to we broadcast EXCEPTION both in MD_EX and EX_INIT
        return 1;

    md->md_state = state;
    for(i=0;i<md->port_number;i++) {
        port = md->ports + i;
        if(port->ops->md_state_notice)
            port->ops->md_state_notice(port, state);
    }
    return 0;
}

static int md_cd_init(struct ccci_modem *md)
{
    int i;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    struct ccci_port *port = NULL;

    CCCI_INF_MSG(md->index, TAG, "CLDMA modem is initializing\n");
    // init CLMDA, must before queue init as we set start address there
    cldma_sw_init(md);
    // init queue
    for(i=0; i<QUEUE_LEN(md_ctrl->txq); i++) {
        md_cd_queue_struct_init(&md_ctrl->txq[i], md, OUT, i, tx_queue_buffer_number[i]);
        md_ctrl->txq[i].free_slot = tx_queue_buffer_number[i];
        cldma_tx_queue_init(&md_ctrl->txq[i]);
#if PACKET_HISTORY_DEPTH
        md->tx_history_ptr[i] = 0;
#endif
    }
    for(i=0; i<QUEUE_LEN(md_ctrl->rxq); i++) {
        md_cd_queue_struct_init(&md_ctrl->rxq[i], md, IN, i, rx_queue_buffer_number[i]);
        cldma_rx_queue_init(&md_ctrl->rxq[i]);
#if PACKET_HISTORY_DEPTH
        md->rx_history_ptr[i] = 0;
#endif
    }
    // init port
    for(i=0; i<md->port_number; i++) {
        port = md->ports + i;
        ccci_port_struct_init(port, md);
        port->ops->init(port);
        if((port->flags&PORT_F_RX_EXCLUSIVE) && (port->modem->capability&MODEM_CAP_NAPI) && 
                ((1<<port->rxq_index)&NAPI_QUEUE_MASK) && port->rxq_index!=0xFF) {
            md_ctrl->rxq[port->rxq_index].napi_port = port;
            CCCI_DBG_MSG(md->index, TAG, "queue%d add NAPI port %s\n", port->rxq_index, port->name);
        }
		// be careful, port->rxq_index may be 0xFF!
    }
    ccci_setup_channel_mapping(md);
    // update state
    md->md_state = GATED;
    return 0;
}

void wdt_enable_irq(struct md_cd_ctrl *md_ctrl)
{
    if(atomic_read(&md_ctrl->wdt_enabled) == 0) {
        enable_irq(md_ctrl->md_wdt_irq_id);
        atomic_inc(&md_ctrl->wdt_enabled);
    }
}

void wdt_disable_irq(struct md_cd_ctrl *md_ctrl)
{
    if(atomic_read(&md_ctrl->wdt_enabled) == 1) {
        /*may be called in isr, so use disable_irq_nosync.
        if use disable_irq in isr, system will hang*/
        disable_irq_nosync(md_ctrl->md_wdt_irq_id);
        atomic_dec(&md_ctrl->wdt_enabled);
    }
}
#if TRAFFIC_MONITOR_INTERVAL
static void md_cd_clear_traffic_data(struct ccci_modem *md)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    memset(md_ctrl->tx_traffic_monitor, 0, sizeof(md_ctrl->tx_traffic_monitor));
    memset(md_ctrl->rx_traffic_monitor, 0, sizeof(md_ctrl->rx_traffic_monitor));
    memset(md_ctrl->tx_pre_traffic_monitor, 0, sizeof(md_ctrl->tx_pre_traffic_monitor));
}
#endif

//used for throttling feature - start
extern unsigned long ccci_modem_boot_count[];
//used for throttling feature - end
static int md_cd_start(struct ccci_modem *md)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    char img_err_str[IMG_ERR_STR_LEN];
    int ret=0, retry, cldma_on=0;

    // 0. init security, as security depends on dummy_char, which is ready very late.
    ccci_init_security();

    CCCI_INF_MSG(md->index, TAG, "CLDMA modem is starting\n");
    // 1. load modem image
	if(1 /*md->config.setting&MD_SETTING_FIRST_BOOT || md->config.setting&MD_SETTING_RELOAD*/) {
        ccci_clear_md_region_protection(md);
		ccci_clear_dsp_region_protection(md);
        ret = ccci_load_firmware(md->index, &md->img_info[IMG_MD], img_err_str, md->post_fix);
        if(ret<0) {
			CCCI_ERR_MSG(md->index, TAG, "load MD firmware fail, %s\n", img_err_str);
            goto out;
        }
		if(md->img_info[IMG_MD].dsp_size!=0 && md->img_info[IMG_MD].dsp_offset!=0xCDCDCDAA) {
			md->img_info[IMG_DSP].address = md->img_info[IMG_MD].address + md->img_info[IMG_MD].dsp_offset;
			ret = ccci_load_firmware(md->index, &md->img_info[IMG_DSP], img_err_str, md->post_fix);
			if(ret < 0) {
				CCCI_ERR_MSG(md->index, TAG, "load DSP firmware fail, %s\n", img_err_str);
				goto out;
			}
			if(md->img_info[IMG_DSP].size > md->img_info[IMG_MD].dsp_size) {
				CCCI_ERR_MSG(md->index, TAG, "DSP image real size too large %d\n", md->img_info[IMG_DSP].size);
				goto out;
			}
			md->mem_layout.dsp_region_phy = md->img_info[IMG_DSP].address;
			md->mem_layout.dsp_region_vir = md->mem_layout.md_region_vir + md->img_info[IMG_MD].dsp_offset;
			md->mem_layout.dsp_region_size = ret;
		}
        ret = 0; // load_std_firmware returns MD image size
        md->config.setting &= ~MD_SETTING_RELOAD;
    }
    // 2. clear share memory and ring buffer
#if 0 // no need now, MD will clear share memory itself
    memset(md->mem_layout.smem_region_vir, 0, md->mem_layout.smem_region_size);
#endif
#if 1 // just in case
    md_cd_clear_all_queue(md, OUT);
    md_cd_clear_all_queue(md, IN);
	ccci_reset_seq_num(md);
#endif
    // 3. enable MPU
    ccci_set_mem_access_protection(md);
	if(md->mem_layout.dsp_region_phy != 0)
		ccci_set_dsp_region_protection(md, 0);
    // 4. power on modem, do NOT touch MD register before this
    if(md->config.setting & MD_SETTING_FIRST_BOOT) {
        ret = md_cd_power_off(md, 0);
        CCCI_INF_MSG(md->index, TAG, "power off MD first %d\n", ret);
        md->config.setting &= ~MD_SETTING_FIRST_BOOT;
    }

#if TRAFFIC_MONITOR_INTERVAL
    md_cd_clear_traffic_data(md);
#endif

#ifdef  ENABLE_CLDMA_AP_SIDE
    md_cldma_hw_reset(md);
#endif  
    ret = md_cd_power_on(md);
    if(ret) {
        CCCI_ERR_MSG(md->index, TAG, "power on MD fail %d\n", ret);
        goto out;
    }
    // 5. update mutex
    atomic_set(&md_ctrl->reset_on_going, 0);
    // 6. start timer
    if(!MD_IN_DEBUG(md))
        mod_timer(&md->bootup_timer, jiffies+BOOT_TIMER_ON*HZ);
    // 7. let modem go
    md_cd_let_md_go(md);
    wdt_enable_irq(md_ctrl);
    // 8. start CLDMA
#ifdef  ENABLE_CLDMA_AP_SIDE
     CCCI_INF_MSG(md->index, TAG, "CLDMA AP side clock is always on\n");
#else
    retry = CLDMA_CG_POLL;
    while(retry-->0) {
        if(!(ccci_read32(md_ctrl->md_global_con0, 0) & (1<<MD_GLOBAL_CON0_CLDMA_BIT))) {
            CCCI_INF_MSG(md->index, TAG, "CLDMA clock is on, retry=%d\n", retry);
            cldma_on = 1;
            break;
        } else {
            CCCI_INF_MSG(md->index, TAG, "CLDMA clock is still off, retry=%d\n", retry);
            mdelay(1000);
        }
    }
    if(!cldma_on) {
        ret = -CCCI_ERR_HIF_NOT_POWER_ON;
        CCCI_ERR_MSG(md->index, TAG, "CLDMA clock is off, retry=%d\n", retry);
        goto out;
    }
#endif
    cldma_reset(md);
    md->ops->broadcast_state(md, BOOTING);
    md->boot_stage = MD_BOOT_STAGE_0;
#ifdef ENABLE_HS1_POLLING_TIMER
    mod_timer(&md_ctrl->hs1_polling_timer, jiffies+0);
#endif
    cldma_start(md);

out:
    CCCI_INF_MSG(md->index, TAG, "CLDMA modem started %d\n", ret);
	//used for throttling feature - start
	ccci_modem_boot_count[md->index]++;
	//used for throttling feature - end
    return ret;
}
static void md_cldma_clear(struct ccci_modem *md)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    int i;
    unsigned int ret;
    unsigned long flags;
    int retry=100;
#ifdef  ENABLE_CLDMA_AP_SIDE
    ret=cldma_read32(md_ctrl->cldma_md_pdn_base, CLDMA_AP_UL_STATUS);
    for(i=0; (CLDMA_BM_ALL_QUEUE&ret) && i<QUEUE_LEN(md_ctrl->rxq); i++) {
        if((CLDMA_BM_ALL_QUEUE&ret)&(1<<i)){
            CCCI_INF_MSG(md->index, TAG, "MD CLDMA txq=%d is active, need AP rx collect!",i);
            md->ops->give_more(md, i);
        }
    }
    while(retry>0) {
        ret=cldma_read32(md_ctrl->cldma_md_pdn_base, CLDMA_AP_UL_STATUS);
        if((CLDMA_BM_ALL_QUEUE&ret)==0 && cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_CLDMA_IP_BUSY)==0) {
            CCCI_INF_MSG(md->index, TAG, "MD CLDMA tx status is off, retry=%d, AP_CLDMA_IP_BUSY=0x%x, MD_TX_STATUS=0x%x, AP_RX_STATUS=0x%x\n", retry,
                cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_CLDMA_IP_BUSY),
                cldma_read32(md_ctrl->cldma_md_pdn_base, CLDMA_AP_UL_STATUS),
                cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_STATUS));
            break;
        } else {
            if((retry%10)==0)
                CCCI_INF_MSG(md->index, TAG, "MD CLDMA tx is active, retry=%d, AP_CLDMA_IP_BUSY=0x%x, MD_TX_STATUS=0x%x, AP_RX_STATUS=0x%x\n", retry,
                    cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_CLDMA_IP_BUSY),
                    cldma_read32(md_ctrl->cldma_md_pdn_base, CLDMA_AP_UL_STATUS),
                    cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_STATUS));
            mdelay(20);
        }
        retry--;
    }
    if(retry==0 && cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_CLDMA_IP_BUSY)!=0){
        CCCI_ERR_MSG(md->index, TAG, "md_cldma_clear: wait md tx done failed.\n");
        md_cd_traffic_monitor_func((unsigned long)md);
        cldma_dump_register(md);
    }else{
        CCCI_INF_MSG(md->index, TAG, "md_cldma_clear: md tx done\n");
    }
#endif
    
    md_cd_lock_cldma_clock_src(1);
    cldma_stop(md);
    md_cd_lock_cldma_clock_src(0);
  
    // 4. reset EE flag
    spin_lock_irqsave(&md->ctrl_lock, flags);
    md->ee_info_flag = 0; // must be after broadcast_state(RESET), check port_kernel.c
    spin_unlock_irqrestore(&md->ctrl_lock, flags);
    // 5. update state
    del_timer(&md->bootup_timer);
    // 6. flush CLDMA work and reset ring buffer
    for(i=0; i<QUEUE_LEN(md_ctrl->txq); i++) {
        flush_work(&md_ctrl->txq[i].cldma_work);
    }
    for(i=0; i<QUEUE_LEN(md_ctrl->rxq); i++) {
        flush_work(&md_ctrl->rxq[i].cldma_work);
    }
    md_cd_clear_all_queue(md, OUT);
    /*
     * there is a race condition between md_power_off and CLDMA IRQ. after we get a CLDMA IRQ,
     * if we power off MD before CLDMA tasklet is scheduled, the tasklet will get 0 when reading CLDMA
     * register, and not schedule workqueue to check RGPD. this will leave an HWO=0 RGPD in ring
     * buffer and cause a queue being stopped. so we flush RGPD here to kill this missing RX_DONE interrupt.
     */
    md_cd_clear_all_queue(md, IN);

#ifdef  ENABLE_CLDMA_AP_SIDE
    md_cldma_hw_reset(md);
#endif   
}

// only run this in thread context, as we use flush_work in it
static int md_cd_reset(struct ccci_modem *md)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

    // 1. mutex check
    if(atomic_add_return(1, &md_ctrl->reset_on_going) > 1){
        CCCI_INF_MSG(md->index, TAG, "One reset flow is on-going\n");
        return -CCCI_ERR_MD_IN_RESET;
    }
    CCCI_INF_MSG(md->index, TAG, "md_cd_reset:CLDMA modem is reseting\n");
    // 2. disable WDT IRQ
    wdt_disable_irq(md_ctrl);

    // 3, stop CLDMA
    md->ops->broadcast_state(md, RESET); // to block port's write operation

    md->boot_stage = MD_BOOT_STAGE_0;

    return 0;
}


static int md_cd_stop(struct ccci_modem *md, unsigned int timeout)
{
    int i, ret=0, count=0;
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    u32 pending;	
    CCCI_INF_MSG(md->index, TAG, "CLDMA modem is power off, timeout=%d\n", timeout);
    md->sim_type = 0xEEEEEEEE; //reset sim_type(MCC/MNC) to 0xEEEEEEEE
#ifdef CLDMA_TIMER_LOOP
    for(i=0; i<QUEUE_LEN(md_ctrl->txq); i++) {
    	while(hrtimer_cancel(&md_ctrl->txq[i].cldma_poll_timer)){
				CCCI_INF_MSG(md->index, TAG, "try to cancel txq%d hrtimer\n", i);
			}
    }
#endif

	md_cd_check_emi_state(md, 1); // Check EMI before

	if (timeout) // only debug in Flight mode
	{
		count=5;
		while(spm_is_md1_sleep()==0)
		{
			count--;
			if(count==0){
				CCCI_INF_MSG(md->index, TAG, "MD is not in sleep mode, dump md status!\n");
				CCCI_INF_MSG(md->index, KERN, "Dump MD EX log\n");
				ccci_mem_dump(md->index, md->smem_layout.ccci_exp_smem_base_vir, md->smem_layout.ccci_exp_dump_size);

				md_cd_dump_debug_register(md);
				cldma_dump_register(md);
#if defined (CONFIG_MTK_AEE_FEATURE)
				aed_md_exception_api(NULL, 0, NULL, 0, "After AP send EPOF, MD didn't go to sleep in 4 seconds.", DB_OPT_DEFAULT);
#endif
				break;
			}
			md_cd_lock_cldma_clock_src(1);
			msleep(1000);
			md_cd_lock_cldma_clock_src(0);
			msleep(20);
		}
		pending = mt_irq_get_pending(md_ctrl->hw_info->md_wdt_irq_id);
		if (pending)
		{
			CCCI_INF_MSG(md->index, TAG, "WDT IRQ occur.");
			CCCI_INF_MSG(md->index, KERN, "Dump MD EX log\n");
			ccci_mem_dump(md->index, md->smem_layout.ccci_exp_smem_base_vir, md->smem_layout.ccci_exp_dump_size);

			md_cd_dump_debug_register(md);
			cldma_dump_register(md);
#if defined (CONFIG_MTK_AEE_FEATURE)
				aed_md_exception_api(NULL, 0, NULL, 0, "WDT IRQ occur.", DB_OPT_DEFAULT);
#endif
		}	
	}
#ifndef  ENABLE_CLDMA_AP_SIDE
    md_cldma_clear(md);
#endif
    // power off MD    
        ret = md_cd_power_off(md, timeout);
    CCCI_INF_MSG(md->index, TAG, "CLDMA modem is power off done, %d\n", ret);
    md->ops->broadcast_state(md, GATED);
    
#ifdef ENABLE_CLDMA_AP_SIDE
    md_cldma_clear(md);
#endif
    
    // ACK CCIF for MD. while entering flight mode, we may send something after MD slept
    cldma_write32(md_ctrl->md_ccif_base, APCCIF_ACK, cldma_read32(md_ctrl->md_ccif_base, APCCIF_RCHNUM));

	md_cd_check_emi_state(md, 0); // Check EMI after
    return 0;
}

static int md_cd_write_room(struct ccci_modem *md, unsigned char qno)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    if(qno >= QUEUE_LEN(md_ctrl->txq))
        return -CCCI_ERR_INVALID_QUEUE_INDEX;
    return md_ctrl->txq[qno].free_slot;
}

/*
*[IMPORTANT NOTES]
* For improve network performance, network port will pass skb directly by req args
* So we should check current port no whethe is network queue by IS_PASS_SKB(md,qno),
* if yes,  skb=(struct sk_buff *)req;
* else req is struct ccci_request *
*/
static int md_cd_send_request(struct ccci_modem *md, unsigned char qno, struct ccci_request* req)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    struct md_cd_queue *queue;
    struct ccci_request *tx_req;
    struct cldma_tgpd *tgpd;
    int ret=0;
    int blocking;
    struct ccci_header *ccci_h;
    struct sk_buff *skb;
    unsigned int ch=0;
    unsigned long flags;
    unsigned long long tx_bytes=0;
    
#ifdef CLDMA_TRACE
    static unsigned long long last_leave_time[CLDMA_TXQ_NUM]={0};
    static unsigned long long sample_time[CLDMA_TXQ_NUM]={0}; 
    static unsigned long long sample_bytes[CLDMA_TXQ_NUM]={0};
    static unsigned long long sample_time_last[CLDMA_TXQ_NUM]={0};
    static unsigned long long sample_bytes_last[CLDMA_TXQ_NUM]={0};
    unsigned long long total_time=0;
    unsigned long long tx_interal;
#endif

#ifdef CLDMA_TRACE
    if(last_leave_time[qno] == 0){
        tx_interal= 0;
    }else{
        tx_interal=sched_clock()-last_leave_time[qno];
    }
    total_time=sched_clock();
#endif

#if TRAFFIC_MONITOR_INTERVAL
    if((jiffies-md_ctrl->traffic_stamp)/HZ >= TRAFFIC_MONITOR_INTERVAL) {
        md_ctrl->traffic_stamp = jiffies;
        mod_timer(&md_ctrl->traffic_monitor, jiffies);
    }
#endif

    if(qno >= QUEUE_LEN(md_ctrl->txq)){
        ret = -CCCI_ERR_INVALID_QUEUE_INDEX;
        goto __EXIT_FUN;
    }


    spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
    if(!(md_ctrl->txq_active & (1<<qno))) {
        spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
        ret = -CCCI_ERR_HIF_NOT_POWER_ON;
        goto __EXIT_FUN;        
    }
    if(IS_PASS_SKB(md,qno)){
        if(req==NULL){
            CCCI_ERR_MSG(md->index, TAG, "md_ctrl->txq_active=%d, qno=%d send data NULL\n", md_ctrl->txq_active,qno);
            ret = -CCCI_ERR_INVALID_PARAM;
            goto __EXIT_FUN;      
        }
        skb = (struct sk_buff *)req;
        blocking=0;
    }else{
        if(req==NULL || req->skb==NULL ||req->skb->data==NULL){
            CCCI_ERR_MSG(md->index, TAG, "md_ctrl->txq_active=%d, qno=%d send data NULL\n", md_ctrl->txq_active,qno);
            ret = -CCCI_ERR_INVALID_PARAM;
            goto __EXIT_FUN;      
        }
        skb = (struct sk_buff *)req->skb;
        blocking = req->blocking;
    }
    ccci_h = (struct ccci_header *)skb->data;
    ch=(ccci_h->channel&0xFF);
    
    spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
    queue = &md_ctrl->txq[qno];

retry:
    md_cd_lock_cldma_clock_src(1); // put it outside of spin_lock_irqsave to avoid disabling IRQ too long
    spin_lock_irqsave(&queue->ring_lock, flags); // we use irqsave as network require a lock in softirq, cause a potential deadlock
    CCCI_DBG_MSG(md->index, TAG, "get a Tx req on q%d free=%d\n", qno, queue->free_slot);
    if(queue->free_slot > 0) {
		ccci_inc_tx_seq_num(md, ccci_h);
		wmb();
        /*
         * resume queue inside spinlock, otherwise there is race conditon between ports over the same queue.
         * one port is just setting TGPD, another port may have resumed the queue.
         *
         * start timer before resume CLDMA and inside spinlock_irqsave protection, avoid race condition with disabling timer in tasklet.
         * use an extra spin lock to avoid race conditon with disabling timer in cldma_stop@md_cd_reset.
         */
        spin_lock(&md_ctrl->cldma_timeout_lock);
        if(md_ctrl->txq_active & (1<<qno)) {
            ccci_dump_log_add(md,OUT,(int)queue->index, ccci_h, 0);
            queue->free_slot--;
            // step forward
            tx_req = queue->tx_xmit;
            tgpd = tx_req->gpd;
            queue->tx_xmit = list_entry(tx_req->entry.next, struct ccci_request, entry);
            // override current IOC setting
            if(!IS_PASS_SKB(md,queue->index) && (req->ioc_override & 0x80)) {
                tx_req->ioc_override = 0x80|(!!(tgpd->gpd_flags&0x80)); // backup current IOC setting
                if(req->ioc_override & 0x1)
                    tgpd->gpd_flags |= 0x80;
                else
                    tgpd->gpd_flags &= 0x7F;
            }
            // copy skb pointer
            if(IS_PASS_SKB(md,qno)){
                tx_req->skb = skb;
                tx_req->policy = FREE;

            }else{
                tx_req->skb = skb;
                tx_req->policy = req->policy;
                // free old request as wrapper, do NOT reference this request after this, use tx_req instead
                req->policy = NOOP;
                ccci_free_req(req);
            }
            // update GPD
            tx_bytes =tx_req->skb->len;
            tx_req->data_buffer_ptr_saved = dma_map_single(&md->plat_dev->dev, tx_req->skb->data, tx_bytes, DMA_TO_DEVICE);
    		tgpd->data_buff_bd_ptr = (u32)(tx_req->data_buffer_ptr_saved);
            tgpd->data_buff_len = tx_req->skb->len;
            tgpd->debug_id = queue->debug_id++;
            tgpd->non_used = 1;

#ifdef ENABLE_CLDMA_TIMER
            if(IS_PASS_SKB(md,qno)){
                md_ctrl->cldma_tx_timeout_timer_start[qno]=local_clock();
                ret=mod_timer(&md_ctrl->cldma_tx_timeout_timer[qno], jiffies+CLDMA_ACTIVE_T*HZ);
                CCCI_DBG_MSG(md->index, TAG, "md_ctrl->txq_active=%d, qno=%d ,ch%d, start timer=%d\n", md_ctrl->txq_active,qno,ch,ret);
                ret=0;
            }
#endif 
            // checksum of GPD
            caculate_checksum((char *)tgpd, tgpd->gpd_flags | 0x1);
            // resume Tx queue
            cldma_write8(&tgpd->gpd_flags, 0, cldma_read8(&tgpd->gpd_flags, 0) | 0x1);

            cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_RESUME_CMD, CLDMA_BM_ALL_QUEUE&(1<<qno));
            cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_RESUME_CMD); // dummy read to create a non-buffable write
#ifndef  ENABLE_CLDMA_AP_SIDE
            md_cd_ccif_send(md, AP_MD_PEER_WAKEUP);
#endif            
            md_ctrl->tx_pre_traffic_monitor[qno]++;
            md_ctrl->logic_ch_pkt_pre_cnt[ch]++;
        }else{
            ccci_dump_log_add(md,OUT,(int)queue->index, ccci_h,1);
            CCCI_ERR_MSG(md->index, TAG, "md_ctrl->txq_active=%d, qno=%d is 0,drop ch%d package\n", md_ctrl->txq_active,qno,ch);
            if(IS_PASS_SKB(md,qno)){
                dev_kfree_skb_any(skb);
            }else{
                req->policy = RECYCLE;
                ccci_free_req(req);
            }
        }
        spin_unlock(&md_ctrl->cldma_timeout_lock);

        spin_unlock_irqrestore(&queue->ring_lock, flags); // TX_DONE will check this flag to recycle TGPD
        md_cd_lock_cldma_clock_src(0);
        
    } else {
        if(cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_STATUS) & (1<<qno)){
            CCCI_ERR_MSG(md->index, TAG, "ch=%d qno=%d is free slot 0, queue busy, CLDMA_AP_UL_STATUS=0x%x\n", ch , qno,cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_STATUS));       
	    queue->busy_count++;
        }else{
            if(IS_PASS_SKB(md,qno))
                CCCI_INF_MSG(md->index, TAG, "ch=%d qno=%d is free slot 0, tx busy, CLDMA_AP_L2TIMR0=0x%x\n",ch , qno, cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMR0));   
        }
        if(likely(md->capability & MODEM_CAP_TXBUSY_STOP)) 
            cldma_queue_broadcast_state(md, TX_FULL, OUT, queue->index);
        spin_unlock_irqrestore(&queue->ring_lock, flags);
        md_cd_lock_cldma_clock_src(0);
        if(blocking) {
            ret = wait_event_interruptible_exclusive(queue->req_wq, (queue->free_slot>0));
            if(ret == -ERESTARTSYS) {
                ret = -EINTR;
                goto __EXIT_FUN;
            }
            #ifdef CLDMA_TRACE
            trace_cldma_error(qno, ch,ret,__LINE__);
            #endif
            goto retry;
        } else {
            #ifdef CLDMA_TRACE
            trace_cldma_error(qno, ch,ret,__LINE__);
            #endif
            ret = -EBUSY;
            goto __EXIT_FUN;
        }
    }

__EXIT_FUN:
    
#ifdef CLDMA_TRACE    
    if(ret){
        CCCI_DBG_MSG(md->index, TAG, "txq_active=%d, qno=%d is 0,drop ch%d package,ret=%d\n", md_ctrl->txq_active,qno,ch,ret);
        trace_cldma_error(qno, ch,ret,__LINE__);
    }else{
        total_time = sched_clock()-total_time;
        sample_time[queue->index] += (total_time+tx_interal);
        sample_bytes[queue->index]+=tx_bytes;
        if(sample_time[queue->index]>=trace_sample_time){
            sample_time_last[queue->index]=sample_time[queue->index];
            sample_bytes_last[queue->index]=sample_bytes[queue->index];
            sample_time[queue->index]=0;
            sample_bytes[queue->index]=0;
            
        }        
        trace_cldma_tx(qno,ch, md_ctrl->txq[qno].free_slot,tx_interal,total_time, tx_bytes,sample_time_last[queue->index],sample_bytes_last[queue->index]);
        last_leave_time[qno]= sched_clock();
    }
#endif
    return ret;
}

static int md_cd_give_more(struct ccci_modem *md, unsigned char qno)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    int ret;

    if(qno >= QUEUE_LEN(md_ctrl->rxq))
        return -CCCI_ERR_INVALID_QUEUE_INDEX;
    CCCI_DBG_MSG(md->index, TAG, "give more on queue %d work %p\n", qno, &md_ctrl->rxq[qno].cldma_work);
    ret = queue_work(md_ctrl->rxq[qno].worker, &md_ctrl->rxq[qno].cldma_work);
    return 0;
}

static int md_cd_napi_poll(struct ccci_modem *md, unsigned char qno, struct napi_struct *napi ,int weight)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    int ret, result, all_clr=0;
    unsigned long flags;
    unsigned long long rx_bytes;
    if(qno >= QUEUE_LEN(md_ctrl->rxq))
        return -CCCI_ERR_INVALID_QUEUE_INDEX;

    ret = cldma_rx_collect(&md_ctrl->rxq[qno], weight, 0, &result,&rx_bytes);
    if(likely(weight >= md_ctrl->rxq[qno].budget))
        all_clr = ret<md_ctrl->rxq[qno].budget?1:0;
    else
        all_clr = ret==0?1:0;
    if(likely(all_clr && result!=NO_REQ && result!=NO_SKB))
        all_clr = 1;
    else
        all_clr = 0;
    if(all_clr) {
        napi_complete(napi);
    }
    md_cd_lock_cldma_clock_src(1);
    // resume Rx queue
    spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
    if(md_ctrl->rxq_active & (1<<qno)) {
    cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_RESUME_CMD, CLDMA_BM_ALL_QUEUE&(1<<md_ctrl->rxq[qno].index));
        cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_RESUME_CMD); // dummy read
        // enable RX_DONE interrupt
        if(all_clr)
        cldma_write32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_L2RIMCR0, CLDMA_BM_ALL_QUEUE&(1<<qno));
    }
    spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
    md_cd_lock_cldma_clock_src(0);

    return ret;
}

static struct ccci_port* md_cd_get_port_by_minor(struct ccci_modem *md, int minor)
{
    int i;
    struct ccci_port *port;

    for(i=0; i<md->port_number; i++) {
        port = md->ports + i;
        if(port->minor == minor)
            return port;
    }
    return NULL;
}

static struct ccci_port* md_cd_get_port_by_channel(struct ccci_modem *md, CCCI_CH ch)
{
    int i;
    struct ccci_port *port;

    for(i=0; i<md->port_number; i++) {
        port = md->ports + i;
        if(port->rx_ch == ch || port->tx_ch == ch)
            return port;
    }
    return NULL;
}

static void dump_runtime_data(struct ccci_modem *md, struct modem_runtime *runtime)
{
    char    ctmp[12];
    int        *p;

    p = (int*)ctmp;
    *p = runtime->Prefix;
    p++;
    *p = runtime->Platform_L;
    p++;
    *p = runtime->Platform_H;

    CCCI_INF_MSG(md->index, TAG, "**********************************************\n");
    CCCI_INF_MSG(md->index, TAG, "Prefix                      %c%c%c%c\n", ctmp[0], ctmp[1], ctmp[2], ctmp[3]);
    CCCI_INF_MSG(md->index, TAG, "Platform_L                  %c%c%c%c\n", ctmp[4], ctmp[5], ctmp[6], ctmp[7]);
    CCCI_INF_MSG(md->index, TAG, "Platform_H                  %c%c%c%c\n", ctmp[8], ctmp[9], ctmp[10], ctmp[11]);
    CCCI_INF_MSG(md->index, TAG, "DriverVersion               0x%x\n", runtime->DriverVersion);
    CCCI_INF_MSG(md->index, TAG, "BootChannel                 %d\n", runtime->BootChannel);
    CCCI_INF_MSG(md->index, TAG, "BootingStartID(Mode)        0x%x\n", runtime->BootingStartID);
    CCCI_INF_MSG(md->index, TAG, "BootAttributes              %d\n", runtime->BootAttributes);
    CCCI_INF_MSG(md->index, TAG, "BootReadyID                 %d\n", runtime->BootReadyID);

    CCCI_INF_MSG(md->index, TAG, "ExceShareMemBase            0x%x\n", runtime->ExceShareMemBase);
    CCCI_INF_MSG(md->index, TAG, "ExceShareMemSize            0x%x\n", runtime->ExceShareMemSize);
    CCCI_INF_MSG(md->index, TAG, "TotalShareMemBase           0x%x\n", runtime->TotalShareMemBase);
    CCCI_INF_MSG(md->index, TAG, "TotalShareMemSize           0x%x\n", runtime->TotalShareMemSize);

    CCCI_INF_MSG(md->index, TAG, "CheckSum                    %d\n", runtime->CheckSum);

    p = (int*)ctmp;
    *p = runtime->Postfix;
    CCCI_INF_MSG(md->index, TAG, "Postfix                     %c%c%c%c\n", ctmp[0], ctmp[1], ctmp[2], ctmp[3]);
    CCCI_INF_MSG(md->index, TAG, "**********************************************\n");

    p = (int*)ctmp;
    *p = runtime->misc_prefix;
    CCCI_INF_MSG(md->index, TAG, "Prefix                      %c%c%c%c\n", ctmp[0], ctmp[1], ctmp[2], ctmp[3]);
    CCCI_INF_MSG(md->index, TAG, "SupportMask                 0x%x\n", runtime->support_mask);
    CCCI_INF_MSG(md->index, TAG, "Index                       0x%x\n", runtime->index);
    CCCI_INF_MSG(md->index, TAG, "Next                        0x%x\n", runtime->next);
    CCCI_INF_MSG(md->index, TAG, "Feature0  0x%x  0x%x  0x%x  0x%x\n", runtime->feature_0_val[0],runtime->feature_0_val[1],runtime->feature_0_val[2],runtime->feature_0_val[3]);
    CCCI_INF_MSG(md->index, TAG, "Feature1  0x%x  0x%x  0x%x  0x%x\n", runtime->feature_1_val[0],runtime->feature_1_val[1],runtime->feature_1_val[2],runtime->feature_1_val[3]);
    CCCI_INF_MSG(md->index, TAG, "Feature2  0x%x  0x%x  0x%x  0x%x\n", runtime->feature_2_val[0],runtime->feature_2_val[1],runtime->feature_2_val[2],runtime->feature_2_val[3]);
    CCCI_INF_MSG(md->index, TAG, "Feature3  0x%x  0x%x  0x%x  0x%x\n", runtime->feature_3_val[0],runtime->feature_3_val[1],runtime->feature_3_val[2],runtime->feature_3_val[3]);
    CCCI_INF_MSG(md->index, TAG, "Feature4  0x%x  0x%x  0x%x  0x%x\n", runtime->feature_4_val[0],runtime->feature_4_val[1],runtime->feature_4_val[2],runtime->feature_4_val[3]);
    CCCI_INF_MSG(md->index, TAG, "Feature5  0x%x  0x%x  0x%x  0x%x\n", runtime->feature_5_val[0],runtime->feature_5_val[1],runtime->feature_5_val[2],runtime->feature_5_val[3]);
    CCCI_INF_MSG(md->index, TAG, "Feature6  0x%x  0x%x  0x%x  0x%x\n", runtime->feature_6_val[0],runtime->feature_6_val[1],runtime->feature_6_val[2],runtime->feature_6_val[3]);
	CCCI_INF_MSG(md->index, TAG, "Feature7  0x%x  0x%x  0x%x  0x%x\n", runtime->feature_7_val[0],runtime->feature_7_val[1],runtime->feature_7_val[2],runtime->feature_7_val[3]);

    p = (int*)ctmp;
    *p = runtime->misc_postfix;
    CCCI_INF_MSG(md->index, TAG, "Postfix                     %c%c%c%c\n", ctmp[0], ctmp[1], ctmp[2], ctmp[3]);

    CCCI_INF_MSG(md->index, TAG, "----------------------------------------------\n");
}

#ifdef FEATURE_MD_GET_CLIB_TIME
extern volatile int current_time_zone;
#endif
static int md_cd_send_runtime_data(struct ccci_modem *md, unsigned int sbp_code)
{
    int packet_size = sizeof(struct modem_runtime)+sizeof(struct ccci_header);
    struct ccci_request *req = NULL;
    struct ccci_header *ccci_h;
    struct modem_runtime *runtime;
    struct file *filp = NULL;
    LOGGING_MODE mdlog_flag = MODE_IDLE;
    int ret;
    char str[16];
    char md_logger_cfg_file[32];
    unsigned int random_seed = 0;

#ifdef FEATURE_MD_GET_CLIB_TIME
    struct timeval t;
#endif
    snprintf(str, sizeof(str), "%s", AP_PLATFORM_INFO);

    req = ccci_alloc_req(OUT, packet_size, 1, 1);
    if(!req) {
        return -CCCI_ERR_ALLOCATE_MEMORY_FAIL;
    }
    ccci_h = (struct ccci_header *)req->skb->data;
    runtime = (struct modem_runtime *)(req->skb->data + sizeof(struct ccci_header));

    ccci_set_ap_region_protection(md);
    // header
    ccci_h->data[0]=0x00;
    ccci_h->data[1]= packet_size;
    ccci_h->reserved = MD_INIT_CHK_ID;
    ccci_h->channel = CCCI_CONTROL_TX;
    memset(runtime, 0, sizeof(struct modem_runtime));
    // runtime data, little endian for string
    runtime->Prefix = 0x46494343; // "CCIF"
    runtime->Postfix = 0x46494343; // "CCIF"
    runtime->Platform_L = *((int*)str);
    runtime->Platform_H = *((int*)&str[4]);
    runtime->BootChannel = CCCI_CONTROL_RX;
    runtime->DriverVersion = CCCI_DRIVER_VER;

    if(md->index == 0)
        snprintf(md_logger_cfg_file, 32, "%s", MD1_LOGGER_FILE_PATH);
    else
        snprintf(md_logger_cfg_file, 32, "%s", MD2_LOGGER_FILE_PATH);
    filp = filp_open(md_logger_cfg_file, O_RDONLY, 0777);
    if (!IS_ERR(filp)) {
        ret = kernel_read(filp, 0, (char*)&mdlog_flag, sizeof(int));
        if (ret != sizeof(int))
            mdlog_flag = MODE_IDLE;
    } else {
        CCCI_ERR_MSG(md->index, TAG, "open %s fail", md_logger_cfg_file);
        filp = NULL;
    }
    if (filp != NULL) {
        filp_close(filp, NULL);
    }

    if (is_meta_mode() || is_advanced_meta_mode())
        runtime->BootingStartID = ((char)mdlog_flag << 8 | META_BOOT_ID);
    else
        runtime->BootingStartID = ((char)mdlog_flag << 8 | NORMAL_BOOT_ID);

    // share memory layout
    runtime->ExceShareMemBase = md->mem_layout.smem_region_phy - md->mem_layout.smem_offset_AP_to_MD;
    runtime->ExceShareMemSize = md->mem_layout.smem_region_size;
    runtime->TotalShareMemBase = md->mem_layout.smem_region_phy - md->mem_layout.smem_offset_AP_to_MD;
    runtime->TotalShareMemSize = md->mem_layout.smem_region_size;
    // misc region, little endian for string
    runtime->misc_prefix = 0x4353494D; // "MISC"
    runtime->misc_postfix = 0x4353494D; // "MISC"
    runtime->index = 0;
    runtime->next = 0;
    // 32K clock less
#if defined (ENABLE_32K_CLK_LESS)
    if(crystal_exist_status())
    {
        CCCI_INF_MSG(md->index, TAG, "MISC_32K_LESS no support, crystal_exist_status 1\n");
        runtime->support_mask |= (FEATURE_NOT_SUPPORT<<(MISC_32K_LESS*2));
    }
    else
    {
        CCCI_INF_MSG(md->index, TAG,"MISC_32K_LESS support\n");
        runtime->support_mask |= (FEATURE_SUPPORT<<(MISC_32K_LESS*2));
    }
#else
    CCCI_INF_MSG(md->index, TAG, "ENABLE_32K_CLK_LESS disabled\n");
    runtime->support_mask |= (FEATURE_NOT_SUPPORT<<(MISC_32K_LESS*2));
#endif

    
    // random seed
    get_random_bytes(&random_seed, sizeof(int));
    runtime->feature_2_val[0] = random_seed;
    runtime->support_mask |= (FEATURE_SUPPORT<<(MISC_RAND_SEED*2));
    // SBP
    if (sbp_code > 0) {
        runtime->support_mask |= (FEATURE_SUPPORT<<(MISC_MD_SBP_SETTING * 2));
        runtime->feature_4_val[0] = sbp_code;
    }
   
    // CCCI debug
#if defined(FEATURE_SEQ_CHECK_EN) || defined(FEATURE_POLL_MD_EN)
    runtime->support_mask |= (FEATURE_SUPPORT<<(MISC_MD_SEQ_CHECK * 2));
    runtime->feature_5_val[0] = 0;
#ifdef FEATURE_SEQ_CHECK_EN
    runtime->feature_5_val[0] |= (1<<0);
#endif
#ifdef FEATURE_POLL_MD_EN
    runtime->feature_5_val[0] |= (1<<1);
#endif
#endif

#ifdef FEATURE_MD_GET_CLIB_TIME
    CCCI_INF_MSG(md->index, TAG, "FEATURE_MD_GET_CLIB_TIME is on \n");
    runtime->support_mask |= (FEATURE_SUPPORT<<(MISC_MD_CLIB_TIME * 2));

    do_gettimeofday(&t);

    // set seconds information
    runtime->feature_6_val[0] = t.tv_sec;
    runtime->feature_6_val[1] = t.tv_sec>>32;
    runtime->feature_6_val[2] = current_time_zone; //sys_tz.tz_minuteswest;
    runtime->feature_6_val[3] = sys_tz.tz_dsttime;// not used for now

#endif
#ifdef FEATURE_C2K_ALWAYS_ON
    runtime->support_mask |= (FEATURE_SUPPORT<<(MISC_MD_C2K_ON*2));
    runtime->feature_7_val[0] = 
        ( 0
    #ifdef CONFIG_MTK_C2K_SUPPORT
        | (1<<0)
    #endif
    #ifdef CONFIG_MTK_SVLTE_SUPPORT
        | (1<<1)
    #endif
		#ifdef CONFIG_MTK_SRLTE_SUPPORT
        | (1<<2)
		#endif
        );
#endif

    dump_runtime_data(md, runtime);
    skb_put(req->skb, packet_size);
    ret =  md->ops->send_request(md, 0, req); // hardcode to queue 0
    if(ret==0 && !MD_IN_DEBUG(md)) {
        mod_timer(&md->bootup_timer, jiffies+BOOT_TIMER_HS1*HZ);
    }
    return ret;
}

static int md_cd_force_assert(struct ccci_modem *md, MD_COMM_TYPE type)
{
    struct ccci_request *req = NULL;
    struct ccci_header *ccci_h;

    CCCI_INF_MSG(md->index, TAG, "force assert MD using %d\n", type);
    switch(type) {
    case CCCI_MESSAGE:
        req = ccci_alloc_req(OUT, sizeof(struct ccci_header), 1, 1);
        if(req) {
            req->policy = RECYCLE;
            ccci_h = (struct ccci_header *)skb_put(req->skb, sizeof(struct ccci_header));
            ccci_h->data[0] = 0xFFFFFFFF;
            ccci_h->data[1] = 0x5A5A5A5A;
            //ccci_h->channel = CCCI_FORCE_ASSERT_CH;
            *(((u32 *)ccci_h)+2) = CCCI_FORCE_ASSERT_CH;
            ccci_h->reserved = 0xA5A5A5A5;
            return md->ops->send_request(md, 0, req); // hardcode to queue 0
        }
        return -CCCI_ERR_ALLOCATE_MEMORY_FAIL;
    case CCIF_INTERRUPT:
        md_cd_ccif_send(md, H2D_FORCE_MD_ASSERT);
        break;
    case CCIF_INTR_SEQ:
        md_cd_ccif_send(md, AP_MD_SEQ_ERROR);
        break;
    };
    return 0;
}
static void md_cd_dump_ccif_reg(struct ccci_modem *md)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    CCCI_INF_MSG(md->index, TAG, "AP_CON(%p)=%x\n", md_ctrl->ap_ccif_base+APCCIF_CON, cldma_read32(md_ctrl->ap_ccif_base, APCCIF_CON));
    CCCI_INF_MSG(md->index, TAG, "AP_BUSY(%p)=%x\n", md_ctrl->ap_ccif_base+APCCIF_BUSY, cldma_read32(md_ctrl->ap_ccif_base, APCCIF_BUSY));
    CCCI_INF_MSG(md->index, TAG, "AP_START(%p)=%x\n", md_ctrl->ap_ccif_base+APCCIF_START, cldma_read32(md_ctrl->ap_ccif_base, APCCIF_START));
    CCCI_INF_MSG(md->index, TAG, "AP_TCHNUM(%p)=%x\n", md_ctrl->ap_ccif_base+APCCIF_TCHNUM, cldma_read32(md_ctrl->ap_ccif_base, APCCIF_TCHNUM));
    CCCI_INF_MSG(md->index, TAG, "AP_RCHNUM(%p)=%x\n", md_ctrl->ap_ccif_base+APCCIF_RCHNUM, cldma_read32(md_ctrl->ap_ccif_base, APCCIF_RCHNUM));
    CCCI_INF_MSG(md->index, TAG, "AP_ACK(%p)=%x\n", md_ctrl->ap_ccif_base+APCCIF_ACK, cldma_read32(md_ctrl->ap_ccif_base, APCCIF_ACK));
    CCCI_INF_MSG(md->index, TAG, "MD_CON(%p)=%x\n", md_ctrl->md_ccif_base+APCCIF_CON, cldma_read32(md_ctrl->md_ccif_base, APCCIF_CON));
    CCCI_INF_MSG(md->index, TAG, "MD_BUSY(%p)=%x\n", md_ctrl->md_ccif_base+APCCIF_BUSY, cldma_read32(md_ctrl->md_ccif_base, APCCIF_BUSY));
    CCCI_INF_MSG(md->index, TAG, "MD_START(%p)=%x\n", md_ctrl->md_ccif_base+APCCIF_START, cldma_read32(md_ctrl->md_ccif_base, APCCIF_START));
    CCCI_INF_MSG(md->index, TAG, "MD_TCHNUM(%p)=%x\n", md_ctrl->md_ccif_base+APCCIF_TCHNUM, cldma_read32(md_ctrl->md_ccif_base, APCCIF_TCHNUM));
    CCCI_INF_MSG(md->index, TAG, "MD_RCHNUM(%p)=%x\n", md_ctrl->md_ccif_base+APCCIF_RCHNUM, cldma_read32(md_ctrl->md_ccif_base, APCCIF_RCHNUM));
    CCCI_INF_MSG(md->index, TAG, "MD_ACK(%p)=%x\n", md_ctrl->md_ccif_base+APCCIF_ACK, cldma_read32(md_ctrl->md_ccif_base, APCCIF_ACK));
}

static int md_cd_dump_info(struct ccci_modem *md, MODEM_DUMP_FLAG flag, void *buff, int length)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    struct ccci_request *req = NULL;

    if(flag & DUMP_FLAG_CCIF_REG)
    {
        CCCI_INF_MSG(md->index, TAG, "Dump CCIF REG\n");
        md_cd_dump_ccif_reg(md);
    }
    if(flag & DUMP_FLAG_CCIF) {
        int i;
        unsigned int *dest_buff = NULL;
		unsigned char ccif_sram[CCCC_SMEM_CCIF_SRAM_SIZE] = {0};
        int  sram_size = md_ctrl->hw_info->sram_size;
        if(buff)
            dest_buff = (unsigned int*)buff;
        else
            dest_buff = (unsigned int*)ccif_sram;
		if(length<sizeof(ccif_sram) && length>0) {
			CCCI_ERR_MSG(md->index, TAG, "dump CCIF SRAM length illegal %d/%zu\n", length, sizeof(ccif_sram));
			dest_buff = (unsigned int*)ccif_sram;
		} else {
			length = sizeof(ccif_sram);
		}

        for(i=0; i<length/sizeof(unsigned int); i++) {
			*(dest_buff+i) = cldma_read32(md_ctrl->ap_ccif_base, 
				APCCIF_CHDATA+(sram_size-length)+i*sizeof(unsigned int));
        }
        CCCI_INF_MSG(md->index, TAG, "Dump CCIF SRAM (last 16bytes)\n");
        ccci_mem_dump(md->index, dest_buff, length);
    }
    if(flag & DUMP_FLAG_CLDMA) 
    {
        cldma_dump_all_gpd(md);
        cldma_dump_register(md);
        ccci_dump_log_history(md, 1, QUEUE_LEN(md_ctrl->txq),QUEUE_LEN(md_ctrl->rxq));
    }
    if(flag & DUMP_FLAG_REG) {
        md_cd_dump_debug_register(md);
    }
    if(flag & DUMP_FLAG_SMEM) {
        CCCI_INF_MSG(md->index, TAG, "Dump share memory\n");
        ccci_mem_dump(md->index,md->smem_layout.ccci_exp_smem_base_vir, md->smem_layout.ccci_exp_dump_size);
    }
    if(flag & DUMP_FLAG_IMAGE) {
        CCCI_INF_MSG(md->index, KERN, "Dump MD image memory\n");
        ccci_mem_dump(md->index,(void*)md->mem_layout.md_region_vir, MD_IMG_DUMP_SIZE);
    }
    if(flag & DUMP_FLAG_LAYOUT) {
        CCCI_INF_MSG(md->index, KERN, "Dump MD layout struct\n");
        ccci_mem_dump(md->index,&md->mem_layout, sizeof(struct ccci_mem_layout));
    }

    if(flag & DUMP_FLAG_DUMP_QUEUE_0) {
        cldma_dump_register(md);

		printk("[CCCI%d-DUMP]dump txq0 current pos: tr_done=%x, tx_xmit=%x\n",md->index+1, (unsigned int)md_ctrl->txq[0].tr_done->gpd_addr, (unsigned int)md_ctrl->txq[0].tx_xmit->gpd_addr);
                ccci_dump_log_history(md,0,0,0);

		printk("[CCCI%d/CLDMA] dump txq0 GPD\n", md->index+1);
		req = list_entry(md_ctrl->txq[0].tr_ring, struct ccci_request, entry);
		cldma_dump_gpd_ring(md->index, req->gpd_addr, tx_queue_buffer_number[0]);

		printk("[CCCI%d/CLDMA] dump rxq0 GPD\n", md->index+1);
		req = list_entry(md_ctrl->rxq[0].tr_ring, struct ccci_request, entry);
		cldma_dump_gpd_ring(md->index,req->gpd_addr, rx_queue_buffer_number[0]);
	}

	CCCI_INF_MSG(md->index, KERN, "Dump MD RGU registers\n");
    md_cd_lock_modem_clock_src(1);
    ccci_mem_dump(md->index, md_ctrl->md_rgu_base, 0x30);
    md_cd_lock_modem_clock_src(0);
	CCCI_INF_MSG(md->index, KERN, "wdt_enabled=%d\n", atomic_read(&md_ctrl->wdt_enabled));
	mt_irq_dump_status(md_ctrl->hw_info->md_wdt_irq_id);

    return length;
}

static int md_cd_ee_callback(struct ccci_modem *md, MODEM_EE_FLAG flag)
{
	struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;

	if (flag & EE_FLAG_ENABLE_WDT)
	{
		wdt_enable_irq(md_ctrl);
	}
	if (flag & EE_FLAG_DISABLE_WDT)
	{
		wdt_disable_irq(md_ctrl);
	}
	return 0;
}

static struct ccci_modem_ops md_cd_ops = {
    .init = &md_cd_init,
    .start = &md_cd_start,
    .stop = &md_cd_stop,
    .reset = &md_cd_reset,
    .send_request = &md_cd_send_request,
    .give_more = &md_cd_give_more,
    .napi_poll = &md_cd_napi_poll,
    .send_runtime_data = &md_cd_send_runtime_data,
    .broadcast_state = &md_cd_broadcast_state,
    .force_assert = &md_cd_force_assert,
    .dump_info = &md_cd_dump_info,
    .write_room = &md_cd_write_room,
    .stop_queue = &md_cd_stop_queue,
    .start_queue = &md_cd_start_queue,
    .get_port_by_minor = &md_cd_get_port_by_minor,
    .get_port_by_channel = &md_cd_get_port_by_channel,
    //.low_power_notify = &md_cd_low_power_notify,
    .ee_callback = &md_cd_ee_callback,
};

static ssize_t md_cd_dump_show(struct ccci_modem *md, char *buf)
{
    int count = 0;
    count = snprintf(buf, 256, "support: ccif cldma register smem image layout\n");
    return count;
}

static ssize_t md_cd_dump_store(struct ccci_modem *md, const char *buf, size_t count)
{
    struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)md->private_data;
    // echo will bring "xxx\n" here, so we eliminate the "\n" during comparing
    if(strncmp(buf, "ccif", count-1) == 0) {
        md->ops->dump_info(md,DUMP_FLAG_CCIF_REG | DUMP_FLAG_CCIF, NULL, 0);
    }
    if(strncmp(buf, "cldma", count-1) == 0) {
        md->ops->dump_info(md, DUMP_FLAG_CLDMA, NULL, 0);
    }
    if(strncmp(buf, "register", count-1) == 0) {
        md->ops->dump_info(md, DUMP_FLAG_REG, NULL, 0);
    }
    if(strncmp(buf, "smem", count-1) == 0) {
        md->ops->dump_info(md, DUMP_FLAG_SMEM, NULL, 0);
    }
    if(strncmp(buf, "image", count-1) == 0) {
        md->ops->dump_info(md, DUMP_FLAG_IMAGE, NULL, 0);
    }
    if(strncmp(buf, "layout", count-1) == 0) {
        md->ops->dump_info(md, DUMP_FLAG_LAYOUT, NULL, 0);
    }
    return count;
}

static ssize_t md_cd_control_show(struct ccci_modem *md, char *buf)
{
    int count = 0;
    count = snprintf(buf, 256, "support: cldma_reset cldma_stop ccif_assert md_type trace_sample\n");
    return count;
}

static ssize_t md_cd_control_store(struct ccci_modem *md, const char *buf, size_t count)
{
    int size=0;
    if(strncmp(buf, "cldma_reset", count-1) == 0) {
        CCCI_INF_MSG(md->index, TAG, "reset CLDMA\n");
        md_cd_lock_cldma_clock_src(1);
        cldma_stop(md);
        md_cd_clear_all_queue(md, OUT);
        md_cd_clear_all_queue(md, IN);
        cldma_reset(md);
        cldma_start(md);
        md_cd_lock_cldma_clock_src(0);
    }
    if(strncmp(buf, "cldma_stop", count-1) == 0) {
        CCCI_INF_MSG(md->index, TAG, "stop CLDMA\n");
        md_cd_lock_cldma_clock_src(1);
        cldma_stop(md);
        md_cd_lock_cldma_clock_src(0);
    }
    if(strncmp(buf, "ccif_assert", count-1) == 0) {
        CCCI_INF_MSG(md->index, TAG, "use CCIF to force MD assert\n");
        md->ops->force_assert(md, CCIF_INTERRUPT);
    }
    if(strncmp(buf, "ccci_trm", count-1)==0) {
        CCCI_INF_MSG(md->index, TAG, "TRM triggered\n");
        md->ops->reset(md);
        ccci_send_virtual_md_msg(md, CCCI_MONITOR_CH, CCCI_MD_MSG_RESET, 0);
    }
    size = strlen("md_type=");
    if(strncmp(buf, "md_type=", size)==0) {
        md->config.load_type_saving = buf[size]-'0';
        CCCI_INF_MSG(md->index, TAG, "md_type_store %d\n",md->config.load_type_saving);
        ccci_send_virtual_md_msg(md, CCCI_MONITOR_CH, CCCI_MD_MSG_STORE_NVRAM_MD_TYPE, 0);
    }
    size = strlen("trace_sample=");
    if(strncmp(buf, "trace_sample=", size)==0) {
        trace_sample_time = (buf[size]-'0')*100000000;
        CCCI_INF_MSG(md->index, TAG, "trace_sample_time %llu\n",trace_sample_time);
    }
    return count;
}

#define GF_PORT_LIST_MAX 128
extern int gf_port_list_reg[GF_PORT_LIST_MAX];
extern int gf_port_list_unreg[GF_PORT_LIST_MAX];
extern int ccci_ipc_set_garbage_filter(struct ccci_modem *md, int reg);

static ssize_t md_cd_filter_show(struct ccci_modem *md, char *buf)
{
    int count = 0;
    int i;

    count += snprintf(buf+count, 128, "register port:");
    for(i=0; i<GF_PORT_LIST_MAX; i++) {
    	if(gf_port_list_reg[i] != 0)
		count += snprintf(buf+count, 128, "%d,", gf_port_list_reg[i]);
	else
		break;
    }
    count += snprintf(buf+count, 128, "\n");
    count += snprintf(buf+count, 128, "unregister port:");
    for(i=0; i<GF_PORT_LIST_MAX; i++) {
    	if(gf_port_list_unreg[i] != 0)
		count += snprintf(buf+count, 128, "%d,", gf_port_list_unreg[i]);
	else
		break;
    }
    count += snprintf(buf+count, 128, "\n");
    return count;
}

static ssize_t md_cd_filter_store(struct ccci_modem *md, const char *buf, size_t count)
{
	char command[16];
	int start_id=0, end_id=0,i;
	
	sscanf(buf, "%s %d %d%*s", command, &start_id, &end_id);
	CCCI_INF_MSG(md->index, TAG, "%s from %d to %d\n", command, start_id, end_id);
	if(strncmp(command, "add", sizeof(command))==0) {
		memset(gf_port_list_reg, 0, sizeof(gf_port_list_reg));
		for(i=0; i<GF_PORT_LIST_MAX&&i<=(end_id-start_id); i++) {
			gf_port_list_reg[i] = start_id+i;
		}
		ccci_ipc_set_garbage_filter(md, 1);
	}
	if(strncmp(command, "remove", sizeof(command))==0) {
		memset(gf_port_list_unreg, 0, sizeof(gf_port_list_unreg));
		for(i=0; i<GF_PORT_LIST_MAX&&i<=(end_id-start_id); i++) {
			gf_port_list_unreg[i] = start_id+i;
		}
		ccci_ipc_set_garbage_filter(md, 0);
	}
	return count;
}

static ssize_t md_cd_parameter_show(struct ccci_modem *md, char *buf)
{
    int count = 0;

    count += snprintf(buf+count, 128, "CHECKSUM_SIZE=%d\n", CHECKSUM_SIZE);
    count += snprintf(buf+count, 128, "PACKET_HISTORY_DEPTH=%d\n", PACKET_HISTORY_DEPTH);
    return count;
}

static ssize_t md_cd_parameter_store(struct ccci_modem *md, const char *buf, size_t count)
{
    return count;
}

CCCI_MD_ATTR(NULL, dump, 0660, md_cd_dump_show, md_cd_dump_store);
CCCI_MD_ATTR(NULL, control, 0660, md_cd_control_show, md_cd_control_store);
CCCI_MD_ATTR(NULL, filter, 0660, md_cd_filter_show, md_cd_filter_store);
CCCI_MD_ATTR(NULL, parameter, 0660, md_cd_parameter_show, md_cd_parameter_store);

static void md_cd_sysfs_init(struct ccci_modem *md)
{
    int ret;
    ccci_md_attr_dump.modem = md;
    ret = sysfs_create_file(&md->kobj, &ccci_md_attr_dump.attr);
    if(ret)
        CCCI_ERR_MSG(md->index, TAG, "fail to add sysfs node %s %d\n",
        ccci_md_attr_dump.attr.name, ret);

    ccci_md_attr_control.modem = md;
    ret = sysfs_create_file(&md->kobj, &ccci_md_attr_control.attr);
    if(ret)
        CCCI_ERR_MSG(md->index, TAG, "fail to add sysfs node %s %d\n",
        ccci_md_attr_control.attr.name, ret);

    ccci_md_attr_parameter.modem = md;
    ret = sysfs_create_file(&md->kobj, &ccci_md_attr_parameter.attr);
    if(ret)
        CCCI_ERR_MSG(md->index, TAG, "fail to add sysfs node %s %d\n",
        ccci_md_attr_parameter.attr.name, ret);

    ccci_md_attr_filter.modem = md;
    ret = sysfs_create_file(&md->kobj, &ccci_md_attr_filter.attr);
    if(ret)
        CCCI_ERR_MSG(md->index, TAG, "fail to add sysfs node %s %d\n",
        ccci_md_attr_filter.attr.name, ret);
}


#ifdef ENABLE_CLDMA_AP_SIDE
static struct syscore_ops md_cldma_sysops = {
    .suspend = ccci_modem_syssuspend,
    .resume = ccci_modem_sysresume,
};
#endif

#define DMA_BIT_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))
static u64 cldma_dmamask = DMA_BIT_MASK((sizeof(unsigned long)<<3));
static int ccci_modem_probe(struct platform_device *plat_dev)
{
    struct ccci_modem *md;
    struct md_cd_ctrl *md_ctrl;
    int md_id,i;
    struct ccci_dev_cfg dev_cfg;
    int ret;
    int sram_size;
    struct md_hw_info *md_hw;

    // Allocate modem hardware info structure memory
    md_hw = kzalloc(sizeof(struct md_hw_info), GFP_KERNEL);
    if(md_hw == NULL) {
        CCCI_INF_MSG(-1, TAG, "md_cldma_probe:alloc md hw mem fail\n");
        return -1;
    }

    ret = md_cd_get_modem_hw_info(plat_dev, &dev_cfg, md_hw);
    if(ret != 0) {
        CCCI_INF_MSG(-1, TAG, "md_cldma_probe:get hw info fail(%d)\n", ret);
        kfree(md_hw);
        md_hw = NULL;
        return -1;
    }

    // Allocate md ctrl memory and do initialize
    md = ccci_allocate_modem(sizeof(struct md_cd_ctrl));
    if(md == NULL) {
        CCCI_INF_MSG(-1, TAG, "md_cldma_probe:alloc modem ctrl mem fail\n");
        kfree(md_hw);
        md_hw = NULL;
        return -1;
    }

    md->index = md_id = dev_cfg.index;
    md->major = dev_cfg.major;
    md->minor_base = dev_cfg.minor_base;
    md->capability = dev_cfg.capability;
    md->plat_dev = plat_dev;
    md->plat_dev->dev.dma_mask=&cldma_dmamask;
    md->plat_dev->dev.coherent_dma_mask = cldma_dmamask;
    CCCI_INF_MSG(md_id, TAG, "modem CLDMA module probe\n");
    // init modem structure
    md->ops = &md_cd_ops;
    CCCI_INF_MSG(md_id, TAG, "md_cldma_probe:md=%p,md->private_data=%p\n",md,md->private_data);

    // init modem private data
    md_ctrl = (struct md_cd_ctrl *)md->private_data;
	md_ctrl->modem = md;
    md_ctrl->hw_info = md_hw;
    md_ctrl->txq_active = 0;
    md_ctrl->rxq_active = 0;
    snprintf(md_ctrl->trm_wakelock_name, sizeof(md_ctrl->trm_wakelock_name), "md%d_cldma_trm", md_id+1);
    wake_lock_init(&md_ctrl->trm_wake_lock, WAKE_LOCK_SUSPEND, md_ctrl->trm_wakelock_name);
    snprintf(md_ctrl->peer_wakelock_name, sizeof(md_ctrl->peer_wakelock_name), "md%d_cldma_peer", md_id+1);
    wake_lock_init(&md_ctrl->peer_wake_lock, WAKE_LOCK_SUSPEND, md_ctrl->peer_wakelock_name);
    md_ctrl->tgpd_dmapool = dma_pool_create("CLDMA_TGPD_DMA",
                        &plat_dev->dev,
                        sizeof(struct cldma_tgpd),
                        16,
                        0);
    md_ctrl->rgpd_dmapool = dma_pool_create("CLDMA_RGPD_DMA",
                        &plat_dev->dev,
                        sizeof(struct cldma_rgpd),
                        16,
                        0);
    INIT_WORK(&md_ctrl->ccif_work, md_cd_ccif_work);
    INIT_DELAYED_WORK(&md_ctrl->ccif_delayed_work, md_cd_ccif_delayed_work);
    init_timer(&md_ctrl->bus_timeout_timer);
    md_ctrl->bus_timeout_timer.function = md_cd_ap2md_bus_timeout_timer_func;
    md_ctrl->bus_timeout_timer.data = (unsigned long)md;
#ifdef ENABLE_HS1_POLLING_TIMER
		init_timer(&md_ctrl->hs1_polling_timer);
		md_ctrl->hs1_polling_timer.function = md_cd_hs1_polling_timer_func;
		md_ctrl->hs1_polling_timer.data =(unsigned long)md;
#endif

#ifdef ENABLE_CLDMA_TIMER
    for(i=0;i<CLDMA_TXQ_NUM;i++){
        init_timer(&md_ctrl->cldma_tx_timeout_timer[i]);
        md_ctrl->cldma_tx_timeout_timer[i].function = cldma_timeout_timer_func;
        md_ctrl->cldma_tx_timeout_timer[i].data = (unsigned long)md;
        md_ctrl->cldma_tx_timeout_timer_start[i]=0;
        md_ctrl->cldma_tx_timeout_timer_end[i]=0;
    }
#endif
    spin_lock_init(&md_ctrl->cldma_timeout_lock);

	md_ctrl->cldma_irq_worker = alloc_workqueue("md%d_cldma_worker", WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1, md->index+1);
	INIT_WORK(&md_ctrl->cldma_irq_work, cldma_irq_work);
    md_ctrl->channel_id = 0;
    atomic_set(&md_ctrl->reset_on_going, 0);
    atomic_set(&md_ctrl->wdt_enabled, 0);
    INIT_WORK(&md_ctrl->wdt_work, md_cd_wdt_work);
#if TRAFFIC_MONITOR_INTERVAL
    init_timer(&md_ctrl->traffic_monitor);
    md_ctrl->traffic_monitor.function = md_cd_traffic_monitor_func;
    md_ctrl->traffic_monitor.data = (unsigned long)md;
#endif

	memset(md_ctrl->logic_ch_pkt_cnt, 0, sizeof(md_ctrl->logic_ch_pkt_cnt));

    // register modem
    ccci_register_modem(md);
#ifdef ENABLE_CLDMA_AP_SIDE
//register SYS CORE suspend resume call back
    register_syscore_ops(&md_cldma_sysops);
#endif
    // add sysfs entries
    md_cd_sysfs_init(md);
    // hook up to device
    plat_dev->dev.platform_data = md;
#ifndef FEATURE_FPGA_PORTING
    // init CCIF
    sram_size = md_ctrl->hw_info->sram_size;
    cldma_write32(md_ctrl->ap_ccif_base, APCCIF_CON, 0x01); // arbitration
    cldma_write32(md_ctrl->ap_ccif_base, APCCIF_ACK, 0xFFFF);
    for(i=0; i<sram_size/sizeof(u32); i++) {
        cldma_write32(md_ctrl->ap_ccif_base, APCCIF_CHDATA+i*sizeof(u32), 0);
    }
#endif

#ifdef FEATURE_FPGA_PORTING
    md_cd_clear_all_queue(md, OUT);
    md_cd_clear_all_queue(md, IN);
    ccci_reset_seq_num(md);
    CCCI_INF_MSG(md_id, TAG, "cldma_reset\n");
    cldma_reset(md);
    CCCI_INF_MSG(md_id, TAG, "cldma_start\n");
    cldma_start(md);
    CCCI_INF_MSG(md_id, TAG, "wait md package...\n");    
    {
        struct cldma_tgpd *md_tgpd;
        struct ccci_header *md_ccci_h;
        unsigned int md_tgpd_addr;
        CCCI_INF_MSG(md_id, TAG, "Write md check sum\n");
        cldma_write32(md_ctrl->cldma_md_pdn_base, CLDMA_AP_UL_CHECKSUM_CHANNEL_ENABLE, 0);
        cldma_write32(md_ctrl->cldma_md_ao_base, CLDMA_AP_SO_CHECKSUM_CHANNEL_ENABLE, 0);
        
        CCCI_INF_MSG(md_id, TAG, "Build md ccif_header\n");
        md_ccci_h=(struct ccci_header *)md->mem_layout.md_region_vir;
        memset(md_ccci_h,0,sizeof(struct ccci_header));
        md_ccci_h->reserved = MD_INIT_CHK_ID;
        CCCI_INF_MSG(md_id, TAG, "Build md cldma_tgpd\n");
        md_tgpd  =(struct cldma_tgpd *)(md->mem_layout.md_region_vir+sizeof(struct ccci_header));
        memset(md_tgpd,0,sizeof(struct cldma_tgpd));
        // update GPD
        md_tgpd->data_buff_bd_ptr = 0;
        md_tgpd->data_buff_len = sizeof(struct ccci_header);
        md_tgpd->debug_id = 0;
        // checksum of GPD
        caculate_checksum((char *)md_tgpd, md_tgpd->gpd_flags | 0x1);
        // resume Tx queue
        cldma_write8(&md_tgpd->gpd_flags, 0, cldma_read8(&md_tgpd->gpd_flags, 0) | 0x1);
        md_tgpd_addr=0+sizeof(struct ccci_header);
        
        cldma_write32(md_ctrl->cldma_md_pdn_base, CLDMA_AP_UL_START_ADDR_0, md_tgpd_addr);
#ifdef ENABLE_CLDMA_AP_SIDE
        cldma_write32(md_ctrl->cldma_md_ao_base, CLDMA_AP_UL_START_ADDR_BK_0, md_tgpd_addr);
#endif
        CCCI_INF_MSG(md_id, TAG, "Start md_tgpd_addr = 0x%x\n", cldma_read32(md_ctrl->cldma_md_pdn_base, CLDMA_AP_UL_START_ADDR_0));
        cldma_write32(md_ctrl->cldma_md_pdn_base, CLDMA_AP_UL_START_CMD, CLDMA_BM_ALL_QUEUE&(1<<0));
        CCCI_INF_MSG(md_id, TAG, "Start md_tgpd_start cmd = 0x%x\n",cldma_read32(md_ctrl->cldma_md_pdn_base, CLDMA_AP_UL_START_CMD)); // dummy read to create a non-buffable write
        CCCI_INF_MSG(md_id, TAG, "Start md cldma_tgpd done\n");
    }
#endif
    return 0;
}

static struct dev_pm_ops ccci_modem_pm_ops = {
    .suspend = ccci_modem_pm_suspend,
    .resume = ccci_modem_pm_resume,
    .freeze = ccci_modem_pm_suspend,
    .thaw = ccci_modem_pm_resume,
    .poweroff = ccci_modem_pm_suspend,
    .restore = ccci_modem_pm_resume,
    .restore_noirq = ccci_modem_pm_restore_noirq,
};

#ifdef CONFIG_OF
static const struct of_device_id mdcldma_of_ids[] = {
    {.compatible = "mediatek,MDCLDMA", },
    {}
};
#endif

static struct platform_driver modem_cldma_driver =
{
    .driver = {
        .name = "cldma_modem",
#ifdef CONFIG_OF
        .of_match_table = mdcldma_of_ids,
#endif

#ifdef CONFIG_PM
        .pm = &ccci_modem_pm_ops,
#endif
    },
    .probe = ccci_modem_probe,
    .remove = ccci_modem_remove,
    .shutdown = ccci_modem_shutdown,
    .suspend = ccci_modem_suspend,
    .resume = ccci_modem_resume,
};

static int __init modem_cd_init(void)
{
    int ret;

    ret = platform_driver_register(&modem_cldma_driver);
    if (ret) {
        CCCI_ERR_MSG(-1, TAG, "clmda modem platform driver register fail(%d)\n", ret);
        return ret;
    }
    return 0;
}

module_init(modem_cd_init);

MODULE_AUTHOR("Xiao Wang <xiao.wang@mediatek.com>");
MODULE_DESCRIPTION("CLDMA modem driver v0.1");
MODULE_LICENSE("GPL");
