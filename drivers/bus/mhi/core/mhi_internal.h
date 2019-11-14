/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 *
 */

#ifndef _MHI_INT_H
#define _MHI_INT_H

extern struct bus_type mhi_bus_type;

/* MHI transfer completion events */
enum mhi_ev_ccs {
	MHI_EV_CC_INVALID = 0x0,
	MHI_EV_CC_SUCCESS = 0x1,
	MHI_EV_CC_EOT = 0x2,
	MHI_EV_CC_OVERFLOW = 0x3,
	MHI_EV_CC_EOB = 0x4,
	MHI_EV_CC_OOB = 0x5,
	MHI_EV_CC_DB_MODE = 0x6,
	MHI_EV_CC_UNDEFINED_ERR = 0x10,
	MHI_EV_CC_BAD_TRE = 0x11,
};

enum mhi_ch_state {
	MHI_CH_STATE_DISABLED = 0x0,
	MHI_CH_STATE_ENABLED = 0x1,
	MHI_CH_STATE_RUNNING = 0x2,
	MHI_CH_STATE_SUSPENDED = 0x3,
	MHI_CH_STATE_STOP = 0x4,
	MHI_CH_STATE_ERROR = 0x5,
};

#define MHI_INVALID_BRSTMODE(mode) (mode != MHI_DB_BRST_DISABLE && \
				    mode != MHI_DB_BRST_ENABLE)

#define NR_OF_CMD_RINGS			1
#define CMD_EL_PER_RING			128
#define PRIMARY_CMD_RING		0
#define MHI_MAX_MTU			0xffff

enum mhi_er_type {
	MHI_ER_TYPE_INVALID = 0x0,
	MHI_ER_TYPE_VALID = 0x1,
};

struct db_cfg {
	bool reset_req;
	bool db_mode;
	u32 pollcfg;
	enum mhi_db_brst_mode brstmode;
	dma_addr_t db_val;
	void (*process_db)(struct mhi_controller *mhi_cntrl,
			   struct db_cfg *db_cfg, void __iomem *io_addr,
			   dma_addr_t db_val);
};

struct mhi_ring {
	dma_addr_t dma_handle;
	dma_addr_t iommu_base;
	u64 *ctxt_wp; /* point to ctxt wp */
	void *pre_aligned;
	void *base;
	void *rp;
	void *wp;
	size_t el_size;
	size_t len;
	size_t elements;
	size_t alloc_size;
	void __iomem *db_addr;
};

struct mhi_cmd {
	struct mhi_ring ring;
	spinlock_t lock;
};

struct mhi_buf_info {
	dma_addr_t p_addr;
	void *v_addr;
	void *bb_addr;
	void *wp;
	size_t len;
	void *cb_buf;
	enum dma_data_direction dir;
};

struct mhi_event {
	u32 er_index;
	u32 intmod;
	u32 irq;
	int chan; /* this event ring is dedicated to a channel */
	u32 priority;
	enum mhi_er_data_type data_type;
	struct mhi_ring ring;
	struct db_cfg db_cfg;
	bool hw_ring;
	bool cl_manage;
	bool offload_ev; /* managed by a device driver */
	spinlock_t lock;
	struct mhi_chan *mhi_chan; /* dedicated to channel */
	struct tasklet_struct task;
	int (*process_event)(struct mhi_controller *mhi_cntrl,
			     struct mhi_event *mhi_event,
			     u32 event_quota);
	struct mhi_controller *mhi_cntrl;
};

struct mhi_chan {
	u32 chan;
	const char *name;
	/*
	 * important, when consuming increment tre_ring first, when releasing
	 * decrement buf_ring first. If tre_ring has space, buf_ring
	 * guranteed to have space so we do not need to check both rings.
	 */
	struct mhi_ring buf_ring;
	struct mhi_ring tre_ring;
	u32 er_index;
	u32 intmod;
	enum dma_data_direction dir;
	struct db_cfg db_cfg;
	enum mhi_ee_type ee;
	enum mhi_buf_type xfer_type;
	enum mhi_ch_state ch_state;
	enum mhi_ev_ccs ccs;
	bool lpm_notify;
	bool configured;
	bool offload_ch;
	bool pre_alloc;
	bool auto_start;
	/* functions that generate the transfer ring elements */
	int (*gen_tre)(struct mhi_controller *mhi_cntrl,
		       struct mhi_chan *mhi_chan, void *buf, void *cb,
		       size_t len, enum mhi_flags flags);
	int (*queue_xfer)(struct mhi_device *mhi_dev, struct mhi_chan *mhi_chan,
			  void *buf, size_t len, enum mhi_flags mflags);
	/* xfer call back */
	struct mhi_device *mhi_dev;
	void (*xfer_cb)(struct mhi_device *mhi_dev, struct mhi_result *result);
	struct mutex mutex;
	struct completion completion;
	rwlock_t lock;
	struct list_head node;
};

/* default MHI timeout */
#define MHI_TIMEOUT_MS (1000)

/* queue transfer buffer */
int mhi_gen_tre(struct mhi_controller *mhi_cntrl, struct mhi_chan *mhi_chan,
		void *buf, void *cb, size_t buf_len, enum mhi_flags flags);
int mhi_queue_buf(struct mhi_device *mhi_dev, struct mhi_chan *mhi_chan,
		  void *buf, size_t len, enum mhi_flags mflags);
int mhi_queue_skb(struct mhi_device *mhi_dev, struct mhi_chan *mhi_chan,
		  void *buf, size_t len, enum mhi_flags mflags);
int mhi_queue_sclist(struct mhi_device *mhi_dev, struct mhi_chan *mhi_chan,
		  void *buf, size_t len, enum mhi_flags mflags);
int mhi_queue_nop(struct mhi_device *mhi_dev, struct mhi_chan *mhi_chan,
		  void *buf, size_t len, enum mhi_flags mflags);

/* register access methods */
void mhi_db_brstmode(struct mhi_controller *mhi_cntrl, struct db_cfg *db_cfg,
		     void __iomem *db_addr, dma_addr_t db_val);
void mhi_db_brstmode_disable(struct mhi_controller *mhi_cntrl,
			     struct db_cfg *db_mode, void __iomem *db_addr,
			     dma_addr_t db_val);

struct mhi_device *mhi_alloc_device(struct mhi_controller *mhi_cntrl);
static inline void mhi_dealloc_device(struct mhi_controller *mhi_cntrl,
				      struct mhi_device *mhi_dev)
{
	kfree(mhi_dev);
}

int mhi_destroy_device(struct device *dev, void *data);
void mhi_create_devices(struct mhi_controller *mhi_cntrl);

int mhi_map_single_no_bb(struct mhi_controller *mhi_cntrl,
			 struct mhi_buf_info *buf_info);
int mhi_map_single_use_bb(struct mhi_controller *mhi_cntrl,
			  struct mhi_buf_info *buf_info);
void mhi_unmap_single_no_bb(struct mhi_controller *mhi_cntrl,
			    struct mhi_buf_info *buf_info);
void mhi_unmap_single_use_bb(struct mhi_controller *mhi_cntrl,
			     struct mhi_buf_info *buf_info);
void mhi_ctrl_ev_task(unsigned long data);
int mhi_process_data_event_ring(struct mhi_controller *mhi_cntrl,
				struct mhi_event *mhi_event, u32 event_quota);
int mhi_process_ctrl_ev_ring(struct mhi_controller *mhi_cntrl,
			     struct mhi_event *mhi_event, u32 event_quota);

/* isr handlers */
irqreturn_t mhi_irq_handler(int irq_number, void *dev);
irqreturn_t mhi_intvec_threaded_handler(int irq_number, void *dev);
irqreturn_t mhi_intvec_handler(int irq_number, void *dev);
void mhi_ev_task(unsigned long data);

#endif /* _MHI_INT_H */
