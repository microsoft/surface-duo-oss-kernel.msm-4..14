/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 *
 */
#ifndef _MHI_H_
#define _MHI_H_

#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/mutex.h>
#include <linux/rwlock_types.h>
#include <linux/slab.h>
#include <linux/spinlock_types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

struct mhi_chan;
struct mhi_event;
struct mhi_ctxt;
struct mhi_cmd;
struct mhi_buf_info;

/**
 * enum mhi_callback - MHI callback
 * @MHI_CB_IDLE: MHI entered idle state
 * @MHI_CB_PENDING_DATA: New data available for client to process
 * @MHI_CB_LPM_ENTER: MHI host entered low power mode
 * @MHI_CB_LPM_EXIT: MHI host about to exit low power mode
 * @MHI_CB_EE_RDDM: MHI device entered RDDM execution enviornment
 */
enum mhi_callback {
	MHI_CB_IDLE,
	MHI_CB_PENDING_DATA,
	MHI_CB_LPM_ENTER,
	MHI_CB_LPM_EXIT,
	MHI_CB_EE_RDDM,
};

/**
 * enum mhi_flags - Transfer flags
 * @MHI_EOB: End of buffer for bulk transfer
 * @MHI_EOT: End of transfer
 * @MHI_CHAIN: Linked transfer
 */
enum mhi_flags {
	MHI_EOB,
	MHI_EOT,
	MHI_CHAIN,
};

/**
 * enum mhi_device_type - Device types
 * @MHI_DEVICE_XFER: Handles data transfer
 * @MHI_DEVICE_TIMESYNC: Use for timesync feature
 * @MHI_DEVICE_CONTROLLER: Control device
 */
enum mhi_device_type {
	MHI_DEVICE_XFER,
	MHI_DEVICE_TIMESYNC,
	MHI_DEVICE_CONTROLLER,
};

/**
 * enum mhi_ee_type - Execution environment types
 * @MHI_EE_PBL: Primary Bootloader
 * @MHI_EE_SBL: Secondary Bootloader
 * @MHI_EE_AMSS: Modem, aka the primary runtime EE
 * @MHI_EE_BHIE: BHI extension
 * @MHI_EE_RDDM: Ram dump download mode
 * @MHI_EE_PTHRU: Passthrough
 * @MHI_EE_EDL: Embedded downloader
 */
enum mhi_ee_type {
	MHI_EE_PBL = 0x0,
	MHI_EE_SBL = 0x1,
	MHI_EE_AMSS = 0x2,
	MHI_EE_BHIE = 0x3,
	MHI_EE_RDDM = 0x4,
	MHI_EE_PTHRU = 0x5,
	MHI_EE_EDL = 0x6,
	MHI_EE_MAX_SUPPORTED = MHI_EE_EDL,
	MHI_EE_DISABLE_TRANSITION, /* local EE, not related to mhi spec */
	MHI_EE_MAX,
};

/**
 * enum mhi_buf_type - Accepted buffer type for the channel
 * @MHI_BUF_RAW: Raw buffer
 * @MHI_BUF_SKB: SKB struct
 * @MHI_BUF_SCLIST: Scatter-gather list
 * @MHI_BUF_NOP: CPU offload channel, host does not accept transfer
 */
enum mhi_buf_type {
	MHI_BUF_RAW,
	MHI_BUF_SKB,
	MHI_BUF_SCLIST,
	MHI_BUF_NOP,
};

/**
 * enum mhi_er_data_type - Event ring data types
 * @MHI_ER_DATA: Only client data over this ring
 * @MHI_ER_CTRL: MHI control data and client data
 * @MHI_ER_TSYNC: Time sync events
 */
enum mhi_er_data_type {
	MHI_ER_DATA,
	MHI_ER_CTRL,
	MHI_ER_TSYNC,
};

/**
 * enum mhi_db_brst_mode - Doorbell mode
 * @MHI_DB_BRST_DISABLE: Burst mode disable
 * @MHI_DB_BRST_ENABLE: Burst mode enable
 */
enum mhi_db_brst_mode {
	MHI_DB_BRST_DISABLE = 0x2,
	MHI_DB_BRST_ENABLE = 0x3,
};

/**
 * struct mhi_channel_config - Channel configuration structure for controller
 * @num: The number assigned to this channel
 * @name: The name of this channel
 * @num_elements: The number of elements that can be queued to this channel
 * @event_ring: The event rung index that services this channel
 * @dir: Direction that data may flow on this channel
 * @ee: Under which execution environment is this channel valid
 * @pollcfg: Polling configuration for burst mode.  0 is default.  milliseconds
	     for UL channels, multiple of 8 ring elements for DL channels
 * @data_type: Data type accepted by this channel
 * @doorbell: Doorbell mode
 * @lpm_notify: The channel master requires low power mode notifications
 * @offload_channel: The client manages the channel completely
 * @doorbell_mode_switch: Channel switches to doorbell mode on M0 transition
 * @auto_queue: Framework will automatically queue buffers for DL traffic
 * @auto_start: Automatically start (open) this channel
 */
struct mhi_channel_config {
	u32 num;
	char *name;
	u32 num_elements;
	u32 event_ring;
	enum dma_data_direction dir;
	enum mhi_ee_type ee;
	u32 pollcfg;
	enum mhi_buf_type data_type;
	enum mhi_db_brst_mode doorbell;
	bool lpm_notify;
	bool offload_channel;
	bool doorbell_mode_switch;
	bool auto_queue;
	bool auto_start;
};

/**
 * struct mhi_event_config - Event ring configuration structure for controller
 * @num_elements: The number of elements that can be queued to this ring
 * @irq_moderation_ms: Delay irq for additional events to be aggregated
 * @irq: IRQ associated with this ring
 * @channel: Dedicated channel number. U32_MAX indicates a non-dedicated ring
 * @mode: Doorbell mode
 * @data_type: Type of data this ring will process
 * @hardware_event: This ring is associated with hardware channels
 * @client_managed: This ring is client managed
 * @offload_channel: This ring is associated with an offloaded channel
 * @priority: Priority of this ring. Use 1 for now
 */
struct mhi_event_config {
	u32 num_elements;
	u32 irq_moderation_ms;
	u32 irq;
	u32 channel;
	enum mhi_db_brst_mode mode;
	enum mhi_er_data_type data_type;
	bool hardware_event;
	bool client_managed;
	bool offload_channel;
	u32 priority;
};

/**
 * struct mhi_controller_config - Root MHI controller configuration
 * @max_channels: Maximum number of channels supported
 * @timeout_ms: Timeout value for operations. 0 means use default
 * @use_bounce_buf: Use a bounce buffer pool due to limited DDR access
 * @buf_len: Size of automatically allocated buffers. 0 means use default
 * @num_channels: Number of channels defined in @ch_cfg
 * @ch_cfg: Array of defined channels
 * @num_events: Number of event rings defined in @event_cfg
 * @event_cfg: Array of defined event rings
 */
struct mhi_controller_config {
	u32 max_channels;
	u32 timeout_ms;
	bool use_bounce_buf;
	u32 buf_len;
	u32 num_channels;
	struct mhi_channel_config *ch_cfg;
	u32 num_events;
	struct mhi_event_config *event_cfg;
};

/**
 * struct mhi_controller - Master MHI controller structure
 * @name: Name of the controller
 * @dev: Driver model device node for the controller
 * @mhi_dev: MHI device instance for the controller
 * @dev_id: Device ID of the controller
 * @bus_id: Physical bus instance used by the controller
 * @regs: Base address of MHI MMIO register space
 * @iova_start: IOMMU starting address for data
 * @iova_stop: IOMMU stop address for data
 * @fw_image: Firmware image name for normal booting
 * @edl_image: Firmware image name for emergency download mode
 * @fbc_download: MHI host needs to do complete image transfer
 * @rddm_size: RAM dump size that host should allocate for debugging purpose
 * @sbl_size: SBL image size
 * @seg_len: BHIe vector size
 * @max_chan: Maximum number of channels the controller supports
 * @mhi_chan: Points to the channel configuration table
 * @lpm_chans: List of channels that require LPM notifications
 * @total_ev_rings: Total # of event rings allocated
 * @hw_ev_rings: Number of hardware event rings
 * @sw_ev_rings: Number of software event rings
 * @nr_irqs_req: Number of IRQs required to operate
 * @nr_irqs: Number of IRQ allocated by bus master
 * @irq: base irq # to request
 * @mhi_event: MHI event ring configurations table
 * @mhi_cmd: MHI command ring configurations table
 * @mhi_ctxt: MHI device context, shared memory between host and device
 * @timeout_ms: Timeout in ms for state transitions
 * @pm_mutex: Mutex for suspend/resume operation
 * @pre_init: MHI host needs to do pre-initialization before power up
 * @pm_lock: Lock for protecting MHI power management state
 * @pm_state: MHI power management state
 * @ee: MHI device execution environment
 * @dev_state: MHI device state
 * @wake_set: Device wakeup set flag
 * @dev_wake: Device wakeup count
 * @alloc_size: Total memory allocations size of the controller
 * @transition_list: List of MHI state transitions
 * @wlock: Lock for protecting device wakeup
 * @M0: M0 state counter for debugging
 * @M2: M2 state counter for debugging
 * @M3: M3 state counter for debugging
 * @st_worker: State transition worker
 * @fw_worker: Firmware download worker
 * @syserr_worker: System error worker
 * @state_event: State change event
 * @status_cb: CB function to notify various power states to bus master
 * @link_status: CB function to query link status of the device
 * @wake_get: CB function to assert device wake
 * @wake_put: CB function to de-assert device wake
 * @runtime_get: CB function to controller runtime resume
 * @runtimet_put: CB function to decrement pm usage
 * @lpm_disable: CB function to request disable link level low power modes
 * @lpm_enable: CB function to request enable link level low power modes again
 * @map_single: CB function to create TRE buffer
 * @unmap_single: CB function to destroy TRE buffer
 * @dtr_dev: Controller supports IOCTL operation using DTR signal
 * @bounce_buf: Use of bounce buffer
 * @buffer_len: Bounce buffer length
 * @priv_data: Points to bus master's private data
 */
struct mhi_controller {
	const char *name;
	struct device *dev;
	struct mhi_device *mhi_dev;
	u32 dev_id;
	u32 bus_id;
	void __iomem *regs;
	dma_addr_t iova_start;
	dma_addr_t iova_stop;
	const char *fw_image;
	const char *edl_image;
	bool fbc_download;
	size_t rddm_size;
	size_t sbl_size;
	size_t seg_len;
	u32 max_chan;
	struct mhi_chan *mhi_chan;
	struct list_head lpm_chans;
	u32 total_ev_rings;
	u32 hw_ev_rings;
	u32 sw_ev_rings;
	u32 nr_irqs_req;
	u32 nr_irqs;
	int *irq;

	struct mhi_event *mhi_event;
	struct mhi_cmd *mhi_cmd;
	struct mhi_ctxt *mhi_ctxt;

	u32 timeout_ms;
	struct mutex pm_mutex;
	bool pre_init;
	rwlock_t pm_lock;
	u32 pm_state;
	u32 ee;
	u32 dev_state;
	bool wake_set;
	atomic_t dev_wake;
	atomic_t alloc_size;
	struct list_head transition_list;
	spinlock_t transition_lock;
	spinlock_t wlock;
	u32 M0, M2, M3;
	struct work_struct st_worker;
	struct work_struct fw_worker;
	struct work_struct syserr_worker;
	wait_queue_head_t state_event;

	void (*status_cb)(struct mhi_controller *mhi_cntrl, void *priv,
			  enum mhi_callback cb);
	int (*link_status)(struct mhi_controller *mhi_cntrl, void *priv);
	void (*wake_get)(struct mhi_controller *mhi_cntrl, bool override);
	void (*wake_put)(struct mhi_controller *mhi_cntrl, bool override);
	int (*runtime_get)(struct mhi_controller *mhi_cntrl, void *priv);
	void (*runtime_put)(struct mhi_controller *mhi_cntrl, void *priv);
	void (*lpm_disable)(struct mhi_controller *mhi_cntrl, void *priv);
	void (*lpm_enable)(struct mhi_controller *mhi_cntrl, void *priv);
	int (*map_single)(struct mhi_controller *mhi_cntrl,
			  struct mhi_buf_info *buf);
	void (*unmap_single)(struct mhi_controller *mhi_cntrl,
			     struct mhi_buf_info *buf);

	struct mhi_device *dtr_dev;
	bool bounce_buf;
	size_t buffer_len;
	void *priv_data;
};

/**
 * struct mhi_device - Structure representing a MHI device which binds
 *                     to channels
 * @dev: Driver model device node for the MHI device
 * @tiocm: Device current terminal settings
 * @id: Pointer to MHI device ID struct
 * @chan_name: Name of the channel to which the device binds
 * @mhi_cntrl: Controller the device belongs to
 * @ul_chan: UL channel for the device
 * @dl_chan: DL channel for the device
 * @dev_wake: Device wakeup counter
 * @dev_type: MHI device type
 */
struct mhi_device {
	struct device dev;
	u32 tiocm;
	const struct mhi_device_id *id;
	const char *chan_name;
	struct mhi_controller *mhi_cntrl;
	struct mhi_chan *ul_chan;
	struct mhi_chan *dl_chan;
	atomic_t dev_wake;
	enum mhi_device_type dev_type;
};

/**
 * struct mhi_result - Completed buffer information
 * @buf_addr: Address of data buffer
 * @dir: Channel direction
 * @bytes_xfer: # of bytes transferred
 * @transaction_status: Status of last transaction
 */
struct mhi_result {
	void *buf_addr;
	enum dma_data_direction dir;
	size_t bytes_xferd;
	int transaction_status;
};

/**
 * struct mhi_driver - Structure representing a MHI client driver
 * @probe: CB function for client driver probe function
 * @remove: CB function for client driver remove function
 * @ul_xfer_cb: CB function for UL data transfer
 * @dl_xfer_cb: CB function for DL data transfer
 * @status_cb: CB functions for asynchronous status
 * @driver: Device driver model driver
 */
struct mhi_driver {
	const struct mhi_device_id *id_table;
	int (*probe)(struct mhi_device *mhi_dev,
		     const struct mhi_device_id *id);
	void (*remove)(struct mhi_device *mhi_dev);
	void (*ul_xfer_cb)(struct mhi_device *mhi_dev,
			   struct mhi_result *result);
	void (*dl_xfer_cb)(struct mhi_device *mhi_dev,
			   struct mhi_result *result);
	void (*status_cb)(struct mhi_device *mhi_dev, enum mhi_callback mhi_cb);
	struct device_driver driver;
};

#define to_mhi_driver(drv) container_of(drv, struct mhi_driver, driver)
#define to_mhi_device(dev) container_of(dev, struct mhi_device, dev)

/**
 * mhi_controller_set_devdata - Set MHI controller private data
 * @mhi_cntrl: MHI controller to set data
 */
static inline void mhi_controller_set_devdata(struct mhi_controller *mhi_cntrl,
					 void *priv)
{
	mhi_cntrl->priv_data = priv;
}

/**
 * mhi_controller_get_devdata - Get MHI controller private data
 * @mhi_cntrl: MHI controller to get data
 */
static inline void *mhi_controller_get_devdata(struct mhi_controller *mhi_cntrl)
{
	return mhi_cntrl->priv_data;
}

/**
 * mhi_free_controller - Free MHI controller resources
 * @mhi_cntrl: MHI controller to free
 */
static inline void mhi_free_controller(struct mhi_controller *mhi_cntrl)
{
	kfree(mhi_cntrl);
}

/**
 * mhi_driver_register - Register driver with MHI framework
 * @mhi_drv: Driver associated with the device
 */
int mhi_driver_register(struct mhi_driver *mhi_drv);

/**
 * mhi_driver_unregister - Unregister a driver for mhi_devices
 * @mhi_drv: Driver associated with the device
 */
void mhi_driver_unregister(struct mhi_driver *mhi_drv);

/**
 * mhi_alloc_controller - Allocate mhi_controller structure.
 */
struct mhi_controller *mhi_alloc_controller(void);

/**
 * mhi_queue_transfer - Queue a buffer to device
 * @mhi_dev: Device associated with the channels
 * @dir: Data direction
 * @buf: Data buffer (skb for hardware channels)
 * @len: Size in bytes
 * @mflags: Interrupt flags for the device
 */
int mhi_queue_transfer(struct mhi_device *mhi_dev, enum dma_data_direction dir,
		       void *buf, size_t len, enum mhi_flags mflags);

/**
 * mhi_register_controller - Register MHI controller
 * @mhi_cntrl: MHI controller to register
 * @config: Configuration to use for the controller
 */
int mhi_register_controller(struct mhi_controller *mhi_cntrl,
			    struct mhi_controller_config *config);

/**
 * mhi_unregister_controller - Unregister MHI controller
 * @mhi_cntrl: MHI controller to unregister
 */
void mhi_unregister_controller(struct mhi_controller *mhi_cntrl);

#endif /* _MHI_H_ */
