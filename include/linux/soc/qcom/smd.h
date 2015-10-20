#ifndef __QCOM_SMD_H__
#define __QCOM_SMD_H__

#include <linux/device.h>
#include <linux/mod_devicetable.h>

struct qcom_smd;
struct qcom_smd_lookup;
struct qcom_smd_device;

typedef int (*qcom_smd_cb_t)(struct qcom_smd_device *, const void *, size_t);

/*
 * SMD channel states.
 */
enum smd_channel_state {
	SMD_CHANNEL_CLOSED,
	SMD_CHANNEL_OPENING,
	SMD_CHANNEL_OPENED,
	SMD_CHANNEL_FLUSHING,
	SMD_CHANNEL_CLOSING,
	SMD_CHANNEL_RESET,
	SMD_CHANNEL_RESET_OPENING
};

struct qcom_smd_channel {
	struct qcom_smd_edge *edge;

	struct qcom_smd_device *qsdev;

	char *name;
	enum smd_channel_state state;
	enum smd_channel_state remote_state;

	struct smd_channel_info *tx_info;
	struct smd_channel_info *rx_info;

	struct smd_channel_info_word *tx_info_word;
	struct smd_channel_info_word *rx_info_word;

	struct mutex tx_lock;
	wait_queue_head_t fblockread_event;

	void *tx_fifo;
	void *rx_fifo;
	int fifo_size;

	void *bounce_buffer;
	qcom_smd_cb_t cb;

	spinlock_t recv_lock;

	int pkt_size;

	struct list_head list;
	struct list_head dev_list;
};

/**
 * struct qcom_smd_id - struct used for matching a smd device
 * @name:	name of the channel
 */
struct qcom_smd_id {
	char name[20];
};

/**
 * struct qcom_smd_device - smd device struct
 * @dev:	the device struct
 * @channel:	handle to the smd channel for this device
 */
struct qcom_smd_device {
	struct device dev;
	struct qcom_smd_channel *channel;
};

/**
 * struct qcom_smd_driver - smd driver struct
 * @driver:	underlying device driver
 * @smd_match_table: static channel match table
 * @probe:	invoked when the smd channel is found
 * @remove:	invoked when the smd channel is closed
 * @callback:	invoked when an inbound message is received on the channel,
 *		should return 0 on success or -EBUSY if the data cannot be
 *		consumed at this time
 */
struct qcom_smd_driver {
	struct device_driver driver;
	const struct qcom_smd_id *smd_match_table;

	int (*probe)(struct qcom_smd_device *dev);
	void (*remove)(struct qcom_smd_device *dev);
	qcom_smd_cb_t callback;
};

int qcom_smd_driver_register(struct qcom_smd_driver *drv);
void qcom_smd_driver_unregister(struct qcom_smd_driver *drv);

#define module_qcom_smd_driver(__smd_driver) \
	module_driver(__smd_driver, qcom_smd_driver_register, \
		      qcom_smd_driver_unregister)

int qcom_smd_send(struct qcom_smd_channel *channel, const void *data, int len);

#endif
