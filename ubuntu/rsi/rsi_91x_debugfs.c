/**
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived from
 * 	   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rsi_debugfs.h"
#include "rsi_sdio.h"
#include "rsi_mgmt.h"

extern int g_bgscan_enable;

/**
 * rsi_sdio_stats_read() - This function returns the sdio status of the driver.
 * @seq: Pointer to the sequence file structure.
 * @data: Pointer to the data.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_sdio_stats_read(struct seq_file *seq, void *data)
{
	struct rsi_common *common = seq->private;
	struct rsi_hw *adapter = common->priv;
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;

	seq_printf(seq, "total_sdio_interrupts: %d\n",
		   dev->rx_info.sdio_int_counter);
	seq_printf(seq, "sdio_msdu_pending_intr_count: %d\n",
		   dev->rx_info.total_sdio_msdu_pending_intr);
	seq_printf(seq, "sdio_buff_full_count : %d\n",
		   dev->rx_info.buf_full_counter);
	seq_printf(seq, "sdio_buf_semi_full_count %d\n",
		   dev->rx_info.buf_semi_full_counter);
	seq_printf(seq, "sdio_unknown_intr_count: %d\n",
		   dev->rx_info.total_sdio_unknown_intr);
	/* RX Path Stats */
	seq_printf(seq, "BUFFER FULL STATUS  : %d\n",
		   dev->rx_info.buffer_full);
	seq_printf(seq, "SEMI BUFFER FULL STATUS  : %d\n",
		   dev->rx_info.semi_buffer_full);
	seq_printf(seq, "MGMT BUFFER FULL STATUS  : %d\n",
		   dev->rx_info.mgmt_buffer_full);
	seq_printf(seq, "BUFFER FULL COUNTER  : %d\n",
		   dev->rx_info.buf_full_counter);
	seq_printf(seq, "BUFFER SEMI FULL COUNTER  : %d\n",
		   dev->rx_info.buf_semi_full_counter);
	seq_printf(seq, "MGMT BUFFER FULL COUNTER  : %d\n",
		   dev->rx_info.mgmt_buf_full_counter);

	return 0;
}

/**
 * rsi_sdio_stats_open() - This function calls single open function of seq_file
 *			   to open file and read contents from it.
 * @inode: Pointer to the inode structure.
 * @file: Pointer to the file structure.
 *
 * Return: Pointer to the opened file status: 0 on success, ENOMEM on failure.
 */
static int rsi_sdio_stats_open(struct inode *inode,
			       struct file *file)
{
	return single_open(file, rsi_sdio_stats_read, inode->i_private);
}

/**
 * rsi_version_read() - This function gives driver and firmware version number.
 * @seq: Pointer to the sequence file structure.
 * @data: Pointer to the data.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_version_read(struct seq_file *seq, void *data)
{
	struct rsi_common *common = seq->private;

	seq_printf(seq, "Driver : %s\nLMAC   : %d.%d.%d.%d\n",
		   common->driver_ver,
		   common->lmac_ver.major,
		   common->lmac_ver.minor,
		   common->lmac_ver.release_num,
		   common->lmac_ver.patch_num);
	return 0;
}

/**
 * rsi_version_open() - This function calls single open function of seq_file to
 *			open file and read contents from it.
 * @inode: Pointer to the inode structure.
 * @file: Pointer to the file structure.
 *
 * Return: Pointer to the opened file status: 0 on success, ENOMEM on failure.
 */
static int rsi_version_open(struct inode *inode,
			    struct file *file)
{
	return single_open(file, rsi_version_read, inode->i_private);
}

/**
 * rsi_stats_read() - This function return the status of the driver.
 * @seq: Pointer to the sequence file structure.
 * @data: Pointer to the data.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_stats_read(struct seq_file *seq, void *data)
{
	struct rsi_common *common = seq->private;

	unsigned char fsm_state[][32] = {
		"FSM_CARD_NOT_READY",
		"FSM_BOOT_PARAMS_SENT",
		"FSM_EEPROM_READ_MAC_ADDR",
		"FSM_RESET_MAC_SENT",
		"FSM_RADIO_CAPS_SENT",
		"FSM_BB_RF_PROG_SENT",
		"FSM_MAC_INIT_DONE"
	};
	seq_puts(seq, "==> RSI STA DRIVER STATUS <==\n");
	seq_puts(seq, "DRIVER_FSM_STATE: ");

	if (common->fsm_state <= FSM_MAC_INIT_DONE)
		seq_printf(seq, "%s", fsm_state[common->fsm_state]);

	seq_printf(seq, "(%d)\n\n", common->fsm_state);

	/* Mgmt TX Path Stats */
	seq_printf(seq, "total_mgmt_pkt_send : %d\n",
		   common->tx_stats.total_tx_pkt_send[MGMT_SOFT_Q]);
	seq_printf(seq, "total_mgmt_pkt_queued : %d\n",
		   skb_queue_len(&common->tx_queue[MGMT_SOFT_Q]));
	seq_printf(seq, "total_mgmt_pkt_freed  : %d\n",
		   common->tx_stats.total_tx_pkt_freed[MGMT_SOFT_Q]);

	/* Data TX Path Stats */
	seq_printf(seq, "total_data_vo_pkt_send: %8d\t",
		   common->tx_stats.total_tx_pkt_send[VO_Q]);
	seq_printf(seq, "total_data_vo_pkt_queued:  %8d\t",
		   skb_queue_len(&common->tx_queue[VO_Q]));
	seq_printf(seq, "total_vo_pkt_freed: %8d\n",
		   common->tx_stats.total_tx_pkt_freed[VO_Q]);
	seq_printf(seq, "total_data_vi_pkt_send: %8d\t",
		   common->tx_stats.total_tx_pkt_send[VI_Q]);
	seq_printf(seq, "total_data_vi_pkt_queued:  %8d\t",
		   skb_queue_len(&common->tx_queue[VI_Q]));
	seq_printf(seq, "total_vi_pkt_freed: %8d\n",
		   common->tx_stats.total_tx_pkt_freed[VI_Q]);
	seq_printf(seq,  "total_data_be_pkt_send: %8d\t",
		   common->tx_stats.total_tx_pkt_send[BE_Q]);
	seq_printf(seq, "total_data_be_pkt_queued:  %8d\t",
		   skb_queue_len(&common->tx_queue[BE_Q]));
	seq_printf(seq, "total_be_pkt_freed: %8d\n",
		   common->tx_stats.total_tx_pkt_freed[BE_Q]);
	seq_printf(seq, "total_data_bk_pkt_send: %8d\t",
		   common->tx_stats.total_tx_pkt_send[BK_Q]);
	seq_printf(seq, "total_data_bk_pkt_queued:  %8d\t",
		   skb_queue_len(&common->tx_queue[BK_Q]));
	seq_printf(seq, "total_bk_pkt_freed: %8d\n",
		   common->tx_stats.total_tx_pkt_freed[BK_Q]);

	seq_puts(seq, "\n");
	return 0;
}

/**
 * rsi_stats_open() - This function calls single open function of seq_file to
 *		      open file and read contents from it.
 * @inode: Pointer to the inode structure.
 * @file: Pointer to the file structure.
 *
 * Return: Pointer to the opened file status: 0 on success, ENOMEM on failure.
 */
static int rsi_stats_open(struct inode *inode,
			  struct file *file)
{
	return single_open(file, rsi_stats_read, inode->i_private);
}

/**
 * rsi_debug_zone_read() - This function display the currently
 *			enabled debug zones.
 * @seq: Pointer to the sequence file structure.
 * @data: Pointer to the data.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_debug_zone_read(struct seq_file *seq, void *data)
{
	ven_rsi_dbg(FSM_ZONE, "%x: rsi_enabled zone", ven_rsi_zone_enabled);
	seq_printf(seq, "The zones available are %#x\n",
		   ven_rsi_zone_enabled);
	return 0;
}

/**
 * rsi_debug_read() - This function calls single open function of seq_file to
 *		      open file and read contents from it.
 * @inode: Pointer to the inode structure.
 * @file: Pointer to the file structure.
 *
 * Return: Pointer to the opened file status: 0 on success, ENOMEM on failure.
 */
static int rsi_debug_read(struct inode *inode,
			  struct file *file)
{
	return single_open(file, rsi_debug_zone_read, inode->i_private);
}

/**
 * rsi_debug_zone_write() - This function writes into hal queues as per user
 *			    requirement.
 * @filp: Pointer to the file structure.
 * @buff: Pointer to the character buffer.
 * @len: Length of the data to be written into buffer.
 * @data: Pointer to the data.
 *
 * Return: len: Number of bytes read.
 */
static ssize_t rsi_debug_zone_write(struct file *filp,
				    const char __user *buff,
				    size_t len,
				    loff_t *data)
{
	unsigned long dbg_zone;
	int ret;

	if (!len)
		return 0;

	ret = kstrtoul_from_user(buff, len, 16, &dbg_zone);

	if (ret)
		return ret;

	ven_rsi_zone_enabled = dbg_zone;
	return len;
}

/**
 * rsi_bgscan_int_read() - This function display the default bgscan param
 *			   values.
 * @seq: Pointer to the sequence file structure.
 * @data: Pointer to the data.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_bgscan_int_read(struct seq_file *file, void *data)
{
	struct rsi_common *common = file->private;
	struct bgscan_config_params *params = NULL;
	int cnt;

	if (!common) {
		ven_rsi_dbg(ERR_ZONE, "No Interface\n");
		return -ENODEV;
	}
	if (common->iface_down) {
		ven_rsi_dbg(ERR_ZONE, "Interface Down\n");
		return -ENODEV;
	}
	params = &common->bgscan_info;

	seq_printf(file, "%d %d %d %d %d %d %d %d\n",
		   common->bgscan_en,
		   params->bgscan_threshold,
		   params->roam_threshold,
		   params->bgscan_periodicity,
		   params->active_scan_duration,
		   params->passive_scan_duration,
		   params->two_probe,
		   params->num_bg_channels);

	for (cnt = 0; cnt < params->num_bg_channels; cnt++) {
		if (params->channels2scan[cnt] & (BIT(15)))
			seq_printf(file, "%d[DFS] ",
				   (params->channels2scan[cnt] & 0x7FFF));
		else
			seq_printf(file, "%d ", params->channels2scan[cnt]);
	}
	seq_printf(file, "\n");

	return 0;
}

static int rsi_bgscan_read(struct inode *inode, struct file *file)
{
	return single_open(file, rsi_bgscan_int_read, inode->i_private);
}

/**
 * rsi_bgscan_write() - This function gets the bgscan params from user
 *			    and configures to device.
 * @file: Pointer to the file structure.
 * @user_buff: user buffer.
 * @count: Length of the data written in buffer.
 * @ppos: offset.
 *
 * Return: Number of bytes read.
 */
static ssize_t rsi_bgscan_write(struct file *file,
			        const char __user *user_buff,
				size_t count,
				loff_t *ppos)

{
	struct rsi_common *common = file->f_inode->i_private;
	struct rsi_hw *adapter = NULL;
	struct ieee80211_bss_conf *bss = NULL;
	char bgscan_buf[200];
	int bgscan_vals[64] = { 0 };
	int total_bytes, cnt = 0;
	int bytes_read = 0, t_bytes;
	int ret;

	if (!common) {
		ven_rsi_dbg(ERR_ZONE, "No Interface\n");
		return -ENODEV;
	}
	if (common->iface_down) {
		ven_rsi_dbg(ERR_ZONE, "Interface Down\n");
		return -ENODEV;
	}
	adapter = common->priv;
	bss = &adapter->vifs[adapter->sc_nvifs - 1]->bss_conf;

	total_bytes = simple_write_to_buffer(bgscan_buf,
					     sizeof(bgscan_buf) - 1,
					     ppos, user_buff, count);
	if (total_bytes < 1)
		return -EINVAL;

	/* make sure that buf is null terminated */
	bgscan_buf[sizeof(bgscan_buf) - 1] = '\0';

	ret = sscanf(bgscan_buf, "%d%n",
		     (int *)&g_bgscan_enable, &t_bytes);
	if (ret <= 0)
		return -EINVAL;

	if (!g_bgscan_enable) {
		/* return here if bgscan is already disabled */
		if (!common->bgscan_en) {
#ifdef PLATFORM_X86
			ven_rsi_dbg(ERR_ZONE, "bgscan already disabled\n");
#endif
			return total_bytes;
		}

		mutex_lock(&common->mutex);
		if (bss->assoc && !rsi_send_bgscan_params(common, 0)) {
#ifdef PLATFORM_X86
			ven_rsi_dbg(ERR_ZONE, "*** bgscan disabled ***\n");
#endif
			common->bgscan_en = 0;
		}
		mutex_unlock(&common->mutex);

		return total_bytes;
	} else if (common->bgscan_en) {
#ifdef PLATFORM_X86
		ven_rsi_dbg(ERR_ZONE, "bgscan already enabled\n");
#endif
		return total_bytes;
	}

	/* Return if bgscan is already in progress */
	if (common->bgscan_en)
		return total_bytes;

	bytes_read += t_bytes;
	while (1) {
		ret = sscanf(bgscan_buf + bytes_read, "%d%n",
			     &bgscan_vals[cnt++],
			     &t_bytes);
		if (ret <= 0)
			break;
		bytes_read += t_bytes;
		
		if ((bgscan_vals[6] > 0) && (cnt > (6 + bgscan_vals[6])))
			break;
	}
	common->bgscan_info.bgscan_threshold = bgscan_vals[0];
	common->bgscan_info.roam_threshold = bgscan_vals[1];
	common->bgscan_info.bgscan_periodicity = bgscan_vals[2];
	common->bgscan_info.active_scan_duration = bgscan_vals[3];
	common->bgscan_info.passive_scan_duration = bgscan_vals[4];
	common->bgscan_info.two_probe = bgscan_vals[5];
	common->bgscan_info.num_user_channels = bgscan_vals[6];
	memset(&common->bgscan_info.user_channels, 0,
	       (MAX_BGSCAN_CHANNELS * 2));
	common->bgscan_info.num_user_channels = 
		((bgscan_vals[6] > MAX_BGSCAN_CHANNELS) ?
		 MAX_BGSCAN_CHANNELS : bgscan_vals[6]); 
	
	for (cnt = 0; cnt < common->bgscan_info.num_user_channels; cnt++)
		common->bgscan_info.user_channels[cnt] = bgscan_vals[7 + cnt];

#ifdef PLATFORM_X86
	ven_rsi_dbg(INFO_ZONE,
		"bgscan_count = %d, roam_count = %d, periodicity = %d\n",
		common->bgscan_info.bgscan_threshold,
		common->bgscan_info.roam_threshold,
		common->bgscan_info.bgscan_periodicity);
	ven_rsi_dbg(INFO_ZONE,
		"active_scan_dur = %d, passive_scan_dur = %d, two_probe = %d\n",
		common->bgscan_info.active_scan_duration,
		common->bgscan_info.passive_scan_duration,
		common->bgscan_info.two_probe);
	ven_rsi_dbg(INFO_ZONE, "Number of scan channels = %d\n",
		common->bgscan_info.num_user_channels);
	rsi_hex_dump(INFO_ZONE, "bgscan channels",
		     (u8 *)common->bgscan_info.user_channels,
		     common->bgscan_info.num_user_channels * 2);
#endif

	/* If connection is not done don't send bgscan params */
	if (!bss->assoc) {
#ifdef PLATFORM_X86
		ven_rsi_dbg(INFO_ZONE, "Station not connected; skip now\n");
#endif
		return total_bytes;
	}

	/* Send bgscan params to device */
	mutex_lock(&common->mutex);
	if (!rsi_send_bgscan_params(common, 1)) {
		if (!rsi_send_bgscan_probe_req(common)) {
#ifdef PLATFORM_X86
			ven_rsi_dbg(INFO_ZONE, "Background scan started ===>\n");
#endif
			common->bgscan_en = 1;
		} else {
#ifdef PLATFORM_X86
			ven_rsi_dbg(ERR_ZONE, "Failed sending bgscan probe req\n");
#endif
			common->bgscan_en = 0;
			g_bgscan_enable = 0;
		}
	
} else {
#ifdef PLATFORM_X86
		ven_rsi_dbg(ERR_ZONE, "Failed sending bgscan params req\n");
#endif
	}
	mutex_unlock(&common->mutex);

	return total_bytes;
}

#define FOPS(fopen) { \
	.owner = THIS_MODULE, \
	.open = (fopen), \
	.read = seq_read, \
	.llseek = seq_lseek, \
}

#define FOPS_RW(fopen, fwrite) { \
	.owner = THIS_MODULE, \
	.open = (fopen), \
	.read = seq_read, \
	.llseek = seq_lseek, \
	.write = (fwrite), \
}

static const struct rsi_dbg_files dev_debugfs_files[] = {
	{"version", 0644, FOPS(rsi_version_open),},
	{"stats", 0644, FOPS(rsi_stats_open),},
	{"debug_zone", 0666, FOPS_RW(rsi_debug_read, rsi_debug_zone_write),},
	{"bgscan", 0666, FOPS_RW(rsi_bgscan_read, rsi_bgscan_write),},
	{"sdio_stats", 0644, FOPS(rsi_sdio_stats_open),},
};

/**
 * rsi_init_dbgfs() - This function initializes the dbgfs entry.
 * @adapter: Pointer to the adapter structure.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_init_dbgfs(struct rsi_hw *adapter)
{
	struct rsi_common *common = adapter->priv;
	struct rsi_debugfs *dev_dbgfs;
	char devdir[6];
	int ii;
	const struct rsi_dbg_files *files;

	dev_dbgfs = kzalloc(sizeof(*dev_dbgfs), GFP_KERNEL);
	if (!dev_dbgfs)
		return -ENOMEM;

	adapter->dfsentry = dev_dbgfs;

	snprintf(devdir, sizeof(devdir), "%s",
		 wiphy_name(adapter->hw->wiphy));

	dev_dbgfs->subdir = debugfs_create_dir(devdir, NULL);

	if (!dev_dbgfs->subdir) {
		kfree(dev_dbgfs);
		return -ENOMEM;
	}

	for (ii = 0; ii < adapter->num_debugfs_entries; ii++) {
		files = &dev_debugfs_files[ii];
		dev_dbgfs->rsi_files[ii] =
		debugfs_create_file(files->name,
				    files->perms,
				    dev_dbgfs->subdir,
				    common,
				    &files->fops);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(rsi_init_dbgfs);

/**
 * rsi_remove_dbgfs() - Removes the previously created dbgfs file entries
 *			in the reverse order of creation.
 * @adapter: Pointer to the adapter structure.
 *
 * Return: None.
 */
void rsi_remove_dbgfs(struct rsi_hw *adapter)
{
	struct rsi_debugfs *dev_dbgfs = adapter->dfsentry;

	if (!dev_dbgfs)
		return;

	debugfs_remove_recursive(dev_dbgfs->subdir);
}
EXPORT_SYMBOL_GPL(rsi_remove_dbgfs);
