#ifndef	_WCNSS_CORE_H_
#define	_WCNSS_CORE_H_

#define	PMU_OFFSET	0x1004
#define	SPARE_OFFSET	0x1088

#define WCNSS_PMU_CFG_IRIS_XO_CFG          BIT(3)
#define WCNSS_PMU_CFG_IRIS_XO_EN           BIT(4)
#define WCNSS_PMU_CFG_GC_BUS_MUX_SEL_TOP   BIT(5)
#define WCNSS_PMU_CFG_IRIS_XO_CFG_STS      BIT(6) /* 1: in progress, 0: done */

#define WCNSS_PMU_CFG_IRIS_RESET           BIT(7)
#define WCNSS_PMU_CFG_IRIS_RESET_STS       BIT(8) /* 1: in progress, 0: done */
#define WCNSS_PMU_CFG_IRIS_XO_READ         BIT(9)
#define WCNSS_PMU_CFG_IRIS_XO_READ_STS     BIT(10)
#define	WCNSS_FW_DOWNLOAD_ENABLE	   BIT(25)

#define WCNSS_PMU_CFG_IRIS_XO_MODE         0x6
#define WCNSS_PMU_CFG_IRIS_XO_MODE_48      (3 << 1)

#define	NV_DOWNLOAD_TIMEOUT	500
#define	NV_FRAGMENT_SIZE	3072
#define MAX_CALIBRATED_DATA_SIZE  (64*1024)
#define LAST_FRAGMENT        (1 << 0)
#define MESSAGE_TO_FOLLOW    (1 << 1)
#define CAN_RECEIVE_CALDATA  (1 << 15)
#define WCNSS_RESP_SUCCESS   1
#define WCNSS_RESP_FAIL      0


#define	WCNSS_NV_DOWNLOAD_REQ	0x01000002
#define	WCNSS_NV_DOWNLOAD_RSP	0x01000003

struct wcn36xx_ctrl_nv_data {
	struct workqueue_struct *wq;
	struct work_struct	rx_work;
	struct work_struct	download_work;
	struct completion	smd_open_compl;
	smd_channel_t		*smd_ch;
	struct platform_device	*pdev;
};

struct smd_msg_hdr {
	unsigned int msg_type;
	unsigned int msg_len;
};

struct nvbin_dnld_req_params {
	/* Fragment sequence number of the NV bin Image. NV Bin Image
	 * might not fit into one message due to size limitation of
	 * the SMD channel FIFO so entire NV blob is chopped into
	 * multiple fragments starting with seqeunce number 0. The
	 * last fragment is indicated by marking is_last_fragment field
	 * to 1. At receiving side, NV blobs would be concatenated
	 * together without any padding bytes in between.
	 */
	unsigned short frag_number;

	/* bit 0: When set to 1 it indicates that no more fragments will
	 * be sent.
	 * bit 1: When set, a new message will be followed by this message
	 * bit 2- bit 14:  Reserved
	 * bit 15: when set, it indicates that the sender is capable of
	 * receiving Calibrated data.
	 */
	unsigned short msg_flags;

	/* NV Image size (number of bytes) */
	unsigned int nvbin_buffer_size;

	/* Following the 'nvbin_buffer_size', there should be
	 * nvbin_buffer_size bytes of NV bin Image i.e.
	 * uint8[nvbin_buffer_size].
	 */
};

struct nvbin_dnld_req_msg {
	/* Note: The length specified in nvbin_dnld_req_msg messages
	 * should be hdr.msg_len = sizeof(nvbin_dnld_req_msg) +
	 * nvbin_buffer_size.
	 */
	struct smd_msg_hdr hdr;
	struct nvbin_dnld_req_params dnld_req_params;
};

struct wcnss_version {
	struct smd_msg_hdr hdr;
	unsigned char  major;
	unsigned char  minor;
	unsigned char  version;
	unsigned char  revision;
};


int wcnss_core_prepare(struct platform_device *pdev);
void wcnss_core_init(void);
void wcnss_core_deinit(void);

#endif

