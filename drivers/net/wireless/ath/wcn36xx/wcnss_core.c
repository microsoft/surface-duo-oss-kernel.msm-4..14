#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/soc/qcom/smd.h>
#include "wcn36xx.h"
#include "wcnss_core.h"

#define	WCNSS_CTRL_TIMEOUT	(msecs_to_jiffies(500))

static int wcnss_core_config(struct platform_device *pdev, void __iomem *base)
{
	int ret = 0;
	u32 value, iris_read_v = INVALID_IRIS_REG;
	int clk_48m = 0;

	value = readl_relaxed(base + SPARE_OFFSET);
	value |= WCNSS_FW_DOWNLOAD_ENABLE;
	writel_relaxed(value, base + SPARE_OFFSET);

	writel_relaxed(0, base + PMU_OFFSET);
	value = readl_relaxed(base + PMU_OFFSET);
	value |= WCNSS_PMU_CFG_GC_BUS_MUX_SEL_TOP |
		WCNSS_PMU_CFG_IRIS_XO_EN;
	writel_relaxed(value, base + PMU_OFFSET);

	iris_read_v = readl_relaxed(base + IRIS_REG_OFFSET);
	pr_info("iris_read_v: 0x%x\n", iris_read_v);

	iris_read_v &= 0xffff;
	iris_read_v |= 0x04;
	writel_relaxed(iris_read_v, base + IRIS_REG_OFFSET);

	value = readl_relaxed(base + PMU_OFFSET);
	value |= WCNSS_PMU_CFG_IRIS_XO_READ;
	writel_relaxed(value, base + PMU_OFFSET);

	while (readl_relaxed(base + PMU_OFFSET) &
			WCNSS_PMU_CFG_IRIS_XO_READ_STS)
		cpu_relax();

	iris_read_v = readl_relaxed(base + 0x1134);
	pr_info("wcnss: IRIS Reg: 0x%08x\n", iris_read_v);
	clk_48m = (iris_read_v >> 30) ? 0 : 1;
	value &= ~WCNSS_PMU_CFG_IRIS_XO_READ;

	/* XO_MODE[b2:b1]. Clear implies 19.2MHz */
	value &= ~WCNSS_PMU_CFG_IRIS_XO_MODE;

	if (clk_48m)
		value |= WCNSS_PMU_CFG_IRIS_XO_MODE_48;

	writel_relaxed(value, base + PMU_OFFSET);

	/* Reset IRIS */
	value |= WCNSS_PMU_CFG_IRIS_RESET;
	writel_relaxed(value, base + PMU_OFFSET);

	while (readl_relaxed(base + PMU_OFFSET) &
			WCNSS_PMU_CFG_IRIS_RESET_STS)
		cpu_relax();

	/* reset IRIS reset bit */
	value &= ~WCNSS_PMU_CFG_IRIS_RESET;
	writel_relaxed(value, base + PMU_OFFSET);

	/* start IRIS XO configuration */
	value |= WCNSS_PMU_CFG_IRIS_XO_CFG;
	writel_relaxed(value, base + PMU_OFFSET);

	/* Wait for XO configuration to finish */
	while (readl_relaxed(base + PMU_OFFSET) &
			WCNSS_PMU_CFG_IRIS_XO_CFG_STS)
		cpu_relax();

	/* Stop IRIS XO configuration */
	value &= ~(WCNSS_PMU_CFG_GC_BUS_MUX_SEL_TOP |
			WCNSS_PMU_CFG_IRIS_XO_CFG);
	writel_relaxed(value, base + PMU_OFFSET);

        msleep(200);

	return ret;
}

int wcnss_core_prepare(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	void __iomem *wcnss_base;

	res = platform_get_resource_byname(pdev,
                        IORESOURCE_MEM, "pronto_phy_base");
	if (!res) {
		ret = -EIO;
		dev_err(&pdev->dev, "resource pronto_phy_base failed\n");
		return ret;
	}

	wcnss_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wcnss_base)) {
		dev_err(&pdev->dev, "pronto_phy_base map failed\n");
		return PTR_ERR(wcnss_base);
	}

	ret = wcnss_core_config(pdev, wcnss_base);
	return ret;
}

struct wcnss_platform_data {
        struct qcom_smd_channel	*channel;
        struct completion	ack;
        struct mutex		lock;

	struct work_struct	rx_work;
	struct work_struct	download_work;

	struct qcom_smd_device	*sdev;
};

static struct completion fw_ready_compl;
#define	NV_FILE_NAME	"wlan/prima/WCNSS_qcom_wlan_nv.bin"
static void wcn36xx_nv_download_work(struct work_struct *worker)
{
	int ret = 0, i;
	const struct firmware *nv = NULL;
	struct wcnss_platform_data *wcnss_data =
		container_of(worker, struct wcnss_platform_data, download_work);
	struct device *dev = &wcnss_data->sdev->dev;

	struct nvbin_dnld_req_msg *msg;
	const void *nv_blob_start;
	char *pkt = NULL;
	int nv_blob_size = 0, fragments;

	ret = request_firmware(&nv, NV_FILE_NAME, dev);
	if (ret || !nv || !nv->data || !nv->size) {
		dev_err(dev, "request firmware for %s (ret = %d)\n",
			NV_FILE_NAME, ret);
		return;
	}

	nv_blob_start = nv->data + 4;
	nv_blob_size = nv->size -4;

	fragments = (nv_blob_size + NV_FRAGMENT_SIZE - 1)/NV_FRAGMENT_SIZE;

	pkt = kzalloc(sizeof(struct nvbin_dnld_req_msg) + NV_FRAGMENT_SIZE,
			GFP_KERNEL);
	if (!pkt) {
		dev_err(dev, "allocation packet for nv download failed\n");
		release_firmware(nv);
	}

	msg = (struct nvbin_dnld_req_msg *)pkt;
	msg->hdr.msg_type = WCNSS_NV_DOWNLOAD_REQ;
	msg->dnld_req_params.msg_flags = 0;

	i = 0;
	do {
		int pkt_len = 0;

		msg->dnld_req_params.frag_number = i;
		if (nv_blob_size > NV_FRAGMENT_SIZE) {
			msg->dnld_req_params.msg_flags &=
				~LAST_FRAGMENT;
			pkt_len = NV_FRAGMENT_SIZE;
		} else {
			pkt_len = nv_blob_size;
			msg->dnld_req_params.msg_flags |=
				LAST_FRAGMENT | CAN_RECEIVE_CALDATA;
		}

		msg->dnld_req_params.nvbin_buffer_size = pkt_len;
		msg->hdr.msg_len =
			sizeof(struct nvbin_dnld_req_msg) + pkt_len;

		memcpy(pkt + sizeof(struct nvbin_dnld_req_msg),
			nv_blob_start + i * NV_FRAGMENT_SIZE, pkt_len);

		ret = qcom_smd_send(wcnss_data->channel, pkt, msg->hdr.msg_len);
		if (ret) {
			dev_err(dev, "nv download failed\n");
			goto out;
		}

		i++;
		nv_blob_size -= NV_FRAGMENT_SIZE;
		msleep(100);
	} while (nv_blob_size > 0);

out:
	kfree(pkt);
	release_firmware(nv);
	return;
}

static int qcom_smd_wcnss_ctrl_callback(struct qcom_smd_device *qsdev,
                                 const void *data,
                                 size_t count)
{
	struct wcnss_platform_data *wcnss_data = dev_get_drvdata(&qsdev->dev);
	struct smd_msg_hdr phdr;
	const unsigned char *tmp = data;

	memcpy_fromio(&phdr, data, sizeof(struct smd_msg_hdr));

	switch (phdr.msg_type) {
       /* CBC COMPLETE means firmware ready for go */
        case WCNSS_CBC_COMPLETE_IND:
                complete(&fw_ready_compl);
                pr_info("wcnss: received WCNSS_CBC_COMPLETE_IND from FW\n");
                break;

	case WCNSS_NV_DOWNLOAD_RSP:
		pr_info("fw_status: %d\n", tmp[sizeof(struct smd_msg_hdr)]);
		break;
	}

	complete(&wcnss_data->ack);
	return 0;
}

static int qcom_smd_wcnss_ctrl_probe(struct qcom_smd_device *sdev)
{
	struct wcnss_platform_data *wcnss_data;

        wcnss_data = devm_kzalloc(&sdev->dev, sizeof(*wcnss_data), GFP_KERNEL);
        if (!wcnss_data)
                return -ENOMEM;

        mutex_init(&wcnss_data->lock);
        init_completion(&wcnss_data->ack);

	wcnss_data->sdev = sdev;

        dev_set_drvdata(&sdev->dev, wcnss_data);
	wcnss_data->channel = sdev->channel;

	INIT_WORK(&wcnss_data->download_work, wcn36xx_nv_download_work);

	of_platform_populate(sdev->dev.of_node, NULL, NULL, &sdev->dev);

	/* We are ready for download here */
	schedule_work(&wcnss_data->download_work);
        return 0;
}

static void qcom_smd_wcnss_ctrl_remove(struct qcom_smd_device *sdev)
{
        of_platform_depopulate(&sdev->dev);
}

static const struct of_device_id qcom_smd_wcnss_ctrl_of_match[] = {
	{ .compatible = "qcom,wcnss-ctrl" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smd_wcnss_ctrl_of_match);

static struct qcom_smd_driver qcom_smd_wcnss_ctrl_driver = {
	.probe = qcom_smd_wcnss_ctrl_probe,
	.remove = qcom_smd_wcnss_ctrl_remove,
	.callback = qcom_smd_wcnss_ctrl_callback,
	.driver  = {
		.name  = "qcom_smd_wcnss_ctrl",
		.owner = THIS_MODULE,
		.of_match_table = qcom_smd_wcnss_ctrl_of_match,
	},
};

void wcnss_core_init(void)
{
	int ret = 0;

	init_completion(&fw_ready_compl);
	qcom_smd_driver_register(&qcom_smd_wcnss_ctrl_driver);

	ret = wait_for_completion_interruptible_timeout(
		&fw_ready_compl, msecs_to_jiffies(FW_READY_TIMEOUT));
	if (ret <= 0) {
                pr_err("timeout waiting for wcnss firmware ready indicator\n");
		return;
        }

	return;
}

void wcnss_core_deinit(void)
{
	qcom_smd_driver_unregister(&qcom_smd_wcnss_ctrl_driver);
}
