#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/rpm-smd-regulator.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/smsm.h>
#include "wcn36xx.h"
#include "wcnss_core.h"

#define VREG_NULL_CONFIG            0x0000
#define VREG_GET_REGULATOR_MASK     0x0001
#define VREG_SET_VOLTAGE_MASK       0x0002
#define VREG_OPTIMUM_MODE_MASK      0x0004
#define VREG_ENABLE_MASK            0x0008

#define WCNSS_INVALID_IRIS_REG      0xbaadbaad

struct vregs_info {
	const char * const name;
	int state;
	const int nominal_min;
	const int low_power_min;
	const int max_voltage;
	const int uA_load;
	struct regulator *regulator;
};

/* IRIS regulators for Pronto v2 hardware */
static struct vregs_info iris_vregs_pronto_v2[] = {
	{"qcom,iris-vddxo",  VREG_NULL_CONFIG, 1800000, 0,
		1800000, 10000,  NULL},
	{"qcom,iris-vddrfa", VREG_NULL_CONFIG, 1300000, 0,
		1300000, 100000, NULL},
	{"qcom,iris-vddpa",  VREG_NULL_CONFIG, 3300000, 0,
		3300000, 515000, NULL},
	{"qcom,iris-vdddig", VREG_NULL_CONFIG, 1800000, 0,
		1800000, 10000,  NULL},
};

/* WCNSS regulators for Pronto v2 hardware */
static struct vregs_info pronto_vregs_pronto_v2[] = {
	{"qcom,pronto-vddmx",  VREG_NULL_CONFIG, 1287500,  0,
		1287500, 0,    NULL},
	{"qcom,pronto-vddcx",  VREG_NULL_CONFIG, RPM_REGULATOR_CORNER_NORMAL,
		RPM_REGULATOR_CORNER_NONE, RPM_REGULATOR_CORNER_SUPER_TURBO,
		0,             NULL},
	{"qcom,pronto-vddpx",  VREG_NULL_CONFIG, 1800000, 0,
		1800000, 0,    NULL},
};

/* Common helper routine to turn on all WCNSS & IRIS vregs */
static int wcnss_vregs_on(struct device *dev,
		struct vregs_info regulators[], uint size)
{
	int i, rc = 0, reg_cnt;

	for (i = 0; i < size; i++) {
			/* Get regulator source */
		regulators[i].regulator =
			regulator_get(dev, regulators[i].name);
		if (IS_ERR(regulators[i].regulator)) {
			rc = PTR_ERR(regulators[i].regulator);
				pr_err("regulator get of %s failed (%d)\n",
					regulators[i].name, rc);
				goto fail;
		}
		regulators[i].state |= VREG_GET_REGULATOR_MASK;
		reg_cnt = regulator_count_voltages(regulators[i].regulator);
		/* Set voltage to nominal. Exclude swtiches e.g. LVS */
		if ((regulators[i].nominal_min || regulators[i].max_voltage)
				&& (reg_cnt > 0)) {
			rc = regulator_set_voltage(regulators[i].regulator,
					regulators[i].nominal_min,
					regulators[i].max_voltage);
			if (rc) {
				pr_err("regulator_set_voltage(%s) failed (%d)\n",
						regulators[i].name, rc);
				goto fail;
			}
			regulators[i].state |= VREG_SET_VOLTAGE_MASK;
		}

		/* Vote for PWM/PFM mode if needed */
		if (regulators[i].uA_load && (reg_cnt > 0)) {
			rc = regulator_set_optimum_mode(regulators[i].regulator,
					regulators[i].uA_load);
			if (rc < 0) {
				pr_err("regulator_set_optimum_mode(%s) failed (%d)\n",
						regulators[i].name, rc);
				goto fail;
			}
			regulators[i].state |= VREG_OPTIMUM_MODE_MASK;
		}

		/* Enable the regulator */
		rc = regulator_enable(regulators[i].regulator);
		if (rc) {
			pr_err("vreg %s enable failed (%d)\n",
				regulators[i].name, rc);
			goto fail;
		}
		regulators[i].state |= VREG_ENABLE_MASK;
	}

	return rc;

fail:
	return rc;

}

static int wcnss_core_config(struct platform_device *pdev, void __iomem *base)
{
	int ret = 0;
	u32 value, iris_read_v = INVALID_IRIS_REG;
	struct clk *clk_rf = NULL;
	struct clk *clk_xo = NULL;
	int clk_48m = 0;

	clk_xo = clk_get(&pdev->dev, "xo");
	if (IS_ERR(clk_xo)) {
		pr_err("Couldn't get xo clock\n");
		return PTR_ERR(clk_xo);
	}
	ret = clk_prepare_enable(clk_xo);
	if (ret) {
		pr_err("xo clk enable failed\n");
		return ret;
	}

	ret = wcnss_vregs_on(&pdev->dev, pronto_vregs_pronto_v2,
			ARRAY_SIZE(pronto_vregs_pronto_v2));
	if (ret) {
		pr_info("pronto_vreg turn on failed\n");
		return ret;
	}

	ret = wcnss_vregs_on(&pdev->dev, iris_vregs_pronto_v2,
			ARRAY_SIZE(iris_vregs_pronto_v2));
	if (ret) {
		pr_info("iris_vreg turn on failed\n");
		return ret;
	}

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

	if (!clk_48m) {
		clk_rf = clk_get(&pdev->dev, "rf_clk");
		if (IS_ERR(clk_rf)) {
			pr_err("couldn't get rf_clk\n");
			return -1;
		}

		ret = clk_prepare_enable(clk_rf);
		if (ret) {
			pr_err("clk_rf enable failed\n");
		}
		clk_put(clk_rf);
	}

        msleep(20);

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

static struct wcn36xx_ctrl_nv_data ctrl_nv_data;
static void wcn36xx_download_notify(void *data, unsigned int event)
{
	struct wcn36xx_ctrl_nv_data *ctrl_nv_data = data;

	switch (event) {
	case SMD_EVENT_OPEN:
		schedule_work(&ctrl_nv_data->download_work);
		break;
	case SMD_EVENT_DATA:
		schedule_work(&ctrl_nv_data->rx_work);
		break;
	case SMD_EVENT_CLOSE:
	case SMD_EVENT_STATUS:
	case SMD_EVENT_REOPEN_READY:
		break;
	default:
		pr_err("%s: SMD_EVENT (%d) not supported\n",
			__func__, event);
		break;
	}
}

static unsigned char wcnss_fw_status(struct wcn36xx_ctrl_nv_data *data)
{
	int len = 0;
	int rc = 0;

	unsigned char fw_status = 0xFF;

	len = smd_read_avail(data->smd_ch);
	if (len < 1) {
		pr_err("%s: invalid firmware status", __func__);
		return fw_status;
	}

	rc = smd_read(data->smd_ch, &fw_status, 1);
	if (rc < 0) {
		pr_err("%s: incomplete data read from smd\n", __func__);
		return fw_status;
	}
	return fw_status;
}

static void wcn36xx_nv_rx_work(struct work_struct *worker)
{
	struct wcn36xx_ctrl_nv_data *data =
		container_of(worker, struct wcn36xx_ctrl_nv_data, rx_work);
	int len = 0, ret = 0;
	unsigned char buf[sizeof(struct wcnss_version)];
	struct smd_msg_hdr *phdr;

	len = smd_read_avail(data->smd_ch);
	if (len > 4096) {
		pr_err("%s: frame larger than allowed size\n", __func__);
		smd_read(data->smd_ch, NULL, len);
		return;
	}

	if (len < sizeof(struct smd_msg_hdr))
		return;

	ret = smd_read(data->smd_ch, buf, sizeof(struct smd_msg_hdr));
	if (ret < sizeof(struct smd_msg_hdr)) {
		pr_err("%s: incomplete header from smd\n", __func__);
		return;
	}

	phdr = (struct smd_msg_hdr *)buf;

	switch (phdr->msg_type) {
	/* CBC COMPLETE means firmware ready for go */
        case WCNSS_CBC_COMPLETE_IND:
		complete(&data->wcnss_fw_ready_compl);
                pr_info("wcnss: received WCNSS_CBC_COMPLETE_IND from FW\n");
                break;

	case WCNSS_NV_DOWNLOAD_RSP:
		pr_info("fw_status: %d\n", wcnss_fw_status(data));
		break;
	}
	return;
}

static int wcn36xx_nv_smd_tx(struct wcn36xx_ctrl_nv_data *nv_data, void *buf, int len)
{
	int ret = 0;

	ret = smd_write_avail(nv_data->smd_ch);
	if (ret < len) {
		pr_err("wcnss: no space available. %d needed. Just %d avail\n",
			len, ret);
		return -ENOSPC;
	}
	ret = smd_write(nv_data->smd_ch, buf, len);
	if (ret < len) {
		pr_err("wcnss: failed to write Command %d", len);
		ret = -ENODEV;
	}
	return ret;
}

#define	NV_FILE_NAME	"wlan/prima/WCNSS_qcom_wlan_nv.bin"
static void wcn36xx_nv_download_work(struct work_struct *worker)
{
	int ret = 0, i, retry = 3;
	const struct firmware *nv = NULL;
	struct wcn36xx_ctrl_nv_data *data =
		container_of(worker, struct wcn36xx_ctrl_nv_data, download_work);
	struct device *dev = &data->pdev->dev;
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

		ret = wcn36xx_nv_smd_tx(data, pkt, msg->hdr.msg_len);

		while ((ret == -ENOSPC) && (retry++ <= 3)) {
			dev_err(dev, "smd_tx failed, %d times retry\n", retry);
			msleep(100);
			ret = wcn36xx_nv_smd_tx(data, pkt, msg->hdr.msg_len);
		}

		if (ret < 0) {
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

static int wcnss_ctrl_remove(struct platform_device *pdev)
{
	smd_close(ctrl_nv_data.smd_ch);

	return 0;
}

static int wcnss_ctrl_probe(struct platform_device *pdev)
{
	int ret = 0;

	INIT_WORK(&ctrl_nv_data.rx_work, wcn36xx_nv_rx_work);
	INIT_WORK(&ctrl_nv_data.download_work, wcn36xx_nv_download_work);
	ctrl_nv_data.pdev = pdev;

	ret = smd_named_open_on_edge("WCNSS_CTRL", SMD_APPS_WCNSS,
		&ctrl_nv_data.smd_ch, &ctrl_nv_data, wcn36xx_download_notify);
	if (ret) {
		dev_err(&pdev->dev, "wcnss_ctrl open failed\n");
		return ret;
	}
	smd_disable_read_intr(ctrl_nv_data.smd_ch);
	return ret;
}

/* platform device for WCNSS_CTRL SMD channel */
static struct platform_driver wcnss_ctrl_driver = {
	.driver = {
		.name	= "WCNSS_CTRL",
		.owner	= THIS_MODULE,
	},
	.probe	= wcnss_ctrl_probe,
	.remove	= wcnss_ctrl_remove,
};

static int wlan_ctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static int wlan_ctrl_probe(struct platform_device *pdev)
{
	complete(&ctrl_nv_data.wlan_ctrl_compl);
	return 0;
}

/* platform device for WLAN_CTRL SMD channel */
static struct platform_driver wlan_ctrl_driver = {
	.driver = {
		.name	= "WLAN_CTRL",
		.owner	= THIS_MODULE,
	},
	.probe	= wlan_ctrl_probe,
	.remove	= wlan_ctrl_remove,
};

int wcnss_core_init(void)
{
	int ret = 0;

	/* We wait two things here:
	 * - WCNSS_CBC_COMPLETE_IND from fw. Which means fw are fully ready.
	 * - "WLAN_CTRL" channel used by wcn36xx platform driver todownload
	 *   NV file.
	 */
	init_completion(&ctrl_nv_data.wcnss_fw_ready_compl);
	platform_driver_register(&wcnss_ctrl_driver);

	ret = wait_for_completion_interruptible_timeout(
		&ctrl_nv_data.wcnss_fw_ready_compl,
		msecs_to_jiffies(FW_READY_TIMEOUT));
	if (ret <= 0) {
		pr_err("timeout waiting for wcnss firmware ready indicator\n");
		return -EAGAIN;
	}

	init_completion(&ctrl_nv_data.wlan_ctrl_compl);
	platform_driver_register(&wlan_ctrl_driver);

	ret = wait_for_completion_interruptible_timeout(
		&ctrl_nv_data.wlan_ctrl_compl,
		msecs_to_jiffies(FW_READY_TIMEOUT));
	if (ret <= 0) {
		platform_driver_unregister(&wcnss_ctrl_driver);
		pr_err("timeout waiting for wcnss firmware ready indicator\n");
		return -EAGAIN;
	}
	return 0;
}

void wcnss_core_deinit(void)
{
	platform_driver_unregister(&wcnss_ctrl_driver);
}

