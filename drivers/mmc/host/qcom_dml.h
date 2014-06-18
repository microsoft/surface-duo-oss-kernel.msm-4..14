#ifndef __MMC_QCOM_DML_H__
#define __MMC_QCOM_DML_H__

#ifdef CONFIG_MMC_QCOM_DML
int dml_hw_init(struct mmci_host *host, struct device_node *np);
void dml_start_xfer(struct mmci_host *host, struct mmc_data *data);
#else
static inline int dml_hw_init(struct mmci_host *host, struct device_node *np)
{
	return -ENOSYS;
}
static inline void dml_start_xfer(struct mmci_host *host, struct mmc_data *data)
{
}
#endif /* CONFIG_MMC_QCOM_DML */

#endif /* __MMC_QCOM_DML_H__ */
