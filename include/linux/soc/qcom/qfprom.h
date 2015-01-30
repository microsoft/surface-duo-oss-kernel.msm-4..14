#ifndef __QCOM_QFPROM_H__
#define __QCOM_QFPROM_H__

#ifdef CONFIG_QCOM_QFPROM

char *qfprom_get_data_byname(struct device *dev, const char *name, int *len);
char *devm_qfprom_get_data_byname(struct device *dev,
					  const char *name, int *len);
char *qfprom_get_data(struct device *dev, int index, int *len);
char *devm_qfprom_get_data(struct device *dev, int index, int *len);

#else

static inline char *qfprom_get_data_byname(struct device *dev,
						   const char *name)
{
	return NULL:
}

static inline char *qfprom_get_data(struct device *dev, int index)
{
	return NULL:
}

static inline char *devm_qfprom_get_data_byname(struct device *dev,
							const char *name);
{
	return NULL:
}

static inline char *devm_qfprom_get_data(struct device *dev, int index)
{
	return NULL:
}

#endif /* CONFIG_QCOM_QFPROM */

#endif /* __QCOM_QFPROM_H__ */
