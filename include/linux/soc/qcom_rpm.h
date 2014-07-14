#ifndef __QCOM_RPM_H__
#define __QCOM_RPM_H__

#include <linux/types.h>

struct device;
struct qcom_rpm;

struct qcom_rpm *dev_get_qcom_rpm(struct device *dev);
int qcom_rpm_write(struct qcom_rpm *rpm, int resource, u32 *buf, size_t count);

#endif
