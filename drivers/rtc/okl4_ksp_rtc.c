/*
 * drivers/rtc/okl4_ksp_rtc.c
 *
 * Copyright: Open Kernel Labs, Inc. 2013
 * Copyright (c) 2014-2018 General Dynamics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * RTC driver for OKL4 Microvisor Kernel-Support-Package-based extension.
 * The device provides access to the date (& time) stored in the actual
 * RTC device, via a KSP Agent interface.
 *
 * If the cell only has limited access to the KSP RTC (e.g. Single get only),
 * access attempts that are (or would be) rejected are emulated, using the
 * time elapsed since the last successful Get or the last attempted Set.
 * This mechanism insulates Linux from KSP RTC permissions.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/time.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <microvisor/microvisor.h>

/*
 * Values for the operation argument of the _okl4_sys_ksp_procedure_call()
 * SYSCALL.
 *
 * The RTC data is transported respectively in the first 2 arguments of the KSP
 * procedure call or the 2 returned words. In both cases, the data is the epoch
 * value of time split into two 32-bit values.
 */
#define KSP_OPERATION_WORD_SET_RTC 0x5EC10CUL /* hex spelling for: SEt CLOCk */
#define KSP_OPERATION_WORD_GET_RTC 0x6EC10CUL /* hex spelling for: GEt CLOCk */


/*
 * Driver-specific device information (referred as "*this" in the code and
 * retrieved via dev_get_drvdata())
 */
struct okl4_ksp_rtc {
	okl4_kcap_t kcap;	/* KSP Agent reference */
	okl4_kcap_t signal_virq; /* KSP signal interrupt */
	struct work_struct rtc_sync_work;

	struct rtc_device *class; /* RTC class */
	struct platform_device *pdev;
};

/*
 * Default tolerance between system clock and RTC when automatically
 * synchronising the two clocks.  The RTC time resolution is 1 second,
 * and when set the time is rounded, so it will usually be within
 * +/- 0.5 sec of the actual time.  To avoid too much system clock
 * jitter, we don't try to set the system clock if the difference
 * between the KSP RTC and the system clock is less than this.
 */
#define DEFAULT_RTC_SYNC_TOLERANCE_NS (1.5 * NSEC_PER_SEC)

/* set to true to enable rtc clock sync */
static bool rtc_sync_enable = false;
module_param(rtc_sync_enable, bool, 0644);

/* set rtc sync tolerance in msec */
static long rtc_sync_tolerance_ms = DEFAULT_RTC_SYNC_TOLERANCE_NS / NSEC_PER_MSEC;
static long rtc_sync_tolerance_ns = DEFAULT_RTC_SYNC_TOLERANCE_NS;
module_param(rtc_sync_tolerance_ms, long, 0644);

/* Internal Get operation on the KSP RTC */
static int okl4_ksp_rtc_gettime(struct okl4_ksp_rtc *this,
		struct rtc_time *rtc_tm)
{
	int ret;
	struct _okl4_sys_ksp_procedure_call_return ksp_ret;
	unsigned long totalsecs;

	ksp_ret = _okl4_sys_ksp_procedure_call(this->kcap,
			KSP_OPERATION_WORD_GET_RTC, 0, 0, 0, 0);
	if (ksp_ret.error != OKL4_OK) {
		switch (ksp_ret.error) {
		case OKL4_ERROR_KSP_INVALID_ARG:
			/* Misconfigured KSP Agent */
			ret = -ENXIO;
			break;
		case OKL4_ERROR_KSP_INSUFFICIENT_RIGHTS:
			/* Single Get mode */
			ret = -EPERM;
			break;
		default:
			/* Typically: KSP RTC not implemented in microvisor SOC */
			ret = -ENODEV;
			break;
		}
		dev_dbg(&this->pdev->dev, "%s failed (0x%x), errno=%d\n", __func__,
				(unsigned int)ksp_ret.error, -ret);
		return ret;
	}

	totalsecs = (ksp_ret.ret1 << 32) | ksp_ret.ret0;
	rtc_time_to_tm(totalsecs, rtc_tm);

	ret = rtc_valid_tm(rtc_tm);

	return ret;
}

/* Internal Set operation on the KSP RTC */
static int okl4_ksp_rtc_settime(struct okl4_ksp_rtc *this, struct rtc_time *rtc_tm)
{
	int ret = 0;
	unsigned long totalsecs;
	struct _okl4_sys_ksp_procedure_call_return ksp_ret;

	rtc_tm_to_time(rtc_tm, &totalsecs);

	ksp_ret = _okl4_sys_ksp_procedure_call(this->kcap,
			KSP_OPERATION_WORD_SET_RTC,
#if __SIZEOF_LONG__ > 4
			totalsecs & 0xFFFFFFFF, totalsecs >> 32,
#else
			totalsecs, 0,
#endif
			0, 0);

	if (ksp_ret.error != OKL4_OK) {
		switch (ksp_ret.error) {
		case OKL4_ERROR_KSP_INVALID_ARG:
			/* Should not happen */
			ret = -ENXIO;
			break;
		case OKL4_ERROR_KSP_INSUFFICIENT_RIGHTS:
			/* Set access disallowed */
			ret = -EACCES;
			break;
		default:
			/* Typically: KSP RTC not implemented in microvisor SOC */
			ret = -ENODEV;
			break;
		}
		dev_dbg(&this->pdev->dev, "%s failed (0x%x), errno=%d\n", __func__,
				(unsigned int)ksp_ret.error, -ret);
	} else {
		dev_dbg(&this->pdev->dev, "%s: %04d.%02d.%02d-%02d:%02d:%02d\n", __func__,
				1900 + rtc_tm->tm_year, rtc_tm->tm_mon + 1, rtc_tm->tm_mday,
				rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec);
	}

	return ret;
}

static int rtc_gettime(struct device *dev,struct rtc_time *rtc_tm)
{
	int ret = 0;
	struct okl4_ksp_rtc *this = dev_get_drvdata(dev);

	if (!this)
		return -ENODEV;

	ret = okl4_ksp_rtc_gettime(this, rtc_tm);
	return ret;
}

static int rtc_settime(struct device *dev, struct rtc_time *rtc_tm)
{
	int ret = 0;

	struct okl4_ksp_rtc *this = dev_get_drvdata(dev);

	if (!this)
		return -ENODEV;

	ret = rtc_valid_tm(rtc_tm);
	if (ret != 0)
		return ret;

	ret = okl4_ksp_rtc_settime(this, rtc_tm);

	if (ret == -EACCES || ret == -ENODEV)
		ret = 0; /* Pretend everything is OK */

	return ret;
}

static const struct rtc_class_ops okl4_ksp_rtcops = {
	.read_time		= rtc_gettime,
	.set_time		= rtc_settime,
};

static void okl4_ksp_rtc_update_sys_clock_work(struct work_struct *work)
{
	struct okl4_ksp_rtc *this = container_of(work, struct okl4_ksp_rtc,
			rtc_sync_work);
	struct rtc_time rtc_tm;
	struct timespec rtc_ts, sys_ts;
	unsigned long rtc_time;
	s64 sys_ns, rtc_ns;
	int ret;

	/* read the clock */
	ret = okl4_ksp_rtc_gettime(this, &rtc_tm);
	if (ret)
		return;

	/* get current system time */
	getnstimeofday(&sys_ts);

	/* convert rtc time to a timespec */
	rtc_tm_to_time(&rtc_tm, &rtc_time);
	rtc_ts = ns_to_timespec((s64)rtc_time * NSEC_PER_SEC);

	/* set the system clock if it's out of tolerance */
	sys_ns = timespec_to_ns(&sys_ts);
	rtc_ns = timespec_to_ns(&rtc_ts);
	if (sys_ns - rtc_ns > rtc_sync_tolerance_ns ||
			rtc_ns - sys_ns > rtc_sync_tolerance_ns)
		do_settimeofday(&rtc_ts);
}

static irqreturn_t rtc_signal_virq_handler(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct okl4_ksp_rtc *this = dev_get_drvdata(dev);

	/* another cell has just set the hardware clock */
	if (rtc_sync_enable) {
		/* we can't call do_setimeofday() with interrupts disabled */
		schedule_work(&this->rtc_sync_work);
	}

	return IRQ_HANDLED;
}

static int okl4_ksp_rtc_remove(struct platform_device *pdev)
{
	struct okl4_ksp_rtc *this = dev_get_drvdata(&pdev->dev);
	if (!this) {
		dev_err(&pdev->dev, "Cannot retrieve drvdata\n");
		return -ENODEV;
	}

	if (this->signal_virq != OKL4_KCAP_INVALID)
		devm_free_irq(&pdev->dev, this->signal_virq, &pdev->dev);
	cancel_work_sync(&this->rtc_sync_work);
	rtc_device_unregister(this->class);
	dev_set_drvdata(&pdev->dev, NULL);
	devm_kfree(&pdev->dev, this);

	return 0;
}

static int okl4_ksp_rtc_probe(struct platform_device *pdev)
{
	struct okl4_ksp_rtc *this;
	u32 kcap;
	struct rtc_device *class;
	struct device *dev = &pdev->dev;
	int ret;

	of_property_read_u32(dev->of_node, "reg", &kcap);

	this = devm_kzalloc(dev, sizeof(struct okl4_ksp_rtc), GFP_KERNEL);
	if (!this)
		return -ENOMEM;

	if (rtc_sync_enable) {
		ret = platform_get_irq(pdev, 0);
		if (ret < 0) {
			dev_err(dev, "Cannot get signal irq: %d\n", ret);
			goto err_free_this;
		}
		this->signal_virq = ret;
		rtc_sync_tolerance_ns = rtc_sync_tolerance_ms * NSEC_PER_MSEC;
		dev_info(dev, "rtc sync enabled, tolerance = %ld ms\n", rtc_sync_tolerance_ms);
		if (rtc_sync_tolerance_ms <= MSEC_PER_SEC / 2)
			dev_warn(dev, "rtc sync tolerance should be greater than 500ms to"
					" reduce clock jitter\n");
		INIT_WORK(&this->rtc_sync_work, okl4_ksp_rtc_update_sys_clock_work);
	} else {
		this->signal_virq = OKL4_KCAP_INVALID;
	}

	class = rtc_device_register("okl4-ksp", dev, &okl4_ksp_rtcops, THIS_MODULE);
	if (IS_ERR(class)) {
		dev_err(dev, "Cannot register with RTC class\n");
		ret = PTR_ERR(class);
		goto err_free_this;
	}

#ifdef RTC_DEV_SLAVE
	/*
	 * Mark this as a slave clock so that it is automatically updated when the
	 * hardware clock is set.
	 */
	set_bit(RTC_DEV_SLAVE, &class->flags);
#endif
	this->pdev = pdev;
	this->kcap = kcap;
	this->class = class;

	dev_set_drvdata(dev, this);

	if (this->signal_virq != OKL4_KCAP_INVALID) {
		ret = devm_request_irq(dev, this->signal_virq, rtc_signal_virq_handler,
				IRQF_TRIGGER_RISING, dev_name(dev), dev);
		if (ret < 0) {
			dev_err(dev, "devm_request_irq failed: %d\n", ret);
			goto err_unregister;
		}
	}

	return 0;

err_unregister:
	dev_set_drvdata(dev, NULL);
	rtc_device_unregister(this->class);
err_free_this:
	devm_kfree(dev, this);
	return ret;
}

static struct of_device_id okl4_ksp_rtc_of_match[] = {
	{ .compatible = "okl,ksp-agent-rtc", },
	{ },
};

static struct platform_driver okl4_ksp_rtc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "okl4-ksp-rtc",
		.of_match_table = of_match_ptr(okl4_ksp_rtc_of_match),
	},
	.probe = okl4_ksp_rtc_probe,
	.remove = okl4_ksp_rtc_remove,
};

static int __init okl4_ksp_rtc_init(void)
{
	int ret;

	ret = platform_driver_register(&okl4_ksp_rtc_driver);
	if (ret < 0)
		pr_err("Failed to register OKL4 KSP RTC: %d\n", ret);

	return ret;
}

static void __exit okl4_ksp_rtc_exit(void)
{
	platform_driver_unregister(&okl4_ksp_rtc_driver);
}

module_init(okl4_ksp_rtc_init);
module_exit(okl4_ksp_rtc_exit);

MODULE_DESCRIPTION("OKL4 KSP-Agent RTC Driver");
MODULE_AUTHOR("Cog Systems Pty Ltd");
