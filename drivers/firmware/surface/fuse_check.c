/*
 * fuse_check.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/io.h>
#include <linux/slab.h>
#include <soc/qcom/scm.h>
#include <linux/sysfs.h>
#include "fuse_check.h"
#include <linux/device.h>

#define TZ_INFO_GET_SECURE_STATE	0x4
#define SIMLOCK_ADDRESS 0x078621c

static int m_fuse_value = 0;
static int m_sysfs_published = 1;
static int m_simlock_value = 0;

static ssize_t handle_fuse_reads(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf);
static ssize_t print_fuse_data(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf);

/*
Masks to use to get different secure status from the SCM data
*/
typedef enum
{
    SECCTRL_SECBOOT_ENABLED_SHIFT = 0x0,
    SECCTRL_SEC_HW_KEY_PROGRAMMED_SHIFT = 0x1,
    SECCTRL_DEBUG_DISABLE_CHECK_SHIFT = 0x2,
    SECCTRL_ANTI_ROLLBACK_CHECK_SHIFT = 0x3,
    SECCTRL_FUSE_CONFIG_CHECK_SHIFT = 0x4,
    SECCTRL_RPMB_PROVISIONED_CHECK_SHIFT = 0x5,
    SECCTRL_DEBUG_RE_ENABLED_CHECK_SHIFT = 0x6,
	SECCTRL_SIMLOCK_SHIFT = 0x1f
} tzbsp_secstatus_status_shift;

#define SECCTRL_SIMLOCK_MASK (0x1 << SECCTRL_SIMLOCK_SHIFT)

#define SECCTRL_SECBOOT_ENABLED_MASK        (0x1 << SECCTRL_SECBOOT_ENABLED_SHIFT)
#define SECCTRL_SEC_HW_KEY_PROGRAMMED_MASK  (0x1 << SECCTRL_SEC_HW_KEY_PROGRAMMED_SHIFT)
#define SECCTRL_DEBUG_DISABLE_CHECK_MASK    (0x1 << SECCTRL_DEBUG_DISABLE_CHECK_SHIFT)
#define SECCTRL_ANTI_ROLLBACK_CHECK_MASK    (0x1 << SECCTRL_ANTI_ROLLBACK_CHECK_SHIFT)
#define SECCTRL_FUSE_CONFIG_CHECK_MASK      (0x1 << SECCTRL_FUSE_CONFIG_CHECK_SHIFT)
#define SECCTRL_RPMB_PROVISIONED_CHECK_MASK (0x1 << SECCTRL_RPMB_PROVISIONED_CHECK_SHIFT)
#define SECCTRL_DEBUG_RE_ENABLED_CHECK_MASK (0x1 << SECCTRL_DEBUG_RE_ENABLED_CHECK_SHIFT)

ATTR(secureboot, handle_fuse_reads)
ATTR(securehwkey, handle_fuse_reads)
ATTR(debugdisable, handle_fuse_reads)
ATTR(antirollback, handle_fuse_reads)
ATTR(rpmbprovision, handle_fuse_reads)
ATTR(fusevalue, handle_fuse_reads)
ATTR(displayfuse, print_fuse_data)
ATTR(simlockfuse, handle_fuse_reads)

//static struct kobj_attribute displayfuse_attribute = __ATTR(displayfuse, 0664, print_fuse_data, NULL);


static struct attribute *fuse_attrs[] = {
	ATTR_LIST(secureboot),
	ATTR_LIST(securehwkey),
	ATTR_LIST(debugdisable),
	ATTR_LIST(antirollback),
	ATTR_LIST(rpmbprovision),
	ATTR_LIST(fusevalue),
	ATTR_LIST(displayfuse),
	ATTR_LIST(simlockfuse),
	NULL,		/* terminator */
};

static struct attribute_group fuse_attrs_group = {
	.name	= "fuse_state",
	.attrs	= fuse_attrs,
};
static ssize_t print_fuse_data(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{

	return sprintf(buf,
					"  Secure Boot Enabled Fuses : %d\n"
					"  secure hw key fuses       : %d\n"
					"  debug disable fuses       : %d\n"
					"  antirollback fuses        : %d\n"
					"  rpmb provisioned          : %d\n"
					"  fusevalue                 : %d\n"
					"  simlock fuse value(raw)   : %d\n",
					(m_fuse_value & SECCTRL_SECBOOT_ENABLED_MASK) == 0,
					(m_fuse_value & SECCTRL_SEC_HW_KEY_PROGRAMMED_MASK)==0,
					(m_fuse_value & SECCTRL_DEBUG_DISABLE_CHECK_MASK)==0,
					(m_fuse_value & SECCTRL_ANTI_ROLLBACK_CHECK_MASK)==0,
					(m_fuse_value & SECCTRL_RPMB_PROVISIONED_CHECK_MASK)==0,
					m_fuse_value ,
					(m_simlock_value)
	);
}

static ssize_t handle_fuse_reads(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	if (ATTRCMP(secureboot))
		return sprintf(buf, "%d\n", (m_fuse_value & SECCTRL_SECBOOT_ENABLED_MASK)==0);

	if (ATTRCMP(securehwkey))
		return sprintf(buf, "%d\n", (m_fuse_value & SECCTRL_SEC_HW_KEY_PROGRAMMED_MASK)==0);

	if (ATTRCMP(debugdisable))
		return sprintf(buf, "%d\n", (m_fuse_value & SECCTRL_DEBUG_DISABLE_CHECK_MASK)==0);

	if (ATTRCMP(antirollback))
		return sprintf(buf, "%d\n", (m_fuse_value & SECCTRL_ANTI_ROLLBACK_CHECK_MASK)==0);

	if (ATTRCMP(rpmbprovision))
		return sprintf(buf, "%d\n", (m_fuse_value & SECCTRL_RPMB_PROVISIONED_CHECK_MASK)==0);

	if (ATTRCMP(fusevalue))
		return sprintf(buf, "%d\n", m_fuse_value );

	if (ATTRCMP(simlockfuse))
		return sprintf(buf, "%d\n", (m_simlock_value & SECCTRL_SIMLOCK_MASK ) != 0) ;

	return -ENOENT;
}


static int initialize_sysfs_nodes(struct kobject *kobj)
{
	return sysfs_create_group(kobj, &fuse_attrs_group);
}

static int read_fuse_value()
{
	struct scm_desc desc = {0};
	int scm_ret = 0;
	int ret = 0;

	desc.args[0] = 0;
	desc.arginfo = 0;
	scm_ret = scm_call2(SCM_SIP_FNID(SCM_SVC_INFO,
			TZ_INFO_GET_SECURE_STATE),
			&desc);
	m_fuse_value = desc.ret[0];

	if (scm_ret) {
		pr_err("%s: SCM call failed %d\n", __func__, scm_ret);
		ret = -EFAULT;
	}
	else {
		pr_info("%s: SCM call returned  %d\n", __func__, m_fuse_value);
	}
	return ret;
}

static int read_sim_lock_value(struct kobject* kobj)
{
    int ret = 0;
	struct device *dev = container_of(kobj, struct device, kobj);
	void __iomem *simfuse;

	simfuse = devm_ioremap_nocache(dev, SIMLOCK_ADDRESS,4);
	if(!simfuse){
		pr_err("%s: cannot map address \n", __func__);
		ret = -EFAULT;
	}else{
		m_simlock_value = ioread32(simfuse);
		pr_err("%s: Sim value read %d\n", __func__, m_simlock_value);
	}
	return ret;
}

int fuse_check_init(struct kobject* kobj)
{
	int ret;
    //read fuse data
	ret = read_fuse_value();

	if(ret != 0){
		pr_err("%s: failed to read fuse value %d\n", __func__, ret);
		goto end;
	}

	// read simlock fuse value
	ret = read_sim_lock_value(kobj);
	if(ret != 0){
		pr_err("%s: failed to sim fuse value %d\n", __func__, ret);
		goto end;
	}

	//create sysfs nodes
	ret = initialize_sysfs_nodes(kobj);
	if(ret != 0){
		m_sysfs_published = 0;
		pr_err("%s: failed to create sysfs nodes  %d\n", __func__, ret);
	}
end:
	return ret;

}
EXPORT_SYMBOL(fuse_check_init);

int fuse_check_deinit(struct kobject* kobj)
{
	if(m_sysfs_published)
		sysfs_remove_group(kobj, &fuse_attrs_group);
	return 0;
}

EXPORT_SYMBOL(fuse_check_deinit);