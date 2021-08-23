/*
 * xbl_hlos_hob.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/soc/surface/surface_utils.h>
#include <linux/string.h>

struct XBL_HLOS_HOB __iomem *XblHlosHob = NULL;

bool get_manuf_mode()
{
    int rc = 0;
    uint8_t IsManufMode = CUST_MODE;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
    {
        IsManufMode = CUST_MODE; // if the hob has not been correctly read for any reason we will return NOT IN MANUF MODE by default
        goto get_manuf_mode_exit;
    }
    IsManufMode = readb_relaxed(&XblHlosHob->IsCustomerMode);
    pr_err("%s: get_manuf_mode:%d \n", __func__, IsManufMode);
get_manuf_mode_exit:
    return (IsManufMode == MCFG_MODE);
}
EXPORT_SYMBOL(get_manuf_mode);

uint16_t get_ocp_error_info()
{
    int rc = 0;
    uint16_t ocp_error_location = 0;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
        goto get_ocp_error_info_exit;

    ocp_error_location = readw_relaxed(&XblHlosHob->OCPErrorLocation);
    pr_err("%s: OCP Error Info :%d \n", __func__, ocp_error_location);

get_ocp_error_info_exit:
    return ocp_error_location;
}
EXPORT_SYMBOL(get_ocp_error_info);

int get_act_mode()
{
    int rc = 0;
    int act_mode = 0;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
    {
        pr_err("%s: xbl_hlos_hob_init returned error:%d \n", __func__, rc);
        goto act_mode_exit;
    }
    act_mode = readb_relaxed(&XblHlosHob->IsActMode);
    pr_err("%s: get_act_mode:%d \n", __func__, act_mode);
act_mode_exit:
    return act_mode;
}
EXPORT_SYMBOL(get_act_mode);

int get_pmic_reset_reason()
{
    int rc = 0;
    int pmic_reset_reason = 0;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
    {
        pr_err("%s: xbl_hlos_hob_init returned error:%d \n", __func__, rc);
        goto get_pmic_reset_reason_exit;
    }
    pmic_reset_reason = readb_relaxed(&XblHlosHob->PmicResetReason);
    pr_err("%s: get_pmic_reset_reason:%d \n", __func__, pmic_reset_reason);
get_pmic_reset_reason_exit:
    return pmic_reset_reason;
}
EXPORT_SYMBOL(get_pmic_reset_reason);

int get_touch_fw_version(char *touch_version)
{
    int rc = 0;
    pr_info("%s: get_manuf_mode:\n", __func__);
    rc = xbl_hlos_hob_init();
    if(rc < 0)
    {
        pr_err("%s: get_touch_fw_version returned error:%d \n", __func__, rc);
        strncpy(touch_version,"0",1);
        goto touch_fw_version;
    }
    strncpy(touch_version,XblHlosHob->TouchFWVersion,MAX_VERSION_LEN);
    pr_info("%s: get_touch_fw_version:%s \n", __func__, touch_version);
touch_fw_version:
    return rc;
}
EXPORT_SYMBOL(get_touch_fw_version);

int xbl_hlos_hob_init()
{
    int rc = 0;
    struct device_node *np;
     if(XblHlosHob != NULL){
          pr_err("%s: Init already called\n", __func__);
          goto xbl_hlos_hob_init_exit;
     }

    np = of_find_compatible_node(NULL, NULL, "surface,oem-smem");
    if (!np)
    {
        pr_err("%s: can't find surface,oem-smem node\n", __func__);
        rc = -ENODEV;
        goto xbl_hlos_hob_init_exit;
    }

    XblHlosHob = of_iomap(np, 0);
    if (!XblHlosHob)
    {
        pr_err("%s: XblHlosHob: Can't map imem\n", __func__);
        rc = -ENODEV;
        goto xbl_hlos_hob_init_exit;
    }
    pr_err("%s: xbl_hlos_hob_init:%d \n", __func__, rc);
xbl_hlos_hob_init_exit:
    return rc;
}
EXPORT_SYMBOL(xbl_hlos_hob_init);

int xbl_hlos_hob_deinit()
{
    if(XblHlosHob)
        iounmap(XblHlosHob);
    return 0;
}
EXPORT_SYMBOL(xbl_hlos_hob_deinit);
