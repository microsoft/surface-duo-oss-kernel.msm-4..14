/*
 * OMAP Voltage Management Routines
 *
 * Author: Thara Gopinath	<thara@ti.com>
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_VOLTAGE_H
#define __ARCH_ARM_MACH_OMAP2_VOLTAGE_H

#include <linux/err.h>

#define VOLTSCALE_VPFORCEUPDATE		1
#define VOLTSCALE_VCBYPASS		2

/*
 * OMAP3 GENERIC setup times. Revisit to see if these needs to be
 * passed from board or PMIC file
 */
#define OMAP3_CLKSETUP		0xff
#define OMAP3_VOLTOFFSET	0xff
#define OMAP3_VOLTSETUP2	0xff

/* Voltage value defines */
#define OMAP3430_VDD_MPU_OPP1_UV		975000
#define OMAP3430_VDD_MPU_OPP2_UV		1075000
#define OMAP3430_VDD_MPU_OPP3_UV		1200000
#define OMAP3430_VDD_MPU_OPP4_UV		1270000
#define OMAP3430_VDD_MPU_OPP5_UV		1350000

#define OMAP3430_VDD_CORE_OPP1_UV		975000
#define OMAP3430_VDD_CORE_OPP2_UV		1050000
#define OMAP3430_VDD_CORE_OPP3_UV		1150000

#define OMAP3630_VDD_MPU_OPP50_UV		1012500
#define OMAP3630_VDD_MPU_OPP100_UV		1200000
#define OMAP3630_VDD_MPU_OPP120_UV		1325000
#define OMAP3630_VDD_MPU_OPP1G_UV		1375000

#define OMAP3630_VDD_CORE_OPP50_UV		1000000
#define OMAP3630_VDD_CORE_OPP100_UV		1200000

#define OMAP4430_VDD_MPU_OPP50_UV		930000
#define OMAP4430_VDD_MPU_OPP100_UV		1100000
#define OMAP4430_VDD_MPU_OPPTURBO_UV		1260000
#define OMAP4430_VDD_MPU_OPPNITRO_UV		1350000

#define OMAP4430_VDD_IVA_OPP50_UV		930000
#define OMAP4430_VDD_IVA_OPP100_UV		1100000
#define OMAP4430_VDD_IVA_OPPTURBO_UV		1260000

#define OMAP4430_VDD_CORE_OPP50_UV		930000
#define OMAP4430_VDD_CORE_OPP100_UV		1100000

/**
 * struct voltagedomain - omap voltage domain global structure.
 * @name:	Name of the voltage domain which can be used as a unique
 *		identifier.
 */
struct voltagedomain {
	char *name;
};

/**
 * struct omap_volt_data - Omap voltage specific data.
 * @voltage_nominal:	The possible voltage value in uV
 * @sr_efuse_offs:	The offset of the efuse register(from system
 *			control module base address) from where to read
 *			the n-target value for the smartreflex module.
 * @sr_errminlimit:	Error min limit value for smartreflex. This value
 *			differs at differnet opp and thus is linked
 *			with voltage.
 * @vp_errorgain:	Error gain value for the voltage processor. This
 *			field also differs according to the voltage/opp.
 */
struct omap_volt_data {
	u32	volt_nominal;
	u32	sr_efuse_offs;
	u8	sr_errminlimit;
	u8	vp_errgain;
};

/**
 * struct omap_volt_pmic_info - PMIC specific data required by voltage driver.
 * @slew_rate:	PMIC slew rate (in uv/us)
 * @step_size:	PMIC voltage step size (in uv)
 * @vsel_to_uv:	PMIC API to convert vsel value to actual voltage in uV.
 * @uv_to_vsel:	PMIC API to convert voltage in uV to vsel value.
 */
struct omap_volt_pmic_info {
	int slew_rate;
	int step_size;
	u32 on_volt;
	u32 onlp_volt;
	u32 ret_volt;
	u32 off_volt;
	u16 volt_setup_time;
	u8 vp_erroroffset;
	u8 vp_vstepmin;
	u8 vp_vstepmax;
	u8 vp_vddmin;
	u8 vp_vddmax;
	u8 vp_timeout_us;
	u8 i2c_slave_addr;
	u8 pmic_reg;
	unsigned long (*vsel_to_uv) (const u8 vsel);
	u8 (*uv_to_vsel) (unsigned long uV);
};

/* Voltage processor register offsets */
struct vp_reg_offs {
	u8 vpconfig;
	u8 vstepmin;
	u8 vstepmax;
	u8 vlimitto;
	u8 vstatus;
	u8 voltage;
};

/* Voltage Processor bit field values, shifts and masks */
struct vp_reg_val {
	/* PRM module */
	u16 prm_mod;
	/* VPx_VPCONFIG */
	u32 vpconfig_erroroffset;
	u16 vpconfig_errorgain;
	u32 vpconfig_errorgain_mask;
	u8 vpconfig_errorgain_shift;
	u32 vpconfig_initvoltage_mask;
	u8 vpconfig_initvoltage_shift;
	u32 vpconfig_timeouten;
	u32 vpconfig_initvdd;
	u32 vpconfig_forceupdate;
	u32 vpconfig_vpenable;
	/* VPx_VSTEPMIN */
	u8 vstepmin_stepmin;
	u16 vstepmin_smpswaittimemin;
	u8 vstepmin_stepmin_shift;
	u8 vstepmin_smpswaittimemin_shift;
	/* VPx_VSTEPMAX */
	u8 vstepmax_stepmax;
	u16 vstepmax_smpswaittimemax;
	u8 vstepmax_stepmax_shift;
	u8 vstepmax_smpswaittimemax_shift;
	/* VPx_VLIMITTO */
	u8 vlimitto_vddmin;
	u8 vlimitto_vddmax;
	u16 vlimitto_timeout;
	u8 vlimitto_vddmin_shift;
	u8 vlimitto_vddmax_shift;
	u8 vlimitto_timeout_shift;
	/* PRM_IRQSTATUS*/
	u32 tranxdone_status;
};

/* Voltage controller registers and offsets */
struct vc_reg_info {
	/* PRM module */
	u16 prm_mod;
	/* VC register offsets */
	u8 smps_sa_reg;
	u8 smps_volra_reg;
	u8 bypass_val_reg;
	u8 cmdval_reg;
	u8 voltsetup_reg;
	/*VC_SMPS_SA*/
	u8 smps_sa_shift;
	u32 smps_sa_mask;
	/* VC_SMPS_VOL_RA */
	u8 smps_volra_shift;
	u32 smps_volra_mask;
	/* VC_BYPASS_VAL */
	u8 data_shift;
	u8 slaveaddr_shift;
	u8 regaddr_shift;
	u32 valid;
	/* VC_CMD_VAL */
	u8 cmd_on_shift;
	u8 cmd_onlp_shift;
	u8 cmd_ret_shift;
	u8 cmd_off_shift;
	u32 cmd_on_mask;
	/* PRM_VOLTSETUP */
	u8 voltsetup_shift;
	u32 voltsetup_mask;
};


/**
 * omap_vdd_dep_volt - Table containing the parent vdd voltage and the
 *			dependent vdd voltage corresponding to it.
 *
 * @main_vdd_volt	: The main vdd voltage
 * @dep_vdd_volt	: The voltage at which the dependent vdd should be
 *			  when the main vdd is at <main_vdd_volt> voltage
 */
struct omap_vdd_dep_volt {
	u32 main_vdd_volt;
	u32 dep_vdd_volt;
};

/**
 * omap_vdd_dep_info - Dependent vdd info
 *
 * @name		: Dependent vdd name
 * @voltdm		: Dependent vdd pointer
 * @dep_table		: Table containing the dependent vdd voltage
 *			  corresponding to every main vdd voltage.
 */
struct omap_vdd_dep_info {
	char *name;
	struct voltagedomain *voltdm;
	struct omap_vdd_dep_volt *dep_table;
};


/**
 * omap_vdd_info - Per Voltage Domain info
 *
 * @volt_data		: voltage table having the distinct voltages supported
 *			  by the domain and other associated per voltage data.
 * @pmic_info		: pmic specific parameters which should be populted by
 *			  the pmic drivers.
 * @vp_offs		: structure containing the offsets for various
 *			  vp registers
 * @vp_reg		: the register values, shifts, masks for various
 *			  vp registers
 * @vc_reg		: structure containing various various vc registers,
 *			  shifts, masks etc.
 * @voltdm		: pointer to the voltage domain structure
 * @debug_dir		: debug directory for this voltage domain.
 * @curr_volt		: current voltage for this vdd.
 * @ocp_mod		: The prm module for accessing the prm irqstatus reg.
 * @prm_irqst_reg	: prm irqstatus register.
 * @vp_enabled		: flag to keep track of whether vp is enabled or not
 * @volt_scale		: API to scale the voltage of the vdd.
 */
struct omap_vdd_info {
	struct omap_volt_data *volt_data;
	struct omap_volt_pmic_info *pmic_info;
	struct vp_reg_offs vp_offs;
	struct vp_reg_val vp_reg;
	struct vc_reg_info vc_reg;
	struct voltagedomain voltdm;
	struct omap_vdd_dep_info *dep_vdd_info;
	int nr_dep_vdd;
	struct dentry *debug_dir;
	u32 curr_volt;
	u16 ocp_mod;
	u8 prm_irqst_reg;
	bool vp_enabled;
	u32 (*read_reg) (u16 mod, u8 offset);
	void (*write_reg) (u32 val, u16 mod, u8 offset);
	int (*volt_scale) (struct omap_vdd_info *vdd,
		unsigned long target_volt);
};

unsigned long omap_vp_get_curr_volt(struct voltagedomain *voltdm);
void omap_vp_enable(struct voltagedomain *voltdm);
void omap_vp_disable(struct voltagedomain *voltdm);
int omap_voltage_scale_vdd(struct voltagedomain *voltdm,
		unsigned long target_volt);
void omap_voltage_reset(struct voltagedomain *voltdm);
void omap_voltage_get_volttable(struct voltagedomain *voltdm,
		struct omap_volt_data **volt_data);
struct omap_volt_data *omap_voltage_get_voltdata(struct voltagedomain *voltdm,
		unsigned long volt);
unsigned long omap_voltage_get_nom_volt(struct voltagedomain *voltdm);
struct dentry *omap_voltage_get_dbgdir(struct voltagedomain *voltdm);
#ifdef CONFIG_PM
int omap_voltage_register_pmic(struct voltagedomain *voltdm,
		struct omap_volt_pmic_info *pmic_info);
void omap_change_voltscale_method(struct voltagedomain *voltdm,
		int voltscale_method);
/* API to get the voltagedomain pointer */
struct voltagedomain *omap_voltage_domain_lookup(char *name);

int omap_voltage_late_init(void);
#else
static inline int omap_voltage_register_pmic(struct voltagedomain *voltdm,
		struct omap_volt_pmic_info *pmic_info)
{
	return -EINVAL;
}
static inline  void omap_change_voltscale_method(struct voltagedomain *voltdm,
		int voltscale_method) {}
static inline int omap_voltage_late_init(void)
{
	return -EINVAL;
}
static inline struct voltagedomain *omap_voltage_domain_lookup(char *name)
{
	return ERR_PTR(-EINVAL);
}
#endif

#endif
