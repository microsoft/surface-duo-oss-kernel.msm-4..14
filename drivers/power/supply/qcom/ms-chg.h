/*
 * ms-chg.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __MSECHG_H
#define __MSECHG_H

#include <linux/types.h>
#include <linux/regulator/driver.h>
#include "ms-battery-protection.h"

#define MSE_VOTER                     "MSE_VOTER"

#define MAX_NUM_GPIOS                 8
#define MAX_ALLOWED_CL_CURRENT        911 //ma
#define MAX_ALLOWED_TEMPERATURE       2000 //deciC

#define EOC_CHARGE_RATE               2 //%
#define EOC_BOTH_CHARGE_RATE          15 //%

#define RSOC_DISABLE_SMB              90 // %

#define FASTCHARGE_STATE_OF_CHARGE    50
#define STEEP_VOLTAGE_CURVE_SOC       10

#define PACK1_MIN_ALLOWED_ERROR       100
#define PACK2_MIN_ALLOWED_ERROR       100

#define WEAK_PSY_POWER_MW (5000 * 900)

#define BQ27742_PROTSTATUS_HANDLED_BITS    0x07
#define BQ27742_PROTSTATE_HANDLED_BITS     0x0C

#define BQ27742_PROTSTATUS_OVP             0x8
#define BQ27742_SAFETYSTATUS_OVP           0x2

#define PACK1_CD_ALLOWED                   98
#define PACK2_CD_ALLOWED                   96

//  MSE battery subsystem GPIO indices
//  Maps to Pmic_Gpios[]
enum GPIO_INDEX{
    CELL_PACK1_DISC,
    CELL_PACK2_DISC,
    BYPASS_SHDN_N,
    RSHARE_CTRL,
    VREF_CTRL1,
    VREF_CTRL2,
    VREF_CTRL3,
    VREF_CTRL4,
    MAX_GPIO_INDEX
};

enum Cl_Cfg {
    CL_CFG_NONE  = 0x23, //Choosing HW default to ensure sufficient back charging
    CL_CFG_P050C = 0x03,
    CL_CFG_P100C = 0x07,
    CL_CFG_P130C = 0x0B,
    CL_CFG_P140C = 0x13,
    CL_CFG_P160C = 0x0F,
    CL_CFG_P180C = 0x17,
    CL_CFG_P200C = 0x1B,
    CL_CFG_P210C = 0x01,
    CL_CFG_P220C = 0x27,
    CL_CFG_P230C = 0x1F,
    CL_CFG_P250C = 0x33,
    CL_CFG_P260C = 0x2F,
    CL_CFG_P270C = 0x37,
    CL_CFG_P290C = 0x3B,
    CL_CFG_P310C = 0x3F,
    CL_CFG_P370C = 0x05,
    CL_CFG_P450C = 0x09,
    CL_CFG_P530C = 0x11,
    CL_CFG_P600C = 0x0D,
    CL_CFG_P670C = 0x15,
    CL_CFG_P730C = 0x19,
    CL_CFG_P800C = 0x25,
    CL_CFG_P850C = 0x1D,
    CL_CFG_P910C = 0x31,
    CL_CFG_P960C = 0x2D,
    CL_CFG_1P01C = 0x35,
    CL_CFG_1P06C = 0x39,
    CL_CFG_1P15C = 0x3D,
    CL_CFG_MAX   = 29
};

//  FG Instances
typedef enum {
    FGAUGE_001,   // Instance 01 of FG
    FGAUGE_002,   // Instance 02 of FG
    MAX_NUM_FGS
} FGAUGE_INSTANCE;

//  MSE battery subsystem runtime states
enum RUNTIME_STATE {
    INVALID_STATE,
    DISCHARGE_STATE,
    CP1_STATE,
    CP2_STATE,
    CP3_STATE,
    EOC_STATE,
    CHARGE_BALANCING_STATE,
    MAX_RUNTIME_STATE
};

//  Packs on MSE battery subsystem
enum PACK_INSTANCE {
    PACK_NONE,
    PACK_001,
    PACK_002,
    PACK_BOTH
};

//  MSE battery subsystem JEITA zones
enum JEITA_ZONES {
    INVALID_ZONE,
    EXTREME_COLD_ZONE,
    HARD_COLD_ZONE,
    SOFT_COLD_ZONE,
    STANDARD_ZONE,
    SOFT_HOT_ZONE,
    HARD_HOT_ZONE,
    EXTREME_HOT_ZONE,
    MAX_JEITA_ZONES
};

struct Cl_Cfg_Table
{
    uint32_t ChargeCurrent;                      // ChargeCurrent (mA)
    enum Cl_Cfg CLCfg;
};

typedef struct MSE_PACK_CONFIG
{
    uint32_t FCC;             // MAX FCC (mA)
    uint32_t SOCCheckPoint1;  // Check point1 during charging (%)
    uint32_t CP1ChargingRate; // Charge rate applied upto CP1
    uint32_t SOCCheckPoint2;  // Check point2 during charging (%)
    uint32_t CP2ChargingRate; // Charge rate applied CP2
    uint32_t SOCCheckPoint3;  // Check point3 during charging (%)
    uint32_t CP3ChargingRate; // Charge rate applied beyond CP2 up to EOC
    int32_t ExtremeColdLimit; // Extreme Cold limit - Temperature below which charging will be disabled (deciC)
    int32_t HardColdLimit;   // Hard Cold limit - Temperature at which charge current will be reduced to JEITA compensated value (deciC)
    int32_t SoftColdLimit;   // Soft Cold limit - Temperature at which charge current will be reduced to JEITA compensated value (deciC)
    int32_t SoftHotLimit;    // Soft Hot limit - Temperature at which charge current will be reduced to JEITA compensated value (deciC)
    int32_t HardHotLimit;    // Hard Hot limit - Temperature at which charge current/float voltage will be reduced to JEITA compensated value (deciC)
    int32_t ExtremeHotLimit; // Extreme Hot limit - Temperature above which charging will be disabled (deciC)
    int32_t TempHyst;        // Temperature Hysteresis to be applied when transitioning to a lower JEITA zone (deciC)
    uint32_t CCCompHCL;       // Charge rate applied when in Hard Cold limit (%)
    uint32_t CCCompSCLandCP1; // Charge rate applied when in Soft Cold limit and in Check Point 1 state (%)
    uint32_t CCCompSCLandCP2; // Charge rate applied when in Soft Cold limit and in Check Point 2 state (%)
    uint32_t CCCompSCLandCP3; // Charge rate applied when in Soft Cold limit and in Check Point 3 state (%)
    uint32_t CCCompSHLandCP1; // Charge rate applied when in Soft Hard limit and in Check Point 1 state (%)
    uint32_t CCCompHHL;       // Charge rate applied when in Hard Hot limit (%)
} *PMSE_PACK_CONFIG;

typedef struct MSE_BATTERY_CONFIG
{
    uint32_t VDelta;                                                 // Vdelta threshold (mv)
    uint32_t SOCDelta;                                               // SOCdelta threshold (%)
    uint32_t SOCDeltaMAX;                                            // SOCdelta allowed threshold (%)
    uint32_t SOCDeltaHYS;                                            // Hysteresis to be applied when resuming charging on both Packs (%)
    uint32_t NumPacks;                                               // Total # of packs in MSE battery subsystem (n)
    uint32_t FloatVoltageHHL;                                        // Float voltage in JEITA ZONE hard hot limit (uV)
    int32_t  FloatVoltageMax;                                        // Max Float voltage (uV)
    uint32_t AutoRechargeThreshold;                                  // RSOC at which we resume charging
    uint32_t AutoRechargeScalingLb;                                  // RSOC at which we resume reporting actual rsoc during discharge phase.
    uint32_t LowEndBootShutdownLb;                                   // RSOC at which we report 1%
    uint32_t LowEndBootShutdownUb;                                   // Upper threshold for lower end RSOC manipulation
    struct MSE_PACK_CONFIG MSEPackConfig[MAX_NUM_FGS];               // Holds Individual pack configurations
} *PMSE_BATTERY_CONFIG;

typedef struct FG_BATT_FAULT_STATUS
{
    bool active_fault;
    bool ovp_detected;
    uint16_t  safety_status;
    uint8_t   protector_status;
    uint8_t   protector_state;
} *PFG_BATT_FAULT_STATUS;

typedef struct MSE_PACK_RT_STATUS
{
    bool ChargingEnabled;
    bool CFETOn;
    enum RUNTIME_STATE RuntimeState;
    uint32_t RuntimeChargeRate;
    uint32_t RuntimeChargeCurrent;
    int32_t RuntimeChargeVoltage;
    struct FG_BATT_FAULT_STATUS FaultStatus;
    enum JEITA_ZONES CurrentJeitaZone;
    bool PackDisconnect;
} *PMSE_PACK_RT_STATUS;

typedef struct FG_BATT_STATUS_TYPE
{
    uint16_t monotonic_soc; //soc applied with smoothing
    uint32_t vbatt;
    int32_t  ibatt;
    int32_t  temperature;
} *PFG_BATT_STATUS_TYPE;

struct MSEHw_Config {
    bool IsDebugBattery;
    bool InDischargePhase;
    bool IsWeakPSU;
    bool InAutoRechargePhase;
    bool IsThermalThrottlingActive;
    int CombinedRsoc;
    uint32_t TotalGpios;
    uint32_t TotalCLGpios;
    uint32_t Pmic_Gpios[MAX_NUM_GPIOS];
    uint32_t PsuHeartbeatTimer;
    uint32_t BattHeartbeatTimer;
    uint8_t RunTimeCLCfg;
    uint64_t OverrideCLCfgCurrent;
    enum PACK_INSTANCE ChargingEnabledPack;
    struct MSE_BATTERY_CONFIG MSEBatterySubsystemConfig;
    struct power_supply	*MSEExtFg[MAX_NUM_FGS];
    struct FG_BATT_STATUS_TYPE MSEBatteryStatus[MAX_NUM_FGS];
    struct MSE_PACK_RT_STATUS MSEPackRTStatus[MAX_NUM_FGS];
    struct BATTERY_PROTECTION BatteryProtection;  // MSE battery protection
};

/****************************************
***** MSEChgHw Function Definitions  ****
*****************************************/
int MSEHwInit(void *chg_input);
void MSEHwShutdown(void *chg_input);
void MSEHwDebugfs(struct dentry *parent, void *data);
int MSEHwGetBatteryCapacity(struct MSEHw_Config *MSEHw_Config, int rawRsoc);

#endif /* __MSECHG_H */
