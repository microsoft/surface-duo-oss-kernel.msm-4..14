/*
 * ms-chg.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/pmic-voter.h>
#include "ms-chg.h"
#include "smb5-lib.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/reboot.h>

/***************************
 *    MSEChgHw Functions   *
 **************************/
#define REPORT_RSOC_FULL 100

// Macro to return absolute value of the difference between two values
#define ABS_DIFF(v1, v2)  (((v1) >= (v2)) ? ((v1) - (v2)) : ((v2) - (v1)))

// Macro to return absolute value
#define ABS(v)  (((v) >= 0) ? (v) : (-v))

#define CL_OVERRIDE(OverrideCLCfgCurrent)	(OverrideCLCfgCurrent <= MAX_ALLOWED_CL_CURRENT ? 1:0) // allow user to override the CL config

struct Cl_Cfg_Table CLCfgTable[CL_CFG_MAX] =
    {{ 0,   CL_CFG_NONE  },
     { 42,  CL_CFG_P050C },
     { 80,  CL_CFG_P100C },
     { 99,  CL_CFG_P130C },
     { 113, CL_CFG_P140C },
     { 128, CL_CFG_P160C },
     { 142, CL_CFG_P180C },
     { 157, CL_CFG_P200C },
     { 158, CL_CFG_P210C },
     { 171, CL_CFG_P220C },
     { 182, CL_CFG_P230C },
     { 197, CL_CFG_P250C },
     { 207, CL_CFG_P260C },
     { 216, CL_CFG_P270C },
     { 228, CL_CFG_P290C },
     { 247, CL_CFG_P310C },
     { 297, CL_CFG_P370C },
     { 361, CL_CFG_P450C },
     { 421, CL_CFG_P530C },
     { 476, CL_CFG_P600C },
     { 532, CL_CFG_P670C },
     { 580, CL_CFG_P730C },
     { 633, CL_CFG_P800C },
     { 677, CL_CFG_P850C },
     { 722, CL_CFG_P910C },
     { 763, CL_CFG_P960C },
     { 803, CL_CFG_1P01C },
     { 843, CL_CFG_1P06C },
     { MAX_ALLOWED_CL_CURRENT,CL_CFG_1P15C }};

static int MSEHwConfigureCLR(struct MSEHw_Config *MSEHw_Config, uint32_t Pack1ChargeCurrent)
{
    int rc = 0;
    uint8_t GpioIndex = 0;
    uint8_t Data = 0;
    uint8_t CLCfgIndex = 0;
    uint8_t CLCfg = 0;
    uint8_t CLGpioStartingIndex = (uint8_t)(MSEHw_Config->TotalGpios - MSEHw_Config->TotalCLGpios);

    // Sanity check the incoming parameter
    if(Pack1ChargeCurrent > MAX_ALLOWED_CL_CURRENT)
    {
        Pack1ChargeCurrent = MAX_ALLOWED_CL_CURRENT;
    }

    // Check if user has set a CL using debug fs
    if(CL_OVERRIDE(MSEHw_Config->OverrideCLCfgCurrent))
    {
        pr_err("we are setting CL_OVERRIDE");
		Pack1ChargeCurrent = MSEHw_Config->OverrideCLCfgCurrent;
    }

    // Identify the most suitable current limit config based on runtime Pack1ChargeCurrent
    for (CLCfgIndex = 0; CLCfgIndex < CL_CFG_MAX; CLCfgIndex++)
    {
        if (Pack1ChargeCurrent <= CLCfgTable[CLCfgIndex].ChargeCurrent)
        {
            CLCfg = (uint8_t)CLCfgTable[CLCfgIndex].CLCfg;
            break;
        }
    }

    if (MSEHw_Config->RunTimeCLCfg != CLCfg)
    {
        for (GpioIndex = CLGpioStartingIndex, CLCfgIndex = 0; GpioIndex < MSEHw_Config->TotalGpios; GpioIndex++, CLCfgIndex++)
        {
            Data = ((CLCfg >> CLCfgIndex) & 0x1);
            if (gpio_is_valid(MSEHw_Config->Pmic_Gpios[GpioIndex]))
            {
                gpio_direction_output(MSEHw_Config->Pmic_Gpios[GpioIndex], Data);
                if (rc < 0)
                {
                    pr_err("gpio_set_value failed to drive CL Cfg:%d failed:%d", GpioIndex, rc);
                    goto Exit;
                }
            }
        }
        pr_err("Successfully updated CLCfg from 0x%x to 0x%x to target :%dma", MSEHw_Config->RunTimeCLCfg, CLCfg, Pack1ChargeCurrent);
        MSEHw_Config->RunTimeCLCfg = CLCfg;
    }

Exit:
    return rc;
}

static int MSEHwGetHwState(struct MSEHw_Config *MSEHw_Config)
{
    int rc = 0;
    uint8_t GpioIndex;
    uint8_t Data;
    uint32_t TotalGpioResources = MSEHw_Config->TotalGpios;
    uint32_t GpioStatus = 0;

    // Query states of all MSE GPIO pins
    for (GpioIndex = 0; GpioIndex < TotalGpioResources; GpioIndex++)
    {
        if(gpio_is_valid(MSEHw_Config->Pmic_Gpios[GpioIndex]))
        {
            Data = gpio_get_value(MSEHw_Config->Pmic_Gpios[GpioIndex]);
            if(Data < 0)
            {
                pr_err("failed to read %d error:%d", GpioIndex, rc);
                rc = -EIO;
            }
        }
        GpioStatus |= (Data << (GpioIndex + 1));
    }
    pr_err("MSE GPIO pins have a value of: %d", GpioStatus);
    return rc;
}

static int MSEHwPackDisconnect(struct MSEHw_Config *MSEHw_Config, bool P1Disconnect, bool P2Disconnect)
{
    int rc = 0;
    uint8_t Data;
    Data = (P1Disconnect ? 1 : 0);
    if (gpio_is_valid(MSEHw_Config->Pmic_Gpios[CELL_PACK1_DISC]))
    {
        gpio_direction_output(MSEHw_Config->Pmic_Gpios[CELL_PACK1_DISC], Data);
        if (rc < 0)
        {
            pr_err("gpio_set_value failed to drive CL Cfg:%d failed:%d", CELL_PACK1_DISC, rc);
            goto Exit;
        }
    }
    Data = (P2Disconnect ? 1 : 0);
    if (gpio_is_valid(MSEHw_Config->Pmic_Gpios[CELL_PACK2_DISC]))
    {
        gpio_direction_output(MSEHw_Config->Pmic_Gpios[CELL_PACK2_DISC], Data);
        if (rc < 0)
        {
            pr_err("gpio_set_value failed to drive CL Cfg:%d failed:%d", CELL_PACK2_DISC, rc);
            goto Exit;
        }
    }

    MSEHw_Config->MSEPackRTStatus[FGAUGE_001].PackDisconnect = gpio_get_value(MSEHw_Config->Pmic_Gpios[CELL_PACK1_DISC]);
    MSEHw_Config->MSEPackRTStatus[FGAUGE_002].PackDisconnect = gpio_get_value(MSEHw_Config->Pmic_Gpios[CELL_PACK2_DISC]);

Exit:
    pr_debug("Pack1 CD:%d    Pack2 CD:%d    ", gpio_get_value(MSEHw_Config->Pmic_Gpios[CELL_PACK1_DISC]), gpio_get_value(MSEHw_Config->Pmic_Gpios[CELL_PACK2_DISC]));
    return rc;
}

static int MSEHwGetCFETStatus(struct MSEHw_Config *MSEHw_Config)
{
    int rc = 0;
    int i = 0;
    union power_supply_propval prop_pack = {0, };

    for (i = 0; i < MAX_NUM_FGS; i++)
    {
        if (power_supply_get_property(MSEHw_Config->MSEExtFg[i], POWER_SUPPLY_PROP_STATUS, &prop_pack))
        {
            pr_err("Could not read pack[%d] extfg POWER_SUPPLY_PROP_STATUS\n", i);
            rc |= -ENODATA;
        }
        else
        {
            MSEHw_Config->MSEPackRTStatus[i].CFETOn = (prop_pack.intval == POWER_SUPPLY_STATUS_FULL) ? false : true;
        }
    }

    pr_debug("Pack1 CFETOn:%d   Pack2 CFETOn:%d", MSEHw_Config->MSEPackRTStatus[FGAUGE_001].CFETOn, MSEHw_Config->MSEPackRTStatus[FGAUGE_002].CFETOn);
    return rc;
}

#define MAX_OVP_RETRIES  5
static int MSEHwGetFaultStatus(struct MSEHw_Config *MSEHw_Config)
{
    int rc = 0;
    int i = 0;
    union power_supply_propval prop_pack = {0, };
    static int TriggerOVPRecoveryNumber[MAX_NUM_FGS] = {0,0};

    for (i = 0; i < MAX_NUM_FGS; i++)
    {
        if (power_supply_get_property(MSEHw_Config->MSEExtFg[i], POWER_SUPPLY_PROP_PROTECTOR_STATUS, &prop_pack))
        {
            pr_err("Could not read pack[%d] extfg protector status\n", i);
            rc |= -ENODATA;
        }
        else
        {
            MSEHw_Config->MSEPackRTStatus[i].FaultStatus.protector_status = prop_pack.intval;
        }

        if (power_supply_get_property(MSEHw_Config->MSEExtFg[i], POWER_SUPPLY_PROP_PROTECTOR_STATE, &prop_pack))
        {
            pr_err("Could not read pack[%d] extfg protector state\n", i);
            rc |= -ENODATA;
        }
        else
        {
            MSEHw_Config->MSEPackRTStatus[i].FaultStatus.protector_state = prop_pack.intval;
        }

        if (power_supply_get_property(MSEHw_Config->MSEExtFg[i], POWER_SUPPLY_PROP_VOLTAGE_OCV, &prop_pack))
        {
            pr_err("Could not read pack[%d] extfg safety status\n", i);
            rc |= -ENODATA;
        }
        else
        {
            MSEHw_Config->MSEPackRTStatus[i].FaultStatus.safety_status = prop_pack.intval;
        }

        if ((MSEHw_Config->MSEPackRTStatus[i].FaultStatus.protector_status & BQ27742_PROTSTATUS_HANDLED_BITS) ||
            (MSEHw_Config->MSEPackRTStatus[i].FaultStatus.protector_state & BQ27742_PROTSTATE_HANDLED_BITS))
        {
            MSEHw_Config->MSEPackRTStatus[i].FaultStatus.active_fault = true;
        }
        else
        {
            MSEHw_Config->MSEPackRTStatus[i].FaultStatus.active_fault = false;
        }

        if((MSEHw_Config->MSEPackRTStatus[i].FaultStatus.protector_status & BQ27742_PROTSTATUS_OVP) ||
            (MSEHw_Config->MSEPackRTStatus[i].FaultStatus.safety_status & BQ27742_SAFETYSTATUS_OVP))
        {
            TriggerOVPRecoveryNumber[i]++;
            if(MAX_OVP_RETRIES == TriggerOVPRecoveryNumber[i])
                MSEHw_Config->MSEPackRTStatus[i].FaultStatus.ovp_detected = true;
        }
        else
        {
            MSEHw_Config->MSEPackRTStatus[i].FaultStatus.ovp_detected = false;
            TriggerOVPRecoveryNumber[i] = 0;
        }

    }

    pr_debug("Pack 1 Active Fault: %d  OVP Detected: %d        Pack 2 Active Fault: %d  OVP Detected: %d",
            MSEHw_Config->MSEPackRTStatus[FGAUGE_001].FaultStatus.active_fault,
            MSEHw_Config->MSEPackRTStatus[FGAUGE_001].FaultStatus.ovp_detected,
            MSEHw_Config->MSEPackRTStatus[FGAUGE_002].FaultStatus.active_fault,
            MSEHw_Config->MSEPackRTStatus[FGAUGE_002].FaultStatus.ovp_detected);

    return rc;
}

static int MSEHwExtFgGetBattInfo(struct MSEHw_Config *MSEHw_Config)
{
    union power_supply_propval prop_pack = {0, };
    int rc = 0;
    int i = 0;

    if (MSEHw_Config->IsDebugBattery)
    {
        // don't query extfgs when debug battery is present
        pr_err("debug battery detected, extfgs will not be queried");
        rc = 0;
        goto Exit;
    }

    for (i = 0; i < MAX_NUM_FGS; i++)
    {
        // query monotonic soc
        if (power_supply_get_property(MSEHw_Config->MSEExtFg[i], POWER_SUPPLY_PROP_CAPACITY, &prop_pack))
        {
            pr_err("Could not read pack[%d] soc\n", i);
            rc |= -ENODATA;
        }
        else
        {
            MSEHw_Config->MSEBatteryStatus[i].monotonic_soc = prop_pack.intval;
        }

        // query vbatt
        if (power_supply_get_property(MSEHw_Config->MSEExtFg[i], POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop_pack))
        {
            pr_err("Could not read pack[%d] vbatt\n", i);
            rc |= -ENODATA;
        }
        else
        {
            MSEHw_Config->MSEBatteryStatus[i].vbatt = (prop_pack.intval / 1000);
        }

        // query ibatt
        if (power_supply_get_property(MSEHw_Config->MSEExtFg[i], POWER_SUPPLY_PROP_CURRENT_NOW, &prop_pack))
        {
            pr_err("Could not read pack[%d] ibatt\n", i);
            rc |= -ENODATA;
        }
        else
        {
            MSEHw_Config->MSEBatteryStatus[i].ibatt = (prop_pack.intval / 1000);
        }

        // query temperature
        if (power_supply_get_property(MSEHw_Config->MSEExtFg[i], POWER_SUPPLY_PROP_TEMP, &prop_pack))
        {
            pr_err("Could not read pack[%d] temperature\n", i);
            rc |= -ENODATA;
        }
        else
        {
            MSEHw_Config->MSEBatteryStatus[i].temperature = prop_pack.intval; // deci C
        }

        pr_debug("Pack[%d]: monotonic soc: %d, vbatt: %d mV, ibatt: %d mA, temperature: %d \n", i,
                MSEHw_Config->MSEBatteryStatus[i].monotonic_soc,
                MSEHw_Config->MSEBatteryStatus[i].vbatt,
                MSEHw_Config->MSEBatteryStatus[i].ibatt,
                MSEHw_Config->MSEBatteryStatus[i].temperature);
    }

Exit:
    return rc;
}

static bool MSEHwGetWeakPSUStatus(struct smb_charger *chg)
{
    int rc = 0;
    bool isWeakPSU = false;
    union power_supply_propval pval = {0, };
    int vbus_mv = 0;
    int icl_ma = 0;
    int input_power_mw = 0;

    // get Input voltage and current by querying USB powersupply
    rc = power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &pval);
    if (rc < 0)
    {
        pr_err("Couldn't get usb psy online property rc=%d\n", rc);
        goto Exit;
    }
    vbus_mv = (pval.intval/1000);

    icl_ma = (get_effective_result_locked(chg->usb_icl_votable)/1000);
    if (icl_ma < 0)
    {
        /* no client is voting, so get the total current directly from USB powersupply */
        rc = power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_HW_CURRENT_MAX, &pval);
        if (rc < 0)
        {
             pr_err("Couldn't get max current rc=%d\n", rc);
             goto Exit;
        }
        icl_ma = pval.intval/1000;
    }
    input_power_mw = (vbus_mv * icl_ma);

    // update isWeakPSU as needed
    isWeakPSU = (input_power_mw > WEAK_PSY_POWER_MW) ? false : true;

Exit:
    return isWeakPSU;
}


void MSEHwApplyJeitaCorrections(struct MSEHw_Config *MSEHw_Config, uint32_t *pMSEChgFloatVoltage)
{
    PMSE_PACK_CONFIG MSEPackConfig = NULL;
    PFG_BATT_STATUS_TYPE MSEBatteryStatus = NULL;
    PMSE_PACK_RT_STATUS MSEPackRTStatus = NULL;
    uint32_t NumPacks = 0;
    uint32_t CurrentPack = 0;
    static int32_t PreviousPackTemperature[MAX_NUM_FGS] = {INT_MIN,INT_MIN};
    static int32_t PreviousPackJeitaZone[MAX_NUM_FGS] = {INVALID_ZONE,INVALID_ZONE};
    static int32_t PreviousHystUbLimit[MAX_NUM_FGS] = {INT_MIN,INT_MIN};
    int32_t CurrentHystUbLimit = 0;
    int32_t TempHyst = 0;
    static bool InHyst[MAX_NUM_FGS] = {false,false};
    *pMSEChgFloatVoltage = MSEHw_Config->MSEBatterySubsystemConfig.FloatVoltageMax; //Initially setting to fv-max-uv

    // Cache Pack Config
    MSEPackConfig = (PMSE_PACK_CONFIG)&MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig;

    // Cache Pack Status
    MSEBatteryStatus = (PFG_BATT_STATUS_TYPE)&MSEHw_Config->MSEBatteryStatus;
    NumPacks = MSEHw_Config->MSEBatterySubsystemConfig.NumPacks;

    // Cache Runtime Pack Status
    MSEPackRTStatus = (PMSE_PACK_RT_STATUS)&MSEHw_Config->MSEPackRTStatus;

    // Apply JEITA corrections for each pack
    for(CurrentPack = 0; CurrentPack < NumPacks; CurrentPack++)
    {
        if(MSEPackRTStatus[CurrentPack].ChargingEnabled)
        {
            //Check if pack temperature has risen from last time
            if(MSEBatteryStatus[CurrentPack].temperature >= PreviousPackTemperature[CurrentPack])
            {
                //No need to apply any Hysteresis
                TempHyst = 0;
                if(InHyst[CurrentPack])
                    TempHyst = MSEPackConfig[CurrentPack].TempHyst;
            }
            else
            {
                //need to apply Hysteresis
                InHyst[CurrentPack] = true;
                TempHyst = MSEPackConfig[CurrentPack].TempHyst;
            }

            if(MSEBatteryStatus[CurrentPack].temperature < (MSEPackConfig[CurrentPack].ExtremeColdLimit - TempHyst))
            {
                MSEPackRTStatus[CurrentPack].RuntimeChargeRate = 0;
                MSEPackRTStatus[CurrentPack].RuntimeChargeVoltage = 0;
                MSEPackRTStatus[CurrentPack].CurrentJeitaZone = EXTREME_COLD_ZONE;
                CurrentHystUbLimit = INT_MIN;
            }
            else if(MSEBatteryStatus[CurrentPack].temperature < (MSEPackConfig[CurrentPack].HardColdLimit - TempHyst))
            {
                MSEPackRTStatus[CurrentPack].RuntimeChargeRate = MSEPackConfig[CurrentPack].CCCompHCL;
                MSEPackRTStatus[CurrentPack].RuntimeChargeVoltage = MSEHw_Config->MSEBatterySubsystemConfig.FloatVoltageMax;
                MSEPackRTStatus[CurrentPack].CurrentJeitaZone = HARD_COLD_ZONE;
                CurrentHystUbLimit = MSEPackConfig[CurrentPack].ExtremeColdLimit;
            }
            else if(MSEBatteryStatus[CurrentPack].temperature < (MSEPackConfig[CurrentPack].SoftColdLimit - TempHyst))
            {
                if(MSEPackRTStatus[CurrentPack].RuntimeState == CP1_STATE)
                    MSEPackRTStatus[CurrentPack].RuntimeChargeRate = MSEPackConfig[CurrentPack].CCCompSCLandCP1;
                else if(MSEPackRTStatus[CurrentPack].RuntimeState == CP2_STATE)
                    MSEPackRTStatus[CurrentPack].RuntimeChargeRate = MSEPackConfig[CurrentPack].CCCompSCLandCP2;
                else if(MSEPackRTStatus[CurrentPack].RuntimeState == CP3_STATE)
                    MSEPackRTStatus[CurrentPack].RuntimeChargeRate = MSEPackConfig[CurrentPack].CCCompSCLandCP3;

                MSEPackRTStatus[CurrentPack].RuntimeChargeVoltage = MSEHw_Config->MSEBatterySubsystemConfig.FloatVoltageMax;
                MSEPackRTStatus[CurrentPack].CurrentJeitaZone = SOFT_COLD_ZONE;
                CurrentHystUbLimit = MSEPackConfig[CurrentPack].HardColdLimit;
            }
            else if(MSEBatteryStatus[CurrentPack].temperature < (MSEPackConfig[CurrentPack].SoftHotLimit - TempHyst))
            {
                // No Jeita correction to be applied in STANDARD_ZONE
                MSEPackRTStatus[CurrentPack].RuntimeChargeVoltage = MSEHw_Config->MSEBatterySubsystemConfig.FloatVoltageMax;
                MSEPackRTStatus[CurrentPack].CurrentJeitaZone = STANDARD_ZONE;
                CurrentHystUbLimit = MSEPackConfig[CurrentPack].SoftColdLimit;
            }
            else if(MSEBatteryStatus[CurrentPack].temperature < (MSEPackConfig[CurrentPack].HardHotLimit - TempHyst))
            {
                if(MSEPackRTStatus[CurrentPack].RuntimeState == CP1_STATE)
                    MSEPackRTStatus[CurrentPack].RuntimeChargeRate = MSEPackConfig[CurrentPack].CCCompSHLandCP1;

                MSEPackRTStatus[CurrentPack].RuntimeChargeVoltage = MSEHw_Config->MSEBatterySubsystemConfig.FloatVoltageMax;
                MSEPackRTStatus[CurrentPack].CurrentJeitaZone = SOFT_HOT_ZONE;
                CurrentHystUbLimit = MSEPackConfig[CurrentPack].SoftHotLimit;
            }
            else if(MSEBatteryStatus[CurrentPack].temperature <= (MSEPackConfig[CurrentPack].ExtremeHotLimit - TempHyst))
            {
                MSEPackRTStatus[CurrentPack].RuntimeChargeRate = MSEPackConfig[CurrentPack].CCCompHHL;
                MSEPackRTStatus[CurrentPack].RuntimeChargeVoltage = MSEHw_Config->MSEBatterySubsystemConfig.FloatVoltageHHL;
                MSEPackRTStatus[CurrentPack].CurrentJeitaZone = HARD_HOT_ZONE;
                CurrentHystUbLimit = MSEPackConfig[CurrentPack].HardHotLimit;
            }
            else //MSEBatteryStatus[CurrentPack].temperature > MSEPackConfig[CurrentPack].ExtremeHotLimit
            {
                MSEPackRTStatus[CurrentPack].RuntimeChargeRate = 0;
                MSEPackRTStatus[CurrentPack].RuntimeChargeVoltage = 0;
                MSEPackRTStatus[CurrentPack].CurrentJeitaZone = EXTREME_HOT_ZONE;
                CurrentHystUbLimit = MSEPackConfig[CurrentPack].HardHotLimit;
            }

            //Check if pack jeita has changed from last time
            if ((MSEPackRTStatus[CurrentPack].CurrentJeitaZone > PreviousPackJeitaZone[CurrentPack]) ||
                (TempHyst && (MSEPackRTStatus[CurrentPack].CurrentJeitaZone < PreviousPackJeitaZone[CurrentPack])))
            {
                //No need to apply any Hysteresis
                InHyst[CurrentPack] = false;
                PreviousHystUbLimit[CurrentPack] = CurrentHystUbLimit;
            }

            // Update PreviousPackTemperature and PreviousPackJeitaZone to be used for future calculations
            PreviousPackTemperature[CurrentPack] = MSEBatteryStatus[CurrentPack].temperature;
            PreviousPackJeitaZone[CurrentPack] = MSEPackRTStatus[CurrentPack].CurrentJeitaZone;

            if(MSEPackRTStatus[CurrentPack].RuntimeChargeVoltage < *pMSEChgFloatVoltage)
                *pMSEChgFloatVoltage = MSEPackRTStatus[CurrentPack].RuntimeChargeVoltage;
        }
        else
        {
            // Clear PreviousPackTemperature and PreviousPackJeitaZone until charging is resumed on this pack
            PreviousPackTemperature[CurrentPack] = 0;
            PreviousPackJeitaZone[CurrentPack] = INVALID_ZONE;

        }
        pr_debug("Current JeitaZone for Pack%d:  %d ChargeRate:  %d FloatVoltage:  %d HystApplied:%d Temp:%d",
            CurrentPack, MSEPackRTStatus[CurrentPack].CurrentJeitaZone, MSEPackRTStatus[CurrentPack].RuntimeChargeRate,
            MSEPackRTStatus[CurrentPack].RuntimeChargeVoltage, TempHyst, MSEBatteryStatus[CurrentPack].temperature);
    }

    if ((MSEPackRTStatus[FGAUGE_002].RuntimeState != EOC_STATE) &&
        (MSEPackRTStatus[FGAUGE_001].RuntimeChargeRate > MSEPackRTStatus[FGAUGE_002].RuntimeChargeRate))
    {
        MSEPackRTStatus[FGAUGE_001].RuntimeChargeRate = MSEPackRTStatus[FGAUGE_002].RuntimeChargeRate;
        pr_debug("Capping Pack1 Charge rate to ChargeRate:  %d to match Pack2", MSEPackRTStatus[FGAUGE_001].RuntimeChargeRate);
    }
}

void MSEHwUpdateRunTimeState(struct MSEHw_Config *MSEHw_Config, bool PacksNotbalanced)
{
    PMSE_PACK_CONFIG MSEPackConfig = NULL;
    PFG_BATT_STATUS_TYPE MSEBatteryStatus = NULL;
    PMSE_PACK_RT_STATUS MSEPackRTStatus = NULL;
    uint32_t NumPacks = 0;
    uint32_t CurrentPack = 0;

    // Cache Pack Config
    MSEPackConfig = (PMSE_PACK_CONFIG)&MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig;

    // Cache Pack Status
    MSEBatteryStatus = (PFG_BATT_STATUS_TYPE)&MSEHw_Config->MSEBatteryStatus;
    NumPacks = MSEHw_Config->MSEBatterySubsystemConfig.NumPacks;

    // Cache Runtime Pack Status
    MSEPackRTStatus = (PMSE_PACK_RT_STATUS)&MSEHw_Config->MSEPackRTStatus;

    for(CurrentPack = 0; CurrentPack < NumPacks; CurrentPack++)
    {
        if(PACK_NONE == MSEHw_Config->ChargingEnabledPack)
        {
            MSEPackRTStatus[CurrentPack].ChargingEnabled = false;
            MSEPackRTStatus[CurrentPack].RuntimeState = DISCHARGE_STATE;
            MSEPackRTStatus[CurrentPack].RuntimeChargeRate = 0;
            MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent = 0;
            MSEPackRTStatus[CurrentPack].CurrentJeitaZone = STANDARD_ZONE;
        }
        else if(MSEBatteryStatus[CurrentPack].monotonic_soc < MSEPackConfig[CurrentPack].SOCCheckPoint1)
        {
            MSEPackRTStatus[CurrentPack].ChargingEnabled = true;
            MSEPackRTStatus[CurrentPack].RuntimeState = CP1_STATE;
            MSEPackRTStatus[CurrentPack].RuntimeChargeRate = MSEPackConfig[CurrentPack].CP1ChargingRate;
        }
        else if(MSEBatteryStatus[CurrentPack].monotonic_soc < MSEPackConfig[CurrentPack].SOCCheckPoint2)
        {
            MSEPackRTStatus[CurrentPack].ChargingEnabled = true;
            MSEPackRTStatus[CurrentPack].RuntimeState = CP2_STATE;
            MSEPackRTStatus[CurrentPack].RuntimeChargeRate = MSEPackConfig[CurrentPack].CP2ChargingRate;
        }
        else if(MSEBatteryStatus[CurrentPack].monotonic_soc < MSEPackConfig[CurrentPack].SOCCheckPoint3)
        {
            MSEPackRTStatus[CurrentPack].ChargingEnabled = true;
            MSEPackRTStatus[CurrentPack].RuntimeState = CP3_STATE;
            MSEPackRTStatus[CurrentPack].RuntimeChargeRate = MSEPackConfig[CurrentPack].CP3ChargingRate;
        }
        else
        {
            MSEPackRTStatus[CurrentPack].ChargingEnabled = true;
            MSEPackRTStatus[CurrentPack].RuntimeState = EOC_STATE;
            MSEPackRTStatus[CurrentPack].RuntimeChargeRate = EOC_CHARGE_RATE;
            MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent = 0;
        }
    }

    if(MSEPackRTStatus[FGAUGE_001].RuntimeState == EOC_STATE && MSEPackRTStatus[FGAUGE_002].RuntimeState == EOC_STATE)
    {
        MSEPackRTStatus[FGAUGE_001].RuntimeChargeRate = EOC_BOTH_CHARGE_RATE;
        MSEPackRTStatus[FGAUGE_002].RuntimeChargeRate = EOC_BOTH_CHARGE_RATE;
    }

    if((PacksNotbalanced) && (PACK_NONE != MSEHw_Config->ChargingEnabledPack))
    {
        if(PACK_001 == MSEHw_Config->ChargingEnabledPack)
        {
            if (MSEPackRTStatus[FGAUGE_001].RuntimeState == CP1_STATE)
            {
                MSEPackRTStatus[FGAUGE_001].RuntimeChargeRate = MSEPackConfig[FGAUGE_001].CP2ChargingRate;
            }
            MSEPackRTStatus[FGAUGE_001].RuntimeState = CHARGE_BALANCING_STATE;
            MSEPackRTStatus[FGAUGE_002].RuntimeState = CHARGE_BALANCING_STATE;
        }
        else if (PACK_002 == MSEHw_Config->ChargingEnabledPack)
        {
            MSEPackRTStatus[FGAUGE_002].RuntimeState = CHARGE_BALANCING_STATE;
            if (MSEPackRTStatus[FGAUGE_001].ChargingEnabled)
            {
                MSEPackRTStatus[FGAUGE_001].ChargingEnabled = false;
                MSEPackRTStatus[FGAUGE_001].RuntimeState = CHARGE_BALANCING_STATE;
                MSEPackRTStatus[FGAUGE_001].RuntimeChargeRate = 0;
            }
        }
        else
        {
            MSEHw_Config->ChargingEnabledPack = PACK_BOTH;
        }
    }

    pr_debug("MSE Battery subsystem runtime Pack1 RuntimeState : %d Charge rate :%d Charging Enabled : %d",
        MSEPackRTStatus[FGAUGE_001].RuntimeState, MSEPackRTStatus[FGAUGE_001].RuntimeChargeRate, MSEPackRTStatus[FGAUGE_001].ChargingEnabled);
    pr_debug("MSE Battery subsystem runtime Pack2 RuntimeState : %d Charge rate :%d Charging Enabled : %d",
        MSEPackRTStatus[FGAUGE_002].RuntimeState, MSEPackRTStatus[FGAUGE_002].RuntimeChargeRate, MSEPackRTStatus[FGAUGE_002].ChargingEnabled);
}

#define NUM_FATAL_ERROR_CONDITION_RETRIES   0x3
int MSEHwConfigureCharging(struct MSEHw_Config *MSEHw_Config, uint32_t *pMSEChgMaxCurrent, uint32_t *pMSEChgFloatVoltage)
{
    int rc = 0;
    uint32_t SOC1, SOC2;
    uint32_t VBAT1, VBAT2;
    int32_t IBATT1, IBATT2;
    uint32_t Pack1ChargeCurrent, Pack2ChargeCurrent;
    bool PacksNotbalanced = false;
    PMSE_PACK_CONFIG MSEPackConfig = NULL;
    PMSE_PACK_RT_STATUS MSEPackRTStatus = NULL;
    static bool CachedIsDischarging = true;
    bool TriggerImbalancedRSOCReset = false;
    bool TriggerFGFaultReset = false;
    static bool CellDisconnectEngaged = false;
    static int TriggerImbalancedRSOCResetNumber = 0;
    static int TriggerFGFaultResetNumber = 0;

    // Cache VBAT1, and VBAT2
    VBAT1 = MSEHw_Config->MSEBatteryStatus[FGAUGE_001].vbatt;
    VBAT2 = MSEHw_Config->MSEBatteryStatus[FGAUGE_002].vbatt;

    // Cache SOC1, and SOC2
    SOC1 = MSEHw_Config->MSEBatteryStatus[FGAUGE_001].monotonic_soc;
    SOC2 = MSEHw_Config->MSEBatteryStatus[FGAUGE_002].monotonic_soc;

    // Cache IBATT1, and IBATT2
    IBATT1 = MSEHw_Config->MSEBatteryStatus[FGAUGE_001].ibatt;
    IBATT2 = MSEHw_Config->MSEBatteryStatus[FGAUGE_002].ibatt;

    // Cache MSEPackConfig
    MSEPackConfig = (PMSE_PACK_CONFIG)&MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig;

    // Cache Runtime Pack Status
    MSEPackRTStatus = (PMSE_PACK_RT_STATUS)&MSEHw_Config->MSEPackRTStatus;

    if(((SOC1 > SOC2) && ((SOC1 - SOC2) > MSEHw_Config->MSEBatterySubsystemConfig.SOCDelta)) ||
       ((SOC2 <= (SOC1 + MSEHw_Config->MSEBatterySubsystemConfig.SOCDeltaHYS)) &&
        (MSEPackRTStatus[FGAUGE_001].RuntimeState == CHARGE_BALANCING_STATE) &&
        (CachedIsDischarging == MSEHw_Config->InDischargePhase)) ||
       ((ABS_DIFF(VBAT1, VBAT2) > MSEHw_Config->MSEBatterySubsystemConfig.VDelta) &&
        ((SOC1 < STEEP_VOLTAGE_CURVE_SOC) || (SOC2 < STEEP_VOLTAGE_CURVE_SOC)) &&
        (MSEHw_Config->InDischargePhase)))
    {
        PacksNotbalanced = true;
    }

    // Check if packs are in discharge phase
    if((MSEHw_Config->InDischargePhase) && (!PacksNotbalanced))
    {
        MSEHw_Config->ChargingEnabledPack = PACK_NONE;
    }
    else if(PacksNotbalanced)
    {
        if(MSEHw_Config->InDischargePhase)
        {
            MSEHw_Config->ChargingEnabledPack = PACK_NONE;
        }
        else
        {
            MSEHw_Config->ChargingEnabledPack = ((SOC1 < SOC2) ? PACK_001 : PACK_002);
        }

        if(((SOC1 > STEEP_VOLTAGE_CURVE_SOC) && (SOC2 > STEEP_VOLTAGE_CURVE_SOC)) &&
           (SOC1 > (SOC2 + MSEHw_Config->MSEBatterySubsystemConfig.SOCDeltaMAX)) &&
           ((VBAT1 > VBAT2) && ABS_DIFF(VBAT1, VBAT2) > MSEHw_Config->MSEBatterySubsystemConfig.VDelta))
        {
            pr_err("ERROR: Packs found to be imbalanced beyond allowable range: VD:%d SD:%d", ABS_DIFF(VBAT1, VBAT2), ABS_DIFF(SOC1, SOC2));
            TriggerImbalancedRSOCReset = true;
        }
        else if (((SOC1 < STEEP_VOLTAGE_CURVE_SOC) || (SOC2 < STEEP_VOLTAGE_CURVE_SOC)) &&
            (((VBAT1 > VBAT2) && ABS_DIFF(VBAT1, VBAT2) > MSEHw_Config->MSEBatterySubsystemConfig.VDelta)))
        {
            pr_err("ERROR: Packs found to be imbalanced beyond allowable range (below STEEP_VOLTAGE_CURVE_SOC): VD:%d SD:%d", ABS_DIFF(VBAT1, VBAT2), ABS_DIFF(SOC1, SOC2));
            TriggerImbalancedRSOCReset = true;
        }
        else
        {
            pr_info("Packs found to be imbalanced within allowable range : %d- %d - %d -%d -%d", MSEHw_Config->ChargingEnabledPack, SOC1, SOC2, VBAT1 ,VBAT2);
        }
    }
    else
    {
        MSEHw_Config->ChargingEnabledPack = PACK_BOTH;
    }

    // Update MSE runtime states
    MSEHwUpdateRunTimeState(MSEHw_Config, PacksNotbalanced);

    // Apply JEITA corrections
    MSEHwApplyJeitaCorrections(MSEHw_Config, pMSEChgFloatVoltage);

    // Cache the IsDischarging flag for future
    CachedIsDischarging = MSEHw_Config->InDischargePhase;

    // Get current CFET status for both Packs
    rc = MSEHwGetCFETStatus(MSEHw_Config);
    if (rc < 0)
    {
        pr_err("MSEHwGetCFETStatus failed with rc: %d", rc);
        goto Exit;
    }

    // Get current Fault status for both Packs
    rc = MSEHwGetFaultStatus(MSEHw_Config);
    if (rc < 0)
    {
        pr_err("MSEHwGetFaultStatus failed with rc: %d", rc);
        goto Exit;
    }

    // Update Pack Telemetry
    BatteryTelemetryGeneratePayloadGeneral(MSEHw_Config);

    if((MSEPackRTStatus[FGAUGE_001].FaultStatus.active_fault) || (MSEPackRTStatus[FGAUGE_002].FaultStatus.active_fault))
    {
        pr_err("ERROR: FG detected a fault one one of the packs: [%d,%d] \n",
            MSEPackRTStatus[FGAUGE_001].FaultStatus.active_fault,
            MSEPackRTStatus[FGAUGE_002].FaultStatus.active_fault);
        TriggerFGFaultReset = true;
    }

    if(TriggerFGFaultReset)
    {
        TriggerFGFaultResetNumber++;
        if(NUM_FATAL_ERROR_CONDITION_RETRIES == TriggerFGFaultResetNumber)
        {
            pr_err("!!!!!!!!!!!! FATAL ERROR CONDITION: FG FAULT !!!!!!!!!!!!!");
            machine_restart("oem-fgfault");
        }
    }
    else
    {
        TriggerFGFaultResetNumber = 0;
    }


    if(TriggerImbalancedRSOCReset)
    {
        TriggerImbalancedRSOCResetNumber++;
        if(NUM_FATAL_ERROR_CONDITION_RETRIES == TriggerImbalancedRSOCResetNumber)
        {
            pr_err("!!!!!!!!!!!! FATAL ERROR CONDITION: RSOC IMBALANCE !!!!!!!!!!!!!");
            machine_restart("oem-rsocimbalance");
        }
    }
    else
    {
        TriggerImbalancedRSOCResetNumber = 0;
    }

    if(MSEPackRTStatus[FGAUGE_001].FaultStatus.ovp_detected)
    {
        rc = MSEHwPackDisconnect(MSEHw_Config, false, true);
        if (rc < 0)
        {
            pr_err("Error after updating CELL_DISC lines %d on Instance2 due to OVP detected on Pack1", rc);
            goto Exit;
        }
    }
    else
    {
        if (MSEPackRTStatus[FGAUGE_001].CFETOn != MSEPackRTStatus[FGAUGE_002].CFETOn)
        {
            if((MSEPackRTStatus[FGAUGE_001].CFETOn) && (0 <= IBATT1) && (PACK1_CD_ALLOWED <= SOC1))
            {
                rc = MSEHwPackDisconnect(MSEHw_Config, true, false);
                if (rc < 0)
                {
                    pr_err("Error after updating CELL_DISC lines %d on Instance1", rc);
                    goto Exit;
                }
                CellDisconnectEngaged = true;
            }
            else if((MSEPackRTStatus[FGAUGE_002].CFETOn) && (0 <= IBATT2) && (PACK2_CD_ALLOWED <= SOC2))
            {
                rc = MSEHwPackDisconnect(MSEHw_Config, false, true);
                if (rc < 0)
                {
                    pr_err("Error after updating CELL_DISC lines %d on Instance2", rc);
                    goto Exit;
                }
                CellDisconnectEngaged = true;
            }
            else if(CellDisconnectEngaged)
            {
                rc = MSEHwPackDisconnect(MSEHw_Config, false, false);
                if (rc < 0)
                {
                    pr_err("Error after updating CELL_DISC lines %d for both instances", rc);
                    goto Exit;
                }
                CellDisconnectEngaged = false;
            }
        }
        else
        {
            // Explicitly connect both packs
            rc = MSEHwPackDisconnect(MSEHw_Config, false, false);
            if (rc < 0)
            {
                pr_err("Error after updating CELL_DISC lines %d for both instances", rc);
                goto Exit;
            }

            if(CellDisconnectEngaged)
            {
                CellDisconnectEngaged = false;
            }
        }
    }

    if ((((!MSEPackRTStatus[FGAUGE_001].CFETOn) && (!MSEPackRTStatus[FGAUGE_002].CFETOn)) ||
         ((!MSEPackRTStatus[FGAUGE_001].CFETOn) && (SOC2 >= MSEHw_Config->MSEBatterySubsystemConfig.AutoRechargeThreshold) && (0 <= IBATT2)) ||
         ((!MSEPackRTStatus[FGAUGE_002].CFETOn) && (SOC1 >= MSEHw_Config->MSEBatterySubsystemConfig.AutoRechargeThreshold) && (0 <= IBATT1))) &&
        (!MSEHw_Config->InAutoRechargePhase))
    {
        MSEHw_Config->InAutoRechargePhase = true;
        BatteryTelemetryGeneratePayloadGeneral(MSEHw_Config);
    }
    else if((MSEPackRTStatus[FGAUGE_001].CFETOn) &&
            (MSEPackRTStatus[FGAUGE_002].CFETOn) &&
            (MSEHw_Config->CombinedRsoc <= MSEHw_Config->MSEBatterySubsystemConfig.AutoRechargeThreshold) &&
            (MSEHw_Config->InAutoRechargePhase))
    {
        MSEHw_Config->InAutoRechargePhase = false;
        BatteryTelemetryGeneratePayloadGeneral(MSEHw_Config);
    }


    // Cache individual charge currents per pack
    Pack1ChargeCurrent = (MSEPackConfig[FGAUGE_001].FCC * MSEPackRTStatus[FGAUGE_001].RuntimeChargeRate)/100;
    Pack2ChargeCurrent = (MSEPackConfig[FGAUGE_002].FCC * MSEPackRTStatus[FGAUGE_002].RuntimeChargeRate)/100;

    if ((PACK_NONE == MSEHw_Config->ChargingEnabledPack) ||
        ((MSEHw_Config->InAutoRechargePhase) && (MSEPackRTStatus[FGAUGE_001].CFETOn) && (MSEPackRTStatus[FGAUGE_002].CFETOn)))
    {
        *pMSEChgMaxCurrent = 0;
    }
    else if(PACK_001 == MSEHw_Config->ChargingEnabledPack)
    {
        *pMSEChgMaxCurrent = Pack1ChargeCurrent;
    }
    else if(PACK_002 == MSEHw_Config->ChargingEnabledPack)
    {
        *pMSEChgMaxCurrent = Pack2ChargeCurrent;
    }
    else //PACK_BOTH
    {
        *pMSEChgMaxCurrent = Pack1ChargeCurrent + Pack2ChargeCurrent;
    }

    if ((MSEHw_Config->IsWeakPSU) || (MSEHw_Config->IsThermalThrottlingActive))
    {
        rc = MSEHwConfigureCLR(MSEHw_Config, 0);
        if (rc < 0)
        {
            pr_err("MSEHwConfigureCLR returned error %d", rc);
            goto Exit;
        }
    }
    else if (MSEHw_Config->InDischargePhase)
    {
        rc = MSEHwConfigureCLR(MSEHw_Config, 0);
        if (rc < 0)
        {
            pr_err("MSEHwConfigureCLR returned error %d", rc);
            goto Exit;
        }
    }

Exit:
    return rc;
}

int MSEHwBalanceChargeCurrent(struct MSEHw_Config *MSEHw_Config, uint32_t *pMSEChgMaxCurrent)
{
    int rc = 0;
    int32_t ExpectedChargeCurrent[MAX_NUM_FGS] = {0};
    int32_t ChargeCurrentCorrection[MAX_NUM_FGS] = {0};
    uint32_t MinAllowedError[MAX_NUM_FGS] = {PACK1_MIN_ALLOWED_ERROR, PACK2_MIN_ALLOWED_ERROR};
    static enum PACK_INSTANCE PrevChargingEnabledPack = PACK_NONE;
    int32_t CurrentMSEFCC = 0;
    PMSE_PACK_CONFIG MSEPackConfig = NULL;
    PMSE_PACK_RT_STATUS MSEPackRTStatus = NULL;
    PFG_BATT_STATUS_TYPE MSEBatteryStatus = NULL;
    uint32_t CurrentPack = 0;
    uint32_t TotalRuntimeChargeCurrent = 0;

    if ((MSEHw_Config->IsWeakPSU) || (MSEHw_Config->InAutoRechargePhase) || (MSEHw_Config->IsThermalThrottlingActive))
    {
        pr_info("Skip charge current balancing when a weak psu is active or when in autorecharge or when active thermal throttling :%d", *pMSEChgMaxCurrent);
        goto Exit;
    }

    // Cache MSEPackConfig
    MSEPackConfig = (PMSE_PACK_CONFIG)&MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig;

    // Cache Runtime Pack Status
    MSEPackRTStatus = (PMSE_PACK_RT_STATUS)&MSEHw_Config->MSEPackRTStatus;

    // Cache Pack Status
    MSEBatteryStatus = (PFG_BATT_STATUS_TYPE)&MSEHw_Config->MSEBatteryStatus;

    // Compute Current MSE FCC.
    CurrentMSEFCC = ((((MSEPackRTStatus[FGAUGE_001].RuntimeChargeRate * MSEPackConfig[FGAUGE_001].FCC) +
                     (MSEPackRTStatus[FGAUGE_002].RuntimeChargeRate * MSEPackConfig[FGAUGE_002].FCC)) + 50)/100);

    for(CurrentPack = 0; CurrentPack < MAX_NUM_FGS; CurrentPack++)
    {
        if (MSEPackRTStatus[CurrentPack].ChargingEnabled)
        {
            ExpectedChargeCurrent[CurrentPack] = (MSEPackRTStatus[CurrentPack].RuntimeChargeRate * MSEPackConfig[CurrentPack].FCC)/100;
            if((MSEBatteryStatus[CurrentPack].ibatt < 0) &&
               (((MSEBatteryStatus[CurrentPack].ibatt + ExpectedChargeCurrent[CurrentPack]) < 0) ||
                ((MSEBatteryStatus[CurrentPack].ibatt + ExpectedChargeCurrent[CurrentPack]) > MinAllowedError[CurrentPack])))
            {
                ChargeCurrentCorrection[CurrentPack] = (MSEBatteryStatus[CurrentPack].ibatt + ExpectedChargeCurrent[CurrentPack]);
            }
        }
        else
        {
            ExpectedChargeCurrent[CurrentPack] = 0;
            ChargeCurrentCorrection[CurrentPack] = 0;
        }

        // When there was a change in packs charging state, reset the Corrections/RuntimeChargeCurrent
        if (PrevChargingEnabledPack != MSEHw_Config->ChargingEnabledPack)
        {
            ChargeCurrentCorrection[CurrentPack] = 0;
            MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent = ExpectedChargeCurrent[CurrentPack];
        }
        else if (MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent < ABS(ChargeCurrentCorrection[CurrentPack]))
        {
            ChargeCurrentCorrection[CurrentPack] = 0;
            MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent = ExpectedChargeCurrent[CurrentPack] + MinAllowedError[CurrentPack];
        }

        if (ChargeCurrentCorrection[CurrentPack] > 0)
        {
            ChargeCurrentCorrection[CurrentPack] = (ChargeCurrentCorrection[CurrentPack] >= MinAllowedError[CurrentPack]) ?
                                                   MinAllowedError[CurrentPack] : ChargeCurrentCorrection[CurrentPack];
            MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent += ChargeCurrentCorrection[CurrentPack];
        }
        else if (ChargeCurrentCorrection[CurrentPack] < 0)
        {
            MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent -= ABS(ChargeCurrentCorrection[CurrentPack]);
        }
    }

    // Cap pack1 RuntimeChargeCurrent to the MAX_ALLOWED_CL_CURRENT
    if(MSEPackRTStatus[FGAUGE_001].RuntimeChargeCurrent > MAX_ALLOWED_CL_CURRENT)
    {
        MSEPackRTStatus[FGAUGE_001].RuntimeChargeCurrent = MAX_ALLOWED_CL_CURRENT;
    }

    rc = MSEHwConfigureCLR(MSEHw_Config, MSEPackRTStatus[FGAUGE_001].RuntimeChargeCurrent);
    if (rc < 0)
    {
        pr_err("MSEHwConfigureCLR returned error rc = %d", rc);
    }

    for (CurrentPack = 0; CurrentPack < MAX_NUM_FGS; CurrentPack++)
    {
        if (MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent > ExpectedChargeCurrent[CurrentPack])
        {
            MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent = ExpectedChargeCurrent[CurrentPack];
        }
        TotalRuntimeChargeCurrent += MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent;
    }

    // Cap total FCC based on TotalRuntimeChargeCurrent
    if (TotalRuntimeChargeCurrent > 0)
    {
        *pMSEChgMaxCurrent = (*pMSEChgMaxCurrent > TotalRuntimeChargeCurrent) ?
            TotalRuntimeChargeCurrent : *pMSEChgMaxCurrent;
    }

    //Recompute FCC one final time to ensure its absolute upper bound
    *pMSEChgMaxCurrent = (*pMSEChgMaxCurrent > CurrentMSEFCC) ?
        CurrentMSEFCC : *pMSEChgMaxCurrent;

    PrevChargingEnabledPack = MSEHw_Config->ChargingEnabledPack;

    for(CurrentPack = 0; CurrentPack < MAX_NUM_FGS; CurrentPack++)
    {
        pr_debug("Pack:%d Charge current:%d, Expected Charge current:%d correction applied:%d Corrected Charge Current:%d ",
                (CurrentPack+1),
                ABS(MSEBatteryStatus[CurrentPack].ibatt),
                ExpectedChargeCurrent[CurrentPack],
                ChargeCurrentCorrection[CurrentPack],
                MSEPackRTStatus[CurrentPack].RuntimeChargeCurrent);
    }

Exit:
    return rc;
}

/********************************
 *    MSEChgHw DTSI Functions   *
 *******************************/

static int MSEHwGetGpioFromDtsi(struct device_node *node, struct MSEHw_Config *MSEHw_Config)
{
	int rc = 0;
	int i;

	// get the number of PMIC GPIOs defined in dts
	rc = of_property_read_u32(node, "ms,num-gpios",
					&MSEHw_Config->TotalGpios);
	if (rc < 0)
		MSEHw_Config->TotalGpios = MAX_NUM_GPIOS;

	MSEHw_Config->TotalCLGpios = MSEHw_Config->TotalGpios - MAX_NUM_FGS;

	for (i = 0; i < MSEHw_Config->TotalGpios; i++) {
		char *name;

		MSEHw_Config->Pmic_Gpios[i] = of_get_named_gpio(node, "ms,pmicgpios", i);
		if (MSEHw_Config->Pmic_Gpios[i] < 0) {
			pr_err("failed to get pmic gpio from dtsi at index: %d", i);
			rc = -ENODEV;
			goto Exit;
		}

		name = kasprintf(GFP_KERNEL, "GPIO%d", i);
		if (!name) {
			pr_err("failed to get construct a name for the pmic gpio at index: %d", i);
			rc = -ENOMEM;
			goto Exit;
		}

		rc = gpio_request(MSEHw_Config->Pmic_Gpios[i], name);
		if (rc < 0) {
			pr_err("failed to request a pmic gpio at index: %d", i);
			kfree(name);
			goto Exit;
		}

		rc = gpio_direction_output(MSEHw_Config->Pmic_Gpios[i], 0);    // set all GPIOs to LOW in init
		if (rc < 0) {
			gpio_free(MSEHw_Config->Pmic_Gpios[i]);
			pr_err("failed to set the output direction for pmic gpio at index: %d", i);
			kfree(name);
			goto Exit;
		}
	}

Exit:
	return rc;
}

// MSE Pack 1 and Pack 2 information from DTSI
static int MSEGetPackConfigDataFromDtsi(struct device_node *node, struct MSEHw_Config *MSEHw_Config)
{
    int rc;
    uint32_t soccheckpoint1, soccheckpoint2, soccheckpoint3;

    // get MSE pack 1 config data
    rc = of_property_read_u32(node, "ms,pack1-fcc",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].FCC);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-cp1chargerate",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].CP1ChargingRate);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-cp2chargerate",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].CP2ChargingRate);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-cp3chargerate",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].CP3ChargingRate);
    if (rc < 0)
        goto Exit;

    //get MSE pack 2 config data
    rc = of_property_read_u32(node, "ms,pack2-fcc",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].FCC);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-cp1chargerate",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].CP1ChargingRate);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-cp2chargerate",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].CP2ChargingRate);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-cp3chargerate",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].CP3ChargingRate);
    if (rc < 0)
        goto Exit;

    // pack 1 and pack 2 have the same soc check points right now
    rc = of_property_read_u32(node, "ms,soccheckpoint1",
                              &soccheckpoint1);
    if (rc < 0)
        goto Exit;
    MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].SOCCheckPoint1 = soccheckpoint1;
    MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].SOCCheckPoint1 = soccheckpoint1;

    rc = of_property_read_u32(node, "ms,soccheckpoint2",
                              &soccheckpoint2);
    if (rc < 0)
        goto Exit;
    MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].SOCCheckPoint2 = soccheckpoint2;
    MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].SOCCheckPoint2 = soccheckpoint2;

    rc = of_property_read_u32(node, "ms,soccheckpoint3",
                              &soccheckpoint3);
    if (rc < 0)
        goto Exit;
    MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].SOCCheckPoint3 = soccheckpoint3;
    MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].SOCCheckPoint3 = soccheckpoint3;

    // get pack1 JEITA properties
    rc = of_property_read_u32(node, "ms,pack1-extremecoldlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].ExtremeColdLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-hardcoldlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].HardColdLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-softcoldlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].SoftColdLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-softhotlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].SoftHotLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-hardhotlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].HardHotLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-extremehotlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].ExtremeHotLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-temphyst",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].TempHyst);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-chargerateHCL",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].CCCompHCL);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-chargerateSCLandcp1",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].CCCompSCLandCP1);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-chargerateSCLandcp2",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].CCCompSCLandCP2);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-chargerateSCLandcp3",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].CCCompSCLandCP3);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-chargerateSHLandcp1",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].CCCompSHLandCP1);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack1-chargerateHHL",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_001].CCCompHHL);
    if (rc < 0)
        goto Exit;

    // get pack2 JEITA properties
    rc = of_property_read_u32(node, "ms,pack2-extremecoldlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].ExtremeColdLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-hardcoldlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].HardColdLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-softcoldlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].SoftColdLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-softhotlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].SoftHotLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-hardhotlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].HardHotLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-extremehotlimit",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].ExtremeHotLimit);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-temphyst",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].TempHyst);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-chargerateHCL",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].CCCompHCL);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-chargerateSCLandcp1",
        &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].CCCompSCLandCP1);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-chargerateSCLandcp2",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].CCCompSCLandCP2);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-chargerateSCLandcp3",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].CCCompSCLandCP3);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-chargerateSHLandcp1",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].CCCompSHLandCP1);
    if (rc < 0)
        goto Exit;

    rc = of_property_read_u32(node, "ms,pack2-chargerateHHL",
                              &MSEHw_Config->MSEBatterySubsystemConfig.MSEPackConfig[FGAUGE_002].CCCompHHL);
    if (rc < 0)
        goto Exit;

Exit:
    return rc;
}

static int MSEHwGetBatterySubsystemConfigFromDtsi(struct device_node *node, struct MSEHw_Config *MSEHw_Config)
{
	int rc = 0;

	// get the MSE Battery Subsystem Config information stored in dtsi
	rc = of_property_read_u32(node, "ms,vdelta",
					&MSEHw_Config->MSEBatterySubsystemConfig.VDelta);
	if (rc < 0)
		goto Exit;

	rc = of_property_read_u32(node, "ms,socdelta",
					&MSEHw_Config->MSEBatterySubsystemConfig.SOCDelta);
	if (rc < 0)
		goto Exit;

	rc = of_property_read_u32(node, "ms,socdeltamax",
					&MSEHw_Config->MSEBatterySubsystemConfig.SOCDeltaMAX);
	if (rc < 0)
		goto Exit;

	rc = of_property_read_u32(node, "ms,socdeltahys",
					&MSEHw_Config->MSEBatterySubsystemConfig.SOCDeltaHYS);
	if (rc < 0)
		goto Exit;

	rc = of_property_read_u32(node, "ms,numpacks",
					&MSEHw_Config->MSEBatterySubsystemConfig.NumPacks);
	if (rc < 0)
		goto Exit;

	rc = of_property_read_u32(node, "ms,floatvoltageHHL",
					&MSEHw_Config->MSEBatterySubsystemConfig.FloatVoltageHHL);
	if (rc < 0)
		goto Exit;

	rc = of_property_read_u32(node, "qcom,fv-max-uv",
					&MSEHw_Config->MSEBatterySubsystemConfig.FloatVoltageMax);
	if (rc < 0)
		goto Exit;

	rc = of_property_read_u32(node, "ms,auto-recharge-soc",
					&MSEHw_Config->MSEBatterySubsystemConfig.AutoRechargeThreshold);
	if (rc < 0)
		goto Exit;

	// get pack1 and pack2 configuration data from DTSI
	rc = MSEGetPackConfigDataFromDtsi(node, MSEHw_Config);
	if(rc < 0) {
		pr_err("MSE_GetPackConfigData failed");
		goto Exit;
	}

Exit:
	return rc;
}

static int MSEHwGetExtFgHandle(struct device_node *node, struct MSEHw_Config *MSEHw_Config)
{
	int i = 0;
	int rc = 0;
    const char *fuel_gauge_names[MAX_NUM_FGS];

    rc = of_property_read_string_array(node, "ms,fuel-gauges", fuel_gauge_names, MAX_NUM_FGS);
    if(rc < 0)
    {
        pr_err("failed to get list of extfgs from dtsi");
        goto Exit;
    }

	for (i = 0; i < MAX_NUM_FGS; i++)
	{
        MSEHw_Config->MSEExtFg[i] = power_supply_get_by_name(fuel_gauge_names[i]);
		if(!MSEHw_Config->MSEExtFg[i])
		{
			pr_err("failed to register MSEExtFg[%d] rc = %ld\n", i, PTR_ERR(MSEHw_Config->MSEExtFg[i]));
			rc = -EINVAL;
			goto Exit;
		}
	}

	rc = MSEHwExtFgGetBattInfo(MSEHw_Config);  // get and store battery status from extfgs
	if (rc < 0)
	{
		pr_err("failed to get pack information from the ExtFgs rc = %d", rc);
		goto Exit;
	}

Exit:
    return rc;
}

int static MSEHwGetHeartbeatTimersFromDtsi(struct device_node *node, struct MSEHw_Config *MSEHw_Config)
{
    int rc = 0;

	// get the PSU attached heartbeat timer stored in dtsi
	rc = of_property_read_u32(node, "ms,psu-heartbeat-timer",
					&MSEHw_Config->PsuHeartbeatTimer);
	if (rc < 0)
		goto Exit;

    // get the DC/Battery heartbeat timer stored in dtsi
    rc = of_property_read_u32(node, "ms,batt-heartbeat-timer",
					&MSEHw_Config->BattHeartbeatTimer);
	if (rc < 0)
		goto Exit;

Exit:
    return rc;
}

/*****************************************
 *    MSEChgHw Work and Init Functions   *
 ****************************************/

int MSEHwGetBatteryCapacity(struct MSEHw_Config *MSEHw_Config, int rawRsoc)
{
    int reportedRsoc = rawRsoc;

    if(MSEHw_Config->IsDebugBattery)
        return reportedRsoc;

    MSEHw_Config->CombinedRsoc = rawRsoc;

    if(MSEHw_Config->BatteryProtection.BatteryLevel >= 0)
        reportedRsoc = MSEHw_Config->BatteryProtection.BatteryLevel;

    return reportedRsoc;
}

static void MSEHwWork(struct work_struct *work)
{
    struct smb_charger *chg = container_of(work, struct smb_charger, MSEChg_Hw_work.work);
    int rc = 0;
    uint32_t MSEMaxBatteryCurrent;
    uint32_t MSEFloatVoltage;
    uint32_t reschedule_ms = 0;
    bool IsForcedDischarging = false;
    struct MSEHw_Config *MSEHw_Config = &chg->MSEHw_Config;
    union power_supply_propval pval = {0, };

    if(MSEHw_Config->IsDebugBattery)  // if debug battery detected do not call any of the MSE Chg functions
        goto Exit;

    // Check if PSU is conneted or not
    rc = power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &pval);
    if (rc < 0)
    {
        pr_err("Couldn't get usb psy online property rc=%d\n", rc);
        return;
    }

    if(pval.intval)
    {
        MSEHw_Config->InDischargePhase = false;
        MSEHw_Config->IsWeakPSU = MSEHwGetWeakPSUStatus(chg);
        reschedule_ms = MSEHw_Config->PsuHeartbeatTimer;
    }
    else
    {
        MSEHw_Config->InDischargePhase = true;
        MSEHw_Config->IsWeakPSU = false;
        if(MSEHw_Config->BatteryProtection.BatteryLevel >= 0)
            reschedule_ms = MSEHw_Config->BattHeartbeatTimer;
        else
            reschedule_ms = MSEHw_Config->PsuHeartbeatTimer;
    }

    // Check if thermal throttling is active
    if (chg->system_temp_level)
    {
        MSEHw_Config->IsThermalThrottlingActive = true;
        pr_info("Skin temp based thermal throttling is active. FCC=%d\n", get_effective_result(chg->fcc_votable));
    }
    else
    {
        MSEHw_Config->IsThermalThrottlingActive = false;
    }

    rc = MSEHwExtFgGetBattInfo(MSEHw_Config);
    if (rc < 0)
    {
        pr_err("MSEHwExtFgGetBattInfo failed with rc = %d", rc);
        goto Exit;
    }

    rc = MSEHwConfigureCharging(MSEHw_Config, &MSEMaxBatteryCurrent, &MSEFloatVoltage);
    if(rc < 0)
    {
        pr_err("MSEHwConfigureCharging failed with rc: %d", rc);
        goto Exit;
    }

    if(!MSEHw_Config->InDischargePhase)
    {
        rc = MSEHwBalanceChargeCurrent(MSEHw_Config, &MSEMaxBatteryCurrent);
        if(rc < 0)
        {
            pr_err("MSEHwBalanceChargeCurrent failed with rc: %d", rc);
            goto Exit;
        }

        // Adjust the pmic votes for charging phase/JEITA/Charge balancing etc.
        if ((MSEFloatVoltage == 0) ||
            (MSEMaxBatteryCurrent == 0))
        {
            vote(chg->chg_disable_votable, MSE_VOTER, true, 0);  // disable charging when max fv/fcc computed is 0 which during end of charge, extreme JEITA conditions
        }
        else
        {
            vote(chg->chg_disable_votable, MSE_VOTER, false, 0);
        }

        vote(chg->fv_votable, MSE_VOTER, true, MSEFloatVoltage);
        vote(chg->fcc_votable, MSE_VOTER, true, MSEMaxBatteryCurrent*1000);

        if(MSEHw_Config->CombinedRsoc >= RSOC_DISABLE_SMB)
        {
            vote(chg->pl_disable_votable, MSE_VOTER, true, 0); // disable SMB during CV mode due to OVP
        }
        else
        {
            vote(chg->pl_disable_votable, MSE_VOTER, false, 0);
        }
    }

    // MSE battery protection module
    rc = MSEBatteryProtectionPeriodic(&MSEHw_Config->BatteryProtection, MSEHw_Config->InDischargePhase, &IsForcedDischarging);
    if (rc < 0)
    {
        pr_err("MSEBatteryProtectionPeriodic failed with rc: %d", rc);
        goto Exit;
    }

    // Adjust the pmic votes for various battery protection modes
    vote(chg->fv_votable, MSE_BATTERYPROTECTION_VOTER, true,
        MSEHw_Config->MSEBatterySubsystemConfig.FloatVoltageMax - (MSEHw_Config->BatteryProtection.BatteryProtectionBPCConfig[FGAUGE_001].bpcProtection*1000));
    if(!MSEHw_Config->InDischargePhase)
    {
        vote(chg->usb_icl_votable, MSE_BATTERYPROTECTION_VOTER, IsForcedDischarging, 0);
    }
    else
    {
        vote(chg->usb_icl_votable, MSE_BATTERYPROTECTION_VOTER, false, 0);
    }

    vote(chg->chg_disable_votable, MSE_BATTERYPROTECTION_VOTER, MSEHw_Config->BatteryProtection.IsDisableCharging, 0);
    vote(chg->fcc_votable, MSE_BATTERYPROTECTION_VOTER, MSEHw_Config->BatteryProtection.IsDisableCharging, 0);

Exit:
    pr_info("MSEFloatVoltage: %d  MSEMaxBatteryCurrent: %d  IsForcedDischarging: %d  ChargingEnabled: %d  ICL: %d",
            MSEFloatVoltage,
            MSEMaxBatteryCurrent,
            IsForcedDischarging,
            !get_effective_result_locked(chg->chg_disable_votable),
            get_effective_result_locked(chg->usb_icl_votable));

    if(!MSEHw_Config->IsDebugBattery)  // reschedule MSEHwWork when we have real batteries
        schedule_delayed_work(&chg->MSEChg_Hw_work, msecs_to_jiffies(reschedule_ms));
}

#define DISABLE_CL_OVERRIDE_VALUE (MAX_ALLOWED_CL_CURRENT + 50)
int MSEHwInit(void *chg_input)
{
    int rc;
    union power_supply_propval val;
    struct device_node *node;
    struct smb_charger *chg;
    struct MSEHw_Config *MSEHw_Config;

    if(!chg_input)
    {
        pr_err("smb_charger input missing");
        goto Exit;
    }

    chg = chg_input;
    MSEHw_Config = &chg->MSEHw_Config;
    node = chg->dev->of_node;

    INIT_DELAYED_WORK(&chg->MSEChg_Hw_work, MSEHwWork);

    if (!node)
    {
        pr_err("device tree node missing\n");
        goto Exit;
    }

    // populate Pmic_Gpios from DTSI
    rc = MSEHwGetGpioFromDtsi(node, MSEHw_Config);
    if (rc < 0)
    {
        pr_err("GPIO parsing from DTSI failed while getting info from dtsi with rc  = %d", rc);
        goto Exit;
    }

    // populate MSE_BATTERY_CONFIG MSEBatterySubsystemConfig from DTSI
    rc = MSEHwGetBatterySubsystemConfigFromDtsi(node, MSEHw_Config);
    if (rc < 0)
    {
        pr_err("MSEHw_GetBatterySubsystemConfig failed while parsing info from dtsi with rc = %d", rc);
        goto Exit;
    }

    // get the heartbeat timers for AC and DC/Batt from DTSI
    rc = MSEHwGetHeartbeatTimersFromDtsi(node, MSEHw_Config);
    if (rc < 0)
    {
        pr_err("MSEHw_GetHeartbeatTimers failed while parsing info from dtsi with rc = %d", rc);
        goto Exit;
    }

    MSEHw_Config->OverrideCLCfgCurrent = DISABLE_CL_OVERRIDE_VALUE;   // initially disabling the debug fs CL override

    // Update current limiter configuration to most conservative setting
    rc = MSEHwConfigureCLR(MSEHw_Config, 0);
    if (rc < 0)
    {
        pr_err("MSEHwConfigureCLR failed with rc = %d", rc);
        goto Exit;
    }

    rc = MSEHwPackDisconnect(MSEHw_Config, false, false);
    if (rc < 0)
    {
        pr_err("MSEHwPackDisconnect failed with rc = %d", rc);
        goto Exit;
    }

    rc = MSEHwGetHwState(MSEHw_Config);
    if (rc < 0)
    {
        pr_err("MSEHwGetHwState failed with rc = %d", rc);
        // Intentionally suppressing the error, since its not fatal and can fail when running with NO translation board.
        rc  = 0;
    }

    rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_DEBUG_BATTERY, &val);  // check for debug battery
    if (rc < 0)
    {
        pr_err("Couldn't get debug battery prop rc=%d, assuming debug battery is present \n", rc);
        MSEHw_Config->IsDebugBattery = true;
    }
    MSEHw_Config->IsDebugBattery = val.intval;

    if (MSEHw_Config->IsDebugBattery)
    {
        // don't query extfgs when debug battery is present
        pr_err("debug battery detected, the MSE Chg subsystem won't be invoked");
        rc = 0;
        goto Exit;
    }

    // Get a handle to the extfgs and information about the packs and store them
    rc = MSEHwGetExtFgHandle(node, MSEHw_Config);
    if (rc < 0)
    {
        pr_err("MSEHw_GetExtFgInfo failed with rc = %d", rc);
        goto Exit;
    }

    //  Enabling MSE battery protection
    rc = MSEBatteryProtectionInit(node, &MSEHw_Config->BatteryProtection);
    if (rc < 0)
    {
        pr_err("MSEBatteryProtectionInit failed rc=%d \n", rc);
        goto Exit;
    }

    schedule_delayed_work(&chg->MSEChg_Hw_work, msecs_to_jiffies(MSEHw_Config->PsuHeartbeatTimer));

Exit:
	return rc;
}

void MSEHwShutdown(void *chg_input)
{
    int i = 0;
    int rc = 0;
    struct smb_charger *chg;
    struct MSEHw_Config *MSEHw_Config;

    chg = chg_input;
    MSEHw_Config = &chg->MSEHw_Config;

    cancel_delayed_work_sync(&chg->MSEChg_Hw_work);

    rc = MSEHwConfigureCLR(MSEHw_Config, 0);
    if (rc < 0)
    {
        pr_err("MSE_ConfigureCLR failed with rc = %d in the MSEHwShutdown", rc);
    }

    MSEBatteryProtectionShutdown(&MSEHw_Config->BatteryProtection);

    for (i = 0; i < MSEHw_Config->TotalGpios; i++)
    {
        gpio_free(MSEHw_Config->Pmic_Gpios[i]);
    }
}

/***********************************
 *    MSEChgHw DebugFs Functions   *
 **********************************/

static int force_clcfg_read(void *data, u64 *val)
{
    struct MSEHw_Config *MSEHw_Config = data;
    *val = MSEHw_Config->OverrideCLCfgCurrent;
    return 0;
}

static int force_clcfg_write(void *data, u64 val)
{
	struct MSEHw_Config *MSEHw_Config = data;
    MSEHw_Config->OverrideCLCfgCurrent = val;
    return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_clcfg_ops, force_clcfg_read,
			force_clcfg_write, "%d\n");

void MSEHwDebugfs(struct dentry *parent, void *data)
{
    struct dentry *file;
    struct MSEHw_Config *MSEHw_Config = data;

    file = debugfs_create_file("force_clcfg", 0777,
                               parent, data, &force_clcfg_ops);
    if (IS_ERR_OR_NULL(file))
        pr_err("Couldn't create force_clcfg file rc=%ld\n",
               (long)file);

    MSEBatteryProtectionDebugfs(parent, &MSEHw_Config->BatteryProtection);
}
