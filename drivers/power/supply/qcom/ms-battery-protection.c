/*
 * ms-battery-protection.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/soc/surface/surface_utils.h>
#include <soc/qcom/socinfo.h>
#include "ms-battery-protection.h"
#include "ms-chg.h"

struct BATTERY_TELEMETRY BatteryTelemetry = {{0}};  // main struct for all battery telemetry
struct MANUF_MODE_CONFIG ManufModeConfig = {0};

struct Hvt_AccelFactor_Table HvtAccelFactorTable[ACCEL_FACTOR_MAX] =
    {{ 250,ACCEL_FACTOR_25C },
     { 300,ACCEL_FACTOR_30C },
     { 350,ACCEL_FACTOR_35C },
     { 400,ACCEL_FACTOR_40C },
     { 450,ACCEL_FACTOR_45C },
     { 500,ACCEL_FACTOR_50C },
     { 550,ACCEL_FACTOR_55C },
     { MAX_ALLOWED_TEMPERATURE,ACCEL_FACTOR_ABOVE55C }};

/*****************************************
 *    MSE Battery Protection Functions   *
 ****************************************/

static uint16_t GetHvtAccelFactor(int16_t temperature_dC)
{
    uint8_t currentIndex;
    uint16_t hvtAccelFactor;

    // Iterate through hvtAccelFactorTable to get the HVT AccelerationFactor
    for (currentIndex = 0; currentIndex < ACCEL_FACTOR_MAX; currentIndex++)
    {
        if (temperature_dC <= HvtAccelFactorTable[currentIndex].HvtTemperatureUth)
        {
            hvtAccelFactor =  HvtAccelFactorTable[currentIndex].AccelFactor;
            break;
        }
    }
    return hvtAccelFactor;
}

static int GetManufacturerBlockAFromExtFg(struct BATTERY_PROTECTION *BatteryProtection)
{
    PSTG_BPC_NVM BatteryProtectionBPCConfig = NULL;
    union power_supply_propval prop_pack = {0, };
    uint32_t CurrentPack = 0;
    uint32_t NumPacks = 0;
    int rc = 0;
    int i = 0;
    char ManufName[MFG_NAME_SIZE + 1];

    // Cache num packs
    NumPacks = BatteryProtection->NumPacks;

    // Cache BPC config
    BatteryProtectionBPCConfig = (PSTG_BPC_NVM)&BatteryProtection->BatteryProtectionBPCConfig;

    for(CurrentPack = 0; CurrentPack < NumPacks; CurrentPack++)
    {
        if (power_supply_get_property(BatteryProtection->MSEExtFg[CurrentPack], POWER_SUPPLY_PROP_MANUFACTURER, &prop_pack))
        {
            pr_err("Could not read pack[%d] manufacturer name \n", CurrentPack);
            rc |= -ENODATA;
        }
        else
        {
            for(i = 0; i < MFG_NAME_SIZE; i++)
                ManufName[i] = prop_pack.strval[i];
            ManufName[MFG_NAME_SIZE] = '\0';

            if(strcmp(ManufName, SUPPLIER_NAME_ONE) == 0)
                BatteryProtectionBPCConfig[CurrentPack].BattSupplierType = BAT_SUPPLIER_1;
            else if(strcmp(ManufName, SUPPLIER_NAME_TWO) == 0)
                BatteryProtectionBPCConfig[CurrentPack].BattSupplierType = BAT_SUPPLIER_2;
            else
                BatteryProtectionBPCConfig[CurrentPack].BattSupplierType = BAT_SUPPLIER_1; // setting supplier one by default
        }
    }
    return rc;
}

static int GetHvtCountFromExtFg(struct BATTERY_PROTECTION *BatteryProtection)
{
    PSTG_BPC_NVM BatteryProtectionBPCConfig = NULL;
    union power_supply_propval prop_pack = {0, };
    uint32_t CurrentPack = 0;
    uint32_t NumPacks = 0;
    int rc = 0;

    // Cache num packs
    NumPacks = BatteryProtection->NumPacks;

    // Cache BPC config
    BatteryProtectionBPCConfig = (PSTG_BPC_NVM)&BatteryProtection->BatteryProtectionBPCConfig;

    for(CurrentPack = 0; CurrentPack < NumPacks; CurrentPack++)
    {
        // query hvtcount and store in STG_BPC_NVM eqvTimeAtHvt_sec
        if (power_supply_get_property(BatteryProtection->MSEExtFg[CurrentPack], POWER_SUPPLY_PROP_CHARGE_COUNTER, &prop_pack))
        {
            pr_err("Could not read pack[%d] hvtcount\n", CurrentPack);
            rc |= -ENODATA;
        }
        else
        {
            BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvt_sec = (uint32_t)prop_pack.intval;
        }
        pr_info("Reading HvtCount from ExtFG Pack[%d]: BatteryProtectionBPCConfig.eqvTimeAtHvt_sec: 0x%x \n", CurrentPack,
                BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvt_sec);
    }
    return rc;
}

static int FlushHvtCountToExtFg(struct BATTERY_PROTECTION *BatteryProtection, uint32_t CurrentPack)
{
    PSTG_BPC_NVM BatteryProtectionBPCConfig = NULL;
    union power_supply_propval val;
    int rc = 0;

    // Cache BPC config
    BatteryProtectionBPCConfig = (PSTG_BPC_NVM)&BatteryProtection->BatteryProtectionBPCConfig;

    val.intval = BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvt_sec;
    rc = power_supply_set_property(BatteryProtection->MSEExtFg[CurrentPack], POWER_SUPPLY_PROP_CHARGE_COUNTER, &val);
    if (rc)
    {
        pr_err("Could not set pack[%d] hvtcount\n", CurrentPack);
        rc = -ENODATA;
    }
    return rc;
}

static void ComputeElapsedTime(struct BATTERY_PROTECTION *BatteryProtection, uint32_t uptime)
{
    static uint32_t lastSysTime_sec[MAX_NUM_FG_PACKS] = {0, 0};
    uint32_t elapsedTime_sec = 0;
    PBATTERY_PROTECTION_ELAPSED_TIME BatteryProtectionElapsedTime = NULL;
    uint32_t CurrentPack = 0;
    uint32_t NumPacks = 0;

    // Cache battery elapsed time
    BatteryProtectionElapsedTime = (PBATTERY_PROTECTION_ELAPSED_TIME)&BatteryProtection->BatteryProtectionElapsedTime;

    // Cache num packs
    NumPacks = BatteryProtection->NumPacks;

    for(CurrentPack = 0; CurrentPack < NumPacks; CurrentPack++)
    {
         // calculate elapsed time since last time battery task ran
        elapsedTime_sec = ((lastSysTime_sec[CurrentPack] && (uptime > lastSysTime_sec[CurrentPack])) ?
                        (uptime - lastSysTime_sec[CurrentPack]) : 0u);
        lastSysTime_sec[CurrentPack] = uptime;

        // update BatteryProtectionElapsedTime with current values
        BatteryProtectionElapsedTime[CurrentPack].elapsedTime_sec = elapsedTime_sec;
        BatteryProtectionElapsedTime[CurrentPack].hvtAF = GetHvtAccelFactor(BatteryProtection->MSEBatteryStatus[CurrentPack].temperature);
        BatteryProtectionElapsedTime[CurrentPack].hvtElapsedTime_sec = ((BatteryProtectionElapsedTime[CurrentPack].elapsedTime_sec *
                                                                BatteryProtectionElapsedTime[CurrentPack].hvtAF) /
                                                                HVT_AF_SCALING_FACTOR);
        BatteryProtectionElapsedTime[CurrentPack].partialHvtElapsedTime_sec += ((BatteryProtectionElapsedTime[CurrentPack].elapsedTime_sec *
                                                                BatteryProtectionElapsedTime[CurrentPack].hvtAF) %
                                                                HVT_AF_SCALING_FACTOR);

        //Update hvtElapsedTime and partialHvtElapsedTime as needed
        if (BatteryProtectionElapsedTime[CurrentPack].partialHvtElapsedTime_sec >= HVT_AF_SCALING_FACTOR)
        {
            BatteryProtectionElapsedTime[CurrentPack].hvtElapsedTime_sec += (BatteryProtectionElapsedTime[CurrentPack].partialHvtElapsedTime_sec /
                                                                            HVT_AF_SCALING_FACTOR);
            BatteryProtectionElapsedTime[CurrentPack].partialHvtElapsedTime_sec = (BatteryProtectionElapsedTime[CurrentPack].partialHvtElapsedTime_sec %
                                                                            HVT_AF_SCALING_FACTOR);
        }
        pr_debug("Pack[%d]  elapsedTime_sec is %d  hvtAF is %d  hvtElapsedTime_sec is %d  partialHvtElapsedTime_sec is %d", CurrentPack,
                BatteryProtectionElapsedTime[CurrentPack].elapsedTime_sec,
                BatteryProtectionElapsedTime[CurrentPack].hvtAF,
                BatteryProtectionElapsedTime[CurrentPack].hvtElapsedTime_sec,
                BatteryProtectionElapsedTime[CurrentPack].partialHvtElapsedTime_sec);
        elapsedTime_sec = 0;  // reset elapsedTime for next loop
    }
}

static void BpcUpdateMinSocVector(uint8_t *pVector, uint8_t size, uint8_t *pIndex, uint16_t interval, uint8_t soc)
{
    uint8_t i;
    if(!pIndex || !pVector) // check for NULL
        return;

    if (interval > size)
    {
        interval = size;
    }
    i = *pIndex;
    while (interval--)
    {
        pVector[i++] = soc;
        if (i >= size)
        {
            i = 0u;
        }
    }
    *pIndex = i;
}

static uint8_t BpcMinimumByte(uint8_t *pVector, uint32_t size)
{
    uint8_t result = 0xFFu;
    while (size-- > 0u)
    {
        if (*pVector < result)
        {
            result = *pVector;
        }
        pVector++;
    }
    return result;
}

static void BatteryTelemetrySecondsToHoursVector(uint16_t *pDst, uint32_t *pSrc, uint32_t size)
{
    while (size--)
    {
        *pDst++ = (*pSrc++ + 3599u) / 3600u;
    }
}

static void BatteryTelemetryGeneratePayloadBPC(struct BATTERY_PROTECTION *BatteryProtection, uint32_t CurrentPack)
{
    PBPC_TELEMETRY BpcTelemetry = NULL;
    PSTG_BPC_NVM BatteryProtectionBPCConfig = NULL;

    // Cache BPC telemetry
    BpcTelemetry = (PBPC_TELEMETRY)&BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BpcTelemetry;
    // Cache BPC config
    BatteryProtectionBPCConfig = (PSTG_BPC_NVM)&BatteryProtection->BatteryProtectionBPCConfig;

    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);

    // BPC TELEMETRY
    BpcTelemetry->BPCProtection = BatteryTelemetry.BatteryProtectionBinary.BscDirectives.BpcProtection;
    BpcTelemetry->BPCEqvTimeat35C = CONVERT_SEC_TO_MIN(BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvt_sec);
    BpcTelemetry->BPCEqvTimeat35CinBSC = CONVERT_SEC_TO_MIN(BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvtInBsc_sec);
    BpcTelemetry->BPCMinSOC28day = BpcMinimumByte(BatteryProtectionBPCConfig[CurrentPack].minSOC28day, sizeof(BatteryProtectionBPCConfig[CurrentPack].minSOC28day));
    BpcTelemetry->BPCMinSOC7day = BpcMinimumByte(BatteryProtectionBPCConfig[CurrentPack].minSOC7day, sizeof(BatteryProtectionBPCConfig[CurrentPack].minSOC7day));
    BpcTelemetry->BPCMinSOC3day = BpcMinimumByte(BatteryProtectionBPCConfig[CurrentPack].minSOC3day, sizeof(BatteryProtectionBPCConfig[CurrentPack].minSOC3day));
    if(BatteryProtectionBPCConfig[CurrentPack].minSOC1day < U8_MAX)  // do not log if it's the start of a new day until we get a legit value
        BpcTelemetry->BPCMinSOC1day = BatteryProtectionBPCConfig[CurrentPack].minSOC1day;

    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);
}

static void BatteryTelemetryGeneratePayloadBSC(struct BATTERY_PROTECTION *BatteryProtection, uint32_t CurrentPack)
{
    struct timespec ts;

    PBSC_TELEMETRY BscTelemetry = NULL;
    PBSC_CONFIG BatteryProtectionBSCConfig = NULL;

    // Cache BSC telemetry
    BscTelemetry = (PBSC_TELEMETRY)&BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BscTelemetry;
    // Cache BSC config
    BatteryProtectionBSCConfig = (PBSC_CONFIG)&BatteryProtection->BatteryProtectionBSCConfig;

    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);

    // BSC TELEMETRY
    BscTelemetry->BSCStartDateInSeconds = BatteryProtectionBSCConfig[CurrentPack].bscStartDate;
    getnstimeofday(&ts);
    if (ts.tv_sec > BatteryProtectionBSCConfig[CurrentPack].bscStartDate)
        BscTelemetry->BSCTotalMinutesFromStart = CONVERT_SEC_TO_MIN(ts.tv_sec - BatteryProtectionBSCConfig[CurrentPack].bscStartDate + 59u);
    BscTelemetry->BSCTotalEngagedMinutes = CONVERT_SEC_TO_MIN(BatteryProtectionBSCConfig[CurrentPack].bscTotalActive_sec + 59U);
    BscTelemetry->BSCTotalEntryEvents = BatteryTelemetry.BatteryProtectionBinary.BscDirectives.BSCTotalEntryEvents;
    BscTelemetry->BSCCurrentlyEngaged = BatteryTelemetry.BatteryProtectionBinary.BscDirectives.InActiveBsc;
    BscTelemetry->BSCExitCriteria = BatteryTelemetry.BatteryProtectionBinary.BscDirectives.BSCExitCriteria;
    BatteryTelemetrySecondsToHoursVector(BscTelemetry->BSCRsocBucketsLowTemp, BatteryProtectionBSCConfig[CurrentPack].hvtLowTempRsocBuckets_sec, HVT_RSOC_BUCKETS);
    BatteryTelemetrySecondsToHoursVector(BscTelemetry->BSCRsocBucketsMediumLowTemp, BatteryProtectionBSCConfig[CurrentPack].hvtMediumLowTempRsocBuckets_sec, HVT_RSOC_BUCKETS);
    BatteryTelemetrySecondsToHoursVector(BscTelemetry->BSCRsocBucketsMediumHighTemp, BatteryProtectionBSCConfig[CurrentPack].hvtMediumHighTempRsocBuckets_sec, HVT_RSOC_BUCKETS);
    BatteryTelemetrySecondsToHoursVector(BscTelemetry->BSCRsocBucketsHighTemp, BatteryProtectionBSCConfig[CurrentPack].hvtHighTempRsocBuckets_sec, HVT_RSOC_BUCKETS);
    BscTelemetry->BSCHvtcount = CONVERT_SEC_TO_MIN(BatteryProtectionBSCConfig[CurrentPack].eqvTimeAtHvt_sec);

    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);
}

static void BatteryTelemetryGeneratePayloadExtFG(struct BATTERY_PROTECTION *BatteryProtection, uint32_t CurrentPack)
{
    PFG_TELEMETRY FgTelemetry =  NULL;
    union power_supply_propval prop_pack = {0, };

    // Cache Fuel Gauge telemetry
    FgTelemetry = (PFG_TELEMETRY)&BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].FgTelemetry;

    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);

    // FUEL GAUGE TELEMETRY
    if (power_supply_get_property(BatteryProtection->MSEExtFg[CurrentPack], POWER_SUPPLY_PROP_CYCLE_COUNT, &prop_pack) == 0)
        FgTelemetry->CycleCount = (uint16_t)prop_pack.intval;
    if (power_supply_get_property(BatteryProtection->MSEExtFg[CurrentPack], POWER_SUPPLY_PROP_SOH, &prop_pack) == 0)
        FgTelemetry->StateOfHealth = (uint8_t)prop_pack.intval;
    if (power_supply_get_property(BatteryProtection->MSEExtFg[CurrentPack], POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &prop_pack) == 0)
        FgTelemetry->DesignCapacity = (uint16_t)prop_pack.intval;

    FgTelemetry->ChargingVoltage = 0;
    FgTelemetry->LifetimeMaxTemp = 0;
    FgTelemetry->LifetimeMinTemp = 0;
    FgTelemetry->LifetimeMaxPackVoltage = 0;
    FgTelemetry->LifetimeMinPackVoltage = 0;
    FgTelemetry->LifetimeMaxChargeCurrent = 0;
    FgTelemetry->LifetimeMaxDischargeCurrent = 0;
    FgTelemetry->LTFlashCount = 0;
    FgTelemetry->LTAFEStatus = 0;
    FgTelemetry->QmaxCell0 = 0;
    FgTelemetry->VoltageAtChargeTerm = 0;
    FgTelemetry->AverageILastRun = 0;
    FgTelemetry->AveragePLastRun = 0;
    FgTelemetry->DeltaVoltage = 0;

    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);
}

void BatteryTelemetryGeneratePayloadGeneral(void *MSEHw_Config_Input)
{
    uint32_t NumPacks;
    struct MSEHw_Config *MSEHw_Config;
    int i = 0;

    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);

    if(!MSEHw_Config_Input)
    {
        pr_err("BatteryTelemetryGeneratePayloadGeneral: MSEHw_Config_Input missing");
        goto Exit;
    }

    MSEHw_Config = MSEHw_Config_Input;

    NumPacks = MSEHw_Config->MSEBatterySubsystemConfig.NumPacks;

    for(i = 0; i < NumPacks; i++)
    {
        BatteryTelemetry.BatteryTelemetryBinary[i].GeneralTelemetry.InAutoRechargePhase = MSEHw_Config->InAutoRechargePhase;
        BatteryTelemetry.BatteryTelemetryBinary[i].GeneralTelemetry.MonotonicSoc = MSEHw_Config->MSEBatteryStatus[i].monotonic_soc;
        BatteryTelemetry.BatteryTelemetryBinary[i].GeneralTelemetry.VBatt = MSEHw_Config->MSEBatteryStatus[i].vbatt;
        BatteryTelemetry.BatteryTelemetryBinary[i].GeneralTelemetry.IBatt = MSEHw_Config->MSEBatteryStatus[i].ibatt;
        BatteryTelemetry.BatteryTelemetryBinary[i].GeneralTelemetry.Temperature = MSEHw_Config->MSEBatteryStatus[i].temperature;
        BatteryTelemetry.BatteryTelemetryBinary[i].GeneralTelemetry.IsWeakPSU = MSEHw_Config->IsWeakPSU;
        BatteryTelemetry.BatteryTelemetryBinary[i].GeneralTelemetry.CFETOn = MSEHw_Config->MSEPackRTStatus[i].CFETOn;
        BatteryTelemetry.BatteryTelemetryBinary[i].GeneralTelemetry.PackDisconnect = MSEHw_Config->MSEPackRTStatus[i].PackDisconnect;
    }

    BatteryTelemetry.BatteryProtectionBinary.InAutoRecharge = MSEHw_Config->InAutoRechargePhase;

Exit:
    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);
}

static void BpcProcessElapsedTime(struct BATTERY_PROTECTION *BatteryProtection)
{
    static uint32_t flush_elapsedTime_sec[MAX_NUM_FG_PACKS] = {0,0}; // Time since last flush
    uint32_t hvtElapsedTime_sec;
    PBATTERY_PROTECTION_ELAPSED_TIME BatteryProtectionElapsedTime = NULL;
    PSTG_BPC_NVM BatteryProtectionBPCConfig = NULL;
    uint32_t CurrentPack = 0;
    uint32_t NumPacks = 0;
    uint32_t timeSinceLastMinSocVectorUpdate;
    uint64_t timestamp_s = ktime_to_ms(ktime_get_boottime())/1000;
    int rc = 0;

    // Cache num packs
    NumPacks = BatteryProtection->NumPacks;

    // Cache battery elapsed time
    BatteryProtectionElapsedTime = (PBATTERY_PROTECTION_ELAPSED_TIME)&BatteryProtection->BatteryProtectionElapsedTime;

    // Cache BPC config
    BatteryProtectionBPCConfig = (PSTG_BPC_NVM)&BatteryProtection->BatteryProtectionBPCConfig;

    for(CurrentPack = 0; CurrentPack < NumPacks; CurrentPack++)
    {
        if(BatteryProtectionElapsedTime[CurrentPack].elapsedTime_sec)
        {
            // update elapsed time
            flush_elapsedTime_sec[CurrentPack] += BatteryProtectionElapsedTime[CurrentPack].elapsedTime_sec;

            // get HvtElapsedTime since last BPC evaluation
            hvtElapsedTime_sec = BatteryProtectionElapsedTime[CurrentPack].hvtElapsedTime_sec;

            // Update Bpc counters as needed
            if (BatteryProtection->MSEBatteryStatus[CurrentPack].monotonic_soc > HVT_TIME_ACCUMULATION_RSOC)
            {
                BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvt_sec += hvtElapsedTime_sec;
            }

            // BPC telemetry
            if (BatteryTelemetry.BatteryProtectionBinary.BscDirectives.InActiveBsc
                 && BatteryProtection->MSEBatteryStatus[CurrentPack].monotonic_soc > HVT_TIME_ACCUMULATION_RSOC_BSC)
                BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvtInBsc_sec += hvtElapsedTime_sec;

            if (BatteryProtection->MSEBatteryStatus[CurrentPack].monotonic_soc < BatteryProtectionBPCConfig[CurrentPack].minSOC1day)
                BatteryProtectionBPCConfig[CurrentPack].minSOC1day = BatteryProtection->MSEBatteryStatus[CurrentPack].monotonic_soc;

            timeSinceLastMinSocVectorUpdate = ((timestamp_s / SECONDS_PER_DAY) - BatteryProtectionBPCConfig[CurrentPack].lastMinSOCDay);

            // Update remaining Bpc structs
            if (timeSinceLastMinSocVectorUpdate)
            {
                BatteryProtectionBPCConfig[CurrentPack].lastMinSOCDay = (timestamp_s / SECONDS_PER_DAY);
                BpcUpdateMinSocVector((uint8_t *)&BatteryProtectionBPCConfig[CurrentPack].minSOC28day, sizeof(BatteryProtectionBPCConfig[CurrentPack].minSOC28day),
                                    &BatteryProtectionBPCConfig[CurrentPack].minSOC28dayIndex, timeSinceLastMinSocVectorUpdate, BatteryProtectionBPCConfig[CurrentPack].minSOC1day);
                BpcUpdateMinSocVector((uint8_t *)&BatteryProtectionBPCConfig[CurrentPack].minSOC7day, sizeof(BatteryProtectionBPCConfig[CurrentPack].minSOC7day),
                                    &BatteryProtectionBPCConfig[CurrentPack].minSOC7dayIndex, timeSinceLastMinSocVectorUpdate, BatteryProtectionBPCConfig[CurrentPack].minSOC1day);
                BpcUpdateMinSocVector((uint8_t *)&BatteryProtectionBPCConfig[CurrentPack].minSOC3day, sizeof(BatteryProtectionBPCConfig[CurrentPack].minSOC3day),
                                    &BatteryProtectionBPCConfig[CurrentPack].minSOC3dayIndex, timeSinceLastMinSocVectorUpdate, BatteryProtectionBPCConfig[CurrentPack].minSOC1day);

                // Every 24 hours clear minSOC1day to pick up the new minimum
                BatteryProtectionBPCConfig[CurrentPack].minSOC1day = U8_MAX;
            }

            // Clear elapsed time as needed
            if (flush_elapsedTime_sec[CurrentPack] >= BPC_EXTFG_DATA_FLUSH_INTERVAL_SEC)
            {
                rc = FlushHvtCountToExtFg(BatteryProtection, CurrentPack);
                if(rc < 0)
                {
                    pr_err("FlushHvtCountToExtFg failed for pack[%d] with rc = %d", CurrentPack, rc);
                }

                // flush the FG telemetry to sysfs node
                BatteryTelemetryGeneratePayloadExtFG(BatteryProtection, CurrentPack);

                flush_elapsedTime_sec[CurrentPack] = 0;
            }
        }
        pr_debug("Pack[%d]: BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvt_sec is %d ", CurrentPack, BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvt_sec);
    }

    BatteryProtectionBPCConfig[FG_001].bpcProtection = BatteryTelemetry.BatteryTelemetryBinary[FG_001].BpcTelemetry.BPCProtection;
    BatteryProtectionBPCConfig[FG_002].bpcProtection = BatteryTelemetry.BatteryTelemetryBinary[FG_002].BpcTelemetry.BPCProtection;
    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);
    BatteryTelemetry.BatteryProtectionBinary.Pack1Temperature = BatteryProtection->MSEBatteryStatus[FG_001].temperature;
    BatteryTelemetry.BatteryProtectionBinary.Pack2Temperature = BatteryProtection->MSEBatteryStatus[FG_002].temperature;
    BatteryTelemetry.BatteryProtectionBinary.Pack1Hvtcount = BatteryProtectionBPCConfig[FG_001].eqvTimeAtHvt_sec;
    BatteryTelemetry.BatteryProtectionBinary.Pack2Hvtcount = BatteryProtectionBPCConfig[FG_002].eqvTimeAtHvt_sec;
    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);

    return;
}

static void BpcEvaluatePeriodic(struct BATTERY_PROTECTION *BatteryProtection)
{
    // When feature is force disabled exit immediately
    if(BatteryProtection->IsDisableBPC)
        return;

    // Update BPC counters as needed
    BpcProcessElapsedTime(BatteryProtection);
}


/********************************************
 *                  MSE BSC                 *
 *******************************************/

static void BscProcessElapsedTime(struct BATTERY_PROTECTION *BatteryProtection, bool *bscIsDischarging)
{
    uint32_t hvtElapsedTime_sec;
    PBATTERY_PROTECTION_ELAPSED_TIME BatteryProtectionElapsedTime = NULL;
    PBSC_CONFIG BatteryProtectionBSCConfig = NULL;
    uint32_t CurrentPack, NumPacks, BucketIndex = 0;
    int32_t TempDeciC, TrueRsoc = 0;
    uint32_t *RsocThresholds = NULL;
    uint32_t *TempBuckets = NULL;
    struct timespec ts;

    // Cache num packs
    NumPacks = BatteryProtection->NumPacks;

    // Cache battery elapsed time
    BatteryProtectionElapsedTime = (PBATTERY_PROTECTION_ELAPSED_TIME)&BatteryProtection->BatteryProtectionElapsedTime;

    // Cache BSC config
    BatteryProtectionBSCConfig = (PBSC_CONFIG)&BatteryProtection->BatteryProtectionBSCConfig;

    for(CurrentPack = 0; CurrentPack < NumPacks; CurrentPack++)
    {
        if(BatteryProtectionElapsedTime[CurrentPack].elapsedTime_sec)
        {
            // get HvtElapsedTime since last BSC evaluation
            hvtElapsedTime_sec = BatteryProtectionElapsedTime[CurrentPack].hvtElapsedTime_sec;

            // Update BSC counters as needed
            if (BatteryProtection->MSEBatteryStatus[CurrentPack].monotonic_soc > HVT_TIME_ACCUMULATION_RSOC)
            {
                if (!BatteryProtectionBSCConfig[CurrentPack].bscStartDate)
                {
                    getnstimeofday(&ts);
                    BatteryProtectionBSCConfig[CurrentPack].bscStartDate = ts.tv_sec;
                }
                BatteryProtectionBSCConfig[CurrentPack].eqvTimeAtHvt_sec += hvtElapsedTime_sec;
            }

            if(BatteryTelemetry.BatteryProtectionBinary.BscDirectives.InActiveBsc)
                BatteryProtectionBSCConfig[CurrentPack].bscTotalActive_sec += BatteryProtectionElapsedTime[CurrentPack].elapsedTime_sec;

            // update BSC bucket
            TempDeciC = BatteryProtection->MSEBatteryStatus[CurrentPack].temperature;
            TempBuckets = (TempDeciC < BatteryProtection->BatteryProtectionBSCStaticConfig.HvtTempBucketLow) ? (&BatteryProtectionBSCConfig[CurrentPack].hvtLowTempRsocBuckets_sec[0]) :
                          (TempDeciC > BatteryProtection->BatteryProtectionBSCStaticConfig.HvtTempBucketHigh) ? (&BatteryProtectionBSCConfig[CurrentPack].hvtHighTempRsocBuckets_sec[0]) :
                          (TempDeciC > BatteryProtection->BatteryProtectionBSCStaticConfig.HvtTempBucketMid) ? (&BatteryProtectionBSCConfig[CurrentPack].hvtMediumHighTempRsocBuckets_sec[0]) :
                          (&BatteryProtectionBSCConfig[CurrentPack].hvtMediumLowTempRsocBuckets_sec[0]);

            RsocThresholds = &BatteryProtection->BatteryProtectionBSCStaticConfig.HvtRsocBucketThresholds[0];
            BucketIndex = 0;

            TrueRsoc = BatteryProtection->MSEBatteryStatus[CurrentPack].monotonic_soc -
                       (BatteryProtection->BatteryProtectionBPCConfig[CurrentPack].bpcProtection / 10);

            // find the bucket index that contains the current rsoc
            while ((TrueRsoc > *RsocThresholds) && (++BucketIndex < (HVT_RSOC_BUCKETS - 1u)))
            {
                RsocThresholds++;
            }
            TempBuckets[BucketIndex] += BatteryProtectionElapsedTime[CurrentPack].elapsedTime_sec;

            pr_debug("Pack[%d]: BatteryProtectionBSCConfig[CurrentPack].eqvTimeAtHvt_sec: %d",
                    CurrentPack, BatteryProtectionBSCConfig[CurrentPack].eqvTimeAtHvt_sec);
        }
    }

    if(BatteryTelemetry.BatteryProtectionBinary.BscDirectives.BscHvtcountReset)
    {
        BatteryProtectionBSCConfig[FG_001].eqvTimeAtHvt_sec = 0;
        BatteryProtectionBSCConfig[FG_002].eqvTimeAtHvt_sec = 0;
    }

    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);
    BatteryTelemetry.BatteryProtectionBinary.SCPack1Hvtcount = BatteryProtectionBSCConfig[FG_001].eqvTimeAtHvt_sec;
    BatteryTelemetry.BatteryProtectionBinary.SCPack2Hvtcount = BatteryProtectionBSCConfig[FG_002].eqvTimeAtHvt_sec;
    BatteryTelemetry.BatteryProtectionBinary.CombinedRsoc = BatteryProtection->CombinedRsoc;
    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);

    *bscIsDischarging = BatteryTelemetry.BatteryProtectionBinary.BscDirectives.IsDischarging;
    BatteryProtection->IsDisableCharging = BatteryTelemetry.BatteryProtectionBinary.BscDirectives.IsDisableCharging;
    BatteryProtection->BatteryLevel = BatteryTelemetry.BatteryProtectionBinary.BscDirectives.BatteryLevel;
}

static void BscEvaluatePeriodic(struct BATTERY_PROTECTION *BatteryProtection, bool *bscIsDischarging)
{
    // When feature is force disabled exit immediately
    if(BatteryProtection->IsDisableBSC)
        return;

    // Update BSC counters as needed
    BscProcessElapsedTime(BatteryProtection, bscIsDischarging);
}

static void ManufacturingModeLevelingEvaluatePeriodic(struct BATTERY_PROTECTION *BatteryProtection, bool *manufModeIsDischarging)
{
    bool IsDischarging = false; // initially setting this to false

    mutex_lock(&ManufModeConfig.ManufModeLevelingLock);

    if(!ManufModeConfig.InManufacturingMode)
        goto Exit;  // return immediately when the device is not in manufacturing mode

    if(BatteryProtection->CombinedRsoc > ManufModeConfig.UthManufModeRsoc)
    {
        IsDischarging = true;
        BatteryProtection->IsDisableCharging = true;
    }
    else if (BatteryProtection->CombinedRsoc < ManufModeConfig.LthManufModeRsoc)
    {
        IsDischarging = false;
        BatteryProtection->IsDisableCharging = false;
    }
    else // CombineRsoc is within ManufModeLeveling Hysteresis , enable input power, charging will managed when outside of Hysteresis
    {
        IsDischarging = false;
    }

    *manufModeIsDischarging = IsDischarging;

Exit:
    mutex_unlock(&ManufModeConfig.ManufModeLevelingLock);
}

/********************************************
 *  MSE Battery Protection Common Functions *
 *******************************************/

static int MSEHwExtFgGetBatteryStatus(struct BATTERY_PROTECTION *BatteryProtection)
{
    union power_supply_propval prop_pack = {0, };
    int rc = 0;
    int i = 0;

    for (i = 0; i < BatteryProtection->NumPacks; i++)
    {
        // query monotonic soc
        if (power_supply_get_property(BatteryProtection->MSEExtFg[i], POWER_SUPPLY_PROP_CAPACITY, &prop_pack))
        {
            pr_err("Could not read pack[%d] soc\n", i);
            rc |= -ENODATA;
        }
        else
        {
            BatteryProtection->MSEBatteryStatus[i].monotonic_soc = prop_pack.intval;
        }

        // query temperature
        if (power_supply_get_property(BatteryProtection->MSEExtFg[i], POWER_SUPPLY_PROP_TEMP, &prop_pack))
        {
            pr_err("Could not read pack[%d] temperature\n", i);
            rc |= -ENODATA;
        }
        else
        {
            BatteryProtection->MSEBatteryStatus[i].temperature = prop_pack.intval; // deci C
        }

        pr_debug("Pack[%d]: monotonic soc: %d, temperature: %d \n", i,
                 BatteryProtection->MSEBatteryStatus[i].monotonic_soc,
                 BatteryProtection->MSEBatteryStatus[i].temperature);
    }

    if (power_supply_get_property(BatteryProtection->bms, POWER_SUPPLY_PROP_CAPACITY, &prop_pack))
    {
        // we should never come here since fg driver is already storing previous capacity and returning that in case of any failures
        pr_err("Could not read combined rsoc\n");
    }
    else
    {
        BatteryProtection->CombinedRsoc = prop_pack.intval;
    }
    return rc;
}

static void BatteryTelemetryHoursToSecondsVector(uint32_t *pDst, uint16_t *pSrc, uint32_t size)
{
    while (size--)
    {
        *pDst++ = (*pSrc++) * 3600u;
    }
}

static void RestoreFromFileBackup(struct BATTERY_PROTECTION *BatteryProtection)
{
    uint32_t CurrentPack, NumPacks = 0;
    PSTG_BPC_NVM BatteryProtectionBPCConfig = NULL;
    PBSC_CONFIG BatteryProtectionBSCConfig = NULL;
    uint32_t bpc_eqvTimeAtHvt_sec = 0;
    bool GenerateExtFgTelemetryPayload[MAX_NUM_FG_PACKS] = {false, false};

    struct MSEHw_Config *MSEHw_Config = container_of(BatteryProtection, struct MSEHw_Config, BatteryProtection);

    // Cache num packs
    NumPacks = BatteryProtection->NumPacks;

    // Cache BPC config
    BatteryProtectionBPCConfig = (PSTG_BPC_NVM)&BatteryProtection->BatteryProtectionBPCConfig;

    // Cache BSC config
    BatteryProtectionBSCConfig = (PBSC_CONFIG)&BatteryProtection->BatteryProtectionBSCConfig;

    for(CurrentPack = 0; CurrentPack < NumPacks; CurrentPack++)
    {
        mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);

        if(BatteryTelemetry.IsFirstBoot[CurrentPack])
        {
            BatteryTelemetry.IsFirstBoot[CurrentPack] = false;
            BatteryTelemetry.IsBatteryTelemetryPresent[CurrentPack] = true;  // we can let the telemetry service query for battery payload since it is first boot
            GenerateExtFgTelemetryPayload[CurrentPack] = true;
        }
        else if (BatteryTelemetry.FileRestore[CurrentPack])
        {
            // restore BPC data
            bpc_eqvTimeAtHvt_sec = CONVERT_MIN_TO_SEC(BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BpcTelemetry.BPCEqvTimeat35C);
            if(bpc_eqvTimeAtHvt_sec > BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvt_sec)
                BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvt_sec = bpc_eqvTimeAtHvt_sec;
            BatteryProtectionBPCConfig[CurrentPack].eqvTimeAtHvtInBsc_sec += CONVERT_MIN_TO_SEC(BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BpcTelemetry.BPCEqvTimeat35CinBSC);
            BatteryProtectionBPCConfig[CurrentPack].minSOC1day = BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BpcTelemetry.BPCMinSOC1day;
            BatteryProtectionBPCConfig[CurrentPack].minSOC3day[0] = BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BpcTelemetry.BPCMinSOC3day;
            BatteryProtectionBPCConfig[CurrentPack].minSOC3dayIndex++;
            BatteryProtectionBPCConfig[CurrentPack].minSOC7day[0] = BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BpcTelemetry.BPCMinSOC7day;
            BatteryProtectionBPCConfig[CurrentPack].minSOC7dayIndex++;
            BatteryProtectionBPCConfig[CurrentPack].minSOC28day[0] = BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BpcTelemetry.BPCMinSOC28day;
            BatteryProtectionBPCConfig[CurrentPack].minSOC28dayIndex++;

            // restore BSC data
            BatteryProtectionBSCConfig[CurrentPack].bscStartDate = BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BscTelemetry.BSCStartDateInSeconds;
            BatteryProtectionBSCConfig[CurrentPack].bscTotalActive_sec += CONVERT_MIN_TO_SEC(BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BscTelemetry.BSCTotalEngagedMinutes);
            BatteryTelemetry.BatteryProtectionBinary.BscDirectives.BSCTotalEntryEvents = BatteryTelemetry.BatteryTelemetryBinary[FG_001].BscTelemetry.BSCTotalEntryEvents; // note: field has been synced between the two packs during write to file backup, so restoring for one pack is sufficient here
            BatteryProtectionBSCConfig[CurrentPack].eqvTimeAtHvt_sec += CONVERT_MIN_TO_SEC(BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BscTelemetry.BSCHvtcount);
            BatteryTelemetryHoursToSecondsVector(BatteryProtectionBSCConfig[CurrentPack].hvtLowTempRsocBuckets_sec, BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BscTelemetry.BSCRsocBucketsLowTemp, HVT_RSOC_BUCKETS);
            BatteryTelemetryHoursToSecondsVector(BatteryProtectionBSCConfig[CurrentPack].hvtMediumLowTempRsocBuckets_sec, BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BscTelemetry.BSCRsocBucketsMediumLowTemp, HVT_RSOC_BUCKETS);
            BatteryTelemetryHoursToSecondsVector(BatteryProtectionBSCConfig[CurrentPack].hvtMediumHighTempRsocBuckets_sec, BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BscTelemetry.BSCRsocBucketsMediumHighTemp, HVT_RSOC_BUCKETS);
            BatteryTelemetryHoursToSecondsVector(BatteryProtectionBSCConfig[CurrentPack].hvtHighTempRsocBuckets_sec, BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BscTelemetry.BSCRsocBucketsHighTemp, HVT_RSOC_BUCKETS);
            BatteryTelemetry.BatteryProtectionBinary.BscDirectives.InActiveBsc = BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].BscTelemetry.BSCCurrentlyEngaged;

            MSEHw_Config->InAutoRechargePhase = BatteryTelemetry.BatteryTelemetryBinary[CurrentPack].GeneralTelemetry.InAutoRechargePhase;
            pr_debug("RestoreFromFileBackup: Pack[%d], InAutoRechargePhase: %d, InActiveBsc: %d", CurrentPack, MSEHw_Config->InAutoRechargePhase, BatteryTelemetry.BatteryProtectionBinary.BscDirectives.InActiveBsc);

            BatteryTelemetry.FileRestore[CurrentPack] = false;
            BatteryTelemetry.IsBatteryTelemetryPresent[CurrentPack] = true;  // we can let the telemetry service query for battery payload after file restore has been completed
            GenerateExtFgTelemetryPayload[CurrentPack] = true;
        }

        mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);

        if(BatteryTelemetry.IsBatteryTelemetryPresent[CurrentPack])
        {
            BatteryTelemetryGeneratePayloadBPC(BatteryProtection, CurrentPack);
            BatteryTelemetryGeneratePayloadBSC(BatteryProtection, CurrentPack);
        }

        if(GenerateExtFgTelemetryPayload[CurrentPack])
        {
            BatteryTelemetryGeneratePayloadExtFG(BatteryProtection, CurrentPack);
            GenerateExtFgTelemetryPayload[CurrentPack] = false;  // we will allow the EXTFG payload to be called only during bootup and then every 2 hours instead of during every heartbeat
        }
    }
}

/********************************************
 *    MSE Battery Protection DTSI parsing   *
 *******************************************/
static int GetBSCStaticInfoFromDtsi(struct device_node *node, struct BATTERY_PROTECTION *BatteryProtection)
{
    int rc = 0;

    rc = of_property_read_u32(node, "ms,hvt-temp-bucket-low",
					&BatteryProtection->BatteryProtectionBSCStaticConfig.HvtTempBucketLow);
    if (rc < 0)
		goto Exit;

    rc = of_property_read_u32(node, "ms,hvt-temp-bucket-mid",
					&BatteryProtection->BatteryProtectionBSCStaticConfig.HvtTempBucketMid);
    if (rc < 0)
		goto Exit;

    rc = of_property_read_u32(node, "ms,hvt-temp-bucket-high",
					&BatteryProtection->BatteryProtectionBSCStaticConfig.HvtTempBucketHigh);
    if (rc < 0)
		goto Exit;

    rc = of_property_read_u32_array(node, "ms,hvt-rsoc-buckets",
					(u32*)&BatteryProtection->BatteryProtectionBSCStaticConfig.HvtRsocBucketThresholds, HVT_RSOC_BUCKETS);
    if (rc < 0)
		goto Exit;

Exit:
    return rc;
}

static int GetBatteryProtectionInfoFromDtsi(struct device_node *node, struct BATTERY_PROTECTION *BatteryProtection)
{
    int i = 0;
    int rc = 0;
    const char *fuel_gauge_names[MAX_NUM_FG_PACKS];

    // get NumPacks from DTSI
    rc = of_property_read_u32(node, "ms,numpacks",
                              &BatteryProtection->NumPacks);
    if (rc < 0)
        goto Exit;

    // get max float voltage from DTSI
    rc = of_property_read_u32(node, "qcom,fv-max-uv",
                              &BatteryProtection->FloatVoltageMax);
    if (rc < 0)
        goto Exit;

    rc = GetBSCStaticInfoFromDtsi(node, BatteryProtection);
    if (rc < 0)
    {
        pr_err("GetBSCStaticInfoFromDtsi failed with rc = %d", rc);
        goto Exit;
    }

    // get fuel gauge handles from DTSI
    rc = of_property_read_string_array(node, "ms,fuel-gauges", fuel_gauge_names, MAX_NUM_FG_PACKS);
    if(rc < 0)
    {
        pr_err("failed to get list of extfgs from dtsi");
        goto Exit;
    }

    for (i = 0; i < MAX_NUM_FG_PACKS; i++)
    {
        BatteryProtection->MSEExtFg[i] = power_supply_get_by_name(fuel_gauge_names[i]);
        if(!BatteryProtection->MSEExtFg[i])
        {
            pr_err("failed to register MSEExtFg[%d] rc = %ld\n", i, PTR_ERR(BatteryProtection->MSEExtFg[i]));
            rc = -EINVAL;
            goto Exit;
        }
    }

    BatteryProtection->bms = power_supply_get_by_name("bms"); // get a handle to bms psy
    if(!BatteryProtection->bms)
    {
        pr_err("failed to register bms rc = %ld\n", PTR_ERR(BatteryProtection->bms));
        rc = -EINVAL;
        goto Exit;
    }

Exit:
    return rc;
}

/*************************************************
 *   MSE Battery Protection Telemetry Functions  *
 ************************************************/

static ssize_t telem_battery_pack_1_payload_read(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);
    memcpy(buf, &BatteryTelemetry.BatteryTelemetryBinary[FG_001], sizeof(BatteryTelemetry.BatteryTelemetryBinary[FG_001]));
    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);

    return sizeof(BatteryTelemetry.BatteryTelemetryBinary[FG_001]);
}

static ssize_t telem_battery_pack_1_payload_write(struct kobject *kobj, struct kobj_attribute *attr,
		      const char *buf, size_t length)
{
    uint32_t BatteryTelemetryVersionRestored = 0;

    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);
    if(length == 1 && buf[0] == 0x0)
    {
        BatteryTelemetry.IsFirstBoot[FG_001] = true;
    }
    else
    {
        if(length > sizeof(BatteryTelemetry.BatteryTelemetryBinary[FG_001].BatteryTelemetryVersion))
        {
            memcpy(&BatteryTelemetryVersionRestored, buf, sizeof(BatteryTelemetry.BatteryTelemetryBinary[FG_001].BatteryTelemetryVersion));

            if(BatteryTelemetryVersionRestored == BATTERY_TELEMETRY_VERSION)
            {
                memcpy(&BatteryTelemetry.BatteryTelemetryBinary[FG_001], buf, sizeof(BatteryTelemetry.BatteryTelemetryBinary[FG_001]));
                BatteryTelemetry.FileRestore[FG_001] = true;
            }
            else
            {
                pr_err("telem_battery_pack_1_payload_write: BatteryProtectionVersion from file backup does not match the driver, discarding the file restore");
                BatteryTelemetry.IsFirstBoot[FG_001] = true;  // if version does not match, treat it as a first boot and do not restore from file backup
            }
        }
    }
    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);
    return length;
}

static ssize_t telem_battery_pack_1_status_read(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
    uint32_t telem_battery_status = 0;

    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);
    telem_battery_status = BatteryTelemetry.FileRestoreOptin | (BatteryTelemetry.IsBatteryTelemetryPresent[FG_001] << 1);
    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);

    return sprintf(buf, "%d", telem_battery_status);
}

static ssize_t telem_battery_pack_2_payload_read(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);
    memcpy(buf, &BatteryTelemetry.BatteryTelemetryBinary[FG_002], sizeof(BatteryTelemetry.BatteryTelemetryBinary[FG_002]));
    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);

    return sizeof(BatteryTelemetry.BatteryTelemetryBinary[FG_002]);
}

static ssize_t telem_battery_pack_2_payload_write(struct kobject *kobj, struct kobj_attribute *attr,
		      const char *buf, size_t length)
{
    uint32_t BatteryTelemetryVersionRestored = 0;

    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);
    if(length == 1 && buf[0] == 0x0)
    {
        BatteryTelemetry.IsFirstBoot[FG_002] = true;
    }
    else
    {
        if(length > sizeof(BatteryTelemetry.BatteryTelemetryBinary[FG_002].BatteryTelemetryVersion))
        {
            memcpy(&BatteryTelemetryVersionRestored, buf, sizeof(BatteryTelemetry.BatteryTelemetryBinary[FG_002].BatteryTelemetryVersion));

            if(BatteryTelemetryVersionRestored == BATTERY_TELEMETRY_VERSION)
            {
                memcpy(&BatteryTelemetry.BatteryTelemetryBinary[FG_002], buf, sizeof(BatteryTelemetry.BatteryTelemetryBinary[FG_002]));
                BatteryTelemetry.FileRestore[FG_002] = true;
            }
            else
            {
                pr_err("telem_battery_pack_2_payload_write: BatteryProtectionVersion from file backup does not match the driver, discarding the file restore");
                BatteryTelemetry.IsFirstBoot[FG_002] = true;  // if version does not match, treat it as a first boot and do not restore from file backup
            }
        }
    }
    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);
    return length;
}

static ssize_t telem_battery_pack_2_status_read(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
    uint32_t telem_battery_status = 0;

    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);
    telem_battery_status = BatteryTelemetry.FileRestoreOptin | (BatteryTelemetry.IsBatteryTelemetryPresent[FG_002] << 1);
    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);

    return sprintf(buf, "%d", telem_battery_status);
}

static ssize_t telem_battery_3_payload_read(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
    memcpy(buf, &BatteryTelemetry.BatteryProtectionBinary, sizeof(BatteryTelemetry.BatteryProtectionBinary));
    return sizeof(BatteryTelemetry.BatteryProtectionBinary);
}

static ssize_t telem_battery_3_payload_write(struct kobject *kobj, struct kobj_attribute *attr,
		      const char *buf, size_t length)
{
    mutex_lock(&BatteryTelemetry.BatteryTelemetryLock);
    memcpy(&BatteryTelemetry.BatteryProtectionBinary.BscDirectives, buf, sizeof(BatteryTelemetry.BatteryProtectionBinary.BscDirectives));
    mutex_unlock(&BatteryTelemetry.BatteryTelemetryLock);
    return length;
}

static struct kobj_attribute telem_battery_pack_1_payload_attribute =
	__ATTR(payload, 0664, telem_battery_pack_1_payload_read, telem_battery_pack_1_payload_write);

static struct kobj_attribute telem_battery_pack_1_status_attribute =
	__ATTR(status, 0664, telem_battery_pack_1_status_read, NULL);

static struct kobj_attribute telem_battery_pack_2_payload_attribute =
	__ATTR(payload, 0664, telem_battery_pack_2_payload_read, telem_battery_pack_2_payload_write);

static struct kobj_attribute telem_battery_pack_2_status_attribute =
	__ATTR(status, 0664, telem_battery_pack_2_status_read, NULL);

static struct kobj_attribute telem_battery_3_payload_attribute =
	__ATTR(payload, 0664, telem_battery_3_payload_read, telem_battery_3_payload_write);

static struct attribute *attrs_pack_1[] = {
    &telem_battery_pack_1_payload_attribute.attr,
    &telem_battery_pack_1_status_attribute.attr,
    NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute *attrs_pack_2[] = {
    &telem_battery_pack_2_payload_attribute.attr,
    &telem_battery_pack_2_status_attribute.attr,
    NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute *attrs_3[] = {
    &telem_battery_3_payload_attribute.attr,
    NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group_pack_1 = {
	.attrs = attrs_pack_1,
};

static struct attribute_group attr_group_pack_2 = {
	.attrs = attrs_pack_2,
};

static struct attribute_group attr_group_3 = {
	.attrs = attrs_3,
};

static int BatteryTelemetryInit(struct BATTERY_PROTECTION *BatteryProtection)
{
    int rc;
    struct kobject *BatteryTelemetryKobj = {0};
    struct kobject *BatteryTelemetryKobjPack1 = {0};
    struct kobject *BatteryTelemetryKobjPack2 = {0};
    struct kobject *BatteryTelemetryKobj3 = {0};

    // initialize telemetry mutex
    mutex_init(&BatteryTelemetry.BatteryTelemetryLock);

    // initialize minSOC1day
    BatteryProtection->BatteryProtectionBPCConfig[FG_001].minSOC1day = BatteryProtection->MSEBatteryStatus[FG_001].monotonic_soc;
    BatteryProtection->BatteryProtectionBPCConfig[FG_002].minSOC1day = BatteryProtection->MSEBatteryStatus[FG_002].monotonic_soc;

    BatteryTelemetry.IsBatteryTelemetryPresent[FG_001] = false; // setting telemetry present flag to false for init
    BatteryTelemetry.IsBatteryTelemetryPresent[FG_002] = false;
    BatteryTelemetry.FileRestore[FG_001] = false; // setting file restore done flag to false for init
    BatteryTelemetry.FileRestore[FG_002] = false;
    BatteryTelemetry.FileRestoreOptin = true; // battery driver will always be opting in for file backup and restore
    BatteryTelemetry.BatteryTelemetryBinary[FG_001].BatteryTelemetryVersion = BATTERY_TELEMETRY_VERSION;  // setting the Battery Telemetry Version
    BatteryTelemetry.BatteryTelemetryBinary[FG_002].BatteryTelemetryVersion = BATTERY_TELEMETRY_VERSION;
    BatteryTelemetry.BatteryProtectionBinary.BscDirectives.BatteryLevel = -1;

    rc = telemetry_init();
    if (rc < 0)
    {
        pr_err("BatteryTelemetryInit: failed to create telemetry parent obj");
        goto Exit;
    }

    BatteryTelemetryKobj = kobject_create_and_add("battery", telemetry_kobj);
    if (!BatteryTelemetryKobj)
    {
        pr_err("BatteryTelemetryInit: failed to create battery parent obj");
        rc = -ENOMEM;
        kobject_put(BatteryTelemetryKobj); //decrease reference count
        goto Exit;
    }

    // Setup sysfs directory for telemetry pack 1
    BatteryTelemetryKobjPack1 = kobject_create_and_add("1", BatteryTelemetryKobj);
    if (!BatteryTelemetryKobjPack1)
    {
        pr_err("BatteryTelemetryInit: failed to create telem_battery_pack_1 parent obj");
        rc = -ENOMEM;
        kobject_put(BatteryTelemetryKobjPack1); //decrease reference count
        goto Exit;
    }

    // Add 'files' under the sysfs directory for pack 1
    rc = sysfs_create_group(BatteryTelemetryKobjPack1, &attr_group_pack_1);
    if (rc)
    {
        pr_err("BatteryTelemetryInit: failed to add sysfs group for telem_battery_pack_1");
        rc = -EINVAL;
        kobject_del(BatteryTelemetryKobjPack1);
        goto Exit;
    }

    // Setup sysfs directory for telemetry for pack 2
    BatteryTelemetryKobjPack2 = kobject_create_and_add("2", BatteryTelemetryKobj);
    if (!BatteryTelemetryKobjPack2)
    {
        pr_err("BatteryTelemetryInit: failed to create telem_battery_pack_2 parent obj");
        rc = -ENOMEM;
        kobject_put(BatteryTelemetryKobjPack2); //decrease reference count
        goto Exit;
    }

    // Add 'files' under the sysfs directory for pack 2
    rc = sysfs_create_group(BatteryTelemetryKobjPack2, &attr_group_pack_2);
    if (rc)
    {
        pr_err("BatteryTelemetryInit: failed to add sysfs group for telem_battery_pack_2");
        rc = -EINVAL;
        kobject_del(BatteryTelemetryKobjPack2);
        goto Exit;
    }

        // Setup sysfs directory for telemetry
    BatteryTelemetryKobj3 = kobject_create_and_add("3", BatteryTelemetryKobj);
    if (!BatteryTelemetryKobj3)
    {
        pr_err("BatteryTelemetryInit: failed to create telem_battery_3 parent obj");
        rc = -ENOMEM;
        kobject_put(BatteryTelemetryKobj3); //decrease reference count
        goto Exit;
    }

    // Add 'files' under the sysfs directory
    rc = sysfs_create_group(BatteryTelemetryKobj3, &attr_group_3);
    if (rc)
    {
        pr_err("BatteryTelemetryInit: failed to add sysfs group for telem_battery_3");
        rc = -EINVAL;
        kobject_del(BatteryTelemetryKobj3);
        goto Exit;
    }

Exit:
    return rc;
}

static ssize_t battery_manuf_mode_leveling_uth_rsoc_read(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
    return sprintf(buf, "%d", ManufModeConfig.UthManufModeRsoc);
}

static ssize_t battery_manuf_mode_leveling_uth_rsoc_write(struct kobject *kobj, struct kobj_attribute *attr, const char *ubuf, size_t count)
{
    unsigned long val;

    if (kstrtoul(ubuf, 10, &val))
		return -EINVAL;

    mutex_lock(&ManufModeConfig.ManufModeLevelingLock);
    ManufModeConfig.UthManufModeRsoc = val;
    mutex_unlock(&ManufModeConfig.ManufModeLevelingLock);
    return count;
}

static ssize_t battery_manuf_mode_leveling_lth_rsoc_read(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
    return sprintf(buf, "%d", ManufModeConfig.LthManufModeRsoc);
}

static ssize_t battery_manuf_mode_leveling_lth_rsoc_write(struct kobject *kobj, struct kobj_attribute *attr, const char *ubuf, size_t count)
{
    unsigned long val;
    if (kstrtoul(ubuf, 10, &val))
        return -EINVAL;

    mutex_lock(&ManufModeConfig.ManufModeLevelingLock);
    ManufModeConfig.LthManufModeRsoc = val;
    mutex_unlock(&ManufModeConfig.ManufModeLevelingLock);
    return count;
}

static struct kobj_attribute battery_manuf_mode_leveling_uth_rsoc =
    __ATTR(uth_manuf_mode_leveling_rsoc, 0664, battery_manuf_mode_leveling_uth_rsoc_read, battery_manuf_mode_leveling_uth_rsoc_write);

static struct kobj_attribute battery_manuf_mode_leveling_lth_rsoc =
    __ATTR(lth_manuf_mode_leveling_rsoc, 0664, battery_manuf_mode_leveling_lth_rsoc_read, battery_manuf_mode_leveling_lth_rsoc_write);


static struct attribute *battery_leveling_attrs[] = {
    &battery_manuf_mode_leveling_uth_rsoc.attr,
    &battery_manuf_mode_leveling_lth_rsoc.attr,
    NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group battery_leveling_attr_group = {
    .attrs = battery_leveling_attrs,
};

static int ManufacturingModeLevelingInit(struct BATTERY_PROTECTION *BatteryProtection)
{
    int rc;

    // initialize manuf mode leveling mutex
    mutex_init(&ManufModeConfig.ManufModeLevelingLock);

    // check if device is in manufacturing mode
    ManufModeConfig.InManufacturingMode = get_manuf_mode();
    ManufModeConfig.UthManufModeRsoc = MANUF_MODE_LEVELING_RSOC;  // initially setting manuf mode upper limit to 66 RSOC
    ManufModeConfig.LthManufModeRsoc = MANUF_MODE_LEVELING_RSOC;  // initially setting manuf mode lower limit to 66 RSOC

    // Add queries under the sysfs directory
    rc = sysfs_create_group(BatteryProtection->BatteryKobj, &battery_leveling_attr_group);
    if (rc)
    {
        pr_err("ManufacturingModeLevelingInit: failed to add sysfs group for telem_battery");
        rc = -EINVAL;
        kobject_del(BatteryProtection->BatteryKobj);
        goto Exit;
    }

Exit:
    return rc;
}

/***********************************************************
 *    MSE Battery Protection Periodic and Init Functions   *
 ***********************************************************/

int MSEBatteryProtectionPeriodic(struct BATTERY_PROTECTION *BatteryProtection, bool InDischargePhase, bool *IsForcedDischarging)
{
    int rc = 0;
    uint32_t timestamp_s = ktime_to_ms(ktime_get_boottime())/1000;

    rc = MSEHwExtFgGetBatteryStatus(BatteryProtection);
    if (rc < 0)
    {
        pr_err("MSEHwExtFgGetBatteryStatus failed with rc = %d", rc);
        goto Exit;
    }

    RestoreFromFileBackup(BatteryProtection);

    ComputeElapsedTime(BatteryProtection, timestamp_s);

    BpcEvaluatePeriodic(BatteryProtection);

    BscEvaluatePeriodic(BatteryProtection, IsForcedDischarging);

    BatteryTelemetry.BatteryProtectionBinary.InDischargePhase = InDischargePhase;

    // IsForcedDischarging will be overwritten only when Manufacturing Mode is enabled
    ManufacturingModeLevelingEvaluatePeriodic(BatteryProtection, IsForcedDischarging);

Exit:
    return rc;
}

static ssize_t battery_protection_state_read(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint8_t retVal = 0;

    if(BatteryTelemetry.BatteryTelemetryBinary[FG_001].BscTelemetry.BSCCurrentlyEngaged)
        retVal += (FG_001 + 1);

    if(BatteryTelemetry.BatteryTelemetryBinary[FG_002].BscTelemetry.BSCCurrentlyEngaged)
        retVal += (FG_002 + 1);

    return sprintf(buf, "%d", retVal);
}

static ssize_t battery_protection_bpc_hvtcount_pack1_read(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint32_t bpc_eqvTimeAtHvt_sec = CONVERT_MIN_TO_SEC(BatteryTelemetry.BatteryTelemetryBinary[FG_001].BpcTelemetry.BPCEqvTimeat35C);
    return sprintf(buf, "%ld", bpc_eqvTimeAtHvt_sec);
}

static ssize_t battery_protection_bpc_hvtcount_pack2_read(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint32_t bpc_eqvTimeAtHvt_sec = CONVERT_MIN_TO_SEC(BatteryTelemetry.BatteryTelemetryBinary[FG_002].BpcTelemetry.BPCEqvTimeat35C);
    return sprintf(buf, "%ld", bpc_eqvTimeAtHvt_sec);
}

static ssize_t battery_protection_bsc_hvtcount_pack1_read(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint32_t bsc_eqvTimeAtHvt_sec = CONVERT_MIN_TO_SEC(BatteryTelemetry.BatteryTelemetryBinary[FG_001].BscTelemetry.BSCHvtcount);
    return sprintf(buf, "%ld", bsc_eqvTimeAtHvt_sec);
}

static ssize_t battery_protection_bsc_hvtcount_pack2_read(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint32_t bsc_eqvTimeAtHvt_sec = CONVERT_MIN_TO_SEC(BatteryTelemetry.BatteryTelemetryBinary[FG_002].BscTelemetry.BSCHvtcount);
    return sprintf(buf, "%ld", bsc_eqvTimeAtHvt_sec);
}

static struct kobj_attribute battery_protection_state_attr =
    __ATTR(bsc_state, 0664, battery_protection_state_read, NULL);

static struct kobj_attribute battery_protection_bpc_hvtcount_pack1_attr =
    __ATTR(bpc_hvtcount_pack1, 0664, battery_protection_bpc_hvtcount_pack1_read, NULL);

static struct kobj_attribute battery_protection_bpc_hvtcount_pack2_attr =
    __ATTR(bpc_hvtcount_pack2, 0664, battery_protection_bpc_hvtcount_pack2_read, NULL);

static struct kobj_attribute battery_protection_bsc_hvtcount_pack1_attr =
    __ATTR(bsc_hvtcount_pack1, 0664, battery_protection_bsc_hvtcount_pack1_read, NULL);

static struct kobj_attribute battery_protection_bsc_hvtcount_pack2_attr =
    __ATTR(bsc_hvtcount_pack2, 0664, battery_protection_bsc_hvtcount_pack2_read, NULL);

static struct attribute *battery_protection_attrs[] = {
    &battery_protection_state_attr.attr,
    &battery_protection_bpc_hvtcount_pack1_attr.attr,
    &battery_protection_bpc_hvtcount_pack2_attr.attr,
    &battery_protection_bsc_hvtcount_pack1_attr.attr,
    &battery_protection_bsc_hvtcount_pack2_attr.attr,
    NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group battery_protection_attr_group = {
    .attrs = battery_protection_attrs,
};

#define MAX_HVT_COUNT_WA    0x1000000  // this is a WA due to old FGs not having the HVT COUNT bits zeroed out, enabled only for EV2 and lower builds
#define EV2_BOARD_ID        0xF        // highest board ID for EV2 builds
int MSEBatteryProtectionInit(struct device_node *node, struct BATTERY_PROTECTION *BatteryProtection)
{
    int rc, i;
    uint32_t PlatformSubtype = 0;

    if (!node)
    {
        pr_err("device tree node missing\n");
        goto Exit;
    }

    // Setup sysfs directory for telemetry
    BatteryProtection->BatteryKobj = kobject_create_and_add("battery", kernel_kobj);
    if (!BatteryProtection->BatteryKobj)
    {
        pr_err("MSE_battery_protection_init: failed to create battery parent obj");
        rc = -ENOMEM;
        kobject_put(BatteryProtection->BatteryKobj); //decrease reference count
        goto Exit;
    }

    // Add BatteryProtection sysfs nodes under battery sysfs
    rc = sysfs_create_group(BatteryProtection->BatteryKobj, &battery_protection_attr_group);
    if (rc)
    {
        pr_err("MSE_battery_protection_init: failed to add sysfs group for battery protection");
        rc = -EINVAL;
        kobject_del(BatteryProtection->BatteryKobj);
        goto Exit;
    }

    // Get battery protection related information from DTSI
    rc = GetBatteryProtectionInfoFromDtsi(node, BatteryProtection);
    if (rc < 0)
    {
        pr_err("GetBatteryProtectionInfoFromDtsi failed with rc = %d", rc);
        goto Exit;
    }

    BatteryProtection->IsDisableBPC = false;   // BPC will be enabled by default
    BatteryProtection->IsDisableBSC = false;   // BSC will be enabled by default
    BatteryProtection->BatteryLevel = -1;

    BatteryProtection->CombinedRsoc = INT_MIN;
    rc = MSEHwExtFgGetBatteryStatus(BatteryProtection);  // get and store battery temperature and RSOC from extfgs
    if (rc < 0)
    {
        pr_err("failed to get pack information from the ExtFgs rc = %d", rc);
        goto Exit;
    }

    rc = GetManufacturerBlockAFromExtFg(BatteryProtection);
    if (rc < 0)
    {
        pr_err("GetManufacturerBlockAFromExtFg failed with rc = %d", rc);
        goto Exit;
    }

    rc = GetHvtCountFromExtFg(BatteryProtection);
    if(rc < 0)
    {
        pr_err("GetHvtCountFromExtFg failed with rc = %d", rc);
        goto Exit;
    }

    // This is a WA due to old FGs not having the HVT COUNT bits zeroed out and will only be enabled for EV2 and lower builds
    PlatformSubtype =  socinfo_get_platform_subtype();
    if(PlatformSubtype <= EV2_BOARD_ID)
    {
        for(i = 0; i < BatteryProtection->NumPacks; i++)
        {
            if(BatteryProtection->BatteryProtectionBPCConfig[i].eqvTimeAtHvt_sec > MAX_HVT_COUNT_WA)
            {
                pr_err("Setting HVT COUNT for pack[%d] to 0x0 for first bootup as a WA for older FGs", i);
                BatteryProtection->BatteryProtectionBPCConfig[i].eqvTimeAtHvt_sec = 0x0;
                FlushHvtCountToExtFg(BatteryProtection, i);
            }
        }
    }

    rc = ManufacturingModeLevelingInit(BatteryProtection);
    if (rc < 0)
    {
        pr_err("ManufacturingModeLevelingInit failed with rc = %d", rc);
        goto Exit;
    }

    rc = BatteryTelemetryInit(BatteryProtection);
    if (rc < 0)
    {
        pr_err("BatteryTelemetryInit failed with rc = %d", rc);
        goto Exit;
    }

Exit:
	return rc;
}

void MSEBatteryProtectionShutdown(struct BATTERY_PROTECTION *BatteryProtection)
{
    int rc = 0, i;
    for(i = 0; i < BatteryProtection->NumPacks; i++)
    {
        rc = FlushHvtCountToExtFg(BatteryProtection, i);
        if(rc < 0)
            rc |=-ENODATA;
    }
    if(rc < 0)
    {
        pr_err("FlushHvtCountToExtFg failed with rc = %d in the MSE_battery_protection_shutdown", rc);
    }
    mutex_destroy(&ManufModeConfig.ManufModeLevelingLock);
    mutex_destroy(&BatteryTelemetry.BatteryTelemetryLock);
    xbl_hlos_hob_deinit();
}

/*************************************************
 *    MSE Battery Protection DebugFs Functions   *
 ************************************************/
static int force_pack2_bsc_hvtcount_read(void *data, u64 *val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    *val = BatteryProtection->BatteryProtectionBSCConfig[FG_002].eqvTimeAtHvt_sec;
    return 0;
}

static int force_pack2_bsc_hvtcount_write(void *data, u64 val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    BatteryProtection->BatteryProtectionBSCConfig[FG_002].eqvTimeAtHvt_sec = val;
    return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_pack2_bsc_hvtcount_ops, force_pack2_bsc_hvtcount_read,
			force_pack2_bsc_hvtcount_write, "%d\n");

static int force_pack1_bsc_hvtcount_read(void *data, u64 *val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    *val = BatteryProtection->BatteryProtectionBSCConfig[FG_001].eqvTimeAtHvt_sec;
    return 0;
}

static int force_pack1_bsc_hvtcount_write(void *data, u64 val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    BatteryProtection->BatteryProtectionBSCConfig[FG_001].eqvTimeAtHvt_sec = val;
    return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_pack1_bsc_hvtcount_ops, force_pack1_bsc_hvtcount_read,
			force_pack1_bsc_hvtcount_write, "%d\n");

static int force_pack2_bpc_hvtcount_read(void *data, u64 *val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    *val = BatteryProtection->BatteryProtectionBPCConfig[FG_002].eqvTimeAtHvt_sec;
    return 0;
}

static int force_pack2_bpc_hvtcount_write(void *data, u64 val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    BatteryProtection->BatteryProtectionBPCConfig[FG_002].eqvTimeAtHvt_sec = val;
    return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_pack2_bpc_hvtcount_ops, force_pack2_bpc_hvtcount_read,
			force_pack2_bpc_hvtcount_write, "%d\n");

static int force_pack1_bpc_hvtcount_read(void *data, u64 *val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    *val = BatteryProtection->BatteryProtectionBPCConfig[FG_001].eqvTimeAtHvt_sec;
    return 0;
}

static int force_pack1_bpc_hvtcount_write(void *data, u64 val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    BatteryProtection->BatteryProtectionBPCConfig[FG_001].eqvTimeAtHvt_sec = val;
    return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_pack1_bpc_hvtcount_ops, force_pack1_bpc_hvtcount_read,
			force_pack1_bpc_hvtcount_write, "%d\n");

static int force_disable_bpc_read(void *data, u64 *val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    *val = BatteryProtection->IsDisableBPC;
    return 0;
}

static int force_disable_bpc_write(void *data, u64 val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    BatteryProtection->IsDisableBPC = val;
    return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_disable_bpc_ops, force_disable_bpc_read,
			force_disable_bpc_write, "%d\n");

static int force_disable_bsc_read(void *data, u64 *val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    *val = BatteryProtection->IsDisableBSC;
    return 0;
}

static int force_disable_bsc_write(void *data, u64 val)
{
    struct BATTERY_PROTECTION *BatteryProtection = data;
    BatteryProtection->IsDisableBSC = val;
    return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_disable_bsc_ops, force_disable_bsc_read,
			force_disable_bsc_write, "%d\n");

void MSEBatteryProtectionDebugfs(struct dentry *parent, void *data)
{
    struct dentry *file;

    file = debugfs_create_file("force_disable_bpc", 0777,
                               parent, data, &force_disable_bpc_ops);
    if (IS_ERR_OR_NULL(file))
        pr_err("Couldn't create force_disable_bpc file rc=%ld\n",
               (long)file);

    file = debugfs_create_file("force_disable_bsc", 0777,
                               parent, data, &force_disable_bsc_ops);
    if (IS_ERR_OR_NULL(file))
        pr_err("Couldn't create force_disable_bsc file rc=%ld\n",
               (long)file);

    file = debugfs_create_file("force_pack1_bpc_hvtcount", 0777,
                               parent, data, &force_pack1_bpc_hvtcount_ops);
    if (IS_ERR_OR_NULL(file))
        pr_err("Couldn't create force_pack1_bpc_hvtcount file rc=%ld\n",
               (long)file);

    file = debugfs_create_file("force_pack2_bpc_hvtcount", 0777,
                               parent, data, &force_pack2_bpc_hvtcount_ops);
    if (IS_ERR_OR_NULL(file))
        pr_err("Couldn't create force_pack2_bpc_hvtcount file rc=%ld\n",
               (long)file);

    file = debugfs_create_file("force_pack1_bsc_hvtcount", 0777,
                               parent, data, &force_pack1_bsc_hvtcount_ops);
    if (IS_ERR_OR_NULL(file))
        pr_err("Couldn't create force_pack1_bsc_hvtcount file rc=%ld\n",
               (long)file);

    file = debugfs_create_file("force_pack2_bsc_hvtcount", 0777,
                               parent, data, &force_pack2_bsc_hvtcount_ops);
    if (IS_ERR_OR_NULL(file))
        pr_err("Couldn't create force_pack2_bsc_hvtcount file rc=%ld\n",
               (long)file);

}
