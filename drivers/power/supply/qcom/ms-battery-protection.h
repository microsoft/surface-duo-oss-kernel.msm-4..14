/*
 * ms-battery-protection.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __MSEBATTERYPROTECTION_H
#define __MSEBATTERYPROTECTION_H

#include <linux/types.h>
#include <linux/regulator/driver.h>

#define BATTERY_TELEMETRY_VERSION  (0x00010000)  // Major Version: Upper 2 bytes, Minor Version: Lower 2 bytes

#define SECONDS_PER_MINUTE         (60U)
#define CONVERT_SEC_TO_MIN(sec)    ((sec) / SECONDS_PER_MINUTE)
#define CONVERT_MIN_TO_SEC(min)    ((min) * SECONDS_PER_MINUTE)

#define MINUTES_PER_HOUR           (60U)
#define CONVERT_MIN_TO_HR(min)     ((min) / MINUTES_PER_HOUR)
#define CONVERT_HR_TO_MIN(hrs)     ((hrs) * MINUTES_PER_HOUR)

#define SECONDS_PER_HOUR           (MINUTES_PER_HOUR * SECONDS_PER_MINUTE)
#define CONVERT_SEC_TO_HR(sec)     ((sec) / SECONDS_PER_HOUR )
#define CONVERT_HR_TO_SEC(hrs)     ((hrs) * SECONDS_PER_HOUR)

#define HOURS_IN_A_DAY             (24U)
#define CONVERT_DAYS_TO_SECS(days) ((days) * HOURS_IN_A_DAY * SECONDS_PER_HOUR)
#define CONVERT_DAYS_TO_MINS(days) ((days) * HOURS_IN_A_DAY * MINUTES_PER_HOUR)
#define CONVERT_MINS_TO_DAYS(min)  ((min)  / (HOURS_IN_A_DAY * MINUTES_PER_HOUR))
#define CONVERT_SECS_TO_DAYS(sec)  ((sec)  / (HOURS_IN_A_DAY * SECONDS_PER_HOUR))

#define SECONDS_PER_DAY            CONVERT_DAYS_TO_SECS(1)

#define MSE_BATTERYPROTECTION_VOTER		    "MSE_BATTERYPROTECTION_VOTER"

#define HVT_AF_SCALING_FACTOR               (100U)
#define HVT_TIME_ACCUMULATION_RSOC          (95U)   // RSOC above which HVT time will be accumulated; In %
#define HVT_TIME_ACCUMULATION_RSOC_BSC      (85U)   // RSOC above which BPCEqvTimeat35CinBSC will be accumulated; In %
// Frequency at which HVTCOUNT will be flushed to extfgs
#define BPC_EXTFG_DATA_FLUSH_INTERVAL_SEC   (7200U) // In seconds; 120 mins

#define MAX_ALLOWED_TEMPERATURE             2000 //deciC

// Supplier Names
#define SUPPLIER_NAME_ONE	"ATL"
#define SUPPLIER_NAME_TWO	"COS"

#define MFG_NAME_SIZE                       0x3 // 3 bytes allotted for manufacturer name

#define HVT_RSOC_BUCKETS                    (5U) // hvt temperature buckets for telemetry

#define MANUF_MODE_LEVELING_RSOC            (66U)  // default upper and lower limits for manuf mode leveling

enum Accel_Factor {
    ACCEL_FACTOR_25C        = 6,
    ACCEL_FACTOR_30C        = 30,
    ACCEL_FACTOR_35C        = 100,
    ACCEL_FACTOR_40C        = 200,
    ACCEL_FACTOR_45C        = 400,
    ACCEL_FACTOR_50C        = 600,
    ACCEL_FACTOR_55C        = 1100,
    ACCEL_FACTOR_ABOVE55C   = 2500,
    ACCEL_FACTOR_MAX        = 8
};

typedef enum {
    FG_001,   // Instance 01 of FG
    FG_002,   // Instance 02 of FG
    MAX_NUM_FG_PACKS
} FG_INSTANCE;

enum Batt_Supplier_Type
{
    BAT_SUPPLIER_INVALID,
    BAT_SUPPLIER_1,
    BAT_SUPPLIER_2,
    BAT_SUPPLIER_MAX
};

struct Hvt_AccelFactor_Table
{
    int16_t HvtTemperatureUth;                               // Upper threshold for temperature band (deci C)
    enum Accel_Factor AccelFactor;                           // Acceleration factor based on temperature band
};

typedef struct BATTERY_PROTECTION_ELAPSED_TIME
{
    uint16_t hvtAF;                        // hvtAccelerationFactor
    uint32_t elapsedTime_sec;              // time elapsed since battery task's last ran
    uint32_t hvtElapsedTime_sec;           // [elapsedTime_sec * hvtAF] / HVT_AF_SCALING_FACTOR
    uint32_t partialHvtElapsedTime_sec;    // [hvtElapsedTime % HVT_AF_SCALING_FACTOR]
} *PBATTERY_PROTECTION_ELAPSED_TIME;

typedef struct STG_BPC_NVM
{
    enum Batt_Supplier_Type BattSupplierType;
    uint32_t eqvTimeAtHvt_sec;
    uint32_t eqvTimeAtHvtInBsc_sec;

    // Min RSOC Telemetry
    int16_t               bpcProtection;           // battery protection
    uint32_t              lastMinSOCDay;           // day that the MinSOC vectors were last updated
    uint8_t               minSOC28day[28];         // MinSOC28day. Rolling 28 day minimum SOC
    uint8_t               minSOC28dayIndex;
    uint8_t               minSOC7day[7];           // MinSOC7day. Rolling 7 day minimum SOC
    uint8_t               minSOC7dayIndex;
    uint8_t               minSOC3day[3];           // MinSOC3day. Rolling 3 day minimum SOC
    uint8_t               minSOC3dayIndex;
    uint8_t               minSOC1day;              // MinSOC1day. Rolling 1 day minimum SOC
} *PSTG_BPC_NVM;

typedef struct BSC_CONFIG
{
    uint32_t eqvTimeAtHvt_sec;
    uint32_t bscStartDate;                          // BSC statistics start time
    uint32_t bscTotalActive_sec;                    // total time in bsc

    // Temperature/Rsoc Buckets for HVT telemetry
    uint32_t hvtLowTempRsocBuckets_sec[HVT_RSOC_BUCKETS];
    uint32_t hvtMediumLowTempRsocBuckets_sec[HVT_RSOC_BUCKETS];
    uint32_t hvtMediumHighTempRsocBuckets_sec[HVT_RSOC_BUCKETS];
    uint32_t hvtHighTempRsocBuckets_sec[HVT_RSOC_BUCKETS];
} *PBSC_CONFIG;

typedef struct BSC_STATIC_CONFIG
{
    int32_t HvtTempBucketLow;
    int32_t HvtTempBucketMid;
    int32_t HvtTempBucketHigh;
    uint32_t HvtRsocBucketThresholds[HVT_RSOC_BUCKETS];
} *PBSC_STATIC_CONFIG;

typedef struct BATT_STATUS
{
    uint16_t monotonic_soc; //soc applied with smoothing
    int32_t  temperature;
} *PBATT_STATUS;

struct BATTERY_PROTECTION {
    struct kobject *BatteryKobj;
    bool IsDisableBPC;
    bool IsDisableBSC;
    bool IsDisableCharging;
    uint32_t NumPacks;
    uint32_t FloatVoltageMax;
    int CombinedRsoc;
    int BatteryLevel;
    struct power_supply	*MSEExtFg[MAX_NUM_FG_PACKS];
    struct power_supply	*bms;
    struct BATT_STATUS MSEBatteryStatus[MAX_NUM_FG_PACKS];
    struct BATTERY_PROTECTION_ELAPSED_TIME BatteryProtectionElapsedTime[MAX_NUM_FG_PACKS];
    struct STG_BPC_NVM BatteryProtectionBPCConfig[MAX_NUM_FG_PACKS];
    struct BSC_STATIC_CONFIG BatteryProtectionBSCStaticConfig;
    struct BSC_CONFIG BatteryProtectionBSCConfig[MAX_NUM_FG_PACKS];
};

typedef struct MANUF_MODE_CONFIG
{
    bool InManufacturingMode;
    uint8_t UthManufModeRsoc;
    uint8_t LthManufModeRsoc;
    struct mutex ManufModeLevelingLock;
} *PMANUF_MODE_CONFIG;

// Battery Telemetry

#pragma pack(1)
typedef struct FG_TELEMETRY
{
    uint16_t CycleCount;
    uint8_t StateOfHealth;
    uint16_t DesignCapacity;
    int16_t ChargingVoltage;
    int16_t LifetimeMaxTemp;
    int16_t LifetimeMinTemp;
    int16_t LifetimeMaxPackVoltage;
    int16_t LifetimeMinPackVoltage;
    int16_t LifetimeMaxChargeCurrent;
    int16_t LifetimeMaxDischargeCurrent;
    uint16_t LTFlashCount;
    uint8_t LTAFEStatus;
    int16_t QmaxCell0;
    int16_t VoltageAtChargeTerm;
    int16_t AverageILastRun;
    int16_t AveragePLastRun;
    int16_t DeltaVoltage;
} *PFG_TELEMETRY;

typedef struct BPC_TELEMETRY
{
    uint8_t   BPCMinSOC28day;                                             // U1   0..100          0       %
    uint8_t   BPCMinSOC7day;                                              // U1   0..100          0       %
    uint8_t   BPCMinSOC3day;                                              // U1   0..100          0       %
    uint8_t   BPCMinSOC1day;                                              // U1   0..100          0       %
    uint16_t  BPCEqvTimeat35C;                                            // U2   0..65535        0       hours
    uint16_t  BPCEqvTimeat35CinBSC;                                       // U2   0..65535        0       hours
    uint8_t   BPCProtection;                                              // U1   0..100          0
} *PBPC_TELEMETRY;

typedef struct BSC_TELEMETRY
{
    uint32_t  BSCStartDateInSeconds;                                    // U4   0..4294967295   n/a     sec (UTC)
    uint32_t  BSCTotalMinutesFromStart;                                 // U4   0..4294967295   0       min
    uint32_t  BSCTotalEngagedMinutes;                                   // U4   0..4294967295   0       min
    uint32_t  BSCTotalEntryEvents;                                      // U4   0..4294967295   0       events
    uint8_t   BSCCurrentlyEngaged;                                      // H1   00..01          00      bitfield
    uint16_t  BSCRsocBucketsLowTemp[HVT_RSOC_BUCKETS];                  // U2   0..65535        0       hours
    uint16_t  BSCRsocBucketsMediumLowTemp[HVT_RSOC_BUCKETS];            // U2   0..65535        0       hours
    uint16_t  BSCRsocBucketsMediumHighTemp[HVT_RSOC_BUCKETS];           // U2   0..65535        0       hours
    uint16_t  BSCRsocBucketsHighTemp[HVT_RSOC_BUCKETS];                 // U2   0..65535        0       hours
    uint8_t   BSCExitCriteria;                                          // U1   0..100          0       %
    uint32_t  BSCHvtcount;                                              // U4   0..4294967295   n/a     sec (UTC)
} *PBSC_TELEMETRY;

typedef struct GENERAL_TELEMETRY
{
    uint8_t   InAutoRechargePhase;
    uint16_t  MonotonicSoc;
    uint32_t  VBatt;
    int32_t   IBatt;
    int32_t   Temperature;
    uint8_t   IsWeakPSU;
    uint8_t   CFETOn;
    uint8_t   PackDisconnect;
} *PGENERAL_TELEMETRY;

typedef struct BATTERY_TELEMETRY_BINARY
{
    uint32_t BatteryTelemetryVersion;                                   // U4   Major Version: Upper 2 bytes, Minor Version: Lower 2 bytes
    struct FG_TELEMETRY FgTelemetry;
    struct BPC_TELEMETRY BpcTelemetry;
    struct BSC_TELEMETRY BscTelemetry;
    struct GENERAL_TELEMETRY GeneralTelemetry;
} *PBATTERY_TELEMETRY_BINARY;

typedef struct BSC_DIRECTIVES
{
    uint8_t BpcProtection;
    uint8_t IsDischarging;
    uint8_t IsDisableCharging;
    uint8_t BscHvtcountReset;
    uint8_t InActiveBsc;
    uint8_t BSCExitCriteria;
    uint32_t BSCTotalEntryEvents;
    uint8_t InAutoRechargeRsocPhase;
    int8_t BatteryLevel;
} *PBSC_DIRECTIVES;

typedef struct BATTERY_PROTECTION_BINARY
{
    int32_t Pack1Temperature;
    uint32_t Pack1Hvtcount;
    int32_t Pack2Temperature;
    uint32_t Pack2Hvtcount;
    int CombinedRsoc;
    uint32_t SCPack1Hvtcount;
    uint32_t SCPack2Hvtcount;
    uint8_t InAutoRecharge;
    uint8_t InDischargePhase;
    struct BSC_DIRECTIVES BscDirectives;
} *PBATTERY_PROTECTION_BINARY;
#pragma pack()

typedef struct BATTERY_TELEMETRY
{
    bool IsBatteryTelemetryPresent[MAX_NUM_FG_PACKS];
    bool FileRestore[MAX_NUM_FG_PACKS];
    bool IsFirstBoot[MAX_NUM_FG_PACKS];
    bool FileRestoreOptin;
    struct mutex BatteryTelemetryLock;
    struct BATTERY_PROTECTION_BINARY BatteryProtectionBinary;
    struct BATTERY_TELEMETRY_BINARY BatteryTelemetryBinary[MAX_NUM_FG_PACKS];
} *PBATTERY_TELEMETRY;

/******************************************************
***** MSE Battery Protection Function Definitions  ****
******************************************************/
int MSEBatteryProtectionInit(struct device_node *node, struct BATTERY_PROTECTION *BatteryProtection);
void MSEBatteryProtectionShutdown(struct BATTERY_PROTECTION *BatteryProtection);
void MSEBatteryProtectionDebugfs(struct dentry *parent, void *data);
int MSEBatteryProtectionPeriodic(struct BATTERY_PROTECTION *BatteryProtection, bool InDischargePhase, bool *IsForcedDischarging);
void BatteryTelemetryGeneratePayloadGeneral(void *MSEHw_Config_Input);

#endif /* __MSEBATTERYPROTECTION_H */
