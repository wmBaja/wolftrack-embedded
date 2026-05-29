#pragma once

#include <analog_rpm_sensor.h>
#include <analog_sensor.h>
#include <as5600_sensor.h>
#include <bno085_sensor.h>
#include <config.h>
#include <i2c_sensor.h>
#include <mlx90614_sensor.h>
#include <pulse_rpm_sensor.h>
#include <Wire.h>

// Example hooks; set to nullptr when unused.
constexpr BoardHooks kExampleHooks{
    nullptr,  // preSetup
    nullptr,  // beforeSleep
    nullptr   // afterWake
};

// Runtime objects for sensors

inline MLX90614SensorRuntime gIRRuntime{&Wire1, PIN_WIRE1_SDA, PIN_WIRE1_SCL};
inline AnalogRpmSensorRuntime gWheelSpeedRpmRuntime{};
inline PulseRpmSensorRuntime gEngineRpmRuntime{};

// Grouped sensors table; add entries as real sensors are implemented.

constexpr uint8_t kEsp32AdcI2cAddress = 0x50U;
constexpr uint16_t kEsp32AdcPollIntervalMs = 5U;

// I2CSensor uses the default Wire bus, matching the front node IMU wiring.
// Keep the ESP32 ADC bridge off Wire1; Wire1 is reserved here for the IR sensor.
constexpr I2CSensorContext kEsp32AdcSensor{
    .base = {
        .name = "ADS CH1",
        .payloadSize = kI2cSensorPayloadSize,
    },
    .addr = kEsp32AdcI2cAddress,
    .clockHz = 100000,
};

constexpr AnalogRpmSubSensorContext kWheelSpeedRpmSensorData{
    .base = {
        .name = "Wheel Speed RPM Data",
        .payloadSize = kAnalogRpmDataSensorPayloadSize,
    },
    .runtime = &gWheelSpeedRpmRuntime,
    .pin = PIN_PD4,
    .zeroDeltaDeadbandCounts = 24U,
    .motionConfirmSamples = 2U,
    .zeroConfirmSamples = 2U,
};

constexpr AnalogRpmSubSensorContext kWheelSpeedRpmSensorStats{
    .base = {
        .name = "Wheel Speed RPM Stats",
        .payloadSize = kAnalogRpmStatsSensorPayloadSize,
    },
    .runtime = &gWheelSpeedRpmRuntime,
    .pin = PIN_PD4,
    .zeroDeltaDeadbandCounts = 24U,
    .motionConfirmSamples = 2U,
    .zeroConfirmSamples = 2U,
};

constexpr PulseRpmSubSensorContext kEngineRpmSensorData{
    .base = {
        .name = "Engine RPM Data",
        .payloadSize = kPulseRpmDataSensorPayloadSize,
    },
    .runtime = &gEngineRpmRuntime,
    .pin = PIN_PF4,
    .milliRevolutionsPerPulse = kFourStrokeEnginePulsesPerRevolution,
    .staleAfterMicros = 300000U,
    .minPulseSpacingMicros = 5000U,
    .countRisingEdge = false,
};

constexpr PulseRpmSubSensorContext kEngineRpmSensorStats{
    .base = {
        .name = "Engine RPM Stats",
        .payloadSize = kPulseRpmStatsSensorPayloadSize,
    },
    .runtime = &gEngineRpmRuntime,
    .pin = PIN_PF4,
    .milliRevolutionsPerPulse = kFourStrokeEnginePulsesPerRevolution,
    .staleAfterMicros = 300000U,
    .minPulseSpacingMicros = 5000U,
    .countRisingEdge = false,
};

constexpr MLX90614SubSensorContext kIRSensorCVTData{
    .base = {
        .name = "IR CVT Temp Data",
        .payloadSize = kMLX90614DataSensorPayloadSize,
    },
    .runtime = &gIRRuntime,
    .i2cAddress = MLX90614_I2CADDR,
    .clockHz = 115000,
};

constexpr MLX90614SubSensorContext kIRSensorCVTStats{
    .base = {
        .name = "IR CVT Temp Stats",
        .payloadSize = kMLX90614StatsSensorPayloadSize,
    },
    .runtime = &gIRRuntime,
    .i2cAddress = MLX90614_I2CADDR,
    .clockHz = 115000,
};

constexpr AnalogSensorContext kRLSuspensionSensor{
    .base = {
        .name = "Rear Left Suspension Sensor",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = PIN_PD5,
};

constexpr AnalogSensorContext kRRSuspensionSensor{
    .base = {
        .name = "Rear Right Suspension Sensor",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = PIN_PD3,
};

constexpr AnalogSensorContext kThrottlePosSensor{
    .base = {
        .name = "Throttle Position Sensor",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = PIN_PD7,
};

constexpr SensorDescriptor kIRSensorGroup[] = {
    MakeMLX90614DataSensor(&kIRSensorCVTData),
};

constexpr SensorDescriptor kIRSensorStatsGroup[] = {
    MakeMLX90614StatsSensor(&kIRSensorCVTStats),
};

constexpr SensorDescriptor kRpmDataGroup[] = {
    MakeAnalogRpmDataSensor(&kWheelSpeedRpmSensorData),
    MakePulseRpmDataSensor(&kEngineRpmSensorData),
};

constexpr SensorDescriptor kRpmStatsGroup[] = {
    MakeAnalogRpmStatsSensor(&kWheelSpeedRpmSensorStats),
    MakePulseRpmStatsSensor(&kEngineRpmSensorStats),
};

constexpr SensorDescriptor kEsp32AdcGroup[] = {
    MakeI2CSensor(&kEsp32AdcSensor),
};

// constexpr SensorDescriptor kSuspensionGroup[] = {
//    MakeAnalogSensor(&kRLSuspensionSensor),
//    MakeAnalogSensor(&kRRSuspensionSensor),
// };

// constexpr SensorDescriptor kThrottlePosGroup[] = {
//    MakeAnalogSensor(&kThrottlePosSensor),
// };

constexpr MessageGroupConfig kRearGroups[] = {
    {
        .name = "ADS_CH1",
        .canId = 0x106,
        .pollIntervalMs = kEsp32AdcPollIntervalMs,
        .sensors = kEsp32AdcGroup,
        .sensorCount = sizeof(kEsp32AdcGroup) / sizeof(kEsp32AdcGroup[0]),
        .useExtendedId = kDefaultUseExtendedIds,
    },
    // Suspension group intentionally disabled because those sensors are not wired.
    // {
    //     .name = "Suspension",
    //     .canId = 0x100,
    //     .pollIntervalMs = 1,
    //     .sensors = kSuspensionGroup,
    //     .sensorCount = sizeof(kSuspensionGroup) / sizeof(kSuspensionGroup[0]),
    // },
    
    // {
    //    .name = "Throttle Position",
    //    .canId = 0x101,
    //    .pollIntervalMs = 20,
    //    .sensors = kThrottlePosGroup,
    //    .sensorCount = sizeof(kThrottlePosGroup) / sizeof(kThrottlePosGroup[0]),
    //},
    {
        .name = "IR CVT Temp Data",
        .canId = 0x102,
        .pollIntervalMs = 50,
        .sensors = kIRSensorGroup,
        .sensorCount = sizeof(kIRSensorGroup) / sizeof(kIRSensorGroup[0]),
        .useExtendedId = kDefaultUseExtendedIds,
    },
    {
        .name = "IR CVT Temp Stats",
        .canId = 0x103,
        .pollIntervalMs = 200,
        .sensors = kIRSensorStatsGroup,
        .sensorCount = sizeof(kIRSensorStatsGroup) / sizeof(kIRSensorStatsGroup[0]),
        .useExtendedId = kDefaultUseExtendedIds,
    },
    {
        .name = "RPM Data",
        .canId = 0x104,
        .pollIntervalMs = 10,
        .sensors = kRpmDataGroup,
        .sensorCount =
            sizeof(kRpmDataGroup) /
            sizeof(kRpmDataGroup[0]),
        .useExtendedId = kDefaultUseExtendedIds,
    },
    {
        .name = "RPM Stats",
        .canId = 0x105,
        .pollIntervalMs = 200,
        .sensors = kRpmStatsGroup,
        .sensorCount =
            sizeof(kRpmStatsGroup) /
            sizeof(kRpmStatsGroup[0]),
        .useExtendedId = kDefaultUseExtendedIds,
    },
};

// Board Config
constexpr BoardConfig kBoardConfig{
    kDefaultCanCsPin,
    kDefaultCanIntPin,
    kDefaultCanStbyPin,
    kDefaultMcpOscillator,
    kDefaultArbitrationBitrate,
    kDefaultDataBitrateFactor,
    kDefaultUseExtendedIds,
    kDefaultStartupDelayMs,
    kDefaultControlCommands,
    kExampleHooks,
    kRearGroups,
    sizeof(kRearGroups) / sizeof(kRearGroups[0]),
};
