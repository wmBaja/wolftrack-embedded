#pragma once

#include <analog_rpm_sensor.h>
#include <analog_sensor.h>
#include <as5600_sensor.h>
#include <bno085_sensor.h>
#include <config.h>
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

constexpr MLX90614SensorContext kIRSensorCVT{
    .base = {
        .name = "IR CVT Temp Sensor",
        .payloadSize = kMLX90614SensorPayloadSize,
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
    MakeMLX90614Sensor(&kIRSensorCVT),
};

constexpr SensorDescriptor kWheelSpeedRpmDataGroup[] = {
    MakeAnalogRpmDataSensor(&kWheelSpeedRpmSensorData),
};

constexpr SensorDescriptor kWheelSpeedRpmStatsGroup[] = {
    MakeAnalogRpmStatsSensor(&kWheelSpeedRpmSensorStats),
};

constexpr SensorDescriptor kEngineRpmDataGroup[] = {
    MakePulseRpmDataSensor(&kEngineRpmSensorData),
};

constexpr SensorDescriptor kEngineRpmStatsGroup[] = {
    MakePulseRpmStatsSensor(&kEngineRpmSensorStats),
};

constexpr SensorDescriptor kFastGroupSensors[] = {
    MakeAnalogSensor(&kRLSuspensionSensor),
    MakeAnalogSensor(&kRRSuspensionSensor),
    MakeAnalogSensor(&kThrottlePosSensor),
};

constexpr MessageGroupConfig kRearGroups[] = {
    {
        .name = "Fast Sensors",
        .canId = 0x100,
        .pollIntervalMs = 10,
        .sensors = kFastGroupSensors,
        .sensorCount = sizeof(kFastGroupSensors) / sizeof(kFastGroupSensors[0]),
    },
    {
        .name = "IR CVT Temp",
        .canId = 0x101,
        .pollIntervalMs = 50,
        .sensors = kIRSensorGroup,
        .sensorCount = sizeof(kIRSensorGroup) / sizeof(kIRSensorGroup[0]),
    },
    {
        .name = "Wheel Speed RPM Data",
        .canId = 0x102,
        .pollIntervalMs = 10,
        .sensors = kWheelSpeedRpmDataGroup,
        .sensorCount =
            sizeof(kWheelSpeedRpmDataGroup) /
            sizeof(kWheelSpeedRpmDataGroup[0]),
    },
    {
        .name = "Wheel Speed RPM Stats",
        .canId = 0x103,
        .pollIntervalMs = 100,
        .sensors = kWheelSpeedRpmStatsGroup,
        .sensorCount =
            sizeof(kWheelSpeedRpmStatsGroup) /
            sizeof(kWheelSpeedRpmStatsGroup[0]),
    },
    {
        .name = "Engine RPM Data",
        .canId = 0x104,
        .pollIntervalMs = 10,
        .sensors = kEngineRpmDataGroup,
        .sensorCount =
            sizeof(kEngineRpmDataGroup) / sizeof(kEngineRpmDataGroup[0]),
    },
    {
        .name = "Engine RPM Stats",
        .canId = 0x105,
        .pollIntervalMs = 100,
        .sensors = kEngineRpmStatsGroup,
        .sensorCount =
            sizeof(kEngineRpmStatsGroup) / sizeof(kEngineRpmStatsGroup[0]),
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
    kDefaultControlCommands,
    kExampleHooks,
    kRearGroups,
    sizeof(kRearGroups) / sizeof(kRearGroups[0]),
};
