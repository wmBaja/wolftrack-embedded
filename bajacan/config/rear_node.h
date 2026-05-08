#pragma once

#include <analog_sensor.h>
#include <as5600_sensor.h>
#include <bno085_sensor.h>
#include <config.h>
#include <mlx90614_sensor.h>
#include <Wire.h>

// Example hooks; set to nullptr when unused.
constexpr BoardHooks kExampleHooks{
    nullptr,  // preSetup
    nullptr,  // beforeSleep
    nullptr   // afterWake
};

// Runtime objects for sensors

inline MLX90614SensorRuntime gIRRuntime{&Wire1};

// Grouped sensors table; add entries as real sensors are implemented.

constexpr AnalogSensorContext kWheelSpeedRawSensor{
    .base = {
        .name = "Wheel Speed Raw",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = PIN_PD4,
};

constexpr MLX90614SensorContext kIRSensorCVT{
    .base = {
        .name = "IR CVT Temp Sensor",
        .payloadSize = kMLX90614SensorPayloadSize,
    },
    .runtime = &gIRRuntime,
    .i2cAddress = MLX90614_I2CADDR,
    .clockHz = 100000,
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

constexpr SensorDescriptor kWheelSpeedRawGroup[] = {
    MakeAnalogSensor(&kWheelSpeedRawSensor),
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
        .name = "Wheel Speed Raw",
        .canId = 0x102,
        .pollIntervalMs = 10,
        .sensors = kWheelSpeedRawGroup,
        .sensorCount = sizeof(kWheelSpeedRawGroup) / sizeof(kWheelSpeedRawGroup[0]),
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
