#pragma once

#include <analog_sensor.h>
#include <i2c_sensor.h>
#include <config.h>

// Example hooks; set to nullptr when unused.
constexpr BoardHooks kExampleHooks{
    nullptr,  // preSetup
    nullptr,  // beforeSleep
    nullptr   // afterWake
};

// Example grouped sensors table; add entries as real sensors are implemented.
constexpr I2CSensorContext kI2CSensor{
    .base = {
        .name = "WAFT Recv",
        .payloadSize = kI2cSensorPayloadSize,
    },
    .addr = 0x42,
    .clockHz = 400000,
};

constexpr AnalogSensorContext kFrontBrakePressureSensor{
    .base = {
        .name = "Front Brake Pressure",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = A0,
};

constexpr AnalogSensorContext kRearBrakePressureSensor{
    .base = {
        .name = "Rear Brake Pressure",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = A1,
};

constexpr SensorDescriptor kFastGroupSensors[] = {
    MakeI2CSensor(&kI2CSensor),
    MakeAnalogSensor(&kFrontBrakePressureSensor),
};

constexpr SensorDescriptor kSlowGroupSensors[] = {
    MakeAnalogSensor(&kRearBrakePressureSensor),
};

constexpr MessageGroupConfig kExampleGroups[] = {
    {
        .name = "Fast Sensors",
        .canId = 0x100,
        .pollIntervalMs = 10,
        .sensors = kFastGroupSensors,
        .sensorCount = sizeof(kFastGroupSensors) / sizeof(kFastGroupSensors[0]),
    },
    {
        .name = "Slow Sensors",
        .canId = 0x101,
        .pollIntervalMs = 20,
        .sensors = kSlowGroupSensors,
        .sensorCount = sizeof(kSlowGroupSensors) / sizeof(kSlowGroupSensors[0]),
    },
};

// Example board configuration demonstrating default CAN wiring and control IDs.
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
    kExampleGroups,
    sizeof(kExampleGroups) / sizeof(kExampleGroups[0]),
};
