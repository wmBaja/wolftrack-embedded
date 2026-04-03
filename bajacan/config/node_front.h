#pragma once

#include <analog_sensor.h>
#include <i2c_sensor.h>
#include <as5600_sensor.h>
#include <config.h>

// Example hooks; set to nullptr when unused.
constexpr BoardHooks kExampleHooks{
    nullptr,  // preSetup
    nullptr,  // beforeSleep
    nullptr   // afterWake
};

inline AS5600SensorRuntime gSteeringEncRuntime{};

// First define sensors
constexpr AnalogSensorContext kFLSuspensionTravel{
    .base = {
        .name = "Front Left Suspension Travel",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = A0,
};

constexpr AnalogSensorContext kFRSuspensionTravel{
    .base = {
        .name = "Front Right Suspension Travel",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = A0,
};

constexpr AnalogSensorContext kLeftBrakePressureSensor{
    .base = {
        .name = "Front Brake Pressure",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = A2,
};

constexpr AnalogSensorContext kRightBrakePressureSensor{
    .base = {
        .name = "Rear Brake Pressure",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = A3,
};

constexpr AS5600SensorContext kSteeringAngleSensor{
    .base = {
        .name = "Steering Position",
        .payloadSize = kAS5600SensorPayloadSize,
    },
    .runtime = &gSteeringEncRuntime,
    .clockHz = 400000,
    .directionPin = AS5600_SW_DIRECTION_PIN, // No physical direction pin, controlled through software (next line)
    .direction = AS5600_CLOCK_WISE,          // Clockwise mag rotation causes increase in angle (wheel right)
    .offsetCentiDegrees = 0,                 // Offset based on mounting, ideally 0
};

// Group sensors into SensorDescriptor arrays
constexpr SensorDescriptor kFastGroupSensors[] = {
    MakeI2CSensor(&kI2CSensor),
    MakeAnalogSensor(&kFrontBrakePressureSensor),
};

constexpr SensorDescriptor kSlowGroupSensors[] = {
    MakeAnalogSensor(&kRearBrakePressureSensor),
};

// Create groups from SensorDescriptor arrays
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

// Default CAN wiring and control IDs.
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
