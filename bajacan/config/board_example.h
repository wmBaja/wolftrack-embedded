#pragma once

#include <analog_sensor.h>
#include <as5600_sensor.h>
#include <config.h>
#include <mlx90614_sensor.h>

// Example hooks; set to nullptr when unused.
constexpr BoardHooks kExampleHooks{
    nullptr,  // preSetup
    nullptr,  // beforeSleep
    nullptr   // afterWake
};

inline AS5600SensorRuntime gExampleEncoderRuntime{};
inline MLX90614SensorRuntime gExampleIrSensorRuntime{};

// Example grouped sensors table; add entries as real sensors are implemented.
constexpr AS5600SensorContext kExampleEncoder{
    .base = {
        .name = "Steering Encoder",
        .payloadSize = kAS5600SensorPayloadSize,
    },
    .runtime = &gExampleEncoderRuntime,
    .clockHz = 400000,
    .directionPin = AS5600_SW_DIRECTION_PIN,
    .direction = AS5600_CLOCK_WISE,
    .offsetCentiDegrees = 0,
};

constexpr MLX90614SensorContext kExampleIrTemperatureSensor{
    .base = {
        .name = "IR Temperature",
        .payloadSize = kMLX90614SensorPayloadSize,
    },
    .runtime = &gExampleIrSensorRuntime,
    .i2cAddress = MLX90614_I2CADDR,
    .clockHz = 100000,
};

// constexpr AnalogSensorContext kFrontBrakePressureSensor{
//     .base = {
//         .name = "Front Brake Pressure",
//         .payloadSize = kAnalogSensorPayloadSize,
//     },
//     .pin = A0,
// };

// constexpr AnalogSensorContext kRearBrakePressureSensor{
//     .base = {
//         .name = "Rear Brake Pressure",
//         .payloadSize = kAnalogSensorPayloadSize,
//     },
//     .pin = A1,
// };

// constexpr SensorDescriptor kFastGroupSensors[] = {
//     MakeAS5600Sensor(&kExampleEncoder),
//     // MakeAnalogSensor(&kFrontBrakePressureSensor),
// };

constexpr SensorDescriptor kSlowGroupSensors[] = {
    MakeMLX90614Sensor(&kExampleIrTemperatureSensor),
    // MakeAnalogSensor(&kRearBrakePressureSensor),
};

constexpr MessageGroupConfig kExampleGroups[] = {
    // {
    //     .name = "Fast Sensors",
    //     .canId = 0x100,
    //     .pollIntervalMs = 10,
    //     .sensors = kFastGroupSensors,
    //     .sensorCount = sizeof(kFastGroupSensors) / sizeof(kFastGroupSensors[0]),
    // },
    {
        .name = "Slow Sensors",
        .canId = 0x101,
        .pollIntervalMs = 100,
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
