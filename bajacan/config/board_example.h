#pragma once

#include <analog_sensor.h>
#include <as5600_sensor.h>
#include <bno085_sensor.h>
#include <config.h>
#include <mlx90614_sensor.h>
#include <pwm_rpm_sensor.h>

// Example hooks; set to nullptr when unused.
constexpr BoardHooks kExampleHooks{
    nullptr,  // preSetup
    nullptr,  // beforeSleep
    nullptr   // afterWake
};

inline AS5600SensorRuntime gExampleEncoderRuntime{};
inline BNO085SensorRuntime gExampleImuRuntime{};
inline MLX90614SensorRuntime gExampleIrSensorRuntime{};
inline PwmRpmSensorRuntime gExamplePwmAngleRuntime{};

// Example grouped sensors table; add entries as real sensors are implemented.

constexpr BNO085SensorContext kExampleIMU{
    .base = {
        .name = "IMU",
        .payloadSize = kBNO085SensorPayloadSize,
    },
    .runtime = &gExampleImuRuntime,
    .i2cAddress = 0x4A,
    .clockHz = 400000,
    .interruptPin = -1,
    .resetPin = -1,
    .reportIntervalMs = kBNO085DefaultReportIntervalMs,
};

// constexpr AS5600SensorContext kExampleEncoder{
//     .base = {
//         .name = "Steering Encoder",
//         .payloadSize = kAS5600SensorPayloadSize,
//     },
//     .runtime = &gExampleEncoderRuntime,
//     .clockHz = 400000,
//     .directionPin = AS5600_SW_DIRECTION_PIN,
//     .direction = AS5600_CLOCK_WISE,
//     .offsetCentiDegrees = 0,
// };

// constexpr MLX90614SubSensorContext kExampleIrTemperatureData{
//     .base = {
//         .name = "IR Temperature Data",
//         .payloadSize = kMLX90614DataSensorPayloadSize,
//     },
//     .runtime = &gExampleIrSensorRuntime,
//     .i2cAddress = MLX90614_I2CADDR,
//     .clockHz = 100000,
// };

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

// constexpr PwmRpmSubSensorContext kExamplePwmRpmDataSensor{
//     .base = {
//         .name = "AS5600 PWM Data",
//         .payloadSize = kPwmRpmDataSensorPayloadSize,
//     },
//     .runtime = &gExamplePwmAngleRuntime,
//     .pin = 2,
//     .timeoutMicros = kPwmRpmDefaultTimeoutMicros,
// };

// constexpr SensorDescriptor kFastGroupSensors[] = {
//     MakeAS5600Sensor(&kExampleEncoder),
//     // MakePwmRpmDataSensor(&kExamplePwmRpmDataSensor),
//     // MakeAnalogSensor(&kFrontBrakePressureSensor),
// };

constexpr SensorDescriptor kSlowGroupSensors[] = {
    MakeBNO085Sensor(&kExampleIMU),
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
        .pollIntervalMs = 500,
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
    kDefaultStartupDelayMs,
    kDefaultControlCommands,
    kExampleHooks,
    kExampleGroups,
    sizeof(kExampleGroups) / sizeof(kExampleGroups[0]),
};
