#pragma once

#include <analog_sensor.h>
#include <as5600_sensor.h>
#include <bno085_sensor.h>
#include <config.h>
#include <mlx90614_sensor.h>
#include <pwm_angle_sensor.h>
#include <Wire.h>

// Example hooks; set to nullptr when unused.
constexpr BoardHooks kExampleHooks{
    nullptr,  // preSetup
    nullptr,  // beforeSleep
    nullptr   // afterWake
};

// Runtime objects for sensors
inline AS5600SensorRuntime gSteeringAngleRuntime{&Wire1};
inline BNO085SensorRuntime gImuRuntime{};

// Grouped sensors table; add entries as real sensors are implemented.

constexpr BNO085SensorContext kIMU{
    .base = {
        .name = "IMU",
        .payloadSize = kBNO085SensorPayloadSize,
    },
    .runtime = &gImuRuntime,
    .i2cAddress = 0x4A,
    .clockHz = 400000,
    .interruptPin = -1,
    .resetPin = -1,
    .reportIntervalMs = kBNO085DefaultReportIntervalMs,
};

constexpr AS5600SensorContext kSteeringPos{
    .base = {
        .name = "Steering Angle Encoder",
        .payloadSize = kAS5600SensorPayloadSize,
    },
    .runtime = &gSteeringAngleRuntime,
    .clockHz = 400000,
    .directionPin = AS5600_SW_DIRECTION_PIN,
    .direction = AS5600_CLOCK_WISE,
    .offsetCentiDegrees = 0,
};

constexpr AnalogSensorContext kFrontBrakePressureSensor{
    .base = {
        .name = "Front Brake Pressure",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = PIN_PD7,
};

constexpr AnalogSensorContext kRearBrakePressureSensor{
    .base = {
        .name = "Rear Brake Pressure",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = PIN_PD4,
};

constexpr AnalogSensorContext kFLSuspensionSensor{
    .base = {
        .name = "Front Left Suspension Sensor",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = PIN_PD3,
};

constexpr AnalogSensorContext kFRSuspensionSensor{
    .base = {
        .name = "Front Right Suspension Sensor",
        .payloadSize = kAnalogSensorPayloadSize,
    },
    .pin = PIN_PD5,
};

constexpr SensorDescriptor kIMUSensorGroup[] = {
    MakeBNO085Sensor(&kIMU),
};

constexpr SensorDescriptor kFastGroupSensors[] = {
    MakeAS5600Sensor(&kSteeringPos),
    MakeAnalogSensor(&kFrontBrakePressureSensor),
    MakeAnalogSensor(&kRearBrakePressureSensor),
    MakeAnalogSensor(&kFLSuspensionSensor),
    MakeAnalogSensor(&kFRSuspensionSensor),
};

constexpr MessageGroupConfig kFrontGroups[] = {
    {
        .name = "Fast Sensors",
        .canId = 0x200,
        .pollIntervalMs = 10,
        .sensors = kFastGroupSensors,
        .sensorCount = sizeof(kFastGroupSensors) / sizeof(kFastGroupSensors[0]),
    },
    {
        .name = "IMU",
        .canId = 0x201,
        .pollIntervalMs = 50,
        .sensors = kIMUSensorGroup,
        .sensorCount = sizeof(kIMUSensorGroup) / sizeof(kIMUSensorGroup[0]),
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
    kFrontGroups,
    sizeof(kFrontGroups) / sizeof(kFrontGroups[0]),
};
