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
inline AS5600SensorRuntime gSteeringAngleRuntime{&Wire1};
inline BNO085SensorRuntime gImuRuntime{};

constexpr uint16_t kSteeringZPosition = 2831U; //Full lock right
constexpr uint16_t kSteeringMPosition = 752U; //Full lock left
constexpr uint16_t kImuDataCanPollIntervalMs = 10U;
constexpr float kSteeringMaxAngleDegrees =
    ((static_cast<float>(kAS5600CountsPerRevolution - kSteeringZPosition +
                         kSteeringMPosition) /
      static_cast<float>(kAS5600CountsPerRevolution)) *
     360.0f) /
    2.0f;
constexpr int16_t kSteeringMaxAngleCentiDegrees =
    static_cast<int16_t>(kSteeringMaxAngleDegrees * 100.0f + 0.5f);


constexpr BNO085SubSensorContext kIMUData{
    .base = {
        .name = "IMU Data",
        .payloadSize = kBNO085DataSensorPayloadSize,
    },
    .runtime = &gImuRuntime,
    .i2cAddress = 0x4A,
    .clockHz = 400000,
    .interruptPin = -1,
    .resetPin = -1,
    .reportIntervalMs = 10,
};

constexpr BNO085SubSensorContext kIMUStats{
    .base = {
        .name = "IMU Stats",
        .payloadSize = kBNO085StatsSensorPayloadSize,
    },
    .runtime = &gImuRuntime,
    .i2cAddress = 0x4A,
    .clockHz = 400000,
    .interruptPin = -1,
    .resetPin = -1,
    .reportIntervalMs = 10, // Underlying sensor runs at 1ms; CAN stats publish at 200ms.
};

constexpr AS5600SubSensorContext kSteeringPosData{
    .base = {
        .name = "Steering Angle Data",
        .payloadSize = kAS5600DataSensorPayloadSize,
    },
    .runtime = &gSteeringAngleRuntime,
    .clockHz = 400000,
    .directionPin = AS5600_SW_DIRECTION_PIN,
    .direction = AS5600_CLOCK_WISE,
    .offsetCentiDegrees = 0,
    .initializePositionWindow = true,
    .zPosition = kSteeringZPosition,
    .mPosition = kSteeringMPosition,
    .angleMapping = AS5600AngleMapping::CenteredWindow,
    .maxMappedAngleCentiDegrees = kSteeringMaxAngleCentiDegrees,
};

constexpr AS5600SubSensorContext kSteeringPosStats{
    .base = {
        .name = "Steering Angle Stats",
        .payloadSize = kAS5600StatsSensorPayloadSize,
    },
    .runtime = &gSteeringAngleRuntime,
    .clockHz = 400000,
    .directionPin = AS5600_SW_DIRECTION_PIN,
    .direction = AS5600_CLOCK_WISE,
    .offsetCentiDegrees = 0,
    .initializePositionWindow = true,
    .zPosition = kSteeringZPosition,
    .mPosition = kSteeringMPosition,
    .angleMapping = AS5600AngleMapping::CenteredWindow,
    .maxMappedAngleCentiDegrees = kSteeringMaxAngleCentiDegrees,
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


// Sensor Groups
constexpr SensorDescriptor kIMUDataGroup[] = {
    MakeBNO085DataSensor(&kIMUData),
};

constexpr SensorDescriptor kIMUStatsGroup[] = {
    MakeBNO085StatsSensor(&kIMUStats),
};

constexpr SensorDescriptor kSteeringDataGroup[] = {
    MakeAS5600DataSensor(&kSteeringPosData),
};

constexpr SensorDescriptor kSteeringStatsGroup[] = {
    MakeAS5600StatsSensor(&kSteeringPosStats),
};

constexpr SensorDescriptor kBrakePressureGroup[] = {
    MakeAnalogSensor(&kFrontBrakePressureSensor),
    MakeAnalogSensor(&kRearBrakePressureSensor),
};

//constexpr SensorDescriptor kSuspensionGroup[] = {
//    MakeAnalogSensor(&kFLSuspensionSensor),
//    MakeAnalogSensor(&kFRSuspensionSensor),
//};


// List of groups (to give to board config)
constexpr MessageGroupConfig kFrontGroups[] = {
    {
        .name = "Steering Angle Data",
        .canId = 0x200,
        .pollIntervalMs = 20,
        .sensors = kSteeringDataGroup,
        .sensorCount = sizeof(kSteeringDataGroup) / sizeof(kSteeringDataGroup[0]),
    },
    {
        .name = "Steering Angle Stats",
        .canId = 0x201,
        .pollIntervalMs = 200,
        .sensors = kSteeringStatsGroup,
        .sensorCount = sizeof(kSteeringStatsGroup) / sizeof(kSteeringStatsGroup[0]),
    },
    {
       .name = "Brake Pressure",
       .canId = 0x202,
       .pollIntervalMs = 50,
       .sensors = kBrakePressureGroup,
       .sensorCount = sizeof(kBrakePressureGroup) / sizeof(kBrakePressureGroup[0]),
    },
    // Suspension group intentionally disabled because those sensors are not wired.
    // {
    //     .name = "Suspension",
    //     .canId = 0x203,
    //     .pollIntervalMs = 1,
    //     .sensors = kSuspensionGroup,
    //     .sensorCount = sizeof(kSuspensionGroup) / sizeof(kSuspensionGroup[0]),
    // },
    {
        .name = "IMU_Data",
        .canId = 0x204,
        .pollIntervalMs = kImuDataCanPollIntervalMs,
        .sensors = kIMUDataGroup,
        .sensorCount = sizeof(kIMUDataGroup) / sizeof(kIMUDataGroup[0]),
    },
    {
        .name = "IMU_Stats",
        .canId = 0x205,
        .pollIntervalMs = 200,
        .sensors = kIMUStatsGroup,
        .sensorCount = sizeof(kIMUStatsGroup) / sizeof(kIMUStatsGroup[0]),
    },
};


// Board Config (last two fields use list of sensor groups)
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
    kFrontGroups,
    sizeof(kFrontGroups) / sizeof(kFrontGroups[0]),
};
