#pragma once

#include <analog_sensor.h>
#include <config.h>

// Example hooks; set to nullptr when unused.
constexpr BoardHooks kExampleHooks{
    nullptr,  // preSetup
    nullptr,  // beforeSleep
    nullptr   // afterWake
};

// Example sensors table; add entries as real sensors are implemented.
constexpr AnalogSensorContext kExampleAnalog0{
    .base =
        {
            .name = "AnalogRaw0",
            .canId = 0x300,
            .pollIntervalMs = 10,
        },
    .pin = 19,  // PD7
};
constexpr AnalogSensorContext kExampleAnalog1{
    .base = 
        {
            .name = "AnalogRaw1",
            .canId = 0x200,
            .pollIntervalMs = 10,
        },
    .pin = 17, // PD5
};

constexpr SensorDescriptor kExampleSensors[] = {
    MakeAnalogSensor(&kExampleAnalog0),
    MakeAnalogSensor(&kExampleAnalog1),
};

// Example board configuration demonstrating default CAN wiring and control IDs.
constexpr BoardConfig kBoardConfig{
    kDefaultCanCsPin,
    kDefaultCanIntPin,
    kDefaultCanStbyPin,
    kDefaultMcpOscHz,
    kDefaultArbitrationBitrate,
    kDefaultDataBitrateFactor,
    kDefaultUseExtendedIds,
    kDefaultControlCommands,
    kExampleHooks,
    kExampleSensors,
    sizeof(kExampleSensors) / sizeof(kExampleSensors[0]),
};
