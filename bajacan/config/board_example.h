#pragma once

#include <config.h>

// Example hooks; set to nullptr when unused.
constexpr BoardHooks kExampleHooks{
    nullptr,  // preSetup
    nullptr,  // beforeSleep
    nullptr   // afterWake
};

// Example sensors table; add entries as real sensors are implemented.
constexpr SensorDescriptor kExampleSensors[] = {};

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
