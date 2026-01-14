// Common configuration contracts shared between board-specific sensor configs
// and the board-agnostic application in src/main.cpp. Board configs should
// include this header and then provide concrete instances of the structures
// declared here.

#pragma once

#include <ACAN2517FD.h>
#include <stdint.h>

// Optional board-level hooks that may be provided by a board config. Any
// callback may be set to nullptr when unused.
struct BoardHooks {
  void (*preSetup)();     // Called before CAN and sensors are initialized.
  void (*beforeSleep)();  // Called right before entering low power sleep.
  void (*afterWake)();    // Called immediately after waking back up.
};

// Description of control messages used to manage power/sleep state. Boards
// should provide concrete values that match their CAN command map.
struct ControlMessageConfig {
  uint32_t sleepCommandId;
  uint32_t wakeCommandId;
  uint8_t sleepCommandByte;  // Optional payload byte used to verify sleep.
  uint8_t wakeCommandByte;   // Optional payload byte used to verify wake.
  uint8_t commandByteIndex;  // Index in payload containing the command byte.
};

// Defaults that match the current system-level CAN control contracts.
constexpr ControlMessageConfig kDefaultControlCommands{
    0x0,  // sleepCommandId
    0x0,  // wakeCommandId
    0x0,  // sleepCommandByte
    0x1,  // wakeCommandByte
    0     // commandByteIndex
};

// Contract that each sensor driver entry must satisfy. Board configs supply a
// table of these entries that main.cpp will iterate over.
struct SensorDescriptor {
  const char *name;
  uint32_t canId;            // CAN ID the sampled payload should be sent on.
  uint16_t pollIntervalMs;   // How often to poll/sample the sensor.
  void *context;             // Driver instance or any opaque pointer.
  bool (*begin)(void *ctx);  // Called once during setup.
  bool (*sample)(void *ctx,
                 CANFDMessage &outFrame);  // Should fill outFrame for sending.
  void (*suspend)(void *ctx);              // Optional; called before sleep.
  void (*resume)(void *ctx);               // Optional; called after wake.
};

// Aggregates the board-specific static data needed by the generic app.
struct BoardConfig {
  uint8_t canCsPin;
  uint8_t canIntPin;
  uint8_t canStbyPin;
  uint32_t canOscillatorHz;
  uint32_t arbitrationBitrate;
  DataBitRateFactor dataBitrateFactor;
  bool useExtendedIds;
  ControlMessageConfig control;
  BoardHooks hooks;
  const SensorDescriptor *sensors;
  size_t sensorCount;
};

// Common CAN defaults shared across boards; override any field in kBoardConfig
// if a given board differs.
constexpr uint32_t kDefaultMcpOscHz = 20'000'000UL;
constexpr uint8_t kDefaultCanCsPin = 7;
constexpr uint8_t kDefaultCanIntPin = 14;
constexpr uint8_t kDefaultCanStbyPin = 13;  // Set per board if STBY is wired.
constexpr uint32_t kDefaultArbitrationBitrate = 500'000UL;  // 500 kbps
constexpr DataBitRateFactor kDefaultDataBitrateFactor =
    DataBitRateFactor::x2;  // 1 Mbps data with 500 kbps arb
constexpr bool kDefaultUseExtendedIds = true;
