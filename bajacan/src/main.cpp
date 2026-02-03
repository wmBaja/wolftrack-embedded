#include <Arduino.h>
#include <SPI.h>
#include <ACAN2517FD.h>
#include <avr/sleep.h>

#include "config.h"        // Common contracts for board configs
#include "debug_print.h"
#include <analog_sensor.h>
#include <can_driver.h>
#include <sensors_config.h>  // Provided by the selected board environment

namespace {

// ACAN2517FD driver instance configured with board-provided pins.
ACAN2517FD gCanDriver{kBoardConfig.canCsPin, SPI, kBoardConfig.canIntPin};
// TEMP: Toggle pin on CAN TX for scope frequency checks (remove when done).
constexpr uint8_t kCanTxTogglePin = 3;
bool gCanTxToggleState = false;

enum class NodeState { Awake, Sleeping };
NodeState gNodeState = NodeState::Awake;

volatile bool gSleepRequested = false;
volatile bool gWakeRequested = false;

struct SensorRuntime {
  const SensorDescriptor *desc;
  const SensorContext *context;
  uint32_t nextPollAtMs;
};

constexpr size_t kSensorCount = kBoardConfig.sensorCount;
SensorRuntime gSensorRuntime[kSensorCount > 0 ? kSensorCount : 1];

void CallIfSet(void (*hook)()) {
  if (hook != nullptr) {
    hook();
  }
}

const SensorContext *GetSensorContext(const SensorDescriptor &desc) {
  return static_cast<const SensorContext *>(desc.context);
}

size_t CountActiveSensors() {
  size_t count = 0;
  for (size_t i = 0; i < kBoardConfig.sensorCount; ++i) {
    const SensorContext *context = GetSensorContext(kBoardConfig.sensors[i]);
    if (context != nullptr && context->pollIntervalMs > 0U) {
      ++count;
    }
  }
  return count;
}

uint32_t StaggeredFirstPollTime(const uint32_t nowMs,
                                const uint16_t pollIntervalMs,
                                const size_t activeIndex,
                                const size_t activeCount) {
  if (pollIntervalMs == 0U || activeCount <= 1U) {
    return nowMs + pollIntervalMs;
  }
  const uint32_t offset =
      (static_cast<uint32_t>(pollIntervalMs) * activeIndex) / activeCount;
  return nowMs + offset;
}

void OnWakeFlag() {
  gWakeRequested = true;
  gSleepRequested = false;
}

void OnCanInterrupt() {
  gCanDriver.isr();
}

bool ConfigureCan() {
  ACAN2517FDSettings settings{kBoardConfig.canOscillator,
                              kBoardConfig.arbitrationBitrate,
                              kBoardConfig.dataBitrateFactor};
  settings.mRequestedMode = ACAN2517FDSettings::NormalFD;
  const uint32_t errorCode = gCanDriver.begin(settings, OnCanInterrupt);
  return errorCode == 0U;
}

bool MatchesCommand(const CANFDMessage &frame, const uint32_t expectedId,
                    const uint8_t expectedByte) {
  // Only consider frames with the expected ID type.
  if (frame.ext != kBoardConfig.useExtendedIds) {
    return false;
  }

  if (frame.id != expectedId) {
    return false;
  }

  if (kBoardConfig.control.commandByteIndex >= frame.len) {
    return false;
  }

  return frame.data[kBoardConfig.control.commandByteIndex] == expectedByte;
}

void HandleControlFrame(const CANFDMessage &frame) {
  if (MatchesCommand(frame, kBoardConfig.control.sleepCommandId,
                     kBoardConfig.control.sleepCommandByte)) {
    gSleepRequested = true;
    gWakeRequested = false;
  }
}

void ServiceIncomingCan() {
  CANFDMessage frame;
  while (gCanDriver.available()) {
    gCanDriver.receive(frame);
    HandleControlFrame(frame);

    // TODO: Route other inbound frames (e.g., configuration or diagnostics).
  }
}

void InitializeSensors() {
  const uint32_t now = millis();
  const size_t activeCount = CountActiveSensors();
  size_t activeIndex = 0;
  for (size_t i = 0; i < kBoardConfig.sensorCount; ++i) {
    SensorRuntime &runtime = gSensorRuntime[i];
    runtime.desc = &kBoardConfig.sensors[i];
    runtime.context = GetSensorContext(*runtime.desc);
    const uint16_t pollIntervalMs =
        runtime.context != nullptr ? runtime.context->pollIntervalMs : 0U;
    if (runtime.context != nullptr && pollIntervalMs > 0U) {
      runtime.nextPollAtMs =
          StaggeredFirstPollTime(now, pollIntervalMs, activeIndex, activeCount);
      ++activeIndex;
    } else {
      runtime.nextPollAtMs = now + pollIntervalMs;
    }

    if (runtime.context == nullptr) {
      continue;
    }

    if (runtime.desc->begin != nullptr) {
      const bool ok = runtime.desc->begin(runtime.desc->context);
      (void)ok;  // TODO: surface init failures via CAN or a status LED.
    }
  }
}

void PollSensors(const uint32_t nowMs) {
  for (size_t i = 0; i < kBoardConfig.sensorCount; ++i) {
    SensorRuntime &runtime = gSensorRuntime[i];
    const SensorDescriptor &desc = *runtime.desc;
    const SensorContext *context = runtime.context;

    if (context == nullptr) {
      continue;
    }

    if (context->pollIntervalMs == 0U) {
      continue;  // Disabled sensor.
    }

    if (nowMs < runtime.nextPollAtMs) {
      continue;
    }

    {
      const uint32_t scheduledAt = runtime.nextPollAtMs;
      const uint32_t intervalMs = context->pollIntervalMs;
      uint32_t nextPoll = scheduledAt + intervalMs;
      if (nextPoll <= nowMs) {
        nextPoll = nowMs + intervalMs;
      }
      runtime.nextPollAtMs = nextPoll;
    }

    if (desc.sample == nullptr) {
      continue;
    }

    CANFDMessage frame;
    frame.id = context->canId;
    frame.ext = kBoardConfig.useExtendedIds;
    frame.len = 0;

    // Sample function populates frame len and data, true if successful
    if (!desc.sample(desc.context, frame)) {
      continue;  // If sample returns false, skip trying to send
    }

    const bool sent = gCanDriver.tryToSend(frame);
    // TEMP: Toggle pin on CAN TX for scope frequency checks (remove when done).
    gCanTxToggleState = !gCanTxToggleState;
    digitalWrite(kCanTxTogglePin, gCanTxToggleState ? HIGH : LOW);


#if BAJACAN_ENABLE_DEBUG_PRINTS
    PrintSensorPoll(context->name, frame, nowMs);
    PrintCanTxResult(frame, nowMs, sent);
#endif
  }
}

void SuspendSensorsForSleep() {
  for (size_t i = 0; i < kBoardConfig.sensorCount; ++i) {
    const SensorDescriptor &desc = *gSensorRuntime[i].desc;
    if (desc.suspend != nullptr) {
      desc.suspend(desc.context);
    }
  }
}

void ResumeSensorsAfterWake() {
  const uint32_t now = millis();
  const size_t activeCount = CountActiveSensors();
  size_t activeIndex = 0;
  for (size_t i = 0; i < kBoardConfig.sensorCount; ++i) {
    SensorRuntime &runtime = gSensorRuntime[i];
    const SensorDescriptor &desc = *runtime.desc;
    const SensorContext *context = runtime.context;
    const uint16_t pollIntervalMs =
        context != nullptr ? context->pollIntervalMs : 0U;
    if (context != nullptr && pollIntervalMs > 0U) {
      runtime.nextPollAtMs =
          StaggeredFirstPollTime(now, pollIntervalMs, activeIndex, activeCount);
      ++activeIndex;
    } else {
      runtime.nextPollAtMs = now + pollIntervalMs;
    }
    if (desc.resume != nullptr) {
      desc.resume(desc.context);
    }
  }
}

void EnterLowPowerSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void PrepareForSleep() {
  if (gNodeState == NodeState::Sleeping) {
    return;
  }

  SuspendSensorsForSleep();
  CallIfSet(kBoardConfig.hooks.beforeSleep);

  SleepCanDriver(gCanDriver, kBoardConfig);

  gNodeState = NodeState::Sleeping;
  gSleepRequested = false;
}

void WakeIfRequested() {
  if (!gWakeRequested) {
    return;
  }

  gWakeRequested = false;
  gSleepRequested = false;

  WakeCanDriver(gCanDriver, kBoardConfig);
  gNodeState = NodeState::Awake;

  ResumeSensorsAfterWake();
  CallIfSet(kBoardConfig.hooks.afterWake);
}

}  // namespace

void setup() {
  CallIfSet(kBoardConfig.hooks.preSetup);

  pinMode(kBoardConfig.canCsPin, OUTPUT);
  pinMode(kBoardConfig.canIntPin, INPUT_PULLUP);
  pinMode(kBoardConfig.canStbyPin, OUTPUT);
  // TEMP: Toggle pin on CAN TX for scope frequency checks (remove when done).
  pinMode(kCanTxTogglePin, OUTPUT);
  SPI.begin();
#if BAJACAN_ENABLE_DEBUG_PRINTS
  Serial.begin(115200); // Serial0 for debug
#endif

  if (!ConfigureCan()) {
    // TODO: Surface CAN init failure via LED blink or debug UART.
    while (true) {
      delay(100);
    }
  }
  gCanDriver.setWakeHandler(OnWakeFlag);
  gCanDriver.enableWakeInterrupt();
  gCanDriver.clearWakeFlag();

  InitializeSensors();
}

void loop() {
  const uint32_t now = millis();

  // Always service CAN to detect wake packets and other inbound commands.
  ServiceIncomingCan();
  WakeIfRequested();

  if (gNodeState == NodeState::Sleeping) {
    EnterLowPowerSleep();  // Pauses after execution until interrupt
    sleep_disable();       // Wake CPU immediately on interrupt
    WakeIfRequested();     // Wake flag set by ISR
    return;
  }

  PollSensors(now);

  if (gSleepRequested) {
    PrepareForSleep();
  }
}
