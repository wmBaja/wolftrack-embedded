#include <Arduino.h>
#include <SPI.h>
#include <ACAN2517FD.h>
#include <avr/sleep.h>

#include "config.h"        // Common contracts for board configs
#include <can_driver.h>
#include <sensors_config.h>  // Provided by the selected board environment

namespace {

// ACAN2517FD driver instance configured with board-provided pins.
ACAN2517FD gCanDriver{kBoardConfig.canCsPin, SPI, kBoardConfig.canIntPin};

enum class NodeState { Awake, Sleeping };
NodeState gNodeState = NodeState::Awake;

volatile bool gSleepRequested = false;
volatile bool gWakeRequested = false;

struct SensorRuntime {
  const SensorDescriptor *desc;
  uint32_t nextPollAtMs;
};

constexpr size_t kSensorCount = kBoardConfig.sensorCount;
SensorRuntime gSensorRuntime[kSensorCount > 0 ? kSensorCount : 1];

void CallIfSet(void (*hook)()) {
  if (hook != nullptr) {
    hook();
  }
}

void OnWakeFlag() {
  gWakeRequested = true;
  gSleepRequested = false;
}

void OnCanInterrupt() {
  gCanDriver.isr();
}

bool ConfigureCan() {
  ACAN2517FDSettings settings{kBoardConfig.canOscillatorHz,
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
  for (size_t i = 0; i < kBoardConfig.sensorCount; ++i) {
    SensorRuntime &runtime = gSensorRuntime[i];
    runtime.desc = &kBoardConfig.sensors[i];
    runtime.nextPollAtMs = now + runtime.desc->pollIntervalMs;

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

    if (desc.pollIntervalMs == 0U) {
      continue;  // Disabled sensor.
    }

    if (nowMs < runtime.nextPollAtMs) {
      continue;
    }

    runtime.nextPollAtMs = nowMs + desc.pollIntervalMs;

    if (desc.sample == nullptr) {
      continue;
    }

    CANFDMessage frame;
    frame.id = desc.canId;
    frame.ext = kBoardConfig.useExtendedIds;
    frame.len = 0;

    // Sample function populates frame len and data, true if successful
    if (!desc.sample(desc.context, frame)) {
      continue;  // If sample returns false, skip trying to send
    }

    gCanDriver.tryToSend(frame);
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
  for (size_t i = 0; i < kBoardConfig.sensorCount; ++i) {
    SensorRuntime &runtime = gSensorRuntime[i];
    const SensorDescriptor &desc = *runtime.desc;

    runtime.nextPollAtMs = now + desc.pollIntervalMs;
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
  SPI.begin();

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
