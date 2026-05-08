#include <Arduino.h>
#include <SPI.h>
#include <ACAN2517FD.h>
#include <avr/sleep.h>

#include <string.h>

#include "config.h"        // Common contracts for board configs
#include "debug_print.h"
#include <analog_sensor.h>
#include <as5600_sensor.h>
#include <bno085_sensor.h>
#include <can_driver.h>
// Force PlatformIO LDF to discover drivers referenced only by board configs.
#include <i2c_sensor.h>
#include <mlx90614_sensor.h>
#include <sensors_config.h>  // Provided by the selected board environment

namespace {

// ACAN2517FD driver instance configured with board-provided pins.
ACAN2517FD gCanDriver{kBoardConfig.canCsPin, SPI, kBoardConfig.canIntPin};

enum class NodeState { Awake, Sleeping };
NodeState gNodeState = NodeState::Awake;

volatile bool gSleepRequested = false;
volatile bool gWakeRequested = false;

constexpr uint8_t kMaxCanFdPayloadBytes = 64U;

struct GroupRuntime {
  const MessageGroupConfig *group;
  uint32_t nextPollAtMs;
  uint8_t payloadBytes;
  bool valid;
};

constexpr size_t kGroupCount = kBoardConfig.groupCount;
GroupRuntime gGroupRuntime[kGroupCount > 0 ? kGroupCount : 1];

void CallIfSet(void (*hook)()) {
  if (hook != nullptr) {
    hook();
  }
}

const SensorContext *GetSensorContext(const SensorDescriptor &desc) {
  return static_cast<const SensorContext *>(desc.context);
}

bool IsActiveGroup(const GroupRuntime &runtime) {
  return runtime.valid && runtime.group != nullptr && runtime.payloadBytes > 0U &&
         runtime.group->pollIntervalMs > 0U;
}

size_t CountActiveGroups() {
  size_t count = 0;
  for (size_t i = 0; i < kBoardConfig.groupCount; ++i) {
    if (IsActiveGroup(gGroupRuntime[i])) {
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

uint32_t ConfigureCan() {
  ACAN2517FDSettings settings{kBoardConfig.canOscillator,
                              kBoardConfig.arbitrationBitrate,
                              kBoardConfig.dataBitrateFactor};
  settings.mRequestedMode = ACAN2517FDSettings::NormalFD;
  return gCanDriver.begin(settings, OnCanInterrupt);
}

uint8_t ComputeGroupPayloadBytes(const MessageGroupConfig &group, bool &valid) {
  if (group.sensors == nullptr || group.sensorCount == 0U) {
    valid = false;
    return 0U;
  }

  uint16_t total = 0U;
  for (size_t i = 0; i < group.sensorCount; ++i) {
    const SensorContext *context = GetSensorContext(group.sensors[i]);
    if (context == nullptr) {
      valid = false;
      return 0U;
    }
    total += context->payloadSize;
    if (total > kMaxCanFdPayloadBytes) {
      valid = false;
      return 0U;
    }
  }

  valid = true;
  return static_cast<uint8_t>(total);
}

void RescheduleGroups(const uint32_t nowMs) {
  const size_t activeCount = CountActiveGroups();
  size_t activeIndex = 0U;
  for (size_t i = 0; i < kBoardConfig.groupCount; ++i) {
    GroupRuntime &runtime = gGroupRuntime[i];
    const uint16_t pollIntervalMs =
        runtime.group != nullptr ? runtime.group->pollIntervalMs : 0U;
    if (IsActiveGroup(runtime)) {
      runtime.nextPollAtMs =
          StaggeredFirstPollTime(nowMs, pollIntervalMs, activeIndex, activeCount);
      ++activeIndex;
    } else {
      runtime.nextPollAtMs = nowMs + pollIntervalMs;
    }
  }
}

bool SampleGroupMember(const SensorDescriptor &desc, const SensorContext &context,
                       uint8_t *dst) {
  if (context.payloadSize == 0U) {
    return true;
  }
  if (dst == nullptr || desc.sample == nullptr) {
    return false;
  }

  CANFDMessage sampleFrame;
  sampleFrame.len = 0U;
  if (!desc.sample(desc.context, sampleFrame)) {
    return false;
  }
  if (sampleFrame.len != context.payloadSize) {
    return false;
  }

  memcpy(dst, sampleFrame.data, context.payloadSize);
  return true;
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
  for (size_t i = 0; i < kBoardConfig.groupCount; ++i) {
    GroupRuntime &runtime = gGroupRuntime[i];
    runtime.group = &kBoardConfig.groups[i];
    runtime.payloadBytes = ComputeGroupPayloadBytes(*runtime.group, runtime.valid);
    runtime.nextPollAtMs = now;

    if (!runtime.valid) {
      PrintGroupConfigError(runtime.group->name, now);
    }

    if (runtime.group->sensors == nullptr) {
      continue;
    }

    for (size_t sensorIndex = 0; sensorIndex < runtime.group->sensorCount;
         ++sensorIndex) {
      const SensorDescriptor &desc = runtime.group->sensors[sensorIndex];
      if (desc.context == nullptr || desc.begin == nullptr) {
        continue;
      }

      const bool ok = desc.begin(desc.context);
      (void)ok;  // TODO: surface init failures via CAN or a status LED.
    }
  }

  RescheduleGroups(millis());
}

void PollGroups(const uint32_t nowMs) {
  for (size_t i = 0; i < kBoardConfig.groupCount; ++i) {
    GroupRuntime &runtime = gGroupRuntime[i];
    if (!IsActiveGroup(runtime)) {
      continue;
    }

    if (nowMs < runtime.nextPollAtMs) {
      continue;
    }

    {
      const uint32_t scheduledAt = runtime.nextPollAtMs;
      const uint32_t intervalMs = runtime.group->pollIntervalMs;
      uint32_t nextPoll = scheduledAt + intervalMs;
      if (nextPoll <= nowMs) {
        nextPoll = nowMs + intervalMs;
      }
      runtime.nextPollAtMs = nextPoll;
    }

    CANFDMessage frame;
    frame.id = runtime.group->canId;
    frame.ext = kBoardConfig.useExtendedIds;
    frame.len = runtime.payloadBytes;
    memset(frame.data, 0, sizeof(frame.data));

    uint8_t payloadOffset = 0U;
    for (size_t sensorIndex = 0; sensorIndex < runtime.group->sensorCount;
         ++sensorIndex) {
      const SensorDescriptor &desc = runtime.group->sensors[sensorIndex];
      const SensorContext *context = GetSensorContext(desc);
      if (context == nullptr) {
        continue;
      }

      if (!SampleGroupMember(desc, *context, &frame.data[payloadOffset])) {
        PrintGroupMemberZeroFill(runtime.group->name, context->name, nowMs);
      }
      payloadOffset += context->payloadSize;
    }

    frame.pad();
    const bool sent = gCanDriver.tryToSend(frame);

#if BAJACAN_ENABLE_DEBUG_PRINTS
    PrintGroupPoll(runtime.group->name, frame, nowMs);
    PrintCanTxResult(frame, nowMs, sent);
#endif
  }
}

void SuspendSensorsForSleep() {
  for (size_t i = 0; i < kBoardConfig.groupCount; ++i) {
    const MessageGroupConfig &group = kBoardConfig.groups[i];
    if (group.sensors == nullptr) {
      continue;
    }
    for (size_t sensorIndex = 0; sensorIndex < group.sensorCount; ++sensorIndex) {
      const SensorDescriptor &desc = group.sensors[sensorIndex];
      if (desc.suspend != nullptr) {
        desc.suspend(desc.context);
      }
    }
  }
}

void ResumeSensorsAfterWake() {
  for (size_t i = 0; i < kBoardConfig.groupCount; ++i) {
    const MessageGroupConfig &group = kBoardConfig.groups[i];
    if (group.sensors == nullptr) {
      continue;
    }
    for (size_t sensorIndex = 0; sensorIndex < group.sensorCount; ++sensorIndex) {
      const SensorDescriptor &desc = group.sensors[sensorIndex];
      if (desc.resume != nullptr) {
        desc.resume(desc.context);
      }
    }
  }
  RescheduleGroups(millis());
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
#if BAJACAN_ENABLE_DEBUG_PRINTS
  Serial.begin(115200); // Serial0 for debug
#endif

  const uint32_t canErrorCode = ConfigureCan();
  if (canErrorCode != 0U) {
    while (true) {
#if BAJACAN_ENABLE_DEBUG_PRINTS
      Serial.print("CAN init failed error=0x");
      Serial.println(canErrorCode, HEX);
      delay(1000);
#else
      delay(100);
#endif
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
    EnterLowPowerSleep();  // Pauses after execution until interrupt  <<==  PAUSED HERE UNTIL WAKE INTERRUPT
    sleep_disable();       // Wake CPU immediately on interrupt       <<==  RESUMES BY EXECUTING THIS LINE
    WakeIfRequested();     // Wake flag set by ISR
    return;
  }

  PollGroups(now);

  if (gSleepRequested) {
    PrepareForSleep();
  }
}
