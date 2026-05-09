#pragma once

#include <Arduino.h>
#include <config.h>
#include <stdint.h>

constexpr uint8_t kPulseRpmSampleFrameVersion = 1U;

constexpr uint8_t kPulseRpmSampleValidTiming = 0x01U;
constexpr uint8_t kPulseRpmSampleValidRpm = 0x02U;

constexpr uint32_t kPulseRpmDefaultStaleAfterMicros = 300000UL;
constexpr uint32_t kPulseRpmDefaultMinPulseSpacingMicros = 500UL;
constexpr uint32_t kPulseRpmDefaultMilliRevolutionsPerPulse = 2000UL;

constexpr int16_t kPulseRpmSensorErrorNone = 0;
constexpr int16_t kPulseRpmSensorErrorNotInitialized = -1;
constexpr int16_t kPulseRpmSensorErrorUnsupportedInterruptPin = -2;
constexpr int16_t kPulseRpmSensorErrorInsufficientSamples = -3;
constexpr int16_t kPulseRpmSensorErrorStaleTimeout = -4;
constexpr int16_t kPulseRpmSensorErrorNoIsrSlotAvailable = -5;

constexpr uint16_t kFourStrokeEnginePulsesPerRevolution = 1000U;
constexpr uint16_t kTwoStrokeEnginePulsesPerRevolution = 500U;

struct __attribute__((packed)) PulseRpmDataSampleFrame {
  uint8_t version;
  uint8_t validMask;
  int16_t error;
  int32_t milliRpm;
};

struct __attribute__((packed)) PulseRpmStatsSampleFrame {
  uint8_t version;
  uint8_t validMask;
  uint32_t pulseIntervalMicros;
  uint32_t sampleAgeMicros;
  uint32_t acceptedPulseCount;
  uint32_t rejectedPulseCount;
  int32_t milliRpm;
  int16_t error;
};

static_assert(sizeof(PulseRpmDataSampleFrame) <= 64U,
              "Pulse RPM data frame must fit in one CAN FD payload");
static_assert(sizeof(PulseRpmStatsSampleFrame) <= 64U,
              "Pulse RPM stats frame must fit in one CAN FD payload");

constexpr uint8_t kPulseRpmDataSensorPayloadSize =
    static_cast<uint8_t>(sizeof(PulseRpmDataSampleFrame));
constexpr uint8_t kPulseRpmStatsSensorPayloadSize =
    static_cast<uint8_t>(sizeof(PulseRpmStatsSampleFrame));

struct PulseRpmSensorRuntime {
  bool initialized = false;
  int16_t initializationError = kPulseRpmSensorErrorNotInitialized;

  volatile uint32_t isrLastAcceptedEdgeMicros = 0U;
  volatile uint32_t isrLastPulseIntervalMicros = 0U;
  volatile uint32_t isrAcceptedPulseCount = 0U;
  volatile uint32_t isrRejectedPulseCount = 0U;

  uint32_t lastProcessedAcceptedPulseCount = 0U;
  bool hasValidInterval = false;
  uint8_t validMask = 0U;
  uint32_t pulseIntervalMicros = 0U;
  uint32_t sampleAgeMicros = 0U;
  uint32_t acceptedPulseCount = 0U;
  uint32_t rejectedPulseCount = 0U;
  int32_t lastMilliRpm = 0;
  int16_t lastError = kPulseRpmSensorErrorNotInitialized;
};

struct PulseRpmSubSensorContext {
  SensorContext base;
  PulseRpmSensorRuntime *runtime;
  uint8_t pin;
  uint32_t milliRevolutionsPerPulse;
  uint32_t staleAfterMicros;
  uint32_t minPulseSpacingMicros;
  bool countRisingEdge;
};

bool PulseRpmSensorBegin(const void *ctx);
bool PulseRpmDataSensorSample(const void *ctx, CANFDMessage &outFrame);
bool PulseRpmStatsSensorSample(const void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor MakePulseRpmDataSensor(
    const PulseRpmSubSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = PulseRpmSensorBegin,
      .sample = PulseRpmDataSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}

constexpr SensorDescriptor MakePulseRpmStatsSensor(
    const PulseRpmSubSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = PulseRpmSensorBegin,
      .sample = PulseRpmStatsSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}
