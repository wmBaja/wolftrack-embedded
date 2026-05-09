#pragma once

#include <Arduino.h>
#include <analog_rpm_sensor_internal.h>
#include <config.h>
#include <stdint.h>

constexpr uint8_t kAnalogRpmSampleFrameVersion = 1U;

constexpr uint8_t kAnalogRpmSampleValidAngle = 0x01U;
constexpr uint8_t kAnalogRpmSampleValidRpm = 0x02U;

constexpr uint16_t kAnalogRpmCountsPerRevolution = 4096U;
constexpr uint16_t kAnalogRpmCentiDegreesPerRevolution = 36000U;
constexpr uint8_t kAnalogRpmAdcResolutionBits = ADC_NATIVE_RESOLUTION;
constexpr uint16_t kAnalogRpmAdcMaxCounts =
    static_cast<uint16_t>((1UL << kAnalogRpmAdcResolutionBits) - 1UL);
constexpr uint8_t kAnalogRpmZeroDeltaDeadbandCounts = 8U;
constexpr uint8_t kAnalogRpmDefaultMotionConfirmSamples = 1U;
constexpr uint8_t kAnalogRpmDefaultZeroConfirmSamples = 1U;
constexpr uint32_t kAnalogRpmMaxSampleIntervalMicros = 25000UL;

constexpr int16_t kAnalogRpmSensorErrorNone = 0;
constexpr int16_t kAnalogRpmSensorErrorNotInitialized = -1;
constexpr int16_t kAnalogRpmSensorErrorZeroDeltaTime = -2;
constexpr int16_t kAnalogRpmSensorErrorSampleTooOld = -3;
constexpr int16_t kAnalogRpmSensorErrorAdcReadFailed = -4;

struct __attribute__((packed)) AnalogRpmDataSampleFrame {
  uint8_t version;
  uint8_t validMask;
  int16_t error;
  int32_t milliRpm;
};

struct __attribute__((packed)) AnalogRpmStatsSampleFrame {
  uint8_t version;
  uint8_t validMask;
  uint16_t rawAdc;
  uint16_t rawAngle;
  uint16_t angleCentiDegrees;
  uint32_t sampleIntervalMicros;
  int32_t rawMilliRpm;
  int32_t filteredMilliRpm;
  int16_t error;
};

static_assert(sizeof(AnalogRpmDataSampleFrame) <= 64U,
              "Analog RPM data frame must fit in one CAN FD payload");
static_assert(sizeof(AnalogRpmStatsSampleFrame) <= 64U,
              "Analog RPM stats frame must fit in one CAN FD payload");

constexpr uint8_t kAnalogRpmDataSensorPayloadSize =
    static_cast<uint8_t>(sizeof(AnalogRpmDataSampleFrame));
constexpr uint8_t kAnalogRpmStatsSensorPayloadSize =
    static_cast<uint8_t>(sizeof(AnalogRpmStatsSampleFrame));

struct AnalogRpmSensorRuntime {
  bool initialized = false;
  bool hasSample = false;
  bool hasFilteredRpm = false;
  bool motionConfirmed = false;
  uint16_t previousRawAngle = 0U;
  uint32_t previousSampleAtMicros = 0U;
  uint32_t lastSampleAtMicros = 0U;
  int8_t pendingMotionDirection = 0;
  uint8_t consecutiveMotionCount = 0U;
  uint8_t consecutiveZeroCount = 0U;

  int16_t lastError = kAnalogRpmSensorErrorNone;
  uint8_t validMask = 0U;
  uint16_t rawAdc = 0U;
  uint16_t rawAngle = 0U;
  uint16_t angleCentiDegrees = 0U;
  uint32_t sampleIntervalMicros = 0U;
  int32_t lastRawMilliRpm = 0;
  int32_t lastFilteredMilliRpm = 0;
};

struct AnalogRpmSubSensorContext {
  SensorContext base;
  AnalogRpmSensorRuntime *runtime;
  uint8_t pin;
  uint8_t zeroDeltaDeadbandCounts;
  uint8_t motionConfirmSamples;
  uint8_t zeroConfirmSamples;
};

bool AnalogRpmSensorBegin(const void *ctx);
bool AnalogRpmDataSensorSample(const void *ctx, CANFDMessage &outFrame);
bool AnalogRpmStatsSensorSample(const void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor MakeAnalogRpmDataSensor(
    const AnalogRpmSubSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = AnalogRpmSensorBegin,
      .sample = AnalogRpmDataSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}

constexpr SensorDescriptor MakeAnalogRpmStatsSensor(
    const AnalogRpmSubSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = AnalogRpmSensorBegin,
      .sample = AnalogRpmStatsSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}
