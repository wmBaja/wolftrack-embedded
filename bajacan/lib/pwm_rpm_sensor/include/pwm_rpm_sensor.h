#pragma once

#include <config.h>
#include <stdint.h>

constexpr uint8_t kPwmRpmSampleFrameVersion = 2U;

constexpr uint8_t kPwmRpmSampleValidDutyCycle = 0x01U;
constexpr uint8_t kPwmRpmSampleValidAngle = 0x02U;
constexpr uint8_t kPwmRpmSampleValidRpm = 0x04U;

constexpr uint16_t kPwmRpmDutyCycleScale = 10000U;
constexpr uint16_t kPwmRpmCentiDegreesPerRevolution = 36000U;

constexpr uint32_t kPwmRpmDefaultTimeoutMicros = 10000UL;
constexpr uint16_t kAS5600PwmFrameBits = 4351U;
constexpr uint16_t kAS5600PwmHeaderBits = 128U;
constexpr uint16_t kAS5600PwmAngleBits = kAS5600PwmFrameBits - (kAS5600PwmHeaderBits*2);
constexpr uint16_t kPwmRpmAS5600CountsPerRevolution = 4096U;

constexpr int16_t kPwmRpmSensorErrorNone = 0;
constexpr int16_t kPwmRpmSensorErrorTimeoutWaitingForEdge = -1;
constexpr int16_t kPwmRpmSensorErrorTimeoutMeasuringPulse = -2;
constexpr int16_t kPwmRpmSensorErrorInvalidPeriod = -3;
constexpr int16_t kPwmRpmSensorErrorZeroDeltaTime = -4;
constexpr int16_t kPwmRpmSensorErrorNotInitialized = -5;

struct __attribute__((packed)) PwmRpmDataSampleFrame {
  uint8_t version;
  uint8_t validMask;
  int16_t error;
  int32_t milliRpm;
};

struct __attribute__((packed)) PwmRpmStatsSampleFrame {
  uint8_t version;
  uint8_t validMask;
  uint16_t dutyCycleBasisPoints;
  uint16_t rawAngle;
  uint16_t angleCentiDegrees;
  uint32_t pwmPeriodMicros;
  uint32_t sampleIntervalMicros;
  int32_t milliRpm;
  int16_t error;
};

static_assert(sizeof(PwmRpmDataSampleFrame) <= 64U,
              "PWM RPM data frame must fit in one CAN FD payload");
static_assert(sizeof(PwmRpmStatsSampleFrame) <= 64U,
              "PWM RPM stats frame must fit in one CAN FD payload");

constexpr uint8_t kPwmRpmDataSensorPayloadSize =
    static_cast<uint8_t>(sizeof(PwmRpmDataSampleFrame));
constexpr uint8_t kPwmRpmStatsSensorPayloadSize =
    static_cast<uint8_t>(sizeof(PwmRpmStatsSampleFrame));

struct PwmRpmSensorRuntime {
  bool initialized = false;
  bool hasPreviousSample = false;
  uint16_t previousRawAngle = 0U;
  uint32_t previousSampleAtMicros = 0U;

  volatile uint32_t isrLastRiseMicros = 0U;
  volatile uint32_t isrHighPulseMicros = 0U;
  volatile uint32_t isrPeriodMicros = 0U;
  volatile uint32_t isrSampleAtMicros = 0U;
  volatile bool isrHasData = false;

  // Cached state for multiple sub-sensors to read
  int16_t lastError = kPwmRpmSensorErrorNone;
  uint8_t validMask = 0U;
  int32_t lastMilliRpm = 0;
  uint16_t dutyCycleBasisPoints = 0U;
  uint16_t rawAngle = 0U;
  uint16_t angleCentiDegrees = 0U;
  uint32_t pwmPeriodMicros = 0U;
  uint32_t sampleIntervalMicros = 0U;
};

struct PwmRpmSubSensorContext {
  SensorContext base;
  PwmRpmSensorRuntime *runtime;
  uint8_t pin;
  // Set longer than one full AS5600 PWM period. Used for timeout detection.
  uint32_t timeoutMicros;
};

bool PwmRpmSensorBegin(const void *ctx);
bool PwmRpmDataSensorSample(const void *ctx, CANFDMessage &outFrame);
bool PwmRpmStatsSensorSample(const void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor MakePwmRpmDataSensor(
    const PwmRpmSubSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = PwmRpmSensorBegin,
      .sample = PwmRpmDataSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}

constexpr SensorDescriptor MakePwmRpmStatsSensor(
    const PwmRpmSubSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = PwmRpmSensorBegin,
      .sample = PwmRpmStatsSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}
