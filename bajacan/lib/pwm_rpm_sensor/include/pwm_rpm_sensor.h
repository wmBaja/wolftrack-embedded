#pragma once

#include <config.h>
#include <stdint.h>

constexpr uint8_t kPwmRpmSampleFrameVersion = 2U;

constexpr uint8_t kPwmRpmSampleValidDutyCycle = 0x01U;
constexpr uint8_t kPwmRpmSampleValidAngle = 0x02U;
constexpr uint8_t kPwmRpmSampleValidRpm = 0x04U;

constexpr uint16_t kPwmRpmDutyCycleScale = 10000U;
constexpr uint16_t kPwmRpmCentiDegreesPerRevolution = 36000U;
// Covers the slowest AS5600 PWM mode (115 Hz) with margin.
constexpr uint32_t kPwmRpmDefaultTimeoutMicros = 10000UL;
constexpr uint16_t kAS5600PwmFrameBits = 4351U;
constexpr uint16_t kAS5600PwmHeaderBits = 128U;
constexpr uint16_t kAS5600PwmAngleBits = 4095U;
constexpr uint16_t kPwmRpmAS5600CountsPerRevolution = 4096U;

constexpr int16_t kPwmRpmSensorErrorNone = 0;
constexpr int16_t kPwmRpmSensorErrorTimeoutWaitingForEdge = -1;
constexpr int16_t kPwmRpmSensorErrorTimeoutMeasuringPulse = -2;
constexpr int16_t kPwmRpmSensorErrorInvalidPeriod = -3;
constexpr int16_t kPwmRpmSensorErrorZeroDeltaTime = -4;
constexpr int16_t kPwmRpmSensorErrorNotInitialized = -5;

struct __attribute__((packed)) PwmRpmSampleFrame {
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

static_assert(sizeof(PwmRpmSampleFrame) <= 64U,
              "PWM RPM sample frame must fit in one CAN FD payload");

constexpr uint8_t kPwmRpmSensorPayloadSize =
    static_cast<uint8_t>(sizeof(PwmRpmSampleFrame));

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
};

struct PwmRpmSensorContext {
  SensorContext base;
  PwmRpmSensorRuntime *runtime;
  uint8_t pin;
  // Set longer than one full AS5600 PWM period. Used for timeout detection.
  uint32_t timeoutMicros;
};

bool PwmRpmSensorBegin(const void *ctx);
bool PwmRpmSensorSample(const void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor MakePwmRpmSensor(
    const PwmRpmSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = PwmRpmSensorBegin,
      .sample = PwmRpmSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}
