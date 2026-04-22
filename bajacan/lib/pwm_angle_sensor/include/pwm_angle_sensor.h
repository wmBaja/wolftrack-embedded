#pragma once

#include <config.h>
#include <stdint.h>

constexpr uint8_t kPWMAngleSampleFrameVersion = 2U;

constexpr uint8_t kPWMAngleSampleValidDutyCycle = 0x01U;
constexpr uint8_t kPWMAngleSampleValidAngle = 0x02U;
constexpr uint8_t kPWMAngleSampleValidRpm = 0x04U;

constexpr uint16_t kPWMAngleDutyCycleScale = 10000U;
constexpr uint16_t kPWMAngleCentiDegreesPerRevolution = 36000U;
// Covers the slowest AS5600 PWM mode (115 Hz) with margin.
constexpr uint32_t kPWMAngleDefaultTimeoutMicros = 10000UL;
constexpr uint16_t kAS5600PwmFrameBits = 4351U;
constexpr uint16_t kAS5600PwmHeaderBits = 128U;
constexpr uint16_t kAS5600PwmAngleBits = 4095U;
constexpr uint16_t kPWMAngleAS5600CountsPerRevolution = 4096U;

constexpr int16_t kPWMAngleSensorErrorNone = 0;
constexpr int16_t kPWMAngleSensorErrorTimeoutWaitingForEdge = -1;
constexpr int16_t kPWMAngleSensorErrorTimeoutMeasuringPulse = -2;
constexpr int16_t kPWMAngleSensorErrorInvalidPeriod = -3;
constexpr int16_t kPWMAngleSensorErrorZeroDeltaTime = -4;
constexpr int16_t kPWMAngleSensorErrorNotInitialized = -5;

struct __attribute__((packed)) PWMAngleSampleFrame {
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

static_assert(sizeof(PWMAngleSampleFrame) <= 64U,
              "PWM angle sample frame must fit in one CAN FD payload");

constexpr uint8_t kPWMAngleSensorPayloadSize =
    static_cast<uint8_t>(sizeof(PWMAngleSampleFrame));

struct PWMAngleSensorRuntime {
  bool initialized = false;
  bool hasPreviousSample = false;
  uint16_t previousRawAngle = 0U;
  uint32_t previousSampleAtMicros = 0U;
};

struct PWMAngleSensorContext {
  SensorContext base;
  PWMAngleSensorRuntime *runtime;
  uint8_t pin;
  // Set longer than one full AS5600 PWM period so the decoder can capture a
  // full rising-edge-to-rising-edge frame.
  uint32_t timeoutMicros;
};

bool PWMAngleSensorBegin(const void *ctx);
bool PWMAngleSensorSample(const void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor MakePWMAngleSensor(
    const PWMAngleSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = PWMAngleSensorBegin,
      .sample = PWMAngleSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}
