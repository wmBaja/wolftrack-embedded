#include <Arduino.h>
#include <AS5600.h>
#include <pwm_angle_sensor.h>

#include <string.h>

namespace {

const PWMAngleSensorContext *GetPWMAngleContext(const void *ctx) {
  return static_cast<const PWMAngleSensorContext *>(ctx);
}

uint32_t GetTimeoutMicros(const PWMAngleSensorContext &config) {
  if (config.timeoutMicros == 0U) {
    return kPWMAngleDefaultTimeoutMicros;
  }
  return config.timeoutMicros;
}

void CopySampleToFrame(const PWMAngleSampleFrame &sample,
                       CANFDMessage &outFrame) {
  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
}

uint16_t PulseWidthToDutyCycleBasisPoints(const uint32_t highPulseMicros,
                                          const uint32_t periodMicros) {
  if (periodMicros == 0U) {
    return 0U;
  }

  const uint64_t scaledHigh =
      static_cast<uint64_t>(highPulseMicros) * kPWMAngleDutyCycleScale;
  const uint32_t rounded =
      static_cast<uint32_t>((scaledHigh + (periodMicros / 2U)) / periodMicros);
  if (rounded >= kPWMAngleDutyCycleScale) {
    return kPWMAngleDutyCycleScale;
  }
  return static_cast<uint16_t>(rounded);
}

uint16_t HighPulseToRawAngle(const uint32_t highPulseMicros,
                             const uint32_t periodMicros) {
  if (periodMicros == 0U) {
    return 0U;
  }

  const uint64_t scaledHighBits =
      static_cast<uint64_t>(highPulseMicros) * kAS5600PwmFrameBits;
  const uint32_t highBits = static_cast<uint32_t>(
      (scaledHighBits + (periodMicros / 2U)) / periodMicros);
  if (highBits <= kAS5600PwmHeaderBits) {
    return 0U;
  }

  const uint32_t rawAngle = highBits - kAS5600PwmHeaderBits;
  if (rawAngle >= kAS5600PwmAngleBits) {
    return kAS5600PwmAngleBits;
  }
  return static_cast<uint16_t>(rawAngle);
}

uint16_t RawAngleToCentiDegrees(const uint16_t rawAngle) {
  const float degrees = static_cast<float>(rawAngle) * AS5600_RAW_TO_DEGREES;
  const float centiDegrees = degrees * 100.0f;
  return static_cast<uint16_t>(centiDegrees + 0.5f);
}

int32_t ComputeWrappedRawDelta(const uint16_t currentRawAngle,
                               const uint16_t previousRawAngle) {
  constexpr int32_t kHalfRevolutionCounts =
      kPWMAngleAS5600CountsPerRevolution / 2;

  int32_t delta =
      static_cast<int32_t>(currentRawAngle) - static_cast<int32_t>(previousRawAngle);
  if (delta > kHalfRevolutionCounts) {
    delta -= kPWMAngleAS5600CountsPerRevolution;
  } else if (delta < -kHalfRevolutionCounts) {
    delta += kPWMAngleAS5600CountsPerRevolution;
  }
  return delta;
}

int32_t ComputeMilliRpmFromDelta(const int32_t deltaRawAngle,
                                 const uint32_t deltaTimeMicros) {
  if (deltaTimeMicros == 0U) {
    return 0;
  }

  const float rpm = (static_cast<float>(deltaRawAngle) * 1000000.0f *
                     AS5600_RAW_TO_RPM) /
                    static_cast<float>(deltaTimeMicros);
  const float milliRpm = rpm * 1000.0f;
  return static_cast<int32_t>(milliRpm >= 0.0f ? milliRpm + 0.5f
                                                : milliRpm - 0.5f);
}

bool WaitForLevel(const uint8_t pin, const int expectedLevel,
                  const uint32_t timeoutMicros) {
  const uint32_t startMicros = micros();
  while (digitalRead(pin) != expectedLevel) {
    if (static_cast<uint32_t>(micros() - startMicros) >= timeoutMicros) {
      return false;
    }
  }
  return true;
}

int16_t MeasureAs5600PwmFrame(const PWMAngleSensorContext &config,
                              uint32_t &outHighPulseMicros,
                              uint32_t &outPeriodMicros,
                              uint32_t &outSampleAtMicros) {
  const uint32_t timeoutMicros = GetTimeoutMicros(config);
  if (digitalRead(config.pin) == HIGH &&
      !WaitForLevel(config.pin, LOW, timeoutMicros)) {
    return kPWMAngleSensorErrorTimeoutWaitingForEdge;
  }

  if (!WaitForLevel(config.pin, HIGH, timeoutMicros)) {
    return kPWMAngleSensorErrorTimeoutWaitingForEdge;
  }
  const uint32_t riseMicros = micros();

  if (!WaitForLevel(config.pin, LOW, timeoutMicros)) {
    return kPWMAngleSensorErrorTimeoutMeasuringPulse;
  }
  const uint32_t fallMicros = micros();

  if (!WaitForLevel(config.pin, HIGH, timeoutMicros)) {
    return kPWMAngleSensorErrorTimeoutWaitingForEdge;
  }
  const uint32_t nextRiseMicros = micros();

  outHighPulseMicros = static_cast<uint32_t>(fallMicros - riseMicros);
  outPeriodMicros = static_cast<uint32_t>(nextRiseMicros - riseMicros);
  outSampleAtMicros = nextRiseMicros;
  if (outPeriodMicros == 0U) {
    return kPWMAngleSensorErrorInvalidPeriod;
  }

  return kPWMAngleSensorErrorNone;
}

}  // namespace

bool PWMAngleSensorBegin(const void *ctx) {
  const PWMAngleSensorContext *config = GetPWMAngleContext(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  pinMode(config->pin, INPUT);
  config->runtime->initialized = true;
  config->runtime->hasPreviousSample = false;
  config->runtime->previousRawAngle = 0U;
  config->runtime->previousSampleAtMicros = 0U;
  return true;
}

bool PWMAngleSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const PWMAngleSensorContext *config = GetPWMAngleContext(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  PWMAngleSampleFrame sample = {};
  sample.version = kPWMAngleSampleFrameVersion;

  if (!config->runtime->initialized) {
    sample.error = kPWMAngleSensorErrorNotInitialized;
    CopySampleToFrame(sample, outFrame);
    return true;
  }

  uint32_t highPulseMicros = 0U;
  uint32_t periodMicros = 0U;
  uint32_t sampleAtMicros = 0U;
  sample.error = MeasureAs5600PwmFrame(*config, highPulseMicros, periodMicros,
                                       sampleAtMicros);
  if (sample.error != kPWMAngleSensorErrorNone) {
    CopySampleToFrame(sample, outFrame);
    return true;
  }

  sample.pwmPeriodMicros = periodMicros;
  sample.dutyCycleBasisPoints =
      PulseWidthToDutyCycleBasisPoints(highPulseMicros, periodMicros);
  sample.rawAngle = HighPulseToRawAngle(highPulseMicros, periodMicros);
  sample.angleCentiDegrees = RawAngleToCentiDegrees(sample.rawAngle);

  if (!config->runtime->hasPreviousSample) {
    config->runtime->hasPreviousSample = true;
    config->runtime->previousRawAngle = sample.rawAngle;
    config->runtime->previousSampleAtMicros = sampleAtMicros;
    CopySampleToFrame(sample, outFrame);
    return true;
  }

  sample.validMask =
      kPWMAngleSampleValidDutyCycle | kPWMAngleSampleValidAngle;
  sample.sampleIntervalMicros =
      static_cast<uint32_t>(sampleAtMicros - config->runtime->previousSampleAtMicros);
  if (sample.sampleIntervalMicros == 0U) {
    sample.error = kPWMAngleSensorErrorZeroDeltaTime;
    CopySampleToFrame(sample, outFrame);
    return true;
  }

  const int32_t deltaRawAngle = ComputeWrappedRawDelta(
      sample.rawAngle, config->runtime->previousRawAngle);
  sample.milliRpm = ComputeMilliRpmFromDelta(deltaRawAngle,
                                             sample.sampleIntervalMicros);
  sample.validMask |= kPWMAngleSampleValidRpm;

  config->runtime->previousRawAngle = sample.rawAngle;
  config->runtime->previousSampleAtMicros = sampleAtMicros;

  CopySampleToFrame(sample, outFrame);
  return true;
}
