#include <Arduino.h>
#include <AS5600.h>
#include <pwm_rpm_sensor.h>

#include <string.h>

namespace {

// Up to 4 PWM sensors supported for interrupts
constexpr size_t kMaxPwmSensors = 4;
PwmRpmSensorRuntime* gPwmRpmRuntimes[kMaxPwmSensors] = {nullptr};
uint8_t gPwmRpmPins[kMaxPwmSensors] = {0};

void PwmRpmIsr(size_t index) {
  PwmRpmSensorRuntime* runtime = gPwmRpmRuntimes[index];
  if (!runtime) return;

  uint32_t now = micros();
  bool isHigh = digitalRead(gPwmRpmPins[index]);

  if (isHigh) {
    if (runtime->isrLastRiseMicros != 0) {
        runtime->isrPeriodMicros = now - runtime->isrLastRiseMicros;
        runtime->isrHasData = true;
    }
    runtime->isrLastRiseMicros = now;
  } else {
    runtime->isrHighPulseMicros = now - runtime->isrLastRiseMicros;
    runtime->isrSampleAtMicros = now; // Sampled at falling edge
  }
}

void PwmRpmIsr0() { PwmRpmIsr(0); }
void PwmRpmIsr1() { PwmRpmIsr(1); }
void PwmRpmIsr2() { PwmRpmIsr(2); }
void PwmRpmIsr3() { PwmRpmIsr(3); }

void (*const gPwmRpmIsrStubs[kMaxPwmSensors])() = {
  PwmRpmIsr0, PwmRpmIsr1, PwmRpmIsr2, PwmRpmIsr3
};

const PwmRpmSensorContext *GetPwmRpmContext(const void *ctx) {
  return static_cast<const PwmRpmSensorContext *>(ctx);
}

uint32_t GetTimeoutMicros(const PwmRpmSensorContext &config) {
  if (config.timeoutMicros == 0U) {
    return kPwmRpmDefaultTimeoutMicros;
  }
  return config.timeoutMicros;
}

void CopySampleToFrame(const PwmRpmSampleFrame &sample,
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
      static_cast<uint64_t>(highPulseMicros) * kPwmRpmDutyCycleScale;
  const uint32_t rounded =
      static_cast<uint32_t>((scaledHigh + (periodMicros / 2U)) / periodMicros);
  if (rounded >= kPwmRpmDutyCycleScale) {
    return kPwmRpmDutyCycleScale;
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
      kPwmRpmAS5600CountsPerRevolution / 2;

  int32_t delta =
      static_cast<int32_t>(currentRawAngle) - static_cast<int32_t>(previousRawAngle);
  if (delta > kHalfRevolutionCounts) {
    delta -= kPwmRpmAS5600CountsPerRevolution;
  } else if (delta < -kHalfRevolutionCounts) {
    delta += kPwmRpmAS5600CountsPerRevolution;
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

}  // namespace

bool PwmRpmSensorBegin(const void *ctx) {
  const PwmRpmSensorContext *config = GetPwmRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  pinMode(config->pin, INPUT);

  // Find a free slot for the ISR
  int slot = -1;
  for (size_t i = 0; i < kMaxPwmSensors; ++i) {
    if (gPwmRpmRuntimes[i] == nullptr || gPwmRpmRuntimes[i] == config->runtime) {
      slot = i;
      break;
    }
  }

  if (slot >= 0) {
    gPwmRpmRuntimes[slot] = config->runtime;
    gPwmRpmPins[slot] = config->pin;
    attachInterrupt(digitalPinToInterrupt(config->pin), gPwmRpmIsrStubs[slot], CHANGE);
  } else {
    return false; // No more ISR slots available
  }

  config->runtime->initialized = true;
  config->runtime->hasPreviousSample = false;
  config->runtime->previousRawAngle = 0U;
  config->runtime->previousSampleAtMicros = 0U;
  
  config->runtime->isrLastRiseMicros = 0U;
  config->runtime->isrHighPulseMicros = 0U;
  config->runtime->isrPeriodMicros = 0U;
  config->runtime->isrSampleAtMicros = 0U;
  config->runtime->isrHasData = false;
  
  return true;
}

bool PwmRpmSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const PwmRpmSensorContext *config = GetPwmRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  PwmRpmSampleFrame sample = {};
  sample.version = kPwmRpmSampleFrameVersion;

  if (!config->runtime->initialized) {
    sample.error = kPwmRpmSensorErrorNotInitialized;
    CopySampleToFrame(sample, outFrame);
    return true;
  }

  // Safely read volatile variables
  noInterrupts();
  bool hasData = config->runtime->isrHasData;
  uint32_t highPulseMicros = config->runtime->isrHighPulseMicros;
  uint32_t periodMicros = config->runtime->isrPeriodMicros;
  uint32_t sampleAtMicros = config->runtime->isrSampleAtMicros;
  config->runtime->isrHasData = false; // Reset flag after reading
  interrupts();

  if (!hasData) {
    // Check for timeout
    uint32_t now = micros();
    if (now - sampleAtMicros > GetTimeoutMicros(*config)) {
      sample.error = kPwmRpmSensorErrorTimeoutWaitingForEdge;
    } else {
      sample.error = kPwmRpmSensorErrorZeroDeltaTime;
    }
    CopySampleToFrame(sample, outFrame);
    return true;
  }

  if (periodMicros == 0U) {
    sample.error = kPwmRpmSensorErrorInvalidPeriod;
    CopySampleToFrame(sample, outFrame);
    return true;
  }

  sample.error = kPwmRpmSensorErrorNone;
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
      kPwmRpmSampleValidDutyCycle | kPwmRpmSampleValidAngle;
  sample.sampleIntervalMicros =
      static_cast<uint32_t>(sampleAtMicros - config->runtime->previousSampleAtMicros);
      
  if (sample.sampleIntervalMicros == 0U) {
    sample.error = kPwmRpmSensorErrorZeroDeltaTime;
    CopySampleToFrame(sample, outFrame);
    return true;
  }

  const int32_t deltaRawAngle = ComputeWrappedRawDelta(
      sample.rawAngle, config->runtime->previousRawAngle);
  sample.milliRpm = ComputeMilliRpmFromDelta(deltaRawAngle,
                                             sample.sampleIntervalMicros);
  sample.validMask |= kPwmRpmSampleValidRpm;

  config->runtime->previousRawAngle = sample.rawAngle;
  config->runtime->previousSampleAtMicros = sampleAtMicros;

  CopySampleToFrame(sample, outFrame);
  return true;
}
