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

const PwmRpmSubSensorContext *GetPwmRpmContext(const void *ctx) {
  return static_cast<const PwmRpmSubSensorContext *>(ctx);
}

uint32_t GetTimeoutMicros(const PwmRpmSubSensorContext &config) {
  if (config.timeoutMicros == 0U) {
    return kPwmRpmDefaultTimeoutMicros;
  }
  return config.timeoutMicros;
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
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(rawAngle) * kPwmRpmCentiDegreesPerRevolution) /
      kPwmRpmAS5600CountsPerRevolution);
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

void UpdateRuntimeData(const PwmRpmSubSensorContext &config) {
  PwmRpmSensorRuntime *runtime = config.runtime;

  noInterrupts();
  bool hasData = runtime->isrHasData;
  uint32_t highPulseMicros = runtime->isrHighPulseMicros;
  uint32_t periodMicros = runtime->isrPeriodMicros;
  uint32_t sampleAtMicros = runtime->isrSampleAtMicros;
  runtime->isrHasData = false;
  interrupts();

  if (!hasData) {
    uint32_t now = micros();
    if (now - sampleAtMicros > GetTimeoutMicros(config)) {
      runtime->lastError = kPwmRpmSensorErrorTimeoutWaitingForEdge;
      runtime->validMask = 0;
    } else {
      runtime->lastError = kPwmRpmSensorErrorZeroDeltaTime;
    }
    return;
  }

  if (periodMicros == 0U) {
    runtime->lastError = kPwmRpmSensorErrorInvalidPeriod;
    runtime->validMask = 0;
    return;
  }

  runtime->lastError = kPwmRpmSensorErrorNone;
  runtime->pwmPeriodMicros = periodMicros;
  runtime->dutyCycleBasisPoints =
      PulseWidthToDutyCycleBasisPoints(highPulseMicros, periodMicros);
  runtime->rawAngle = HighPulseToRawAngle(highPulseMicros, periodMicros);
  runtime->angleCentiDegrees = RawAngleToCentiDegrees(runtime->rawAngle);

  if (!runtime->hasPreviousSample) {
    runtime->hasPreviousSample = true;
    runtime->previousRawAngle = runtime->rawAngle;
    runtime->previousSampleAtMicros = sampleAtMicros;
    runtime->validMask = kPwmRpmSampleValidDutyCycle | kPwmRpmSampleValidAngle;
    return;
  }

  runtime->sampleIntervalMicros =
      static_cast<uint32_t>(sampleAtMicros - runtime->previousSampleAtMicros);
      
  if (runtime->sampleIntervalMicros == 0U) {
    runtime->lastError = kPwmRpmSensorErrorZeroDeltaTime;
    return;
  }

  const int32_t deltaRawAngle = ComputeWrappedRawDelta(
      runtime->rawAngle, runtime->previousRawAngle);
  runtime->lastMilliRpm = ComputeMilliRpmFromDelta(deltaRawAngle,
                                             runtime->sampleIntervalMicros);
                                             
  runtime->validMask = kPwmRpmSampleValidDutyCycle | kPwmRpmSampleValidAngle | kPwmRpmSampleValidRpm;

  runtime->previousRawAngle = runtime->rawAngle;
  runtime->previousSampleAtMicros = sampleAtMicros;
}

}  // namespace

bool PwmRpmSensorBegin(const void *ctx) {
  const PwmRpmSubSensorContext *config = GetPwmRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  // Only initialize hardware once per runtime
  if (config->runtime->initialized) {
    return true;
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

bool PwmRpmDataSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const PwmRpmSubSensorContext *config = GetPwmRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr || !config->runtime->initialized) {
    PwmRpmDataSampleFrame sample = {};
    sample.version = kPwmRpmSampleFrameVersion;
    sample.error = kPwmRpmSensorErrorNotInitialized;
    outFrame.len = sizeof(sample);
    memcpy(outFrame.data, &sample, sizeof(sample));
    return true;
  }

  UpdateRuntimeData(*config);

  PwmRpmDataSampleFrame sample = {};
  sample.version = kPwmRpmSampleFrameVersion;
  sample.validMask = config->runtime->validMask;
  sample.error = config->runtime->lastError;
  sample.milliRpm = config->runtime->lastMilliRpm;

  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
  return true;
}

bool PwmRpmStatsSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const PwmRpmSubSensorContext *config = GetPwmRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr || !config->runtime->initialized) {
    PwmRpmStatsSampleFrame sample = {};
    sample.version = kPwmRpmSampleFrameVersion;
    sample.error = kPwmRpmSensorErrorNotInitialized;
    outFrame.len = sizeof(sample);
    memcpy(outFrame.data, &sample, sizeof(sample));
    return true;
  }

  UpdateRuntimeData(*config);

  PwmRpmStatsSampleFrame sample = {};
  sample.version = kPwmRpmSampleFrameVersion;
  sample.validMask = config->runtime->validMask;
  sample.dutyCycleBasisPoints = config->runtime->dutyCycleBasisPoints;
  sample.rawAngle = config->runtime->rawAngle;
  sample.angleCentiDegrees = config->runtime->angleCentiDegrees;
  sample.pwmPeriodMicros = config->runtime->pwmPeriodMicros;
  sample.sampleIntervalMicros = config->runtime->sampleIntervalMicros;
  sample.milliRpm = config->runtime->lastMilliRpm;
  sample.error = config->runtime->lastError;

  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
  return true;
}
