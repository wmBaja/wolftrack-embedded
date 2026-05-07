#include <Arduino.h>
#include <analog_rpm_sensor.h>

#include <string.h>

namespace {

const AnalogRpmSubSensorContext *GetAnalogRpmContext(const void *ctx) {
  return static_cast<const AnalogRpmSubSensorContext *>(ctx);
}

uint16_t AdcToRawAngle(const uint16_t rawAdc) {
  const uint32_t scaled =
      (static_cast<uint32_t>(rawAdc) * (kAnalogRpmCountsPerRevolution - 1U)) +
      (kAnalogRpmAdcMaxCounts / 2U);
  const uint32_t rawAngle = scaled / kAnalogRpmAdcMaxCounts;
  if (rawAngle >= kAnalogRpmCountsPerRevolution) {
    return kAnalogRpmCountsPerRevolution - 1U;
  }
  return static_cast<uint16_t>(rawAngle);
}

uint16_t RawAngleToCentiDegrees(const uint16_t rawAngle) {
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(rawAngle) * kAnalogRpmCentiDegreesPerRevolution) /
      kAnalogRpmCountsPerRevolution);
}

int32_t ComputeWrappedRawDelta(const uint16_t currentRawAngle,
                               const uint16_t previousRawAngle) {
  constexpr int32_t kHalfRevolutionCounts =
      kAnalogRpmCountsPerRevolution / 2;

  int32_t delta = static_cast<int32_t>(currentRawAngle) -
                  static_cast<int32_t>(previousRawAngle);
  if (delta > kHalfRevolutionCounts) {
    delta -= kAnalogRpmCountsPerRevolution;
  } else if (delta < -kHalfRevolutionCounts) {
    delta += kAnalogRpmCountsPerRevolution;
  }
  return delta;
}

int32_t ComputeMilliRpmFromDelta(const int32_t deltaRawAngle,
                                 const uint32_t deltaTimeMicros) {
  if (deltaTimeMicros == 0U) {
    return 0;
  }

  const float revolutions =
      static_cast<float>(deltaRawAngle) /
      static_cast<float>(kAnalogRpmCountsPerRevolution);
  const float rpm =
      (revolutions * 60000000.0f) / static_cast<float>(deltaTimeMicros);
  const float milliRpm = rpm * 1000.0f;
  return static_cast<int32_t>(milliRpm >= 0.0f ? milliRpm + 0.5f
                                                : milliRpm - 0.5f);
}

int32_t AbsoluteValue(const int32_t value) {
  return value >= 0 ? value : -value;
}

void ResetRuntime(AnalogRpmSensorRuntime &runtime) {
  runtime.initialized = true;
  runtime.hasPreviousSample = false;
  runtime.hasFilteredRpm = false;
  runtime.previousRawAngle = 0U;
  runtime.previousSampleAtMicros = 0U;
  runtime.lastError = kAnalogRpmSensorErrorNone;
  runtime.validMask = 0U;
  runtime.rawAdc = 0U;
  runtime.rawAngle = 0U;
  runtime.angleCentiDegrees = 0U;
  runtime.sampleIntervalMicros = 0U;
  runtime.lastRawMilliRpm = 0;
  runtime.lastFilteredMilliRpm = 0;
}

void UpdateRuntimeData(const AnalogRpmSubSensorContext &config) {
  AnalogRpmSensorRuntime *runtime = config.runtime;
  if (runtime == nullptr) {
    return;
  }

  runtime->rawAdc = static_cast<uint16_t>(analogRead(config.pin));
  runtime->rawAngle = AdcToRawAngle(runtime->rawAdc);
  runtime->angleCentiDegrees = RawAngleToCentiDegrees(runtime->rawAngle);

  const uint32_t now = micros();
  runtime->lastError = kAnalogRpmSensorErrorNone;
  runtime->validMask = kAnalogRpmSampleValidAngle;
  runtime->sampleIntervalMicros = 0U;
  runtime->lastRawMilliRpm = 0;

  if (!runtime->hasPreviousSample) {
    runtime->hasPreviousSample = true;
    runtime->previousRawAngle = runtime->rawAngle;
    runtime->previousSampleAtMicros = now;
    runtime->lastFilteredMilliRpm = 0;
    runtime->hasFilteredRpm = false;
    return;
  }

  runtime->sampleIntervalMicros =
      static_cast<uint32_t>(now - runtime->previousSampleAtMicros);

  if (runtime->sampleIntervalMicros == 0U) {
    runtime->lastError = kAnalogRpmSensorErrorZeroDeltaTime;
    runtime->lastFilteredMilliRpm = 0;
    runtime->hasFilteredRpm = false;
    runtime->previousRawAngle = runtime->rawAngle;
    runtime->previousSampleAtMicros = now;
    return;
  }

  if (runtime->sampleIntervalMicros > kAnalogRpmMaxSampleIntervalMicros) {
    runtime->lastError = kAnalogRpmSensorErrorSampleTooOld;
    runtime->lastFilteredMilliRpm = 0;
    runtime->hasFilteredRpm = false;
    runtime->previousRawAngle = runtime->rawAngle;
    runtime->previousSampleAtMicros = now;
    return;
  }

  const int32_t deltaRawAngle =
      ComputeWrappedRawDelta(runtime->rawAngle, runtime->previousRawAngle);
  if (AbsoluteValue(deltaRawAngle) <= kAnalogRpmZeroDeltaDeadbandCounts) {
    runtime->lastRawMilliRpm = 0;
  } else {
    runtime->lastRawMilliRpm =
        ComputeMilliRpmFromDelta(deltaRawAngle, runtime->sampleIntervalMicros);
  }

  if (!runtime->hasFilteredRpm) {
    runtime->lastFilteredMilliRpm = runtime->lastRawMilliRpm;
    runtime->hasFilteredRpm = true;
  } else {
    runtime->lastFilteredMilliRpm +=
        (runtime->lastRawMilliRpm - runtime->lastFilteredMilliRpm) / 4;
  }

  runtime->validMask = kAnalogRpmSampleValidAngle | kAnalogRpmSampleValidRpm;
  runtime->previousRawAngle = runtime->rawAngle;
  runtime->previousSampleAtMicros = now;
}

}  // namespace

bool AnalogRpmSensorBegin(const void *ctx) {
  const AnalogRpmSubSensorContext *config = GetAnalogRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  if (config->runtime->initialized) {
    return true;
  }

  pinMode(config->pin, INPUT);
  ResetRuntime(*config->runtime);
  return true;
}

bool AnalogRpmDataSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const AnalogRpmSubSensorContext *config = GetAnalogRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr ||
      !config->runtime->initialized) {
    AnalogRpmDataSampleFrame sample = {};
    sample.version = kAnalogRpmSampleFrameVersion;
    sample.error = kAnalogRpmSensorErrorNotInitialized;
    outFrame.len = sizeof(sample);
    memcpy(outFrame.data, &sample, sizeof(sample));
    return true;
  }

  UpdateRuntimeData(*config);

  AnalogRpmDataSampleFrame sample = {};
  sample.version = kAnalogRpmSampleFrameVersion;
  sample.validMask = config->runtime->validMask;
  sample.error = config->runtime->lastError;
  sample.milliRpm = config->runtime->lastFilteredMilliRpm;

  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
  return true;
}

bool AnalogRpmStatsSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const AnalogRpmSubSensorContext *config = GetAnalogRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr ||
      !config->runtime->initialized) {
    AnalogRpmStatsSampleFrame sample = {};
    sample.version = kAnalogRpmSampleFrameVersion;
    sample.error = kAnalogRpmSensorErrorNotInitialized;
    outFrame.len = sizeof(sample);
    memcpy(outFrame.data, &sample, sizeof(sample));
    return true;
  }

  UpdateRuntimeData(*config);

  AnalogRpmStatsSampleFrame sample = {};
  sample.version = kAnalogRpmSampleFrameVersion;
  sample.validMask = config->runtime->validMask;
  sample.rawAdc = config->runtime->rawAdc;
  sample.rawAngle = config->runtime->rawAngle;
  sample.angleCentiDegrees = config->runtime->angleCentiDegrees;
  sample.sampleIntervalMicros = config->runtime->sampleIntervalMicros;
  sample.rawMilliRpm = config->runtime->lastRawMilliRpm;
  sample.filteredMilliRpm = config->runtime->lastFilteredMilliRpm;
  sample.error = config->runtime->lastError;

  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
  return true;
}
