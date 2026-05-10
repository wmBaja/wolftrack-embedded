#include <Arduino.h>
#include <analog_rpm_sensor.h>

#include <string.h>

namespace {

constexpr uint8_t kAnalogRpmMedianSampleCount = 5U;

const AnalogRpmSubSensorContext *GetAnalogRpmContext(const void *ctx) {
  return static_cast<const AnalogRpmSubSensorContext *>(ctx);
}

uint8_t GetZeroDeltaDeadbandCounts(const AnalogRpmSubSensorContext &config) {
  if (config.zeroDeltaDeadbandCounts == 0U) {
    return kAnalogRpmZeroDeltaDeadbandCounts;
  }
  return config.zeroDeltaDeadbandCounts;
}

uint8_t GetMotionConfirmSamples(const AnalogRpmSubSensorContext &config) {
  if (config.motionConfirmSamples == 0U) {
    return kAnalogRpmDefaultMotionConfirmSamples;
  }
  return config.motionConfirmSamples;
}

uint8_t GetZeroConfirmSamples(const AnalogRpmSubSensorContext &config) {
  if (config.zeroConfirmSamples == 0U) {
    return kAnalogRpmDefaultZeroConfirmSamples;
  }
  return config.zeroConfirmSamples;
}

void SortAscending(int32_t *values, const uint8_t count) {
  for (uint8_t i = 1U; i < count; ++i) {
    const int32_t key = values[i];
    uint8_t j = i;
    while (j > 0U && values[j - 1U] > key) {
      values[j] = values[j - 1U];
      --j;
    }
    values[j] = key;
  }
}

int32_t ReadSettledAdc(const uint8_t pin) {
  int32_t validSamples[kAnalogRpmMedianSampleCount] = {0};
  uint8_t validCount = 0U;
  int32_t lastReading = -1;

  for (uint8_t i = 0U; i < kAnalogRpmMedianSampleCount; ++i) {
    const int32_t reading = analogRead(pin);
    lastReading = reading;
    if (reading < 0) {
      continue;
    }
    validSamples[validCount] = reading;
    ++validCount;
  }

  if (validCount == 0U) {
    return lastReading;
  }

  SortAscending(validSamples, validCount);

  // If the spread of back-to-back samples is very large (>1024 counts), 
  // we are caught in the middle of a wraparound slew transition.
  // A median filter would actively select the worst mid-slew artifact.
  // Instead, bypass the slew by returning the boundary value.
  if (validCount > 1U && (validSamples[validCount - 1U] - validSamples[0]) > 1024) {
    return validSamples[0];
  }

  return validSamples[validCount / 2U];
}

AnalogRpmMotionFilterConfig MakeMotionFilterConfig(
    const AnalogRpmSubSensorContext &config) {
  return AnalogRpmMotionFilterConfig{
      .zeroDeltaDeadbandCounts = GetZeroDeltaDeadbandCounts(config),
      .motionConfirmSamples = GetMotionConfirmSamples(config),
      .zeroConfirmSamples = GetZeroConfirmSamples(config),
  };
}

AnalogRpmMotionFilterState MakeMotionFilterState(
    const AnalogRpmSensorRuntime &runtime) {
  return AnalogRpmMotionFilterState{
      .motionConfirmed = runtime.motionConfirmed,
      .hasFilteredRpm = runtime.hasFilteredRpm,
      .pendingMotionDirection = runtime.pendingMotionDirection,
      .consecutiveMotionCount = runtime.consecutiveMotionCount,
      .consecutiveZeroCount = runtime.consecutiveZeroCount,
      .lastRawMilliRpm = runtime.lastRawMilliRpm,
      .lastFilteredMilliRpm = runtime.lastFilteredMilliRpm,
  };
}

void ApplyMotionFilterState(const AnalogRpmMotionFilterState &state,
                            AnalogRpmSensorRuntime &runtime) {
  runtime.motionConfirmed = state.motionConfirmed;
  runtime.hasFilteredRpm = state.hasFilteredRpm;
  runtime.pendingMotionDirection = state.pendingMotionDirection;
  runtime.consecutiveMotionCount = state.consecutiveMotionCount;
  runtime.consecutiveZeroCount = state.consecutiveZeroCount;
  runtime.lastRawMilliRpm = state.lastRawMilliRpm;
  runtime.lastFilteredMilliRpm = state.lastFilteredMilliRpm;
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
  return ComputeAnalogRpmWrappedRawDelta(currentRawAngle, previousRawAngle);
}

int32_t ComputeMilliRpmFromDelta(const int32_t deltaRawAngle,
                                 const uint32_t deltaTimeMicros) {
  return ComputeAnalogRpmFromDelta(deltaRawAngle, deltaTimeMicros);
}

int32_t AbsoluteValue(const int32_t value) {
  return AnalogRpmAbsoluteValue(value);
}

void ResetRuntime(AnalogRpmSensorRuntime &runtime) {
  runtime.initialized = true;
  runtime.hasSample = false;
  runtime.hasFilteredRpm = false;
  runtime.motionConfirmed = false;
  runtime.previousRawAngle = 0U;
  runtime.previousSampleAtMicros = 0U;
  runtime.lastSampleAtMicros = 0U;
  runtime.pendingMotionDirection = 0;
  runtime.consecutiveMotionCount = 0U;
  runtime.consecutiveZeroCount = 0U;
  runtime.lastError = kAnalogRpmSensorErrorNone;
  runtime.validMask = 0U;
  runtime.rawAdc = 0U;
  runtime.rawAngle = 0U;
  runtime.angleCentiDegrees = 0U;
  runtime.sampleIntervalMicros = 0U;
  runtime.lastRawMilliRpm = 0;
  runtime.lastFilteredMilliRpm = 0;
  runtime.consecutiveRejectedSamples = 0U;
  runtime.accumulatedDeltaRawAngle = 0;
  runtime.accumulatedDeltaMicros = 0U;
}

void UpdateRuntimeData(const AnalogRpmSubSensorContext &config) {
  AnalogRpmSensorRuntime *runtime = config.runtime;
  if (runtime == nullptr) {
    return;
  }

  const uint32_t now = micros(); // Capture before ADC to reduce jitter
  const int32_t rawAdcReading = ReadSettledAdc(config.pin);

  runtime->lastError = kAnalogRpmSensorErrorNone;
  runtime->validMask = 0U;

  if (rawAdcReading < 0) {
    runtime->lastError = kAnalogRpmSensorErrorAdcReadFailed;
    runtime->sampleIntervalMicros = 0U;
    AnalogRpmMotionFilterState filterState = MakeMotionFilterState(*runtime);
    ResetAnalogRpmMotionFilterState(filterState);
    ApplyMotionFilterState(filterState, *runtime);
    return;
  }

  runtime->rawAdc = static_cast<uint16_t>(rawAdcReading);
  runtime->rawAngle = AdcToRawAngle(runtime->rawAdc);
  runtime->angleCentiDegrees = RawAngleToCentiDegrees(runtime->rawAngle);
  runtime->validMask = kAnalogRpmSampleValidAngle;
  runtime->lastSampleAtMicros = now;

  if (!runtime->hasSample) {
    runtime->hasSample = true;
    runtime->previousRawAngle = runtime->rawAngle;
    runtime->previousSampleAtMicros = now;
    runtime->sampleIntervalMicros = 0U;
    runtime->lastRawMilliRpm = 0;
    runtime->lastFilteredMilliRpm = 0;
    runtime->hasFilteredRpm = false;
    runtime->consecutiveRejectedSamples = 0U;
    runtime->accumulatedDeltaRawAngle = 0;
    runtime->accumulatedDeltaMicros = 0U;
    return;
  }

  runtime->sampleIntervalMicros =
      static_cast<uint32_t>(now - runtime->previousSampleAtMicros);

  if (runtime->sampleIntervalMicros == 0U) {
    runtime->lastError = kAnalogRpmSensorErrorZeroDeltaTime;
    AnalogRpmMotionFilterState filterState = MakeMotionFilterState(*runtime);
    ResetAnalogRpmMotionFilterState(filterState);
    ApplyMotionFilterState(filterState, *runtime);
    runtime->previousRawAngle = runtime->rawAngle;
    runtime->previousSampleAtMicros = now;
    return;
  }

  const int32_t deltaRawAngle =
      ComputeWrappedRawDelta(runtime->rawAngle, runtime->previousRawAngle);

  // Outlier rejection (Wraparound slew artifact filter)
  if (runtime->hasFilteredRpm) {
    // Calculate what the delta SHOULD be based on the current filtered RPM
    int64_t expected_delta = (static_cast<int64_t>(runtime->lastFilteredMilliRpm) * 
                              kAnalogRpmCountsPerRevolution * 
                              runtime->sampleIntervalMicros) / 60000000000LL;
    
    int64_t delta_error = static_cast<int64_t>(deltaRawAngle) - expected_delta;
    
    // If the actual delta deviates massively from the expected physical delta (e.g. > 90 degrees)
    if (AbsoluteValue(static_cast<int32_t>(delta_error)) > 1024) {
      if (runtime->consecutiveRejectedSamples < 3U) {
        runtime->consecutiveRejectedSamples++;
        runtime->lastError = kAnalogRpmSensorErrorAdcReadFailed;
        // Do NOT update previousRawAngle or previousSampleAtMicros to bridge over the artifact
        return;
      }
    }
  }
  runtime->consecutiveRejectedSamples = 0U;

  // Always update previous states for the next raw delta calculation
  runtime->previousRawAngle = runtime->rawAngle;
  runtime->previousSampleAtMicros = now;

  // Accumulate delta and time to prevent starvation at low speeds
  runtime->accumulatedDeltaRawAngle += deltaRawAngle;
  runtime->accumulatedDeltaMicros += runtime->sampleIntervalMicros;

  int32_t candidateMilliRpm = 0;
  bool processFilter = false;

  if (AbsoluteValue(runtime->accumulatedDeltaRawAngle) > GetZeroDeltaDeadbandCounts(config)) {
    candidateMilliRpm = ComputeMilliRpmFromDelta(runtime->accumulatedDeltaRawAngle, runtime->accumulatedDeltaMicros);
    processFilter = true;
  } else if (runtime->accumulatedDeltaMicros > 100000UL) { 
    // 100ms timeout for zero speed
    candidateMilliRpm = 0;
    processFilter = true;
  }

  if (processFilter) {
    AnalogRpmMotionFilterState filterState = MakeMotionFilterState(*runtime);
    UpdateAnalogRpmMotionFilter(MakeMotionFilterConfig(config), runtime->accumulatedDeltaRawAngle,
                                candidateMilliRpm, filterState);
    ApplyMotionFilterState(filterState, *runtime);

    runtime->accumulatedDeltaRawAngle = 0;
    runtime->accumulatedDeltaMicros = 0U;
  }

  if (runtime->sampleIntervalMicros > kAnalogRpmMaxSampleIntervalMicros) {
    runtime->lastError = kAnalogRpmSensorErrorSampleTooOld;
  }

  runtime->validMask = kAnalogRpmSampleValidAngle | kAnalogRpmSampleValidRpm;
}

void CopyDataFrame(const AnalogRpmSensorRuntime &runtime,
                   AnalogRpmDataSampleFrame &sample) {
  sample.version = kAnalogRpmSampleFrameVersion;
  sample.validMask = runtime.validMask;
  sample.error = runtime.lastError;
  sample.milliRpm = runtime.lastFilteredMilliRpm;
}

void CopyStatsFrame(const AnalogRpmSensorRuntime &runtime,
                    AnalogRpmStatsSampleFrame &sample) {
  sample.version = kAnalogRpmSampleFrameVersion;
  sample.validMask = runtime.validMask;
  sample.rawAdc = runtime.rawAdc;
  sample.rawAngle = runtime.rawAngle;
  sample.angleCentiDegrees = runtime.angleCentiDegrees;
  sample.sampleIntervalMicros = runtime.sampleIntervalMicros;
  sample.rawMilliRpm = runtime.lastRawMilliRpm;
  sample.filteredMilliRpm = runtime.lastFilteredMilliRpm;
  sample.error = runtime.lastError;
}

void ApplyCachedSampleFreshness(const uint32_t now,
                                AnalogRpmStatsSampleFrame &sample,
                                const AnalogRpmSensorRuntime &runtime) {
  if (!runtime.hasSample) {
    sample.validMask = 0U;
    sample.error = kAnalogRpmSensorErrorSampleTooOld;
    sample.sampleIntervalMicros = 0U;
    sample.rawMilliRpm = 0;
    sample.filteredMilliRpm = 0;
    return;
  }

  if (runtime.lastError == kAnalogRpmSensorErrorAdcReadFailed) {
    return;
  }

  const uint32_t sampleAgeMicros = now - runtime.lastSampleAtMicros;
  if (sampleAgeMicros <= kAnalogRpmMaxSampleIntervalMicros) {
    return;
  }

  sample.error = kAnalogRpmSensorErrorSampleTooOld;
  sample.validMask &= static_cast<uint8_t>(~kAnalogRpmSampleValidRpm);
  sample.rawMilliRpm = 0;
  sample.filteredMilliRpm = 0;
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
  CopyDataFrame(*config->runtime, sample);

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

  AnalogRpmStatsSampleFrame sample = {};
  CopyStatsFrame(*config->runtime, sample);
  ApplyCachedSampleFreshness(micros(), sample, *config->runtime);

  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
  return true;
}
