#include <Arduino.h>
#include <pulse_rpm_sensor.h>

#include <string.h>

namespace {

constexpr size_t kMaxPulseRpmSensors = 4U;
constexpr uint32_t kPulseRpmOutlierPercent = 50U;
constexpr uint32_t kPulseRpmStepChangePercent = 15U;

PulseRpmSensorRuntime *gPulseRpmRuntimes[kMaxPulseRpmSensors] = {nullptr};
PulseRpmSubSensorContext const *gPulseRpmConfigs[kMaxPulseRpmSensors] = {
    nullptr};

struct PulseRpmIsrSnapshot {
  uint32_t lastAcceptedEdgeMicros;
  uint32_t lastPulseIntervalMicros;
  uint32_t acceptedPulseCount;
  uint32_t rejectedPulseCount;
};

const PulseRpmSubSensorContext *GetPulseRpmContext(const void *ctx) {
  return static_cast<const PulseRpmSubSensorContext *>(ctx);
}

uint32_t GetMilliRevolutionsPerPulse(const PulseRpmSubSensorContext &config) {
  if (config.milliRevolutionsPerPulse == 0U) {
    return kPulseRpmDefaultMilliRevolutionsPerPulse;
  }
  return config.milliRevolutionsPerPulse;
}

uint32_t GetStaleAfterMicros(const PulseRpmSubSensorContext &config) {
  if (config.staleAfterMicros == 0U) {
    return kPulseRpmDefaultStaleAfterMicros;
  }
  return config.staleAfterMicros;
}

uint32_t GetMinPulseSpacingMicros(const PulseRpmSubSensorContext &config) {
  if (config.minPulseSpacingMicros == 0U) {
    return kPulseRpmDefaultMinPulseSpacingMicros;
  }
  return config.minPulseSpacingMicros;
}

uint32_t ComputeAverageInterval(const PulseRpmSensorRuntime &runtime) {
  if (runtime.acceptedPulseIntervalCount == 0U) {
    return 0U;
  }

  return (runtime.acceptedPulseIntervalSum +
          (runtime.acceptedPulseIntervalCount / 2U)) /
         runtime.acceptedPulseIntervalCount;
}

void ResetAcceptedIntervalHistory(PulseRpmSensorRuntime &runtime) {
  memset(runtime.acceptedPulseIntervals, 0, sizeof(runtime.acceptedPulseIntervals));
  runtime.acceptedPulseIntervalSum = 0U;
  runtime.acceptedPulseIntervalHead = 0U;
  runtime.acceptedPulseIntervalCount = 0U;
  runtime.averagedPulseIntervalMicros = 0U;
}

void ClearPendingOutlier(PulseRpmSensorRuntime &runtime) {
  runtime.pendingOutlierIntervalMicros = 0U;
  runtime.hasPendingOutlier = false;
}

void PushAcceptedInterval(PulseRpmSensorRuntime &runtime,
                          const uint32_t intervalMicros) {
  if (runtime.acceptedPulseIntervalCount <
      kPulseRpmIntervalAverageWindow) {
    runtime.acceptedPulseIntervals[runtime.acceptedPulseIntervalHead] =
        intervalMicros;
    runtime.acceptedPulseIntervalSum += intervalMicros;
    ++runtime.acceptedPulseIntervalCount;
  } else {
    runtime.acceptedPulseIntervalSum -=
        runtime.acceptedPulseIntervals[runtime.acceptedPulseIntervalHead];
    runtime.acceptedPulseIntervals[runtime.acceptedPulseIntervalHead] =
        intervalMicros;
    runtime.acceptedPulseIntervalSum += intervalMicros;
  }

  runtime.acceptedPulseIntervalHead =
      static_cast<uint8_t>((runtime.acceptedPulseIntervalHead + 1U) %
                           kPulseRpmIntervalAverageWindow);
  runtime.averagedPulseIntervalMicros = ComputeAverageInterval(runtime);
}

void SeedAcceptedIntervals(PulseRpmSensorRuntime &runtime, const uint32_t first,
                           const uint32_t second) {
  ResetAcceptedIntervalHistory(runtime);
  PushAcceptedInterval(runtime, first);
  PushAcceptedInterval(runtime, second);
}

bool IsWithinPercentWindow(const uint32_t candidate, const uint32_t reference,
                           const uint32_t percent) {
  if (reference == 0U) {
    return candidate == 0U;
  }

  const uint64_t tolerance =
      (static_cast<uint64_t>(reference) * percent + 99ULL) / 100ULL;
  const uint64_t lowerBound =
      reference > tolerance ? reference - tolerance : 0ULL;
  const uint64_t upperBound = static_cast<uint64_t>(reference) + tolerance;
  return static_cast<uint64_t>(candidate) >= lowerBound &&
         static_cast<uint64_t>(candidate) <= upperBound;
}

void ResetRuntime(PulseRpmSensorRuntime &runtime) {
  runtime.initialized = false;
  runtime.initializationError = kPulseRpmSensorErrorNotInitialized;
  runtime.isrLastAcceptedEdgeMicros = 0U;
  runtime.isrLastPulseIntervalMicros = 0U;
  runtime.isrAcceptedPulseCount = 0U;
  runtime.isrRejectedPulseCount = 0U;
  runtime.lastProcessedAcceptedPulseCount = 0U;
  runtime.filterRejectedPulseCount = 0U;
  ResetAcceptedIntervalHistory(runtime);
  ClearPendingOutlier(runtime);
  runtime.hasValidInterval = false;
  runtime.validMask = 0U;
  runtime.pulseIntervalMicros = 0U;
  runtime.sampleAgeMicros = 0U;
  runtime.acceptedPulseCount = 0U;
  runtime.rejectedPulseCount = 0U;
  runtime.lastMilliRpm = 0;
  runtime.lastError = kPulseRpmSensorErrorNotInitialized;
}

void PulseRpmIsr(const size_t index) {
  PulseRpmSensorRuntime *runtime = gPulseRpmRuntimes[index];
  const PulseRpmSubSensorContext *config = gPulseRpmConfigs[index];
  if (runtime == nullptr || config == nullptr) {
    return;
  }

  const uint32_t now = micros();
  const uint32_t acceptedCount = runtime->isrAcceptedPulseCount;
  if (acceptedCount != 0U) {
    const uint32_t deltaMicros = now - runtime->isrLastAcceptedEdgeMicros;
    if (deltaMicros < GetMinPulseSpacingMicros(*config)) {
      ++runtime->isrRejectedPulseCount;
      return;
    }

    runtime->isrLastPulseIntervalMicros = deltaMicros;
  }

  runtime->isrLastAcceptedEdgeMicros = now;
  runtime->isrAcceptedPulseCount = acceptedCount + 1U;
}

void PulseRpmIsr0() { PulseRpmIsr(0U); }
void PulseRpmIsr1() { PulseRpmIsr(1U); }
void PulseRpmIsr2() { PulseRpmIsr(2U); }
void PulseRpmIsr3() { PulseRpmIsr(3U); }

void (*const gPulseRpmIsrStubs[kMaxPulseRpmSensors])() = {
    PulseRpmIsr0,
    PulseRpmIsr1,
    PulseRpmIsr2,
    PulseRpmIsr3,
};

PulseRpmIsrSnapshot CopySnapshot(const PulseRpmSensorRuntime &runtime) {
  PulseRpmIsrSnapshot snapshot = {};
  noInterrupts();
  snapshot.lastAcceptedEdgeMicros = runtime.isrLastAcceptedEdgeMicros;
  snapshot.lastPulseIntervalMicros = runtime.isrLastPulseIntervalMicros;
  snapshot.acceptedPulseCount = runtime.isrAcceptedPulseCount;
  snapshot.rejectedPulseCount = runtime.isrRejectedPulseCount;
  interrupts();
  return snapshot;
}

int32_t ComputeMilliRpm(const PulseRpmSubSensorContext &config,
                        const uint32_t pulseIntervalMicros) {
  if (pulseIntervalMicros == 0U) {
    return 0;
  }

  const uint64_t numerator =
      60000000ULL * static_cast<uint64_t>(GetMilliRevolutionsPerPulse(config));
  const uint64_t milliRpm =
      (numerator + (pulseIntervalMicros / 2U)) / pulseIntervalMicros;
  return static_cast<int32_t>(milliRpm);
}

void UpdateAcceptedRpm(const PulseRpmSubSensorContext &config,
                       PulseRpmSensorRuntime &runtime,
                       const uint32_t intervalMicros) {
  PushAcceptedInterval(runtime, intervalMicros);
  runtime.hasValidInterval = runtime.acceptedPulseIntervalCount > 0U;
  runtime.lastMilliRpm =
      ComputeMilliRpm(config, runtime.averagedPulseIntervalMicros);
  ClearPendingOutlier(runtime);
}

void HandleNewPulseInterval(const PulseRpmSubSensorContext &config,
                            PulseRpmSensorRuntime &runtime,
                            const uint32_t intervalMicros) {
  runtime.pulseIntervalMicros = intervalMicros;

  if (intervalMicros == 0U) {
    return;
  }

  if (runtime.acceptedPulseIntervalCount == 0U) {
    UpdateAcceptedRpm(config, runtime, intervalMicros);
    return;
  }

  const uint32_t averageInterval = runtime.averagedPulseIntervalMicros;
  const bool matchesAverage =
      IsWithinPercentWindow(intervalMicros, averageInterval,
                            kPulseRpmOutlierPercent);
  if (matchesAverage) {
    if (runtime.hasPendingOutlier) {
      ++runtime.filterRejectedPulseCount;
      ClearPendingOutlier(runtime);
    }

    UpdateAcceptedRpm(config, runtime, intervalMicros);
    return;
  }

  if (!runtime.hasPendingOutlier) {
    runtime.pendingOutlierIntervalMicros = intervalMicros;
    runtime.hasPendingOutlier = true;
    return;
  }

  const bool matchesPending = IsWithinPercentWindow(
      intervalMicros, runtime.pendingOutlierIntervalMicros,
      kPulseRpmStepChangePercent);
  if (matchesPending) {
    SeedAcceptedIntervals(runtime, runtime.pendingOutlierIntervalMicros,
                          intervalMicros);
    runtime.hasValidInterval = true;
    runtime.lastMilliRpm =
        ComputeMilliRpm(config, runtime.averagedPulseIntervalMicros);
    ClearPendingOutlier(runtime);
    return;
  }

  ++runtime.filterRejectedPulseCount;
  runtime.pendingOutlierIntervalMicros = intervalMicros;
}

void UpdateRuntimeData(const PulseRpmSubSensorContext &config) {
  PulseRpmSensorRuntime *runtime = config.runtime;
  if (runtime == nullptr) {
    return;
  }

  const uint32_t now = micros();
  const PulseRpmIsrSnapshot snapshot = CopySnapshot(*runtime);
  const uint32_t staleAfterMicros = GetStaleAfterMicros(config);

  runtime->acceptedPulseCount = snapshot.acceptedPulseCount;
  runtime->rejectedPulseCount =
      snapshot.rejectedPulseCount + runtime->filterRejectedPulseCount;
  runtime->sampleAgeMicros = snapshot.acceptedPulseCount == 0U
                                 ? 0U
                                 : now - snapshot.lastAcceptedEdgeMicros;

  if (snapshot.acceptedPulseCount > runtime->lastProcessedAcceptedPulseCount) {
    runtime->lastProcessedAcceptedPulseCount = snapshot.acceptedPulseCount;
    if (snapshot.acceptedPulseCount >= 2U) {
      HandleNewPulseInterval(config, *runtime, snapshot.lastPulseIntervalMicros);
    } else {
      runtime->pulseIntervalMicros = 0U;
    }

    runtime->rejectedPulseCount =
        snapshot.rejectedPulseCount + runtime->filterRejectedPulseCount;
  }

  runtime->validMask = 0U;
  runtime->lastError = kPulseRpmSensorErrorInsufficientSamples;

  if (snapshot.acceptedPulseCount == 0U) {
    runtime->pulseIntervalMicros = 0U;
    runtime->lastMilliRpm = 0;
    return;
  }

  runtime->validMask = kPulseRpmSampleValidTiming;
  if (snapshot.acceptedPulseCount < 2U) {
    runtime->pulseIntervalMicros = 0U;
    runtime->lastMilliRpm = 0;
    runtime->hasValidInterval = false;
    ResetAcceptedIntervalHistory(*runtime);
    ClearPendingOutlier(*runtime);
    if (runtime->sampleAgeMicros > staleAfterMicros) {
      runtime->lastError = kPulseRpmSensorErrorStaleTimeout;
    }
    return;
  }

  if (!runtime->hasValidInterval) {
    runtime->lastError = kPulseRpmSensorErrorInsufficientSamples;
    return;
  }

  if (runtime->sampleAgeMicros > staleAfterMicros) {
    runtime->lastError = kPulseRpmSensorErrorStaleTimeout;
    runtime->lastMilliRpm = 0;
    return;
  }

  runtime->validMask |= kPulseRpmSampleValidRpm;
  runtime->lastError = kPulseRpmSensorErrorNone;
}

void CopyDataFrame(const PulseRpmSensorRuntime &runtime,
                   PulseRpmDataSampleFrame &sample) {
  sample.version = kPulseRpmSampleFrameVersion;
  sample.validMask = runtime.validMask;
  sample.error = runtime.lastError;
  sample.milliRpm = runtime.lastMilliRpm;
}

void CopyStatsFrame(const PulseRpmSensorRuntime &runtime,
                    PulseRpmStatsSampleFrame &sample) {
  sample.version = kPulseRpmSampleFrameVersion;
  sample.validMask = runtime.validMask;
  sample.pulseIntervalMicros = runtime.pulseIntervalMicros;
  sample.averagedPulseIntervalMicros = runtime.averagedPulseIntervalMicros;
  sample.pendingOutlierIntervalMicros = runtime.pendingOutlierIntervalMicros;
  sample.sampleAgeMicros = runtime.sampleAgeMicros;
  sample.acceptedPulseCount = runtime.acceptedPulseCount;
  sample.rejectedPulseCount = runtime.rejectedPulseCount;
  sample.milliRpm = runtime.lastMilliRpm;
  sample.error = runtime.lastError;
}

void FillUninitializedDataFrame(const PulseRpmSensorRuntime *runtime,
                                PulseRpmDataSampleFrame &sample) {
  sample.version = kPulseRpmSampleFrameVersion;
  sample.validMask = 0U;
  sample.error = runtime != nullptr ? runtime->initializationError
                                    : kPulseRpmSensorErrorNotInitialized;
  sample.milliRpm = 0;
}

void FillUninitializedStatsFrame(const PulseRpmSensorRuntime *runtime,
                                 PulseRpmStatsSampleFrame &sample) {
  sample.version = kPulseRpmSampleFrameVersion;
  sample.validMask = 0U;
  sample.pulseIntervalMicros = 0U;
  sample.averagedPulseIntervalMicros = 0U;
  sample.pendingOutlierIntervalMicros = 0U;
  sample.sampleAgeMicros = 0U;
  sample.acceptedPulseCount = 0U;
  sample.rejectedPulseCount = 0U;
  sample.milliRpm = 0;
  sample.error = runtime != nullptr ? runtime->initializationError
                                    : kPulseRpmSensorErrorNotInitialized;
}

}  // namespace

bool PulseRpmSensorBegin(const void *ctx) {
  const PulseRpmSubSensorContext *config = GetPulseRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  if (config->runtime->initialized) {
    return true;
  }

  ResetRuntime(*config->runtime);
  pinMode(config->pin, INPUT);

  const int8_t interruptNumber = digitalPinToInterrupt(config->pin);
  if (interruptNumber < 0) {
    config->runtime->initializationError =
        kPulseRpmSensorErrorUnsupportedInterruptPin;
    config->runtime->lastError = config->runtime->initializationError;
    return false;
  }

  int slot = -1;
  for (size_t i = 0; i < kMaxPulseRpmSensors; ++i) {
    if (gPulseRpmRuntimes[i] == nullptr || gPulseRpmRuntimes[i] == config->runtime) {
      slot = static_cast<int>(i);
      break;
    }
  }

  if (slot < 0) {
    config->runtime->initializationError = kPulseRpmSensorErrorNoIsrSlotAvailable;
    config->runtime->lastError = config->runtime->initializationError;
    return false;
  }

  gPulseRpmRuntimes[slot] = config->runtime;
  gPulseRpmConfigs[slot] = config;
  attachInterrupt(interruptNumber, gPulseRpmIsrStubs[slot],
                  config->countRisingEdge ? RISING : FALLING);

  config->runtime->initialized = true;
  config->runtime->initializationError = kPulseRpmSensorErrorNone;
  config->runtime->lastError = kPulseRpmSensorErrorInsufficientSamples;
  return true;
}

bool PulseRpmDataSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const PulseRpmSubSensorContext *config = GetPulseRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  PulseRpmDataSampleFrame sample = {};
  if (!config->runtime->initialized) {
    FillUninitializedDataFrame(config->runtime, sample);
  } else {
    UpdateRuntimeData(*config);
    CopyDataFrame(*config->runtime, sample);
  }

  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
  return true;
}

bool PulseRpmStatsSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const PulseRpmSubSensorContext *config = GetPulseRpmContext(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  PulseRpmStatsSampleFrame sample = {};
  if (!config->runtime->initialized) {
    FillUninitializedStatsFrame(config->runtime, sample);
  } else {
    UpdateRuntimeData(*config);
    CopyStatsFrame(*config->runtime, sample);
  }

  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
  return true;
}
