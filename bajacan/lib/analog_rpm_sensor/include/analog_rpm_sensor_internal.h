#pragma once

#include <stdint.h>

constexpr uint16_t kAnalogRpmInternalCountsPerRevolution = 4096U;
constexpr uint8_t kAnalogRpmInternalDefaultMotionConfirmSamples = 1U;
constexpr uint8_t kAnalogRpmInternalDefaultZeroConfirmSamples = 1U;

struct AnalogRpmMotionFilterConfig {
  uint8_t zeroDeltaDeadbandCounts;
  uint8_t motionConfirmSamples;
  uint8_t zeroConfirmSamples;
};

struct AnalogRpmMotionFilterState {
  bool motionConfirmed;
  bool hasFilteredRpm;
  int8_t pendingMotionDirection;
  uint8_t consecutiveMotionCount;
  uint8_t consecutiveZeroCount;
  int32_t lastRawMilliRpm;
  int32_t lastFilteredMilliRpm;
};

constexpr uint8_t NormalizeAnalogRpmSampleCount(const uint8_t configured,
                                                const uint8_t fallback) {
  return configured == 0U ? fallback : configured;
}

constexpr int32_t ComputeAnalogRpmWrappedRawDelta(const uint16_t currentRawAngle,
                                                  const uint16_t previousRawAngle) {
  constexpr int32_t kHalfRevolutionCounts =
      kAnalogRpmInternalCountsPerRevolution / 2;

  int32_t delta = static_cast<int32_t>(currentRawAngle) -
                  static_cast<int32_t>(previousRawAngle);
  if (delta > kHalfRevolutionCounts) {
    delta -= kAnalogRpmInternalCountsPerRevolution;
  } else if (delta < -kHalfRevolutionCounts) {
    delta += kAnalogRpmInternalCountsPerRevolution;
  }
  return delta;
}

inline int32_t ComputeAnalogRpmFromDelta(const int32_t deltaRawAngle,
                                         const uint32_t deltaTimeMicros) {
  if (deltaTimeMicros == 0U) {
    return 0;
  }

  const float revolutions =
      static_cast<float>(deltaRawAngle) /
      static_cast<float>(kAnalogRpmInternalCountsPerRevolution);
  const float rpm =
      (revolutions * 60000000.0f) / static_cast<float>(deltaTimeMicros);
  const float milliRpm = rpm * 1000.0f;
  return static_cast<int32_t>(milliRpm >= 0.0f ? milliRpm + 0.5f
                                                : milliRpm - 0.5f);
}

constexpr int32_t AnalogRpmAbsoluteValue(const int32_t value) {
  return value >= 0 ? value : -value;
}

constexpr int8_t ComputeAnalogRpmDirection(const int32_t deltaRawAngle) {
  return deltaRawAngle > 0 ? 1 : (deltaRawAngle < 0 ? -1 : 0);
}

constexpr int32_t ApplyAnalogRpmIirFilter(const int32_t filteredMilliRpm,
                                          const int32_t rawMilliRpm) {
  return filteredMilliRpm + ((rawMilliRpm - filteredMilliRpm) / 4);
}

inline void ResetAnalogRpmMotionFilterState(AnalogRpmMotionFilterState &state) {
  state.motionConfirmed = false;
  state.hasFilteredRpm = false;
  state.pendingMotionDirection = 0;
  state.consecutiveMotionCount = 0U;
  state.consecutiveZeroCount = 0U;
  state.lastRawMilliRpm = 0;
  state.lastFilteredMilliRpm = 0;
}

inline void UpdateAnalogRpmMotionFilter(
    const AnalogRpmMotionFilterConfig &config, const int32_t deltaRawAngle,
    const int32_t candidateMilliRpm, AnalogRpmMotionFilterState &state) {
  const uint8_t motionConfirmSamples = NormalizeAnalogRpmSampleCount(
      config.motionConfirmSamples, kAnalogRpmInternalDefaultMotionConfirmSamples);
  const uint8_t zeroConfirmSamples = NormalizeAnalogRpmSampleCount(
      config.zeroConfirmSamples, kAnalogRpmInternalDefaultZeroConfirmSamples);

  if (AnalogRpmAbsoluteValue(deltaRawAngle) <= config.zeroDeltaDeadbandCounts) {
    state.lastRawMilliRpm = 0;
    state.consecutiveMotionCount = 0U;
    state.pendingMotionDirection = 0;
    if (state.consecutiveZeroCount < zeroConfirmSamples) {
      ++state.consecutiveZeroCount;
    }

    if (state.motionConfirmed &&
        state.consecutiveZeroCount < zeroConfirmSamples) {
      return;
    }

    ResetAnalogRpmMotionFilterState(state);
    return;
  }

  state.consecutiveZeroCount = 0U;
  const int8_t direction = ComputeAnalogRpmDirection(deltaRawAngle);
  if (!state.motionConfirmed) {
    if (state.pendingMotionDirection != direction) {
      state.pendingMotionDirection = direction;
      state.consecutiveMotionCount = 1U;
    } else if (state.consecutiveMotionCount < motionConfirmSamples) {
      ++state.consecutiveMotionCount;
    }

    if (state.consecutiveMotionCount < motionConfirmSamples) {
      state.lastRawMilliRpm = 0;
      state.lastFilteredMilliRpm = 0;
      state.hasFilteredRpm = false;
      return;
    }

    state.motionConfirmed = true;
    state.pendingMotionDirection = 0;
    state.consecutiveMotionCount = 0U;
  }

  state.lastRawMilliRpm = candidateMilliRpm;
  if (!state.hasFilteredRpm) {
    state.lastFilteredMilliRpm = candidateMilliRpm;
    state.hasFilteredRpm = true;
  } else {
    state.lastFilteredMilliRpm =
        ApplyAnalogRpmIirFilter(state.lastFilteredMilliRpm, candidateMilliRpm);
  }
}
