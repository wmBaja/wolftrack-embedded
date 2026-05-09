#include <analog_rpm_sensor_internal.h>

#include <assert.h>

namespace {

AnalogRpmMotionFilterConfig MakeConfig(const uint8_t deadband,
                                       const uint8_t motionConfirm,
                                       const uint8_t zeroConfirm) {
  return AnalogRpmMotionFilterConfig{
      .zeroDeltaDeadbandCounts = deadband,
      .motionConfirmSamples = motionConfirm,
      .zeroConfirmSamples = zeroConfirm,
  };
}

AnalogRpmMotionFilterState MakeState() {
  AnalogRpmMotionFilterState state{};
  ResetAnalogRpmMotionFilterState(state);
  return state;
}

void TestStationaryJitterStaysAtZero() {
  AnalogRpmMotionFilterState state = MakeState();
  const AnalogRpmMotionFilterConfig config = MakeConfig(24U, 2U, 2U);
  const int32_t deltas[] = {12, -17, 13, -10, 24, -23};

  for (const int32_t delta : deltas) {
    UpdateAnalogRpmMotionFilter(config, delta, 0, state);
    assert(state.lastRawMilliRpm == 0);
    assert(state.lastFilteredMilliRpm == 0);
    assert(!state.motionConfirmed);
  }
}

void TestSingleOutlierDoesNotReportMotion() {
  AnalogRpmMotionFilterState state = MakeState();
  const AnalogRpmMotionFilterConfig config = MakeConfig(24U, 2U, 2U);
  const int32_t candidate = ComputeAnalogRpmFromDelta(32, 10000U);

  UpdateAnalogRpmMotionFilter(config, 32, candidate, state);
  assert(state.lastRawMilliRpm == 0);
  assert(state.lastFilteredMilliRpm == 0);
  assert(!state.motionConfirmed);

  UpdateAnalogRpmMotionFilter(config, 0, 0, state);
  assert(state.lastRawMilliRpm == 0);
  assert(state.lastFilteredMilliRpm == 0);
  assert(!state.motionConfirmed);
}

void TestTwoConfirmedSamplesEnterMotion() {
  AnalogRpmMotionFilterState state = MakeState();
  const AnalogRpmMotionFilterConfig config = MakeConfig(24U, 2U, 2U);
  const int32_t firstCandidate = ComputeAnalogRpmFromDelta(26, 10000U);
  const int32_t secondCandidate = ComputeAnalogRpmFromDelta(28, 10000U);

  UpdateAnalogRpmMotionFilter(config, 26, firstCandidate, state);
  assert(state.lastRawMilliRpm == 0);
  assert(state.lastFilteredMilliRpm == 0);
  assert(!state.motionConfirmed);

  UpdateAnalogRpmMotionFilter(config, 28, secondCandidate, state);
  assert(state.lastRawMilliRpm == secondCandidate);
  assert(state.lastFilteredMilliRpm == secondCandidate);
  assert(state.motionConfirmed);
}

void TestTwoZeroSamplesReturnToZero() {
  AnalogRpmMotionFilterState state = MakeState();
  const AnalogRpmMotionFilterConfig config = MakeConfig(24U, 2U, 2U);
  const int32_t firstCandidate = ComputeAnalogRpmFromDelta(27, 10000U);
  const int32_t secondCandidate = ComputeAnalogRpmFromDelta(29, 10000U);

  UpdateAnalogRpmMotionFilter(config, 27, firstCandidate, state);
  UpdateAnalogRpmMotionFilter(config, 29, secondCandidate, state);
  assert(state.motionConfirmed);
  assert(state.lastFilteredMilliRpm != 0);

  UpdateAnalogRpmMotionFilter(config, 8, 0, state);
  assert(state.lastRawMilliRpm == 0);
  assert(state.lastFilteredMilliRpm != 0);
  assert(state.motionConfirmed);

  UpdateAnalogRpmMotionFilter(config, -6, 0, state);
  assert(state.lastRawMilliRpm == 0);
  assert(state.lastFilteredMilliRpm == 0);
  assert(!state.motionConfirmed);
}

void TestWraparoundDeltaDirection() {
  const int32_t positiveWrap = ComputeAnalogRpmWrappedRawDelta(2U, 4090U);
  const int32_t negativeWrap = ComputeAnalogRpmWrappedRawDelta(4090U, 2U);
  assert(positiveWrap == 8);
  assert(negativeWrap == -8);

  AnalogRpmMotionFilterState state = MakeState();
  const AnalogRpmMotionFilterConfig config = MakeConfig(1U, 1U, 1U);
  const int32_t candidate = ComputeAnalogRpmFromDelta(positiveWrap, 10000U);
  UpdateAnalogRpmMotionFilter(config, positiveWrap, candidate, state);
  assert(state.motionConfirmed);
  assert(state.lastRawMilliRpm > 0);
  assert(state.lastFilteredMilliRpm > 0);
}

}  // namespace

int main() {
  TestStationaryJitterStaysAtZero();
  TestSingleOutlierDoesNotReportMotion();
  TestTwoConfirmedSamplesEnterMotion();
  TestTwoZeroSamplesReturnToZero();
  TestWraparoundDeltaDirection();
  return 0;
}
