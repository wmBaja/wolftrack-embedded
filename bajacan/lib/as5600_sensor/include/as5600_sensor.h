#pragma once

#include <AS5600.h>
#include <config.h>
#include <stdint.h>

constexpr uint16_t kAS5600CountsPerRevolution = 4096U;
constexpr uint32_t kAS5600CentiDegreesPerRevolution = 36000UL;
constexpr uint16_t kAS5600MaxAngleCount = kAS5600CountsPerRevolution - 1U;

// Raw AS5600 status-register bit masks from the datasheet.
constexpr uint8_t kAS5600StatusMagnetTooStrong = 0x08U;
constexpr uint8_t kAS5600StatusMagnetTooWeak = 0x10U;
constexpr uint8_t kAS5600StatusMagnetDetected = 0x20U;

constexpr int16_t kAS5600SensorErrorNotInitialized = -1;

enum class AS5600AngleMapping : uint8_t {
  RawRotation = 0,
  CenteredOffset = 1,
};

constexpr uint16_t AS5600RawAngleToCentiDegrees(const uint16_t rawAngle) {
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(rawAngle & 0x0FFFU) *
       kAS5600CentiDegreesPerRevolution) /
      kAS5600CountsPerRevolution);
}

constexpr int32_t AS5600RawDelta(const uint16_t rawAngle, const uint16_t centerRawAngle) {
  return static_cast<int32_t>(rawAngle & 0x0FFFU) -
         static_cast<int32_t>(centerRawAngle & 0x0FFFU);
}

constexpr int16_t AS5600WrappedRawDelta(const uint16_t rawAngle, const uint16_t centerRawAngle) {
  int32_t delta = static_cast<int32_t>(rawAngle & 0x0FFFU) -
                  static_cast<int32_t>(centerRawAngle & 0x0FFFU);

  if (delta > static_cast<int32_t>(kAS5600CountsPerRevolution / 2U)) {
    delta -= static_cast<int32_t>(kAS5600CountsPerRevolution);
  } else if (delta < -static_cast<int32_t>(kAS5600CountsPerRevolution / 2U)) {
    delta += static_cast<int32_t>(kAS5600CountsPerRevolution);
  }

  return static_cast<int16_t>(delta);
}

constexpr int16_t AS5600RawAngleToCenteredCentiDegrees(
    const uint16_t rawAngle, const uint16_t centerRawAngle) {
  return static_cast<int16_t>(
      (static_cast<int32_t>(AS5600WrappedRawDelta(rawAngle, centerRawAngle)) *
       static_cast<int32_t>(kAS5600CentiDegreesPerRevolution)) /
      static_cast<int32_t>(kAS5600CountsPerRevolution));
}

constexpr uint8_t kAS5600SampleFrameVersion = 1U;

constexpr uint8_t kAS5600SampleValidMagnet = 0x02U;

struct __attribute__((packed)) AS5600DataSampleFrame {
  int16_t angleCentiDegrees;
};

struct __attribute__((packed)) AS5600StatsSampleFrame {
  uint8_t version;
  uint8_t validMask;
  uint16_t rawAngle;
  uint16_t magnitude;
  uint8_t agc;
  uint8_t status;
  int16_t error;
};

static_assert(sizeof(AS5600DataSampleFrame) <= 64,
              "AS5600 data frame must fit in one CAN FD payload");
static_assert(sizeof(AS5600StatsSampleFrame) <= 64,
              "AS5600 stats frame must fit in one CAN FD payload");
constexpr uint8_t kAS5600DataSensorPayloadSize =
    static_cast<uint8_t>(sizeof(AS5600DataSampleFrame));
constexpr uint8_t kAS5600StatsSensorPayloadSize =
    static_cast<uint8_t>(sizeof(AS5600StatsSampleFrame));

struct AS5600SensorRuntime {
  explicit AS5600SensorRuntime(TwoWire *wireBus = &Wire)
      : wire(wireBus), driver(wireBus) {}

  TwoWire *wire = &Wire;
  AS5600 driver;
  bool initialized = false;
};

struct AS5600SubSensorContext {
  SensorContext base;
  AS5600SensorRuntime *runtime;
  uint32_t clockHz;
  uint8_t directionPin;
  uint8_t direction;
  int32_t offsetCentiDegrees;
  uint16_t centerRawAngle;
  AS5600AngleMapping angleMapping;
};

bool AS5600SensorBegin(const void *ctx);
bool AS5600DataSensorSample(const void *ctx, CANFDMessage &outFrame);
bool AS5600StatsSensorSample(const void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor MakeAS5600DataSensor(
    const AS5600SubSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = AS5600SensorBegin,
      .sample = AS5600DataSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}

constexpr SensorDescriptor MakeAS5600StatsSensor(
    const AS5600SubSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = AS5600SensorBegin,
      .sample = AS5600StatsSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}
