#pragma once

#include <AS5600.h>
#include <config.h>
#include <stdint.h>

constexpr uint16_t kAS5600CountsPerRevolution = 4096U;
constexpr uint32_t kAS5600CentiDegreesPerRevolution = 36000UL;

// Raw AS5600 status-register bit masks from the datasheet.
constexpr uint8_t kAS5600StatusMagnetTooStrong = 0x08U;
constexpr uint8_t kAS5600StatusMagnetTooWeak = 0x10U;
constexpr uint8_t kAS5600StatusMagnetDetected = 0x20U;

constexpr int16_t kAS5600SensorErrorNotInitialized = -1;

constexpr uint16_t AS5600RawAngleToCentiDegrees(const uint16_t rawAngle) {
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(rawAngle & 0x0FFFU) *
       kAS5600CentiDegreesPerRevolution) /
      kAS5600CountsPerRevolution);
}

struct __attribute__((packed)) AS5600SampleFrame {
  uint16_t rawAngle;
  uint16_t angleCentiDegrees;
  uint16_t magnitude;
  uint8_t agc;
  uint8_t status;
  int16_t error;
};

static_assert(sizeof(AS5600SampleFrame) <= 64,
              "AS5600 sample frame must fit in one CAN FD payload");
constexpr uint8_t kAS5600SensorPayloadSize =
    static_cast<uint8_t>(sizeof(AS5600SampleFrame));

struct AS5600SensorRuntime {
  explicit AS5600SensorRuntime(TwoWire *wireBus = &Wire)
      : wire(wireBus), driver(wireBus) {}

  TwoWire *wire = &Wire;
  AS5600 driver;
  bool initialized = false;
};

struct AS5600SensorContext {
  SensorContext base;
  AS5600SensorRuntime *runtime;
  uint32_t clockHz;
  uint8_t directionPin;
  uint8_t direction;
  int32_t offsetCentiDegrees;
};

bool AS5600SensorBegin(const void *ctx);
bool AS5600SensorSample(const void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor MakeAS5600Sensor(const AS5600SensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = AS5600SensorBegin,
      .sample = AS5600SensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}
