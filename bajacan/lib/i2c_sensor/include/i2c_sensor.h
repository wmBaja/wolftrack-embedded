#pragma once

#include <config.h>
#include <stddef.h>
#include <stdint.h>

constexpr uint8_t kI2cSampleFrameVersion = 1;
constexpr uint8_t kI2cSamplesPerFrame = 2;

struct __attribute__((packed)) I2cSampleRecord {
  uint16_t dt_ms;
  int32_t ch1_raw;
  int32_t ch2_raw;
};

struct __attribute__((packed)) I2cSampleFrame {
  uint8_t version;
  uint8_t sample_count;
  uint16_t sequence;
  I2cSampleRecord samples[kI2cSamplesPerFrame];
};

static_assert(sizeof(I2cSampleFrame) <= 64,
              "I2C sample frame must fit in one I2C transaction");

struct __attribute__((packed)) I2cCanSampleFrame {
  float ch1_voltage;
  uint32_t raw_count;
};

constexpr uint8_t kI2cCanSampleFrameVersion = 2U;
constexpr uint16_t kI2cCanBootSequence = 0xB007U;
constexpr uint32_t kI2cCanBootRawCount = 0x00C0DE03UL;

static_assert(sizeof(I2cCanSampleFrame) == 8,
              "I2C CAN sample frame must be exactly 8 bytes");
constexpr uint8_t kI2cSensorPayloadSize =
    static_cast<uint8_t>(sizeof(I2cCanSampleFrame));

struct I2CSensorContext {
  SensorContext base;
  uint8_t addr;
  uint32_t clockHz;
};

bool I2CSensorBegin(const void *ctx);
bool I2CSensorSample(const void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor MakeI2CSensor(const I2CSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = I2CSensorBegin,
      .sample = I2CSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}
