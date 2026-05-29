#include <Arduino.h>
#include <Wire.h>
#include <i2c_sensor.h>

#include <string.h>

namespace {

constexpr uint32_t kAdcRaw24BitMask = 0x00FFFFFFUL;
constexpr uint32_t kI2cDiagnosticRawBase = 0x00BAE000UL;
constexpr uint8_t kBootDiagnosticSampleCount = 100U;
constexpr uint8_t kI2cFrameChunkSize = 12U;
static_assert((sizeof(I2cSampleFrame) % kI2cFrameChunkSize) == 0,
              "I2C sample frame chunks must divide the full frame evenly");

enum class I2cReadStatus : uint16_t {
  Ok = 0U,
  Boot = kI2cCanBootSequence,
  ShortRequest = 1U,
  ShortRead = 2U,
  BadVersion = 3U,
  BadSampleCount = 4U,
};

struct I2cReadResult {
  I2cReadStatus status;
  uint8_t detail;
};

const I2CSensorContext *GetI2CContext(const void *ctx) {
  return static_cast<const I2CSensorContext *>(ctx);
}

uint32_t RawCountToUint24(const int32_t raw) {
  return static_cast<uint32_t>(raw) & kAdcRaw24BitMask;
}

void CopyCanFrame(const I2cCanSampleFrame &canFrame, CANFDMessage &outFrame) {
  outFrame.len = sizeof(canFrame);
  memcpy(outFrame.data, &canFrame, sizeof(canFrame));
}

void CopyDiagnosticFrame(const I2cReadStatus status, CANFDMessage &outFrame,
                         const uint8_t detail = 0U) {
  const uint16_t statusCode = static_cast<uint16_t>(status);
  const I2cCanSampleFrame canFrame = {
      .ch1_voltage = 0.0F,
      .raw_count =
          (status == I2cReadStatus::Boot)
              ? kI2cCanBootRawCount
              : static_cast<uint32_t>(kI2cDiagnosticRawBase |
                                      (static_cast<uint32_t>(statusCode) << 8) |
                                      detail),
  };
  CopyCanFrame(canFrame, outFrame);
}

I2cReadResult ReadFrame(const I2CSensorContext &config, I2cSampleFrame &frame) {
  uint8_t *dst = reinterpret_cast<uint8_t *>(&frame);
  for (uint8_t offset = 0U; offset < sizeof(frame);
       offset += kI2cFrameChunkSize) {
    const size_t bytesRequested =
        Wire.requestFrom(config.addr, kI2cFrameChunkSize);
    if (bytesRequested != kI2cFrameChunkSize) {
      while (Wire.available() > 0) {
        (void)Wire.read();
      }
      return {
          .status = I2cReadStatus::ShortRequest,
          .detail = static_cast<uint8_t>(bytesRequested),
      };
    }

    size_t bytesRead = 0;
    while (bytesRead < kI2cFrameChunkSize && Wire.available() > 0) {
      dst[offset + bytesRead] = static_cast<uint8_t>(Wire.read());
      ++bytesRead;
    }

    if (bytesRead != kI2cFrameChunkSize) {
      return {
          .status = I2cReadStatus::ShortRead,
          .detail = static_cast<uint8_t>(bytesRead),
      };
    }
  }

  if (frame.version != kI2cSampleFrameVersion) {
    return {
        .status = I2cReadStatus::BadVersion,
        .detail = frame.version,
    };
  }

  if (frame.sample_count == 0U || frame.sample_count > kI2cSamplesPerFrame) {
    return {
        .status = I2cReadStatus::BadSampleCount,
        .detail = frame.sample_count,
    };
  }

  return {
      .status = I2cReadStatus::Ok,
      .detail = 0U,
  };
}

}  // namespace

bool I2CSensorBegin(const void *ctx) {
  const I2CSensorContext *config = GetI2CContext(ctx);
  if (config == nullptr) {
    return false;
  }

  Wire.begin();
  if (config->clockHz != 0U) {
    Wire.setClock(config->clockHz);
  }
  return true;
}

bool I2CSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const I2CSensorContext *config = GetI2CContext(ctx);
  if (config == nullptr) {
    return false;
  }

  static uint8_t bootDiagnosticSamplesRemaining = kBootDiagnosticSampleCount;
  if (bootDiagnosticSamplesRemaining > 0U) {
    --bootDiagnosticSamplesRemaining;
    CopyDiagnosticFrame(I2cReadStatus::Boot, outFrame);
    return true;
  }

  I2cSampleFrame frame = {};
  const I2cReadResult result = ReadFrame(*config, frame);
  if (result.status != I2cReadStatus::Ok) {
    CopyDiagnosticFrame(result.status, outFrame, result.detail);
    return true;
  }

  const I2cSampleRecord &sample = frame.samples[frame.sample_count - 1U];
  const I2cCanSampleFrame canFrame = {
      .ch1_voltage = 0.0F,
      .raw_count = RawCountToUint24(sample.ch1_raw),
  };

  CopyCanFrame(canFrame, outFrame);
  return true;
}
