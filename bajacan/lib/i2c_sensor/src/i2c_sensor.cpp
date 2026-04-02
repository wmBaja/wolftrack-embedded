#include <Arduino.h>
#include <Wire.h>
#include <i2c_sensor.h>

#include <string.h>

namespace {

const I2CSensorContext *GetI2CContext(const void *ctx) {
  return static_cast<const I2CSensorContext *>(ctx);
}

bool ReadFrame(const I2CSensorContext &config, I2cSampleFrame &frame) {
  const uint8_t expectedBytes = static_cast<uint8_t>(sizeof(frame));
  const size_t bytesRequested = Wire.requestFrom(config.addr, expectedBytes);
  if (bytesRequested != expectedBytes) {
    while (Wire.available() > 0) {
      (void)Wire.read();
    }
    return false;
  }

  uint8_t *dst = reinterpret_cast<uint8_t *>(&frame);
  size_t bytesRead = 0;
  while (bytesRead < expectedBytes && Wire.available() > 0) {
    dst[bytesRead++] = static_cast<uint8_t>(Wire.read());
  }


  if (frame.version != kI2cSampleFrameVersion) {
    return false;
  }

  if (frame.sample_count > kI2cSamplesPerFrame) {
    return false;
  }

  return true;
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

  I2cSampleFrame frame = {};
  if (!ReadFrame(*config, frame)) {
    return false;
  }

  outFrame.len = sizeof(frame);
  memcpy(outFrame.data, &frame, sizeof(frame));
  return true;
}
