#pragma once

#include <config.h>

struct AnalogSensorContext {
  SensorContext base;
  uint8_t pin;
};

bool AnalogSensorBegin(const void *ctx);
bool AnalogSensorSample(const void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor MakeAnalogSensor(const AnalogSensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = AnalogSensorBegin,
      .sample = AnalogSensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}
