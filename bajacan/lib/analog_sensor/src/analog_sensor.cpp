#include <Arduino.h>
#include <analog_sensor.h>

namespace {
const AnalogSensorContext *GetAnalogContext(const void *ctx) {
  return static_cast<const AnalogSensorContext *>(ctx);
}
}

bool AnalogSensorBegin(const void *ctx) {
  const AnalogSensorContext *config = GetAnalogContext(ctx);
  if (config == nullptr) {
    return false;
  }
  pinMode(config->pin, INPUT);
  return true;
}

bool AnalogSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const AnalogSensorContext *config = GetAnalogContext(ctx);
  if (config == nullptr) {
    return false;
  }
  const uint16_t reading = analogRead(config->pin);
  outFrame.id = config->can_id;outFrame.len = sizeof(reading);
  memcpy(outFrame.data, &reading, sizeof(reading));
  return true;
}
