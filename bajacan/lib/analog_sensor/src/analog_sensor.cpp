#include <Arduino.h>
#include <analog_sensor.h>

namespace {
constexpr uint8_t kAnalogSensorPin = A0;
}

bool AnalogSensorBegin(void *) {
  pinMode(kAnalogSensorPin, INPUT);
  return true;
}

bool AnalogSensorSample(void *, CANFDMessage &outFrame) {
  const uint16_t reading = analogRead(kAnalogSensorPin);
  outFrame.len = 2;
  outFrame.data[0] = reading >> 8;
  outFrame.data[1] = reading & 0xFF;
  return true;
}

const SensorDescriptor kAnalogSensor{
    .name = "AnalogRaw",
    .canId = 0x300,
    .pollIntervalMs = 20,
    .context = nullptr,
    .begin = AnalogSensorBegin,
    .sample = AnalogSensorSample,
    .suspend = nullptr,
    .resume = nullptr,
};
