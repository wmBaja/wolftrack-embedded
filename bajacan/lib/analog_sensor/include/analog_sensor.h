#pragma once

#include <config.h>

bool AnalogSensorBegin(void *ctx);
bool AnalogSensorSample(void *ctx, CANFDMessage &outFrame);

extern const SensorDescriptor kAnalogSensor;
