#include <Arduino.h>
#include <Wire.h>
#include <as5600_sensor.h>

#include <string.h>

namespace {

const AS5600SensorContext *GetAS5600Context(const void *ctx) {
  return static_cast<const AS5600SensorContext *>(ctx);
}

AS5600 *GetDriver(const AS5600SensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return &config.runtime->driver;
}

void CopySampleToFrame(const AS5600SampleFrame &sample, CANFDMessage &outFrame) {
  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
}

bool CaptureLastError(AS5600 &driver, int16_t &outError) {
  const int error = driver.lastError();
  if (error == AS5600_OK) {
    return false;
  }
  outError = static_cast<int16_t>(error);
  return true;
}

}  // namespace

bool AS5600SensorBegin(const void *ctx) {
  const AS5600SensorContext *config = GetAS5600Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  config->runtime->initialized = false;

  Wire.begin();
  if (config->clockHz != 0U) {
    Wire.setClock(config->clockHz);
  }

  AS5600 *driver = GetDriver(*config);
  if (driver == nullptr) {
    return false;
  }
  if (!driver->begin(config->directionPin)) {
    return false;
  }

  driver->setDirection(config->direction);
  if (!driver->setOffset(static_cast<float>(config->offsetCentiDegrees) /
                         100.0f)) {
    return false;
  }

  config->runtime->initialized = true;
  return true;
}

bool AS5600SensorSample(const void *ctx, CANFDMessage &outFrame) {
  const AS5600SensorContext *config = GetAS5600Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  AS5600SampleFrame sample = {};
  AS5600 *driver = GetDriver(*config);
  if (driver == nullptr) {
    return false;
  }

  if (!config->runtime->initialized) {
    sample.error = kAS5600SensorErrorNotInitialized;
    CopySampleToFrame(sample, outFrame);
    return true;
  }

  sample.rawAngle = driver->rawAngle();
  if (!CaptureLastError(*driver, sample.error)) {
    sample.angleCentiDegrees = AS5600RawAngleToCentiDegrees(sample.rawAngle);

    sample.status = driver->readStatus();
    CaptureLastError(*driver, sample.error);
  }

  if (sample.error == AS5600_OK) {
    sample.agc = driver->readAGC();
    CaptureLastError(*driver, sample.error);
  }

  if (sample.error == AS5600_OK) {
    sample.magnitude = driver->readMagnitude();
    CaptureLastError(*driver, sample.error);
  }

  CopySampleToFrame(sample, outFrame);
  return true;
}
