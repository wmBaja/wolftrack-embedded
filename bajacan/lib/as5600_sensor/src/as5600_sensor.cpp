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

TwoWire *GetWire(const AS5600SensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return config.runtime->wire;
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

int16_t MapAngleToCenteredCentiDegrees(const uint16_t angle,
                                       const int16_t maxCentiDegrees) {
  const uint32_t numerator =
      (static_cast<uint32_t>(angle & 0x0FFFU) *
       static_cast<uint32_t>(maxCentiDegrees) * 2U) +
      (kAS5600MaxAngleCount / 2U);
  const int32_t centered =
      static_cast<int32_t>(numerator / kAS5600MaxAngleCount) -
      static_cast<int32_t>(maxCentiDegrees);
  return static_cast<int16_t>(centered);
}

}  // namespace

bool AS5600SensorBegin(const void *ctx) {
  const AS5600SensorContext *config = GetAS5600Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  config->runtime->initialized = false;

  TwoWire *wire = GetWire(*config);
  if (wire == nullptr) {
    return false;
  }

  config->runtime->driver = AS5600(wire);

  wire->begin();
  if (config->clockHz != 0U) {
    wire->setClock(config->clockHz);
  }

  AS5600 *driver = GetDriver(*config);
  if (driver == nullptr) {
    return false;
  }

  for (uint8_t attempt = 1U; attempt <= 5U; ++attempt) {
    if (driver->begin(config->directionPin)) {
      break;
    }

    if (attempt == 5U) {
      return false;
    }

    delay(100);
  }

  driver->setDirection(config->direction);
  if (!driver->setOffset(static_cast<float>(config->offsetCentiDegrees) /
                         100.0f)) {
    return false;
  }
  if (config->initializePositionWindow) {
    if (!driver->setZPosition(config->zPosition)) {
      return false;
    }
    if (!driver->setMPosition(config->mPosition)) {
      return false;
    }
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
    uint16_t angleForMapping = sample.rawAngle;
    if (config->angleMapping == AS5600AngleMapping::CenteredWindow) {
      angleForMapping = driver->readAngle();
      if (!CaptureLastError(*driver, sample.error)) {
        sample.angleCentiDegrees = MapAngleToCenteredCentiDegrees(
            angleForMapping, config->maxMappedAngleCentiDegrees);
      }
    } else {
      sample.angleCentiDegrees = static_cast<int16_t>(
          AS5600RawAngleToCentiDegrees(sample.rawAngle));
    }

    if (sample.error == AS5600_OK) {
      sample.status = driver->readStatus();
      CaptureLastError(*driver, sample.error);
    }
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
