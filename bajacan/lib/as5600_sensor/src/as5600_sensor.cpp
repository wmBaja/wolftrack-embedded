#include <Arduino.h>
#include <Wire.h>
#include <as5600_sensor.h>

#include <string.h>

namespace {

struct AS5600Capture {
  uint16_t rawAngle = 0U;
  int16_t angleCentiDegrees = 0;
  uint16_t magnitude = 0U;
  uint8_t agc = 0U;
  uint8_t status = 0U;
  int16_t error = 0;
  uint8_t validMask = 0U;
};

const AS5600SubSensorContext *GetAS5600Context(const void *ctx) {
  return static_cast<const AS5600SubSensorContext *>(ctx);
}

AS5600 *GetDriver(const AS5600SubSensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return &config.runtime->driver;
}

TwoWire *GetWire(const AS5600SubSensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return config.runtime->wire;
}

void CopyDataFrameToCan(const AS5600DataSampleFrame &sample,
                        CANFDMessage &outFrame) {
  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
}

void CopyStatsFrameToCan(const AS5600StatsSampleFrame &sample,
                         CANFDMessage &outFrame) {
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

bool CaptureAs5600Sample(const AS5600SubSensorContext &config,
                         AS5600Capture &sample) {
  AS5600 *driver = GetDriver(config);
  if (driver == nullptr) {
    return false;
  }

  if (!config.runtime->initialized) {
    sample.error = kAS5600SensorErrorNotInitialized;
    return true;
  }

  sample.rawAngle = driver->rawAngle();
  if (CaptureLastError(*driver, sample.error)) {
    return true;
  }

  uint16_t angleForMapping = sample.rawAngle;
  if (config.angleMapping == AS5600AngleMapping::CenteredWindow) {
    angleForMapping = driver->readAngle();
    if (CaptureLastError(*driver, sample.error)) {
      return true;
    }
  }

  if (config.angleMapping == AS5600AngleMapping::CenteredWindow) {
    sample.angleCentiDegrees = MapAngleToCenteredCentiDegrees(
        angleForMapping, config.maxMappedAngleCentiDegrees);
  } else {
    sample.angleCentiDegrees = static_cast<int16_t>(
        AS5600RawAngleToCentiDegrees(sample.rawAngle));
  }

  sample.status = driver->readStatus();
  if (CaptureLastError(*driver, sample.error)) {
    return true;
  }

  sample.agc = driver->readAGC();
  if (CaptureLastError(*driver, sample.error)) {
    return true;
  }

  sample.magnitude = driver->readMagnitude();
  if (CaptureLastError(*driver, sample.error)) {
    return true;
  }

  sample.validMask |= kAS5600SampleValidMagnet;
  return true;
}

}  // namespace

bool AS5600SensorBegin(const void *ctx) {
  const AS5600SubSensorContext *config = GetAS5600Context(ctx);
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

bool AS5600DataSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const AS5600SubSensorContext *config = GetAS5600Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  AS5600Capture capture = {};
  if (!CaptureAs5600Sample(*config, capture)) {
    return false;
  }

  AS5600DataSampleFrame sample = {
      .angleCentiDegrees = capture.angleCentiDegrees,
  };
  CopyDataFrameToCan(sample, outFrame);
  return true;
}

bool AS5600StatsSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const AS5600SubSensorContext *config = GetAS5600Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  AS5600Capture capture = {};
  if (!CaptureAs5600Sample(*config, capture)) {
    return false;
  }

  AS5600StatsSampleFrame sample = {
      .version = kAS5600SampleFrameVersion,
      .validMask = capture.validMask,
      .rawAngle = capture.rawAngle,
      .magnitude = capture.magnitude,
      .agc = capture.agc,
      .status = capture.status,
      .error = capture.error,
  };
  CopyStatsFrameToCan(sample, outFrame);
  return true;
}
