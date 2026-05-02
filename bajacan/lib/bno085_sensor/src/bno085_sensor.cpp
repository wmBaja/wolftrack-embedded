#include <Arduino.h>
#include <Wire.h>
#include <bno085_sensor.h>

#include <string.h>

namespace {

const BNO085SensorContext *GetBNO085Context(const void *ctx) {
  return static_cast<const BNO085SensorContext *>(ctx);
}

BNO08x *GetDriver(const BNO085SensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return &config.runtime->driver;
}

uint16_t GetReportIntervalMs(const BNO085SensorContext &config) {
  if (config.reportIntervalMs == 0U) {
    return kBNO085DefaultReportIntervalMs;
  }
  return config.reportIntervalMs;
}

void ResetCachedSample(BNO085SensorRuntime &runtime) {
  runtime.validMask = 0U;
  runtime.accelerometerAccuracy = 0U;
  runtime.gyroAccuracy = 0U;
  runtime.linearAccelerationAccuracy = 0U;
  runtime.rotationVectorAccuracy = 0U;
  runtime.lastError = kBNO085SensorErrorNone;
  runtime.accelerometer = {};
  runtime.angularVelocity = {};
  runtime.linearAcceleration = {};
  runtime.rotationVector = {};
  runtime.rotationVectorAccuracyRad = 0.0f;
}

template <typename Frame>
void FillCommonFields(const BNO085SensorRuntime &runtime, Frame &frame) {
  frame.version = kBNO085SampleFrameVersion;
  frame.validMask = runtime.validMask;
  frame.accelerometerAccuracy = runtime.accelerometerAccuracy;
  frame.gyroAccuracy = runtime.gyroAccuracy;
  frame.linearAccelerationAccuracy = runtime.linearAccelerationAccuracy;
  frame.rotationVectorAccuracy = runtime.rotationVectorAccuracy;
  frame.error = runtime.lastError;
  frame.accelerometer = runtime.accelerometer;
  frame.angularVelocity = runtime.angularVelocity;
  frame.linearAcceleration = runtime.linearAcceleration;
}

void CopyRuntimeToFrame(const BNO085SensorRuntime &runtime,
                        CANFDMessage &outFrame) {
  BNO085SampleFrame frame = {};
  FillCommonFields(runtime, frame);

  if constexpr (kBNO085SensorHasRotationVector) {
    frame.rotationVector = runtime.rotationVector;
    frame.rotationVectorAccuracyRad = runtime.rotationVectorAccuracyRad;
  }

  outFrame.len = sizeof(frame);
  memcpy(outFrame.data, &frame, sizeof(frame));
}

bool ConfigureReports(const BNO085SensorContext &config) {
  if (config.runtime == nullptr || !config.runtime->initialized) {
    return false;
  }

  BNO08x *driver = GetDriver(config);
  if (driver == nullptr) {
    return false;
  }

  if (config.runtime->reportsConfigured) {
    return true;
  }

  const uint16_t intervalMs = GetReportIntervalMs(config);
  bool ok = driver->enableAccelerometer(intervalMs);
  ok = ok && driver->enableGyro(intervalMs);
  ok = ok && driver->enableLinearAccelerometer(intervalMs);

  if constexpr (kBNO085SensorHasRotationVector) {
    ok = ok && driver->enableRotationVector(intervalMs);
  }

  config.runtime->reportsConfigured = ok;
  config.runtime->lastError =
      ok ? kBNO085SensorErrorNone : kBNO085SensorErrorConfigureReportsFailed;
  return ok;
}

void CaptureSensorEvent(BNO085SensorRuntime &runtime, BNO08x &driver) {
  switch (driver.getSensorEventID()) {
    case SENSOR_REPORTID_ACCELEROMETER:
      runtime.accelerometer = {
          driver.getAccelX(),
          driver.getAccelY(),
          driver.getAccelZ(),
      };
      runtime.accelerometerAccuracy = driver.getAccelAccuracy();
      runtime.validMask |= kBNO085ReportMaskAccelerometer;
      break;

    case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
      runtime.angularVelocity = {
          driver.getGyroX(),
          driver.getGyroY(),
          driver.getGyroZ(),
      };
      runtime.gyroAccuracy = driver.getGyroAccuracy();
      runtime.validMask |= kBNO085ReportMaskGyro;
      break;

    case SENSOR_REPORTID_LINEAR_ACCELERATION:
      runtime.linearAcceleration = {
          driver.getLinAccelX(),
          driver.getLinAccelY(),
          driver.getLinAccelZ(),
      };
      runtime.linearAccelerationAccuracy = driver.getLinAccelAccuracy();
      runtime.validMask |= kBNO085ReportMaskLinearAcceleration;
      break;

    case SENSOR_REPORTID_ROTATION_VECTOR:
      if constexpr (kBNO085SensorHasRotationVector) {
        runtime.rotationVector = {
            driver.getQuatI(),
            driver.getQuatJ(),
            driver.getQuatK(),
            driver.getQuatReal(),
        };
        runtime.rotationVectorAccuracy = driver.getQuatAccuracy();
        runtime.rotationVectorAccuracyRad = driver.getQuatRadianAccuracy();
        runtime.validMask |= kBNO085ReportMaskRotationVector;
      }
      break;

    default:
      break;
  }
}

void RefreshCachedSample(const BNO085SensorContext &config) {
  if (config.runtime == nullptr || !config.runtime->initialized) {
    return;
  }

  BNO08x *driver = GetDriver(config);
  if (driver == nullptr) {
    return;
  }

  if (driver->wasReset()) {
    config.runtime->reportsConfigured = false;
    ResetCachedSample(*config.runtime);
  }

  (void)ConfigureReports(config);

  while (driver->getSensorEvent()) {
    CaptureSensorEvent(*config.runtime, *driver);
  }
}

}  // namespace

bool BNO085SensorBegin(const void *ctx) {
  const BNO085SensorContext *config = GetBNO085Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  config->runtime->initialized = false;
  config->runtime->reportsConfigured = false;
  ResetCachedSample(*config->runtime);

  Wire.begin();
  if (config->clockHz != 0U) {
    Wire.setClock(config->clockHz);
  }

  for (uint8_t attempt = 1U; attempt <= 5U; ++attempt) {
    BNO08x *driver = GetDriver(*config);
    if (driver == nullptr) {
      config->runtime->lastError = kBNO085SensorErrorBeginFailed;
      return false;
    }

    if (driver->begin(config->i2cAddress, Wire, config->interruptPin,
                      config->resetPin)) {
      config->runtime->initialized = true;
      return ConfigureReports(*config);
    }

    // Delay and retry to allow Adafruit boards time to boot
    if (attempt < 5U) {
      delay(100);
    }
  }

  config->runtime->lastError = kBNO085SensorErrorBeginFailed;
  return false;
}

bool BNO085SensorSample(const void *ctx, CANFDMessage &outFrame) {
  const BNO085SensorContext *config = GetBNO085Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  if (!config->runtime->initialized) {
    if (config->runtime->lastError == kBNO085SensorErrorNone) {
      config->runtime->lastError = kBNO085SensorErrorNotInitialized;
    }
    CopyRuntimeToFrame(*config->runtime, outFrame);
    return true;
  }

  RefreshCachedSample(*config);
  CopyRuntimeToFrame(*config->runtime, outFrame);
  return true;
}

void BNO085SensorSuspend(const void *ctx) {
  const BNO085SensorContext *config = GetBNO085Context(ctx);
  if (config == nullptr || config->runtime == nullptr ||
      !config->runtime->initialized) {
    return;
  }

  BNO08x *driver = GetDriver(*config);
  if (driver == nullptr) {
    return;
  }

  if (!driver->modeSleep()) {
    config->runtime->lastError = kBNO085SensorErrorSleepFailed;
    return;
  }

  config->runtime->reportsConfigured = false;
  ResetCachedSample(*config->runtime);
}

void BNO085SensorResume(const void *ctx) {
  const BNO085SensorContext *config = GetBNO085Context(ctx);
  if (config == nullptr || config->runtime == nullptr ||
      !config->runtime->initialized) {
    return;
  }

  BNO08x *driver = GetDriver(*config);
  if (driver == nullptr) {
    return;
  }

  if (!driver->modeOn()) {
    config->runtime->lastError = kBNO085SensorErrorWakeFailed;
    return;
  }

  if (config->clockHz != 0U) {
    Wire.setClock(config->clockHz);
  }

  config->runtime->reportsConfigured = false;
  ResetCachedSample(*config->runtime);
  (void)ConfigureReports(*config);
}
