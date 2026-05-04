#include <Arduino.h>
#include <Wire.h>
#include <bno085_sensor.h>

#include <string.h>

namespace {

const BNO085SubSensorContext *GetBNO085Context(const void *ctx) {
  return static_cast<const BNO085SubSensorContext *>(ctx);
}

BNO08x *GetDriver(const BNO085SubSensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return &config.runtime->driver;
}

uint16_t GetReportIntervalMs(const BNO085SubSensorContext &config) {
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

void CopyDataRuntimeToFrame(const BNO085SensorRuntime &runtime, CANFDMessage &outFrame) {
  BNO085DataSampleFrame frame = {};
  frame.version = kBNO085SampleFrameVersion;
  frame.validMask = runtime.validMask;
  frame.accelerometer = runtime.accelerometer;
  frame.angularVelocity = runtime.angularVelocity;
  frame.linearAcceleration = runtime.linearAcceleration;

  if constexpr (kBNO085DataSensorHasRotationVector) {
    frame.rotationVector = runtime.rotationVector;
  }

  outFrame.len = sizeof(frame);
  memcpy(outFrame.data, &frame, sizeof(frame));
}

void CopyStatsRuntimeToFrame(const BNO085SensorRuntime &runtime, CANFDMessage &outFrame) {
  BNO085StatsSampleFrame frame = {};
  frame.version = kBNO085SampleFrameVersion;
  frame.validMask = runtime.validMask;
  frame.error = runtime.lastError;
  frame.accelerometerAccuracy = runtime.accelerometerAccuracy;
  frame.gyroAccuracy = runtime.gyroAccuracy;
  frame.linearAccelerationAccuracy = runtime.linearAccelerationAccuracy;
  frame.rotationVectorAccuracy = runtime.rotationVectorAccuracy;
  frame.rotationVectorAccuracyRad = runtime.rotationVectorAccuracyRad;

  outFrame.len = sizeof(frame);
  memcpy(outFrame.data, &frame, sizeof(frame));
}

bool ConfigureReports(const BNO085SubSensorContext &config) {
  if (config.runtime == nullptr || !config.runtime->initialized) {
    return false;
  }

  BNO08x *driver = GetDriver(config);
  if (driver == nullptr) {
    return false;
  }

  // Use a single flag since we need all reports for both data and stats
  if (config.runtime->dataReportsConfigured) {
    return true;
  }

  const uint16_t intervalMs = GetReportIntervalMs(config);
  bool ok = driver->enableAccelerometer(intervalMs);
  ok = ok && driver->enableGyro(intervalMs);
  ok = ok && driver->enableLinearAccelerometer(intervalMs);

  if constexpr (kBNO085DataSensorHasRotationVector) {
    ok = ok && driver->enableRotationVector(intervalMs);
  }

  config.runtime->dataReportsConfigured = ok;
  config.runtime->statsReportsConfigured = ok;
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
      runtime.rotationVector = {
          driver.getQuatI(),
          driver.getQuatJ(),
          driver.getQuatK(),
          driver.getQuatReal(),
      };
      runtime.rotationVectorAccuracy = driver.getQuatAccuracy();
      runtime.rotationVectorAccuracyRad = driver.getQuatRadianAccuracy();
      runtime.validMask |= kBNO085ReportMaskRotationVector;
      break;

    default:
      break;
  }
}

void RefreshCachedSample(const BNO085SubSensorContext &config) {
  if (config.runtime == nullptr || !config.runtime->initialized) {
    return;
  }

  BNO08x *driver = GetDriver(config);
  if (driver == nullptr) {
    return;
  }

  if (driver->wasReset()) {
    config.runtime->dataReportsConfigured = false;
    config.runtime->statsReportsConfigured = false;
    ResetCachedSample(*config.runtime);
  }

  (void)ConfigureReports(config);

  while (driver->getSensorEvent()) {
    CaptureSensorEvent(*config.runtime, *driver);
  }
}

}  // namespace

bool BNO085SensorBegin(const void *ctx) {
  const BNO085SubSensorContext *config = GetBNO085Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  // If already initialized by the other sub-sensor, just return success
  if (config->runtime->initialized) {
    return true;
  }

  config->runtime->initialized = false;
  config->runtime->dataReportsConfigured = false;
  config->runtime->statsReportsConfigured = false;
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

    if (attempt < 5U) {
      delay(100);
    }
  }

  config->runtime->lastError = kBNO085SensorErrorBeginFailed;
  return false;
}

bool BNO085DataSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const BNO085SubSensorContext *config = GetBNO085Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  if (!config->runtime->initialized) {
    if (config->runtime->lastError == kBNO085SensorErrorNone) {
      config->runtime->lastError = kBNO085SensorErrorNotInitialized;
    }
    CopyDataRuntimeToFrame(*config->runtime, outFrame);
    return true;
  }

  RefreshCachedSample(*config);
  CopyDataRuntimeToFrame(*config->runtime, outFrame);
  return true;
}

bool BNO085StatsSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const BNO085SubSensorContext *config = GetBNO085Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  if (!config->runtime->initialized) {
    if (config->runtime->lastError == kBNO085SensorErrorNone) {
      config->runtime->lastError = kBNO085SensorErrorNotInitialized;
    }
    CopyStatsRuntimeToFrame(*config->runtime, outFrame);
    return true;
  }

  RefreshCachedSample(*config);
  CopyStatsRuntimeToFrame(*config->runtime, outFrame);
  return true;
}

void BNO085SensorSuspend(const void *ctx) {
  const BNO085SubSensorContext *config = GetBNO085Context(ctx);
  if (config == nullptr || config->runtime == nullptr ||
      !config->runtime->initialized) {
    return;
  }

  // Only suspend once, even if called for both Data and Stats
  if (!config->runtime->dataReportsConfigured && !config->runtime->statsReportsConfigured) {
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

  config->runtime->dataReportsConfigured = false;
  config->runtime->statsReportsConfigured = false;
  ResetCachedSample(*config->runtime);
}

void BNO085SensorResume(const void *ctx) {
  const BNO085SubSensorContext *config = GetBNO085Context(ctx);
  if (config == nullptr || config->runtime == nullptr ||
      !config->runtime->initialized) {
    return;
  }

  // Only resume once
  if (config->runtime->dataReportsConfigured) {
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

  config->runtime->dataReportsConfigured = false;
  config->runtime->statsReportsConfigured = false;
  ResetCachedSample(*config->runtime);
  (void)ConfigureReports(*config);
}
