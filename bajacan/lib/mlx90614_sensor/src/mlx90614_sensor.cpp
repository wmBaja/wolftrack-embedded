#include <Arduino.h>
#include <Wire.h>
#include <mlx90614_sensor.h>

#include <string.h>

namespace {

const MLX90614SensorContext *GetMLX90614Context(const void *ctx) {
  return static_cast<const MLX90614SensorContext *>(ctx);
}

MLX90614Driver *GetDriver(const MLX90614SensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return &config.runtime->driver;
}

void ApplyClock(const MLX90614SensorContext &config) {
  if (config.clockHz != 0U) {
    Wire.setClock(config.clockHz);
  }
}

int32_t RawTemperatureToCentiDegrees(const uint16_t rawTemperature) {
  return static_cast<int32_t>(rawTemperature) * 2 - 27315;
}

uint16_t EmissivityRegToTenThousandths(const uint16_t emissivityReg) {
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(emissivityReg) * kMLX90614EmissivityScale +
       32767U) /
      65535U);
}

void CopySampleToFrame(const MLX90614SampleFrame &sample,
                       CANFDMessage &outFrame) {
  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
}

void CaptureReadError(MLX90614SampleFrame &sample, const int16_t error) {
  if (sample.error == kMLX90614SensorErrorNone) {
    sample.error = error;
  }
}

bool ReadRegister(MLX90614Driver &driver, const uint8_t reg, uint16_t &outValue) {
  uint8_t buf[2] = {};
  if (driver.ReadRegister(reg, buf) != sizeof(buf)) {
    return false;
  }

  outValue = static_cast<uint16_t>(buf[0]) |
             (static_cast<uint16_t>(buf[1]) << 8);
  return true;
}

void PopulateTemperatures(MLX90614Driver &driver, MLX90614SampleFrame &sample) {
  uint16_t ambientRaw = 0U;
  if (!ReadRegister(driver, MLX90614_TA, ambientRaw)) {
    CaptureReadError(sample, kMLX90614SensorErrorAmbientReadFailed);
  } else {
    sample.ambientCentiDegrees = RawTemperatureToCentiDegrees(ambientRaw);
    sample.validMask |= kMLX90614SampleValidAmbientTemperature;
  }

  uint16_t objectRaw = 0U;
  if (!ReadRegister(driver, MLX90614_TOBJ1, objectRaw)) {
    CaptureReadError(sample, kMLX90614SensorErrorObjectReadFailed);
  } else {
    sample.objectCentiDegrees = RawTemperatureToCentiDegrees(objectRaw);
    sample.validMask |= kMLX90614SampleValidObjectTemperature;
  }
}

void PopulateEmissivity(MLX90614Driver &driver, MLX90614SampleFrame &sample) {
  uint16_t emissivityReg = 0U;
  if (!ReadRegister(driver, MLX90614_EMISSIVITY, emissivityReg)) {
    CaptureReadError(sample, kMLX90614SensorErrorEmissivityReadFailed);
    return;
  }

  sample.emissivityTenThousandths =
      EmissivityRegToTenThousandths(emissivityReg);
  sample.validMask |= kMLX90614SampleValidEmissivity;
}

}  // namespace

bool MLX90614SensorBegin(const void *ctx) {
  const MLX90614SensorContext *config = GetMLX90614Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  config->runtime->initialized = false;
  config->runtime->lastError = kMLX90614SensorErrorNone;
  config->runtime->driver =
      MLX90614Driver(config->i2cAddress, &Wire, config->clockHz);

  MLX90614Driver *driver = GetDriver(*config);
  if (driver == nullptr) {
    config->runtime->lastError = kMLX90614SensorErrorBeginFailed;
    return false;
  }

  ApplyClock(*config);
  const int beginResult = driver->begin();
  if (beginResult != 0) {
    config->runtime->lastError = static_cast<int16_t>(beginResult);
    return false;
  }

  ApplyClock(*config);
  config->runtime->initialized = true;
  return true;
}

bool MLX90614SensorSample(const void *ctx, CANFDMessage &outFrame) {
  const MLX90614SensorContext *config = GetMLX90614Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  MLX90614SampleFrame sample = {};
  sample.version = kMLX90614SampleFrameVersion;

  if (!config->runtime->initialized) {
    sample.error = config->runtime->lastError;
    if (sample.error == kMLX90614SensorErrorNone) {
      sample.error = kMLX90614SensorErrorNotInitialized;
    }
    CopySampleToFrame(sample, outFrame);
    return true;
  }

  MLX90614Driver *driver = GetDriver(*config);
  if (driver == nullptr) {
    return false;
  }

  ApplyClock(*config);
  sample.error = kMLX90614SensorErrorNone;
  PopulateTemperatures(*driver, sample);
  PopulateEmissivity(*driver, sample);
  CopySampleToFrame(sample, outFrame);
  return true;
}
