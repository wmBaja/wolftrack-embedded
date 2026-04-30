#include <Arduino.h>
#include <Wire.h>
#include <debug_print.h>
#include <mlx90614_sensor.h>

#include <string.h>

namespace {

constexpr uint8_t kMLX90614BeginAttempts = 5U;
constexpr uint16_t kMLX90614BeginRetryDelayMs = 500U;
constexpr uint8_t kMLX90614RegisterReadAttempts = 3U;
constexpr uint16_t kMLX90614RegisterRetryDelayMs = 3U;
constexpr uint16_t kMLX90614InterRegisterDelayMs = 2U;
constexpr uint32_t kMLX90614EmissivityRefreshIntervalMs = 5000U;

const MLX90614SensorContext *GetMLX90614Context(const void *ctx) {
  return static_cast<const MLX90614SensorContext *>(ctx);
}

MLX90614Driver *GetDriver(const MLX90614SensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return &config.runtime->driver;
}

TwoWire *GetWire(const MLX90614SensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return config.runtime->wire;
}

void ApplyClock(const MLX90614SensorContext &config) {
  TwoWire *wire = GetWire(config);
  if (wire != nullptr && config.clockHz != 0U) {
    wire->setClock(config.clockHz);
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

#if BAJACAN_ENABLE_DEBUG_PRINTS
void PrintHexByte(const uint8_t value) {
  if (value < 0x10U) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void PrintI2cAddress(const uint8_t address) {
  Serial.print("0x");
  PrintHexByte(address);
}

void PrintBeginAttempt(const MLX90614SensorContext &config, const uint8_t attempt,
                       const int beginResult) {
  PrintTimestampMs(millis());
  Serial.print("MLX90614 begin attempt ");
  Serial.print(attempt);
  Serial.print('/');
  Serial.print(kMLX90614BeginAttempts);
  Serial.print(" addr=");
  PrintI2cAddress(config.i2cAddress);
  Serial.print(" clock=");
  Serial.print(config.clockHz);
  Serial.print("Hz result=");
  Serial.println(beginResult);
}
#else
void PrintBeginAttempt(const MLX90614SensorContext &config,
                       const uint8_t attempt, const int beginResult) {
  (void)config;
  (void)attempt;
  (void)beginResult;
}
#endif

bool ReadRegister(MLX90614Driver &driver, const uint8_t reg, uint16_t &outValue) {
  uint8_t buf[2] = {};
  if (driver.ReadRegister(reg, buf) != sizeof(buf)) {
    return false;
  }

  outValue = static_cast<uint16_t>(buf[0]) |
             (static_cast<uint16_t>(buf[1]) << 8);
  return true;
}

bool ReadRegisterWithRetry(const MLX90614SensorContext &config,
                           MLX90614Driver &driver, const uint8_t reg,
                           uint16_t &outValue) {
  for (uint8_t attempt = 1U; attempt <= kMLX90614RegisterReadAttempts;
       ++attempt) {
    ApplyClock(config);
    if (ReadRegister(driver, reg, outValue)) {
      return true;
    }

    if (attempt < kMLX90614RegisterReadAttempts) {
      delay(kMLX90614RegisterRetryDelayMs);
    }
  }

  return false;
}

void DelayBetweenRegisterReads() {
  if (kMLX90614InterRegisterDelayMs > 0U) {
    delay(kMLX90614InterRegisterDelayMs);
  }
}

void PopulateTemperatures(const MLX90614SensorContext &config,
                          MLX90614Driver &driver,
                          MLX90614SampleFrame &sample) {
  uint16_t ambientRaw = 0U;
  if (!ReadRegisterWithRetry(config, driver, MLX90614_TA, ambientRaw)) {
    CaptureReadError(sample, kMLX90614SensorErrorAmbientReadFailed);
  } else {
    sample.ambientCentiDegrees = RawTemperatureToCentiDegrees(ambientRaw);
    sample.validMask |= kMLX90614SampleValidAmbientTemperature;
  }

  DelayBetweenRegisterReads();

  uint16_t objectRaw = 0U;
  if (!ReadRegisterWithRetry(config, driver, MLX90614_TOBJ1, objectRaw)) {
    CaptureReadError(sample, kMLX90614SensorErrorObjectReadFailed);
  } else {
    sample.objectCentiDegrees = RawTemperatureToCentiDegrees(objectRaw);
    sample.validMask |= kMLX90614SampleValidObjectTemperature;
  }
}

void UpdateCachedEmissivity(const MLX90614SensorContext &config,
                            MLX90614SensorRuntime &runtime,
                            MLX90614Driver &driver) {
  runtime.nextEmissivityRefreshAtMs =
      millis() + kMLX90614EmissivityRefreshIntervalMs;

  uint16_t emissivityReg = 0U;
  if (!ReadRegisterWithRetry(config, driver, MLX90614_EMISSIVITY,
                             emissivityReg)) {
    return;
  }

  runtime.cachedEmissivityReg = emissivityReg;
  runtime.emissivityCached = true;
}

void PopulateEmissivity(const MLX90614SensorContext &config,
                        MLX90614SensorRuntime &runtime,
                        MLX90614Driver &driver, MLX90614SampleFrame &sample) {
  const uint32_t nowMs = millis();
  if (!runtime.emissivityCached || nowMs >= runtime.nextEmissivityRefreshAtMs) {
    UpdateCachedEmissivity(config, runtime, driver);
  }

  if (!runtime.emissivityCached) {
    CaptureReadError(sample, kMLX90614SensorErrorEmissivityReadFailed);
    return;
  }

  sample.emissivityTenThousandths =
      EmissivityRegToTenThousandths(runtime.cachedEmissivityReg);
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
  config->runtime->emissivityCached = false;
  config->runtime->cachedEmissivityReg = 0U;
  config->runtime->nextEmissivityRefreshAtMs = 0U;

  for (uint8_t attempt = 1U; attempt <= kMLX90614BeginAttempts; ++attempt) {
    TwoWire *wire = GetWire(*config);
    if (wire == nullptr) {
      config->runtime->lastError = kMLX90614SensorErrorBeginDataBus;
      return false;
    }

    config->runtime->driver =
        MLX90614Driver(config->i2cAddress, wire, config->clockHz);

    MLX90614Driver *driver = GetDriver(*config);
    if (driver == nullptr) {
      config->runtime->lastError = kMLX90614SensorErrorBeginFailed;
      return false;
    }

    ApplyClock(*config);
    const int beginResult = driver->begin();
    if (beginResult == 0) {
      ApplyClock(*config);
      UpdateCachedEmissivity(*config, *config->runtime, *driver);
      config->runtime->initialized = true;
      config->runtime->lastError = kMLX90614SensorErrorNone;
      return true;
    }

    config->runtime->lastError = static_cast<int16_t>(beginResult);
    PrintBeginAttempt(*config, attempt, beginResult);
    if (attempt < kMLX90614BeginAttempts) {
      delay(kMLX90614BeginRetryDelayMs);
    }
  }

  return false;
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
  PopulateTemperatures(*config, *driver, sample);
  DelayBetweenRegisterReads();
  PopulateEmissivity(*config, *config->runtime, *driver, sample);
  CopySampleToFrame(sample, outFrame);
  return true;
}
