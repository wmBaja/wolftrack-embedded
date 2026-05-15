#include <Arduino.h>
#include <Wire.h>
#include <debug_print.h>
#include <mlx90614_sensor.h>

#include <string.h>

namespace {

constexpr uint8_t kMLX90614BeginAttempts = 5U;
constexpr uint16_t kMLX90614BeginRetryDelayMs = 100U;
constexpr uint8_t kMLX90614RegisterReadAttempts = 3U;
constexpr uint16_t kMLX90614RegisterRetryDelayMs = 3U;
constexpr uint16_t kMLX90614InterRegisterDelayMs = 2U;
constexpr uint32_t kMLX90614EmissivityRefreshIntervalMs = 5000U;
constexpr uint8_t kMLX90614RequestedSmbusLevel = 0U;
constexpr uint8_t kMLX90614RequestedLongSetup = 1U;
constexpr uint8_t kMLX90614RequestedSdaHold = 3U;

struct MLX90614ValidationResult {
  double ambientC = NAN;
  double objectC = NAN;
  double emissivity = NAN;
  bool lowLevelReadAttempted = false;
  bool lowLevelReadSucceeded = false;
  uint8_t lowLevelWriteStatus = 0xFFU;
  uint8_t lowLevelBytesRead = 0U;
  uint8_t lowLevelData[3] = {};
};

struct MLX90614Capture {
  uint8_t validMask = 0U;
  int32_t ambientCentiDegrees = 0;
  int32_t objectCentiDegrees = 0;
  uint16_t emissivityTenThousandths = 0U;
  int16_t error = kMLX90614SensorErrorNone;
};

const MLX90614SubSensorContext *GetMLX90614Context(const void *ctx) {
  return static_cast<const MLX90614SubSensorContext *>(ctx);
}

MLX90614Driver *GetDriver(const MLX90614SubSensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return &config.runtime->driver;
}

TwoWire *GetWire(const MLX90614SubSensorContext &config) {
  if (config.runtime == nullptr) {
    return nullptr;
  }
  return config.runtime->wire;
}

void ApplyClock(const MLX90614SubSensorContext &config) {
  TwoWire *wire = GetWire(config);
  if (wire != nullptr && config.clockHz != 0U) {
    wire->setClock(config.clockHz);
  }
}

void ApplyPins(const MLX90614SubSensorContext &config) {
#if defined(ARDUINO_ARCH_MEGAAVR)
  TwoWire *wire = GetWire(config);
  if (wire == nullptr) {
    return;
  }

  if (config.runtime->sdaPin >= 0 && config.runtime->sclPin >= 0) {
    (void)wire->pins(static_cast<uint8_t>(config.runtime->sdaPin),
                     static_cast<uint8_t>(config.runtime->sclPin));
  }
#else
  (void)config;
#endif
}

void ApplySpecialConfig(const MLX90614SubSensorContext &config) {
#if defined(ARDUINO_ARCH_MEGAAVR)
  TwoWire *wire = GetWire(config);
  if (wire != nullptr) {
    (void)wire->specialConfig(kMLX90614RequestedSmbusLevel,
                              kMLX90614RequestedLongSetup,
                              kMLX90614RequestedSdaHold);
  }
#else
  (void)config;
#endif
}

void ApplyInternalPullups(const MLX90614SubSensorContext &config) {
#if defined(ARDUINO_ARCH_MEGAAVR)
  TwoWire *wire = GetWire(config);
  if (wire != nullptr) {
    wire->usePullups();
  }
#else
  (void)config;
#endif
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

void CopyDataFrameToCan(const MLX90614DataSampleFrame &sample,
                        CANFDMessage &outFrame) {
  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
}

void CopyStatsFrameToCan(const MLX90614StatsSampleFrame &sample,
                         CANFDMessage &outFrame) {
  outFrame.len = sizeof(sample);
  memcpy(outFrame.data, &sample, sizeof(sample));
}

void CaptureReadError(MLX90614Capture &sample, const int16_t error) {
  if (sample.error == kMLX90614SensorErrorNone) {
    sample.error = error;
  }
}

bool ReadRegister(MLX90614Driver &driver, const uint8_t reg, uint16_t &outValue,
                  MLX90614ValidationResult *validation = nullptr) {
  uint8_t buf[2] = {};
  size_t count = driver.ReadRegister(reg, buf);

  if (validation != nullptr) {
    validation->lowLevelReadAttempted = true;
    validation->lowLevelWriteStatus = (count == 0U) ? 0xFFU : 0U;
    validation->lowLevelBytesRead = static_cast<uint8_t>(count);
    if (count > 0U) {
      validation->lowLevelData[0] = buf[0];
    }
    if (count > 1U) {
      validation->lowLevelData[1] = buf[1];
    }
    validation->lowLevelReadSucceeded = (count == 2U);
  }

  if (count != sizeof(buf)) {
    return false;
  }

  outValue = static_cast<uint16_t>(buf[0]) |
             (static_cast<uint16_t>(buf[1]) << 8);
  return true;
}

bool ReadRegisterWithRetry(const MLX90614SubSensorContext &config,
                           MLX90614Driver &driver, const uint8_t reg,
                           uint16_t &outValue,
                           MLX90614ValidationResult *validation = nullptr) {
  for (uint8_t attempt = 1U; attempt <= kMLX90614RegisterReadAttempts;
       ++attempt) {
    ApplyClock(config);
    if (ReadRegister(driver, reg, outValue, validation)) {
      return true;
    }

    if (attempt < kMLX90614RegisterReadAttempts) {
      delay(kMLX90614RegisterRetryDelayMs);
    }
  }

  return false;
}

MLX90614ValidationResult ReadValidationRegisters(
    const MLX90614SubSensorContext &config, MLX90614Driver &driver) {
  MLX90614ValidationResult result = {};

  uint16_t ambientRaw = 0U;
  if (ReadRegisterWithRetry(config, driver, MLX90614_TA, ambientRaw, &result)) {
    result.ambientC = static_cast<double>(RawTemperatureToCentiDegrees(ambientRaw)) /
                      100.0;
  }

  if (kMLX90614InterRegisterDelayMs > 0U) {
    delay(kMLX90614InterRegisterDelayMs);
  }

  uint16_t objectRaw = 0U;
  if (ReadRegisterWithRetry(config, driver, MLX90614_TOBJ1, objectRaw)) {
    result.objectC = static_cast<double>(RawTemperatureToCentiDegrees(objectRaw)) /
                     100.0;
  }

  if (kMLX90614InterRegisterDelayMs > 0U) {
    delay(kMLX90614InterRegisterDelayMs);
  }

  uint16_t emissivityReg = 0U;
  if (ReadRegisterWithRetry(config, driver, MLX90614_EMISSIVITY, emissivityReg)) {
    result.emissivity = static_cast<double>(emissivityReg) / 65535.0;
  }

  return result;
}

bool ValidationPassed(const MLX90614ValidationResult &result) {
  return !isnan(result.ambientC) && !isnan(result.objectC) &&
         !isnan(result.emissivity);
}

int16_t ValidationErrorCode(const MLX90614ValidationResult &result) {
  if (result.lowLevelReadAttempted && result.lowLevelWriteStatus != 0U) {
    return kMLX90614SensorErrorLowLevelWriteFailed;
  }
  if (result.lowLevelReadAttempted && !result.lowLevelReadSucceeded) {
    return kMLX90614SensorErrorLowLevelShortRead;
  }
  if (isnan(result.ambientC)) {
    return kMLX90614SensorErrorAmbientReadFailed;
  }
  if (isnan(result.objectC)) {
    return kMLX90614SensorErrorObjectReadFailed;
  }
  if (isnan(result.emissivity)) {
    return kMLX90614SensorErrorEmissivityReadFailed;
  }
  return kMLX90614SensorErrorBeginFailed;
}

bool WakeDriver(const MLX90614SubSensorContext &config, MLX90614Driver &driver) {
  ApplyPins(config);
  ApplySpecialConfig(config);
  ApplyInternalPullups(config);
  ApplyClock(config);
  driver.Wake();
  ApplyClock(config);
  return true;
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

void PrintBeginAttempt(const MLX90614SubSensorContext &config, const uint8_t attempt,
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

void PrintDoubleOrNaN(const double value) {
  if (isnan(value)) {
    Serial.print("nan");
    return;
  }

  Serial.print(value, 2);
}

void PrintValidationResult(const MLX90614ValidationResult &result,
                           const uint8_t attempt, const char *stage) {
  PrintTimestampMs(millis());
  Serial.print("MLX90614 ");
  Serial.print(stage);
  Serial.print(" attempt ");
  Serial.print(attempt);
  Serial.print(" ambientC=");
  PrintDoubleOrNaN(result.ambientC);
  Serial.print(" objectC=");
  PrintDoubleOrNaN(result.objectC);
  Serial.print(" emissivity=");
  PrintDoubleOrNaN(result.emissivity);
  if (result.lowLevelReadAttempted) {
    Serial.print(" lowLevelWriteStatus=");
    Serial.print(result.lowLevelWriteStatus);
    Serial.print(" lowLevelBytesRead=");
    Serial.print(result.lowLevelBytesRead);
    if (result.lowLevelBytesRead > 0U) {
      Serial.print(" lowLevelData=");
      PrintHexByte(result.lowLevelData[0]);
      Serial.print(' ');
      PrintHexByte(result.lowLevelData[1]);
      Serial.print(' ');
      PrintHexByte(result.lowLevelData[2]);
    }
  }
  Serial.println();
}

void PrintSampleDiagnostics(const MLX90614Capture &sample) {
  PrintTimestampMs(millis());
  Serial.print("MLX90614 first sample validMask=0x");
  PrintHexByte(sample.validMask);
  Serial.print(" error=");
  Serial.print(sample.error);
  Serial.print(" ambient=");
  if ((sample.validMask & kMLX90614SampleValidAmbientTemperature) != 0U) {
    Serial.print(sample.ambientCentiDegrees);
  } else {
    Serial.print("invalid");
  }
  Serial.print(" object=");
  if ((sample.validMask & kMLX90614SampleValidObjectTemperature) != 0U) {
    Serial.print(sample.objectCentiDegrees);
  } else {
    Serial.print("invalid");
  }
  Serial.print(" emissivity=");
  if ((sample.validMask & kMLX90614SampleValidEmissivity) != 0U) {
    Serial.print(sample.emissivityTenThousandths);
  } else {
    Serial.print("invalid");
  }
  Serial.println();
}
#else
void PrintBeginAttempt(const MLX90614SubSensorContext &config,
                       const uint8_t attempt, const int beginResult) {
  (void)config;
  (void)attempt;
  (void)beginResult;
}

void PrintValidationResult(const MLX90614ValidationResult &result,
                           const uint8_t attempt, const char *stage) {
  (void)result;
  (void)attempt;
  (void)stage;
}

void PrintSampleDiagnostics(const MLX90614Capture &sample) {
  (void)sample;
}
#endif

void PopulateTemperatures(const MLX90614SubSensorContext &config,
                          MLX90614Driver &driver,
                          MLX90614Capture &sample) {
  uint16_t ambientRaw = 0U;
  if (!ReadRegisterWithRetry(config, driver, MLX90614_TA, ambientRaw)) {
    CaptureReadError(sample, kMLX90614SensorErrorAmbientReadFailed);
  } else {
    sample.ambientCentiDegrees = RawTemperatureToCentiDegrees(ambientRaw);
    sample.validMask |= kMLX90614SampleValidAmbientTemperature;
  }

  if (kMLX90614InterRegisterDelayMs > 0U) {
    delay(kMLX90614InterRegisterDelayMs);
  }

  uint16_t objectRaw = 0U;
  if (!ReadRegisterWithRetry(config, driver, MLX90614_TOBJ1, objectRaw)) {
    CaptureReadError(sample, kMLX90614SensorErrorObjectReadFailed);
  } else {
    sample.objectCentiDegrees = RawTemperatureToCentiDegrees(objectRaw);
    sample.validMask |= kMLX90614SampleValidObjectTemperature;
  }
}

void UpdateCachedEmissivity(const MLX90614SubSensorContext &config,
                            MLX90614SensorRuntime &runtime,
                            MLX90614Driver &driver) {
  runtime.nextEmissivityRefreshAtMs =
      millis() + kMLX90614EmissivityRefreshIntervalMs;

  uint16_t emissivityReg = 0U;
  if (!ReadRegisterWithRetry(config, driver, MLX90614_EMISSIVITY, emissivityReg)) {
    return;
  }

  runtime.cachedEmissivityReg = emissivityReg;
  runtime.emissivityCached = true;
}

void PopulateEmissivity(const MLX90614SubSensorContext &config,
                        MLX90614SensorRuntime &runtime,
                        MLX90614Driver &driver, MLX90614Capture &sample) {
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

bool CaptureSample(const MLX90614SubSensorContext &config,
                   MLX90614Capture &sample) {
  if (!config.runtime->initialized) {
    sample.error = config.runtime->lastError;
    if (sample.error == kMLX90614SensorErrorNone) {
      sample.error = kMLX90614SensorErrorNotInitialized;
    }
    return true;
  }

  MLX90614Driver *driver = GetDriver(config);
  if (driver == nullptr) {
    return false;
  }

  ApplyClock(config);
  sample.error = kMLX90614SensorErrorNone;
  PopulateTemperatures(config, *driver, sample);
  if (kMLX90614InterRegisterDelayMs > 0U) {
    delay(kMLX90614InterRegisterDelayMs);
  }
  PopulateEmissivity(config, *config.runtime, *driver, sample);
  config.runtime->lastError = sample.error;
  if (!config.runtime->firstSampleDebugPrinted) {
    PrintSampleDiagnostics(sample);
    config.runtime->firstSampleDebugPrinted = true;
  }
  return true;
}

}  // namespace

bool MLX90614SensorBegin(const void *ctx) {
  const MLX90614SubSensorContext *config = GetMLX90614Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  config->runtime->initialized = false;
  config->runtime->firstSampleDebugPrinted = false;
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

    config->runtime->driver = MLX90614Driver(config->i2cAddress, wire,
                                             config->runtime->sdaPin,
                                             config->runtime->sclPin);
    MLX90614Driver *driver = GetDriver(*config);
    if (driver == nullptr) {
      config->runtime->lastError = kMLX90614SensorErrorBeginFailed;
      return false;
    }

    ApplyPins(*config);
    ApplySpecialConfig(*config);
    ApplyInternalPullups(*config);
    ApplyClock(*config);
    const int beginResult = driver->begin();
    PrintBeginAttempt(*config, attempt, beginResult);
    if (beginResult == NO_ERR) {
      ApplyClock(*config);
      const MLX90614ValidationResult validation =
          ReadValidationRegisters(*config, *driver);
      PrintValidationResult(validation, attempt, "validation");
      if (ValidationPassed(validation)) {
        config->runtime->cachedEmissivityReg = static_cast<uint16_t>(
            validation.emissivity * 65535.0);
        config->runtime->emissivityCached = true;
        config->runtime->nextEmissivityRefreshAtMs =
            millis() + kMLX90614EmissivityRefreshIntervalMs;
        config->runtime->initialized = true;
        config->runtime->lastError = kMLX90614SensorErrorNone;
        return true;
      }
      config->runtime->lastError = ValidationErrorCode(validation);
    } else if (beginResult == ERR_IC_VERSION) {
      config->runtime->lastError = kMLX90614SensorErrorBeginInvalidId;
    } else {
      config->runtime->lastError = kMLX90614SensorErrorBeginDataBus;
    }

    (void)WakeDriver(*config, *driver);
    ApplyClock(*config);
    const MLX90614ValidationResult wakeValidation =
        ReadValidationRegisters(*config, *driver);
    PrintValidationResult(wakeValidation, attempt, "wake-validation");
    if (ValidationPassed(wakeValidation)) {
      config->runtime->cachedEmissivityReg = static_cast<uint16_t>(
          wakeValidation.emissivity * 65535.0);
      config->runtime->emissivityCached = true;
      config->runtime->nextEmissivityRefreshAtMs =
          millis() + kMLX90614EmissivityRefreshIntervalMs;
      config->runtime->initialized = true;
      config->runtime->lastError = kMLX90614SensorErrorNone;
      return true;
    }

    const int16_t wakeError = ValidationErrorCode(wakeValidation);
    if (wakeError != kMLX90614SensorErrorBeginFailed) {
      config->runtime->lastError = wakeError;
    }

    if (attempt < kMLX90614BeginAttempts) {
      delay(kMLX90614BeginRetryDelayMs);
    }
  }

  if (config->runtime->lastError == kMLX90614SensorErrorNone) {
    config->runtime->lastError = kMLX90614SensorErrorBeginFailed;
  }
  return false;
}

bool MLX90614DataSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const MLX90614SubSensorContext *config = GetMLX90614Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  MLX90614Capture capture = {};
  if (!CaptureSample(*config, capture)) {
    return false;
  }

  MLX90614DataSampleFrame sample = {
      .ambientCentiDegrees = capture.ambientCentiDegrees,
      .objectCentiDegrees = capture.objectCentiDegrees,
  };
  CopyDataFrameToCan(sample, outFrame);
  return true;
}

bool MLX90614StatsSensorSample(const void *ctx, CANFDMessage &outFrame) {
  const MLX90614SubSensorContext *config = GetMLX90614Context(ctx);
  if (config == nullptr || config->runtime == nullptr) {
    return false;
  }

  MLX90614Capture capture = {};
  if (!CaptureSample(*config, capture)) {
    return false;
  }

  MLX90614StatsSampleFrame sample = {
      .version = kMLX90614SampleFrameVersion,
      .validMask = capture.validMask,
      .emissivityTenThousandths = capture.emissivityTenThousandths,
      .error = capture.error,
  };
  CopyStatsFrameToCan(sample, outFrame);
  return true;
}
