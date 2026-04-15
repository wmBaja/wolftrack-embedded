#pragma once

#include <SparkFun_BNO08x_Arduino_Library.h>
#include <config.h>
#include <stdint.h>

constexpr uint8_t kBNO085SampleFrameVersion = 1U;

constexpr uint8_t kBNO085ReportMaskAccelerometer = 0x01U;
constexpr uint8_t kBNO085ReportMaskGyro = 0x02U;
constexpr uint8_t kBNO085ReportMaskLinearAcceleration = 0x04U;
constexpr uint8_t kBNO085ReportMaskRotationVector = 0x08U;

constexpr uint16_t kBNO085DefaultReportIntervalMs = 10U;

constexpr int16_t kBNO085SensorErrorNone = 0;
constexpr int16_t kBNO085SensorErrorNotInitialized = -1;
constexpr int16_t kBNO085SensorErrorBeginFailed = -2;
constexpr int16_t kBNO085SensorErrorConfigureReportsFailed = -3;
constexpr int16_t kBNO085SensorErrorWakeFailed = -4;
constexpr int16_t kBNO085SensorErrorSleepFailed = -5;

struct __attribute__((packed)) BNO085Vector3 {
  float x;
  float y;
  float z;
};

struct __attribute__((packed)) BNO085Quaternion {
  float i;
  float j;
  float k;
  float real;
};

struct __attribute__((packed)) BNO085SampleFrameWithoutRotation {
  uint8_t version;
  uint8_t validMask;
  uint8_t accelerometerAccuracy;
  uint8_t gyroAccuracy;
  uint8_t linearAccelerationAccuracy;
  uint8_t rotationVectorAccuracy;
  int16_t error;
  BNO085Vector3 accelerometer;
  BNO085Vector3 angularVelocity;
  BNO085Vector3 linearAcceleration;
};

struct __attribute__((packed)) BNO085SampleFrameWithRotation {
  uint8_t version;
  uint8_t validMask;
  uint8_t accelerometerAccuracy;
  uint8_t gyroAccuracy;
  uint8_t linearAccelerationAccuracy;
  uint8_t rotationVectorAccuracy;
  int16_t error;
  BNO085Vector3 accelerometer;
  BNO085Vector3 angularVelocity;
  BNO085Vector3 linearAcceleration;
  BNO085Quaternion rotationVector;
  float rotationVectorAccuracyRad;
};

static_assert(sizeof(BNO085SampleFrameWithoutRotation) <= 64U,
              "BNO085 sample frame without rotation must fit in one CAN FD payload");

constexpr bool kBNO085SensorHasRotationVector =
    sizeof(BNO085SampleFrameWithRotation) <= 64U;

template <bool Condition, typename TrueType, typename FalseType>
struct BNO085SelectType {
  using type = TrueType;
};

template <typename TrueType, typename FalseType>
struct BNO085SelectType<false, TrueType, FalseType> {
  using type = FalseType;
};

using BNO085SampleFrame =
    typename BNO085SelectType<kBNO085SensorHasRotationVector,
                              BNO085SampleFrameWithRotation,
                              BNO085SampleFrameWithoutRotation>::type;

static_assert(sizeof(BNO085SampleFrame) <= 64U,
              "BNO085 sample frame must fit in one CAN FD payload");

constexpr uint8_t kBNO085SensorPayloadSize =
    static_cast<uint8_t>(sizeof(BNO085SampleFrame));

struct BNO085SensorRuntime {
  BNO08x driver;
  bool initialized = false;
  bool reportsConfigured = false;
  uint8_t validMask = 0U;
  uint8_t accelerometerAccuracy = 0U;
  uint8_t gyroAccuracy = 0U;
  uint8_t linearAccelerationAccuracy = 0U;
  uint8_t rotationVectorAccuracy = 0U;
  int16_t lastError = kBNO085SensorErrorNone;
  BNO085Vector3 accelerometer = {};
  BNO085Vector3 angularVelocity = {};
  BNO085Vector3 linearAcceleration = {};
  BNO085Quaternion rotationVector = {};
  float rotationVectorAccuracyRad = 0.0f;
};

struct BNO085SensorContext {
  SensorContext base;
  BNO085SensorRuntime *runtime;
  uint8_t i2cAddress;
  uint32_t clockHz;
  int8_t interruptPin;
  int8_t resetPin;
  uint16_t reportIntervalMs;
};

bool BNO085SensorBegin(const void *ctx);
bool BNO085SensorSample(const void *ctx, CANFDMessage &outFrame);
void BNO085SensorSuspend(const void *ctx);
void BNO085SensorResume(const void *ctx);

constexpr SensorDescriptor MakeBNO085Sensor(const BNO085SensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = BNO085SensorBegin,
      .sample = BNO085SensorSample,
      .suspend = BNO085SensorSuspend,
      .resume = BNO085SensorResume,
  };
}
