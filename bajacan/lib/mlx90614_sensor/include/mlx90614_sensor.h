#pragma once

#include <DFRobot_MLX90614.h>
#include <config.h>
#include <stdint.h>

// Preserve the conventional default MLX90614 address used by existing configs.
constexpr uint8_t MLX90614_I2CADDR = 0x5AU;

constexpr uint8_t kMLX90614SampleFrameVersion = 1U;

constexpr uint8_t kMLX90614SampleValidAmbientTemperature = 0x01U;
constexpr uint8_t kMLX90614SampleValidObjectTemperature = 0x02U;
constexpr uint8_t kMLX90614SampleValidEmissivity = 0x04U;

constexpr uint16_t kMLX90614EmissivityScale = 10000U;

constexpr int16_t kMLX90614SensorErrorNone = 0;
constexpr int16_t kMLX90614SensorErrorBeginDataBus = -1;
constexpr int16_t kMLX90614SensorErrorBeginInvalidId = -2;
constexpr int16_t kMLX90614SensorErrorAmbientReadFailed = -3;
constexpr int16_t kMLX90614SensorErrorObjectReadFailed = -4;
constexpr int16_t kMLX90614SensorErrorEmissivityReadFailed = -5;
constexpr int16_t kMLX90614SensorErrorBeginFailed = -15;
constexpr int16_t kMLX90614SensorErrorNotInitialized = -16;

struct __attribute__((packed)) MLX90614SampleFrame {
  uint8_t version;
  uint8_t validMask;
  int32_t ambientCentiDegrees;
  int32_t objectCentiDegrees;
  uint16_t emissivityTenThousandths;
  int16_t error;
};

static_assert(sizeof(MLX90614SampleFrame) <= 64U,
              "MLX90614 sample frame must fit in one CAN FD payload");

constexpr uint8_t kMLX90614SensorPayloadSize =
    static_cast<uint8_t>(sizeof(MLX90614SampleFrame));

class MLX90614Driver : public DFRobot_MLX90614_I2C {
 public:
  MLX90614Driver(uint8_t i2cAddress = MLX90614_I2CADDR,
                 TwoWire *wire = &Wire, uint32_t clockHz = 0U)
      : DFRobot_MLX90614_I2C(i2cAddress, wire),
        wire_(wire),
        clockHz_(clockHz) {}

  int begin(void) override {
    if (wire_ == nullptr) {
      return kMLX90614SensorErrorBeginDataBus;
    }

    wire_->begin();
    ApplyConfiguredClock();
    delay(50);

    uint8_t idBuf[2] = {};
    int result = ReadIdRegister(idBuf);
    if (result == NO_ERR) {
      return result;
    }

    enterSleepMode(false);
    ApplyConfiguredClock();
    delay(50);

    result = ReadIdRegister(idBuf);
    ApplyConfiguredClock();
    return result;
  }

  size_t ReadRegister(uint8_t reg, void *buf) {
    return DFRobot_MLX90614_I2C::readReg(reg, buf);
  }

 private:
  int ReadIdRegister(uint8_t *buf) {
    if (buf == nullptr || ReadRegister(MLX90614_ID_NUMBER, buf) != 2U) {
      return ERR_DATA_BUS;
    }

    const uint16_t id =
        static_cast<uint16_t>(buf[0]) |
        (static_cast<uint16_t>(buf[1]) << 8);
    return id == 0U ? ERR_IC_VERSION : NO_ERR;
  }

  void ApplyConfiguredClock() {
    if (wire_ != nullptr && clockHz_ != 0U) {
      wire_->setClock(clockHz_);
    }
  }

  TwoWire *wire_;
  uint32_t clockHz_;
};

struct MLX90614SensorRuntime {
  explicit MLX90614SensorRuntime(TwoWire *wireBus = &Wire)
      : wire(wireBus), driver(MLX90614_I2CADDR, wireBus, 0U) {}

  TwoWire *wire = &Wire;
  MLX90614Driver driver;
  bool initialized = false;
  int16_t lastError = kMLX90614SensorErrorNone;
  uint16_t cachedEmissivityReg = 0U;
  bool emissivityCached = false;
  uint32_t nextEmissivityRefreshAtMs = 0U;
};

struct MLX90614SensorContext {
  SensorContext base;
  MLX90614SensorRuntime *runtime;
  uint8_t i2cAddress;
  uint32_t clockHz;
};

bool MLX90614SensorBegin(const void *ctx);
bool MLX90614SensorSample(const void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor MakeMLX90614Sensor(
    const MLX90614SensorContext *ctx) {
  return SensorDescriptor{
      .context = ctx,
      .begin = MLX90614SensorBegin,
      .sample = MLX90614SensorSample,
      .suspend = nullptr,
      .resume = nullptr,
  };
}
