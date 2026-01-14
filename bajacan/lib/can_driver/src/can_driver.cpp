#include <Arduino.h>
#include <can_driver.h>

void SleepCanDriver(ACAN2517FD &driver, const BoardConfig &config) {
  driver.clearWakeFlag();
  driver.enableWakeInterrupt();
  digitalWrite(config.canStbyPin, HIGH);
  driver.setOperationMode(ACAN2517FDSettings::Sleep);
}

void WakeCanDriver(ACAN2517FD &driver, const BoardConfig &config) {
  digitalWrite(config.canStbyPin, LOW);
  driver.clearWakeFlag();
  driver.setOperationMode(ACAN2517FDSettings::NormalFD);
}
