#include <Arduino.h>
#include <debug_print.h>

void PrintCanFrame(const CANFDMessage &frame) {
  Serial.print("id=0x");
  Serial.print(frame.id, HEX);
  Serial.print(" ext=");
  Serial.print(frame.ext ? 1 : 0);
  Serial.print(" len=");
  Serial.print(frame.len);
  Serial.print(" data=");
  for (uint8_t i = 0; i < frame.len; ++i) {
    if (i != 0) {
      Serial.print(' ');
    }
    if (frame.data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(frame.data[i], HEX);
  }
}

void PrintTimestampMs(const uint32_t nowMs) {
  Serial.print('[');
  Serial.print(nowMs);
  Serial.print(" ms] ");
}

void PrintSensorPoll(const char *name, const CANFDMessage &frame,
                     const uint32_t nowMs) {
  PrintTimestampMs(nowMs);
  Serial.print("Sensor ");
  Serial.print(name != nullptr ? name : "Unknown");
  Serial.print(" polled ");
  PrintCanFrame(frame);
  Serial.println();
}

void PrintCanTxResult(const CANFDMessage &frame, const uint32_t nowMs,
                      const bool sent) {
  PrintTimestampMs(nowMs);
  // if (sent) {
  //   Serial.print("CAN TX ok ");
  //   PrintCanFrame(frame);
  //   Serial.println();
  // } else {
  //   Serial.print("CAN TX failed id=0x");
  //   Serial.print(frame.id, HEX);
  //   Serial.println();
  // }
}
