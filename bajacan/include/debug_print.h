#pragma once

#include <ACAN2517FD.h>
#include <stdint.h>

void PrintCanFrame(const CANFDMessage &frame);
void PrintTimestampMs(uint32_t nowMs);
void PrintSensorPoll(const char *name, const CANFDMessage &frame,
                     uint32_t nowMs);
void PrintCanTxResult(const CANFDMessage &frame, uint32_t nowMs, bool sent);
