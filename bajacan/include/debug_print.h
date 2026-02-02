#pragma once

#include <ACAN2517FD.h>
#include <stdint.h>

#ifndef BAJACAN_ENABLE_DEBUG_PRINTS
#define BAJACAN_ENABLE_DEBUG_PRINTS 0
#endif

void PrintCanFrame(const CANFDMessage &frame);
void PrintTimestampMs(uint32_t nowMs);
void PrintSensorPoll(const char *name, const CANFDMessage &frame,
                     uint32_t nowMs);
void PrintCanTxResult(const CANFDMessage &frame, uint32_t nowMs, bool sent);
