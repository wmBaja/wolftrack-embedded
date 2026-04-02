#pragma once

#include <ACAN2517FD.h>
#include <stdint.h>

#ifndef BAJACAN_ENABLE_DEBUG_PRINTS
#define BAJACAN_ENABLE_DEBUG_PRINTS 0
#endif

void PrintCanFrame(const CANFDMessage &frame);
void PrintTimestampMs(uint32_t nowMs);
void PrintGroupPoll(const char *name, const CANFDMessage &frame,
                    uint32_t nowMs);
void PrintGroupMemberZeroFill(const char *groupName, const char *sensorName,
                              uint32_t nowMs);
void PrintGroupConfigError(const char *groupName, uint32_t nowMs);
void PrintCanTxResult(const CANFDMessage &frame, uint32_t nowMs, bool sent);
