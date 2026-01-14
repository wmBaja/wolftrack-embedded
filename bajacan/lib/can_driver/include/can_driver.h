#pragma once

#include <ACAN2517FD.h>
#include <config.h>

void SleepCanDriver(ACAN2517FD &driver, const BoardConfig &config);
void WakeCanDriver(ACAN2517FD &driver, const BoardConfig &config);
