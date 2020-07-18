#pragma once
#include <inttypes.h>

extern void timerInit(uint8_t timer);
extern void timerSet(uint8_t timer, uint16_t val);
extern void timerDisable(uint8_t timer);