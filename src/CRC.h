#pragma once
#include <inttypes.h>

extern void CRC_reset();
extern void CRC_add(uint8_t v);
extern uint8_t CRC_get();
