#pragma once

#include <inttypes.h>



#define JOY_SYNC 0x69
#define JOY_BAUD 115200
#define JOY_PORT Serial1

#define JOY_ADC_CHAN_CNT 3


struct remoteRegister_t
{
    const uint8_t SYNC = JOY_SYNC;
    uint16_t JOY[JOY_ADC_CHAN_CNT];
    struct
    {
        uint8_t Button0 : 1;
        uint8_t Button1 : 1;
        uint8_t Button2 : 1;
        uint8_t Button3 : 1;
        uint8_t DpadUP : 1;
        uint8_t DpadRIGHT : 1;
        uint8_t DpadLEFT : 1;
        uint8_t DpadDOWN : 1;
    } GPIO;
    uint8_t CRC;
};

extern remoteRegister_t remoteRegister;

extern bool joystick_update();
extern void joystick_init();
