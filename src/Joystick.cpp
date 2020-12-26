#include "Joystick.h"
#include <stdio.h>
#include <Arduino.h>
#include "CRC.h"

remoteRegister_t remoteRegister, _remoteRegister;

bool joystick_update()
{
    static uint8_t joy_index = 0;

    if (!JOY_PORT.available())
        return false;
    
    uint8_t c = JOY_PORT.read();
    printf_P(PSTR("%02hX "), c);
    if (joy_index == 0)
    {
        if (c != JOY_SYNC)
        {
            puts_P(PSTR("~!SYNC"));
            return false;
        }
    }
    else
    {
        *((uint8_t*)(&_remoteRegister) + joy_index) = c;
    }
    
    joy_index++;
    
    if (joy_index < sizeof(_remoteRegister))
        return false; //not all data was received

    CRC_reset();
    for (uint8_t* i = (uint8_t*)(&_remoteRegister); i < (uint8_t*)(&_remoteRegister) + joy_index - 1; i++)
    {
        CRC_add(*i);
    }
    if (_remoteRegister.CRC != CRC_get())
    {
        puts_P(PSTR("CRC ERROR"));
        joy_index = 0;
        return false;
    }
    puts_P(PSTR("OK"));
    joy_index = 0;
    memcpy(&remoteRegister, &_remoteRegister, sizeof(remoteRegister));
    return true;
}

void joystick_init()
{
    JOY_PORT.begin(JOY_BAUD);
}