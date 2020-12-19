#pragma once
#include "GPIO.h"

class rasnita_t
{
public:
    rasnita_t(uint8_t pin) : pin(GPIO(pin)) {};
    void init();
    void process(bool target);
    
private:
    GPIO pin;
};