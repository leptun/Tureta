#include "Rasnita.h"
#include <Arduino.h>

void rasnita_t::init()
{
    pin.write(LOW);
    pin.setInput();
}

void rasnita_t::process(bool val)
{
    if (val)
        pin.setOutput();
    else
        pin.setInput();
}