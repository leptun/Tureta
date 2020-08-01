#pragma once
#include "inttypes.h"
#include "Arduino.h"

class GPIO
{
public:
    GPIO(uint8_t pin)
    : DDRx(portModeRegister(digitalPinToPort(pin)))
    , PINx(portInputRegister(digitalPinToPort(pin)))
    , PORTx(portOutputRegister(digitalPinToPort(pin)))
    , on_mask(digitalPinToBitMask(pin))
    , off_mask(~on_mask)
    {};

    bool read();
    void write(bool val);
    void setInput();
    void setOutput();

private:
    volatile uint8_t* DDRx;
    volatile uint8_t* PINx;
    volatile uint8_t* PORTx;
    const uint8_t on_mask;
    const uint8_t off_mask;
};