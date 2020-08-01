#include "GPIO.h"
#include "macros.h"

bool GPIO::read()
{
    return *PINx & on_mask;
}

void GPIO::write(bool val)
{
    CRITICAL_SECTION_START;
    if (val)
    {
        *PORTx |= on_mask;
    }
    else
    {
        *PORTx &= off_mask;
    }
    CRITICAL_SECTION_END;
}

void GPIO::setInput()
{
    CRITICAL_SECTION_START;
    *DDRx &= off_mask;
    CRITICAL_SECTION_END;

}

void GPIO::setOutput()
{
    CRITICAL_SECTION_START;
    *DDRx |= on_mask;
    CRITICAL_SECTION_END;
}