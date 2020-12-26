#include "trace.h"
#include <Arduino.h>
#include <stdio.h>

#define TRACE_PORT Serial

extern "C"
{
    FILE uartout; //= {0}; Global variable is always zero initialized. No need to explicitly state this.
}

static int uart_putchar(char c, FILE *)
{
    TRACE_PORT.write(c);
    return 0;
}

void trace_init(unsigned long baud)
{
    TRACE_PORT.begin(115200);
    // UCSR0B &= ~(1<<RXEN0); //disable RX capability. Frees up pin 0 and OC3A
    fdev_setup_stream(&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE); //setup uart out stream
    stdout = &uartout;
    
    printf_P(PSTR("start\n")); //puts_P isn't more efficient here since we don't use it anywhere else.
}