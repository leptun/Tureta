#include "Serial.h"
#include <Arduino.h>
#include <stdio.h>

extern "C"
{
    FILE uartout; //= {0}; Global variable is always zero initialized. No need to explicitly state this.
}

int uart_putchar(char c, FILE *)
{
    SerialPort.write(c);
    return 0;
}

void UART_init(unsigned long baud)
{
    SerialPort.begin(115200);
    fdev_setup_stream(&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE); //setup uart out stream
    stdout = &uartout;
    
    printf_P(PSTR("start\n"));
}