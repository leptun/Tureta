#include <Arduino.h>
#include <stdio.h>
#include <wiring_private.h>
#include "speed_lookuptable.h"

#define OCR_MIN (MAX_TIMER/2)

void timerInit(uint8_t timer)
{
    // printf_P(PSTR("timerInit: %hu"), timer);

    switch (timer)
    {
    #ifdef TCCR1A
    #ifdef COM1A1
        case TIMER1A:
    #endif //COM1A1
    #ifdef COM1B1
        case TIMER1B:
    #endif //COM1B1
    #ifdef COM1C1
        case TIMER1C:
    #endif //COM1C1
            cbi(TCCR1B, CS12);
            sbi(TCCR1B, CS11);
            cbi(TCCR1B, CS10);
            sbi(TCCR1B, WGM13);
            sbi(TCCR1B, WGM12);
            sbi(TCCR1A, WGM11);
            sbi(TCCR1A, WGM10);
        #ifdef COM1B1
            OCR1B = OCR_MIN;
        #endif //COM1B1
        #ifdef COM1C1
            OCR1C = OCR_MIN;
        #endif //COM1C1
        break;
    #endif //TCCR1A

    #ifdef TCCR3A
    #ifdef COM3A1
        case TIMER3A:
    #endif //COM3A1
    #ifdef COM3B1
        case TIMER3B:
    #endif //COM3B1
    #ifdef COM3C1
        case TIMER3C:
    #endif //COM3C1
            cbi(TCCR3B, CS32);
            sbi(TCCR3B, CS31);
            cbi(TCCR3B, CS30);
            sbi(TCCR3B, WGM33);
            sbi(TCCR3B, WGM32);
            sbi(TCCR3A, WGM31);
            sbi(TCCR3A, WGM30);
        #ifdef COM3B1
            OCR3B = OCR_MIN;
        #endif //COM3B1
        #ifdef COM3C1
            OCR3C = OCR_MIN;
        #endif //COM3C1
        break;
    #endif //TCCR3A

    #if defined(TCCR4C) && !defined(COM4D1)
    #ifdef COM4A1
        case TIMER4A:
    #endif //COM4A1
    #ifdef COM4B1
        case TIMER4B:
    #endif //COM4B1
    #ifdef COM4C1
        case TIMER4C:
    #endif //COM4C1
            cbi(TCCR4B, CS42);
            sbi(TCCR4B, CS41);
            cbi(TCCR4B, CS40);
            sbi(TCCR4B, WGM43);
            sbi(TCCR4B, WGM42);
            sbi(TCCR4A, WGM41);
            sbi(TCCR4A, WGM40);
        #ifdef COM4B1
            OCR4B = OCR_MIN;
        #endif //COM4B1
        #ifdef COM4C1
            OCR4C = OCR_MIN;
        #endif //COM4C1
        break;
    #endif //defined(TCCR4C) && !defined(COM4D1)

    #ifdef TCCR5A
    #ifdef COM5A1
        case TIMER5A:
    #endif //COM5A1
    #ifdef COM5B1
        case TIMER5B:
    #endif //COM5B1
    #ifdef COM5C1
        case TIMER5C:
    #endif //COM5C1
            cbi(TCCR5B, CS52);
            sbi(TCCR5B, CS51);
            cbi(TCCR5B, CS50);
            sbi(TCCR5B, WGM53);
            sbi(TCCR5B, WGM52);
            sbi(TCCR5A, WGM51);
            sbi(TCCR5A, WGM50);
        #ifdef COM5B1
            OCR5B = OCR_MIN;
        #endif //COM5B1
        #ifdef COM5C1
            OCR5C = OCR_MIN;
        #endif //COM5C1
        break;
    #endif //TCCR5A
        case NOT_ON_TIMER:
        default:
            Serial.print(F("Error: "));
            Serial.print(timer);
            Serial.println(F(" is not a valid timer!"));
    }
    // printf_P(PSTR("%hu, %hu, %u"), TCCR1A, TCCR1B, ICR1);
}

void timerSet(uint8_t timer, uint16_t val)
{
    // printf_P(PSTR("timerSet: %hu, %u"), timer, val);

    switch (timer)
    {
        #ifdef TCCR1A
        #ifdef COM1A1
            case TIMER1A:
                sbi(TCCR1A, COM1A0);
                OCR1A = val>>1;
            break;
        #endif //COM1A1
        #ifdef COM1B1
            case TIMER1B:
                sbi(TCCR1A, COM1B1);
                OCR1A = val;
            break;
        #endif //COM1B1
        #ifdef COM1C1
        case TIMER1C:
            case TIMER1C:
                sbi(TCCR1A, COM1C1);
                OCR1A = val;
            break;
        #endif //COM1C1
        #endif //TCCR1A

        #ifdef TCCR3A
        #ifdef COM3A1
            case TIMER3A:
                sbi(TCCR3A, COM3A0);
                OCR3A = val>>1;
            break;
        #endif //COM3A1
        #ifdef COM3B1
            case TIMER3B:
                #if defined(__AVR_ATmega328PB__) // Fix 324PB/328PB silicon bug
                sbi(PORTD, PD2);
                #elif defined(__AVR_ATmega324PB__)
                sbi(PORTB, PB7);
                #endif
                sbi(TCCR3A, COM3B1);
                OCR3A = val;
            break;
        #endif //COM3B1
        #ifdef COM3C1
        case TIMER3C:
            case TIMER3C:
                sbi(TCCR3A, COM3C1);
                OCR3A = val;
            break;
        #endif //COM3C1
        #endif //TCCR3A

        #if defined(TCCR4C) && !defined(COM4D1)
        #ifdef COM4A1
            case TIMER4A:
                sbi(TCCR4A, COM4A0);
                OCR4A = val>>1;
            break;
        #endif //COM4A1
        #ifdef COM4B1
            case TIMER4B:
                #if defined(__AVR_ATmega328PB__) // Fix 324PB/328PB silicon bug
                sbi(PORTD, PD2);
                #elif defined(__AVR_ATmega324PB__)
                sbi(PORTB, PB7);
                #endif
                sbi(TCCR4A, COM4B1);
                OCR4A = val;
            break;
        #endif //COM4B1
        #ifdef COM4C1
        case TIMER4C:
            case TIMER4C:
                sbi(TCCR4A, COM4C1);
                OCR4A = val;
            break;
        #endif //COM4C1
        #endif //defined(TCCR4C) && !defined(COM4D1)

        #ifdef TCCR5A
        #ifdef COM5A1
            case TIMER5A:
                sbi(TCCR5A, COM5A0);
                OCR5A = val>>1;
            break;
        #endif //COM5A1
        #ifdef COM5B1
            case TIMER5B:
                sbi(TCCR5A, COM5B1);
                OCR5A = val;
            break;
        #endif //COM5B1
        #ifdef COM5C1
        case TIMER5C:
            case TIMER5C:
                sbi(TCCR5A, COM5C1);
                OCR5A = val;
            break;
        #endif //COM5C1
        #endif //TCCR5A
    }
    // printf_P(PSTR("%hu, %hu, %u"), TCCR3A, TCCR3B, OCR3A);
}

void timerDisable(uint8_t timer)
{
    // printf_P(PSTR("timerDisable: %hu"), timer);

    switch (timer)
    {
        #ifdef TCCR1A
        #ifdef COM1A1
            case TIMER1A:
                cbi(TCCR1A, COM1A0);
            break;
        #endif //COM1A1
        #ifdef COM1B1
            case TIMER1B:
                cbi(TCCR1B, COM1B1);
            break;
        #endif //COM1B1
        #ifdef COM1C1
        case TIMER1C:
            case TIMER1C:
                cbi(TCCR1C, COM1C1);
            break;
        #endif //COM1C1
        #endif //TCCR1A

        #ifdef TCCR3A
        #ifdef COM3A1
            case TIMER3A:
                cbi(TCCR3A, COM3A0);
            break;
        #endif //COM3A1
        #ifdef COM3B1
            case TIMER3B:
                #if defined(__AVR_ATmega328PB__) // Fix 324PB/328PB silicon bug
                cbi(PORTD, PD2);
                #elif defined(__AVR_ATmega324PB__)
                cbi(PORTB, PB7);
                #endif
                cbi(TCCR3B, COM3B1);
            break;
        #endif //COM3B1
        #ifdef COM3C1
        case TIMER3C:
            case TIMER3C:
                cbi(TCCR3C, COM3C1);
            break;
        #endif //COM3C1
        #endif //TCCR3A

        #if defined(TCCR4C) && !defined(COM4D1)
        #ifdef COM4A1
            case TIMER4A:
                cbi(TCCR4A, COM4A0);
            break;
        #endif //COM4A1
        #ifdef COM4B1
            case TIMER4B:
                #if defined(__AVR_ATmega328PB__) // Fix 324PB/328PB silicon bug
                cbi(PORTD, PD2);
                #elif defined(__AVR_ATmega324PB__)
                cbi(PORTB, PB7);
                #endif
                cbi(TCCR4B, COM4B1);
            break;
        #endif //COM4B1
        #ifdef COM4C1
        case TIMER4C:
            case TIMER4C:
                cbi(TCCR4C, COM4C1);
            break;
        #endif //COM4C1
        #endif //defined(TCCR4C) && !defined(COM4D1)

        #ifdef TCCR5A
        #ifdef COM5A1
            case TIMER5A:
                cbi(TCCR5A, COM5A0);
            break;
        #endif //COM5A1
        #ifdef COM5B1
            case TIMER5B:
                cbi(TCCR5B, COM5B1);
            break;
        #endif //COM5B1
        #ifdef COM5C1
        case TIMER5C:
            case TIMER5C:
                cbi(TCCR5C, COM5C1);
            break;
        #endif //COM5C1
        #endif //TCCR5A
    }

    // printf_P(PSTR("%hu, %hu, %u"), TCCR1A, TCCR1B, ICR1);
}