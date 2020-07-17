#include <Arduino.h>
#include <wiring_private.h>
#include <avr/pgmspace.h>
#include "Timer.h"
#include "speed_lookuptable.h"

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

#define joyDeadzone 15
#define NUM_AXIS 2

int16_t joyIdlePos = 512;
ShortTimer analogUpdateTimer;

static void timerInit(uint8_t timer)
{
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
        cbi(TCCR1B, CS11);
        sbi(TCCR1B, CS10);
        sbi(TCCR1A, WGM13);
        cbi(TCCR1A, WGM10);
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
        cbi(TCCR3B, CS31);
        sbi(TCCR3B, CS30);
        sbi(TCCR3A, WGM33);
        cbi(TCCR3A, WGM30);
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
        cbi(TCCR4B, CS41);
        sbi(TCCR4B, CS40);
        sbi(TCCR4A, WGM43);
        cbi(TCCR4A, WGM40);
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
        cbi(TCCR5B, CS51);
        sbi(TCCR5B, CS50);
        sbi(TCCR5A, WGM53);
        cbi(TCCR5A, WGM50);
        break;
      #endif //TCCR5A

      case NOT_ON_TIMER:
      default:
        Serial.print(F("Error: "));
        Serial.print(timer);
        Serial.println(F(" is not a valid timer!"));
    }
}

static void timerSet(uint8_t timer, uint16_t val)
{
    switch (timer)
    {
        #ifdef TCCR1A
        #ifdef COM1A1
            case TIMER1A:
                sbi(TCCR1A, COM1A1);
                ICR1 = val;
            break;
        #endif //COM1A1
        #ifdef COM1B1
            case TIMER1B:
                sbi(TCCR1B, COM1B1);
                ICR1 = val;
            break;
        #endif //COM1B1
        #ifdef COM1C1
        case TIMER1C:
            case TIMER1C:
                sbi(TCCR1C, COM1C1);
                ICR1 = val;
            break;
        #endif //COM1C1
        #endif //TCCR1A

        #ifdef TCCR3A
        #ifdef COM3A1
            case TIMER3A:
                sbi(TCCR3A, COM3A1);
                ICR3 = val;
            break;
        #endif //COM3A1
        #ifdef COM3B1
            case TIMER3B:
                #if defined(__AVR_ATmega328PB__) // Fix 324PB/328PB silicon bug
                sbi(PORTD, PD2);
                #elif defined(__AVR_ATmega324PB__)
                sbi(PORTB, PB7);
                #endif
                sbi(TCCR3B, COM3B1);
                ICR3 = val;
            break;
        #endif //COM3B1
        #ifdef COM3C1
        case TIMER3C:
            case TIMER3C:
                sbi(TCCR3C, COM3C1);
                ICR3 = val;
            break;
        #endif //COM3C1
        #endif //TCCR3A

        #if defined(TCCR4C) && !defined(COM4D1)
        #ifdef COM4A1
            case TIMER4A:
                sbi(TCCR4A, COM4A1);
                ICR4 = val;
            break;
        #endif //COM4A1
        #ifdef COM4B1
            case TIMER4B:
                #if defined(__AVR_ATmega328PB__) // Fix 324PB/328PB silicon bug
                sbi(PORTD, PD2);
                #elif defined(__AVR_ATmega324PB__)
                sbi(PORTB, PB7);
                #endif
                sbi(TCCR4B, COM4B1);
                ICR4 = val;
            break;
        #endif //COM4B1
        #ifdef COM4C1
        case TIMER4C:
            case TIMER4C:
                sbi(TCCR4C, COM4C1);
                ICR4 = val;
            break;
        #endif //COM4C1
        #endif //defined(TCCR4C) && !defined(COM4D1)

        #ifdef TCCR5A
        #ifdef COM5A1
            case TIMER5A:
                sbi(TCCR5A, COM5A1);
                ICR5 = val;
            break;
        #endif //COM5A1
        #ifdef COM5B1
            case TIMER5B:
                sbi(TCCR5B, COM5B1);
                ICR5 = val;
            break;
        #endif //COM5B1
        #ifdef COM5C1
        case TIMER5C:
            case TIMER5C:
                sbi(TCCR5C, COM5C1);
                ICR5 = val;
            break;
        #endif //COM5C1
        #endif //TCCR5A
    }
}

static void timerDisable(uint8_t timer)
{
    switch (timer)
    {
        #ifdef TCCR1A
        #ifdef COM1A1
            case TIMER1A:
                cbi(TCCR1A, COM1A1);
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
                cbi(TCCR3A, COM3A1);
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
                cbi(TCCR4A, COM4A1);
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
                cbi(TCCR5A, COM5A1);
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
}

class axis_t
{
public:
    axis_t(int8_t step_pin, int8_t dir_pin, int8_t en_pin, int8_t joy_pin, int8_t dir_flip)
        : step_pin(step_pin)
        , dir_pin(dir_pin)
        , en_pin(en_pin)
        , joy_pin(joy_pin)
        , dir_flip(dir_flip)
    {};

    void init();
    void process();

private:
    int8_t joyToDirection(int16_t value);
    uint16_t joyToSpeed(int16_t value);

    int8_t step_pin;
    int8_t dir_pin;
    int8_t en_pin;
    int8_t joy_pin;
    int8_t dir_flip;
};

axis_t axis[NUM_AXIS] =
{
    {0, 8, -1, A2, 1},
    {2, 7, -1, A3, 1},
};

int8_t axis_t::joyToDirection(int16_t value)
{
    int16_t delta = value - joyIdlePos;
    if (abs(delta) < joyDeadzone)
        return 0;
    else if (delta < 0)
        return -1 * dir_flip;
    else
        return 1 * dir_flip;
}

uint16_t axis_t::joyToSpeed(int16_t value)
{
    return calc_timer(map(abs(value - joyIdlePos), joyDeadzone, joyIdlePos, 32, 10000));
}

void axis_t::init()
{
    pinMode(step_pin, OUTPUT); digitalWrite(step_pin, LOW);
    pinMode(dir_pin, OUTPUT); digitalWrite(dir_pin, LOW);
    pinMode(en_pin, OUTPUT); digitalWrite(en_pin, LOW);
    pinMode(joy_pin, INPUT); digitalWrite(joy_pin, LOW);
    timerInit(digitalPinToTimer(step_pin));
}

void axis_t::process()
{
    int16_t joyRead = analogRead(joy_pin);
    uint16_t timerVal = joyToSpeed(joyRead);
    uint8_t timer = digitalPinToTimer(step_pin);
    
    int8_t stepperDirection = joyToDirection(joyRead);
    if (stepperDirection != 0)
    {
        digitalWrite(dir_pin, (stepperDirection > 0));
        timerSet(timer, timerVal);
    }
    else
    {
        timerDisable(timer);
    }
    
    static char outText[37];
    sprintf_P(outText, PSTR("%i, %hi, %u"), joyRead, stepperDirection, timerVal);
    Serial.println(outText);
}

void setup() {
    Serial.begin(115200);
    UCSR0B &= ~(1<<RXEN0);
    Serial.println(F("start"));

    for (uint8_t i = 0; i < NUM_AXIS; i++)
        axis[i].init();
    
    analogUpdateTimer.start();
}

void loop() {
    if (analogUpdateTimer.expired(10))
    {
        for (uint8_t i = 0; i < NUM_AXIS; i++)
            axis[i].process();
        analogUpdateTimer.start();
    }
}
