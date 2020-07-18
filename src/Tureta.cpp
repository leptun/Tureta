#include <Arduino.h>
#include <avr/pgmspace.h>
#include "Timer.h"
#include "speed_lookuptable.h"
#include "hwTimer.h"

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

#define joyDeadzone 20
#define NUM_AXIS 1
#define INPUT_PERIOD 10

int16_t joyIdlePos = 512;
ShortTimer analogUpdateTimer;
char debugBuffer[100];

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

    uint8_t timer;
};

axis_t axis[NUM_AXIS] =
{
    {9, 8, -1, A2, 1},
    // {2, 7, -1, A3, 1},
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
    timer = digitalPinToTimer(step_pin);
    timerInit(timer);
}

void axis_t::process()
{
    int16_t joyRead = analogRead(joy_pin);
    uint16_t timerVal = joyToSpeed(joyRead);
    
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
    
    // sprintf_P(debugBuffer, PSTR("%i, %hi, %u"), joyRead, stepperDirection, timerVal);
    sprintf_P(debugBuffer, PSTR("TCCR1A:%02hX, TCCR1B:%02hX, ICR1:%u, OCR1A:%u"), TCCR1A, TCCR1B, ICR1, OCR1A);
    Serial.println(debugBuffer);
}

void setup() {
    Serial.begin(115200);
    // UCSR0B &= ~(1<<RXEN0);
    Serial.println(F("start"));

    for (uint8_t i = 0; i < NUM_AXIS; i++)
        axis[i].init();
    
    analogUpdateTimer.start();
}

void loop() {
    if (analogUpdateTimer.expired(INPUT_PERIOD))
    {
        for (uint8_t i = 0; i < NUM_AXIS; i++)
            axis[i].process();
        analogUpdateTimer.start();
    }
}
