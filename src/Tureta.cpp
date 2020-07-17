#include <Arduino.h>
#include <wiring_private.h>
#include <avr/pgmspace.h>
#include "fastio.h"
#include "Timer.h"
#include "speed_lookuptable.h"

#define joyDeadzone 15
#define NUM_AXIS 2

class axis_t
{
public:
    axis_t(int8_t step_pin, int8_t dir_pin, int8_t en_pin, int8_t joy_pin, int8_t dir_flip, uint8_t timer)
        : step_pin(step_pin)
        , dir_pin(dir_pin)
        , en_pin(en_pin)
        , joy_pin(joy_pin)
        , dir_flip(dir_flip)
        , timer(timer)
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
    {0, 8, -1, A2, 1, 3},
    {2, 7, -1, A3, 1, 4},
};

int16_t joyIdlePos = 511;
ShortTimer analogUpdateTimer;

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
    pinMode(joy_pin, INPUT);
}

void axis_t::process()
{
    int16_t joyRead = analogRead(joy_pin);
    uint16_t timerVal = joyToSpeed(joyRead);
    
    int8_t stepperDirection = joyToDirection(joyRead);
    if (stepperDirection != 0)
    {
        digitalWrite(dir_pin, (stepperDirection > 0));
        analogWrite(step_pin, timerVal);
    }
    else
    {
        digitalWrite(step_pin, LOW);
    }
    
    analogUpdateTimer.start();
    
    static char outText[37];
    sprintf_P(outText, PSTR("%i, %hi, %u"), joyRead, stepperDirection, timerVal);
    Serial.println(outText);
}

void timerInit()
{
    CRITICAL_SECTION_START;
/* 
#if defined(TCCR1B) && defined(CS11) && defined(WGM10) && defined(WGM13)
    cbi(TCCR1B, CS11);    // set timer 1 prescale factor to 1
    sbi(TCCR1B, CS10);
    sbi(TCCR1A, WGM13);   // put timer 1 in 8-bit phase and frequency correct pwm mode
#endif
 */
#if defined(TCCR3B) && defined(CS31) && defined(WGM30) && defined(WGM33)
    cbi(TCCR3B, CS31);    // set timer 3 prescale factor to 1
    sbi(TCCR3B, CS30);
    sbi(TCCR3A, WGM33);   // put timer 3 in 8-bit phase and frequency correct pwm mode
#endif
#if defined(TCCR4B) && defined(CS41) && defined(WGM40) && defined(WGM43)
  cbi(TCCR4B, CS41);    // set timer 4 prescale factor to 1
  sbi(TCCR4B, CS40);
  sbi(TCCR4A, WGM43);   // put timer 4 in 8-bit phase and frequency correct pwm mode
#endif
    CRITICAL_SECTION_END;
}

void setup() {
    timerInit();
    Serial.begin(115200);
    UCSR0B &= ~(1<<RXEN0);
    Serial.println(F("start"));
    
    SET_OUTPUT(LED_BUILTIN);
    WRITE(LED_BUILTIN, HIGH);
    delay(100);
    WRITE(LED_BUILTIN, LOW);

    for (uint8_t i = 0; i < NUM_AXIS; i++)
        axis[i].init();
    
    analogUpdateTimer.start();
}

void loop() {
    if (analogUpdateTimer.expired(10))
    {
        for (uint8_t i = 0; i < NUM_AXIS; i++)
            axis[i].process();
    }
}
