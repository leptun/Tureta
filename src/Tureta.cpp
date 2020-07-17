#include <Arduino.h>
#include <avr/pgmspace.h>
#include "fastio.h"
#include "Timer.h"
#include "speed_lookuptable.h"

#define joyDeadzone 15

#define joyX 16
#define joyY 17

#define stepX 9
#define dirX 8
#define enX 7

#define stepY 5
#define dirY 13
#define enY 13

enum AXIS
{
    X_AXIS,
    Y_AXIS,
    NUM_AXIS,
};

struct axis_t
{
    int8_t step_pin;
    int8_t dir_pin;
    int8_t en_pin;
    int8_t joy_pin;
    int8_t dir_flip;
};

const struct axis_t axis[NUM_AXIS] =
{
    {9, 8, -1, A2, 1},
    {2, 7, -1, A3, 1},
};

int16_t joyIdlePos = 511;
ShortTimer analogUpdateTimer;

int8_t joyToDirection(int16_t value)
{
    int16_t delta = value - joyIdlePos;
    if (abs(delta) < joyDeadzone)
        return 0;
    else if (delta < 0)
        return -1;
    else
        return 1;
}

uint16_t joyToSpeed(int16_t value)
{
    return calc_timer(map(abs(value - joyIdlePos), joyDeadzone, joyIdlePos, 32, 10000));
}

void st_init()
{
    CRITICAL_SECTION_START;
    
    SET_OUTPUT(stepX);
    SET_OUTPUT(dirX);
    SET_OUTPUT(enX);
    WRITE(stepX, LOW);
    WRITE(dirX, LOW);
    WRITE(enX, LOW);
    
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    TCCR1A |= (1 << WGM10);
    TCCR1B |= (1 << WGM13);
    TCCR1B |= (1 << CS10);
    // TCCR1B |= (1 << CS11);
    CRITICAL_SECTION_END;
}

void setup() {
    Serial.begin(115200);
    
    // while (!Serial);
    
    Serial.println(F("start"));
    
    SET_OUTPUT(LED_BUILTIN);
    WRITE(LED_BUILTIN, HIGH);
    delay(100);
    WRITE(LED_BUILTIN, LOW);
    
    SET_INPUT(joyX);
    SET_INPUT(joyY);
    
    st_init();
    
    analogUpdateTimer.start();
    /* for (uint16_t i = 0; i < 0xFFFF; i++)
    {
        static char outData[14];
        sprintf_P(outData, PSTR("%u, %u"), i, calc_timer(i));
        Serial.println(outData);
    } */
}

void loop() {
    if (analogUpdateTimer.expired(10))
    {
        int16_t readX = analogRead(joyX);
        int16_t readY = analogRead(joyY);
        
        int8_t StepperX_direction = joyToDirection(readX);
        if (StepperX_direction != 0)
        {
            OCR1A = joyToSpeed(readX);
            WRITE(dirX, (StepperX_direction > 0));
            TCCR1A |= (1 << COM1A0);
        }
        else
        {
            TCCR1A &= ~(1 << COM1A0);
        }
        
/*         StepperY_direction = StepperY.joyToDirection(readY);
        if (StepperY.direction != 0)
        {
            StepperY.speedCompare = StepperY.joyToSpeed(readY);
            WRITE(dirY, (StepperY.direction > 0));
        } */
        
        analogUpdateTimer.start();
        
        static char outText[37];
        sprintf_P(outText, PSTR("%i, %i, %hi, %u"), readX, readY, StepperX_direction, OCR1A);
        Serial.println(outText);
    }
}
