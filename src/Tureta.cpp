#include <Arduino.h>
#include <avr/pgmspace.h>
#include "Serial.h"
#include "Timer.h"
#include "speed_lookuptable.h"
#include "hwTimer.h"
#include "CRC.h"

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

#define joyDeadzone 20
#define NUM_AXIS (sizeof(axis) / sizeof(axis[0]))

#define COMM_SYNC 0x69
#define COMM_BAUD 115200
#define ADC_CHAN_CNT 3

const int16_t joyIdlePos = 512;
uint8_t comm_index = 0;

struct
{
    const uint8_t SYNC = COMM_SYNC;
    uint16_t JOY[ADC_CHAN_CNT];
    struct
    {
        uint8_t Button0 : 1;
        uint8_t Button1 : 1;
        uint8_t Button2 : 1;
        uint8_t Button3 : 1;
        uint8_t DpadUP : 1;
        uint8_t DpadRIGHT : 1;
        uint8_t DpadLEFT : 1;
        uint8_t DpadDOWN : 1;
    } GPIO;
    uint8_t CRC;
} remoteRegister;

class axis_t
{
public:
    axis_t(int8_t step_pin, int8_t dir_pin, int8_t en_pin, uint8_t joy_chan, int8_t dir_flip)
        : step_pin(step_pin)
        , dir_pin(dir_pin)
        , en_pin(en_pin)
        , joy_chan(joy_chan)
        , dir_flip(dir_flip)
        , timer(digitalPinToTimer(step_pin))
    {};

    void init();
    void process();
    bool checkEndstops();

private:
    int8_t joyToDirection(int16_t value);
    uint16_t joyToSpeed(int16_t value);

    int8_t step_pin;
    int8_t dir_pin;
    int8_t en_pin;
    int8_t joy_chan;
    int8_t dir_flip;

    uint8_t timer;
};

axis_t axis[] =
{
    {2, 7, -1, 0, 1},
    {9, 8, -1, 1, 1},
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
    timerInit(timer);
}

void axis_t::process()
{
    uint16_t joyRead = remoteRegister.JOY[joy_chan];
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
    
    printf_P(PSTR("%i, %hi, %u, %hu\n"), joyRead, stepperDirection, timerVal, timer);
    // printf_P(PSTR("TCCR4A:%02hX, TCCR4B:%02hX, OCR4A:%u, OCR4B:%u\n"), TCCR4A, TCCR4B, OCR4A, OCR4B);
}

bool axis_t::checkEndstops()
{
    return true;
}

void setup() {
    UART_init(115200);
    // UCSR0B &= ~(1<<RXEN0);

    Serial1.begin(115200);

    for (uint8_t i = 0; i < NUM_AXIS; i++)
        axis[i].init();
}

void loop() {
    for (uint8_t i = 0; i < NUM_AXIS; i++)
        axis[i].checkEndstops();


    if (!Serial1.available())
        return;
    
    uint8_t c = Serial1.read();
    printf_P(PSTR("%02hX "), c);
    if (comm_index == 0)
    {
        if (c != COMM_SYNC)
        {
            Serial.println(F("~!SYNC"));
            return;
        }
    }
    else
    {
        *((uint8_t*)(&remoteRegister) + comm_index) = c;
    }
    
    comm_index++;
    
    if (comm_index < sizeof(remoteRegister))
        return; //not all data was received

    CRC_reset();
    for (uint8_t* i = (uint8_t*)(&remoteRegister); i < (uint8_t*)(&remoteRegister) + comm_index - 1; i++)
    {
        CRC_add(*i);
    }
    if (remoteRegister.CRC != CRC_get())
    {
        Serial.println(F("CRC ERROR"));
        comm_index = 0;
        return;
    }
    Serial.println(F("OK"));

    for (uint8_t i = 0; i < NUM_AXIS; i++)
        axis[i].process();
    comm_index = 0;
}
