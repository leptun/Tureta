#include <Arduino.h>
#include <avr/pgmspace.h>
#include "macros.h"
#include "Serial.h"
#include "Timer.h"
#include "speed_lookuptable.h"
#include "hwTimer.h"
#include "CRC.h"
#include "GPIO.h"
#include "Rasnita.h"
#include "Button.h"

#define joyDeadzone 50
#define NUM_AXIS (sizeof(axis) / sizeof(axis[0]))

#define COMM_SYNC 0x69
#define COMM_BAUD 115200
#define ADC_CHAN_CNT 3

const int16_t joyIdlePos = 512;
uint8_t comm_index = 0;

toggle_t steppersEnabled;
toggle_t motionEnabled(true);


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
} remoteRegister, _remoteRegister;

enum class direction_t : int8_t
{
    idle = 0,
    MIN = -1,
    MAX = 1,
};

class axis_t
{
public:
    axis_t(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin, uint8_t endstop_MIN_pin, uint8_t endstop_MAX_pin, uint8_t joy_chan, direction_t dir_flip)
        : step_pin(GPIO(step_pin))
        , dir_pin(GPIO(dir_pin))
        , en_pin(GPIO(en_pin))
        , endstop_MIN_pin(GPIO(endstop_MIN_pin))
        , endstop_MAX_pin(GPIO(endstop_MAX_pin))
        , joy_chan(joy_chan)
        , dir_flip(dir_flip)
        , timer(digitalPinToTimer(step_pin))
    {};

    void init();
    void process();
    direction_t checkEndstops();
    void setEn(bool en);

private:
    direction_t joyToDirection(int16_t value);
    uint16_t joyToSpeed(int16_t value);

    GPIO step_pin, dir_pin, en_pin, endstop_MIN_pin, endstop_MAX_pin;
    const uint8_t joy_chan;
    const direction_t dir_flip;
    const uint8_t timer;

    direction_t direction = direction_t::idle;
};

axis_t axis[] =
{
    {2, 5, 6, A0, A1, 0, direction_t::MAX},
    {9, 8, 7, A2, A3, 1, direction_t::MAX},
};

rasnita_t rasnita(4);

direction_t axis_t::joyToDirection(int16_t value)
{
    int16_t delta = value - joyIdlePos;
    if (abs(delta) < joyDeadzone)
        return direction_t::idle;
    else if (delta < 0)
        return (direction_t)(-1 * (int8_t)dir_flip);
    else
        return (direction_t)(1 * (int8_t)dir_flip);
}

uint16_t axis_t::joyToSpeed(int16_t value)
{
    return calc_timer(map(abs(value - joyIdlePos), joyDeadzone, joyIdlePos, 32, 2000));
}

void axis_t::init()
{
    step_pin.write(LOW); step_pin.setOutput();
    dir_pin.write(LOW); dir_pin.setOutput();
    en_pin.write(HIGH); en_pin.setOutput(); //initialize as disabled
    endstop_MIN_pin.write(HIGH); endstop_MIN_pin.setInput();
    endstop_MAX_pin.write(HIGH); endstop_MAX_pin.setInput();
    timerInit(timer);
}

void axis_t::process()
{
    uint16_t joyRead = remoteRegister.JOY[joy_chan];
    uint16_t timerVal = joyToSpeed(joyRead);
    
    direction_t stepperDirection = joyToDirection(joyRead);
    direction_t endstops = checkEndstops();
    if (stepperDirection != direction_t::idle && (endstops == direction_t::idle || endstops != stepperDirection) && motionEnabled.getToggleVal())
    {
        dir_pin.write(stepperDirection == direction_t::MAX);
        timerSet(timer, timerVal);
    }
    else
    {
        timerDisable(timer);
    }
    direction = stepperDirection;
    
    // printf_P(PSTR("%i, %hi, %u, %hi\n"), joyRead, (int8_t)stepperDirection, timerVal, (int8_t)endstops);
    // printf_P(PSTR("TCCR4A:%02hX, TCCR4B:%02hX, OCR4A:%u, OCR4B:%u\n"), TCCR4A, TCCR4B, OCR4A, OCR4B);
}

direction_t axis_t::checkEndstops()
{
    direction_t tempState = direction_t::idle;
    if (!endstop_MIN_pin.read())
        tempState = direction_t::MIN;
    else if (!endstop_MAX_pin.read())
        tempState = direction_t::MAX;

    if (tempState != direction_t::idle && direction == tempState)
    {
        timerDisable(timer);
        direction = direction_t::idle;
    }

    return tempState;
}
void axis_t::setEn(bool enabled)
{
    en_pin.write(!enabled); //active low signal
}

void setup() {
    UART_init(115200);
    // UCSR0B &= ~(1<<RXEN0);

    Serial1.begin(115200);

    for (auto a : axis)
        a.init();
    rasnita.init();
}

bool serialUpdate()
{
    if (!Serial1.available())
        return false;
    
    uint8_t c = Serial1.read();
    // printf_P(PSTR("%02hX "), c);
    if (comm_index == 0)
    {
        if (c != COMM_SYNC)
        {
            Serial.println(F("~!SYNC"));
            return false;
        }
    }
    else
    {
        *((uint8_t*)(&_remoteRegister) + comm_index) = c;
    }
    
    comm_index++;
    
    if (comm_index < sizeof(_remoteRegister))
        return false; //not all data was received

    CRC_reset();
    for (uint8_t* i = (uint8_t*)(&_remoteRegister); i < (uint8_t*)(&_remoteRegister) + comm_index - 1; i++)
    {
        CRC_add(*i);
    }
    if (_remoteRegister.CRC != CRC_get())
    {
        Serial.println(F("CRC ERROR"));
        comm_index = 0;
        return false;
    }
    // Serial.println(F("OK"));
    comm_index = 0;
    memcpy(&remoteRegister, &_remoteRegister, sizeof(remoteRegister));
    return true;
}

void loop() {
    bool updated = serialUpdate();

    for (auto a : axis)
        a.checkEndstops();
    
    if (steppersEnabled.update(remoteRegister.GPIO.Button1))
        for (auto a : axis)
            a.setEn(steppersEnabled.getToggleVal());
    
    motionEnabled.update(remoteRegister.GPIO.Button3);

    rasnita.process(remoteRegister.GPIO.Button0 || remoteRegister.GPIO.Button2);
    if (updated)
        for (auto a : axis)
            a.process();
}
