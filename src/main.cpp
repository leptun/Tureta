#include <Arduino.h>
#include <avr/pgmspace.h>
#include "macros.h"
#include "trace.h"
#include "Timer.h"
#include "speed_lookuptable.h"
#include "hwTimer.h"
#include "CRC.h"
#include "GPIO.h"
#include "Rasnita.h"
#include "Button.h"
#include "Joystick.h"

#define joyDeadzone 50

const int16_t joyIdlePos = 512;

toggle_t steppersEnabled;
toggle_t motionEnabled(true);


enum class direction_t : int8_t
{
    MIN = -1,
    idle = 0,
    MAX = 1,
    ERROR = 2,
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
    {2, 3, 4, A7, A6, 0, direction_t::MAX},
    {9, 8, 7, A5, A4, 1, direction_t::MAX},
};
rasnita_t rasnita(6);
GPIO JOY_RESET(10);
GPIO BEEPER(13);

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
    return calc_timer(map(abs(value - joyIdlePos), joyDeadzone, joyIdlePos, 32, 4000));
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
    if (stepperDirection != direction_t::idle && ((endstops == direction_t::idle || endstops != stepperDirection) && endstops != direction_t::ERROR) && motionEnabled.getToggleVal())
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
    if (endstop_MIN_pin.read() && endstop_MAX_pin.read())
        tempState = direction_t::ERROR;
    else if (endstop_MIN_pin.read())
        tempState = direction_t::MIN;
    else if (endstop_MAX_pin.read())
        tempState = direction_t::MAX;

    if ((tempState != direction_t::idle && direction == tempState) || tempState == direction_t::ERROR)
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
    trace_init(115200);

    joystick_init();

    for (auto a : axis)
        a.init();
    rasnita.init();
}

void loop() {
    bool updated = joystick_update();

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
