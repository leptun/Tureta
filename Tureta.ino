#include "fastio.h"
#include "Timer.h"

#define joyX A0
#define joyY A1

#define stepX 4
#define dirX 3
#define enX 2

unsigned int speedX = 0;
unsigned int speedY = 0;

ShortTimer analogUpdateTimer;

ISR(TIMER1_COMPA_vect){
    WRITE(stepX, HIGH);
    asm("nop");
    WRITE(stepX, LOW);
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
    OCR1A = 256;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    /* // turn on CTC mode
    TCCR1B |= (1 << WGM12); */
    // turn on PWM phase correct mode
    TCCR1A |= (1 << WGM11) | (1 << WGM10);;
    TCCR1B |= (1 << WGM13);
    // Set CS11 bits for 8 prescaler
    TCCR1B |= (1 << CS11);// | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    CRITICAL_SECTION_END;
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("start"));
    
    st_init();
    
    speedX = 256;
    speedY = 100;
}

void loop() {
    if (!analogUpdateTimer.running())
        analogUpdateTimer.start();
    if (analogUpdateTimer.expired(100))
    {
        char outText[11];
        int readX = analogRead(joyX);
        int readY = analogRead(joyY);
        sprintf(outText, "%i, %i", readX, readY);
        Serial.println(outText);
    }
    
    if (Serial.available() > 0)
    {
        char getData = Serial.read();
        switch(getData)
        {
            case ']': speedX++; break;
            case '[': speedX--; break;
        }
        if (speedX < 10)
            speedX = 10;
        if (speedX > 300)
            speedX = 300;
    }
    
    CRITICAL_SECTION_START;
    OCR1A = speedX;
    CRITICAL_SECTION_END;
    // Serial.println(speedX);
}
