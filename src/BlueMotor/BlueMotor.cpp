#include <Arduino.h>
#include "BlueMotor.h"
#include <Romi32U4.h>

long oldValue = 0;
long newValue;
long count = 0;
unsigned time = 0;

BlueMotor::BlueMotor()
{
}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    attachInterrupt(digitalPinToInterrupt(ENCA), isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isl, CHANGE);
    
    reset();
}

long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}


void BlueMotor::isr()
{
    BlueMotor obj;
    if(digitalRead(obj.ENCA) == digitalRead(obj.ENCB)) {
        count++;
    }
    else {
        count--;
    }
}

void BlueMotor::isl()
{
    BlueMotor obj;
    if(digitalRead(obj.ENCA) == digitalRead(obj.ENCB)) {
        count--;
    }
    else {
        count++;
    }
}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

void BlueMotor::setEffortWithoutDB(int effort) {
    if(effort < 0) {
        effort = map(effort,0,-400,-300,-400);
    } else if (effort > 0) {
        effort = map(effort,0,400,300,400);
    }
    setEffort(effort);
}

bool BlueMotor::getToggleOff() {
    return toggleOff;
}

void BlueMotor::setToggleOff(bool toggle) {
    toggleOff = toggle;
    Serial.println(getToggleOff());
}

void BlueMotor::moveTo(long target)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                     //then stop
    float pVal = 1;
    float startPosition = getPosition();
    // target = startPosition + target;
    float distanceToPosition = target - getPosition();
    if(tolerance < abs(distanceToPosition)) {

        distanceToPosition = target - getPosition();
        Serial.print("distance: ");
        Serial.print(distanceToPosition);
        int currentEffort = pVal * distanceToPosition;
        Serial.print(" effort: ");
        Serial.print(currentEffort);
        Serial.print(" Position: ");
        Serial.println(getPosition());


        setEffortWithoutDB(currentEffort);
    } else toggleOff = true;
    //setEffortWithoutDB(0);
}


