#pragma once

class BlueMotor
{
public:
    BlueMotor();
    void setEffort(int effort);
    void setEffortWithoutDB(int effort);
    void moveTo(long position);
    long getPosition();
    void reset();
    void setup();
    bool getToggleOff();
    void setToggleOff(bool toggle);

private:
    void setEffort(int effort, bool clockwise);
    static void isr();
    static void isl();
    const int tolerance = 3;
    const int PWMOutPin = 11;
    const int AIN2 = 12;
    const int AIN1 = 4;
    const int ENCA = 0;
    const int ENCB = 1;
    bool toggleOff = false;

};