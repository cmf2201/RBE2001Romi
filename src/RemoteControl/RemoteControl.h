#pragma once

class RemoteControl
{
    private:
        uint8_t pin = -1;
    public:
        RemoteControl(uint8_t p) : pin(p) {};
        void setup();
        void test();


};