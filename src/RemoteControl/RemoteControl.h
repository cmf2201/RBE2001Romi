#pragma once

class RemoteControl
{
    private:
    
    public:
        uint8_t remotePin = -1;
        RemoteControl(uint8_t p) : remotePin(p) {};
        void setup();


};