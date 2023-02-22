#pragma once

class RemoteControl
{
    #define maxNumberOfFunctions 20
    private:
        typedef void (*Func)(void);
        Func functions[maxNumberOfFunctions];
        bool activeFunctions[maxNumberOfFunctions];
        uint8_t remoteButtons[maxNumberOfFunctions];
        uint8_t pin = -1;
        int currentFunctionCount = 0;
        
        
    public:
        RemoteControl(uint8_t p) : pin(p) {};
        void setup();
        void test();
        void running(Func func, uint8_t remoteButton);
        void runCurrentFunctions();
        void remoteFunctions();
        void checkRemoteButtons();


};