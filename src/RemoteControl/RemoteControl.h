#pragma once

class RemoteControl
{
    #define maxNumberOfFunctions 60
    private:
        typedef void (*Func)(void);
        Func functions[maxNumberOfFunctions];
        bool activeFunctions[maxNumberOfFunctions];
        bool runOnce[maxNumberOfFunctions];
        bool eStopped = false;
        int eStopIndex = maxNumberOfFunctions;
        int ePauseIndex = maxNumberOfFunctions;
        int eUnPauseIndex = maxNumberOfFunctions;
        bool ePaused = false;
        uint8_t remoteButtons[maxNumberOfFunctions];
        uint8_t pin = -1;
        int currentFunctionCount = 0;
        
        
    public:
        RemoteControl(uint8_t p) : pin(p) {};
        void setup();
        void test();
        void toggleFunc(Func func, uint8_t remoteButton);
        void onPress(Func func, uint8_t remoteButton);
        void eStop(Func func, uint8_t remoteButton);
        void ePause(Func pauseFunc, Func unPausefunc, uint8_t remoteButton);
        void runCurrentFunctions();
        void remoteFunctions();
        void checkRemoteButtons();
        void stopFunction(uint8_t remoteButton);
        void startFunction(uint8_t remoteButton);
        void startFunction(uint8_t remoteButton,int printIt);

};